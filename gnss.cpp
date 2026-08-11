// gnss.cpp - NMEA parsing lifted from tab5_gnss_sensors.ino, wrapped in a task.

#include "gnss.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdlib.h>

static SemaphoreHandle_t g_lock = nullptr;
static GnssFix g_pub;                 // published copy, guarded by g_lock
static uint32_t g_sentences = 0;

// PPS is edge-counted in an ISR. Polling misses pulses, because the render
// path can occupy far longer than the pulse width.
static volatile uint32_t g_ppsCount = 0, g_ppsLast = 0, g_ppsInterval = 0;

static void IRAM_ATTR ppsIsr() {
    uint32_t now = millis();
    if (g_ppsLast) g_ppsInterval = now - g_ppsLast;
    g_ppsLast = now;
    g_ppsCount++;
}

// ---- parsing (unchanged from the working sketch) ----------------------------
static int splitFields(char *s, char **f, int maxf) {
    int n = 0; f[n++] = s;
    for (char *p = s; *p && n < maxf; p++) {
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
        else if (*p == '*') { *p = 0; break; }
    }
    return n;
}

// Handles both 2-digit latitude and 3-digit longitude without being told
// which: integer-dividing by 100 strips whatever degree field is present.
static double nmeaCoord(const char *v, const char *h) {
    if (!v || !*v) return 0;
    double raw = atof(v); int deg = (int)(raw / 100);
    double d = deg + (raw - deg * 100) / 60.0;
    if (h && (*h == 'S' || *h == 'W')) d = -d;
    return d;
}

static int conIndex(const char *t) {
    if (!strncmp(t, "GP", 2)) return 0;
    if (!strncmp(t, "GL", 2)) return 1;
    if (!strncmp(t, "GA", 2)) return 2;
    if (!strncmp(t, "GB", 2)) return 3;
    return -1;
}

static void parseSentence(char *s, GnssFix &fix) {
    if (s[0] != '$') return;
    g_sentences++;
    fix.lastSentence = millis();
    char talker[3] = { s[1], s[2], 0 };
    char type[4]   = { s[3], s[4], s[5], 0 };
    char *f[24];
    int n = splitFields(s, f, 24);

    if (!strcmp(type, "RMC") && n > 9) {
        fix.status = f[2][0] ? f[2][0] : 'V';
        strncpy(fix.utc, f[1], sizeof(fix.utc) - 1);
        strncpy(fix.date, f[9], sizeof(fix.date) - 1);
        if (fix.status == 'A') {
            fix.lat = nmeaCoord(f[3], f[4]);
            fix.lon = nmeaCoord(f[5], f[6]);
        }
    } else if (!strcmp(type, "VTG") && n > 7) {
        fix.course   = atof(f[1]);
        fix.speedKmh = atof(f[7]);          // km/h directly
    } else if (!strcmp(type, "GGA") && n > 9) {
        fix.sats = atoi(f[7]);
        fix.hdop = f[8][0] ? atof(f[8]) : 99.99;
        fix.altitude = atof(f[9]);
    } else if (!strcmp(type, "GSA") && n > 17) {
        int m = atoi(f[2]); if (m > fix.mode || m == 1) fix.mode = m;
    } else if (!strcmp(type, "GSV") && n >= 4) {
        int ci = conIndex(talker);
        if (ci >= 0) {
            if (atoi(f[2]) == 1) fix.cons[ci].bestSnr = 0;
            fix.cons[ci].visible = atoi(f[3]);
            for (int i = 4; i + 3 < n; i += 4) {
                int snr = atoi(f[i + 3]);
                if (snr > fix.cons[ci].bestSnr) fix.cons[ci].bestSnr = snr;
            }
        }
    }
}


// ---- UBX ------------------------------------------------------------------
// The receiver speaks NMEA outbound, which is all the parser above needs, but
// assistance is UBX-only. This is the minimum needed for that: frame a message,
// and recognise one arriving in the middle of the NMEA stream.

static SemaphoreHandle_t g_ubx_lock = nullptr;   // one UBX exchange at a time

// A captured UBX response, filled by the reader task and read by the caller.
struct UbxCapture {
    uint8_t  cls, id;          // what to capture; 0xFF matches anything
    uint8_t *buf;              // whole frames appended here, header to checksum
    size_t   cap, len;
    uint32_t frames;
    uint8_t  ack_cls, ack_id;  // frame that ends the capture
    volatile bool done;
    uint32_t last_ms;          // for idle-timeout termination
};
static UbxCapture *g_cap = nullptr;

static void ubx_checksum(const uint8_t *b, size_t n, uint8_t *a, uint8_t *k) {
    uint8_t ca = 0, ck = 0;
    for (size_t i = 0; i < n; i++) { ca += b[i]; ck += ca; }
    *a = ca; *k = ck;
}

// Frame and send. Payload may be null for a poll.
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uint8_t ca = 0, ck = 0;
    ubx_checksum(hdr, 4, &ca, &ck);
    // Checksum runs over class, id, length and payload, so it has to be
    // continued across the two buffers rather than restarted.
    for (uint16_t i = 0; i < len; i++) { ca += payload[i]; ck += ca; }

    Serial1.write((uint8_t)0xB5); Serial1.write((uint8_t)0x62);
    Serial1.write(hdr, 4);
    if (len) Serial1.write(payload, len);
    Serial1.write(ca); Serial1.write(ck);
    Serial1.flush();
}

// Offered every byte the reader sees. Returns true while it is consuming a UBX
// frame, so the NMEA line assembler leaves those bytes alone.
static bool ubx_feed(uint8_t c) {
    static int      st = 0;
    static uint8_t  cls, id;
    static uint16_t len, got;
    static uint8_t  frame[512];
    static size_t   flen;

    switch (st) {
        case 0: if (c == 0xB5) { st = 1; return true; } return false;
        case 1:
            if (c == 0x62) { st = 2; flen = 0; frame[flen++] = 0xB5; frame[flen++] = 0x62; return true; }
            st = 0; return false;                    // false start, hand back
        case 2: cls = c; frame[flen++] = c; st = 3; return true;
        case 3: id  = c; frame[flen++] = c; st = 4; return true;
        case 4: len = c; frame[flen++] = c; st = 5; return true;
        case 5:
            len |= (uint16_t)c << 8; frame[flen++] = c; got = 0;
            // A length this large means the stream is not really UBX; drop it
            // rather than overrun, and let NMEA resynchronise on the next '$'.
            st = (len > sizeof(frame) - 8) ? 0 : 6;
            return true;
        case 6:
            frame[flen++] = c;
            if (++got >= len) st = 7;
            return true;
        case 7: frame[flen++] = c; st = 8; return true;    // checksum A
        case 8: {
            frame[flen++] = c;
            st = 0;
            UbxCapture *cap = g_cap;
            if (!cap || cap->done) return true;
            if (cls == cap->ack_cls && id == cap->ack_id) { cap->done = true; return true; }
            if (cap->cls != 0xFF && cls != cap->cls) return true;
            if (cap->id  != 0xFF && id  != cap->id)  return true;
            if (cap->len + flen <= cap->cap) {
                memcpy(cap->buf + cap->len, frame, flen);
                cap->len += flen;
                cap->frames++;
                cap->last_ms = millis();
            }
            return true;
        }
    }
    st = 0;
    return false;
}

// ---- task ------------------------------------------------------------------
static void gnss_task(void *arg) {
    (void)arg;
    char line[128];
    int  pos = 0;
    GnssFix local;                    // parsed into privately, published whole

    for (;;) {
        if (!Serial1.available()) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        while (Serial1.available()) {
            char c = Serial1.read();
            // UBX frames are interleaved with NMEA on the same wire. Offer each
            // byte to the binary detector first; it claims the ones that belong
            // to a frame so they never reach the line assembler.
            if (ubx_feed((uint8_t)c)) continue;
            if (c == '\n') {
                line[pos] = 0;
                parseSentence(line, local);
                pos = 0;
                // Publish the whole struct at once so a reader never sees a
                // fix half-updated across sentences.
                if (xSemaphoreTake(g_lock, portMAX_DELAY) == pdTRUE) {
                    g_pub = local;
                    xSemaphoreGive(g_lock);
                }
            } else if (c != '\r' && pos < (int)sizeof(line) - 1) {
                line[pos++] = c;
            }
        }
    }
}


// ---- AssistNow Autonomous --------------------------------------------------
// The receiver predicts its own satellite orbits from ephemeris it has already
// observed, good for about three days. No server, no token, no network - it
// derives the data rather than fetching it.
//
// The catch is where the result lives: battery-backed RAM, held by the 0.22 F
// supercap on V_BCKP, which lasts something like three to five hours at the
// module's backup current. A three-day prediction on five hours of storage is
// mostly thrown away overnight. So the database is polled out to the host and
// written to the card, then pushed back at the next boot - which is what makes
// the prediction outlive the supercap.
//
// UBX-CFG-NAVX5 exists in three versions of different lengths, so this reads
// the current one and edits it rather than constructing a message, which is
// also how the u-blox reference drivers do it.

// All six are u-blox message class/ID assignments.
// src: u-blox M8 receiver description & protocol specification (UBX-13003221),
//      "UBX Class IDs" and the per-message sections. Same values in the M9/M10
//      documents; they have been stable across generations.
#define UBX_CLS_CFG   0x06   // UBX-CFG
#define UBX_ID_NAVX5  0x23   // UBX-CFG-NAVX5, carries the AOP enable bit
#define UBX_CLS_MGA   0x13   // UBX-MGA (multiple GNSS assistance)
#define UBX_ID_DBD    0x80   // UBX-MGA-DBD, the navigation database dump
#define UBX_ID_MGA_ACK 0x60  // UBX-MGA-ACK, what terminates a DBD poll
#define UBX_CLS_ACK   0x05   // UBX-ACK

// Run one UBX exchange: send, then collect matching frames until the
// terminating frame arrives or things go quiet.
static bool ubx_exchange(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len,
                         UbxCapture *cap, uint32_t timeout_ms, uint32_t idle_ms)
{
    if (!g_ubx_lock) return false;
    if (xSemaphoreTake(g_ubx_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

    cap->len = 0; cap->frames = 0; cap->done = false; cap->last_ms = millis();
    g_cap = cap;

    ubx_send(cls, id, pl, len);

    uint32_t t0 = millis();
    while (!cap->done && millis() - t0 < timeout_ms) {
        // Idle termination as well as the ack, so a receiver with ackAiding
        // off still finishes instead of always burning the full timeout.
        if (cap->frames && idle_ms && millis() - cap->last_ms > idle_ms) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    g_cap = nullptr;
    xSemaphoreGive(g_ubx_lock);
    return cap->frames > 0 || cap->done;
}

// Enable AssistNow Autonomous, and ack-aiding along with it - the latter is
// what terminates a database poll, and it lives in the same message.
bool gnss_enable_aop() {
    static uint8_t buf[128];
    UbxCapture cap = { UBX_CLS_CFG, UBX_ID_NAVX5, buf, sizeof buf, 0, 0,
                       0xFF, 0xFF, false, 0 };

    if (!ubx_exchange(UBX_CLS_CFG, UBX_ID_NAVX5, nullptr, 0, &cap, 1500, 250)) {
        Serial.println("gnss: CFG-NAVX5 poll got no reply");
        return false;
    }
    // frame is B5 62 cls id lenL lenH <payload> ckA ckB
    if (cap.len < 6 + 32 + 2) {
        Serial.printf("gnss: CFG-NAVX5 reply too short (%u B)\n", (unsigned)cap.len);
        return false;
    }
    uint16_t plen = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    uint8_t *p = buf + 6;

    // aopCfg is byte 27 and aopOrbMaxErr bytes 30-31 in every version, and
    // ackAiding is byte 17.
    //
    // mask1 selects which of those the receiver actually applies, so writing a
    // byte without setting its mask bit changes nothing. That is not
    // hypothetical: this first shipped with mask1 = 0x4000 (aop alone), and
    // ackAiding was written and silently ignored - the database poll then had
    // to fall back on its idle timeout because the terminating MGA-ACK never
    // came. Both bits, or only one of the two settings takes.
    //
    //   bit 10 (0x0400) ackAid
    //   bit 14 (0x4000) aop
    p[2]  = 0x00;
    p[3]  = 0x44;            // aop + ackAid; nothing else disturbed
    p[4]  = 0x00;
    p[5]  = 0x00;
    p[17] = 1;               // ackAiding
    p[27] = 1;               // aopCfg: AssistNow Autonomous on
    p[30] = 0; p[31] = 0;    // aopOrbMaxErr 0 = leave at the firmware default

    ubx_send(UBX_CLS_CFG, UBX_ID_NAVX5, p, plen);
    Serial.printf("gnss: AssistNow Autonomous enabled (NAVX5 v%u, %u B)\n",
                  (unsigned)p[0], (unsigned)plen);
    return true;
}

// Poll the navigation database out of the receiver. Returns bytes captured.
size_t gnss_dbd_read(uint8_t *dst, size_t cap_bytes) {
    UbxCapture cap = { UBX_CLS_MGA, UBX_ID_DBD, dst, cap_bytes, 0, 0,
                       UBX_CLS_MGA, UBX_ID_MGA_ACK, false, 0 };
    if (!ubx_exchange(UBX_CLS_MGA, UBX_ID_DBD, nullptr, 0, &cap, 8000, 500))
        return 0;
    Serial.printf("gnss: navigation database %u frames, %u bytes%s\n",
                  (unsigned)cap.frames, (unsigned)cap.len,
                  cap.done ? "" : " (idle timeout, no MGA-ACK)");
    return cap.len;
}

// Push a previously saved database back. The stored bytes are already complete
// UBX frames, so they go out verbatim - spaced out, because the receiver drops
// assistance messages it is too busy to take.
bool gnss_dbd_write(const uint8_t *src, size_t len) {
    if (!src || len < 8) return false;
    if (!g_ubx_lock) return false;
    if (xSemaphoreTake(g_ubx_lock, pdMS_TO_TICKS(2000)) != pdTRUE) return false;

    size_t off = 0; uint32_t sent = 0;
    while (off + 8 <= len) {
        if (src[off] != 0xB5 || src[off + 1] != 0x62) break;   // not a frame
        uint16_t plen = (uint16_t)src[off + 4] | ((uint16_t)src[off + 5] << 8);
        size_t flen = 6 + plen + 2;
        if (off + flen > len) break;                            // truncated
        Serial1.write(src + off, flen);
        Serial1.flush();
        off += flen; sent++;
        vTaskDelay(pdMS_TO_TICKS(7));   // u-blox reference spacing
    }
    xSemaphoreGive(g_ubx_lock);
    Serial.printf("gnss: restored %lu database frames (%u of %u bytes)\n",
                  (unsigned long)sent, (unsigned)off, (unsigned)len);
    return sent > 0;
}

bool gnss_start(int rx_pin, int tx_pin, uint32_t baud, int pps_pin,
                int core, int priority)
{
    g_lock = xSemaphoreCreateMutex();
    if (!g_lock) return false;
    g_ubx_lock = xSemaphoreCreateMutex();
    if (!g_ubx_lock) return false;

    // Must precede begin() - ignored once the port is open. The default 256 B
    // FIFO overflows during a long draw at 38400 baud.
    Serial1.setRxBufferSize(2048);
    Serial1.begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    if (pps_pin >= 0) {
        pinMode(pps_pin, INPUT_PULLDOWN);
        attachInterrupt(pps_pin, ppsIsr, RISING);
    }

    g_pub.lastSentence = millis();

    return xTaskCreatePinnedToCore(gnss_task, "gnss", 4096, nullptr,
                                   priority, nullptr, core) == pdPASS;
}

void gnss_get(GnssFix *out) {
    if (xSemaphoreTake(g_lock, portMAX_DELAY) == pdTRUE) {
        *out = g_pub;
        xSemaphoreGive(g_lock);
    }
}

uint32_t gnss_sentences()    { return g_sentences; }
uint32_t gnss_pps_count()    { return g_ppsCount; }
uint32_t gnss_pps_interval() { return g_ppsInterval; }
