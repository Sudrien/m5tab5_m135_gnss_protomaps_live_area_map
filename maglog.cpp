// maglog.cpp - see maglog.h.

#include "maglog.h"

#include <Arduino.h>
#include <FS.h>
#include <string.h>

#include "compass.h"
#include "storage.h"

static const char *MAGLOG_PATH = "/maglog.csv";

// Below this the GNSS course field is not a heading, it is noise. u-blox
// stops reporting course over ground when the doppler solution cannot
// distinguish direction from drift, and what arrives instead is an empty
// field - which atof()s to zero and looks exactly like due north.
//
// src: chosen. 8 km/h is a fast walk; the receiver itself gives no explicit
//      "course valid" flag to key off, so this is a judgement and is stated
//      as one. Raise it if the residuals at the low end look like mush.
static const double MAGLOG_MIN_KMH = 8.0;

// One row a second. The GNSS rate policy in tab5_map.cpp holds the receiver at
// its fast rate whenever the device is moving this quickly, so a 1 Hz log is
// never asking for fixes that are not already being produced.
static const uint32_t MAGLOG_PERIOD_MS = 1000;

// Rows are buffered and written in batches: a row is ~150 bytes and an SD
// write costs a 512-byte sector either way, so writing each one as it happens
// spends thirty times the flash for the same data. Thirty seconds of exposure
// to a power cut is the trade, and maglog_flush() shortens it on the idle and
// power-button paths.
static const size_t   MAGLOG_BUF = 4096;
static const uint32_t MAGLOG_FLUSH_MS = 30000;

// Stop rather than fill the card. At ~150 bytes a row and 1 Hz this is about
// twenty hours of continuous qualifying motion, which is far more than the
// modelling needs and far less than the smallest card this project targets.
static const uint32_t MAGLOG_MAX_BYTES = 16u * 1024u * 1024u;

static bool     s_available = false;
// Off at boot. Logging writes to the card continuously and quietly, and a
// logger that starts itself is one nobody remembers turning on - the footer
// button turns it on when it is wanted, for the trip it is wanted for.
static bool     s_enabled   = false;
static bool     s_full      = false;
static uint32_t s_rows      = 0;
static uint32_t s_bytes     = 0;
static uint32_t s_lastWrite = 0;
static uint32_t s_lastFlush = 0;
static uint32_t s_lastSeq   = 0;

static char   s_buf[MAGLOG_BUF];
static size_t s_len = 0;

// The header names every column, because a bare CSV of twenty float columns is
// unreadable six months later and this file exists to be read six months
// later.
static const char *MAGLOG_HEADER =
    "ms,utc,date,lat,lon,speed_kmh,course_deg,hdop,sats,mode,"
    "heading_deg,field_ut,roll_deg,pitch_deg,"
    "mag_raw_x,mag_raw_y,mag_raw_z,"
    "mag_cal_x,mag_cal_y,mag_cal_z,"
    "acc_x,acc_y,acc_z,calibrated\n";

void maglog_begin() {
    s_available = false;
    s_full = false;
    s_len = 0;

    fs::FS *fs = storage_fs();
    if (!fs || !storage_available()) {
        Serial.println("maglog: no storage - magnetometer logging is off");
        return;
    }

    bool fresh = !fs->exists(MAGLOG_PATH);
    // FILE_APPEND rather than FILE_WRITE: FILE_WRITE truncates on this FS
    // wrapper, which would throw away every previous drive on the first boot
    // after the card was moved to another device and back.
    File f = fs->open(MAGLOG_PATH, FILE_APPEND);
    if (!f) {
        Serial.printf("maglog: cannot open %s - logging is off\n", MAGLOG_PATH);
        return;
    }
    if (fresh) f.print(MAGLOG_HEADER);
    s_bytes = (uint32_t)f.size();
    f.close();

    s_available = true;
    s_lastFlush = millis();
    if (s_bytes >= MAGLOG_MAX_BYTES) {
        s_full = true;
        Serial.printf("maglog: %s is already %u KB - at the cap, not logging. "
                      "Move or delete it to resume.\n",
                      MAGLOG_PATH, (unsigned)(s_bytes / 1024));
    } else {
        Serial.printf("maglog: %s open%s, %u KB so far\n",
                      MAGLOG_PATH, fresh ? " (new)" : "",
                      (unsigned)(s_bytes / 1024));
    }
}

bool maglog_available() { return s_available; }
bool maglog_enabled()   { return s_available && s_enabled && !s_full; }
uint32_t maglog_rows()  { return s_rows; }
uint32_t maglog_bytes() { return s_bytes; }

void maglog_set_enabled(bool on) {
    s_enabled = on;
    if (!on) maglog_flush();
    Serial.printf("maglog: %s\n", on ? "logging on" : "logging off");
}

// The idle-path form. maglog already flushes on its own 30 s timer inside
// maglog_poll(), so this is a second opportunity rather than the only one -
// but it costs nothing and it keeps the two loggers symmetrical, which is
// what stopped this being noticed the first time.
void maglog_flush_if_due() {
    if (!s_available || !s_len) return;
    if (millis() - s_lastFlush < MAGLOG_FLUSH_MS) return;
    maglog_flush();
}

void maglog_flush() {
    if (!s_available || !s_len) return;

    fs::FS *fs = storage_fs();
    if (!fs) { s_len = 0; return; }
    File f = fs->open(MAGLOG_PATH, FILE_APPEND);
    if (!f) {
        // Dropping the buffer rather than growing it forever: the card has
        // most likely been pulled, and holding rows for a filesystem that is
        // not coming back is how a logger turns into a memory leak.
        Serial.println("maglog: append failed, dropping buffered rows");
        s_len = 0;
        return;
    }
    f.write((const uint8_t *)s_buf, s_len);
    s_bytes = (uint32_t)f.size();
    f.close();
    s_len = 0;
    s_lastFlush = millis();

    if (s_bytes >= MAGLOG_MAX_BYTES && !s_full) {
        s_full = true;
        Serial.printf("maglog: reached the %u MB cap, logging stopped\n",
                      (unsigned)(MAGLOG_MAX_BYTES / (1024 * 1024)));
    }
}

// Is the GNSS course worth pairing a magnetometer reading with?
//
// All four gates matter and none of them is the speed one on its own: a 3D fix
// with a poor HDOP produces a course that wanders by tens of degrees between
// fixes, and pairing that with a steady magnetometer reading manufactures a
// residual that is entirely the receiver's.
static bool courseUsable(const GnssFix &fix) {
    return fix.status == 'A'
        && fix.mode == 3
        && fix.hdop > 0 && fix.hdop < 2.5
        && fix.speedKmh >= MAGLOG_MIN_KMH;
}

void maglog_poll(const GnssFix &fix) {
    if (!maglog_enabled()) return;

    uint32_t now = millis();
    if (now - s_lastWrite < MAGLOG_PERIOD_MS) {
        if (s_len && now - s_lastFlush > MAGLOG_FLUSH_MS) maglog_flush();
        return;
    }

    if (!courseUsable(fix)) return;

    CompassSample cs;
    if (!compass_sample(&cs)) return;

    // A sample the previous row already carried means compass_update() has not
    // run since - either the compass is down or the bus is busy - and writing
    // it again would put a duplicate reading against a fresh fix, which is
    // exactly the kind of false correlation this log exists to measure.
    if (cs.seq == s_lastSeq) return;

    // Also refuse a sample that is stale in wall-clock terms even if it is
    // new: compass_update() runs at 10 Hz, so anything older than a couple of
    // hundred milliseconds means the loop stalled, and the fix it would be
    // paired with is from a different moment.
    if (now - cs.ms > 300) return;

    s_lastSeq = cs.seq;
    s_lastWrite = now;

    char row[256];
    int n = snprintf(row, sizeof row,
        "%lu,%s,%s,%.7f,%.7f,%.2f,%.2f,%.2f,%d,%d,"
        "%.2f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,"
        "%.0f,%.0f,%.0f,%d\n",
        (unsigned long)now, fix.utc, fix.date,
        fix.lat, fix.lon, fix.speedKmh, fix.course, fix.hdop, fix.sats, fix.mode,
        cs.heading, cs.field_ut, cs.roll, cs.pitch,
        cs.raw[0], cs.raw[1], cs.raw[2],
        cs.corrected[0], cs.corrected[1], cs.corrected[2],
        cs.acc[0], cs.acc[1], cs.acc[2],
        cs.calibrated ? 1 : 0);
    if (n <= 0) return;
    if ((size_t)n >= sizeof s_buf) return;          // cannot happen; not asserted

    if (s_len + (size_t)n > sizeof s_buf) maglog_flush();
    memcpy(s_buf + s_len, row, (size_t)n);
    s_len += (size_t)n;
    s_rows++;

    if (now - s_lastFlush > MAGLOG_FLUSH_MS) maglog_flush();
}
