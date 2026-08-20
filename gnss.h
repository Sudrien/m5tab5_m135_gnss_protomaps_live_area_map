// gnss.h - M135 GNSS reader as a FreeRTOS task.
//
// The parsing is taken verbatim from the working tab5_gnss_sensors sketch;
// only the plumbing around it is new. Instead of pumpGnss() being called
// three times per loop() to survive a long draw, the drain lives in its own
// high-priority task and simply preempts whatever else is running. That
// matters more here than it did before: a tile render occupies its core for
// hundreds of milliseconds at a stretch.

#ifndef GNSS_H
#define GNSS_H

#include <stdint.h>
#include <stddef.h>   // size_t, used by the assistance API below

struct Constellation {
    const char *name;
    int visible;
    int bestSnr;
};

struct GnssFix {
    char   status = 'V';               // RMC: 'A' = valid
    int    mode = 1;                   // GSA: 1 none, 2 = 2D, 3 = 3D
    int    sats = 0;
    double lat = 0, lon = 0, altitude = 0, hdop = 99.99;
    double speedKmh = 0, course = 0;
    char   utc[16] = "", date[16] = "";
    uint32_t lastSentence = 0;         // millis() of last parsed sentence
    Constellation cons[4] = {{"GPS",0,0},{"GLO",0,0},{"GAL",0,0},{"BDS",0,0}};
};

// Quality gates for the staged zoom-in.
static inline bool gnss_coarse(const GnssFix &f) { return f.status == 'A'; }
static inline bool gnss_fine(const GnssFix &f) {
    return f.status == 'A' && f.mode == 3 && f.hdop > 0 && f.hdop < 2.5;
}

// Pin names follow the module's own labels, as in the original sketch:
// rx_pin is the pin the module TRANSMITS on (G7 with the DIP in position 1),
// tx_pin is the pin it listens on (G6). They are passed to Serial1.begin in
// that same order.
bool gnss_start(int rx_pin, int tx_pin, uint32_t baud, int pps_pin,
                int core, int priority);

void gnss_get(GnssFix *out);

// millis() when the receiver first reported a valid fix, and first reported
// one good enough for gnss_fine(). Zero until it has.
//
// Stamped in the parser, not by whoever reads the fix. That distinction is
// the whole point: a reader running on a task that is busy elsewhere reports
// when it looked, not when the receiver answered, and during startup those
// can be tens of seconds apart.
uint32_t gnss_first_coarse_ms();
uint32_t gnss_first_fine_ms();

// ---- measurement rate ------------------------------------------------------
// The receiver's own solution rate, in milliseconds between fixes. Slowing it
// is the one power saving available here that costs nothing else: the module
// stays hot, keeps its ephemeris and its AOP predictions, and reacquires
// instantly - unlike a backup or standby mode, which trades a real fix for a
// cold-ish start every time the device moves again.
//
// Fire-and-forget: UBX-CFG-RATE is sent and not waited on, because the ACK
// arrives interleaved with NMEA on the same wire and the reader task is the
// only thing draining it. A dropped message means the rate stays where it was,
// which is the status quo rather than a failure - and the policy that calls
// this re-asserts it.
bool     gnss_set_rate_ms(uint16_t ms);

// What was last asked for, not what the receiver confirmed. See above.
uint16_t gnss_rate_ms();

uint32_t gnss_sentences();
uint32_t gnss_pps_count();
uint32_t gnss_pps_interval();

// ---- AssistNow Autonomous --------------------------------------------------
// Predicted orbits computed by the receiver itself - no server, no token. The
// predictions live in battery-backed RAM held by the supercap on V_BCKP, which
// lasts hours rather than the three days the predictions are good for, so they
// are saved to the card and pushed back at boot.

// Turn on AssistNow Autonomous (and ack-aiding, which terminates a database
// poll). Call once after gnss_start().
bool gnss_enable_aop();

// Poll the navigation database out of the receiver into `dst`. Returns bytes
// written, 0 on failure. The bytes are complete UBX frames.
size_t gnss_dbd_read(uint8_t *dst, size_t cap_bytes);

// Push a previously saved database back, before the receiver starts searching.
bool gnss_dbd_write(const uint8_t *src, size_t len);

#endif // GNSS_H
