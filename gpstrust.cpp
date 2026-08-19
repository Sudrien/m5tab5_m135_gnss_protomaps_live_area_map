// gpstrust.cpp - see gpstrust.h.

#include "gpstrust.h"

#include <Arduino.h>
#include <M5Unified.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "wifiloc.h"
#include "gnss.h"

// ---- thresholds ------------------------------------------------------------
// Every one of these is a judgement, and every one is set where an innocent
// cause is already unlikely rather than where an attack becomes possible. The
// cost of a false positive here is an amber word on the status bar; the cost
// of setting them tight is that word appearing on every drive, which would
// train the user to ignore it - and an indicator nobody reads is worse than no
// indicator, because it looks like coverage.
//
// src: all chosen, unattributed - see PROVENANCE.md.

// Implied speed between consecutive fixes. 400 km/h is faster than any road
// vehicle and slower than a jump to a spoofer's chosen position, which is
// typically kilometres. A tunnel exit can beat this legitimately, which is why
// one flag is not a verdict.
static const double TRUST_MAX_KMH = 400.0;

// Position-derived speed against the receiver's own Doppler speed. These come
// from different measurements - pseudorange differences versus carrier phase -
// and a transmitter that gets one right often gets the other wrong. They
// disagree honestly at low speed, where the position delta is mostly noise, so
// the check only runs above a walking pace and allows a wide margin.
static const double TRUST_SPEED_MIN_KMH = 15.0;
static const double TRUST_SPEED_TOL     = 0.5;    // fractional

// GNSS time against the RTC. The RTC free-runs at a few seconds a day, and it
// is seeded from GNSS or SNTP whenever either is available, so a disagreement
// of minutes is not drift. Meaconing - replaying a recorded signal - shows up
// here specifically, as time running behind.
static const int32_t TRUST_CLOCK_TOL_S = 180;

// PPS. The pulse is a hardware output disciplined to the solution, so a
// receiver being dragged off by a transmitter often disturbs it before the
// NMEA shows anything. Wide bounds: the ISR timestamps with millis(), which
// is not a precision clock, and a busy loop can delay the read.
static const uint32_t TRUST_PPS_LO = 900, TRUST_PPS_HI = 1100;

// Satellite SNR spread. A genuine constellation is scattered - satellites low
// on the horizon come in weak, ones overhead come in strong. A single
// transmitter illuminates every channel from one direction at one power, so
// the reported values bunch. This looks at the spread between the best
// constellation and the worst, and only when enough are visible for the
// comparison to mean anything.
static const int TRUST_SNR_MIN_SATS   = 8;
static const int TRUST_SNR_MIN_SPREAD = 4;    // dB between best and worst

// Wi-Fi cross-check. This is the strongest signal available here, because a
// GNSS transmitter has no control over the access points in the area - but the
// centroids are only as good as the survey behind them, so the threshold is
// far outside their own error.
static const double TRUST_WIFI_MAX_M = 3000.0;

// Altitude. Below the Dead Sea shore and above the airliner ceiling are both
// wrong for this device; a frozen altitude across many fixes while the
// position moves is a different signature and cheaper to spot than it looks.
static const double TRUST_ALT_MIN = -450.0, TRUST_ALT_MAX = 12000.0;
static const int    TRUST_ALT_FREEZE_N = 30;

// ---- state -----------------------------------------------------------------
static bool     s_have_prev = false;
static double   s_prev_lat = 0, s_prev_lon = 0;
static uint32_t s_prev_ms = 0;
static double   s_prev_alt = 0;
static int      s_alt_same = 0;

static uint32_t s_flags = 0;
static TrustLevel s_level = TRUST_UNKNOWN;
static char     s_text[64] = "";

// Flags are sticky for a few seconds. A check that fires on one fix and clears
// on the next would produce a status bar that flickers between states too fast
// to read, and the thing worth noticing is that something fired at all.
static const uint32_t TRUST_HOLD_MS = 8000;
static uint32_t s_flag_ms[8] = { 0 };

// Named flag_set rather than the obvious raise(): <signal.h> declares
// int raise(int), and it arrives here transitively through Arduino.h, so a
// static of that name is an ambiguating redeclaration rather than a shadow.
static void flag_set(int bit) {
    s_flags |= (1u << bit);
    s_flag_ms[bit] = millis();
}

static void expire() {
    for (int i = 0; i < 8; i++) {
        if (!(s_flags & (1u << i))) continue;
        if (millis() - s_flag_ms[i] > TRUST_HOLD_MS) s_flags &= ~(1u << i);
    }
}

void gpstrust_reset() {
    s_have_prev = false;
    s_flags = 0;
    s_level = TRUST_UNKNOWN;
    s_alt_same = 0;
    s_text[0] = 0;
}

// Metres between two positions. Equirectangular rather than haversine: the
// distances being tested are short enough that the difference is far below the
// thresholds above, and this runs on every fix.
static double dist_m(double lat1, double lon1, double lat2, double lon2) {
    double mlat = (lat1 + lat2) * 0.5 * M_PI / 180.0;
    double dx = (lon2 - lon1) * 111320.0 * cos(mlat);
    double dy = (lat2 - lat1) * 111320.0;
    return sqrt(dx * dx + dy * dy);
}

// GNSS time, as a unix timestamp, from the RMC date and time fields.
// Returns 0 when either field is absent or malformed - which is the normal
// state before the first fix and must not read as "the year 2000".
static time_t gnss_epoch(const GnssFix &fix) {
    if (strlen(fix.utc) < 6 || strlen(fix.date) < 6) return 0;
    struct tm t;
    memset(&t, 0, sizeof t);
    // hhmmss.sss and ddmmyy, both fixed-width in NMEA.
    t.tm_hour = (fix.utc[0] - '0') * 10 + (fix.utc[1] - '0');
    t.tm_min  = (fix.utc[2] - '0') * 10 + (fix.utc[3] - '0');
    t.tm_sec  = (fix.utc[4] - '0') * 10 + (fix.utc[5] - '0');
    t.tm_mday = (fix.date[0] - '0') * 10 + (fix.date[1] - '0');
    t.tm_mon  = (fix.date[2] - '0') * 10 + (fix.date[3] - '0') - 1;
    int yy    = (fix.date[4] - '0') * 10 + (fix.date[5] - '0');
    // Two-digit year: NMEA has no century. 80 is the usual pivot and the
    // receiver's own epoch rollover is a separate problem this does not solve.
    t.tm_year = (yy < 80 ? yy + 100 : yy);
    if (t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 1 || t.tm_mday > 31)
        return 0;
    return mktime(&t);
}

void gpstrust_update(const GnssFix &fix) {
    expire();

    if (fix.status != 'A') {
        // No fix is not suspicious, but the history is now stale: the next fix
        // may legitimately be anywhere, so the jump check must not compare
        // against a position from before the outage.
        s_have_prev = false;
        if (!s_flags) { s_level = TRUST_UNKNOWN; s_text[0] = 0; }
        return;
    }

    uint32_t now = millis();

    // ---- jump, and position-derived speed ----------------------------------
    if (s_have_prev && now > s_prev_ms) {
        double dt = (now - s_prev_ms) / 1000.0;
        if (dt > 0.25) {
            double m = dist_m(s_prev_lat, s_prev_lon, fix.lat, fix.lon);
            double kmh = (m / dt) * 3.6;

            if (kmh > TRUST_MAX_KMH) flag_set(0);           // TRUST_F_JUMP

            // Doppler against geometry. Both are the receiver's, but they come
            // from different measurements, and a transmitter has to fake both
            // consistently to pass this.
            if (fix.speedKmh > TRUST_SPEED_MIN_KMH || kmh > TRUST_SPEED_MIN_KMH) {
                double big = fix.speedKmh > kmh ? fix.speedKmh : kmh;
                double small = fix.speedKmh > kmh ? kmh : fix.speedKmh;
                if (big > 0 && (big - small) / big > TRUST_SPEED_TOL)
                    flag_set(1);                            // TRUST_F_SPEED
            }
        }
    }

    // ---- altitude ----------------------------------------------------------
    if (fix.altitude < TRUST_ALT_MIN || fix.altitude > TRUST_ALT_MAX) {
        flag_set(6);                                        // TRUST_F_ALT
    } else if (s_have_prev) {
        // Frozen altitude while the position moves. Some simulators output a
        // constant height because varying it convincingly is work.
        if (fabs(fix.altitude - s_prev_alt) < 0.01 &&
            dist_m(s_prev_lat, s_prev_lon, fix.lat, fix.lon) > 20.0) {
            if (++s_alt_same > TRUST_ALT_FREEZE_N) flag_set(6);
        } else {
            s_alt_same = 0;
        }
    }

    // ---- clock -------------------------------------------------------------
    // Against the RTC rather than against system time: system time is set
    // *from* GNSS elsewhere in this firmware, so comparing the two would be
    // comparing a number with itself.
    if (M5.Rtc.isEnabled()) {
        time_t g = gnss_epoch(fix);
        if (g > 0) {
            auto dt = M5.Rtc.getDateTime();
            struct tm rt;
            memset(&rt, 0, sizeof rt);
            rt.tm_year = dt.date.year - 1900;
            rt.tm_mon  = dt.date.month - 1;
            rt.tm_mday = dt.date.date;
            rt.tm_hour = dt.time.hours;
            rt.tm_min  = dt.time.minutes;
            rt.tm_sec  = dt.time.seconds;
            time_t r = mktime(&rt);
            // An RTC that has never been set reads as some epoch value far in
            // the past; that is a flat battery, not an attack.
            if (r > 1767225600) {
                int32_t d = (int32_t)(g - r);
                if (d < 0) d = -d;
                if (d > TRUST_CLOCK_TOL_S) flag_set(2);     // TRUST_F_CLOCK
            }
        }
    }

    // ---- satellite SNR spread ----------------------------------------------
    {
        int sats = 0, best = 0, worst = 127, seen = 0;
        for (int i = 0; i < 4; i++) {
            sats += fix.cons[i].visible;
            if (!fix.cons[i].visible || !fix.cons[i].bestSnr) continue;
            seen++;
            if (fix.cons[i].bestSnr > best)  best  = fix.cons[i].bestSnr;
            if (fix.cons[i].bestSnr < worst) worst = fix.cons[i].bestSnr;
        }
        // Only meaningful with several constellations visible. A single
        // transmitter usually cannot produce a credible multi-constellation
        // picture at all, so few constellations with many satellites is itself
        // the pattern - but that is also what a cheap antenna indoors looks
        // like, so this stays conservative and only flags the bunching.
        if (seen >= 2 && sats >= TRUST_SNR_MIN_SATS &&
            (best - worst) < TRUST_SNR_MIN_SPREAD)
            flag_set(3);                                    // TRUST_F_SNR
    }

    // ---- PPS ---------------------------------------------------------------
    {
        uint32_t iv = gnss_pps_interval();
        // Zero means no pulse has been seen - the pin may not be wired, which
        // is the documented default (DIP position 3 is optional).
        if (iv && (iv < TRUST_PPS_LO || iv > TRUST_PPS_HI))
            flag_set(4);                                    // TRUST_F_PPS
    }

    // ---- Wi-Fi cross-check -------------------------------------------------
    // The one check whose reference the transmitter does not control.
    //
    // Only run against a fresh estimate, and only when the estimate itself is
    // built from enough access points to be worth anything. A disagreement
    // here with a good estimate on both sides is the strongest single
    // indication available on this device.
    {
        double wlat, wlon; float wacc; uint32_t wage;
        if (wifiloc_used() >= 5 && wifiloc_position(&wlat, &wlon, &wacc, &wage)
            && wage < 120000) {
            double m = dist_m(fix.lat, fix.lon, wlat, wlon);
            // The estimate's own spread is added to the threshold rather than
            // ignored: a sparse survey legitimately puts the centroid a long
            // way off, and flagging that would be blaming GNSS for wifiloc.
            if (m > TRUST_WIFI_MAX_M + wacc) flag_set(5);   // TRUST_F_WIFI
        }
    }

    s_prev_lat = fix.lat;
    s_prev_lon = fix.lon;
    s_prev_alt = fix.altitude;
    s_prev_ms = now;
    s_have_prev = true;

    // ---- verdict -----------------------------------------------------------
    int n = 0;
    for (int i = 0; i < 8; i++) if (s_flags & (1u << i)) n++;

    TrustLevel was = s_level;
    s_level = (n == 0) ? TRUST_OK : (n == 1) ? TRUST_ODD : TRUST_BAD;

    // Name the flags rather than scoring them. "position disputed" tells the
    // user nothing actionable; "wifi disagrees" tells them which of the two
    // things they can go and check.
    s_text[0] = 0;
    if (n) {
        const char *names[8] = { "jump", "speed", "clock", "sat SNR",
                                 "PPS", "wifi disagrees", "altitude", "" };
        size_t used = 0;
        for (int i = 0; i < 8 && used < sizeof s_text - 1; i++) {
            if (!(s_flags & (1u << i)) || !names[i][0]) continue;
            used += snprintf(s_text + used, sizeof s_text - used,
                             "%s%s", used ? ", " : "", names[i]);
        }
    }

    if (s_level != was && s_level >= TRUST_ODD)
        Serial.printf("gpstrust: %s - %s\n",
                      s_level == TRUST_BAD ? "MULTIPLE CHECKS FAILED"
                                           : "one check failed",
                      s_text);
}

TrustLevel  gpstrust_level() { return s_level; }
uint32_t    gpstrust_flags() { return s_flags; }
const char *gpstrust_text()  { return s_text; }
