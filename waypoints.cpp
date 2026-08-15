// waypoints.cpp - see waypoints.h.

#include <Arduino.h>
#include <FS.h>
#include <math.h>
#include <time.h>
#include <string.h>

#include "waypoints.h"
#include "storage.h"

static const char *WP_PATH = "/waypoints.bin";

// src: chosen. ASCII "WPT1", same convention as LFX1/AOP1 elsewhere in the
// project, so a hexdump of the card names its own files.
static const uint32_t WP_MAGIC = 0x57505431u;

struct WpFile {
    uint32_t magic;
    uint16_t version;
    uint16_t count;
};

static Waypoint g_wp[WP_MAX];
static int      g_n = 0;
static int      g_target = -1;
static bool     g_loaded = false;

// ---- file ------------------------------------------------------------------

static void wp_save_all() {
    fs::FS *fs = storage_fs();
    if (!fs) return;
    File f = fs->open(WP_PATH, FILE_WRITE);
    if (!f) { Serial.println("wp: open for write failed"); return; }
    WpFile h{ WP_MAGIC, 1, (uint16_t)g_n };
    f.write((const uint8_t *)&h, sizeof h);
    if (g_n) f.write((const uint8_t *)g_wp, sizeof(Waypoint) * g_n);
    f.close();
    Serial.printf("wp: saved %d\n", g_n);
}

void wp_begin() {
    if (g_loaded) return;
    g_n = 0;
    g_target = -1;

    fs::FS *fs = storage_fs();
    if (!fs || !fs->exists(WP_PATH)) { g_loaded = true; return; }
    File f = fs->open(WP_PATH, FILE_READ);
    if (!f) { g_loaded = true; return; }

    WpFile h{};
    if (f.read((uint8_t *)&h, sizeof h) == (int)sizeof h &&
        h.magic == WP_MAGIC && h.version == 1 && h.count <= WP_MAX) {
        int want = h.count;
        int got  = f.read((uint8_t *)g_wp, sizeof(Waypoint) * want);
        // A short read means a truncated file - keep whatever whole records
        // survived rather than discarding the lot. Losing the last pin is a
        // nuisance; losing the other thirty-one is a different thing.
        g_n = got > 0 ? got / (int)sizeof(Waypoint) : 0;
    } else {
        Serial.println("wp: header not recognised, starting empty");
    }
    f.close();
    g_loaded = true;
    Serial.printf("wp: loaded %d\n", g_n);
}

// ---- list ------------------------------------------------------------------

int wp_count() { return g_n; }

bool wp_get(int i, Waypoint *out) {
    if (i < 0 || i >= g_n || !out) return false;
    *out = g_wp[i];
    return true;
}

int wp_add(double lat, double lon, const char *name) {
    wp_begin();
    if (g_n >= WP_MAX) { Serial.println("wp: list full"); return -1; }

    Waypoint &w = g_wp[g_n];
    w.lat = lat;
    w.lon = lon;
    w.saved_utc = 0;

    time_t now = time(nullptr);
    if (now > 100000) w.saved_utc = (int64_t)now;

    if (name && name[0]) {
        strncpy(w.name, name, WP_NAME_MAX - 1);
        w.name[WP_NAME_MAX - 1] = 0;
    } else if (w.saved_utc) {
        struct tm tmv;
        localtime_r((time_t *)&w.saved_utc, &tmv);
        snprintf(w.name, WP_NAME_MAX, "%02d:%02d", tmv.tm_hour, tmv.tm_min);
    } else {
        snprintf(w.name, WP_NAME_MAX, "pin %d", g_n + 1);
    }

    g_n++;
    wp_save_all();
    Serial.printf("wp: added %s at %.5f %.5f\n", w.name, lat, lon);
    return g_n - 1;
}

int wp_add_fix(const GnssFix &fix) {
    if (!gnss_coarse(fix)) {
        Serial.println("wp: no fix, refusing to pin 0,0");
        return -1;
    }
    return wp_add(fix.lat, fix.lon, nullptr);
}

bool wp_remove(int i) {
    if (i < 0 || i >= g_n) return false;
    for (int k = i; k < g_n - 1; k++) g_wp[k] = g_wp[k + 1];
    g_n--;

    // The target is an index into a list that just shifted underneath it.
    if (g_target == i) g_target = -1;
    else if (g_target > i) g_target--;

    wp_save_all();
    return true;
}

void wp_clear() {
    g_n = 0;
    g_target = -1;
    wp_save_all();
}

// ---- target ----------------------------------------------------------------

int  wp_target() { return (g_target >= 0 && g_target < g_n) ? g_target : -1; }

void wp_set_target(int i) {
    g_target = (i >= 0 && i < g_n) ? i : -1;
    if (g_target < 0) Serial.println("wp: target cleared");
    else Serial.printf("wp: navigating to %s\n", g_wp[g_target].name);
}

// ---- geometry --------------------------------------------------------------
// Haversine on a sphere of the WGS84 mean radius. The ellipsoid correction is
// a few parts in a thousand, which is smaller than the fix error this is fed
// with and far smaller than the "straight line, not a road" error that is
// inherent to the whole idea.
static const double EARTH_R_M = 6371008.8;
static const double D2R = 0.017453292519943295;

double wp_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double p1 = lat1 * D2R, p2 = lat2 * D2R;
    double dp = (lat2 - lat1) * D2R, dl = (lon2 - lon1) * D2R;
    double a = sin(dp / 2) * sin(dp / 2)
             + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2.0 * EARTH_R_M * atan2(sqrt(a), sqrt(1.0 - a));
}

double wp_bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double p1 = lat1 * D2R, p2 = lat2 * D2R, dl = (lon2 - lon1) * D2R;
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) / D2R;
    return b < 0 ? b + 360.0 : b;
}

bool wp_target_range(const GnssFix &fix, double *metres, double *bearing_deg) {
    int t = wp_target();
    if (t < 0 || !gnss_coarse(fix)) return false;
    if (metres)      *metres      = wp_distance_m(fix.lat, fix.lon,
                                                  g_wp[t].lat, g_wp[t].lon);
    if (bearing_deg) *bearing_deg = wp_bearing_deg(fix.lat, fix.lon,
                                                   g_wp[t].lat, g_wp[t].lon);
    return true;
}

static const char *compass16(double deg) {
    static const char *pts[16] = {
        "N","NNE","NE","ENE","E","ESE","SE","SSE",
        "S","SSW","SW","WSW","W","WNW","NW","NNW" };
    int i = (int)((deg + 11.25) / 22.5) & 15;
    return pts[i];
}

void wp_target_text(const GnssFix &fix, char *out, size_t cap) {
    if (!out || !cap) return;
    out[0] = 0;
    double m, b;
    int t = wp_target();
    if (t < 0 || !wp_target_range(fix, &m, &b)) return;

    // Under 30 m the bearing from a consumer receiver is noise, and an arrow
    // spinning on the spot reads as a fault. Say "here" and stop pointing.
    if (m < 30.0) {
        snprintf(out, cap, "%s: here (%dm)", g_wp[t].name, (int)m);
        return;
    }
    if (m < 1000.0)
        snprintf(out, cap, "%s: %d m %s", g_wp[t].name, (int)m, compass16(b));
    else
        snprintf(out, cap, "%s: %.1f km %s", g_wp[t].name, m / 1000.0,
                 compass16(b));
}
