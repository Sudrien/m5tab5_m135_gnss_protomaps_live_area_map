// wifiloc.cpp - see wifiloc.h.

#include "wifiloc.h"

#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

#include "storage.h"
#include "mapengine.h"

static const char *WIFILOC_PATH = "/wifiloc.db";

// src: chosen. ASCII "WLC1", same reasoning as LASTFIX_MAGIC and AOP_MAGIC -
// a file whose first four bytes are junk must be rejected rather than parsed.
static const uint32_t WIFILOC_MAGIC = 0x574C4331u;

// 16384 records at 48 bytes is 768 KB of PSRAM, and 16384 access points is a
// lot of travel - several hundred kilometres of ordinary urban driving. The
// cap exists because the table is flat and rewritten whole, not because the
// memory is scarce: a table ten times this size would make the periodic write
// take long enough to notice.
static const uint32_t WIFILOC_MAX = 16384;

// An AP heard at positions spread wider than this is not in a fixed place -
// a phone hotspot, a bus, another car in traffic. Folding one of those into
// the database pulls every estimate that later hears it towards wherever the
// device happened to be travelling, and it is the single most effective way to
// corrupt a self-built survey.
//
// 400 m is generous: it has to exceed the genuine spread of a fixed AP heard
// from both ends of its range, which for a strong AP on an open road is a
// couple of hundred metres.
//
// src: chosen, unattributed - see PROVENANCE.md.
static const double WIFILOC_MOBILE_M = 400.0;

// Below this many observations a centroid is one or two passes down one side
// of the AP, and is displaced by most of the radio range. Such records are
// kept and kept updating - they improve - but are not used for an estimate.
static const uint32_t WIFILOC_MIN_OBS = 3;

// Three is the floor for anything resembling a position; below that this is
// proximity to one AP, which is not what the map should be told.
static const int WIFILOC_MIN_APS = 3;

// Learning: how often to scan while the fix is good, and how far the device
// must have moved since the last learning scan. The distance gate is what
// stops a parked device pouring thousands of observations of its own house
// into the database and pinning those centroids to the driveway.
static const uint32_t WIFILOC_LEARN_MS = 20000;
static const double   WIFILOC_LEARN_M  = 40.0;

// Locating: how long without a fix before this starts trying, and how often to
// retry. The delay matters - a momentary RMC dropout under a bridge should not
// switch the map onto a Wi-Fi estimate and back again.
static const uint32_t WIFILOC_LOST_MS   = 15000;
static const uint32_t WIFILOC_RETRY_MS  = 20000;

// An estimate older than this is not a position any more, at any speed worth
// drawing a map for.
static const uint32_t WIFILOC_STALE_MS  = 60000;

// Weakest signal worth folding in at all. Below about -88 dBm the RSSI is
// mostly noise and the AP may be a kilometre away or behind a wall three
// metres away; either way it carries no useful information about position.
static const int8_t WIFILOC_MIN_RSSI = -88;

struct ApRec {
    uint64_t id;                 // hash of the BSSID; never the BSSID itself
    uint32_t n;                  // observations folded in
    double   sum_lat, sum_lon;   // running sums, divided on read
    float    min_lat, max_lat;   // extent, for the mobility test
    float    min_lon, max_lon;
    int8_t   best_rssi;          // strongest ever heard, for diagnostics
    uint8_t  mobile;             // sticky: once judged mobile, stays excluded
    uint8_t  pad[2];
};

static ApRec  *s_tab = nullptr;
static uint32_t s_count = 0;
static uint32_t s_dirty = 0;
static bool     s_available = false;
static bool     s_enabled = true;

// Estimate state.
static double   s_est_lat = 0, s_est_lon = 0;
static float    s_est_acc = 0;
static uint32_t s_est_ms = 0;
static bool     s_est_valid = false;
static int      s_est_used = 0;

// Scan state machine. Only one scan can be in flight, and what to do with the
// result depends on why it was started - so the reason is recorded when it is
// launched rather than re-derived from the fix when it lands, which would race
// with the fix arriving in between.
enum ScanWhy { SCAN_NONE = 0, SCAN_LEARN, SCAN_LOCATE };
static ScanWhy  s_scanWhy = SCAN_NONE;
static uint32_t s_scanStart = 0;
static uint32_t s_lastLearn = 0, s_lastLocate = 0;
static double   s_lastLearnLat = 0, s_lastLearnLon = 0;
static bool     s_haveLearnPos = false;
static uint32_t s_lostSince = 0;
static uint32_t s_lastWrite = 0;

// How often a dirty table is written, when nothing else has asked for it.
// The whole table is rewritten each time - about a second at 16k records -
// so this is deliberately infrequent, and the cost of a power cut in between
// is the learning from that interval and nothing older.
static const uint32_t WIFILOC_WRITE_MS = 180000;

// Enough new observations to be worth a write on their own, whatever the
// clock says. A drive through a dense area can add hundreds of records in a
// couple of minutes, and losing those to a battery pull would be the same
// bug as the one this pair of gates exists to fix.
static const uint32_t WIFILOC_WRITE_DIRTY = 400;

// A scan that never completes must not wedge the state machine. The radio can
// be reset underneath us by a portal run or an association attempt, and
// WiFi.scanComplete() then returns "running" forever.
static const uint32_t WIFILOC_SCAN_TIMEOUT_MS = 15000;

// FNV-1a over the six address bytes. Not a security hash and not claimed as
// one: the address space is 2^48 and an attacker with the file could brute
// force it. It is here so the file is not a directly usable list of networks,
// which is a meaningful difference in what a lost card discloses, not a
// guarantee about what could be recovered from one.
static uint64_t bssid_hash(const uint8_t *b) {
    uint64_t h = 1469598103934665603ULL;
    for (int i = 0; i < 6; i++) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}

// Metres per degree, good enough for the spread tests at any latitude the
// device is likely to be at. Not a projection - the map has one of those and
// this is not it; these are threshold comparisons, not coordinates.
static double lat_m(double dlat) { return dlat * 111320.0; }
static double lon_m(double dlon, double at_lat) {
    return dlon * 111320.0 * cos(at_lat * M_PI / 180.0);
}

static ApRec *find(uint64_t id) {
    for (uint32_t i = 0; i < s_count; i++)
        if (s_tab[i].id == id) return &s_tab[i];
    return nullptr;
}

void wifiloc_begin() {
    s_available = false;
    s_count = s_dirty = 0;

    // PSRAM explicitly: 768 KB out of internal heap would be most of it, and
    // the DMA-capable internal blocks are what the TLS handshake and the SDIO
    // link need. This is a lookup table touched a few times a minute - PSRAM
    // latency is irrelevant to it.
    s_tab = (ApRec *)heap_caps_malloc(sizeof(ApRec) * WIFILOC_MAX,
                                      MALLOC_CAP_SPIRAM);
    if (!s_tab) {
        Serial.println("wifiloc: no PSRAM for the table - disabled");
        return;
    }
    memset(s_tab, 0, sizeof(ApRec) * WIFILOC_MAX);
    s_available = true;

    fs::FS *fs = storage_fs();
    if (!fs || !storage_available()) {
        Serial.println("wifiloc: no storage - learning in RAM only, "
                       "nothing will be kept");
        return;
    }
    if (!fs->exists(WIFILOC_PATH)) {
        Serial.println("wifiloc: no database yet, starting empty");
        return;
    }

    File f = fs->open(WIFILOC_PATH, FILE_READ);
    if (!f) { Serial.println("wifiloc: cannot open the database"); return; }

    uint32_t hdr[2] = { 0, 0 };
    if (f.read((uint8_t *)hdr, sizeof hdr) != (int)sizeof hdr ||
        hdr[0] != WIFILOC_MAGIC) {
        f.close();
        Serial.println("wifiloc: database header is not ours, ignoring it");
        return;
    }
    uint32_t n = hdr[1] > WIFILOC_MAX ? WIFILOC_MAX : hdr[1];
    int got = f.read((uint8_t *)s_tab, sizeof(ApRec) * n);
    f.close();

    // A short read means a truncated file - an interrupted write, most likely.
    // Keeping whatever whole records arrived is right: every record is
    // self-contained, so a partial file is a smaller database rather than a
    // corrupt one.
    s_count = (got > 0) ? (uint32_t)(got / (int)sizeof(ApRec)) : 0;
    Serial.printf("wifiloc: %lu access points loaded%s\n",
                  (unsigned long)s_count,
                  (uint32_t)(got / (int)sizeof(ApRec)) != n ? " (file was short)" : "");
}

bool wifiloc_available() { return s_available; }
bool wifiloc_enabled()   { return s_available && s_enabled; }
uint32_t wifiloc_entries() { return s_count; }
uint32_t wifiloc_dirty()   { return s_dirty; }
int  wifiloc_used()        { return s_est_valid ? s_est_used : 0; }

void wifiloc_set_enabled(bool on) {
    s_enabled = on;
    if (!on) { s_est_valid = false; wifiloc_flush(); }
    Serial.printf("wifiloc: %s\n", on ? "on" : "off");
}

// Periodic write, called from the idle path. Separate from wifiloc_flush()
// so the unconditional version stays available for shutdown and for the
// power button, where the point is to write now rather than to write soon.
void wifiloc_flush_if_due() {
    if (!s_available || !s_dirty) return;
    if (s_dirty < WIFILOC_WRITE_DIRTY &&
        s_lastWrite && millis() - s_lastWrite < WIFILOC_WRITE_MS) return;
    wifiloc_flush();
}

void wifiloc_flush() {
    if (!s_available || !s_dirty) return;
    s_lastWrite = millis();
    fs::FS *fs = storage_fs();
    if (!fs || !storage_available()) { s_dirty = 0; return; }

    // Written whole, to a temporary, then renamed - the same dance aopSave and
    // calSave use, and for the same reason: an interrupted write must not
    // leave a file that is valid enough to load. The table is flat and small
    // enough that rewriting it costs about a second, which is why this only
    // happens on the idle path.
    const char *tmp = "/wifiloc.tmp";
    fs->remove(tmp);
    File f = fs->open(tmp, FILE_WRITE);
    if (!f) { Serial.println("wifiloc: cannot write the database"); return; }

    uint32_t hdr[2] = { WIFILOC_MAGIC, s_count };
    bool ok = f.write((const uint8_t *)hdr, sizeof hdr) == sizeof hdr;
    if (ok && s_count)
        ok = f.write((const uint8_t *)s_tab, sizeof(ApRec) * s_count)
             == sizeof(ApRec) * s_count;
    f.close();
    if (!ok) { fs->remove(tmp); Serial.println("wifiloc: write failed"); return; }

    fs->remove(WIFILOC_PATH);
    fs->rename(tmp, WIFILOC_PATH);
    Serial.printf("wifiloc: %lu access points written\n", (unsigned long)s_count);
    s_dirty = 0;
}

// ---- learning --------------------------------------------------------------
static void learn(const GnssFix &fix, int n) {
    int added = 0, folded = 0, mobile = 0;

    for (int i = 0; i < n; i++) {
        if (WiFi.RSSI(i) < WIFILOC_MIN_RSSI) continue;
        uint8_t *b = WiFi.BSSID(i);
        if (!b) continue;
        uint64_t id = bssid_hash(b);

        ApRec *r = find(id);
        if (!r) {
            if (s_count >= WIFILOC_MAX) continue;   // full: keep what is known
            r = &s_tab[s_count++];
            memset(r, 0, sizeof *r);
            r->id = id;
            r->min_lat = r->max_lat = (float)fix.lat;
            r->min_lon = r->max_lon = (float)fix.lon;
            r->best_rssi = -127;
            added++;
        } else {
            folded++;
        }

        r->n++;
        r->sum_lat += fix.lat;
        r->sum_lon += fix.lon;
        if ((float)fix.lat < r->min_lat) r->min_lat = (float)fix.lat;
        if ((float)fix.lat > r->max_lat) r->max_lat = (float)fix.lat;
        if ((float)fix.lon < r->min_lon) r->min_lon = (float)fix.lon;
        if ((float)fix.lon > r->max_lon) r->max_lon = (float)fix.lon;
        if (WiFi.RSSI(i) > r->best_rssi) r->best_rssi = (int8_t)WiFi.RSSI(i);

        // Sticky, and deliberately not reversible. An AP that has been heard
        // across half a city is not going to become fixed later, and a record
        // that flipped back would start contributing a centroid built from
        // exactly the observations that proved it mobile.
        if (!r->mobile && r->n >= WIFILOC_MIN_OBS) {
            double dx = lon_m(r->max_lon - r->min_lon, fix.lat);
            double dy = lat_m(r->max_lat - r->min_lat);
            if (sqrt(dx * dx + dy * dy) > WIFILOC_MOBILE_M) {
                r->mobile = 1;
                mobile++;
            }
        }
        s_dirty++;
    }

    if (added || folded)
        Serial.printf("wifiloc: learned %d new, %d updated, %d newly mobile "
                      "(%lu total)\n",
                      added, folded, mobile, (unsigned long)s_count);
}

// ---- locating --------------------------------------------------------------
static void locate(int n) {
    double wsum = 0, wlat = 0, wlon = 0;
    int used = 0;

    // Two passes: the weighted mean first, then the spread of the contributing
    // centroids about it. The spread is what gets reported as accuracy, and it
    // is honest in a way a fabricated metres-per-dBm figure would not be - it
    // measures how much the APs being heard actually disagree.
    struct { double lat, lon, w; } hit[32];
    int hits = 0;

    for (int i = 0; i < n && hits < 32; i++) {
        int rssi = WiFi.RSSI(i);
        if (rssi < WIFILOC_MIN_RSSI) continue;
        uint8_t *b = WiFi.BSSID(i);
        if (!b) continue;

        ApRec *r = find(bssid_hash(b));
        if (!r || r->mobile || r->n < WIFILOC_MIN_OBS) continue;

        // Received power, linear. This is the whole weighting model: an AP
        // heard 10 dB stronger counts ten times as much, which is the only
        // claim RSSI supports without a path loss exponent and a transmit
        // power neither of which is known.
        double w = pow(10.0, rssi / 10.0);

        hit[hits].lat = r->sum_lat / r->n;
        hit[hits].lon = r->sum_lon / r->n;
        hit[hits].w = w;
        hits++;

        wsum += w;
        wlat += w * (r->sum_lat / r->n);
        wlon += w * (r->sum_lon / r->n);
        used++;
    }

    if (used < WIFILOC_MIN_APS || wsum <= 0) {
        Serial.printf("wifiloc: %d known AP%s in range, need %d - no estimate\n",
                      used, used == 1 ? "" : "s", WIFILOC_MIN_APS);
        return;
    }

    double lat = wlat / wsum, lon = wlon / wsum;

    double vs = 0;
    for (int i = 0; i < hits; i++) {
        double dx = lon_m(hit[i].lon - lon, lat);
        double dy = lat_m(hit[i].lat - lat);
        vs += hit[i].w * (dx * dx + dy * dy);
    }
    double spread = sqrt(vs / wsum);

    s_est_lat = lat;
    s_est_lon = lon;
    s_est_acc = (float)spread;
    s_est_used = used;
    s_est_ms = millis();
    s_est_valid = true;

    Serial.printf("wifiloc: estimate %.5f %.5f from %d APs, spread %.0f m\n",
                  lat, lon, used, spread);
}

bool wifiloc_position(double *lat, double *lon, float *acc_m, uint32_t *age_ms) {
    if (!s_est_valid) return false;
    uint32_t age = millis() - s_est_ms;
    if (age > WIFILOC_STALE_MS) return false;
    if (lat) *lat = s_est_lat;
    if (lon) *lon = s_est_lon;
    if (acc_m) *acc_m = s_est_acc;
    if (age_ms) *age_ms = age;
    return true;
}

// ---- state machine ---------------------------------------------------------
void wifiloc_poll(const GnssFix &fix) {
    if (!wifiloc_enabled()) return;

    // A scan in flight takes precedence over starting another one.
    if (s_scanWhy != SCAN_NONE) {
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) {
            if (millis() - s_scanStart > WIFILOC_SCAN_TIMEOUT_MS) {
                Serial.println("wifiloc: scan did not complete, abandoning it");
                WiFi.scanDelete();
                s_scanWhy = SCAN_NONE;
            }
            return;
        }
        if (n == WIFI_SCAN_FAILED) {
            WiFi.scanDelete();
            s_scanWhy = SCAN_NONE;
            return;
        }
        if (s_scanWhy == SCAN_LEARN) learn(fix, n);
        else                         locate(n);
        WiFi.scanDelete();
        s_scanWhy = SCAN_NONE;
        return;
    }

    // Prefetching and the world floor download both saturate the link for
    // minutes at a time, and a scan interrupts the association. Neither job
    // is worth disturbing for a database entry.
    if (map_prefetch_busy()) return;

    bool good = gnss_fine(fix);

    if (good) {
        s_lostSince = 0;

        // Only a fine fix teaches. A 2D or high-HDOP position is wrong by
        // enough to smear a centroid, and a centroid built from smeared
        // observations is worse than no record at all - it will be believed
        // later, when there is nothing to check it against.
        if (millis() - s_lastLearn < WIFILOC_LEARN_MS) return;

        if (s_haveLearnPos) {
            double dx = lon_m(fix.lon - s_lastLearnLon, fix.lat);
            double dy = lat_m(fix.lat - s_lastLearnLat);
            if (sqrt(dx * dx + dy * dy) < WIFILOC_LEARN_M) return;
        }

        if (WiFi.scanNetworks(true, false) == WIFI_SCAN_FAILED) return;
        s_scanWhy = SCAN_LEARN;
        s_scanStart = millis();
        s_lastLearn = millis();
        s_lastLearnLat = fix.lat;
        s_lastLearnLon = fix.lon;
        s_haveLearnPos = true;
        return;
    }

    // No usable fix. Wait out a brief dropout before deciding this is a place
    // without sky rather than a bridge.
    if (gnss_coarse(fix)) { s_lostSince = 0; return; }
    if (!s_lostSince) { s_lostSince = millis(); return; }
    if (millis() - s_lostSince < WIFILOC_LOST_MS) return;
    if (s_lastLocate && millis() - s_lastLocate < WIFILOC_RETRY_MS) return;
    if (!s_count) return;                     // nothing to match against yet

    if (WiFi.scanNetworks(true, false) == WIFI_SCAN_FAILED) return;
    s_scanWhy = SCAN_LOCATE;
    s_scanStart = millis();
    s_lastLocate = millis();
}
