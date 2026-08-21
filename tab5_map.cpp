// tab5_map.ino - GNSS-following vector map on the M5Stack Tab5 (ESP32-P4).
//
// Pins and GNSS parsing come from the working tab5_gnss_sensors sketch.
//   DIP: TX pos 1, RX pos 1 -> module TX = G7, module RX = G6
//        PPS pos 3 (optional) -> G51
//
// Task layout
//   core 0, prio 5   gnss        UART drain and NMEA parsing
//   core 0, prio 2   loop()      compositing, touch, status overlay
//   core 1, prio 1   tilerender  PMTiles read, inflate, decode, rasterise
//
// The renderer sits alone on core 1 at the lowest priority. A tile takes
// several hundred milliseconds on this part, and must never be able to delay
// the serial drain - which is exactly what the old triple-pumpGnss() dance
// was working around. With the drain in a preemptible task it goes away.
//
// SETUP: copy your extract to the card as /local.pmtiles.

#include <Arduino.h>
#include <time.h>
#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_app_desc.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/sdmmc_host.h>

#include <WiFi.h>

#include <sunset.h>

#include "gnss.h"
#include "mapengine.h"
#include "wifistore.h"
#include "portal.h"
#include "netsource.h"
#include "tilecache.h"
#include "storage.h"
#include "features.h"
#include "mapconfig.h"
#include "style.h"
#include "waypoints.h"
#include "compass.h"
#include "maglog.h"
#include "wifiloc.h"
#include "gpstrust.h"

constexpr int      PIN_GNSS_TX = 7;    // module transmits here -> ESP32 RX
constexpr int      PIN_GNSS_RX = 6;    // module listens here   -> ESP32 TX
constexpr int      PIN_PPS     = 51;
constexpr uint32_t GNSS_BAUD   = 38400;

// The offline floor. Fetched tiles cache alongside it under /t/<build>/;
// this archive only has to cover the low zooms.
static const char *PMT_PATH = "/world.pmtiles";

// The clock runs in UTC throughout - displayed with a trailing Z so there is
// no ambiguity about which it is.
//
// Local time would mean either a hardcoded zone, wrong the moment the device
// travels, or embedded DST rules that go stale as governments change them.
// GNSS and the tile build dates are both UTC already, so keeping one clock
// removes a whole category of off-by-an-hour and off-by-a-day bugs.

// ---- ESP-Hosted (WiFi) wiring ----------------------------------------------
// The P4 has no radio of its own; WiFi comes from an ESP32-C6 companion over
// SDIO. That link is on slot 1 with its own dedicated pins, entirely separate
// from the SD card on slot 0 (GPIO 43/44/39-42) - there is no bus conflict,
// despite the failure mode looking exactly like one.
//
// These are normally supplied by the m5stack_tab5 variant. Setting them here
// as well means the sketch works when built against the generic esp32p4
// board, where those defines are absent and ESP-Hosted has no idea where to
// find the C6.
// SDIO to the ESP32-C6 co-processor.
//
// src: local logging. esp_hosted echoes the pins it ends up using, and this
//      set is what comes back on a working association:
//        sdio_wrapper: GPIOs: CLK[12] CMD[13] D0[11] D1[10] D2[9] D3[8]
//                             Slave_Reset[15]
//      That line is the only confirmation available - the values go in through
//      WiFi.setPins() and there is no read-back API - so it is worth keeping in
//      the boot log rather than trimming.
constexpr int PIN_C6_CLK = 12;
constexpr int PIN_C6_CMD = 13;
constexpr int PIN_C6_D0  = 11;
constexpr int PIN_C6_D1  = 10;
constexpr int PIN_C6_D2  = 9;
constexpr int PIN_C6_D3  = 8;
constexpr int PIN_C6_RST = 15;

// Must run before anything touches the WiFi stack, in both builds.
//
// This was briefly skipped under IDF, because esp_hosted registers a global
// constructor that brings the transport up before setup() and then refuses to
// be reconfigured. The project CMakeLists now removes that constructor from
// the managed component, so arduino-esp32 owns the sequence in both builds and
// this call is once again the thing that sets the pins.
//
// The CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_* values in sdkconfig.defaults are kept
// in step with the constants above regardless, so the two cannot disagree.
static void wifiSetPins() {
    bool ok = WiFi.setPins(PIN_C6_CLK, PIN_C6_CMD, PIN_C6_D0, PIN_C6_D1,
                           PIN_C6_D2, PIN_C6_D3, PIN_C6_RST);
    Serial.printf("wifi: setPins clk=%d cmd=%d d0..d3=%d,%d,%d,%d rst=%d -> %s\n",
                  PIN_C6_CLK, PIN_C6_CMD, PIN_C6_D0, PIN_C6_D1,
                  PIN_C6_D2, PIN_C6_D3, PIN_C6_RST, ok ? "ok" : "FAILED");
}

// Bring the radio up on its own and report what happened. Worth doing as a
// distinct step: if the C6 link is down, the MAC reads as all zeros, which
// distinguishes "no companion chip" from "wrong password" - two failures that
// otherwise look identical from the application side.
static bool wifiRadioUp() {
    // DEBUG: WiFi.mode() is where esp_hosted actually brings the transport up,
    // and it prints nothing until it is well underway. If the missing eleven
    // seconds are here, this is what shows it.
    uint32_t dbg = millis();
    wifiSetPins();
    Serial.printf("dbg:   +%lu ms wifiSetPins\n",
                  (unsigned long)(millis() - dbg)); dbg = millis();

    bool modeOk = WiFi.mode(WIFI_STA);
    Serial.printf("dbg:   +%lu ms WiFi.mode(STA) -> %s\n",
                  (unsigned long)(millis() - dbg), modeOk ? "ok" : "FAILED");
    dbg = millis();

    if (!modeOk) { Serial.println("wifi: WiFi.mode(STA) failed"); return false; }
    delay(200);

    String mac = WiFi.macAddress();
    Serial.printf("wifi: station MAC %s\n", mac.c_str());
    if (mac == "00:00:00:00:00:00" || mac.length() == 0) {
        Serial.println("wifi: MAC is zero - ESP-Hosted link to the C6 is not up.");
        Serial.println("      Check the board selection (esp32:esp32:m5stack_tab5)");
        Serial.println("      and that the C6 still has its SDIO WiFi firmware.");
        return false;
    }
    dbg = millis();
    int n = WiFi.scanNetworks();
    Serial.printf("dbg:   +%lu ms WiFi.scanNetworks\n",
                  (unsigned long)(millis() - dbg));
    Serial.printf("wifi: scan found %d network%s\n", n, n == 1 ? "" : "s");
    for (int i = 0; i < n && i < 8; i++)
        // WiFiScan::RSSI(uint8_t) returns int32_t - 'long' on RISC-V - while
        // WiFiSTA::RSSI() a few lines down returns int8_t and promotes to int.
        // Same spelling, different types; only this one needs the cast.
        Serial.printf("      %-32s %4ld dBm  %s\n",
                      WiFi.SSID(i).c_str(), (long)WiFi.RSSI(i),
                      WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
    return true;
}

// Speed-driven zoom with hysteresis. A single threshold makes anything
// hovering near it flip repeatedly, and every flip invalidates all nine tiles.
// Working zoom levels come from mapconfig.h so the engine and this file
// cannot disagree. Both default to Z_FLOOR, which pins the map to a single
// level - no speed-driven switching, and a quarter of the tiles to cache.
//
// z15 is the ceiling if you do want two: the Protomaps planet build carries
// zoom 0 to 15 only, so z16 returns NOTFOUND everywhere.
static const uint8_t Z_WIDE = Z_LEVEL_WIDE, Z_CLOSE = Z_LEVEL_CLOSE;
static const uint32_t ZOOM_HOLD_MS = 8000;
static uint32_t g_lastZoomChange = 0;

// Try the stored credential first. The portal only comes up if there is no
// usable credential or the join fails, so a working device never stops to ask.
// Touch-and-hold during the first two seconds forces setup, which is the
// escape hatch for changing networks without reflashing.
static bool connectWifi(uint32_t timeout_ms) {
    WifiCred c;
    if (!wifistore_load(&c)) { Serial.println("wifi: no usable stored credential"); return false; }

    Serial.printf("wifi: joining '%s' using %s\n", c.ssid,
                  c.is_psk ? "derived PSK" : "passphrase");
    WiFi.persistent(false);
    // The stored value is the 64-hex PSK, which the supplicant accepts in
    // place of a passphrase - the passphrase itself was never written down.
    WiFi.begin(c.ssid, c.secret);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) delay(100);

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("wifi: connected, IP %s, RSSI %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        // SNTP is deliberately NOT requested here. See ntpPolicy(): the
        // receiver carries UTC itself, so the network clock is a fallback for
        // when it has not produced one, not something to ask for on every
        // join.
        return true;
    }
    Serial.printf("wifi: join failed, status %d\n", (int)WiFi.status());
    WiFi.disconnect(true);
    return false;
}

static bool wantsSetup() {
    uint32_t t0 = millis();
    while (millis() - t0 < 2000) {
        M5.update();
        if (M5.Touch.getCount()) return true;
        delay(20);
    }
    return false;
}

// DURABILITY
//
// The cache flushes its index every 256 writes, so an abrupt power cut can
// leave that many tiles present in the blob but absent from the index. They
// are not lost - every record carries a header, and startup rescans to
// recover them - but the restart cost is avoidable.
//
// Two cheap measures shrink the window to nearly nothing in practice:
//
//   1. Flush whenever the renderer goes quiet. Tiles arrive in bursts around
//      a grid shift; a couple of seconds later nothing is in flight and the
//      index can be written with no contention.
//
//   2. Flush on any power-button activity, since every shutdown gesture
//      starts with a press. Whether the PMU gives firmware a chance to act
//      before cutting the rails is board-specific, so this is opportunistic -
//      the rescan path remains the actual guarantee.
static void flushIfIdle() {
    static uint32_t lastActivity = 0;
    static uint32_t lastPending = 0;

    // The other two writers do not share the tile cache's gate, and putting
    // them behind it was a bug: on a device with the planet archive on the
    // card nothing is ever fetched, so tilecache_pending() is permanently
    // zero, this function returns on the line below, and neither the
    // magnetometer log nor the access point database was ever written from
    // here. The log survived on its own 30 s timer; the database has no
    // timer, so an entire session of learning was lost at every power-off -
    // which is exactly what "wifiloc: no database yet" said on the boot
    // after a drive that had learned 55 access points.
    wifiloc_flush_if_due();
    maglog_flush_if_due();

    uint32_t pending = tilecache_pending();
    if (pending != lastPending) { lastPending = pending; lastActivity = millis(); }
    if (!pending) return;

    MapStats st; map_stats(&st);
    if (st.queue_depth) return;                  // still rendering, stay out of the way
    if (millis() - lastActivity < 2000) return;  // let a burst finish

    tilecache_flush();
    maglog_flush();
    wifiloc_flush();
    lastPending = 0;
}

// Cache warming: a visible button in the corner, with a confirmation step.
//
// Radius 7 is 15x15 tiles - about 13 km at z15, 27 km at z14 - which takes
// several minutes over HTTP. That is too much to start by accident, and an
// invisible whole-screen tap target would do exactly that every time the
// device was picked up.
static const int PREFETCH_RADIUS = 7;

// ---- GNSS measurement rate -------------------------------------------------
// The receiver is the second largest continuous draw on this device after the
// panel, and it costs the same whether the device is doing 100 km/h or sitting
// on a desk. Slowing the solution rate when there is nothing to follow is the
// cheapest saving available, and unlike any of the module's low-power modes it
// gives up nothing: the receiver stays tracking, keeps its ephemeris and its
// AOP predictions, and the next fix is a fix rather than a reacquisition.
//
// Three rates, chosen against what the map does with a fix rather than against
// the receiver's capabilities:
//
//   FAST  vehicle speeds. The 'rule of thirds' sliding view shifts the grid
//         several times a minute at these speeds, and each shift wants a
//         current position to decide where to put it.
//   WALK  walking pace. The marker moves about two metres between fixes at
//         2 s, which is inside the receiver's own error.
//   IDLE  stationary. Nothing on screen changes between fixes; the only
//         reason to keep asking at all is so that starting to move is
//         noticed within a few seconds.
//
// src: all three are judgements, unattributed - see PROVENANCE.md.
static const uint16_t GNSS_RATE_FAST = 1000;
static const uint16_t GNSS_RATE_WALK = 2000;
static const uint16_t GNSS_RATE_IDLE = 5000;

// Thresholds in km/h, with a gap between them: a single threshold makes
// anything hovering near it flip on every fix, and each flip is a UBX message
// the receiver has to process. Same reasoning as the zoom hysteresis above.
static const double GNSS_MOVING_KMH = 12.0;   // above: FAST
static const double GNSS_SLOW_KMH   = 4.0;    // above: WALK, below: IDLE

// How long a speed band has to hold before the rate follows it, and the floor
// between two changes. Both are deliberately long: pulling away from a stop
// light should not have to survive a rate change mid-manoeuvre, and the cost
// of being one band too fast for ten seconds is nil.
static const uint32_t GNSS_RATE_SETTLE_MS = 10000;

static void gnssRatePolicy(const GnssFix &fix) {
    // No fix, no policy. A receiver that is still searching must be left at
    // its fastest rate: the acquisition itself is what the power is being
    // spent on, and slowing it down makes the search longer, not cheaper.
    // Same for a 2D-only fix, where the speed field is not to be trusted.
    uint16_t want;
    if (!gnss_coarse(fix) || fix.mode != 3)     want = GNSS_RATE_FAST;
    else if (fix.speedKmh >= GNSS_MOVING_KMH)   want = GNSS_RATE_FAST;
    else if (fix.speedKmh >= GNSS_SLOW_KMH)     want = GNSS_RATE_WALK;
    else                                        want = GNSS_RATE_IDLE;

    // Prefetching walks the grid across a wide area and the world floor
    // download runs for minutes; neither wants the position it is anchored to
    // going stale underneath it.
    if (map_prefetch_busy()) want = GNSS_RATE_FAST;

    static uint16_t pending = 0;
    static uint32_t pendingSince = 0;
    static uint32_t lastChange = 0;

    if (want == gnss_rate_ms()) { pending = 0; return; }

    if (want != pending) { pending = want; pendingSince = millis(); return; }
    if (millis() - pendingSince < GNSS_RATE_SETTLE_MS) return;
    if (lastChange && millis() - lastChange < GNSS_RATE_SETTLE_MS) return;

    lastChange = millis();
    pending = 0;
    gnss_set_rate_ms(want);
    Serial.printf("gnss: rate -> %u ms (%.1f km/h, mode %d)\n",
                  (unsigned)want, fix.speedKmh, fix.mode);
}

// ---- day / night -----------------------------------------------------------
// There is no ambient light sensor on this board, so the palette is driven by
// the sun's actual position - which the device can compute exactly, having a
// position from GNSS and a UTC date from GNSS or SNTP. That beats a fixed
// clock time, which would be wrong by hours across a year and wrong by more
// if the device travels.
//
// Everything here works in UTC, matching the rest of the firmware, so the
// SunSet timezone offset is zero and its results are minutes past UTC
// midnight.
enum ThemeMode { THEME_AUTO = 0, THEME_DAY, THEME_NIGHT };
static ThemeMode g_themeMode = THEME_AUTO;

// Declared here because applyTheme has to know: a brightness change must not
// wake a screen that was deliberately turned off.
static bool g_screenOff = false;

// True while the map is being drawn from a Wi-Fi centroid estimate rather than
// from the receiver. The status bar reads this rather than inspecting the fix
// it is handed, because that fix has been deliberately dressed up to look
// usable to the map engine and would otherwise print as a real 2D position.
static bool  g_estimated = false;
static float g_estimatedAcc = 0.0f;

static SunSet g_sun;
static bool   g_sunValid = false;
static double g_sunriseMin = 0, g_sunsetMin = 0;
static int    g_sunDay = -1;

// The instant sunIsUpAt() last decided from, minutes past midnight UTC.
//
// Cached rather than recomputed by the twilight test below, for two reasons.
// The cheap one is that it saves a second time()/gmtime_r() on a function
// that runs every pass of loop(). The one that matters is that the palette
// and the backlight must be decided from the same instant: recomputing means
// the two calls can straddle a minute boundary, and the pair that goes wrong
// is exactly the pair at a crossing, where the palette flips to night while
// the backlight still reads one minute short of the window.
static double g_sunNowMin = 0;
static bool   g_sunNowValid = false;

// Backlight levels. Night is dim enough not to ruin dark adaptation but
// still readable; day is full, since sunlight is the harder problem.
// src: M5GFX LGFX_Device::setBrightness takes a uint8_t, so 255 is the top of
//      the range and not a chosen number.
// src: BRIGHT_NIGHT is a judgement, unattributed - see PROVENANCE.md.
static const uint8_t BRIGHT_DAY = 255, BRIGHT_NIGHT = 60;

// The step between them, for the hour around a crossing.
//
// 255 is absurd at dusk. The sun is below the horizon or nearly so, the eye
// has begun to adapt, and the panel is the brightest thing in the vehicle -
// but it is not yet dark enough for the night palette to be readable, so the
// existing two levels have nothing to offer here. This is the backlight
// admitting to an in-between state that the palette cannot.
//
// src: BRIGHT_DUSK is a judgement, unattributed - see PROVENANCE.md.
static const uint8_t BRIGHT_DUSK = 140;

// Half-width of the window, in minutes either side of sunrise and sunset.
//
// src: civil twilight - the sun between the horizon and 6 degrees below it -
//      runs roughly 30 minutes at the mid latitudes this device is used at.
//      That is where the number comes from, but it is a fixed window standing
//      in for a quantity that actually varies with latitude and season, and
//      it is badly wrong toward the poles, where twilight lasts hours. The
//      error is in the safe direction: too narrow a window means a few
//      minutes at 255 that could have been at 140, which is a nuisance, where
//      too wide would mean a dim screen in full sun, which is unreadable.
static const double DUSK_HALFWIDTH_MIN = 30.0;

// Manual override of the backlight.
//
// There is no ambient light sensor on this board. Everything above infers the
// light from the sun's position, which is the best a device without a sensor
// can do and is still routinely wrong: an overcast morning, a multi-storey
// car park, a tunnel approach, a low winter sun straight through the
// windscreen. The sun calculation cannot see any of that, and until now there
// was no way to tell it so.
//
// The manual levels are exactly the three automatic ones rather than a new
// scale. Two reasons. They are already chosen to be readable at their
// intended light, so reusing them adds no new judgement to attribute - the
// PROVENANCE entry for the set covers the manual case unchanged. And it keeps
// the failure mode small: the worst a wrong manual choice can do is put the
// screen at a level automatic would itself have picked an hour earlier.
//
// Deliberately not persisted, matching g_themeMode. Both are overrides of a
// decision the device makes correctly most of the time, and an override that
// survives a reboot is one nobody remembers setting - the same argument
// map_pan_reset() in screenOff() makes about a panned view.
enum BrightMode { BRIGHTM_AUTO = 0, BRIGHTM_LOW, BRIGHTM_MED, BRIGHTM_HIGH };
static BrightMode g_brightMode = BRIGHTM_AUTO;

// Idle dim: how long untouched before the backlight steps down, and how far.
//
// A device left on and parked - at a trailhead, on a desk, in a garage
// overnight - burns the largest single load in the project lighting a screen
// nobody is looking at. This is the case worth catching, and it is separable
// from the screen-off button because it needs no gesture to come back from
// and no decision from the user.
//
// A fraction rather than a fixed level, so the step preserves the
// relationship the level above it established: 40% of a night screen is still
// a night screen. The floor keeps it visibly lit rather than apparently dead,
// which matters because the way back is a touch and nobody touches a screen
// they think has crashed.
//
// src: all three are judgements, unattributed - see PROVENANCE.md.
static const uint32_t IDLE_DIM_MS    = 120000;   // two minutes
static const int      IDLE_DIM_PCT   = 40;
static const uint8_t  IDLE_DIM_FLOOR = 30;
static uint8_t g_brightness = BRIGHT_DAY;

// Defined here rather than with the button state below because idleDimmed()
// reads it several hundred lines earlier than the touch handler writes it.
static uint32_t g_lastTouchMs = 0;

// Split from sunIsUp() so the boot path can ask the same question about a
// remembered position, before any fix exists. See themeBoot().
static bool sunIsUpAt(double lat, double lon) {
    time_t now = time(nullptr);
    if (now < 1767225600) { g_sunNowValid = false; return true; }  // clock unset
    struct tm t;
    gmtime_r(&now, &t);

    // Recompute once a day, or when the position moves far enough to matter.
    if (t.tm_yday != g_sunDay) {
        g_sun.setPosition(lat, lon, 0);                  // 0 = UTC
        g_sun.setCurrentDate(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
        g_sunriseMin = g_sun.calcSunrise();
        g_sunsetMin  = g_sun.calcSunset();
        g_sunDay = t.tm_yday;
        g_sunValid = true;
        int rh = ((int)g_sunriseMin % 1440) / 60, rm = ((int)g_sunriseMin % 1440) % 60;
        int sh = ((int)g_sunsetMin  % 1440) / 60, sm = ((int)g_sunsetMin  % 1440) % 60;
        Serial.printf("sun: rise %02d:%02dZ set %02d:%02dZ%s at %.3f,%.3f\n",
                      rh, rm, sh, sm,
                      g_sunsetMin >= 1440 ? " (next UTC day)" : "",
                      lat, lon);
    }
    if (!g_sunValid) return true;

    double nowMin = t.tm_hour * 60.0 + t.tm_min;
    g_sunNowMin = nowMin;
    g_sunNowValid = true;

    // SunSet returns minutes past midnight in the timezone it was given, and
    // we give it UTC - so a location whose evening falls after 00:00Z gets a
    // sunset past 1440. At this longitude sunset is 00:47Z the following day,
    // reported as 24:47. Normalising and handling the wrap is not optional
    // here; without it the palette flips to night for the 47 minutes after
    // UTC midnight while the sun is still up.
    double rise = fmod(g_sunriseMin, 1440.0); if (rise < 0) rise += 1440.0;
    double set  = fmod(g_sunsetMin,  1440.0); if (set  < 0) set  += 1440.0;

    if (rise < set) return nowMin >= rise && nowMin < set;   // ordinary day
    if (rise > set) return nowMin >= rise || nowMin < set;   // spans midnight

    // Equal means no crossing at all: polar day or polar night, and SunSet
    // cannot tell us which. Daylight is the safer guess - a map too bright
    // is a nuisance, a map too dark at noon is unusable.
    return true;
}

// True when the day cached above came from a remembered position rather than a
// fix. The cache is keyed on the day of the year alone, so without this the
// first real fix of the boot would be ignored until midnight - fine if you
// have not moved, wrong if the device was carried somewhere between runs.
static bool g_sunFromStored = false;

// Minutes to the nearest sunrise or sunset, or -1 when the question does not
// apply.
//
// Reads the cache sunIsUpAt() just filled and must only be called after it.
// Returning -1 rather than a large number for "no crossing" keeps the two
// unanswerable cases - clock not set, and a latitude with no crossing at all -
// from being confused with "a long way from dusk", which they are not: one is
// ignorance and the other is a polar day.
static double minutesToTwilight() {
    if (!g_sunValid || !g_sunNowValid) return -1.0;

    double rise = fmod(g_sunriseMin, 1440.0); if (rise < 0) rise += 1440.0;
    double set  = fmod(g_sunsetMin,  1440.0); if (set  < 0) set  += 1440.0;

    // Equal means no crossing: polar day or polar night, which sunIsUpAt()
    // resolves to daylight because that is the safer guess. There is no
    // twilight to be near, and without this guard the two distances below
    // would be equal and would both read as zero once a day - dimming the
    // screen to dusk in the middle of a polar noon.
    if (rise == set) return -1.0;

    // Circular distance on a 1440-minute clock. Straight subtraction is wrong
    // for the case this function exists to catch: at 23:50 with a sunrise at
    // 00:10 the difference is 1420 minutes and the answer is 20.
    double dr = fabs(g_sunNowMin - rise); if (dr > 720.0) dr = 1440.0 - dr;
    double ds = fabs(g_sunNowMin - set);  if (ds > 720.0) ds = 1440.0 - ds;
    return dr < ds ? dr : ds;
}

static bool sunIsUp(const GnssFix &fix) {
    if (fix.status != 'A' || !fix.utc[0]) return true;   // assume day if unsure
    if (g_sunFromStored) { g_sunDay = -1; g_sunFromStored = false; }
    return sunIsUpAt(fix.lat, fix.lon);
}

// What automatic would choose right now.
//
// The palette is binary and stays that way. This is the backlight only, and
// the separation is the substance of the change: a third palette would have
// to be a half-lit one, and no such thing reads well - the night palette's
// contrast is built for a dark cabin and the day palette's for sunlight, and
// anything between them is worse than both. Brightness has no such problem.
// It is a continuum, and 140 is simply less light than 255.
//
// The dusk step applies only when the theme is automatic too. THEME_DAY and
// THEME_NIGHT are the user overriding the sun calculation, and a device told
// to be in day mode should be readable as if it were day; stepping it down at
// 18:40 because the sun disagrees would be the override failing to override.
//
// Reads map_is_dark() rather than taking the palette as an argument, so the
// footer can call it to label the button. applyTheme() calls this after
// map_set_dark(), so the two agree; anywhere else the palette is whatever was
// last applied, which is what the button should be reporting anyway.
static uint8_t brightnessAuto() {
    if (g_themeMode == THEME_DAY)   return BRIGHT_DAY;
    if (g_themeMode == THEME_NIGHT) return BRIGHT_NIGHT;

    double d = minutesToTwilight();
    if (d >= 0.0 && d <= DUSK_HALFWIDTH_MIN) return BRIGHT_DUSK;
    return map_is_dark() ? BRIGHT_NIGHT : BRIGHT_DAY;
}

// What the backlight should be, honouring a manual override.
//
// A fixed level beats the theme override as well as the sun, which is the
// only ordering that makes sense once the two decisions are separate: someone
// who has forced the night palette to keep the map dark-adapted and then asks
// for full brightness is asking for exactly that, and there is no reading of
// "night theme" that should quietly veto it.
static uint8_t brightnessWanted() {
    switch (g_brightMode) {
        case BRIGHTM_LOW:  return BRIGHT_NIGHT;
        case BRIGHTM_MED:  return BRIGHT_DUSK;
        case BRIGHTM_HIGH: return BRIGHT_DAY;
        default:           return brightnessAuto();
    }
}

// Whether the backlight should be stepped down for want of anyone looking.
//
// The obvious version of this - dim after N minutes untouched - is wrong on
// the device's main job. Driving is exactly the case where the screen is
// being read constantly and touched not at all, and a satnav that fades out
// two minutes into a motorway leg is broken, not thrifty. Touch is a proxy
// for attention and it is a bad one; movement is the better proxy, and this
// device already knows.
//
// So entry needs stationary as well as untouched, and "stationary" reuses the
// band gnssRatePolicy() has already settled rather than testing the speed
// again. That band is debounced by GNSS_RATE_SETTLE_MS and hysteresised
// against the flip-flopping a single threshold would cause, and a second
// independent notion of stopped would be free to disagree with it.
//
// Entry and exit are deliberately asymmetric - slow to dim, immediate to
// brighten:
//
//   - Entry waits for the settled IDLE band. Being slow costs nothing but a
//     few seconds of light.
//   - Exit takes the instantaneous speed, which crosses GNSS_SLOW_KMH some
//     ten to twenty seconds before the band follows it. Without that, pulling
//     away from a stop would leave the screen dim for the whole settle
//     window, which is precisely the manoeuvre where it is being looked at.
//
// No fix, or a 2D-only fix, means no dim. gnssRatePolicy() holds the rate at
// FAST in those cases, so the band test below already refuses, but the
// reasoning is worth stating: a receiver that cannot say whether the device
// is moving has not said it is stopped, and the safe reading of silence here
// is to leave the screen alone.
static bool idleDimmed(const GnssFix &fix) {
    if (g_screenOff) return false;

    // Unsigned subtraction, so the 49.7-day millis() wrap takes care of
    // itself - the difference stays correct across it.
    if (millis() - g_lastTouchMs < IDLE_DIM_MS) return false;

    if (gnss_coarse(fix) && fix.mode == 3 && fix.speedKmh >= GNSS_SLOW_KMH)
        return false;

    return gnss_rate_ms() == GNSS_RATE_IDLE;
}

static uint8_t applyIdleDim(uint8_t want, const GnssFix &fix) {
    static bool wasDim = false;
    bool dim = idleDimmed(fix);
    if (dim != wasDim) {
        wasDim = dim;
        Serial.printf("bright: idle dim %s\n", dim ? "on" : "off");
    }
    if (!dim) return want;

    int v = want * IDLE_DIM_PCT / 100;
    return v < IDLE_DIM_FLOOR ? IDLE_DIM_FLOOR : (uint8_t)v;
}

static void applyTheme(const GnssFix &fix) {
    bool wantDark;
    switch (g_themeMode) {
        case THEME_DAY:   wantDark = false; break;
        case THEME_NIGHT: wantDark = true;  break;
        default:          wantDark = !sunIsUp(fix); break;
    }
    if (wantDark != map_is_dark()) map_set_dark(wantDark);

    // The idle dim wraps the chosen level rather than replacing it, so a
    // manual override still decides what "full" means and this only decides
    // whether anyone is there to see it. The two questions are independent:
    // "how much light does this place need" and "is anyone looking".
    uint8_t want = applyIdleDim(brightnessWanted(), fix);

    // Track the level the screen *should* have, but only drive the backlight
    // when the screen is actually on.
    //
    // Without the guard, a day/night transition while asleep turned the
    // backlight back on without resuming drawing - a lit panel showing
    // nothing at all, no map, no status bar, no buttons. Indistinguishable
    // from a crash, and it survives until someone happens to touch the
    // middle of the screen. The dusk step adds two more crossings a day at
    // which this can fire, so the guard matters more than it did.
    if (want != g_brightness) {
        g_brightness = want;
        if (!g_screenOff) M5.Display.setBrightness(want);
    }
}

// ---- footer buttons --------------------------------------------------------
// Three across the bottom: cache, theme, screen off. The map is clipped out
// of this strip (FOOTER_H in mapengine) so they are not fighting it for
// pixels every frame.
static const int BTN_H = 54, BTN_M = 12;

// ---- flicker-free strips ---------------------------------------------------
// The status bar and the buttons were painted straight to the panel: fill the
// region, then draw the text over it. There is no framebuffer behind the
// display, so those are two separate visible states, and the gap between them
// shows as a blink on every update - which is most obvious on the status bar
// because it repaints whenever any counter changes.
//
// Composing into an off-screen canvas and pushing it once makes the update a
// single write of final pixels, so there is no intermediate state to see. The
// canvases live in PSRAM (about 180 KB together) and are allocated once.
//
// If either allocation fails the direct path still works, blink and all, so a
// tight-memory build degrades rather than losing its UI.
// Set once by panelBegin(). Consulted wherever drawing starts, because with
// no panel every M5.Display call dereferences a null pointer - there are 125
// such calls across the project, so this is gated at the few entry points that
// reach them rather than at each one.
static bool g_panelOk = false;

// True only once setup() has run to completion.
//
// setup() has several early returns - no SD card, map data that will not open.
// Each leaves the system half-built: no map engine, no tile cache, and no
// watchdog subscription, because esp_task_wdt_add() is the second-to-last
// thing it does. loop() then ran anyway and took a mutex that was never
// created, which is the xQueueSemaphoreTake(pxQueue) assert, right after
// esp_task_wdt_reset() complained the task was not found.
static bool g_setupOk = false;

static const int UI_STATUS_H = 52;          // must match STATUS_H in mapengine

static M5Canvas g_statusCv(&M5.Display);
static M5Canvas g_btnCv(&M5.Display);
static bool     g_statusCvOk  = false;
static bool     g_btnCvOk     = false;

// The touch target is taller than the drawn button. A 54 px outline is a
// small thing to hit on a moving vehicle, and there is nothing else along
// the bottom edge to steal a press from - so the target extends upward into
// the map and downward to the screen edge.
static const int BTN_PAD_TOP = 26, BTN_PAD_SIDE = 6;

// Six across the bottom. At 1280 px that is 199 px each - narrower than the
// five-button row was, but the row got the compass dial's 110 px back when the
// dial went, so the net change is small. The touch zone still extends past the
// drawn outline in every direction (BTN_PAD_*), so the usable area is larger
// again than the number suggests.
//
// The ball compass that used to sit at the left end is gone. It was answering
// a question - "which way am I facing" - that the map, drawn north-up with a
// marker on it, mostly already answers, and answering it took a 110 px circle,
// a PSRAM sprite, a per-frame repaint and a projection. The magnetometer is
// still read, and now goes to the card instead of the screen: see maglog.cpp.
// BTN_MAG toggles that, BTN_CAL is what calibration moved to.
enum { BTN_CACHE = 0, BTN_THEME, BTN_BRIGHT, BTN_POI, BTN_PINS,
       BTN_MAG, BTN_CAL, BTN_WIFI, BTN_HOME, BTN_SLEEP, BTN_COUNT };

// Ten buttons at 1280 px is 114 px each - buttonRect() below divides
// 1280 - BTN_M * (BTN_COUNT + 1) between them - and at text size 2 the
// built-in font is 12 px per character, so a label has room for about nine
// characters before it runs into its own border. Every label below is
// written to that budget, which is why several of them read as abbreviations
// rather than as sentences. Nothing enforces it; a longer one is clipped by
// the sprite rather than overflowing into the neighbouring button, so the
// failure mode is a truncated word and not a corrupted row.
//
// BTN_BRIGHT sits next to BTN_THEME because the two are read together: the
// theme button says what the palette is doing and the brightness button says
// what the backlight is doing, and after this commit those are genuinely two
// decisions rather than one.

static uint32_t g_confirmUntil = 0;      // armed state for the cache button

static void buttonRect(int i, int *x, int *y, int *w, int *h) {
    if (!g_panelOk) { *x = *y = *w = *h = 0; return; }
    // The whole width is the button row's now - the compass dial used to take
    // COMPASS_D + BTN_M off the left end here, and removing that is the only
    // change this function needed.
    int total = M5.Display.width() - BTN_M * (BTN_COUNT + 1);
    *w = total / BTN_COUNT;
    *h = BTN_H;
    *x = BTN_M + i * (*w + BTN_M);
    *y = M5.Display.height() - BTN_H - BTN_M;
}

static int buttonAt(int px, int py) {
    if (!g_panelOk) return -1;
    for (int i = 0; i < BTN_COUNT; i++) {
        int x, y, w, h; buttonRect(i, &x, &y, &w, &h);
        int hx = x - BTN_PAD_SIDE, hw = w + BTN_PAD_SIDE * 2;
        int hy = y - BTN_PAD_TOP;
        int hh = M5.Display.height() - hy;      // down to the screen edge
        if (px >= hx && px < hx + hw && py >= hy && py < hy + hh) return i;
    }
    return -1;
}

// Waking is restricted to the middle ninth of the screen.
//
// The device is meant to be carried, and an edge brush against a bag or a
// leg should not light it up and drain the battery. The centre requires a
// deliberate, flat-handed press, and it is also the one region no button
// occupies - so a wake tap can never be mistaken for a button tap.
static bool inWakeZone(int px, int py) {
    if (!g_panelOk) return false;
    int W = M5.Display.width(), H = M5.Display.height();
    return px >= W / 3 && px < 2 * W / 3 &&
           py >= H / 3 && py < 2 * H / 3;
}

// Allocate the composition canvases. Safe to call more than once.
static void uiCanvasesBegin() {
    if (!g_panelOk) {
        Serial.println("ui: no panel, skipping canvas allocation");
        return;
    }
    if (!g_statusCvOk) {
        g_statusCv.setPsram(true);
        g_statusCv.setColorDepth(16);
        g_statusCvOk = g_statusCv.createSprite(M5.Display.width(), UI_STATUS_H);
    }
    if (!g_btnCvOk) {
        int x, y, w, h; buttonRect(0, &x, &y, &w, &h);
        g_btnCv.setPsram(true);
        g_btnCv.setColorDepth(16);
        g_btnCvOk = g_btnCv.createSprite(w, h);
    }
    Serial.printf("ui: status canvas %s, button canvas %s\n",
                  g_statusCvOk ? "ok" : "FAILED (will draw direct)",
                  g_btnCvOk ? "ok" : "FAILED (will draw direct)");
}

static void drawButton(int i, const char *label, uint16_t bg) {
    if (!g_panelOk) return;   // no display attached
    int x, y, w, h; buttonRect(i, &x, &y, &w, &h);

    if (g_btnCvOk) {
        // The corners outside the rounded rect are not ours to paint.
        //
        // Filling them with what the footer is *supposed* to sit on was a
        // guess, and it was wrong - they came out a different colour from
        // the surrounding strip. Rather than guess better, the sprite is
        // pushed with a colour key so those pixels are never written at all
        // and whatever is behind them simply stays. That is correct no
        // matter what painted the footer or when.
        //
        // The key only has to be absent from this sprite: the button fills
        // are greys, blues and greens, the border and label are white.
        const uint16_t KEY = 0xF81F;            // magenta, unused here
        g_btnCv.fillSprite(KEY);
        g_btnCv.fillRoundRect(0, 0, w, h, 10, bg);
        g_btnCv.drawRoundRect(0, 0, w, h, 10, TFT_WHITE);
        g_btnCv.setTextDatum(middle_center);
        g_btnCv.setTextColor(TFT_WHITE);
        g_btnCv.setTextSize(2);
        g_btnCv.drawString(label, w / 2, h / 2);
        g_btnCv.pushSprite(x, y, KEY);
        return;
    }

    M5.Display.fillRoundRect(x, y, w, h, 10, bg);
    M5.Display.drawRoundRect(x, y, w, h, 10, TFT_WHITE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(2);
    M5.Display.drawString(label, x + w / 2, y + h / 2);
    M5.Display.setTextDatum(top_left);
}

// ---- footer repaint gate ---------------------------------------------------
// The footer used to recompose and push all nine buttons on every pass, at
// 15 Hz, for a row whose content changes on the order of once a minute.
//
// That is not the cheap operation the call site makes it look like. Each
// button is a fillSprite, a fillRoundRect, a drawRoundRect, a drawString and
// then a *colour-keyed* pushSprite - and the key is what costs: the sprite
// cannot go out as one block transfer, because every pixel has to be tested
// against KEY to decide whether it is written at all. Nine buttons of
// 116x54 is about 56k of those decisions per pass, 845k per second, to put
// back a row of pixels that were already correct.
//
// So the buttons are composed into a table first and pushed only where the
// table differs from what is on the panel. A single button changing - the
// prefetch percentage ticking, the wifi count moving - now costs one
// pushSprite rather than nine.
//
// The 2 s heartbeat is the same belt-and-braces drawStatus() uses, and is
// here for the same reason: the footer shares the bottom of the panel with
// everything that calls fillScreen() - screenOff, screenOn, pinPanelClose,
// confirmFormat, the boot screens, the portal - and a cache that trusted
// itself completely would show a blank strip until the content happened to
// change. Bounding the staleness at two seconds is worth far more than the
// last 3% of the saving, and unlike an explicit invalidate() it cannot fall
// out of step with a fillScreen added later.
struct FooterBtn {
    char     label[40];   // matches the widest call-site buffer below
    uint16_t bg;
};
static FooterBtn g_btnWant[BTN_COUNT];
static FooterBtn g_btnShown[BTN_COUNT];
static bool      g_btnShownValid = false;
static uint32_t  g_btnLastDraw   = 0;

// Compose one button into the table. Same signature as drawButton() so the
// call sites below read unchanged.
static void setButton(int i, const char *label, uint16_t bg) {
    if (i < 0 || i >= BTN_COUNT) return;
    snprintf(g_btnWant[i].label, sizeof g_btnWant[i].label, "%s", label);
    g_btnWant[i].bg = bg;
}

static void flushButtons() {
    // Past the heartbeat everything is redrawn, whether or not it changed.
    bool all = !g_btnShownValid || (millis() - g_btnLastDraw > 2000);

    bool any = false;
    for (int i = 0; i < BTN_COUNT; i++) {
        if (!all &&
            g_btnWant[i].bg == g_btnShown[i].bg &&
            strcmp(g_btnWant[i].label, g_btnShown[i].label) == 0) continue;
        drawButton(i, g_btnWant[i].label, g_btnWant[i].bg);
        g_btnShown[i] = g_btnWant[i];
        any = true;
    }
    if (all || any) {
        g_btnLastDraw   = millis();
        g_btnShownValid = true;
    }
}

static void drawFooter(const GnssFix &fix) {
    if (!g_panelOk) return;   // no display attached
    // Forgetting here rather than in screenOff() is what makes waking
    // immediate: the screen going off is always followed by a fillScreen and
    // then by this early return, so clearing the cache on the way out means
    // the first pass after screenOn() redraws the row at once instead of
    // waiting out the heartbeat with a blank strip at the bottom.
    if (g_screenOff) { g_btnShownValid = false; return; }

    bool armed = (millis() < g_confirmUntil);
    bool busy  = map_prefetch_busy();
    bool net   = (WiFi.status() == WL_CONNECTED);
    // Only meaningful while idle; during a run the walk is the authority.
    //
    // Memoised: the answer is a few hundred inverse-Mercators and it only
    // changes as the grid moves or an archive is mounted, neither of which
    // happens between two repaints. The touch handler asks unmemoised,
    // because there the answer decides whether to spend minutes downloading.
    static uint32_t held_at = 0;
    static bool     held_memo = false;
    if (!busy && (held_at == 0 || millis() - held_at > 2000)) {
        held_memo = map_prefetch_pending(PREFETCH_RADIUS, Z_WIDE, Z_CLOSE) == 0;
        held_at   = millis();
    }
    bool held = !busy && held_memo;

    char label[40];
    if (busy)       snprintf(label, sizeof label, "cache %d%%", map_prefetch_progress());
    else if (held)  snprintf(label, sizeof label, "offline");
    else if (armed) snprintf(label, sizeof label, "confirm?");
    else if (!net)  snprintf(label, sizeof label, "wifi set");
    else            snprintf(label, sizeof label, "cache%dk",
                             (int)(((2 * PREFETCH_RADIUS + 1) * 40075.0
                                    * 0.74 / (1 << DATA_ZOOM_OF(Z_FLOOR)))));
    setButton(BTN_CACHE, label,
               busy  ? TFT_DARKGREY :
               held  ? M5.Display.color565(30, 90, 50) :
               armed ? TFT_ORANGE   :
               net   ? M5.Display.color565(40, 70, 150)
                     : M5.Display.color565(70, 70, 70));

    // The theme button names the mode, and in auto also shows which way it
    // currently resolves - otherwise "auto" tells you nothing about why the
    // screen looks the way it does.
    const char *tl = g_themeMode == THEME_DAY   ? "day"
                   : g_themeMode == THEME_NIGHT ? "night"
                   : (map_is_dark() ? "auto:nite" : "auto:day");
    setButton(BTN_THEME, tl,
               g_themeMode == THEME_AUTO ? M5.Display.color565(60, 90, 60)
                                         : M5.Display.color565(70, 70, 90));

    // The brightness button, in the same idiom: it names the mode, and in
    // auto it also shows the level that mode currently resolves to. Without
    // the number, "auto" says nothing about why the screen looks as it does -
    // which is the same reason the theme button spells out "auto:day".
    //
    // Nine characters is the label budget, and "fixed:140" is exactly nine.
    char bl[12];
    snprintf(bl, sizeof bl, "%s:%u",
             g_brightMode == BRIGHTM_AUTO ? "auto" : "fixed",
             (unsigned)brightnessWanted());
    setButton(BTN_BRIGHT, bl,
               g_brightMode == BRIGHTM_AUTO ? M5.Display.color565(60, 90, 60)
                                            : M5.Display.color565(70, 70, 90));

    // Labels are an overlay, so this is genuinely instant - there is no
    // "re-rendering" state to show, unlike the theme button.
    bool lab = map_labels_on();
    setButton(BTN_POI, lab ? "labels" : "no labels",
               lab ? M5.Display.color565(60, 80, 110)
                   : M5.Display.color565(70, 70, 70));

    // The pin button doubles as the guidance readout: while a target is set
    // the button names it, so the one control that turns guidance off also
    // says that guidance is on. A separate indicator would be a fifth thing
    // competing for the same strip.
    {
        char pl[40];
        int t = wp_target();
        if (t >= 0) {
            Waypoint w;
            if (wp_get(t, &w)) snprintf(pl, sizeof pl, "to %s", w.name);
            else               snprintf(pl, sizeof pl, "pins");
        } else {
            snprintf(pl, sizeof pl, "pins (%d)", wp_count());
        }
        setButton(BTN_PINS, pl,
                   t >= 0 ? M5.Display.color565(150, 60, 30)
                          : M5.Display.color565(70, 70, 70));
    }

    // The magnetometer button is a logging control, not a heading readout.
    // It shows the row count rather than a state word, because "on" says the
    // switch is set and the count says data is actually landing on the card -
    // which is the thing that goes wrong silently.
    {
        char ml[40];
        if (!compass_ok())            snprintf(ml, sizeof ml, "no mag");
        else if (!maglog_available()) snprintf(ml, sizeof ml, "mag: -");
        else if (!maglog_enabled())   snprintf(ml, sizeof ml, "mag off");
        else snprintf(ml, sizeof ml, "mag %lu", (unsigned long)maglog_rows());
        setButton(BTN_MAG, ml,
                   (compass_ok() && maglog_enabled())
                       ? M5.Display.color565(60, 80, 110)
                       : M5.Display.color565(70, 70, 70));
    }

    // Calibration used to live on the dial. It still has to be a deliberate
    // act rather than something automatic - it needs the device turned through
    // every orientation, which nothing can ask for on its own behalf, and a
    // half-finished sweep biases every reading afterwards - so it keeps its
    // own control now that there is no dial to tap.
    {
        char cl[40];
        if (!compass_ok())               snprintf(cl, sizeof cl, "no mag");
        else if (compass_calibrating())  snprintf(cl, sizeof cl, "cal %d%%",
                                                  compass_calibrate_progress());
        else snprintf(cl, sizeof cl, "%s", compass_status());
        setButton(BTN_CAL, cl,
                   compass_calibrating() ? TFT_ORANGE
                                         : M5.Display.color565(70, 70, 70));
    }

    // The access point count, which is the only honest progress indicator this
    // feature has: the database is useless below a few hundred records and
    // there is nothing else on the device that says how far along it is. It
    // counts total records rather than usable ones - a record needs three
    // observations before an estimate will touch it - because the total is
    // what grows visibly on a drive and the distinction belongs in the log
    // line, not on a 116 px button.
    //
    // Lit amber while an estimate is actually driving the map, which is the
    // one moment the number stops being trivia.
    {
        char wl[40];
        if (!wifiloc_available())    snprintf(wl, sizeof wl, "wifi: -");
        else if (!wifiloc_enabled()) snprintf(wl, sizeof wl, "wifi off");
        else if (g_estimated)        snprintf(wl, sizeof wl, "~%d AP", wifiloc_used());
        else snprintf(wl, sizeof wl, "wifi %lu", (unsigned long)wifiloc_entries());
        setButton(BTN_WIFI, wl,
                   g_estimated ? M5.Display.color565(150, 100, 20)
                   : wifiloc_enabled() ? M5.Display.color565(60, 80, 110)
                                       : M5.Display.color565(70, 70, 70));
    }

    // Lit only while the view is somewhere the device is not. A panned view
    // that nobody remembers panning is the failure this button exists for, so
    // it is coloured to be noticed from across a dashboard rather than to
    // blend into the row.
    {
        bool panned = map_panning();
        setButton(BTN_HOME, panned ? "recentre" : "centred",
                   panned ? M5.Display.color565(150, 60, 30)
                          : M5.Display.color565(70, 70, 70));
    }

    setButton(BTN_SLEEP, "screen off", M5.Display.color565(70, 70, 70));

    flushButtons();
}

// Panning is dropped when the screen goes off. Waking to a map of somewhere
// the device is not - with no memory of having panned there, possibly hours
// later - is the one way this feature can actively mislead, and the cost of
// preventing it is one recentre nobody asked for.
static void screenOff() {
    if (!g_panelOk) return;   // no display attached
    // Draw the wake target before the backlight goes down, so it is clear
    // where to press. An unmarked centre-only zone is undiscoverable.
    int W = M5.Display.width(), H = M5.Display.height();
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.drawRoundRect(W / 3, H / 3, W / 3, H / 3, 16, TFT_DARKGREY);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.setTextSize(2);
    M5.Display.drawString("touch here to wake", W / 2, H / 2);
    M5.Display.setTextDatum(top_left);
    delay(700);

    map_pan_reset();
    g_screenOff = true;
    map_set_visible(false);
    M5.Display.setBrightness(0);
    M5.Display.fillScreen(TFT_BLACK);

    // Deliberately NOT M5.Display.sleep() here. It looks like the obvious
    // next step past setBrightness(0) - the backlight is dark but the
    // MIPI-DSI controller is still refreshing an invisible frame out of the
    // framebuffer - and sleep() does put a stop to that. It was tried.
    //
    // It also took touch down with it, with no way back except a power
    // cycle. Per DISPLAY_IDF_NOTES.md, LCD_RST and TP_RST are one reset pair
    // on this board, held and released together, not two independent lines -
    // the panel and the touch controller are one module, not a panel with a
    // touch peripheral bolted on. Whatever sleep()/wakeup() do to the DSI
    // side of that pair is not guaranteed to leave the touch side answering,
    // and screenOff()'s only exit is a touch in the wake zone below. If that
    // exit is gone, so is the device, until it's unplugged and back.
    //
    // setBrightness(0) is most of the available saving anyway - the panel
    // redrawing black costs little next to the LED string actually being
    // lit. A deeper sleep is worth revisiting, but it needs a wake source
    // that doesn't live behind the same reset pair as the thing being put to
    // sleep - a timer or a physical button, not touch.
    //
    // GNSS is untouched, so the position stays current. Tile fetching
    // continues and rasterising does not - see map_set_visible().
    Serial.println("screen: off (GPS and rendering continue)");
}

static void screenOn() {
    if (!g_panelOk) return;   // no display attached
    g_screenOff = false;
    map_set_visible(true);
    // g_brightness may have been updated by a theme change while asleep, so
    // this applies whatever the current conditions call for rather than
    // whatever was set when the screen went off.
    M5.Display.setBrightness(g_brightness);
    M5.Display.fillScreen(style_background());
    Serial.println("screen: on");
}

// ---- saved-point panel -----------------------------------------------------
// A list over the map: save the current position, pick one to be guided back
// to, or delete one.
//
// Not a modal spin loop like confirmFormat(). That one runs during setup(),
// before esp_task_wdt_add(); anything blocking after it has to feed the
// watchdog itself, and a panel the user may leave open for a minute is the
// wrong place to be doing that. This draws and returns, and loop() keeps
// turning underneath it.
// Composed into an off-screen canvas and pushed once, for the same reason the
// status bar and the buttons are: there is no framebuffer behind this display,
// so filling the panel and then drawing over it are two separate visible
// states. At 15 Hz that gap is a strobe.
//
// Two things were wrong, and the canvas is only the second. The first is that
// the panel was being recomposed every frame at all - nothing on it changes
// between fixes except the ranges, and those change once a second at walking
// pace. It now redraws when its content actually differs, exactly like
// drawStatus.
//
// The sprite is w*h*2 bytes - about 820 KB at 1280x720 - so it is allocated
// when the panel opens and freed when it closes rather than held for the run.
// If the allocation fails the direct path still works, strobe and all, which
// is the same way the status bar degrades.
static bool g_pinPanel = false;
static int  g_pinScroll = 0;

static M5Canvas g_pinCv(&M5.Display);
static bool     g_pinCvOk = false;

// Bumped on every open, and folded into the redraw signature, so reopening a
// panel whose contents happen to be identical to last time still paints it -
// the screen underneath was the map by then.
static uint32_t g_pinEpoch = 0;

static const int PP_ROW_H = 64;
static const int PP_HEAD_H = 70, PP_FOOT_H = 62;

static void pinPanelRect(int *x, int *y, int *w, int *h) {
    if (!g_panelOk) { *x = *y = *w = *h = 0; return; }
    int W = M5.Display.width(), H = M5.Display.height();
    *w = W * 2 / 3;
    *h = H * 2 / 3;
    *x = (W - *w) / 2;
    *y = (H - *h) / 2;
}

static int pinPanelRows() {
    int x, y, w, h; pinPanelRect(&x, &y, &w, &h);
    int rows = (h - PP_HEAD_H - PP_FOOT_H) / PP_ROW_H;
    return rows < 1 ? 1 : rows;
}

static void pinPanelOpen() {
    g_pinPanel = true;
    g_pinScroll = 0;
    g_pinEpoch++;
    if (!g_panelOk || g_pinCvOk) return;
    int x, y, w, h; pinPanelRect(&x, &y, &w, &h);
    g_pinCv.setPsram(true);
    g_pinCv.setColorDepth(16);
    g_pinCvOk = g_pinCv.createSprite(w, h);
    if (!g_pinCvOk)
        Serial.println("pins: panel canvas failed, drawing direct");
}

static void pinPanelClose() {
    g_pinPanel = false;
    if (g_pinCvOk) { g_pinCv.deleteSprite(); g_pinCvOk = false; }
    if (!g_panelOk) return;
    // The panel painted over the map, the status bar and the footer alike, and
    // the engine would see an unchanged view and skip the repaint - the same
    // trap the wifi portal leaves behind on the cache button.
    M5.Display.fillScreen(style_background());
    map_invalidate();
    g_btnShownValid = false;      // ...and the footer cache believes otherwise
}

static void drawPinPanel(const GnssFix &fix) {
    if (!g_panelOk || !g_pinPanel) return;
    int x, y, w, h; pinPanelRect(&x, &y, &w, &h);

    const int rows = pinPanelRows();
    const bool canSave = gnss_coarse(fix) && wp_count() < WP_MAX;
    const int target = wp_target();

    // What is on the panel, as a string. Ranges are quantised to whole metres
    // in the rows below, so a stationary device settles and stops repainting
    // entirely rather than jittering on the last decimal of a fix.
    char sig[256];
    int used = snprintf(sig, sizeof sig, "%lu|%d|%d|%d|%d|%d",
                        (unsigned long)g_pinEpoch,
                        wp_count(), g_pinScroll, target, (int)canSave, rows);
    for (int r = 0; r < rows && used < (int)sizeof sig - 1; r++) {
        int i = g_pinScroll + r;
        if (i >= wp_count()) break;
        Waypoint wp;
        if (!wp_get(i, &wp)) continue;
        int m = gnss_coarse(fix)
              ? (int)wp_distance_m(fix.lat, fix.lon, wp.lat, wp.lon) : -1;
        used += snprintf(sig + used, sizeof sig - used, "|%d", m);
    }
    static char lastSig[256] = "";
    static bool haveLast = false;
    if (haveLast && strcmp(sig, lastSig) == 0) return;
    // snprintf rather than strncpy, and not only to quiet -Wstringop-truncation
    // at -O2. Source and destination are the same size here, so the copy is
    // bounded at sizeof - 1 and a maximal sig leaves strncpy writing no
    // terminator at all; what saves it is that lastSig is static, hence
    // zero-initialised, and its final byte is never written. That is true and
    // unstated, and it stops being true the moment this buffer stops being
    // static. snprintf terminates unconditionally and needs no such argument.
    snprintf(lastSig, sizeof lastSig, "%s", sig);
    haveLast = true;

    // Everything below draws in panel-local coordinates, so the same code
    // serves the sprite and the fallback. The offset is added once, here.
    lgfx::LovyanGFX *g = g_pinCvOk ? (lgfx::LovyanGFX *)&g_pinCv
                                   : (lgfx::LovyanGFX *)&M5.Display;
    const int ox = g_pinCvOk ? 0 : x, oy = g_pinCvOk ? 0 : y;

    if (g_pinCvOk) g_pinCv.fillSprite(M5.Display.color565(20, 20, 26));
    else           M5.Display.fillRoundRect(x, y, w, h, 14,
                                            M5.Display.color565(20, 20, 26));
    g->drawRoundRect(ox, oy, w, h, 14, TFT_WHITE);

    g->setTextDatum(top_left);
    g->setTextColor(TFT_WHITE);
    g->setTextSize(2);
    g->drawString("saved points", ox + 18, oy + 18);

    // Saving without a fix would store 0,0 - the one failure here that looks
    // exactly like success until you try to walk back to it.
    g->fillRoundRect(ox + w - 200, oy + 12, 186, 44, 8,
                     canSave ? M5.Display.color565(40, 110, 60)
                             : M5.Display.color565(60, 60, 60));
    g->setTextDatum(middle_center);
    g->setTextColor(canSave ? TFT_WHITE : TFT_LIGHTGREY);
    g->drawString(!gnss_coarse(fix) ? "no fix"
                  : wp_count() >= WP_MAX ? "list full" : "save here",
                  ox + w - 107, oy + 34);

    for (int r = 0; r < rows; r++) {
        int i = g_pinScroll + r;
        if (i >= wp_count()) break;
        int ry = oy + PP_HEAD_H + r * PP_ROW_H;

        Waypoint wp;
        if (!wp_get(i, &wp)) continue;
        bool active = (i == target);

        g->fillRoundRect(ox + 14, ry, w - 28, PP_ROW_H - 8, 8,
                         active ? M5.Display.color565(110, 45, 25)
                                : M5.Display.color565(45, 45, 55));

        // Range and bearing where there is a fix to measure from, and the
        // stored coordinates where there is not - a row that says nothing at
        // all while the receiver is searching is a row that looks broken.
        char line[80];
        if (gnss_coarse(fix)) {
            double m = wp_distance_m(fix.lat, fix.lon, wp.lat, wp.lon);
            double b = wp_bearing_deg(fix.lat, fix.lon, wp.lat, wp.lon);
            if (m < 1000.0)
                snprintf(line, sizeof line, "%s   %d m   %03d deg",
                         wp.name, (int)m, (int)b);
            else
                snprintf(line, sizeof line, "%s   %.1f km   %03d deg",
                         wp.name, m / 1000.0, (int)b);
        } else {
            snprintf(line, sizeof line, "%s   %.5f %.5f",
                     wp.name, wp.lat, wp.lon);
        }

        g->setTextDatum(middle_left);
        g->setTextColor(TFT_WHITE);
        g->setTextSize(2);
        g->drawString(line, ox + 30, ry + (PP_ROW_H - 8) / 2);

        g->setTextDatum(middle_center);
        g->setTextColor(M5.Display.color565(235, 120, 120));
        g->drawString("del", ox + w - 50, ry + (PP_ROW_H - 8) / 2);
    }

    if (wp_count() == 0) {
        g->setTextDatum(middle_center);
        g->setTextColor(TFT_LIGHTGREY);
        g->setTextSize(2);
        g->drawString("none yet - save here drops one", ox + w / 2, oy + h / 2);
    }

    g->setTextDatum(middle_center);
    g->setTextColor(TFT_WHITE);
    g->fillRoundRect(ox + 14, oy + h - 56, 150, 44, 8,
                     M5.Display.color565(70, 70, 70));
    g->drawString("close", ox + 89, oy + h - 34);

    if (target >= 0) {
        g->fillRoundRect(ox + 178, oy + h - 56, 210, 44, 8,
                         M5.Display.color565(110, 45, 25));
        g->drawString("stop guiding", ox + 283, oy + h - 34);
    }

    if (wp_count() > rows) {
        g->fillRoundRect(ox + w - 154, oy + h - 56, 64, 44, 8,
                         M5.Display.color565(70, 70, 70));
        g->drawString("up", ox + w - 122, oy + h - 34);
        g->fillRoundRect(ox + w - 80, oy + h - 56, 64, 44, 8,
                         M5.Display.color565(70, 70, 70));
        g->drawString("down", ox + w - 48, oy + h - 34);
    }
    g->setTextDatum(top_left);

    // One write of final pixels. The rounded corners are square in the sprite
    // and would paint over the map, so they are keyed out - same trick, and
    // same reasoning, as drawButton.
    if (g_pinCvOk) {
        // Not a colour the panel uses: the fills are greys, greens and rusts,
        // the text white and pale red.
        const uint16_t KEY = 0xF81F;
        for (int i = 0; i < 14; i++) {
            for (int j = 0; j < 14; j++) {
                int dx = 14 - i, dy = 14 - j;
                if (dx * dx + dy * dy <= 196) continue;
                g_pinCv.drawPixel(i, j, KEY);
                g_pinCv.drawPixel(w - 1 - i, j, KEY);
                g_pinCv.drawPixel(i, h - 1 - j, KEY);
                g_pinCv.drawPixel(w - 1 - i, h - 1 - j, KEY);
            }
        }
        g_pinCv.pushSprite(x, y, KEY);
    }
}

// True when the tap belonged to the panel, so handleTouch stops there.
static bool pinPanelTouch(int px, int py, const GnssFix &fix) {
    if (!g_pinPanel) return false;
    int x, y, w, h; pinPanelRect(&x, &y, &w, &h);

    // A tap anywhere outside closes it. Requiring the close button would trap
    // anyone who did not find it, over a map they can no longer see.
    if (px < x || px >= x + w || py < y || py >= y + h) { pinPanelClose(); return true; }

    if (py < y + PP_HEAD_H - 10) {
        if (px > x + w - 200 && wp_add_fix(fix) >= 0) map_invalidate();
        return true;
    }

    if (py > y + h - PP_FOOT_H) {
        if (px < x + 170) {
            pinPanelClose();
        } else if (px < x + 400 && wp_target() >= 0) {
            wp_clear_target();
            map_invalidate();
        } else if (px > x + w - 154 && px < x + w - 86) {
            if (g_pinScroll > 0) g_pinScroll--;
        } else if (px > x + w - 86) {
            if (g_pinScroll + pinPanelRows() < wp_count()) g_pinScroll++;
        }
        return true;
    }

    int r = (py - (y + PP_HEAD_H)) / PP_ROW_H;
    int i = g_pinScroll + r;
    if (r >= 0 && r < pinPanelRows() && i < wp_count()) {
        if (px > x + w - 84) {
            wp_remove(i);
            // The list just got shorter under the scroll offset, which would
            // otherwise leave the panel showing an empty page of a non-empty
            // list.
            while (g_pinScroll > 0 && g_pinScroll + pinPanelRows() > wp_count())
                g_pinScroll--;
            map_invalidate();
        } else {
            // Tapping the current target clears it, so the row is its own
            // toggle and the footer button is only ever a shortcut.
            wp_set_target(i == wp_target() ? -1 : i);
            pinPanelClose();
        }
    }
    return true;
}

static void handleTouch(const GnssFix &fix) {
    // No panel means no buttons and no wake zone: every hit test below asks
    // the display for its geometry. An earlier pass left this unguarded on the
    // reasoning that loop() was already gated on g_panelOk - but the gate is
    // on the *drawing* block further down, and handleTouch runs before it.
    if (!g_panelOk) return;
    static bool wasDown = false;
    bool down = M5.Touch.getCount() > 0;
    bool tapped = down && !wasDown;
    wasDown = down;
    if (!tapped) return;

    g_lastTouchMs = millis();

    // Waking takes a press in the middle ninth, and that press does nothing
    // else - waking straight into a button would let one tap turn the screen
    // off again.
    if (g_screenOff) {
        auto w = M5.Touch.getDetail();
        if (inWakeZone(w.x, w.y)) screenOn();
        return;
    }

    auto t = M5.Touch.getDetail();

    // The panel is over everything, so it gets first refusal on the tap -
    // otherwise a row landing on a footer button would trigger both.
    if (pinPanelTouch(t.x, t.y, fix)) return;

    // Pan by thirds: a tap on the map steps the view one third of a screen
    // towards wherever it landed.
    //
    // Discrete taps rather than a drag. A drag would have to be tracked across
    // frames, throttled so the renderer is not handed tiles faster than it can
    // serve them, and given something to draw in the meantime - and on a
    // moving vehicle a nine-square tap target is easier to hit than a gesture
    // is to perform. The screen is divided the same way the follow band is,
    // so the middle square is a deliberate no-op: it is the region the marker
    // normally occupies, and it is also the wake zone, so nothing about a tap
    // there should move the map out from under it.
    //
    // Tested before buttonAt() would matter but after the pin panel, and
    // bounded above the footer's padded hit zone so an overshot button press
    // pans nothing.
    {
        int W = M5.Display.width();
        int mapTop = UI_STATUS_H;
        int mapBot = M5.Display.height() - BTN_H - BTN_M - BTN_PAD_TOP;
        if (t.y >= mapTop && t.y < mapBot) {
            int col = (t.x * 3) / W;
            int row = ((t.y - mapTop) * 3) / (mapBot - mapTop);
            // Clamps against a touch controller reporting a coordinate at
            // or past the panel edge, which would index a fourth column or
            // row that does not exist.
            if (col < 0) col = 0;
            if (col > 2) col = 2;
            if (row < 0) row = 0;
            if (row > 2) row = 2;
            int dx = col - 1, dy = row - 1;
            if (dx || dy) {
                if (!map_pan_step(dx, dy))
                    Serial.println("pan: nothing to pan - no fix has centred "
                                   "the grid yet");
            }
            g_confirmUntil = 0;
            return;
        }
    }

    int b = buttonAt(t.x, t.y);
    if (b != BTN_CACHE) g_confirmUntil = 0;

    switch (b) {
    case BTN_SLEEP:
        screenOff();
        break;

    case BTN_PINS:
        if (g_pinPanel) pinPanelClose();
        else            pinPanelOpen();
        break;

    case BTN_POI:
        map_set_labels(!map_labels_on());
        break;

    case BTN_WIFI:
        if (!wifiloc_available()) {
            Serial.println("wifiloc: unavailable - no PSRAM table was "
                           "allocated, see the boot log");
            break;
        }
        // Toggling off keeps the database and stops both learning and
        // locating. Worth having as a control rather than a rebuild: a scan
        // interrupts the association, so somebody downloading tiles over a
        // marginal link has a reason to stop this for a while.
        wifiloc_set_enabled(!wifiloc_enabled());
        break;

    case BTN_HOME:
        // Harmless when already following, and deliberately still live: it is
        // the control you reach for when you are not sure whether the map is
        // showing you or somewhere you left it.
        map_pan_reset();
        break;

    case BTN_MAG:
        if (!maglog_available()) {
            Serial.println("maglog: nothing to log to - no filesystem mounted");
            break;
        }
        maglog_set_enabled(!maglog_enabled());
        break;

    case BTN_CAL:
        if (!compass_ok()) break;
        // Tapping an already-calibrated compass refines it rather than
        // throwing the accumulated sweep away - repeated tumbles converge
        // instead of each one starting over. Delete /compasscal.bin to force
        // a genuinely fresh start.
        //
        // The log is stopped for the duration either way: compass_update()
        // publishes no sample while calibrating, so maglog_poll() would find
        // nothing new and write nothing, but flushing here means the rows
        // taken before the sweep are on the card before the device starts
        // being waved about.
        maglog_flush();
        if (compass_calibrating()) compass_calibrate_cancel();
        else                       compass_calibrate_start_refine();
        break;

    case BTN_THEME:
        g_themeMode = (ThemeMode)((g_themeMode + 1) % 3);
        Serial.printf("theme: %s\n",
                      g_themeMode == THEME_AUTO ? "auto" :
                      g_themeMode == THEME_DAY  ? "day" : "night");
        break;

    case BTN_BRIGHT:
        // auto -> low -> medium -> high -> auto. Cycling back to auto rather
        // than stopping at the top matters: a manual level is a temporary
        // correction for a condition the sun calculation cannot see, and the
        // condition passes. Leaving no way back short of a reboot would turn
        // every tunnel into a permanently wrong screen.
        //
        // The order runs upward from auto so that the first press from
        // automatic is downward in light, which is the direction it is
        // usually wanted: the complaint that prompts reaching for this button
        // is far more often "too bright" than "too dim", because the
        // too-bright case is painful and the too-dim case is merely
        // inconvenient.
        g_brightMode = (BrightMode)((g_brightMode + 1) % 4);
        // Applied on the next pass by applyTheme(), which owns g_brightness
        // and the screen-off guard. Setting the backlight here would bypass
        // both and light a sleeping panel.
        Serial.printf("bright: %s (%u)\n",
                      g_brightMode == BRIGHTM_AUTO ? "auto" : "fixed",
                      (unsigned)brightnessWanted());
        break;

    case BTN_CACHE:
        if (map_prefetch_busy()) break;
        // Checked before wifi: with a planet file on the card there is
        // nothing to download, so demanding an association first - and
        // opening the setup portal over the map to get one - is asking the
        // user to solve a problem they do not have.
        if (map_prefetch_pending(PREFETCH_RADIUS, Z_WIDE, Z_CLOSE) == 0) {
            Serial.println("cache: this area is already offline on the card");
            break;
        }
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("wifi: opening setup portal from button");
            wifiSetPins();
            portal_run(300000);
            // No configTime here either - ntpPolicy() owns that decision and
            // will ask on the next loop if the receiver still has no date.
            // The portal painted over everything. Without this the engine
            // sees an unchanged view, unchanged tiles and an unchanged
            // marker, skips the repaint, and leaves a lit but empty screen
            // until something else happens to move.
            M5.Display.fillScreen(style_background());
            map_invalidate();
            g_btnShownValid = false;
            break;
        }
        if (millis() < g_confirmUntil) {
            g_confirmUntil = 0;
            map_prefetch_start(PREFETCH_RADIUS, Z_WIDE, Z_CLOSE);
        } else {
            g_confirmUntil = millis() + 5000;
            Serial.println("prefetch: tap again within 5s to start");
        }
        break;

    default:
        break;
    }
}

static void handlePowerButton() {
    // Any press at all is treated as "something is about to happen".
    // Cheap to act on, and the alternative is guessing which gesture the
    // PMU maps to shutdown on this particular board.
    if (M5.BtnPWR.wasPressed() || M5.BtnPWR.wasClicked()) {
        Serial.println("power: button activity, flushing cache");
        tilecache_flush();
        maglog_flush();
        wifiloc_flush();
    }
}

// ---- boot screen -----------------------------------------------------------
// Startup takes several seconds - SD mount, wifi association, SNTP, archive
// open - and a blank panel through all of it gives no indication whether the
// device is working or hung. Each step reports as it happens.
static int  g_bootLine = 0;
static bool g_bootActive = true;

static void bootBegin() {
    if (!g_panelOk) return;   // no display attached
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(3);
    M5.Display.drawString("Tab5 Map", 40, 36);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.drawString("starting up", 40, 76);
    g_bootLine = 0;
    g_bootActive = true;
}

static void bootStepEx(const char *msg, bool ok, bool pending) {
    // Logged first, unconditionally. Everything below depends on the panel
    // being up, and when it is not, height() is 0, the geometry check below
    // returns early, and the serial trace disappears along with the screen -
    // which is exactly the boot where the trace is needed most.
    Serial.printf("boot: %-6s %s\n", pending ? "..." : (ok ? "ok" : "FAIL"), msg);

    if (!g_bootActive || !g_panelOk) { if (!pending) g_bootLine++; return; }
    int y = 130 + g_bootLine * 30;
    if (y > M5.Display.height() - 40) return;
    // A pending line is overwritten in place by its result, and the result is
    // often shorter than the message it replaces ("connecting to wifi" ->
    // "wifi 192.168.5.62"). Without clearing the row first, the tail of the
    // longer text stays on screen underneath the new one.
    M5.Display.fillRect(0, y - 2, M5.Display.width(), 28, TFT_BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(pending ? TFT_YELLOW : (ok ? TFT_GREEN : TFT_RED));
    M5.Display.drawString(pending ? "..." : (ok ? " ok " : "fail"), 40, y);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.drawString(msg, 110, y);
    if (!pending) g_bootLine++;
}

// No default arguments: the Arduino preprocessor copies them into the
// prototype it generates, and the definition then repeats them, which is a
// compile error rather than a warning.
static void bootStep(const char *msg)          { bootStepEx(msg, true, false); }
static void bootStepFail(const char *msg)      { bootStepEx(msg, false, false); }
static void bootStepBusy(const char *msg)      { bootStepEx(msg, true, true); }

static void bootEnd() {
    if (!g_panelOk) return;   // no display attached
    // Idempotent: setup() now hands the screen over as soon as the map has
    // something to show, and again at the end for the paths that never got
    // that far. A second fillScreen would wipe a map that is already up.
    if (!g_bootActive) return;
    g_bootActive = false;
    // The world check is the last thing on the boot screen and the boot
    // screen is about to be painted over, so its half-megabyte goes back to
    // the heap here rather than being held for the run.
    map_world_check_free();
    M5.Display.fillScreen(style_background());
}

// Mount the card, trying the fast bus first and falling back.
//
// `allow_format` must never be set on the ordinary boot path, and the reason
// is this function's own fallback order. SD_MMC.begin() takes a
// format_if_mount_failed flag, and putting it on the 4-bit attempt would mean
// a card that merely needs 1-bit mode - marginal signal integrity, long
// traces, a cheap card - gets erased instead of falling through to the attempt
// that would have worked. The same applies to a card holding a filesystem this
// build cannot read: exFAT, which is what most cards over 32 GB ship with,
// fails to mount and would be silently reformatted along with everything on
// it.
//
// So formatting is a last resort, after every bus has been tried, and only
// when something above has established that the user actually asked for it.
// Defined below. waitForStorage() sits above both because it belongs with the
// boot sequence it serves, not with the mount primitives it calls.
static bool mountSD(const char **bus, bool allow_format = false);
static bool confirmFormat();

// Quieten the storage stack while polling an empty slot.
//
// A failed mount is the *expected* result here, once a second, and the drivers
// below have no idea of that - between them they produce five lines and two
// "HINT: Please reboot the board" suggestions per attempt, which is the entire
// console for as long as the slot stays empty.
//
// Only the ESP-IDF tags can be reached this way. The two Arduino-core lines
// (SD_MMC.cpp and sd_diskio.cpp) go through ARDUHAL's log_printf, which ends
// in ets_printf - not esp_log_write - so no runtime call can silence them and
// esp_log_set_vprintf does not see them either. They are compiled in at
// CORE_DEBUG_LEVEL. The lever for those is CONFIG_ARDUHAL_LOG_DEFAULT_LEVEL in
// sdkconfig.defaults, which is deliberately not touched: it would take every
// Arduino-core error with it, and core errors are how the wifi and SD paths
// report themselves. Instead the poll simply causes fewer of them - see
// mountSDQuick().
//
// SD_HOST is in the list for sdmmc_host_deinit(), which logs
// "invalid argument: null pointer" at ERROR every time it is called on a host
// that is already down - the ordinary case in a probe loop.
static void sdQuiet(bool quiet) {
    static const char *tags[] = {
        "sdmmc_common", "sdmmc_init", "sdmmc_sd", "sdmmc_io", "sdmmc_mmc",
        "sdmmc_periph", "vfs_fat_sdmmc", "SD_HOST",
        // Arduino's SD_MMC builds an on-chip LDO power control for the slot on
        // every begin() (SD_MMC.cpp:267, which sets .ldo_chan_id and leaves
        // voltage_mv at zero), so the regulator complains once per attempt.
        // Whether it should be doing that at all is a separate question - see
        // the board variant note in CMakeLists.txt - but it is not something a
        // probe loop needs to say a hundred times.
        "ldo",
    };
    for (auto t : tags) esp_log_level_set(t, quiet ? ESP_LOG_NONE : ESP_LOG_INFO);
}

// The probe used once a second, as opposed to the full ladder used once at
// boot.
//
// SDMMC only. The SPI fallback in mountSD() costs about a second per attempt
// and two more error lines, and this board's slot is wired for SDMMC - SPI is
// there for the boot attempt's sake, not because it is ever expected to be the
// answer here.
//
// sdmmc_host_deinit() after every failure is not optional. A failed mount
// leaves the host initialised with its slot GPIOs still checked out;
// SD_MMC.end() does not take it down, because Arduino's end() only unmounts
// what it believes it mounted. Without this every later attempt logs
// "SDMMC host already initialized, skipping init flow" and reuses a host that
// was left in whatever state the failure put it in.
static bool mountSDQuick(const char **bus) {
    if (SD_MMC.begin("/sdcard", false, false, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true, false, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES))  { *bus = "SDMMC 1-bit"; return true; }
    SD_MMC.end();
    sdmmc_host_deinit();
    return false;
}

// Wait for something to appear in the microSD slot or the USB-A port.
//
// This replaces "insert a card and restart". Everything the program draws
// lives on removable media, so a boot without any is not a degraded mode, it
// is a stop - but stopping and asking for a power cycle is a poor way to say
// so when the fix is to plug something in and the device is sitting right
// there with a screen on it.
//
// Polled at 1 Hz, because there is nothing to interrupt on. The microSD
// connector's card-detect switch is not wired to the SoC on this board - M5's
// BSP passes GPIO_NUM_NC for it - so presence is discovered by trying to mount
// and treating failure as an empty slot. That is also why the loop is quiet:
// at one attempt per second, logging every failure is the whole console.
//
// SD_MMC.end() after each failed attempt is not optional. A failed mount
// leaves the SDMMC host initialised with its slot GPIOs still checked out, and
// the next attempt re-registers them - which the GPIO driver reports as
// "conflict found for GPIO[42]", once per second, forever, starting the moment
// you pull a card.
static bool waitForStorage(const char **bus) {
    uint32_t started = millis();
    uint32_t lastSay = 0;
    bool announced = false;

    sdQuiet(true);

    for (;;) {
        // USB first, matching pick()'s preference - and cheap, since a drive
        // that is not there costs a function call, while an absent card costs
        // two mount attempts and their timeouts.
        //
        // A drive enumerates on the USB driver's own task, so this only ever
        // sees one a poll or two after it is plugged in.
        // On the main task, deliberately: storage_usb_power() writes the
        // expander on the internal I2C bus and pick() no longer does it for
        // us. Idempotent after the first call.
        storage_usb_begin();
        storage_rescan();
        if (storage_available()) { sdQuiet(false); *bus = storage_name(); return true; }

        if (mountSDQuick(bus)) { sdQuiet(false); return true; }

        // A card that is present but unreadable is a different problem, and the
        // one case where formatting is worth offering. Ask once - confirmFormat()
        // blocks on the screen, and asking again every second would be a trap
        // rather than a prompt.
        static bool askedFormat = false;
        if (!askedFormat && storage_card_present()) {
            askedFormat = true;
            sdQuiet(false);
            Serial.println("sd: card present but no readable filesystem");
            if (confirmFormat()) {
                bootBegin();
                bootStepBusy("formatting card");
                if (mountSD(bus, true)) { bootStep("card formatted"); return true; }
                bootStepFail("format failed");
                SD_MMC.end();
                sdmmc_host_deinit();
            }
            sdQuiet(true);
        }

        if (!announced) {
            announced = true;
            Serial.println("sd: no card - waiting for a card or a USB drive "
                           "(no restart needed)");
            if (g_panelOk) {
                M5.Display.setTextColor(TFT_YELLOW);
                M5.Display.setTextSize(2);
                M5.Display.drawString("Insert a microSD card or a USB drive",
                                      40, M5.Display.height() - 90);
                M5.Display.setTextColor(TFT_DARKGREY);
                M5.Display.drawString("Waiting - nothing else to do",
                                      40, M5.Display.height() - 60);
            }
        }

        // A line a minute, so a log left running overnight still says what the
        // device was doing without being all it says.
        if (millis() - lastSay > 60000) {
            lastSay = millis();
            Serial.printf("sd: still waiting (%lu s)\n",
                          (unsigned long)((millis() - started) / 1000));
        }

        // Back off. Someone standing at the device with a card in their hand is
        // served by the first few seconds being responsive; a device left
        // waiting is served by not filling the console for the next hour. The
        // two Arduino-core error lines per attempt cannot be silenced, so the
        // only control over them is how often they happen.
        uint32_t waited = millis() - started;
        uint32_t gap = waited < 15000 ? 1000 : waited < 60000 ? 3000 : 10000;

        // M5.update() keeps touch alive for confirmFormat().
        for (uint32_t i = 0; i < gap / 50; i++) { M5.update(); delay(50); }
    }
}

// The trailing arguments to SD_MMC.begin() are the library's own defaults for
// frequency, and STORAGE_MAX_OPEN_FILES for the descriptor count - which is
// not a default. Arduino's is 5, and five is not enough to hold a set of band
// archives open alongside the tile cache.
static bool mountSD(const char **bus, bool allow_format) {
    if (SD_MMC.begin("/sdcard", false, false, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true, false, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES))  { *bus = "SDMMC 1-bit"; return true; }
    if (SD.begin())                     { *bus = "SPI";         return true; }

    if (!allow_format) return false;

    // Every read-only attempt has failed by now, so there is nothing here that
    // this build could have opened.
    Serial.println("sd: formatting card (FAT, all existing data lost)");
    if (SD_MMC.begin("/sdcard", false, true, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES)) { *bus = "SDMMC 4-bit, formatted"; return true; }
    if (SD_MMC.begin("/sdcard", true,  true, BOARD_MAX_SDMMC_FREQ,
                     STORAGE_MAX_OPEN_FILES)) { *bus = "SDMMC 1-bit, formatted"; return true; }
    return false;
}

// Ask before erasing anything. Two deliberate touches, the same shape as the
// cache button, because the cost of a stray tap here is the whole card.
static bool confirmFormat() {
    // Nothing to ask on. Erasing a card without being able to show the warning
    // is not a decision to make on the user's behalf.
    if (!g_panelOk) {
        Serial.println("sd: no display, not offering to format");
        return false;
    }
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextSize(3);
    M5.Display.setTextColor(TFT_RED);
    M5.Display.drawString("No readable filesystem", 40, 60);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.drawString("The card did not mount on any bus.", 40, 130);
    M5.Display.drawString("It may be unformatted, or formatted as exFAT,", 40, 162);
    M5.Display.drawString("which this build cannot read.", 40, 194);
    M5.Display.setTextColor(TFT_YELLOW);
    M5.Display.drawString("Touch twice to format it as FAT.", 40, 250);
    M5.Display.drawString("EVERYTHING ON THE CARD WILL BE LOST.", 40, 282);
    M5.Display.setTextColor(TFT_LIGHTGREY);
    M5.Display.drawString("Do nothing to leave it untouched.", 40, 340);

    for (int taps = 0; taps < 2; taps++) {
        uint32_t t0 = millis();
        bool got = false;
        while (millis() - t0 < 8000) {
            M5.update();
            if (M5.Touch.getCount()) {
                got = true;
                while (M5.Touch.getCount()) { M5.update(); delay(20); }  // release
                break;
            }
            delay(20);
        }
        if (!got) { Serial.println("sd: format declined"); return false; }
        if (taps == 0) {
            M5.Display.setTextColor(TFT_RED);
            M5.Display.setTextSize(3);
            M5.Display.drawString("Touch again to confirm", 40, 400);
        }
    }
    Serial.println("sd: format confirmed by user");
    return true;
}

static void pickZoom(const GnssFix &fix) {
    // Nothing to choose between when both levels are the same, and skipping
    // early avoids the hysteresis timer holding a change that never comes.
    if (Z_WIDE == Z_CLOSE) return;
    if (!gnss_fine(fix)) return;
    uint32_t now = millis();
    if (now - g_lastZoomChange < ZOOM_HOLD_MS) return;

    uint8_t cur = map_zoom(), want;
    double v = fix.speedKmh;
    // Separate up and down thresholds; the dead band between them stops a
    // vehicle sitting near a boundary from thrashing the grid, and every
    // change invalidates all nine tiles.
    if (v > 25.0)                         want = Z_WIDE;
    else if (v > 15.0 && cur == Z_WIDE)   want = Z_WIDE;
    else                                  want = Z_CLOSE;

    if (want != cur) { map_set_zoom(want, fix); g_lastZoomChange = now; }
}

// Clock and battery, right-aligned in the status bar. Always UTC.
//
// The clock prefers the system time, which SNTP sets and the RTC holds
// across a reboot; GNSS is the fallback, correct as soon as there is a fix.
// Both sources are UTC, so neither needs converting.
// `g` is where this draws. The status bar composes into an off-screen canvas
// whose origin coincides with the panel's, so the coordinates are the same
// either way and only the target changes.
// Has the receiver ever seen a satellite?
//
// The M135 has no antenna detection in hardware. The NEO-M9N supports a
// u-blox antenna supervisor, but it needs ANT_DETECT / ANT_SHORT_N wired to
// something, and on this module pins 14-17 (LNA_EN and the three Reserved)
// are all unconnected - so UBX-MON-RF would report aStatus as INIT/DONTKNOW
// rather than OK/OPEN/SHORT. There is nothing to read.
//
// What can be inferred: with no antenna the RF input sees essentially nothing,
// so GSV reports no satellites at all, on any constellation, forever. With an
// antenna - even indoors, even with no fix - a few SVs normally appear with
// low C/N0 within a minute or so.
//
// This is a hint, not a measurement. A receiver deep inside a building can
// also report nothing, which is why the banner asks rather than states. The
// timeout is well past the NEO-M9N's typical cold-start TTFF so a slow fix in
// poor conditions does not trip it.
#ifndef ANTENNA_SUSPECT_MS
#define ANTENNA_SUSPECT_MS 90000
#endif


// ---- AssistNow Autonomous store --------------------------------------------
// The receiver's predicted orbits, parked on the card so they outlive the
// V_BCKP supercap.
//
// Freshness is the whole point: predictions are good for roughly three days,
// and a stale database is worse than none - it would be pushed, rejected or
// used badly, and cost time at exactly the moment a fast fix matters. So the
// file carries the time it was written and is ignored once it is too old.
//
// Written when there is a good fix, because that is when the receiver has
// actually observed the ephemeris the predictions are built from. Polling a
// receiver that has never seen a satellite returns nothing worth keeping.
static const char *AOP_PATH      = "/aopdb.bin";
// src: chosen. ASCII "AOP1"; this project's own container, not u-blox's.
static const uint32_t AOP_MAGIC  = 0x414F5031;   // "AOP1"
// src: u-blox AssistNow Autonomous - the receiver computes orbit
//      predictions valid for up to three days ahead, so a database older
//      than that has nothing useful left in it. Stated in the M8 receiver
//      description under AssistNow Autonomous; confirm against the
//      document for your own module, the span is generation-dependent.
static const uint32_t AOP_MAX_AGE_H = 72;
static const size_t   AOP_CAP    = 16 * 1024;

struct AopHeader {
    uint32_t magic;
    uint32_t bytes;
    int64_t  written_utc;    // seconds; 0 if the clock was not trusted
};

static uint32_t g_aopSavedAt = 0;
static bool     g_aopRestored = false;   // assistance was pushed this boot
static double   g_aopAgeHours = -1.0;    // age of what was pushed, -1 unknown
static uint32_t g_gnssStartMs = 0;       // when the receiver got power

// ---- network time ----------------------------------------------------------
// SNTP is a fallback, not the default.
//
// Everything downstream needs a UTC date and nothing else: netsource uses it
// for build discovery, and the sun calculation needs the day of the year. The
// receiver supplies exactly that in every RMC sentence, for free, with no
// network - and this device is built to work with no network at all.
//
// Asking for it on every join was harmless but wrong-headed: it spends a DNS
// lookup and a round trip on something a receiver already searching will
// answer by itself, and it means an offline boot silently behaves differently
// from an online one for reasons unrelated to the map.
//
// So the request is deferred until both sources have had a fair chance, and
// then made only if the receiver has not answered. "A fair chance" is measured
// from g_gnssStartMs - when the receiver got power, which is what TTFF is also
// measured from - rather than from boot; the grace is longer than an assisted
// fix takes and
// shorter than an unassisted cold start, which is the window where the network
// is genuinely the faster answer.
//
// src: 45 s is a judgement, unattributed - see PROVENANCE.md. An assisted
// first fix was measured at 27 s on this unit; a cold one can take minutes.
static const uint32_t NTP_GRACE_MS = 45000;

static bool g_ntpDecided = false;

static void ntpPolicy(const GnssFix &fix) {
    if (g_ntpDecided) return;

    // The receiver got there first. Nothing further is needed - and saying so
    // matters, because otherwise the absence of an SNTP line in the log looks
    // like a failure rather than a decision.
    if (fix.status == 'A' && fix.date[0]) {
        g_ntpDecided = true;
        Serial.println("time: UTC from GNSS, SNTP not needed");
        return;
    }

    if (!g_gnssStartMs || millis() - g_gnssStartMs < NTP_GRACE_MS) return;

    // Grace expired with no date from the sky. If there is a network, ask it.
    if (WiFi.status() != WL_CONNECTED) return;

    g_ntpDecided = true;
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.printf("time: no GNSS date after %lu s - requesting SNTP\n",
                  (unsigned long)(NTP_GRACE_MS / 1000));
}


static void aopRestore() {
    fs::FS *fs = storage_fs();
    if (!fs || !fs->exists(AOP_PATH)) { Serial.println("aop: no stored database"); return; }

    File f = fs->open(AOP_PATH, FILE_READ);
    if (!f) return;

    AopHeader h;
    if (f.read((uint8_t *)&h, sizeof h) != (int)sizeof h || h.magic != AOP_MAGIC) {
        Serial.println("aop: stored database unrecognised, ignoring");
        f.close();
        return;
    }

    time_t now = time(nullptr);
    if (h.written_utc > 0 && now > 100000) {
        double age_h = (double)(now - h.written_utc) / 3600.0;
        if (age_h > AOP_MAX_AGE_H) {
            Serial.printf("aop: stored database is %.1f h old, past %lu h - "
                          "not using it\n", age_h, (unsigned long)AOP_MAX_AGE_H);
            f.close();
            return;
        }
        Serial.printf("aop: stored database %.1f h old, %lu bytes\n",
                      age_h, (unsigned long)h.bytes);
        g_aopAgeHours = age_h;
    } else {
        // No trustworthy clock at either end. Pushing anyway is the better bet:
        // the receiver validates what it is given, and the alternative is a
        // cold start.
        Serial.printf("aop: stored database %lu bytes, age unknown\n",
                      (unsigned long)h.bytes);
    }

    if (h.bytes == 0 || h.bytes > AOP_CAP) { f.close(); return; }
    uint8_t *buf = (uint8_t *)malloc(h.bytes);
    if (!buf) { f.close(); return; }
    if (f.read(buf, h.bytes) == (int)h.bytes) g_aopRestored = gnss_dbd_write(buf, h.bytes);
    free(buf);
    f.close();
}

// ---- remembered position, for the boot palette -----------------------------
//
// The map used to spend the whole of boot and the first half-minute of the GPS
// search painted in the day palette, then flip to night the moment the first
// fix landed. In the dark that is a bright grey screen followed by a jolt.
//
// The device can answer the question before then. It has an RTC, and the last
// place it had a fix is a good enough position for a sunrise calculation - a
// sunrise time is not sensitive to a few hundred kilometres, and a device that
// has moved further than that between boots gets one wrong palette until the
// first fix corrects it.
//
// Deliberately tiny and deliberately best-effort: no file, no clock or no fix
// yet all fall back to what the code did before.
static const char *LASTFIX_PATH = "/lastfix.bin";
static const char *CAL_PATH     = "/compasscal.bin";
struct LastFix {
    uint32_t magic;
    float    lat, lon;
    int64_t  written_utc;
};
// src: chosen. ASCII "LFX1", same reasoning as AOP_MAGIC.
static const uint32_t LASTFIX_MAGIC = 0x4C465831u;   // "LFX1"

static bool lastFixLoad(double *lat, double *lon) {
    fs::FS *fs = storage_fs();
    if (!fs || !fs->exists(LASTFIX_PATH)) return false;
    File f = fs->open(LASTFIX_PATH, FILE_READ);
    if (!f) return false;
    LastFix lf{};
    bool ok = f.read((uint8_t *)&lf, sizeof lf) == (int)sizeof lf
              && lf.magic == LASTFIX_MAGIC;
    f.close();
    if (!ok) return false;
    *lat = lf.lat;
    *lon = lf.lon;
    return true;
}

static void lastFixSave(const GnssFix &fix) {
    fs::FS *fs = storage_fs();
    if (!fs) return;
    File f = fs->open(LASTFIX_PATH, FILE_WRITE);
    if (!f) return;
    LastFix lf{ LASTFIX_MAGIC, (float)fix.lat, (float)fix.lon, 0 };
    time_t now = time(nullptr);
    if (now > 100000) lf.written_utc = (int64_t)now;
    f.write((const uint8_t *)&lf, sizeof lf);
    f.close();
}

// ---- compass calibration on the card -------------------------------------
//
// compass.cpp owns the format and hands over an opaque blob; this side just
// writes bytes. Same tmp-and-rename dance as aopSave, for the same reason: an
// interrupted write must not leave a file that is valid enough to load.
static void calSave() {
    size_t n = compass_cal_blob_size();
    if (n == 0) return;

    fs::FS *fs = storage_fs();
    if (!fs) return;

    uint8_t *buf = (uint8_t *)malloc(n);
    if (!buf) return;
    if (compass_cal_export(buf, n) != n) { free(buf); return; }

    const char *tmp = "/compasscal.tmp";
    File f = fs->open(tmp, FILE_WRITE);
    if (!f) { free(buf); return; }
    bool ok = f.write(buf, n) == n;
    f.close();
    free(buf);

    if (ok) {
        fs->remove(CAL_PATH);
        ok = fs->rename(tmp, CAL_PATH);
    } else {
        fs->remove(tmp);
    }
    Serial.printf("compass: calibration %s to %s\n",
                  ok ? "saved" : "NOT saved", CAL_PATH);
}

static void calLoad() {
    size_t n = compass_cal_blob_size();
    if (n == 0) return;

    fs::FS *fs = storage_fs();
    if (!fs || !fs->exists(CAL_PATH)) return;

    File f = fs->open(CAL_PATH, FILE_READ);
    if (!f) return;
    if (f.size() != n) {
        // A size mismatch means the struct changed between builds. Removing it
        // is kinder than leaving a file that fails to load on every boot.
        f.close();
        fs->remove(CAL_PATH);
        Serial.println("compass: stored calibration is from another build, removed");
        return;
    }

    uint8_t *buf = (uint8_t *)malloc(n);
    if (!buf) { f.close(); return; }
    bool got = f.read(buf, n) == (int)n;
    f.close();
    if (got && !compass_cal_import(buf, n)) {
        fs->remove(CAL_PATH);
        Serial.println("compass: stored calibration failed its checks, removed");
    }
    free(buf);
}

// Pick the palette before anything is painted in it.
//
// Called once, after map_begin (map_set_dark needs the grid) and before
// bootEnd() fills the screen with the background colour. Everything it needs -
// a clock from the RTC, a position from the last run - is already in place by
// then: the RTC is read in setup(), and the card is mounted several steps
// earlier.
static void themeBoot() {
    if (g_themeMode != THEME_AUTO) {
        if (g_themeMode == THEME_NIGHT) map_set_dark(true);
        return;
    }

    double lat, lon;
    if (!lastFixLoad(&lat, &lon)) {
        Serial.println("theme: no remembered position, starting in day palette");
        return;
    }

    // src: chosen. 1767225600 is 2026-01-01T00:00:00Z, comfortably after this
    //      firmware was written and comfortably before any real use, so it
    //      separates "the RTC has a time" from "the RTC came up at zero or at
    //      its own power-on default" without rejecting a valid clock.
    time_t now = time(nullptr);
    if (now < 1767225600) {
        Serial.println("theme: no clock yet, starting in day palette");
        return;
    }

    bool dark = !sunIsUpAt(lat, lon);
    g_sunFromStored = true;      // recompute on the first real fix
    Serial.printf("theme: remembered %.3f,%.3f -> %s palette\n",
                  lat, lon, dark ? "night" : "day");
    if (dark) {
        map_set_dark(true);
        g_brightness = BRIGHT_NIGHT;
        if (g_panelOk && !g_screenOff) M5.Display.setBrightness(BRIGHT_NIGHT);
    }
}

static void aopSave() {
    uint8_t *buf = (uint8_t *)malloc(AOP_CAP);
    if (!buf) return;

    size_t n = gnss_dbd_read(buf, AOP_CAP);
    if (n == 0) { free(buf); return; }

    fs::FS *fs = storage_fs();
    if (!fs) { free(buf); return; }

    // Written to a temporary and renamed, so an interrupted write cannot leave
    // a half-file that looks valid enough to push into the receiver.
    const char *tmp = "/aopdb.tmp";
    File f = fs->open(tmp, FILE_WRITE);
    if (!f) { free(buf); return; }

    AopHeader h = { AOP_MAGIC, (uint32_t)n, 0 };
    time_t now = time(nullptr);
    if (now > 100000) h.written_utc = (int64_t)now;

    bool ok = f.write((const uint8_t *)&h, sizeof h) == sizeof h &&
              f.write(buf, n) == n;
    f.close();
    free(buf);

    if (ok) {
        fs->remove(AOP_PATH);
        ok = fs->rename(tmp, AOP_PATH);
    } else {
        fs->remove(tmp);
    }
    Serial.printf("aop: %s database, %u bytes\n", ok ? "saved" : "FAILED to save",
                  (unsigned)n);
}

// Save periodically once the fix is good. The predictions improve as the
// receiver observes more, so a later snapshot is a better one - but writing
// constantly would just wear the card for nothing.
#ifndef AOP_SAVE_INTERVAL_MS
#define AOP_SAVE_INTERVAL_MS (30 * 60 * 1000UL)
#endif

// The save runs on its own task, not on the loop that draws the screen.
//
// aopSave() polls the receiver and waits for the reply - normally a few hundred
// milliseconds, but up to eight seconds if the module does not answer. The main
// loop redraws at ~15 fps, so doing that inline would freeze the UI for as long
// as the exchange took, and the failure case is the longest one.
//
// Concurrent SD access is safe: ESP-IDF builds FatFs with FF_FS_REENTRANT 1, so
// same-volume file access is serialised inside the filesystem. This task and
// the render worker reading tiles cannot interleave badly.
static volatile bool g_aopBusy = false;

static void aopSaveTask(void *arg) {
    (void)arg;
    aopSave();
    g_aopBusy = false;
    vTaskDelete(nullptr);
}


// Time to first fix, measured from when the receiver was powered.
//
// Not from gnss_start(): the module is searching throughout SD mount and wifi
// association, so timing from the point the sketch starts listening would
// subtract several seconds of work the receiver had already done and report a
// TTFF better than the device actually delivers.
//
// Printed with what assistance was actually pushed, because a TTFF number is
// meaningless without knowing which case produced it. Comparing an assisted
// boot against a cold one is the whole test.
static void ttffReport(const GnssFix &fix) {
    static bool anyDone = false, fineDone = false;
    if (fineDone) return;
    uint32_t t0 = g_gnssStartMs ? g_gnssStartMs : 0;

    if (!anyDone && gnss_coarse(fix)) {
        anyDone = true;
        // gnss_first_coarse_ms(), not millis(): this runs from loop(), and
        // loop() does not turn during setup(). See gnss.h.
        Serial.printf("gnss: first fix after %.1f s (%s), %d sats\n",
                      (gnss_first_coarse_ms() - t0) / 1000.0,
                      g_aopRestored
                        ? (g_aopAgeHours >= 0
                             ? "assisted" : "assisted, age unknown")
                        : "no assistance",
                      fix.sats);
    }
    if (anyDone && !fineDone && gnss_fine(fix)) {
        fineDone = true;
        Serial.printf("gnss: 3D fix hdop %.2f after %.1f s\n",
                      fix.hdop, (gnss_first_fine_ms() - t0) / 1000.0);
    }
}

static void aopMaintain(const GnssFix &fix) {
    if (g_aopBusy) return;
    if (!gnss_fine(fix)) return;
    if (g_aopSavedAt && millis() - g_aopSavedAt < AOP_SAVE_INTERVAL_MS) return;
    // First save waits a few minutes after a good fix, to give the receiver
    // time to collect ephemeris worth predicting from.
    if (!g_aopSavedAt && millis() < 5 * 60 * 1000UL) return;

    // Stamped before the attempt, so a receiver that never answers backs off
    // for the full interval instead of being re-polled on the next frame.
    g_aopSavedAt = millis();
    g_aopBusy = true;

    // Low priority on core 0: the GNSS reader is priority 5 on the same core
    // and must keep draining the UART for this exchange to capture anything.
    if (xTaskCreatePinnedToCore(aopSaveTask, "aopsave", 8192, nullptr, 1,
                                nullptr, 0) != pdPASS) {
        g_aopBusy = false;
        Serial.println("aop: could not start save task");
    }
}

static bool antennaSuspect(const GnssFix &fix) {
    static uint32_t lastSeen = 0;
    static bool     everSeen = false;

    int visible = 0, snr = 0;
    for (int i = 0; i < 4; i++) {
        visible += fix.cons[i].visible;
        if (fix.cons[i].bestSnr > snr) snr = fix.cons[i].bestSnr;
    }
    if (visible > 0 || snr > 0) { lastSeen = millis(); everSeen = true; }

    // Once satellites have been seen, an antenna is present; losing them again
    // is a sky-view problem, not a cabling one, and saying otherwise would be
    // wrong every time the device goes under a bridge.
    if (everSeen) return false;
    (void)lastSeen;

    bool suspect = millis() > ANTENNA_SUSPECT_MS;
    // Said on the serial link as well as the screen, once. A log that shows
    // the map simply never rendering gives no hint that the receiver has been
    // sitting there with no signal the whole time - which is exactly the case
    // where someone is reading the log to find out why.
    static bool announced = false;
    if (suspect && !announced) {
        announced = true;
        Serial.printf("gnss: no satellite on any constellation after %lu s - "
                      "antenna disconnected, or no sky view\n",
                      (unsigned long)(ANTENNA_SUSPECT_MS / 1000));
    }
    return suspect;
}

// ---- battery ---------------------------------------------------------------
//
// Read the pack voltage and work out the percentage here rather than taking
// M5Unified's answer, because its answer depends on a pack assumption we
// cannot verify from inside the program.
//
// Power_Class::getBatteryLevel() for the Tab5 is:
//
//     mv = Ina226.getBusVoltage() * 500;              // "2S Li-Po (*1000/2)"
//     level = (mv - 3300) * 100 / (float)(4150 - 3350);
//
// The 500 is 1000/2: it converts the bus voltage to volts-per-cell on the
// assumption of two cells in series. If the pack is 1S, the halving makes a
// healthy 3.9 V read as 1950 mV per cell, which is below the 3300 floor, and
// the level clamps to 0 - permanently, at any state of charge. That is exactly
// what a stuck 0% looks like.
//
// getBatteryVoltage() on the same board returns the bus voltage unhalved, so
// it is the honest number and the one to reason from.
//
// MAP_BATTERY_CELLS says how many cells are in series. If 0%-when-full turns
// out to be the halving, set it to 1 and the reading is correct without
// touching M5Unified.
#ifndef MAP_BATTERY_CELLS
#define MAP_BATTERY_CELLS 2
#endif

// Per-cell millivolts for empty and full. Same endpoints M5Unified uses, and
// unremarkable for Li-Po - 3.3 V is about as low as a cell should be taken and
// 4.15 V is a full one at rest.
static const int CELL_MV_EMPTY = 3300;
static const int CELL_MV_FULL  = 4150;

// Percentage, or -1 when the pack voltage is not believable.
//
// "Not believable" is the important case. A missing or unresponsive INA226
// reads zero, and zero volts is not an empty battery - it is no measurement.
// Showing that as a red 0% is worse than showing nothing, because it is
// indistinguishable from a real flat pack.
//
// Read directly rather than through M5Unified, and cache the result.
//
// Two separate problems, one answer.
//
// The read itself: INA226_Class::readRegister16 pre-zeroes its buffer, calls
// the bus and returns buf[0]<<8|buf[1] without checking whether the transfer
// happened. A NACK and a genuine zero are the same value by the time
// getBatteryVoltage() sees it, so a single failed transaction is
// indistinguishable from a flat pack - and the code above quite correctly
// treats zero as "no measurement" and prints "no batt". One failure is enough
// to produce it. Reading the bus voltage register here means the ack is
// visible, so a failure can be retried instead of displayed.
//
// The rate: drawClockBattery() runs at the 15 fps the map is drawn at, so this
// was fifteen INA226 transactions and fifteen expander reads (isCharging) per
// second on the internal bus. That is the contention window that made a race
// with anything else on the bus a matter of when rather than whether. Once a
// second is more than enough for a battery, and the status bar reads the
// cached value.
//
// src: INA226 datasheet - register 0x02 is the bus voltage, 1.25 mV/LSB,
//      unsigned. M5Unified's Tab5 path reads the same register.
static const uint8_t INA226_ADDR   = 0x41;
static const uint8_t INA226_REG_V  = 0x02;
static const uint32_t INA226_FREQ  = 400000;

// How many consecutive failed polls before the reading is given up on.
//
// Not zero, and not one. A single dropped transaction is noise - a retry a
// second later costs nothing and the pack has not moved. Five seconds of
// silence is a sensor that is genuinely not answering, which is worth saying.
static const int BATT_MISS_MAX = 5;

// How long a reading may go unrefreshed before it stops being shown.
//
// Ninety seconds is chosen against the discharge rate, not against the poll
// rate: a pack that lasts hours cannot move a visible percentage point in that
// time, so a stale value is still a true one, while a "no batt" caused by a
// busy renderer is simply false.
static const uint32_t BATT_STALE_MS = 90000;

static uint32_t s_battGoodMs = 0;

static int  s_battMv    = 0;
static bool s_battOk    = false;
static bool s_battChg   = false;
static int  s_battMiss  = 0;

// One transaction. True only if the part actually acked.
// Why the last read failed. Kept separately because "the transfer did not
// happen" and "the transfer happened and the answer was zero" have different
// causes and different fixes, and collapsing them into one bool is the same
// mistake M5Unified makes one layer down.
enum InaFail { INA_OK, INA_NAK, INA_ZERO };
static InaFail s_inaLast = INA_OK;

static bool inaBusMillivolts(int *mv) {
    uint8_t b[2] = { 0, 0 };
    if (!M5.In_I2C.readRegister(INA226_ADDR, INA226_REG_V, b, 2, INA226_FREQ)) {
        s_inaLast = INA_NAK;
        return false;
    }
    uint16_t raw = (uint16_t)(b[0] << 8 | b[1]);
    if (raw == 0) {                             // acked, but not a measurement
        s_inaLast = INA_ZERO;
        return false;
    }
    s_inaLast = INA_OK;
    *mv = (int)raw * 5 / 4;                     // 1.25 mV/LSB
    return true;
}

// Read a 16-bit register, reporting whether the transfer happened at all.
static bool inaReg(uint8_t reg, uint16_t *out) {
    uint8_t b[2] = { 0, 0 };
    if (!M5.In_I2C.readRegister(INA226_ADDR, reg, b, 2, INA226_FREQ)) return false;
    *out = (uint16_t)(b[0] << 8 | b[1]);
    return true;
}

// Re-initialise the internal I2C controller.
//
// This is the actual repair, and it was found by accident: the diagnostic used
// to read the SDA/SCL levels through the GPIO driver and then re-begin() the
// bus to hand the pins back, and the register reads it performed immediately
// afterwards succeeded - having failed 64 polls in a row up to that moment.
// Nothing else in that path touches the device.
//
// What wedges it is a transaction that gets preempted and times out. During
// the first thirty seconds of a run the tile renderer takes the core almost
// completely - loop() managed two iterations in one measured window - and a
// register read is start, write, repeated start, read, stop, which is long
// enough to be interrupted in the middle. The controller is then left in a
// state where a bare address probe still completes (which is why every device
// on the bus appeared healthy) but a repeated start never does.
//
// It does not recover on its own. The starvation had been over for half a
// minute, with loop() back to thousands of iterations per window, and the
// reads were still failing.
static bool i2cReinit(void) {
    int sda = M5.In_I2C.getSDA();
    int scl = M5.In_I2C.getSCL();
    return M5.In_I2C.begin(M5.In_I2C.getPort(), sda, scl);
}

// Put the part back into continuous shunt-and-bus conversion.
//
// src: INA226 datasheet, CONFIG (0x00). Bits 2:0 are the operating mode, and
//      000 is power-down - in which every data register reads zero while the
//      device still acknowledges its own address, which is precisely the state
//      the probe could not see. 0x4527 is what this board boots with:
//      16 averages, 1.1 ms conversion times, mode 111.
static const uint16_t INA226_CFG_WANTED = 0x4527;

static bool inaConfigure(void) {
    uint8_t b[2] = { (uint8_t)(INA226_CFG_WANTED >> 8),
                     (uint8_t)(INA226_CFG_WANTED & 0xFF) };
    return M5.In_I2C.writeRegister(INA226_ADDR, 0x00, b, 2, INA226_FREQ);
}

// Why the reads stopped. Printed once per failure episode, because the answer
// decides what the fix is and guessing between the two costs a day.
//
// Three questions, in the order that narrows fastest:
//
//   1. Is anything else on the internal bus still answering? The GT911 touch
//      controller and the two I/O expanders live there. If 0x41 has gone quiet
//      and they have not, the bus is fine and the INA226 is the problem. If
//      they have all gone quiet together, the bus is wedged - a transaction
//      aborted mid-byte leaves a slave holding SDA low and nothing recovers on
//      its own.
//
//   2. What are the lines actually doing? SDA low with SCL high, with no
//      transaction in flight, *is* the wedge - there is no other way for that
//      state to exist at rest.
//
//   3. Was there memory to do the transfer with? M5Unified's I2C path
//      allocates on the internal heap, and a failed allocation returns false
//      without ever driving a pin. That looks identical from the call site and
//      is not a bus fault at all - see the USB MSC transfer buffer note in
//      storage.h, which is the other large consumer of internal DMA memory.
static void battDiagnose() {
    // src: M5Unified I2C_Class exposes the pins it was configured with, so
    //      this does not have to hardcode a board's wiring.
    int sda = M5.In_I2C.getSDA();
    int scl = M5.In_I2C.getSCL();

    Serial.printf("power: last good reading %lu ms ago\n",
                  (unsigned long)(millis() - s_battGoodMs));
    Serial.printf("power: INA226 silent for %d polls - internal heap %u B, "
                  "largest block %u B, largest DMA block %u B\n",
                  s_battMiss,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

    // Who is still there. A bare address probe, no register read, so a device
    // that is present but confused still shows up.
    static const struct { uint8_t addr; const char *what; } roll[] = {
        { 0x41, "INA226 (fuel gauge)" },
        { 0x43, "PI4IOE #1" },
        { 0x44, "PI4IOE #2" },
        { 0x14, "GT911 touch" },
        { 0x51, "RTC" },
    };
    int alive = 0;
    for (auto &d : roll) {
        bool ack = M5.In_I2C.start(d.addr, false, INA226_FREQ);
        M5.In_I2C.stop();
        if (ack) alive++;
        Serial.printf("power:   0x%02X %-20s %s\n",
                      d.addr, d.what, ack ? "ack" : "-");
    }

    // At rest both lines idle high. Anything else is the fault itself.
    // Reading the levels means taking the pins away from the I2C peripheral,
    // so they have to be handed back. The first version of this did not, and
    // every diagnosis permanently detached GPIO31/32 from the controller -
    // which turned a transient fault into the permanent one it was trying to
    // explain. Diagnostics must not be able to cause the thing they measure.
    int sda_lvl = digitalRead(sda);
    int scl_lvl = digitalRead(scl);
    i2cReinit();                        // hand the pins back to the controller
    Serial.printf("power:   SDA(GPIO%d)=%d SCL(GPIO%d)=%d\n",
                  sda, sda_lvl, scl, scl_lvl);

    // SDA low at rest is the wedge, whoever else is still answering.
    //
    // This used to require that nothing at all acked, which is too strict: a
    // GT911 will acknowledge its address while being useless for data, so one
    // stubborn ack was enough to route a genuinely stuck bus into the "this is
    // the part, not the bus" branch and skip the recovery. There is no benign
    // reading of a low SDA with no transaction in flight.
    if (sda_lvl == 0) {
        Serial.println("power: bus wedged - a slave is holding SDA low. "
                       "Clocking it out.");
        // Nine clocks with SDA released walks the stuck slave through the rest
        // of the byte it thinks it is sending, after which it sees a stop.
        // M5Unified's release() is exactly this; calling it rather than
        // bit-banging keeps the driver's idea of the bus state in step.
        M5.In_I2C.release();
        // begin() wants an i2c_port_t, and the class already knows which one
        // it was configured with - passing a literal here would hardcode a
        // board's wiring for no gain. I2C_NUM_0 is the Tab5 internal bus if
        // getPort() is missing from an older M5Unified; the external bus is
        // I2C_NUM_1.
        M5.In_I2C.begin(M5.In_I2C.getPort(), sda, scl);
        s_battMiss = 0;                 // give the recovered bus a fresh count
    } else if (alive == 0) {
        Serial.println("power: nothing on the internal bus answers, but SDA is "
                       "high - the controller is not transmitting. Suspect a "
                       "failed allocation rather than the wiring.");
    } else {
        // The bus is healthy and 0x41 acks, so the question is what the
        // registers say. An address probe cannot tell a working part from one
        // sitting in power-down: both acknowledge, and only the data registers
        // differ.
        uint16_t die = 0, cfg = 0, bus = 0;
        bool die_ok = inaReg(0xFF, &die);
        bool cfg_ok = inaReg(0x00, &cfg);
        bool bus_ok = inaReg(0x02, &bus);

        Serial.printf("power:   last read %s, die=0x%04X%s cfg=0x%04X%s "
                      "bus=0x%04X%s\n",
                      s_inaLast == INA_NAK  ? "did not complete"
                    : s_inaLast == INA_ZERO ? "completed and returned zero"
                                            : "succeeded",
                      die, die_ok ? "" : "(no read)",
                      cfg, cfg_ok ? "" : "(no read)",
                      bus, bus_ok ? "" : "(no read)");

        if (cfg_ok && (cfg & 0x0007) == 0) {
            // Mode 000. Every data register reads zero and the part still
            // acks, which is the whole reason the earlier probe said the
            // hardware was fine.
            Serial.printf("power: INA226 is in power-down (cfg=0x%04X) - "
                          "something cleared the mode bits after boot, where "
                          "they read 0x%04X. Reconfiguring.\n",
                          cfg, INA226_CFG_WANTED);
            if (inaConfigure()) {
                s_battMiss = 0;
                delay(5);               // one conversion at 1.1 ms averaged x16
            } else {
                Serial.println("power: the reconfigure write did not complete");
            }
        } else if (cfg_ok && cfg != INA226_CFG_WANTED) {
            Serial.printf("power: INA226 config has changed since boot "
                          "(0x%04X, was 0x%04X) but conversion is still "
                          "enabled - not the mode bits\n",
                          cfg, INA226_CFG_WANTED);
        } else if (s_inaLast == INA_NAK) {
            Serial.println("power: 0x41 acks its address but a register read "
                           "does not complete - the repeated start is what is "
                           "failing, not the device");
        } else {
            Serial.println("power: INA226 configured and answering, and still "
                           "reading zero volts - this is the part");
        }
    }
}

// Called from loop(). Rate-limited internally, so calling it more often than
// it polls is free.
static void batteryPoll(bool force) {
    static uint32_t last = 0;
    if (!force && millis() - last < 1000) return;
    last = millis();

    // Three attempts, spaced.
    //
    // A register read is start-write-restart-read-stop, and this task gets
    // preempted mid-sequence by the tile renderer - at which point the
    // transaction times out and returns false with nothing wrong anywhere.
    // A single retry two milliseconds later lands inside the same render, so
    // it fails for the same reason; spacing them gives at least one a chance
    // to fall in a gap. This is a workaround for the starvation below, not a
    // fix for it.
    int mv = 0;
    bool ok = false;
    for (int i = 0; i < 3 && !ok; i++) {
        if (i) delay(20);
        ok = inaBusMillivolts(&mv);
    }

    if (ok) {
        if (!s_battOk) Serial.println("power: INA226 answering again");
        s_battMv    = mv;
        s_battOk    = true;
        s_battMiss  = 0;
        s_battGoodMs = millis();
        // isCharging() is an expander read on the same bus, and just as
        // interruptible. Only worth attempting when the bus has just proved
        // it will carry a transaction.
        s_battChg = M5.Power.isCharging();
        return;
    }

    // Give up on wall-clock time, not on a poll count.
    //
    // The count was wrong because it assumed loop() runs at a steady rate, and
    // the whole problem is that it does not: at four iterations in thirty
    // seconds, five "consecutive" misses is a minute of real time, and the
    // status bar had already been lying for most of it. A pack voltage does
    // not move meaningfully in a minute, so the honest thing is to keep
    // showing the last real measurement and only admit defeat when it is old
    // enough to be worth doubting.
    s_battMiss++;

    // Three failures is enough to act on.
    //
    // Not because three is meaningful in itself, but because the failure this
    // recovers from is permanent: once the controller is wedged, waiting adds
    // nothing but a stale display. Rate-limited to one attempt every five
    // seconds so that a fault it cannot fix does not turn into a re-init loop.
    if (s_battMiss >= 3) {
        static uint32_t lastTry = 0;
        if (lastTry == 0 || millis() - lastTry > 5000) {
            lastTry = millis();
            bool ok = i2cReinit();
            // Said once per episode rather than once per attempt. If this line
            // appears regularly, the starvation that causes the wedge is the
            // thing to fix - this only clears up after it.
            static bool said = false;
            if (!said) {
                said = true;
                Serial.printf("power: internal I2C wedged after %d failed "
                              "reads - re-initialising the controller (%s)\n",
                              s_battMiss, ok ? "ok" : "failed");
            }
        }
    }

    if (s_battOk && millis() - s_battGoodMs > BATT_STALE_MS) {
        battDiagnose();
        s_battOk = false;
    }
}

static int batteryPercent(int *out_mv) {
    if (out_mv) *out_mv = s_battMv;
    if (!s_battOk) return -1;

    int mv = s_battMv;

    // Below one flat cell there is nothing plausible to report, whatever the
    // cell count.
    if (mv < 2500) return -1;

    int cell = mv / MAP_BATTERY_CELLS;
    int pct  = (cell - CELL_MV_EMPTY) * 100 / (CELL_MV_FULL - CELL_MV_EMPTY);
    return pct < 0 ? 0 : pct > 100 ? 100 : pct;
}

// Said once at boot, because a wrong battery reading is easy to notice and
// hard to diagnose without the raw numbers behind it.
static void powerReport() {
    batteryPoll(true);                  // nothing has polled yet at boot
    int mv = 0;
    int pct = batteryPercent(&mv);
    int m5  = (int)M5.Power.getBatteryLevel();
    bool chg = M5.Power.isCharging();

    Serial.printf("power: pack %d mV, %d cell(s) -> %d%%, M5Unified says %d%%, "
                  "charging %s\n",
                  mv, MAP_BATTERY_CELLS, pct, m5, chg ? "yes" : "no");

    if (mv < 2500)
        Serial.println("power: pack voltage reads zero - the INA226 on the "
                       "power bus is not answering, so no percentage is real");
    else if (m5 == 0 && pct > 5)
        Serial.println("power: M5Unified reads 0% where the voltage says "
                       "otherwise - its Tab5 path assumes a 2S pack; "
                       "build with -DMAP_BATTERY_CELLS=1 if this one is 1S");

    // CHG_STAT is expander 0x44 pin P6, and M5Unified treats a high level as
    // charging. Charger status pins are commonly active low, so print the raw
    // bit alongside its interpretation - if the sign is inverted, this is
    // where it shows.
    Serial.printf("power: CHG_STAT (expander 0x44 P6) reads %d\n",
                  M5.getIOExpander(1).digitalRead(6) ? 1 : 0);

    // Ask the fuel gauge itself, on the bus, without going through M5Unified.
    //
    // Everything above depends on Power_Class having reached its INA226, and
    // a zero from getBusVoltage() cannot tell "the chip said zero" from "the
    // read failed and the buffer was still zero" - INA226_Class::readRegister16
    // returns buf[0]<<8|buf[1] with buf pre-zeroed and no error check.
    //
    // Register 0xFF is the die ID and reads 0x2260 on a working part. That
    // number is the whole diagnosis:
    //
    //   0x2260  the chip is there and answering, so the fault is in how
    //           M5Unified is driving it, and the CONFIG register below says
    //           whether it was ever configured
    //   0x0000  nothing at 0x41 on the internal bus - wrong address for this
    //           unit, or the part is not populated
    //
    // Address and settings cross-checked against M5Stack's own Tab5 demo
    // (platforms/tab5/main/hal/hal_esp32.cpp), which does:
    //     ina226.begin(i2c_bus_handle, 0x41);
    //     ina226.configure(AVERAGES_16, BUS_CONV_1100US, SHUNT_CONV_1100US,
    //                      MODE_SHUNT_BUS_CONT);
    //     ina226.calibrate(0.005, 8.192);
    // M5Unified uses the same address and the same continuous mode, so if the
    // chip answers here, the difference is elsewhere.
    {
        uint8_t buf[2] = { 0, 0 };
        bool ok = M5.In_I2C.readRegister(INA226_ADDR, 0xFF, buf, 2, 400000);
        uint16_t die = (uint16_t)(buf[0] << 8 | buf[1]);

        uint8_t cfgb[2] = { 0, 0 };
        M5.In_I2C.readRegister(INA226_ADDR, 0x00, cfgb, 2, 400000);
        uint16_t cfg = (uint16_t)(cfgb[0] << 8 | cfgb[1]);

        uint8_t busb[2] = { 0, 0 };
        M5.In_I2C.readRegister(INA226_ADDR, 0x02, busb, 2, 400000);
        int16_t bus_raw = (int16_t)(busb[0] << 8 | busb[1]);

        Serial.printf("power: INA226 @0x%02X %s die=0x%04X cfg=0x%04X "
                      "bus_raw=%d (%.3f V)\n",
                      INA226_ADDR, ok ? "acked" : "NO ACK", die, cfg,
                      (int)bus_raw, bus_raw * 0.00125f);

        if (die != 0x2260)
            Serial.println("power: that is not an INA226 answering - the "
                           "reading cannot work until this does");
        else if ((cfg & 0x0007) == 0)
            Serial.println("power: INA226 is in power-down mode - it was "
                           "never configured, so every register reads stale");
    }
}

static void drawClockBattery(const GnssFix &fix, lgfx::LovyanGFX *g) {
    if (!g_panelOk) return;   // no display attached
    const int W = M5.Display.width();
    char buf[48];

    bool haveTime = false;
    struct tm lt;
    time_t now = time(nullptr);
    if (now > 1767225600) {                 // 2026-01-01, so plainly set
        gmtime_r(&now, &lt);
        haveTime = true;
    } else if (fix.utc[0] && fix.status == 'A') {
        // NMEA UTC is hhmmss.sss; no date arithmetic, so this is raw UTC.
        int hh = (fix.utc[0]-'0')*10 + (fix.utc[1]-'0');
        int mm = (fix.utc[2]-'0')*10 + (fix.utc[3]-'0');
        snprintf(buf, sizeof buf, "%02d:%02dZ", hh, mm);
        g->setTextDatum(top_right);
        g->setTextColor(TFT_WHITE);
        g->drawString(buf, W - 12, 6);
        g->setTextDatum(top_left);
        return;
    }

    int pct = batteryPercent(nullptr);
    bool charging = s_battChg;          // cached; see batteryPoll()

    // Say "no batt" rather than nothing when there is no measurement.
    //
    // Suppressing the percentage was right - a red 0% from an unread sensor is
    // a lie - but silence is its own kind of wrong: a reading that vanishes
    // looks like a bug in the status bar rather than an absent battery, and it
    // is indistinguishable from the clock simply having taken the space.
    const char *batt_txt = pct >= 0 ? nullptr : "no batt";

    if (haveTime && pct >= 0)
        snprintf(buf, sizeof buf, "%02d:%02dZ   %s%d%%",
                 lt.tm_hour, lt.tm_min, charging ? "+" : "", pct);
    else if (haveTime)
        snprintf(buf, sizeof buf, "%02d:%02dZ   %s", lt.tm_hour, lt.tm_min, batt_txt);
    else if (pct >= 0)
        snprintf(buf, sizeof buf, "%s%d%%", charging ? "+" : "", pct);
    else
        snprintf(buf, sizeof buf, "%s", batt_txt);

    // Colour follows the battery, since that is the part worth noticing at a
    // glance on a device you are carrying.
    uint16_t col = TFT_WHITE;
    if (pct < 0) {
        // Dim, because it is a statement about the sensor rather than a
        // warning about the battery. Nothing is wrong with the device.
        col = TFT_DARKGREY;
    } else if (!charging) {
        if (pct <= 10)      col = TFT_RED;
        else if (pct <= 25) col = TFT_ORANGE;
    } else {
        col = TFT_GREENYELLOW;                   // charging, and measured
    }
    g->setTextDatum(top_right);
    g->setTextColor(col);
    g->drawString(buf, W - 12, 6);
    g->setTextDatum(top_left);
}

// Status bar buffer sizes.
//
// STATS is 192 rather than 128 because the stats line formats eight uint32
// counters plus the build name and an offline marker: 43 bytes of literal
// text, up to 80 of digits, 15 for the build and 10 for " (offline)" - 149
// worst case. Truncation there would silently cut the line being read.
//
// SIG is *derived*, not chosen. It holds both lines, two separators, a %ld
// and the NUL. Widening STATS by hand while leaving the signature buffer at
// its old 300 is exactly what broke the build a moment ago, so the arithmetic
// is written down instead.
static const size_t STATUS_LINE_MAX  = 128;
static const size_t STATUS_STATS_MAX = 192;
// The place name joined the signature, so it has to be budgeted for here or
// the bar would stop noticing when it changes - snprintf would truncate the
// clock field off the end and the comparison would go stale.
// Four ranks joined with ", " - "Kerrytown, Ann Arbor, Michigan, United
// States" is 44 characters, and a long region name in a country that uses
// them can beat that comfortably.
static const size_t STATUS_PLACE_MAX = 112;
// Plus the navigation readout, which changes as the range does and so has to
// be in the signature or the bar would stop noticing that it moved.
static const size_t STATUS_NAV_MAX   = 64;
static const size_t STATUS_SIG_MAX   =
    STATUS_LINE_MAX + 1 + STATUS_STATS_MAX + 1 + STATUS_PLACE_MAX + 1 + 20 + 1
    + STATUS_NAV_MAX + 1;

static void drawStatus(const GnssFix &fix) {
    if (!g_panelOk) return;   // no display attached
    MapStats st; map_stats(&st);
    char buf[STATUS_STATS_MAX];
    const int W = M5.Display.width();

    // The map is clipped to leave these rows alone (see STATUS_H in
    // mapengine), so the bar can be repainted only when its text actually
    // changes. Both halves of that are needed: redrawing a filled rect every
    // frame is what caused the flash, and skipping the redraw without the
    // clip would simply let the map paint over it.
    static char last[STATUS_SIG_MAX] = "";
    static uint32_t lastDraw = 0;

    // Where we are, in words, ahead of where we are in numbers. This is the
    // one part of the bar that is worth reading at a glance while driving,
    // so it leads - and it stays put rather than appearing and vanishing,
    // which is why map_place_text() holds the last known name.
    char place[STATUS_PLACE_MAX];
    bool havePlace = map_place_text(place, sizeof place);

    // Where the target is, in words. Empty when nothing is being navigated to,
    // which is the common case, so it costs a byte in the signature and no
    // pixels on the bar.
    char nav[STATUS_NAV_MAX];
    wp_target_text(fix, nav, sizeof nav);

    bool have = (fix.status == 'A');
    char statusLine1[STATUS_LINE_MAX];
    if (g_estimated) {
        // Named for what it is. A coordinate with no qualifier reads as a fix,
        // and this one can be a hundred metres out and will not improve by
        // waiting - the user needs to know to distrust the marker rather than
        // to assume the receiver is still settling.
        snprintf(statusLine1, sizeof statusLine1,
                 "%.5f %.5f  z%u  WIFI ESTIMATE ~%.0f m from %d APs  no GNSS",
                 fix.lat, fix.lon, map_zoom(),
                 g_estimatedAcc, wifiloc_used());
    } else if (have) {
        // The trust note is appended rather than replacing anything: the
        // position is still the thing being read, and the checks are a
        // qualifier on it, not a substitute for it.
        const char *tn = gpstrust_text();
        snprintf(statusLine1, sizeof statusLine1,
                 "%.5f %.5f  z%u  %.0f km/h  %s HDOP %.1f%s%s",
                 fix.lat, fix.lon, map_zoom(), fix.speedKmh,
                 fix.mode == 3 ? "3D" : fix.mode == 2 ? "2D" : "--",
                 fix.hdop,
                 tn[0] ? "   CHECK: " : "", tn);
    } else if (!map_marker_valid()) {
        // The map is drawn from the last known position and there is nothing
        // on it. Without this the bar reads "acquiring" over a perfectly
        // detailed map of somewhere, which invites the map itself to be read
        // as the answer.
        snprintf(statusLine1, sizeof statusLine1,
                 "last known area - no position yet   sats %d  %lu sent",
                 fix.sats, (unsigned long)gnss_sentences());
    } else {
        snprintf(statusLine1, sizeof statusLine1,
                 "acquiring - open sky, 30-90s cold start   sats %d  %lu sent",
                 fix.sats, (unsigned long)gnss_sentences());
    }

    NetStats ns; netsource_stats(&ns);
    CacheStats cs; tilecache_stats(&cs);

    // Which counters get the space depends on where tiles are coming from.
    //
    // The bar used to show cache hits, net hits and the blob entry count
    // unconditionally. On a device with a planet archive on the card those
    // are all permanently zero - nothing is fetched, so nothing is cached -
    // while local_hits, the one counter actually moving, was tracked and
    // never displayed. Three zeros and no explanation reads as a broken
    // downloader rather than as the offline path working exactly as intended.
    //
    // So: local hits are shown whenever a local archive is open, and the
    // network-side counters are dropped once it is clear they are not the
    // story - no build adopted, nothing cached, nothing fetched.
    bool netpath = ns.build[0] || cs.entries || ns.net_hits || ns.cache_hits;
    if (ns.locals && !netpath) {
        snprintf(buf, sizeof buf,
                 "tiles %lu q%lu  local %lu  render %lums  blit %lu/%lums  "
                 "%u archive%s",
                 (unsigned long)st.rendered, (unsigned long)st.queue_depth,
                 (unsigned long)ns.local_hits,
                 (unsigned long)st.last_render_ms,
                 (unsigned long)st.last_draw_work_ms,
                 (unsigned long)st.peak_draw_work_ms,
                 (unsigned)ns.locals, ns.locals == 1 ? "" : "s");
    } else {
        snprintf(buf, sizeof buf,
                 "tiles %lu q%lu  c%lu n%lu l%lu  render %lums  blit %lu/%lums  "
                 "blob %lu  %s%s",
                 (unsigned long)st.rendered, (unsigned long)st.queue_depth,
                 (unsigned long)ns.cache_hits, (unsigned long)ns.net_hits,
                 (unsigned long)ns.local_hits,
                 (unsigned long)st.last_render_ms,
                 (unsigned long)st.last_draw_work_ms,
                 (unsigned long)st.peak_draw_work_ms,
                 (unsigned long)cs.entries,
                 ns.build[0] ? ns.build : "none", ns.online ? "" : " (offline)");
    }
    if (map_prefetch_busy()) {
        char pf[64];
        snprintf(pf, sizeof pf, "  PREFETCH %d%%", map_prefetch_progress());
        strncat(buf, pf, sizeof buf - strlen(buf) - 1);
    }

    // Include the minute in the change test, or the clock would sit stale
    // until something else on the bar happened to change.
    time_t nowt = time(nullptr);
    char combined[STATUS_SIG_MAX];
    snprintf(combined, sizeof combined, "%s|%s|%s|%s|%ld",
             statusLine1, buf, place, nav, (long)(nowt / 60));
    if (strcmp(combined, last) == 0 && millis() - lastDraw < 2000) return;
    // snprintf, for the reason given at the lastSig copy in drawPinPanel().
    snprintf(last, sizeof last, "%s", combined);
    lastDraw = millis();

    // Amber for an estimate: neither the green of a fix nor the red of
    // nothing, because it is neither. The bar colour is the part of this read
    // at a glance, so it has to differ - a green bar over a hundred-metre
    // guess is the one outcome worth avoiding here.
    // Trust outranks the fix colour. A green bar over a position that failed
    // two independent consistency checks is the single most misleading thing
    // this display could do, so the verdict takes the background and the fix
    // state takes what is left.
    TrustLevel tl = gpstrust_level();
    const uint16_t barBg =
          (have && tl == TRUST_BAD) ? M5.Display.color565(120, 20, 20)
        : (have && tl == TRUST_ODD) ? M5.Display.color565(90, 60, 0)
        : g_estimated               ? M5.Display.color565(90, 60, 0)
        : have ? (map_is_dark() ? M5.Display.color565(10, 40, 20) : TFT_DARKGREEN)
               : 0x6000;

    // Compose the whole bar, then push it in one write. Drawing the fill and
    // the text straight to the panel makes the bare fill briefly visible,
    // which is the blink.
    lgfx::LovyanGFX *g = g_statusCvOk ? (lgfx::LovyanGFX *)&g_statusCv
                                      : (lgfx::LovyanGFX *)&M5.Display;

    if (g_statusCvOk) g_statusCv.fillSprite(barBg);
    else              M5.Display.fillRect(0, 0, W, UI_STATUS_H, barBg);

    g->setTextDatum(top_left);
    int x1 = 12;
    if (havePlace) {
        // Brighter and drawn first, with the numbers flowing after it on the
        // same row. A separate row would need the bar taller, and STATUS_H
        // is shared with mapengine's clip - changing it means changing both.
        g->setTextSize(2);
        g->setTextColor(TFT_YELLOW);
        g->drawString(place, x1, 6);
        x1 += (int)g->textWidth(place) + 18;
    }
    g->setTextSize(2);
    g->setTextColor(TFT_WHITE);
    g->drawString(statusLine1, x1, 6);
    g->setTextColor(TFT_LIGHTGREY);
    int x2 = 12;
    if (nav[0]) {
        // Same colour as the pin and its guide line, so the three read as one
        // thing rather than three unrelated orange elements.
        g->setTextColor(M5.Display.color565(255, 130, 80));
        g->drawString(nav, x2, 28);
        x2 += (int)g->textWidth(nav) + 18;
        g->setTextColor(TFT_LIGHTGREY);
    }
    g->drawString(buf, x2, 28);
    drawClockBattery(fix, g);

    // Stale-link warning: the module going quiet looks identical to "no fix"
    // unless it is called out separately.
    if (millis() - fix.lastSentence > 3000) {
        g->setTextDatum(top_right);
        g->setTextColor(TFT_RED);
        g->drawString("NO GNSS", W - 12, 30);
        g->setTextDatum(top_left);
    } else if (antennaSuspect(fix)) {
        // The module is talking but has never reported a single satellite,
        // which is what a missing antenna looks like from here. Phrased as a
        // question because it is inference, not measurement - see
        // antennaSuspect().
        g->setTextDatum(top_right);
        g->setTextColor(TFT_ORANGE);
        g->drawString("CHECK ANTENNA?", W - 12, 30);
        g->setTextDatum(top_left);
    }

    if (g_statusCvOk) g_statusCv.pushSprite(0, 0);
}

// Survives a soft reset, so the retry below cannot become a boot loop.
RTC_NOINIT_ATTR static uint32_t s_panelMagic;
RTC_NOINIT_ATTR static uint32_t s_panelRetries;

// Bring the panel up, and notice when it has not.
//
// After a full flash write the panel sometimes fails to initialise: PSRAM
// reads back as entirely free because the framebuffer was never allocated
// (1800 KB for 1280x720 at 16bpp - the exact gap between a good boot and a
// dark one), width() and height() return 0, and every later draw silently
// goes nowhere. The board looks like it has a backlight problem; in fact
// nothing has been initialised to light up.
//
// It comes up on the next flash because that upload writes no sectors - it
// only verifies - so the reset follows an idle bus rather than six seconds of
// sustained flash writing. Giving it a moment and asking again is enough in
// most cases, and a single restart covers the rest.
// Is there a panel at all?
//
// This used to be answered "no" on every IDF boot, for a reason that turned out
// to have nothing to do with the panel: M5GFX was compiled without ARDUINO and
// this file with it, so the inline M5Unified::begin() read a garbage _board and
// returned before initialising anything. CMakeLists.txt fixes that at the
// component level; DISPLAY_IDF_NOTES.md has the full account.
//
// The check stays, and so do the g_panelOk guards below. They are not
// scaffolding for that one bug - a panel can also fail to come up after a
// sustained flash write, on either build, and every M5.Display call in this
// project dereferences a pointer that is null when it does.
//
// width() is not safe to call to find out: LGFXBase::width() is
// `return _panel->width()`, so with no panel attached it dereferences null and
// panics rather than returning 0. That is exactly what happened on the first
// IDF boot - a Load access fault inside IPanel::width with this=0x0, before
// any of this function's own diagnostics could run.
//
// getPanel() just hands back the pointer, so it can be tested first.
static bool panelUp() {
    return M5.Display.getPanel() != nullptr &&
           M5.Display.width() > 0 && M5.Display.height() > 0;
}

static bool panelBegin() {
    if (panelUp()) { g_panelOk = true; return true; }

    // There is no useful in-process retry. This used to call
    // M5.Display.init() again and report it as a retry; that call cannot do
    // anything, and saying so in the log was worse than silence.
    //
    // M5GFX::init_impl opens with:
    //
    //     if (getBoard() != board_t::board_unknown) { return true; }
    //
    // and _board is 22 by this point even on the boots where the panel was
    // never found - so init() returns true immediately, having re-probed
    // nothing. The log line "panel attached but not up, retrying init"
    // followed by a failure with no second M5GFX error in between is exactly
    // that: the retry never reached any hardware.
    //
    // M5GFX has its own retry, and it does not fire here either:
    //
    //     int retry = 4;
    //     do {
    //       if (retry == 1) use_reset = true;
    //       board = autodetect(use_reset, board);
    //     } while (board_t::board_unknown == board && --retry >= 0);
    //
    // It loops only while the board is unknown. The Tab5 branch of autodetect
    // sets board = board_M5Tab5 as soon as it sees the two I/O expanders on
    // I2C, which is *before* it probes the panel; when the panel probe then
    // fails it takes `goto init_clear` and returns 22 anyway. So the loop sees
    // a known board and stops after one attempt. The four retries M5GFX
    // provides are unreachable for this exact failure.
    //
    // Nor is re-running it by hand attractive: autodetect already did
    // `_bus_last.reset(bus_dsi)` and `bus_dsi->init()` succeeded, so a second
    // attempt re-enters a half-initialised DSI stack.
    //
    // What is left is a restart, which is cheap and - unlike everything above
    // - is known to work: the same binary fails to find the panel on the boot
    // after a flash write and finds it on the next one.

    // src: chosen, arbitrary. RTC_NOINIT memory is undefined after a power-on
    //      reset, so the counter beside it is only trustworthy when a value
    //      this unlikely is sitting next to it.
    if (s_panelMagic != 0x5AB5D157u) {          // first boot, counter is junk
        s_panelMagic = 0x5AB5D157u;
        s_panelRetries = 0;
    }

    // One restart, then give up and run headless.
    //
    // The bound matters more than the retry does. A panel that is genuinely
    // absent or broken must not turn a device that logs GPS and serves the
    // portal into a boot loop, and s_panelRetries is cleared once setup()
    // reaches a good boot, so a single bad boot months later still gets its
    // one restart.
    //
    // This is deliberately not what the code did before. The old comment here
    // read "restarting cannot help when M5.begin() will decide the same thing
    // next boot", which was true of the ARDUINO layout mismatch - that failed
    // identically every time - and is false of what is left. Detection is
    // flaky, not broken.
    if (s_panelRetries < 1) {
        s_panelRetries++;
        Serial.println("display: panel not detected - restarting once");
        Serial.println("display: M5GFX found the board but not the panel; "
                       "detection is flaky on the boot after a flash write");
        Serial.flush();
        delay(150);
        ESP.restart();
    }

    // Ask for the null panel. Harmless, but do NOT rely on it.
    //
    // M5GFX implements setPanel as
    //     static Panel_NULL nullobj;
    //     _panel = (nullptr == panel) ? &nullobj : panel;
    // which should make every later call a safe no-op. It was previously
    // recorded here that the managed_components copy "does not behave that
    // way", because with this line in place _panel still read as null and
    // setup() faulted on _panel->height(). That was a wrong conclusion about
    // the library and a third symptom of the ARDUINO layout mismatch:
    // setPanel() is out of line and wrote _panel at the component's offset,
    // while getPanel() is inline and read it at this file's offset.
    //
    // The g_panelOk guards throughout this file are load-bearing regardless.
    // Every function that touches M5.Display checks it.
    M5.Display.setPanel(nullptr);

    Serial.println("display: panel still not detected after a restart - "
                   "continuing headless");
    Serial.println("display: M5GFX logs the reason under its own tag at ERROR - "
                   "'M5Tab5 display panel was not detected' means the ST touch "
                   "firmware read and both DSI ID reads came back empty");
    Serial.println("display: everything except the screen still runs; check the "
                   "boot log below for SD, wifi and GNSS");
    return false;
}

#if MAP_M5_SMOKE_TEST
// Nothing but M5.begin(). See MAP_M5_SMOKE_TEST in features.h.
void setup() {
    delay(150);
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    delay(300);
    Serial.printf("\nsmoke: M5.getBoard()=%d  M5.Display.getBoard()=%d "
                  "(board_M5Tab5 is 22, board_unknown is 0)\n",
                  (int)M5.getBoard(), (int)M5.Display.getBoard());
    Serial.printf("smoke: panel=%p  %ldx%ld\n",
                  (void *)M5.Display.getPanel(),
                  M5.Display.getPanel() ? (long)M5.Display.width()  : 0L,
                  M5.Display.getPanel() ? (long)M5.Display.height() : 0L);
    Serial.printf("smoke: PSRAM free %u KB (a 1280x720x16 buffer is ~1800 KB)\n",
                  (unsigned)(ESP.getFreePsram() / 1024));
    if (M5.Display.getPanel() && M5.Display.width() > 0) {
        M5.Display.fillScreen(TFT_BLUE);
        Serial.println("smoke: screen filled blue - look at the panel");
    }
}
void loop() { delay(1000); }
#else

void setup() {
    // A moment before touching I2C. M5.begin() brings up the panel over the
    // internal bus, and the boot that fails is the one immediately following
    // a sustained flash write.
    delay(150);

    // Nothing between the delay above and M5.begin(). Measured, not assumed.
    //
    // Three lines were once inserted here - esp_log_level_set() for the M5GFX
    // and lgfx tags, and Serial.begin(). With them present the Arduino build
    // stopped bringing up the display: board detection still reported 22
    // (board_M5Tab5), but no framebuffer was allocated. PSRAM free went from
    // 30963 KB to 32764 KB, and the 1.8 MB difference is exactly a
    // 1280x720x16 buffer. Removing all three restored it.
    //
    // Which of the three did it was never isolated - they went in together and
    // came out together. If something ever needs to run before M5.begin(),
    // reflash the Arduino build and check that line before assuming it is
    // harmless.
    auto cfg = M5.config();
    M5.begin(cfg);

    // Serial immediately after M5.begin(), never before it.
    //
    // Before: the panel does not come up. After: everything from here on is
    // visible, including the board probe below and panelBegin()'s explanation
    // if the display is missing. The earlier arrangement had this above
    // M5.begin() and cost the Arduino build its display.
    Serial.begin(115200);

    // What did the libraries decide this board is? Printed on every build,
    // working or not, because the Arduino build is the control: it detects the
    // Tab5 correctly on the same hardware, so its numbers say what success
    // looks like and the IDF build's say how it differs.
    //
    // From m5gfx/boards.hpp: board_unknown = 0, board_M5Tab5 = 22, and the
    // enum runs 0..201 plus 512.
    const int boardM5   = (int)M5.getBoard();
    const int boardGfx  = (int)M5.Display.getBoard();
    Serial.printf("display: M5.getBoard()=%d  M5.Display.getBoard()=%d "
                  "(board_M5Tab5 is 22, board_unknown is 0)\n",
                  boardM5, boardGfx);

    // These two fields are the cheapest detector for a layout mismatch between
    // this translation unit and the M5GFX/M5Unified components, which is what
    // kept the IDF build headless for a long time. See DISPLAY_IDF_NOTES.md.
    //
    // A value outside board_t cannot be the result of detection running and
    // failing - that gives 0. It means the read landed at the wrong offset,
    // because LGFXBase gains a Print base class when ARDUINO is defined and
    // only one side of the build had it. M5Unified::begin() is inline, so it
    // is compiled here, and its first line gives up when _board reads non-zero:
    //
    //     if (_board != m5gfx::board_t::board_unknown) { return; }
    //
    // so nothing initialises and there is no error anywhere to find.
    // src: M5GFX lgfx/boards.hpp, enum board_t - the values run 0..201
    //      contiguously, plus board_M5AtomS3R at 512. Anything outside that is
    //      not a board id at all.
    auto plausible = [](int b) { return b == 0 || (b >= 1 && b <= 201) || b == 512; };
    if (!plausible(boardM5) || !plausible(boardGfx)) {
        Serial.println("display: board id is not a value board_t can hold - the "
                       "M5 object is being read at the wrong offset");
        Serial.println("display: M5GFX/M5Unified were compiled without ARDUINO "
                       "while this file was compiled with it");
        Serial.println("display: CMakeLists.txt patches both components to "
                       "require arduino-esp32 - check that patch applied "
                       "(look for 'patched to compile with ARDUINO' at configure "
                       "time, and for 'tab5_map: ARDUINO' in "
                       "managed_components/m5stack__m5gfx/CMakeLists.txt)");
    }

    // Seed the system clock from the RTC before anything asks the time.
    //
    // Nothing did this before, so time(nullptr) was zero until SNTP answered -
    // which is after wifi, and never at all offline. sunIsUp() bails out on a
    // clock below its sanity threshold, so the palette could not be decided
    // early even in principle. SNTP overwrites this later when it arrives, and
    // writes the corrected value back to the RTC (see loop()).
    if (M5.Rtc.isEnabled()) M5.Rtc.setSystemTimeFromRtc();

    powerReport();

    bool panel = panelBegin();

    // Clear the restart budget only when the panel is actually up.
    //
    // This used to clear it on any boot that got this far, headless included,
    // which handed a device with no panel a fresh restart every boot - the
    // boot loop the bound exists to prevent. panelBegin() only returns false
    // after it has already spent the budget.
    if (panel && s_panelMagic == 0x5AB5D157u) s_panelRetries = 0;

    // Tell the map engine too - it has its own M5.Display calls, and
    // view_follow() faulted on the first GPS fix before this existed.
    map_set_headless(!panel);

    if (panel) {
        M5.Display.setRotation(3);             // landscape, as in the GNSS sketch
        M5.Display.setBrightness(BRIGHT_DAY);
        M5.Display.fillScreen(TFT_BLACK);
    }

    M5.Power.setExtOutput(true);               // powers the M135
    // TTFF is measured from here, not from gnss_start(). This is when the
    // receiver gets power and begins searching; everything between - SD, wifi
    // association, pushing assistance - happens while it is already looking.
    // Timing from gnss_start() would quietly subtract all of that and report a
    // number several seconds better than the device actually delivers.
    g_gnssStartMs = millis();
    delay(500);

    // No Wire.begin() anywhere: M5.begin() already configured the internal
    // I2C bus, and reconfiguring it breaks touch, the RTC and the IMU.

    // Which reset this was, so a dark-screen boot can be told apart from a
    // sketch fault. POWERON is a real power cycle; SW / USB / JTAG mean the
    // peripherals outside the P4 did not restart with it.
    const char *rr = "?";
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  rr = "power-on";      break;
        case ESP_RST_SW:       rr = "software";      break;
        case ESP_RST_PANIC:    rr = "panic";         break;
        case ESP_RST_INT_WDT:  rr = "int watchdog";  break;
        case ESP_RST_TASK_WDT: rr = "task watchdog"; break;
        case ESP_RST_WDT:      rr = "other watchdog";break;
        case ESP_RST_DEEPSLEEP:rr = "deep sleep";    break;
        case ESP_RST_BROWNOUT: rr = "brownout";      break;
        case ESP_RST_EXT:      rr = "external pin";  break;
        default:               rr = "unknown";       break;
    }

    // Panel state alongside the reset reason. The reset reason turned out not
    // to distinguish a dark boot from a good one - both report the same - so
    // the useful signal is the panel geometry, which is 0x0 exactly when the
    // framebuffer was never allocated.
    // The version goes on the banner, not just in IDF's app_init block.
    //
    // Both are printed at boot, but only one of them survives being pasted
    // into a bug report: the app_init lines sit above the panel-detect
    // restart, so a log that starts at the banner - which is where anyone
    // reading it starts - carries no build identity at all. Two logs from
    // adjacent builds then look identical, and a timing number cannot be
    // attributed to the change it was meant to measure.
    //
    // esp_app_get_description() reads the header the bootloader already
    // validated, so this is a struct field rather than any extra work, and it
    // reports the same "<git-describe>-dirty" string IDF prints. The Arduino
    // build has the same call through the IDF component underneath it.
    const esp_app_desc_t *app = esp_app_get_description();
    Serial.printf("\n=== Tab5 map (%s) %s built %s ===\nPSRAM %u KB free\n"
                  "reset: %s, panel %ldx%ld %s\n",
                  map_build_flavour(), app->version, app->date,
                  (unsigned)(ESP.getFreePsram() / 1024), rr,
                  // LovyanGFX returns int32_t, which is 'long' on RISC-V -
                  // and width() dereferences the panel, so with none attached
                  // it must not be called at all.
                  panel ? (long)M5.Display.width() : 0L,
                  panel ? (long)M5.Display.height() : 0L,
                  panel ? "ok" : "DOWN");

    // After setRotation, so the canvas width matches the panel's, and before
    // anything draws the status bar or the buttons.
    uiCanvasesBegin();

    bootBegin();

    const char *bus = "none";
    bootStepBusy("mounting storage");

    // Bring the USB port up first. It costs three I2C writes and returns
    // immediately - the drive enumerates on the driver's own task - so doing
    // it here means the SD attempt below runs while a flash drive is coming
    // up, rather than after it.
    storage_usb_begin();

    if (!mountSD(&bus)) {
        // Not a dead end any more.
        //
        // This used to print "insert a card and restart" and stop. The restart
        // was never the point - the map data, the tile cache and the wifi
        // credentials all live on removable media, so there is nothing to do
        // until some arrives, but there is also no reason the device cannot
        // notice it arriving. waitForStorage() polls the slot and the USB-A
        // port until one of them has something on it.
        //
        // The distinction the old code drew is kept, because it still matters:
        // an empty slot is the ordinary case and formatting is only worth
        // offering for a card that is present and unreadable.
        bool unreadable = storage_card_present();
        SD_MMC.end();
        sdmmc_host_deinit();               // leave the slot clean for the poll
        bootStepFail(unreadable ? "storage - card unreadable"
                                : "storage - nothing inserted");
        if (!waitForStorage(&bus)) {
            g_bootActive = false;
            return;
        }
        bootBegin();                       // the wait wrote over the boot list
        bootStep("storage found");
    }
    {
        storage_rescan();                  // settle the shared choice once mounted
        char m[64];
        snprintf(m, sizeof m, "%s via %s", storage_name(), bus);
        bootStep(m);
    }

    // WiFi is optional: the map runs entirely offline from the local archive.
    // Setup is only forced when asked for, or when there is nothing stored.
    // The touch window stays here, at the front: it is the user's two seconds
    // and it must not move behind thirty seconds of other work.
    wifistore_diag();

    bootStepBusy("touch now to force wifi setup");
    bool forced = wantsSetup();

    // GNSS first, and at high priority: the FIFO overflows if the drain is
    // starved, and the renderer will happily saturate its core.
    if (!gnss_start(PIN_GNSS_TX, PIN_GNSS_RX, GNSS_BAUD, PIN_PPS, 0, 5))
        Serial.println("gnss task failed to start");

    // Assistance, in this order deliberately.
    //
    // AOP first, because ack-aiding is set in the same message and the
    // database push wants it. Then the stored database, before the receiver
    // has got far into searching - assistance is worth most when it arrives
    // ahead of the work it replaces.
    //
    // Both are best-effort: a receiver that does not answer leaves this
    // exactly where it started, a normal cold start.
    delay(200);                       // let the module finish talking after reset
    // The history the consistency checks compare against is meaningless
    // across a receiver restart: the first fix after one can legitimately be
    // anywhere the device has moved since.
    gpstrust_reset();
    gnss_enable_aop();
    aopRestore();

    bootStepBusy("opening map data");
    if (!map_begin(PMT_PATH, Z_CLOSE, 1, 1)) {
        bootStepFail("map data - init failed");
        Serial.println("map: init failed - see errors above");
        if (g_panelOk) {
            M5.Display.setTextColor(TFT_RED);
            M5.Display.setTextSize(2);
            M5.Display.drawString("See serial output", 40, M5.Display.height() - 60);
        }
        g_bootActive = false;
        return;
    }
    {
        char m[64];
        snprintf(m, sizeof m, "map ready, %dpx tiles, z%d+", SUBTILE_PX, Z_FLOOR);
        bootStep(m);
    }

    // Prove the archive end to end before the radio is touched.
    //
    // map_begin() above only proves the header parsed - the file could still
    // be truncated, the directory could be unreadable, the tiles could be
    // something other than gzipped MVT. Rendering z0/0/0 exercises all of it
    // against the one tile any complete archive must contain, and it is the
    // cheapest tile in the file.
    //
    // The point of doing it *here* is what it rules out. A device that draws
    // the world knows its card is good, so anything that goes wrong later is
    // the network or the sky. A device that does not knows the opposite,
    // without having spent forty seconds on a radio to find out.
    {
        Serial.printf("dbg: archive check starting at %lu ms uptime\n",
                      (unsigned long)millis());
        bootStepBusy("checking the map archive");
        map_world_check_start();
        uint32_t t0 = millis();
        while (map_world_check_state() == TILE_PENDING && millis() - t0 < 15000)
            delay(50);

        tile_state_t w = map_world_check_state();
        if (w == TILE_READY) {
            char m[64];
            snprintf(m, sizeof m, "archive readable (world in %lu ms)",
                     (unsigned long)(millis() - t0));
            bootStep(m);
            // Centred on the boot screen, above the list, at a size that
            // reads as a globe rather than a thumbnail.
            if (g_panelOk) {
                int side = M5.Display.height() / 3;
                map_world_check_draw(M5.Display.width() / 2,
                                     M5.Display.height() / 3, side);
            }
        } else if (w == TILE_NODATA) {
            bootStepFail("archive has no z0 tile - is it complete?");
        } else if (w == TILE_PENDING) {
            bootStepFail("archive check timed out - card may be slow");
        } else {
            bootStepFail("archive unreadable - check the card");
        }
        // Whatever the answer, carry on. A missing z0 does not stop the tiles
        // the user actually travels over from drawing, and refusing to boot
        // on it would turn a diagnostic into an outage.
    }
    // The magnetometer used to be brought up here. It now waits until after
    // the first picture - see sensorsLateBegin() below the boot wait.

    // The saved-point list, off the same card. Read here rather than lazily:
    // the first map_draw asks for it, and that runs on the UI task while the
    // render worker already has the archive open.
    wp_begin();
    {
        char m[40];
        snprintf(m, sizeof m, "waypoints: %d saved", wp_count());
        bootStep(m);
    }

    // Decide day or night before bootEnd() paints anything in it. After
    // map_begin, because map_set_dark() reaches into the tile grid.
    themeBoot();

    Serial.printf("dbg: sensors and theme done at %lu ms uptime\n",
                  (unsigned long)millis());

    // Draw the remembered area while the receiver searches. The palette is
    // already decided by this point, so the map comes up in the right colours
    // rather than being painted twice - and map_seed_position() has to run
    // after map_begin() in any case, since it centres the grid.
    //
    // No marker: see map_seed_position(). What is on screen is "here is where
    // you were", not "here is where you are".
    {
        double slat, slon;
        if (lastFixLoad(&slat, &slon)) map_seed_position(slat, slon);
    }

    // Wait for the map to appear before touching the radio.
    //
    // This is the whole point of the ordering above. Bringing the C6 up costs
    // about five seconds and a scan another three, and joining a network
    // another eleven; none of it is needed to draw a map that is already on
    // the card. Doing it first meant the screen sat on a boot list for twenty
    // seconds while the one thing the device exists to show was waiting behind
    // a network it does not require.
    //
    // Bounded, and two ways. A device with no remembered position has nothing
    // to render, so there is nothing to wait for - map_has_anchor() says so
    // and this returns at once. And a first render is slow: the overview alone
    // has been measured at over five seconds on this part, so the deadline is
    // generous rather than tight, and expiring it is not a failure. Either way
    // the radio work below still happens.
    if (map_has_anchor()) {
        bootStep("drawing the last known area");

        // The boot list has served its purpose; hand the screen over now so
        // the map appears as it is rendered rather than after the wait.
        bootEnd();

        uint32_t t0 = millis();
        while (!map_has_picture() && millis() - t0 < 12000) {
            M5.update();
            GnssFix f; gnss_get(&f);
            map_draw(f);
            drawStatus(f);
            delay(50);
        }
        // One pass after the picture lands, so the first frame is on the
        // panel before the radio work below starts competing for the bus.
        {
            GnssFix f; gnss_get(&f);
            map_draw(f);
            drawStatus(f);
        }
        Serial.printf("boot: first picture after %lu ms%s (uptime %lu ms)\n",
                      (unsigned long)(millis() - t0),
                      map_has_picture() ? "" : " (gave up waiting)",
                      (unsigned long)millis());
    }

    // ---- sensors, now that there is a map on the panel --------------------
    //
    // The magnetometer bring-up is not cheap: compass_begin() pushes 8 KB of
    // BMI270 configuration over I2C and then reads back an internal status,
    // and calLoad() opens the card for the stored calibration. On this board
    // that lands between "archive readable" and "drawing the last known
    // area", which is exactly the wrong place - it delays the first picture
    // by however long it takes, and nothing it produces is needed to draw
    // one. There is no heading on screen, and the log it feeds is gated on
    // walking pace, so its first useful sample is a drive away regardless.
    //
    // Moving it here costs nothing it was buying. The card is still mounted,
    // so a device that has been calibrated before still has its calibration
    // in hand before any heading could be drawn. The ordering trap that put
    // it after M5.begin() in the first place still holds - EXT_5V lives on
    // the expander at 0x43 and Power_Class::begin() rewrites that expander's
    // direction register - and this is later still, so it is no less safe.
    //
    // It stays ahead of the radio block below on purpose. The C6 brings up
    // SDIO on the same SDMMC host the card uses, and an 8 KB I2C upload is
    // better finished before that starts than interleaved with it.
    if (compass_begin()) {
        calLoad();
        Serial.println("compass: ready");
    } else if (compass_supported()) {
        Serial.println("compass: not available (see log above)");
    }

    // After compass_begin() so the button label can say "no mag" rather than
    // "no card" when the sensor is the thing that is missing.
    maglog_begin();



    // ---- radio, now that the map is up ------------------------------------
    // Everything from here needs the C6, and none of it is needed to draw.
    // DEBUG: bracket every step from the end of the boot wait to the first
    // line esp_hosted prints, because eleven seconds go missing in there and
    // nothing currently logs between them. Each stamp is relative to the
    // previous one, so the gap names itself.
    uint32_t dbg = millis();
    Serial.printf("dbg: boot wait done at %lu ms uptime\n", (unsigned long)dbg);

    bootStepBusy("starting wifi radio");
    Serial.printf("dbg: +%lu ms bootStepBusy drew\n",
                  (unsigned long)(millis() - dbg)); dbg = millis();

    // The renderer and SDIO enumeration both want the buses, and only one of
    // them is on a deadline. Measured: card init takes 47 ms with the worker
    // idle and 4779 ms with it running. The tiles can wait four seconds.
    map_worker_pause();
    Serial.printf("dbg: +%lu ms map_worker_pause returned\n",
                  (unsigned long)(millis() - dbg)); dbg = millis();

    bool radio = wifiRadioUp();
    Serial.printf("dbg: +%lu ms wifiRadioUp returned (%s)\n",
                  (unsigned long)(millis() - dbg), radio ? "up" : "failed");
    dbg = millis();

    map_worker_resume();
    if (radio) bootStep("wifi radio ready"); else bootStepFail("wifi radio unavailable");

    if (!radio) {
        Serial.println("wifi: radio unavailable, continuing offline");
    } else if (forced || !wifistore_exists()) {
        Serial.println(forced ? "wifi: setup forced by touch"
                              : "wifi: no stored credential, starting portal");
        if (g_panelOk) M5.Display.fillScreen(TFT_BLACK);
        if (!portal_run(300000))
            Serial.println("wifi: portal exited without saving");
        // The portal painted over whatever was there, including a map that
        // may already have been drawn above.
        if (g_panelOk) M5.Display.fillScreen(style_background());
        map_invalidate();
    } else {
        bootStepBusy("connecting to wifi");
        if (connectWifi(12000)) {
            char m[64];
            snprintf(m, sizeof m, "wifi %s", WiFi.localIP().toString().c_str());
            bootStep(m);
        } else {
            // A stored credential that does not connect is usually a network
            // that is out of range, not a wrong password - and this device is
            // built to work without one: the world floor is on the card and
            // the tile cache is thousands of tiles deep.
            //
            // Forcing the setup portal here made an offline boot impossible.
            // It took over the screen for five minutes to ask a question the
            // user had already answered, when the right behaviour was to carry
            // on and draw the map. The portal is still one button away, and
            // the "set up wifi" label appears on it whenever there is no
            // connection.
            //
            // A credential that is genuinely wrong still reaches the portal;
            // it just costs a deliberate tap rather than every boot out of
            // range.
            bootStepFail("wifi unreachable - continuing offline");
            Serial.println("wifi: stored network not reachable, staying offline "
                           "(cached tiles still draw; use the button to set up wifi)");
        }
    }

    // The Wi-Fi centroid database. After the radio attempt above, so a failed
    // C6 link is already logged and this does not look like the thing that
    // failed, and after the card so the database has somewhere to persist.
    wifiloc_begin();

    // Kick off the world floor if it has never been stored. It runs in the
    // background and survives being interrupted, so there is no reason to
    // hold up startup for it.
    if (netsource_world_local()) {
        bootStep("world floor not needed (local archive covers z0-6)");
    } else if (WiFi.status() == WL_CONNECTED && !netsource_world_ready()) {
        bootStep("storing world floor in background");
        map_world_floor_start();
    }

    bootStepBusy("waiting for GPS fix");

    bootEnd();
    // 30 s: far longer than any legitimate operation in loop(), so this only
    // fires on a genuine wedge. portal_run blocks for minutes by design and
    // resets the timer itself.
    esp_task_wdt_config_t wdt = { .timeout_ms = 30000,
                                  .idle_core_mask = 0,
                                  .trigger_panic = true };
    esp_task_wdt_reconfigure(&wdt);
    esp_task_wdt_add(nullptr);

    g_setupOk = true;
    Serial.println("running");
}

// Heartbeat and watchdog.
//
// A wedged loop() is invisible: the backlight stays wherever it was, the
// panel keeps its last contents, and touch stops responding - which looks
// identical to a crash, a sleep, and a black map. The heartbeat says which,
// and the watchdog turns a silent hang into a reboot with a backtrace
// instead of a device that has to be power-cycled blind.
static void loopHeartbeat() {
    static uint32_t last = 0;
    static uint32_t iters = 0;
    iters++;
    if (millis() - last < 30000) return;
    last = millis();

    // Free heap alone cannot tell a leak from ordinary churn, and it was the
    // free-heap number that looked fine right up until the AES DMA descriptor
    // allocation failed. Three figures separate the cases:
    //
    //   heap      falling and not recovering        -> something is retained
    //   min       the low-water mark since boot     -> worst moment so far
    //   dma       largest single DMA-capable block  -> what a handshake can get
    //
    // A TLS handshake needs a contiguous DMA block, so `dma` collapsing while
    // `heap` still looks healthy is fragmentation, and is just as fatal.
    Serial.printf("alive: loop %lu iters, uptime %lus, heap %u KB "
                  "(min %u KB, dma %u KB), psram %u KB, stack headroom %u B\n",
                  (unsigned long)iters, (unsigned long)(millis() / 1000),
                  (unsigned)(ESP.getFreeHeap() / 1024),
                  (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_DMA) / 1024),
                  (unsigned)(ESP.getFreePsram() / 1024),
                  (unsigned)uxTaskGetStackHighWaterMark(nullptr));
    iters = 0;
}

void loop() {
    // Nothing below is safe until setup() finished - see g_setupOk. Idle
    // visibly rather than faulting, so the reason stays on screen and on the
    // serial line instead of being replaced by a panic.
    if (!g_setupOk) {
        static uint32_t last = 0;
        if (millis() - last > 5000) {
            last = millis();
            Serial.println("halted: setup did not complete - see the error above");
        }
        delay(200);
        return;
    }

    esp_task_wdt_reset();
    loopHeartbeat();
    M5.update();

    GnssFix fix;
    gnss_get(&fix);

    batteryPoll(false);                // 1 Hz internally

    // The sensor polls wait for a picture; the rest of the pass does not.
    //
    // loop() begins the moment setup() returns, which on a cold start can be
    // ahead of the first picture - the boot wait gives up after 12 s whether
    // or not a tile has landed. Until then the renderer wants the PSRAM bus
    // and the UI task needs to keep compositing, and these polls compete for
    // both while producing nothing anyone can see.
    //
    // None of them loses anything by waiting. compass_update() feeds no
    // display, maglog_poll() writes nothing below walking pace, and
    // wifiloc_poll() needs a radio the boot sequence has not started yet.
    // This is a reordering, not a reduction: once there is a picture they run
    // every pass exactly as before.
    //
    // It has to be a gate around these three and NOT an early return from
    // loop(). map_draw() and handleTouch() are further down, and a return here
    // would stop the very compositing that makes map_has_picture() true - the
    // condition would then never clear and the device would sit dark.
    const bool painted = map_has_picture();

    if (painted) compass_update();     // 10 Hz internally, loop()'s task only

    // Written from the loop rather than from inside calibrate_finish: that
    // runs deep in compass_update and has no business touching the card, and
    // the flag makes the write happen once, on the next pass, on this task.
    if (painted && compass_cal_dirty()) { compass_cal_clear_dirty(); calSave(); }
    handleTouch(fix);

    // True north correction. The map is drawn in true north and the sensor
    // reports magnetic, and the difference is over ten degrees across much of
    // the continental US - enough that the needle would visibly disagree with
    // the streets under it. Recomputed only when the position has moved far
    // enough to matter.
    if (painted && gnss_coarse(fix)) compass_set_position(fix.lat, fix.lon);
    gnssRatePolicy(fix);
    ntpPolicy(fix);
    if (painted) {
        maglog_poll(fix);
        wifiloc_poll(fix);
    }
    // After wifiloc_poll, so the cross-check inside sees this second's
    // estimate rather than the previous one.
    gpstrust_update(fix);

    // What the map is shown, which is not always what the receiver said.
    //
    // `fix` stays exactly as the receiver reported it and is what the log, the
    // rate policy, the assistance maintenance and the last-position save all
    // continue to see. `view` is a copy that may carry a Wi-Fi estimate
    // instead, and it is what the engine and the UI are given.
    //
    // The estimate is dressed as a 2D fix on purpose. gnss_coarse() must pass
    // or the engine will not centre the grid at all; gnss_fine() must fail or
    // the fine zoom gate, the waypoint precision and lastFixSave would all
    // treat a hundred-metre guess as a survey point. mode 2 does both. The
    // spread is put in HDOP so the one number the rest of the code already
    // uses to mean "how much to trust this" carries something meaningful,
    // scaled to the same rough range a real HDOP occupies.
    GnssFix view = fix;
    g_estimated = false;
    if (!gnss_coarse(fix)) {
        double elat, elon; float eacc; uint32_t eage;
        if (wifiloc_position(&elat, &elon, &eacc, &eage)) {
            view.lat = elat;
            view.lon = elon;
            view.status = 'A';
            view.mode = 2;
            view.hdop = eacc / 20.0;      // ~100 m spread reads as HDOP 5
            if (view.hdop < 3.0) view.hdop = 3.0;
            // Speed and course belong to the receiver and it has not got any.
            // Leaving the previous values would drive the zoom hysteresis and
            // the rate policy from a heading nobody measured.
            view.speedKmh = 0;
            view.course = 0;
            g_estimated = true;
            g_estimatedAcc = eacc;
        }
    }
    handlePowerButton();
    flushIfIdle();
    applyTheme(view);

    // Remember where we are, for the next boot's palette. Ten minutes is
    // frequent enough that the position is never far wrong and rare enough
    // that it is not a write pattern - a sunrise time does not change fast
    // enough to justify anything tighter.
    {
        static uint32_t lastPosSave = 0;
        if (gnss_fine(fix) && (lastPosSave == 0 || millis() - lastPosSave > 600000)) {
            lastPosSave = millis();
            lastFixSave(fix);
        }
    }

    // Belt and braces: if anything at all lights the panel while the screen
    // is meant to be off, put it back. A lit screen showing nothing gives
    // the user no indication that a touch in the middle would fix it.
    if (g_screenOff && M5.Display.getBrightness() != 0)
        M5.Display.setBrightness(0);

    map_update(view);
    pickZoom(view);
    aopMaintain(fix);
    ttffReport(fix);

    // Build discovery needs a calendar date. Two independent sources, since
    // each can be unavailable: SNTP needs the network, GNSS needs sky. The
    // first one to arrive wins and the other is skipped.
    static bool dateSet = false;
    if (!dateSet) {
        if (fix.date[0] && fix.status == 'A') {
            netsource_set_date(fix.date);
            dateSet = true;
            Serial.printf("netsource: UTC date %s from GNSS\n", fix.date);
        } else {
            static uint32_t lastTry = 0;
            if (millis() - lastTry > 3000) {
                lastTry = millis();
                if (netsource_set_date_from_clock()) {
                    dateSet = true;
                    // Write the verified time back to the RTC, so the next
                    // cold boot starts from something sane instead of
                    // whatever the chip happened to be holding.
                    time_t now = time(nullptr);
                    struct tm t;
                    gmtime_r(&now, &t);
                    m5::rtc_datetime_t dt;
                    dt.date.year = t.tm_year + 1900;
                    dt.date.month = t.tm_mon + 1;
                    dt.date.date = t.tm_mday;
                    dt.date.weekDay = t.tm_wday;
                    dt.time.hours = t.tm_hour;
                    dt.time.minutes = t.tm_min;
                    dt.time.seconds = t.tm_sec;
                    M5.Rtc.setDateTime(dt);
                    Serial.println("rtc: updated from SNTP");
                }
            }
        }
    }

    // Periodic timing to serial. The status bar carries this too, but the
    // screen is not always the thing being watched - and blit cost against
    // render cost is the number that says whether compositing is stealing
    // time from the renderer.
    {
        static uint32_t lastStats = 0;
        if (millis() - lastStats > 15000) {
            lastStats = millis();
            MapStats st; map_stats(&st);
            NetStats ns; netsource_stats(&ns);
            CacheStats cs; tilecache_stats(&cs);
            Serial.printf("stats: rendered %lu (last %lu ms)  blit last %lu ms "
                          "max %lu avg %lu over %lu  q%lu  cache %lu/%lu net %lu  "
                          "coarse %lu/%lug%luw  psram %u KB\n",
                          (unsigned long)st.rendered, (unsigned long)st.last_render_ms,
                          (unsigned long)st.last_draw_work_ms,
                          (unsigned long)st.max_draw_work_ms,
                          (unsigned long)(st.draws ? st.draw_work_total_ms / st.draws : 0),
                          (unsigned long)st.draws, (unsigned long)st.queue_depth,
                          (unsigned long)ns.cache_hits, (unsigned long)cs.entries,
                          (unsigned long)ns.net_hits,
                          (unsigned long)st.coarse_renders,
                          (unsigned long)st.coarse_gap,
                          (unsigned long)st.coarse_wait,
                          (unsigned)(ESP.getFreePsram() / 1024));
            // Both lines cover the last interval only, not since boot - the
            // window is cleared below once they have been printed.
            //
            // Wall clock and preemption, on their own line.
            //
            // The blit figures above are work now, so this line carries what
            // they no longer do: wall clock, and how much of it was spent not
            // running. Read the two together - blit avg is what compositing
            // costs, stall max is how long the UI task has been frozen out by
            // something higher priority, and the second is a scheduling
            // problem that no amount of work on blit_region will touch.
            if (st.cpu_time_valid) {
                Serial.printf("draw:  wall last %lu ms max %lu avg %lu"
                              "   stall max %lu ms   over %lu"
                              "   (peak since boot: work %lu, stall %lu)\n",
                              (unsigned long)st.last_draw_wall_ms,
                              (unsigned long)st.max_draw_wall_ms,
                              (unsigned long)(st.draws ? st.draw_wall_total_ms / st.draws : 0),
                              (unsigned long)st.max_stall_ms,
                              (unsigned long)st.draws,
                              (unsigned long)st.peak_draw_work_ms,
                              (unsigned long)st.peak_stall_ms);
            } else {
                Serial.println("draw:  wall only - no run-time counter in this "
                               "build, so the blit figures above are wall clock");
            }

            // Start the next window here, after both lines have been printed
            // from the same snapshot. Resetting before the second line would
            // have it describe a window the first line did not.
            map_stats_reset_window();

            // Separate line rather than more fields on that one: the compass
            // is the thing most likely to need watching over time (a drifting
            // |B| means the calibration has gone stale, or something magnetic
            // moved), and it is easier to grep for on its own.
            if (compass_ok()) {
                float h = compass_heading();
                // compass_status() already ends in the field magnitude, so this
            // does not repeat it.
            Serial.printf("compass: %s, %.0f deg %s  "
                              "log %s %lu rows %u KB  gnss %u ms  "
                              "wifi %lu APs\n",
                              compass_status(),
                              h < 0 ? 0.0f : h, compass_label(h),
                              !maglog_available() ? "unavailable"
                                                  : maglog_enabled() ? "on" : "off",
                              (unsigned long)maglog_rows(),
                              (unsigned)(maglog_bytes() / 1024),
                              (unsigned)gnss_rate_ms(),
                              (unsigned long)wifiloc_entries());
            }
        }
    }

    // ~15 fps. The map only changes when a tile commits or the marker moves,
    // neither of which happens at frame rate.
    static uint32_t last = 0;
    uint32_t now = millis();
    if (g_panelOk && !g_screenOff && now - last >= 66) {
        last = now;
        // The panel covers the middle of the screen and the marker moves
        // underneath it. Skipping the map draw entirely while it is open is
        // simpler than clipping around it, and costs nothing: the panel is a
        // deliberate, short-lived interaction, and pinPanelClose() forces the
        // repaint on the way out.
        if (!g_pinPanel) map_draw(view);
        drawStatus(view);
        drawFooter(view);
        drawPinPanel(view);
    }

    // Loop cadence.
    //
    // 5 ms - about 200 Hz - is what a screen being composited at 15 fps wants:
    // it keeps touch latency below one frame and gives map_draw() a fresh
    // chance to notice a tile commit. With the screen off none of that is
    // being asked for. The panel is dark, map_set_visible(false) has already
    // stopped the renderer drawing, and drawStatus() and drawFooter() both
    // return at their first line - so the pass reduces to polling things that
    // poll themselves.
    //
    // It is not free even so. Every pass is an M5.update(), which is an I2C
    // transaction to the touch controller, plus a fix copied under a mutex and
    // a walk of the poll functions. At 200 Hz, for hours, with nothing on
    // screen, that is the second-largest thing the device does while asleep
    // after the receiver.
    //
    // 50 ms is chosen against the one thing that still has to work: the wake
    // tap. screenOff() draws a target in the middle ninth and inWakeZone()
    // wants a deliberate flat-handed press there, which lasts far longer than
    // a 20 Hz sample can miss - and the same 50 ms is still inside the 100 ms
    // gate compass_update() applies internally, so the magnetometer log keeps
    // its cadence rather than thinning out. Everything else in the pass -
    // batteryPoll at 1 Hz, gnssRatePolicy, ntpPolicy, maglog_poll,
    // wifiloc_poll - already rate-limits itself well below 20 Hz.
    //
    // What deliberately does not change: GNSS runs in its own task, the render
    // worker keeps fetching tile bytes into the card cache, and the watchdog
    // is fed at the top of every pass with 30 s of headroom against a 20 Hz
    // rate. Driving through a town with the display asleep still ends with the
    // corridor cached, which is the property screenOff() was built around.
    vTaskDelay(pdMS_TO_TICKS(g_screenOff ? 50 : 5));
}

#endif  // MAP_M5_SMOKE_TEST
