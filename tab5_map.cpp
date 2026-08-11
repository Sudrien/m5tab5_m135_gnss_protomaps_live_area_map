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
    wifiSetPins();
    if (!WiFi.mode(WIFI_STA)) { Serial.println("wifi: WiFi.mode(STA) failed"); return false; }
    delay(200);

    String mac = WiFi.macAddress();
    Serial.printf("wifi: station MAC %s\n", mac.c_str());
    if (mac == "00:00:00:00:00:00" || mac.length() == 0) {
        Serial.println("wifi: MAC is zero - ESP-Hosted link to the C6 is not up.");
        Serial.println("      Check the board selection (esp32:esp32:m5stack_tab5)");
        Serial.println("      and that the C6 still has its SDIO WiFi firmware.");
        return false;
    }
    int n = WiFi.scanNetworks();
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
        // Only the date matters, so no timezone is configured - everything
        // downstream works in UTC. The sync is asynchronous; netsource polls
        // for completion rather than blocking here.
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        Serial.println("wifi: SNTP requested");
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

    uint32_t pending = tilecache_pending();
    if (pending != lastPending) { lastPending = pending; lastActivity = millis(); }
    if (!pending) return;

    MapStats st; map_stats(&st);
    if (st.queue_depth) return;                  // still rendering, stay out of the way
    if (millis() - lastActivity < 2000) return;  // let a burst finish

    tilecache_flush();
    lastPending = 0;
}

// Cache warming: a visible button in the corner, with a confirmation step.
//
// Radius 7 is 15x15 tiles - about 13 km at z15, 27 km at z14 - which takes
// several minutes over HTTP. That is too much to start by accident, and an
// invisible whole-screen tap target would do exactly that every time the
// device was picked up.
static const int PREFETCH_RADIUS = 7;

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

static SunSet g_sun;
static bool   g_sunValid = false;
static double g_sunriseMin = 0, g_sunsetMin = 0;
static int    g_sunDay = -1;

// Backlight levels. Night is dim enough not to ruin dark adaptation but
// still readable; day is full, since sunlight is the harder problem.
// src: M5GFX LGFX_Device::setBrightness takes a uint8_t, so 255 is the top of
//      the range and not a chosen number.
// src: BRIGHT_NIGHT is a judgement, unattributed - see PROVENANCE.md.
static const uint8_t BRIGHT_DAY = 255, BRIGHT_NIGHT = 60;
static uint8_t g_brightness = BRIGHT_DAY;

// Split from sunIsUp() so the boot path can ask the same question about a
// remembered position, before any fix exists. See themeBoot().
static bool sunIsUpAt(double lat, double lon) {
    time_t now = time(nullptr);
    if (now < 1767225600) return true;                   // clock not set yet
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

static bool sunIsUp(const GnssFix &fix) {
    if (fix.status != 'A' || !fix.utc[0]) return true;   // assume day if unsure
    if (g_sunFromStored) { g_sunDay = -1; g_sunFromStored = false; }
    return sunIsUpAt(fix.lat, fix.lon);
}

static void applyTheme(const GnssFix &fix) {
    bool wantDark;
    switch (g_themeMode) {
        case THEME_DAY:   wantDark = false; break;
        case THEME_NIGHT: wantDark = true;  break;
        default:          wantDark = !sunIsUp(fix); break;
    }
    if (wantDark != map_is_dark()) map_set_dark(wantDark);

    // Track the level the screen *should* have, but only drive the backlight
    // when the screen is actually on.
    //
    // Without the guard, a day/night transition while asleep turned the
    // backlight back on without resuming drawing - a lit panel showing
    // nothing at all, no map, no status bar, no buttons. Indistinguishable
    // from a crash, and it survives until someone happens to touch the
    // middle of the screen.
    uint8_t want = wantDark ? BRIGHT_NIGHT : BRIGHT_DAY;
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
static bool     g_statusCvOk = false;
static bool     g_btnCvOk    = false;

// The touch target is taller than the drawn button. A 54 px outline is a
// small thing to hit on a moving vehicle, and there is nothing else along
// the bottom edge to steal a press from - so the target extends upward into
// the map and downward to the screen edge.
static const int BTN_PAD_TOP = 26, BTN_PAD_SIDE = 6;

enum { BTN_CACHE = 0, BTN_THEME, BTN_SLEEP, BTN_COUNT };

static uint32_t g_confirmUntil = 0;      // armed state for the cache button
static uint32_t g_lastTouchMs = 0;

static void buttonRect(int i, int *x, int *y, int *w, int *h) {
    if (!g_panelOk) { *x = *y = *w = *h = 0; return; }
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

static void drawFooter() {
    if (!g_panelOk) return;   // no display attached
    if (g_screenOff) return;

    bool armed = (millis() < g_confirmUntil);
    bool busy  = map_prefetch_busy();
    bool net   = (WiFi.status() == WL_CONNECTED);

    char label[40];
    if (busy)       snprintf(label, sizeof label, "caching %d%%", map_prefetch_progress());
    else if (armed) snprintf(label, sizeof label, "tap to confirm");
    else if (!net)  snprintf(label, sizeof label, "set up wifi");
    else            snprintf(label, sizeof label, "cache %d km",
                             (int)(((2 * PREFETCH_RADIUS + 1) * 40075.0
                                    * 0.74 / (1 << DATA_ZOOM_OF(Z_FLOOR)))));
    drawButton(BTN_CACHE, label,
               busy  ? TFT_DARKGREY :
               armed ? TFT_ORANGE   :
               net   ? M5.Display.color565(40, 70, 150)
                     : M5.Display.color565(70, 70, 70));

    // The theme button names the mode, and in auto also shows which way it
    // currently resolves - otherwise "auto" tells you nothing about why the
    // screen looks the way it does.
    const char *tl = g_themeMode == THEME_DAY   ? "day"
                   : g_themeMode == THEME_NIGHT ? "night"
                   : (map_is_dark() ? "auto - night" : "auto - day");
    drawButton(BTN_THEME, tl,
               g_themeMode == THEME_AUTO ? M5.Display.color565(60, 90, 60)
                                         : M5.Display.color565(70, 70, 90));

    drawButton(BTN_SLEEP, "screen off", M5.Display.color565(70, 70, 70));
}

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

    g_screenOff = true;
    map_set_visible(false);
    M5.Display.setBrightness(0);
    M5.Display.fillScreen(TFT_BLACK);
    // GNSS and the render worker are untouched: the position stays current
    // and tiles keep arriving, so waking is instant rather than a cold start.
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

static void handleTouch() {
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
    int b = buttonAt(t.x, t.y);
    if (b != BTN_CACHE) g_confirmUntil = 0;

    switch (b) {
    case BTN_SLEEP:
        screenOff();
        break;

    case BTN_THEME:
        g_themeMode = (ThemeMode)((g_themeMode + 1) % 3);
        Serial.printf("theme: %s\n",
                      g_themeMode == THEME_AUTO ? "auto" :
                      g_themeMode == THEME_DAY  ? "day" : "night");
        break;

    case BTN_CACHE:
        if (map_prefetch_busy()) break;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("wifi: opening setup portal from button");
            wifiSetPins();
            portal_run(300000);
            if (WiFi.status() == WL_CONNECTED)
                configTime(0, 0, "pool.ntp.org", "time.nist.gov");
            // The portal painted over everything. Without this the engine
            // sees an unchanged view, unchanged tiles and an unchanged
            // marker, skips the repaint, and leaves a lit but empty screen
            // until something else happens to move.
            M5.Display.fillScreen(style_background());
            map_invalidate();
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
    g_bootActive = false;
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
    if (SD_MMC.begin("/sdcard", false)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true))  { *bus = "SDMMC 1-bit"; return true; }
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

static bool mountSD(const char **bus, bool allow_format) {
    if (SD_MMC.begin("/sdcard", false)) { *bus = "SDMMC 4-bit"; return true; }
    if (SD_MMC.begin("/sdcard", true))  { *bus = "SDMMC 1-bit"; return true; }
    if (SD.begin())                     { *bus = "SPI";         return true; }

    if (!allow_format) return false;

    // Every read-only attempt has failed by now, so there is nothing here that
    // this build could have opened.
    Serial.println("sd: formatting card (FAT, all existing data lost)");
    if (SD_MMC.begin("/sdcard", false, true)) { *bus = "SDMMC 4-bit, formatted"; return true; }
    if (SD_MMC.begin("/sdcard", true,  true)) { *bus = "SDMMC 1-bit, formatted"; return true; }
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
        Serial.printf("gnss: first fix after %.1f s (%s), %d sats\n",
                      (millis() - t0) / 1000.0,
                      g_aopRestored
                        ? (g_aopAgeHours >= 0
                             ? "assisted" : "assisted, age unknown")
                        : "no assistance",
                      fix.sats);
    }
    if (anyDone && !fineDone && gnss_fine(fix)) {
        fineDone = true;
        Serial.printf("gnss: 3D fix hdop %.2f after %.1f s\n",
                      fix.hdop, (millis() - t0) / 1000.0);
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

    int pct = M5.Power.getBatteryLevel();
    bool charging = M5.Power.isCharging();

    if (haveTime && pct >= 0)
        snprintf(buf, sizeof buf, "%02d:%02dZ   %s%d%%",
                 lt.tm_hour, lt.tm_min, charging ? "+" : "", pct);
    else if (haveTime)
        snprintf(buf, sizeof buf, "%02d:%02dZ", lt.tm_hour, lt.tm_min);
    else if (pct >= 0)
        snprintf(buf, sizeof buf, "%s%d%%", charging ? "+" : "", pct);
    else
        return;

    // Colour follows the battery, since that is the part worth noticing at a
    // glance on a device you are carrying.
    uint16_t col = TFT_WHITE;
    if (!charging && pct >= 0) {
        if (pct <= 10)      col = TFT_RED;
        else if (pct <= 25) col = TFT_ORANGE;
    } else if (charging) {
        col = TFT_GREENYELLOW;
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
static const size_t STATUS_SIG_MAX   =
    STATUS_LINE_MAX + 1 + STATUS_STATS_MAX + 1 + 20 + 1;

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

    bool have = (fix.status == 'A');
    char statusLine1[STATUS_LINE_MAX];
    if (have) {
        snprintf(statusLine1, sizeof statusLine1,
                 "%.5f %.5f  z%u  %.0f km/h  %s HDOP %.1f",
                 fix.lat, fix.lon, map_zoom(), fix.speedKmh,
                 fix.mode == 3 ? "3D" : fix.mode == 2 ? "2D" : "--",
                 fix.hdop);
    } else {
        snprintf(statusLine1, sizeof statusLine1,
                 "acquiring - open sky, 30-90s cold start   sats %d  %lu sent",
                 fix.sats, (unsigned long)gnss_sentences());
    }

    NetStats ns; netsource_stats(&ns);
    CacheStats cs; tilecache_stats(&cs);
    snprintf(buf, sizeof buf,
             "tiles %lu q%lu  c%lu n%lu  render %lums  blit %lu/%lums  blob %lu  %s%s",
             (unsigned long)st.rendered, (unsigned long)st.queue_depth,
             (unsigned long)ns.cache_hits, (unsigned long)ns.net_hits,
             (unsigned long)st.last_render_ms,
             (unsigned long)st.last_draw_ms, (unsigned long)st.max_draw_ms,
             (unsigned long)cs.entries,
             ns.build[0] ? ns.build : "none", ns.online ? "" : " (offline)");
    if (map_prefetch_busy()) {
        char pf[64];
        snprintf(pf, sizeof pf, "  PREFETCH %d%%", map_prefetch_progress());
        strncat(buf, pf, sizeof buf - strlen(buf) - 1);
    }

    // Include the minute in the change test, or the clock would sit stale
    // until something else on the bar happened to change.
    time_t nowt = time(nullptr);
    char combined[STATUS_SIG_MAX];
    snprintf(combined, sizeof combined, "%s|%s|%ld",
             statusLine1, buf, (long)(nowt / 60));
    if (strcmp(combined, last) == 0 && millis() - lastDraw < 2000) return;
    strncpy(last, combined, sizeof last - 1);
    lastDraw = millis();

    const uint16_t barBg = have
        ? (map_is_dark() ? M5.Display.color565(10, 40, 20) : TFT_DARKGREEN)
        : 0x6000;

    // Compose the whole bar, then push it in one write. Drawing the fill and
    // the text straight to the panel makes the bare fill briefly visible,
    // which is the blink.
    lgfx::LovyanGFX *g = g_statusCvOk ? (lgfx::LovyanGFX *)&g_statusCv
                                      : (lgfx::LovyanGFX *)&M5.Display;

    if (g_statusCvOk) g_statusCv.fillSprite(barBg);
    else              M5.Display.fillRect(0, 0, W, UI_STATUS_H, barBg);

    g->setTextDatum(top_left);
    g->setTextSize(2);
    g->setTextColor(TFT_WHITE);
    g->drawString(statusLine1, 12, 6);
    g->setTextColor(TFT_LIGHTGREY);
    g->drawString(buf, 12, 28);
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
    Serial.printf("\n=== Tab5 map (%s) ===\nPSRAM %u KB free\n"
                  "reset: %s, panel %ldx%ld %s\n",
                  map_build_flavour(),
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
    wifistore_diag();

    bootStepBusy("touch now to force wifi setup");
    bool forced = wantsSetup();

    bootStepBusy("starting wifi radio");
    bool radio = wifiRadioUp();
    if (radio) bootStep("wifi radio ready"); else bootStepFail("wifi radio unavailable");

    if (!radio) {
        Serial.println("wifi: radio unavailable, continuing offline");
    } else if (forced || !wifistore_exists()) {
        Serial.println(forced ? "wifi: setup forced by touch"
                              : "wifi: no stored credential, starting portal");
        if (g_panelOk) M5.Display.fillScreen(TFT_BLACK);
        if (!portal_run(300000))
            Serial.println("wifi: portal exited without saving");
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
    // No fillScreen here. This used to clear the boot list into the map
    // background colour halfway through startup - a flat near-white field with
    // the remaining boot lines drawn onto black strips, held for the thirty
    // seconds it takes to get a fix, then flipped to the night palette in one
    // step. The boot screen stays black until bootEnd(), which now paints the
    // background once, in the colour themeBoot() has already chosen.

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
    // Kick off the world floor if it has never been stored. It runs in the
    // background and survives being interrupted, so there is no reason to
    // hold up startup for it.
    if (WiFi.status() == WL_CONNECTED && !netsource_world_ready()) {
        bootStep("storing world floor in background");
        map_world_floor_start();
    }

    // Decide day or night before bootEnd() paints anything in it. After
    // map_begin, because map_set_dark() reaches into the tile grid.
    themeBoot();

    bootStepBusy("waiting for GPS fix");
    delay(600);
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

    handleTouch();
    handlePowerButton();
    flushIfIdle();
    applyTheme(fix);

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

    map_update(fix);
    pickZoom(fix);
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
                          (unsigned long)st.last_draw_ms, (unsigned long)st.max_draw_ms,
                          (unsigned long)(st.draws ? st.draw_total_ms / st.draws : 0),
                          (unsigned long)st.draws, (unsigned long)st.queue_depth,
                          (unsigned long)ns.cache_hits, (unsigned long)cs.entries,
                          (unsigned long)ns.net_hits,
                          (unsigned long)st.coarse_renders,
                          (unsigned long)st.coarse_gap,
                          (unsigned long)st.coarse_wait,
                          (unsigned)(ESP.getFreePsram() / 1024));
        }
    }

    // ~15 fps. The map only changes when a tile commits or the marker moves,
    // neither of which happens at frame rate.
    static uint32_t last = 0;
    uint32_t now = millis();
    if (g_panelOk && !g_screenOff && now - last >= 66) {
        last = now;
        map_draw(fix);
        drawStatus(fix);
        drawFooter();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
}

#endif  // MAP_M5_SMOKE_TEST
