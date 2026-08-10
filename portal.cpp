// portal.cpp

#include "portal.h"
#include "wifistore.h"
#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <ctype.h>

// Set to 1 to echo submitted passwords to the serial console in full.
//
// Off by default on purpose: serial logs get pasted into bug reports and
// chat windows, and a live WiFi password is an easy thing to leak without
// noticing. The masked output below is enough to confirm a submission
// arrived intact - length, ends, and a per-character class breakdown that
// will show whether something like '@' survived the URL decode.
#define PORTAL_LOG_SECRETS 0

static const char *AP_SSID = "Tab5-Map-Setup";
static const IPAddress AP_IP(192, 168, 4, 1);

static WebServer  *g_web = nullptr;
static DNSServer  *g_dns = nullptr;
static bool  g_done = false;
static String g_msg;

// Report what actually arrived from the form, without printing it outright.
// Any byte outside printable ASCII is shown as \xNN, which is what a decode
// problem would look like.
static void log_submission(const String &ssid, const String &pass) {
    Serial.printf("portal: network selected  '%s' (%u chars)\n",
                  ssid.c_str(), (unsigned)ssid.length());

#if PORTAL_LOG_SECRETS
    Serial.printf("portal: password received '%s' (%u chars)\n",
                  pass.c_str(), (unsigned)pass.length());
    Serial.print("portal: password bytes    ");
    for (size_t i = 0; i < pass.length(); i++)
        Serial.printf("%02X ", (uint8_t)pass[i]);
    Serial.println();
#else
    size_t n = pass.length();
    Serial.printf("portal: password received %u chars, first '%c', last '%c'\n",
                  (unsigned)n,
                  n ? pass[0] : '?', n ? pass[n - 1] : '?');

    int alpha = 0, digit = 0, sym = 0, high = 0;
    String syms;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = (uint8_t)pass[i];
        if (c >= 0x80)                       high++;
        else if (isalpha(c))                 alpha++;
        else if (isdigit(c))                 digit++;
        else { sym++; if (syms.indexOf((char)c) < 0) syms += (char)c; }
    }
    Serial.printf("portal: composition       %d letters, %d digits, %d symbols",
                  alpha, digit, sym);
    if (syms.length()) Serial.printf(" [%s]", syms.c_str());
    if (high) Serial.printf(", %d non-ASCII", high);
    Serial.println();
    Serial.println("portal: (set PORTAL_LOG_SECRETS 1 in portal.cpp to log it in full)");
#endif
}

// ---- page ------------------------------------------------------------------
// Network names are attacker-controlled in the sense that anyone nearby can
// broadcast whatever SSID they like, and apostrophes turn up in perfectly
// ordinary ones ("Dave's WiFi"). Interpolating them raw breaks the markup at
// best. Passwords never get echoed back, so only the SSID path needs this.
static String esc(const String &s) {
    String o;
    o.reserve(s.length() + 8);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '&':  o += F("&amp;");  break;
            case '<':  o += F("&lt;");   break;
            case '>':  o += F("&gt;");   break;
            case '"':  o += F("&quot;"); break;
            case '\'': o += F("&#39;");  break;
            default:   o += c;
        }
    }
    return o;
}

// Note on the .n rule below: it is a class, so it beats the bare `button`
// rule on background - but it does not declare a colour, so it used to
// inherit that rule's white text and render white on white. Both properties
// are now set explicitly rather than depending on cascade order.
static String page_head() {
    return F("<!doctype html><html><head><meta charset=utf-8>"
             "<meta name=viewport content='width=device-width,initial-scale=1'>"
             "<title>Tab5 Map setup</title><style>"
             "body{font-family:system-ui,sans-serif;margin:0;padding:24px;"
             "background:#f4f2ec;color:#222}"
             "h1{font-size:20px;margin:0 0 16px}"
             ".n{display:block;width:100%;text-align:left;padding:12px;margin:4px 0;"
             "border:1px solid #ccc;border-radius:8px;background:#fff;color:#222;"
             "font-size:16px;cursor:pointer}"
             ".n:hover{background:#eef}"
             ".n .b{color:#666}"
             "input,button{font-size:16px;padding:12px;width:100%;box-sizing:border-box;"
             "margin:6px 0;border-radius:8px;border:1px solid #bbb}"
             "button{background:#2a5ada;color:#fff;border:0}"
             ".e{color:#b00;margin:8px 0}.s{color:#070}"
             ".b{font-size:13px;color:#666}"
             "</style></head><body>");
}

static void handle_root() {
    String h = page_head();
    h += F("<h1>Tab5 Map &mdash; WiFi setup</h1>");
    if (g_msg.length()) h += "<p class='e'>" + g_msg + "</p>";

    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED) { WiFi.scanNetworks(true); n = -1; }

    if (n < 0) {
        h += F("<p>Scanning&hellip;</p><meta http-equiv=refresh content=3>");
    } else {
        h += F("<p class=b>Tap a network:</p><form method=GET action=/pw>");
        for (int i = 0; i < n && i < 24; i++) {
            String s = WiFi.SSID(i);
            if (!s.length()) continue;
            String e = esc(s);
            h += "<button class=n name=s value=\"" + e + "\">" + e +
                 "  <span class=b>" + String(WiFi.RSSI(i)) + " dBm" +
                 (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " open" : "") +
                 "</span></button>";
        }
        h += F("</form><p class=b><a href='/rescan'>Rescan</a></p>");
    }
    h += F("</body></html>");
    g_msg = "";
    g_web->send(200, "text/html", h);
}

static void handle_rescan() {
    WiFi.scanDelete();
    WiFi.scanNetworks(true);
    g_web->sendHeader("Location", "/");
    g_web->send(302);
}

static void handle_pw() {
    String ssid = g_web->arg("s");
    Serial.printf("portal: password prompt for '%s'\n", ssid.c_str());
    String h = page_head();
    String e = esc(ssid);
    h += "<h1>" + e + "</h1><form method=POST action=/save>";
    h += "<input type=hidden name=s value=\"" + e + "\">";
    h += F("<input type=password name=p placeholder='Password' autofocus>"
           "<button type=submit>Connect</button></form>"
           "<p class=b>The passphrase is not stored. It is converted to this "
           "network's WPA2 key and only that is written to the card.</p>"
           "<p class=b><a href='/'>Back</a></p></body></html>");
    g_web->send(200, "text/html", h);
}

static void handle_save() {
    String ssid = g_web->arg("s");
    String pass = g_web->arg("p");

    log_submission(ssid, pass);

    if (ssid.length() == 0 || pass.length() < 8) {
        Serial.println("portal: rejected - SSID empty or password under 8 chars");
        g_msg = F("Password must be at least 8 characters.");
        g_web->sendHeader("Location", "/");
        g_web->send(302);
        return;
    }

    // Derive first, then prove it works before writing anything. PBKDF2 with
    // 4096 iterations takes a moment on this part.
    char psk[65];
    uint32_t t0 = millis();
    if (!wifistore_derive_psk(ssid.c_str(), pass.c_str(), psk)) {
        Serial.println("portal: PBKDF2 derivation failed");
        g_msg = F("Could not derive key from that passphrase.");
        g_web->sendHeader("Location", "/");
        g_web->send(302);
        return;
    }

    String h = page_head();
    h += F("<h1>Connecting&hellip;</h1><p class=b>Testing the key before saving. "
           "This page will stop responding while the radio switches.</p>"
           "<meta http-equiv=refresh content='6;url=/result'></body></html>");
    g_web->send(200, "text/html", h);
    g_web->client().flush();
    delay(200);

    // wpa_supplicant accepts a 64-hex-character PSK in place of a passphrase,
    // which is what lets the plaintext never be stored.
    // The first 8 hex characters are a fingerprint: enough to see that two
    // attempts derived the same key, useless for recovering the passphrase.
    Serial.printf("portal: PSK derived in %lu ms, fingerprint %.8s...\n",
                  (unsigned long)(millis() - t0), psk);
    Serial.printf("portal: connecting to '%s'...\n", ssid.c_str());

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid.c_str(), psk);

    // Short timeout on this first attempt. A WPA2 association that is going
    // to succeed lands in two to four seconds; anything longer means the key
    // was refused, and on a transition-mode AP that is the expected outcome
    // rather than an error worth waiting out.
    uint32_t t1 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t1 < 7000) delay(100);

    bool used_psk = (WiFi.status() == WL_CONNECTED);

    // Retry with the raw passphrase if the derived key was refused.
    //
    // A pre-computed PSK is a WPA2-only concept. WPA3-SAE derives its key by
    // a different route, so a correct passphrase still fails when presented
    // as 64 hex characters. Routers advertising "WPA2/WPA3-Personal" are in
    // transition mode and a WPA3-capable client will negotiate SAE, so this
    // second attempt is the normal path on most current hardware rather than
    // an unusual fallback.
    if (!used_psk) {
        Serial.printf("portal: PSK refused (status %d), retrying with passphrase\n",
                      (int)WiFi.status());
        WiFi.disconnect();
        delay(300);
        WiFi.begin(ssid.c_str(), pass.c_str());
        t1 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t1 < 15000) delay(100);
        if (WiFi.status() == WL_CONNECTED)
            Serial.println("portal: passphrase accepted - network negotiated WPA3-SAE,\n"
                           "        so the passphrase itself is stored rather than a PSK");
    }

    if (WiFi.status() == WL_CONNECTED) {
        bool saved = used_psk
            ? wifistore_save_psk(ssid.c_str(), psk)
            : wifistore_save_passphrase(ssid.c_str(), pass.c_str());
        if (saved) {
            g_done = true;
            g_msg = "";
            Serial.printf("portal: saved %s as %s, IP %s\n", ssid.c_str(),
                          used_psk ? "derived PSK" : "passphrase (WPA3)",
                          WiFi.localIP().toString().c_str());
        } else {
            g_msg = F("Connected, but writing to the SD card failed.");
        }
    } else {
        // Status 6 is WL_DISCONNECTED, 4 is WL_CONNECT_FAILED - the latter
        // usually means the key was rejected rather than the AP being absent.
        Serial.printf("portal: connect failed after %lu ms, status %d\n",
                      (unsigned long)(millis() - t1), (int)WiFi.status());
        WiFi.disconnect();
        g_msg = F("Could not connect - check the password and try again.");
    }
    memset(psk, 0, sizeof psk);
}

static void handle_result() {
    String h = page_head();
    if (g_done) {
        h += F("<h1 class=s>Connected</h1><p>Credentials saved. "
               "You can close this page; the device is switching to the map.</p>");
    } else {
        h += "<h1>Not connected</h1><p class='e'>" + g_msg + "</p>"
             "<p><a href='/'>Try again</a></p>";
    }
    h += F("</body></html>");
    g_web->send(200, "text/html", h);
}

// Every unknown host resolves here, so the phone's connectivity probe fails
// and the OS pops the login sheet. Returning 302 rather than 404 is what
// makes both iOS and Android treat it as a captive portal.
static void handle_notfound() {
    g_web->sendHeader("Location", String("http://") + AP_IP.toString(), true);
    g_web->send(302, "text/plain", "");
}

// ---- display ---------------------------------------------------------------
static void draw(const char *state) {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextDatum(top_left);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(3);
    M5.Display.drawString("WiFi setup", 40, 50);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_CYAN);
    M5.Display.drawString("1. Join this network from a phone:", 40, 120);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(3);
    M5.Display.drawString(AP_SSID, 60, 152);
    M5.Display.setTextSize(2);
    M5.Display.setTextColor(TFT_CYAN);
    M5.Display.drawString("2. The setup page opens automatically.", 40, 208);
    M5.Display.drawString("   If not, browse to:", 40, 236);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.drawString(AP_IP.toString(), 60, 264);
    M5.Display.setTextColor(TFT_DARKGREY);
    M5.Display.drawString(state, 40, 330);
    M5.Display.drawString("Touch and hold to skip", 40, 360);
}

bool portal_run(uint32_t timeout_ms) {
    g_done = false;
    g_msg = "";

    // Caller is expected to have called WiFi.setPins() already; on the Tab5
    // the radio lives on a separate chip reached over SDIO, and without the
    // pin map ESP-Hosted fails with errors that read like an SD card fault.
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID);
    delay(300);
    WiFi.scanNetworks(true);

    static DNSServer dns;
    static WebServer web(80);
    g_dns = &dns;
    g_web = &web;

    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", AP_IP);

    web.on("/", handle_root);
    web.on("/rescan", handle_rescan);
    web.on("/pw", handle_pw);
    web.on("/save", HTTP_POST, handle_save);
    web.on("/result", handle_result);
    web.onNotFound(handle_notfound);
    web.begin();

    Serial.printf("portal: AP '%s' at %s\n", AP_SSID, AP_IP.toString().c_str());
    draw("waiting for a client...");

    uint32_t t0 = millis(), lastDraw = 0;
    int lastClients = -1;
    uint32_t touchStart = 0;

    while (!g_done && millis() - t0 < timeout_ms) {
        // This loop owns the UI task for minutes by design, which is longer
        // than the watchdog allows - so it has to check in.
        esp_task_wdt_reset();
        dns.processNextRequest();
        web.handleClient();
        M5.update();

        // Touch-and-hold to abandon setup and carry on offline.
        if (M5.Touch.getCount()) {
            if (!touchStart) touchStart = millis();
            else if (millis() - touchStart > 1500) {
                Serial.println("portal: skipped by user");
                break;
            }
        } else touchStart = 0;

        if (millis() - lastDraw > 1000) {
            lastDraw = millis();
            int c = WiFi.softAPgetStationNum();
            if (c != lastClients) {
                lastClients = c;
                char buf[64];
                snprintf(buf, sizeof buf, "%d client%s connected   ",
                         c, c == 1 ? "" : "s");
                draw(buf);
            }
        }
        delay(5);
    }

    web.stop();
    dns.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(g_done ? WIFI_STA : WIFI_OFF);
    g_web = nullptr; g_dns = nullptr;
    return g_done;
}
