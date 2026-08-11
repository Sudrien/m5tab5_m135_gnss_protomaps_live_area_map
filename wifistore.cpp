// wifistore.cpp

#include "wifistore.h"
#include <Arduino.h>
#include "storage.h"
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_efuse.h>
#include <esp_random.h>
#include <mbedtls/pkcs5.h>
#include <mbedtls/md.h>
#include <mbedtls/version.h>
#include <mbedtls/aes.h>
#include <string.h>

static const char *CRED_PATH = "/wifi.bin";
// src: chosen. Both are this project's own file format - the magic exists to
//      reject a file that is not ours before decryption is attempted, and
//      the version to allow a layout change later without a silent misread.
static const uint8_t MAGIC[8] = { 'T','A','B','5','W','I','F','I' };
static const uint8_t VERSION  = 1;

// File layout:
//   magic[8] | version | iv[16] | len[2] | ciphertext[len] | hmac[32]
// The HMAC covers everything before it, so truncation and tampering are both
// caught rather than producing a plausible-looking credential.

static fs::FS *fsptr() {
    // Whichever store the boot sequence mounted. map_begin got there first.
    return storage_fs();
}

// ---- device-bound keys -----------------------------------------------------
// Two independent keys from the same device id, separated by domain strings so
// the encryption key can never be used as the MAC key or vice versa.
//
// The identifier must be the FACTORY eFuse MAC, not the WiFi station MAC. On
// the P4 the WiFi MAC belongs to the ESP32-C6 companion and is not readable
// until ESP-Hosted is up - so a key derived from it would differ between a
// save (after connecting) and the next boot's load (before connecting), and
// the record would fail to decrypt on a device that is working perfectly.
static bool device_id(uint8_t mac[6]) {
    memset(mac, 0, 6);
    if (esp_efuse_mac_get_default(mac) == ESP_OK) {
        for (int i = 0; i < 6; i++) if (mac[i]) return true;
    }
    return false;   // all zeros means we have no stable identity to bind to
}

static void device_key(const char *domain, uint8_t out[32]) {
    uint8_t mac[6];
    device_id(mac);

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const uint8_t *)domain, strlen(domain));
    mbedtls_md_update(&ctx, mac, sizeof mac);
    mbedtls_md_finish(&ctx, out);
    mbedtls_md_free(&ctx);
}

static void hmac_sha256(const uint8_t *key, const uint8_t *data, size_t len,
                        uint8_t out[32]) {
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, key, 32);
    mbedtls_md_hmac_update(&ctx, data, len);
    mbedtls_md_hmac_finish(&ctx, out);
    mbedtls_md_free(&ctx);
}

// ---- PSK derivation --------------------------------------------------------
bool wifistore_derive_psk(const char *ssid, const char *passphrase,
                          char out_hex[65])
{
    if (!ssid || !*ssid || !passphrase) return false;
    size_t plen = strlen(passphrase);
    if (plen < 8 || plen > 63) return false;   // WPA2 passphrase limits

    uint8_t psk[32];

    // This is the WPA2 key derivation exactly as specified: the SSID is the
    // salt, which is what binds the result to one network.
    //
    // mbedTLS 3.6 removed mbedtls_pkcs5_pbkdf2_hmac in favour of the _ext
    // form, which takes the digest type directly instead of a pre-configured
    // md context. Both spellings are handled so this builds against either
    // core version.
#if MBEDTLS_VERSION_NUMBER >= 0x03060000
    int rc = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA1,
                (const uint8_t *)passphrase, plen,
                (const uint8_t *)ssid, strlen(ssid),
                4096, 32, psk);
    if (rc != 0) return false;
#else
    const mbedtls_md_info_t *sha1 = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
    if (!sha1) return false;

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    if (mbedtls_md_setup(&ctx, sha1, 1) != 0) { mbedtls_md_free(&ctx); return false; }

    int rc = mbedtls_pkcs5_pbkdf2_hmac(&ctx,
                (const uint8_t *)passphrase, plen,
                (const uint8_t *)ssid, strlen(ssid),
                4096, 32, psk);
    mbedtls_md_free(&ctx);
    if (rc != 0) return false;
#endif

    for (int i = 0; i < 32; i++) sprintf(out_hex + i * 2, "%02x", psk[i]);
    out_hex[64] = 0;
    return true;
}

// ---- write -----------------------------------------------------------------
static bool save_secret(const char *ssid, const char *secret, bool is_psk) {
    if (!ssid || !*ssid || !secret || !*secret) return false;
    if (strlen(secret) > 64) return false;

    // Layout inside the ciphertext: ssid[33] | flag | secret[64]
    uint8_t plain[112];
    memset(plain, 0, sizeof plain);
    strncpy((char *)plain, ssid, 32);
    plain[32] = is_psk ? 1 : 0;
    strncpy((char *)plain + 33, secret, 64);
    // PKCS#7 pad to the AES block size.
    size_t raw = 112;
    size_t pad = 16 - (raw % 16);
    if (pad == 16) pad = 0;
    uint8_t buf[128];
    memcpy(buf, plain, raw);
    for (size_t i = 0; i < pad; i++) buf[raw + i] = (uint8_t)pad;
    size_t clen = raw + pad;

    uint8_t iv[16], iv_copy[16];
    esp_fill_random(iv, sizeof iv);
    memcpy(iv_copy, iv, sizeof iv);

    uint8_t key[32];
    device_key("tab5-map-wifi-enc-v1", key);

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, key, 256);
    uint8_t ct[128];
    int rc = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, clen, iv_copy, buf, ct);
    mbedtls_aes_free(&aes);
    if (rc != 0) return false;

    uint8_t rec[8 + 1 + 16 + 2 + 128];
    size_t n = 0;
    memcpy(rec + n, MAGIC, 8); n += 8;
    rec[n++] = VERSION;
    memcpy(rec + n, iv, 16); n += 16;
    rec[n++] = (uint8_t)(clen & 0xFF);
    rec[n++] = (uint8_t)(clen >> 8);
    memcpy(rec + n, ct, clen); n += clen;

    uint8_t mkey[32], mac[32];
    device_key("tab5-map-wifi-mac-v1", mkey);
    hmac_sha256(mkey, rec, n, mac);

    fs::FS *f = fsptr();
    f->remove(CRED_PATH);
    File out = f->open(CRED_PATH, FILE_WRITE);
    if (!out) return false;
    out.write(rec, n);
    out.write(mac, 32);
    out.close();

    // Wipe the derived material from stack memory on the way out.
    memset(buf, 0, sizeof buf);
    memset(plain, 0, sizeof plain);
    memset(key, 0, sizeof key);
    return true;
}

bool wifistore_save_psk(const char *ssid, const char *psk_hex) {
    if (!psk_hex || strlen(psk_hex) != 64) return false;
    return save_secret(ssid, psk_hex, true);
}

bool wifistore_save_passphrase(const char *ssid, const char *passphrase) {
    return save_secret(ssid, passphrase, false);
}

bool wifistore_save(const char *ssid, const char *passphrase) {
    char psk[65];
    if (!wifistore_derive_psk(ssid, passphrase, psk)) return false;
    bool ok = wifistore_save_psk(ssid, psk);
    memset(psk, 0, sizeof psk);
    return ok;
}

// ---- read ------------------------------------------------------------------
bool wifistore_load(WifiCred *out) {
    if (!out) return false;
    out->valid = false;

    File in = fsptr()->open(CRED_PATH, FILE_READ);
    if (!in) return false;
    size_t sz = in.size();
    if (sz < 8 + 1 + 16 + 2 + 16 + 32 || sz > 256) { in.close(); return false; }

    uint8_t rec[256];
    if (in.read(rec, sz) != (int)sz) { in.close(); return false; }
    in.close();

    if (memcmp(rec, MAGIC, 8) != 0 || rec[8] != VERSION) return false;

    size_t body = sz - 32;
    uint8_t mkey[32], want[32];
    device_key("tab5-map-wifi-mac-v1", mkey);
    hmac_sha256(mkey, rec, body, want);
    // Constant-time compare: not that timing matters much here, but a
    // memcmp on a MAC is a bad habit to leave lying around.
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) diff |= want[i] ^ rec[body + i];
    if (diff) {
        Serial.println("wifistore: HMAC mismatch (wrong device, or tampered)");
        return false;
    }

    uint8_t iv[16];
    memcpy(iv, rec + 9, 16);
    size_t clen = rec[25] | ((size_t)rec[26] << 8);
    if (clen == 0 || clen % 16 || 27 + clen != body) return false;

    uint8_t key[32];
    device_key("tab5-map-wifi-enc-v1", key);
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_dec(&aes, key, 256);
    uint8_t plain[128];
    int rc = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, clen, iv,
                                   rec + 27, plain);
    mbedtls_aes_free(&aes);
    memset(key, 0, sizeof key);
    if (rc != 0) return false;

    out->is_psk = (plain[32] == 1);
    plain[32] = 0;
    strncpy(out->ssid, (const char *)plain, 32);
    out->ssid[32] = 0;
    strncpy(out->secret, (const char *)plain + 33, 64);
    out->secret[64] = 0;
    memset(plain, 0, sizeof plain);

    if (!out->ssid[0] || !out->secret[0]) return false;
    if (out->is_psk && strlen(out->secret) != 64) return false;
    out->valid = true;
    return true;
}

void wifistore_diag() {
    uint8_t mac[6];
    bool ok = device_id(mac);
    Serial.printf("wifistore: device id %02X:%02X:%02X:%02X:%02X:%02X %s\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  ok ? "" : "(ZERO - keys will not be device-unique)");
    File f = fsptr()->open(CRED_PATH, FILE_READ);
    if (!f) { Serial.println("wifistore: no /wifi.bin"); return; }
    Serial.printf("wifistore: /wifi.bin %u bytes\n", (unsigned)f.size());
    f.close();

    WifiCred c;
    if (wifistore_load(&c))
        Serial.printf("wifistore: decodes ok, ssid '%s', stored as %s\n",
                      c.ssid, c.is_psk ? "derived PSK" : "passphrase (WPA3)");
    else                    Serial.println("wifistore: record present but did NOT decode");
}

bool wifistore_exists() {
    File f = fsptr()->open(CRED_PATH, FILE_READ);
    if (!f) return false;
    f.close();
    return true;
}

bool wifistore_clear() {
    return fsptr()->remove(CRED_PATH);
}
