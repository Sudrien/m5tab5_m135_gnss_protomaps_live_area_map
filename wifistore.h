// wifistore.h - device-bound credential storage on the SD card.
//
// WHAT THIS DOES AND DOES NOT PROTECT
//
// WiFi credentials cannot be one-way hashed: the device has to present a
// usable credential to authenticate, so anything it can read, an attacker
// holding the device can read too. Two things are still worth doing:
//
//   1. Where possible, store the derived PSK rather than the passphrase.
//      WPA2 runs PBKDF2-HMAC-SHA1(passphrase, ssid, 4096) to produce a
//      256-bit PSK, and wpa_supplicant accepts that directly as 64 hex
//      characters, so the passphrase - which people reuse across services -
//      never touches the card.
//
//      This only applies to WPA2. WPA3-SAE derives its key by a different
//      route and has no precomputable equivalent, so on SAE networks the
//      passphrase itself must be stored. Routers advertising
//      "WPA2/WPA3-Personal" run transition mode and a WPA3-capable client
//      will pick SAE, which makes the passphrase path the common case on
//      current hardware, not the exception. `is_psk` records which kind a
//      given record holds.
//
//   2. Encrypt at rest with a key derived from the factory eFuse MAC, so the
//      file is inert if the card is read on another machine. This applies to
//      both kinds of record.
//
// The realistic threat here is a misplaced SD card, and that is covered.
// A stolen *device* is not, and no scheme storing usable credentials could
// cover it. If you need that, use NVS with flash encryption enabled.

#ifndef WIFISTORE_H
#define WIFISTORE_H

#include <stdint.h>

struct WifiCred {
    char ssid[33] = "";
    char secret[65] = "";   // 64 hex PSK, or a passphrase for WPA3-SAE
    bool is_psk   = true;   // false when `secret` is a raw passphrase
    bool valid    = false;
};

// Derive the PSK from a passphrase and write the encrypted record.
// Takes a few hundred ms: PBKDF2 with 4096 iterations is deliberately slow.
bool wifistore_save(const char *ssid, const char *passphrase);

// Save an already-derived PSK (64 hex chars), skipping PBKDF2.
bool wifistore_save_psk(const char *ssid, const char *psk_hex);

// Save a raw passphrase. Needed only for WPA3-SAE, which does not use a
// pre-computed PSK - there the passphrase itself is the credential and
// there is nothing to derive in advance.
bool wifistore_save_passphrase(const char *ssid, const char *passphrase);

bool wifistore_load(WifiCred *out);
bool wifistore_clear();
bool wifistore_exists();

// Exposed so the portal can verify a passphrase before committing it.
bool wifistore_derive_psk(const char *ssid, const char *passphrase,
                          char out_hex[65]);

// Print what is stored and whether it decodes, without revealing the PSK.
void wifistore_diag();

#endif // WIFISTORE_H
