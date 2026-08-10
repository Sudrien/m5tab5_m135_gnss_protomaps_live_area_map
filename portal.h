// portal.h - captive portal for WiFi setup.
//
// Brings up an open AP, answers every DNS query with its own address so the
// phone's connectivity check redirects into the setup page, and serves a
// scan list with a password field.
//
// Credentials are verified before they are stored: the portal actually joins
// the network with the derived PSK and only writes the file if the join
// succeeds. Saving first and discovering the typo on the next boot is a far
// worse experience, especially on a device with no keyboard.

#ifndef PORTAL_H
#define PORTAL_H

#include <stdint.h>

// Blocks until credentials are saved, the user cancels, or the timeout
// expires. Returns true if a working credential was stored.
// Draws its own progress on the display.
bool portal_run(uint32_t timeout_ms);

#endif // PORTAL_H
