// storage.h - one place that decides which filesystem everything else uses.
//
// The choice was previously made in three files independently, each with its
// own copy of the same SD_MMC-else-SD test. That works only while there is
// exactly one answer; the moment a second backing store exists, three copies
// are three chances to disagree about which one is mounted, and the failure
// would be a subset of the code silently reading from a different device.
//
// Everything here is deliberately thin: it does not mount anything, it only
// reports what the boot sequence already mounted.

#pragma once
#include <FS.h>

// The filesystem holding /t (tile cache, manifests) and /wifi.bin.
// Never null - falls back to the SPI SD object, whose calls fail cleanly if
// nothing is mounted, which is the same behaviour the callers had before.
fs::FS *storage_fs();

// Human-readable name of the active store, for the boot line.
const char *storage_name();

// Re-run the selection. Call after mounting or removing a device.
void storage_rescan();

// Is there actually a device behind storage_fs(), or is this the empty
// fallback? storage_fs() is never null by design, so "did anything mount"
// cannot be answered by comparing it against nullptr.
bool storage_available();

// Bring the USB-A port's VBUS up.
//
// The port is electrically dead until this runs: USB5V_EN is P3 of the second
// PI4IOE5V6416 at I2C 0x44, not a GPIO, and M5Unified's Power_Class::begin()
// leaves that pin configured as an input. Nothing reports the port as off -
// the host stack installs, the client task registers, and no device ever
// enumerates.
//
// Safe to call more than once, and a no-op on builds without USB support.
void storage_usb_power(bool on);

// True while a card is physically in the slot.
//
// Polled, because it has to be: the microSD connector's detect switch is not
// wired to the SoC on this board - M5's BSP passes GPIO_NUM_NC for it - so
// there is no edge to interrupt on.
bool storage_card_present();
