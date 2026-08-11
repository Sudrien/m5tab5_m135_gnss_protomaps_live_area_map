#include "storage.h"
#include "features.h"
#include <SD.h>
#include <SD_MMC.h>
#include <M5Unified.h>

#if MAP_HAVE_USB_MSC
// Defined in main/idf_usb_msc.cpp, which only exists in the IDF build.
bool     usb_msc_begin();
bool     usb_msc_mounted();
fs::FS  *usb_msc_fs();
#endif

static fs::FS     *g_fs   = nullptr;
static const char *g_name = "none";
static bool        g_have  = false;

// ---- USB-A bus power -------------------------------------------------------
//
// PI4IOE5V6416 #2 at 0x44, pin P3. Three registers, in this order, and all
// three matter:
//
//   IO_DIR   (0x03) bit 3 = 1   make P3 an output
//   OUT_H_IM (0x07) bit 3 = 0   take it out of high impedance
//   OUT_SET  (0x05) bit 3 = 1   drive it high
//
// The high-impedance register is the one that is easy to miss. The expander
// parks pins high-Z after reset, so writing OUT_SET alone leaves P3 floating
// and VBUS off - with no error anywhere, because nothing on the P4 side has
// gone wrong.
//
// M5Unified actively works against us here: M5GFX's autodetect sets IO_DIR to
// 0b10111001 (P3 an output), and Power_Class::begin() then writes 0b10110001
// over it, putting P3 back to an input. So this has to run *after* M5.begin(),
// not before, and it cannot be done once at startup and forgotten if anything
// later re-runs the power init.
// src: M5Unified utility/Power_Class.cpp, Tab5 case - the second expander is
//      addressed at 0x44 there (pi4io2_i2c_addr in M5GFX's autodetect agrees).
static const uint8_t PI4IOE2_ADDR    = 0x44;

// src: PI4IOE5V6416 datasheet, register map. 0x03 configuration (0 = output,
//      1 = input on this part - note M5's tables use 1 = output, see below),
//      0x05 output port, 0x07 output high-impedance enable.
// src: m5tab5_esp_idf_usb_host_example writes these three in this order, and
//      the order matters: direction, then out of high-Z, then drive.
static const uint8_t PI4IOE_IO_DIR   = 0x03;
static const uint8_t PI4IOE_OUT_SET  = 0x05;
static const uint8_t PI4IOE_OUT_H_IM = 0x07;

// src: M5Unified utility/Power_Class.cpp, the commented bit table for 0x44 -
//      bit 0 WLAN_PWR_EN, bit 3 USB5V_EN, bit 4 PWROFF_PLUSE, bit 5
//      nCHG_QC_EN, bit 6 CHG_STAT, bit 7 CHG_EN.
// src: m5tab5_esp_idf_usb_host_example, PI4IOE2_IO_DIR = 0xB9, whose bit 3 is
//      what distinguishes it from M5Unified's 0xB1.
static const uint8_t USB5V_EN_BIT    = 1 << 3;

// src: M5GFX M5GFX.cpp, Tab5 autodetect - every expander write there is
//      i2c_write_register8_array(..., 100000). Matched rather than chosen: the
//      two drivers talk to the same device on the same bus, and there is no
//      reason for this one to be the odd one out. The part is rated for more.
static const uint32_t EXP_FREQ       = 100000;

void storage_usb_power(bool on) {
#if MAP_HAVE_USB_MSC
    // Idempotent, and quiet about it.
    //
    // pick() runs on every storage_rescan(), which during a wait is once a
    // second, and the first version of this re-wrote all three expander
    // registers, slept 100 ms for inrush and printed a line every time. The
    // registers do not need rewriting and the port does not need re-powering;
    // only a change does.
    static int applied = -1;                       // -1 = never written
    if (applied == (int)on) return;
    applied = (int)on;

    M5.In_I2C.bitOn (PI4IOE2_ADDR, PI4IOE_IO_DIR,   USB5V_EN_BIT, EXP_FREQ);
    M5.In_I2C.bitOff(PI4IOE2_ADDR, PI4IOE_OUT_H_IM, USB5V_EN_BIT, EXP_FREQ);
    if (on) M5.In_I2C.bitOn (PI4IOE2_ADDR, PI4IOE_OUT_SET, USB5V_EN_BIT, EXP_FREQ);
    else    M5.In_I2C.bitOff(PI4IOE2_ADDR, PI4IOE_OUT_SET, USB5V_EN_BIT, EXP_FREQ);
    Serial.printf("usb: VBUS %s (expander 0x%02X, P3)\n", on ? "on" : "off",
                  PI4IOE2_ADDR);
    // The port's inrush wants a moment before the host stack starts driving
    // bus resets at whatever is plugged into it.
    if (on) delay(100);
#else
    (void)on;
#endif
}

bool storage_card_present() { return SD_MMC.cardType() != CARD_NONE; }

// Adding USB mass storage
// -----------------------
// The consumers of this file are already written against fs::FS, so the
// application side of USB support is this function and nothing else - add a
// branch, and the tile cache, the manifest reader and the credential store all
// follow with no changes.
//
// The work that remains is entirely below Arduino:
//
//   1. USB host MSC is not exposed by the Arduino ESP32 core. It lives in
//      ESP-IDF's usb_host_msc component, which mounts a drive through FATFS at
//      a VFS path. Pulling an IDF managed component into a .ino sketch is the
//      awkward part - it generally means moving to an IDF component build, or
//      wrapping the driver in a library the sketch can include.
//
//   2. Once mounted at a VFS path, exposing it as an fs::FS is straightforward:
//      that is exactly what SD_MMC is - a VFSImpl pointed at /sdcard.
//
//   3. Hardware: confirmed available. The Tab5 block diagram shows USBA wired
//      to the P4 as USB2_OTG_D+/D- - the OTG-capable controller, not the
//      USB-Serial/JTAG that USBC uses for flashing - and SYS_USB5V feeding the
//      USBA connector, so the port can both host and supply bus power to a
//      drive. Nothing in the hardware blocks this; the obstacle is entirely
//      the driver plumbing in 1 and 2.
//
//      Note this is the same controller the board exposes for USB host in
//      general, so enabling it does not conflict with flashing over USBC.
//
// Order matters here. SD is checked first so that a card, when present, keeps
// winning - a user who has both plugged in should not have their cache move
// depending on enumeration timing.
// Which medium wins when both are present.
//
// USB first by default, because a flash drive is the intended main store: it
// is larger, faster to load a 40 MB tile cache onto from a desktop, and does
// not require taking the back off anything. The card slot stays as the
// fallback and as the only option on an Arduino build.
//
// Set to 0 to restore SD-first.
#ifndef MAP_STORAGE_PREFER_USB
#  define MAP_STORAGE_PREFER_USB MAP_HAVE_USB_MSC
#endif

// Power the port and install the host stack without waiting for an answer.
//
// Worth calling early and separately from pick(): enumeration happens on the
// USB driver's own task and takes a moment, so starting it before the SD
// attempt means the drive is often already there by the time anything asks.
void storage_usb_begin() {
#if MAP_HAVE_USB_MSC
    storage_usb_power(true);
    usb_msc_begin();
#endif
}

static void pick() {
    g_have = true;

#if MAP_HAVE_USB_MSC && MAP_STORAGE_PREFER_USB
    storage_usb_begin();
    if (usb_msc_mounted()) { g_fs = usb_msc_fs(); g_name = "USB"; return; }
#endif

    if (SD_MMC.cardType() != CARD_NONE) { g_fs = &SD_MMC; g_name = "SD (SDMMC)"; return; }

#if MAP_HAVE_USB_MSC && !MAP_STORAGE_PREFER_USB
    // Bringing the stack up is idempotent and cheap; a drive only appears
    // later, on the driver's own task, so the first pick() almost never sees
    // one. storage_rescan() is what eventually finds it.
    //
    // VBUS first, every time. It is one I2C write per register and the port is
    // dead without it - see storage_usb_power().
    storage_usb_begin();
    if (usb_msc_mounted()) { g_fs = usb_msc_fs(); g_name = "USB"; return; }
#endif

    // Nothing mounted. The SPI SD object is still handed back rather than
    // null, because every caller is written against a valid fs::FS whose calls
    // fail cleanly - but g_have records that this is the empty fallback, so
    // the boot path can tell "no media" from "media, and it is this one".
    g_fs = &SD; g_name = "SD (SPI)"; g_have = false;
}

fs::FS *storage_fs() {
    if (!g_fs) pick();
    return g_fs;
}

const char *storage_name() {
    if (!g_fs) pick();
    return g_name;
}

void storage_rescan() {
    g_fs = nullptr;
    pick();
}

size_t storage_read(fs::File &f, uint8_t *dst, size_t len) {
    size_t done = 0;
    while (done < len) {
        size_t want = len - done;
        if (want > STORAGE_IO_CHUNK) want = STORAGE_IO_CHUNK;
        int got = f.read(dst + done, want);
        if (got <= 0) break;
        done += (size_t)got;
        if ((size_t)got < want) break;          // short read: end of file
    }
    return done;
}

size_t storage_write(fs::File &f, const uint8_t *src, size_t len) {
    size_t done = 0;
    while (done < len) {
        size_t want = len - done;
        if (want > STORAGE_IO_CHUNK) want = STORAGE_IO_CHUNK;
        size_t put = f.write(src + done, want);
        done += put;
        if (put < want) break;                  // out of space, or a bus error
    }
    return done;
}

bool storage_available() {
    if (!g_fs) pick();
    return g_have;
}
