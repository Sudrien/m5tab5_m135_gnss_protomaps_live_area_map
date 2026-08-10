// features.h - what this build can do.
//
// The Arduino ESP32 core is ESP-IDF underneath, so this is not two codebases.
// The same sources build either way; the difference is that an IDF project can
// pull in managed components (USB host MSC, for one) that the Arduino build has
// no mechanism to add. Anything in that category is gated here rather than
// scattered through the files that use it.
//
// MAP_BUILD_IDF is set by the IDF build (see CMakeLists.txt). It is not derived
// from ARDUINO, because arduino-esp32 used as an IDF component defines ARDUINO
// too - the two are not mutually exclusive and testing for it would silently
// select the wrong branch in exactly the configuration this exists to support.

#pragma once

#ifndef MAP_BUILD_IDF
#define MAP_BUILD_IDF 0
#endif

// ---- USB mass storage ------------------------------------------------------
// Needs ESP-IDF's usb_host_msc component, so it is an IDF-build feature: the
// hardware is capable either way - the Tab5 block diagram shows USBA on the
// P4's USB2_OTG_D+/D- with SYS_USB5V feeding the connector, so it can host a
// drive and power it - but the driver is out of reach from Arduino.
//
// On by default under IDF now. It was off for two reasons, and both are gone:
//
//   - The dependency had to resolve for the project to configure at all, and a
//     pin the registry could not satisfy stopped the build before anything
//     else was tried. main/idf_component.yml no longer pins a version.
//   - The code had never been run against hardware. It is still not proven
//     here, but every board-specific detail in it now comes from a working
//     reference (m5tab5_esp_idf_usb_host_example) rather than from reading
//     headers - in particular that the port has no VBUS until USB5V_EN, which
//     is P3 of the I/O expander at 0x44, is driven high. Without that the host
//     stack installs cleanly and nothing ever enumerates, with no error
//     anywhere. See storage_usb_power().
//
// To turn it off again:
//   idf.py -DMAP_HAVE_USB_MSC=0 build
#ifndef MAP_HAVE_USB_MSC
#  define MAP_HAVE_USB_MSC MAP_BUILD_IDF
#endif

// ---- what is NOT gained by building under IDF -------------------------------
// exFAT. Worth stating explicitly because it is the obvious assumption and it
// is wrong: ESP-IDF hardcodes FF_FS_EXFAT to 0 in components/fatfs/src/ffconf.h
// and exposes no Kconfig option for it, so neither build can read an exFAT
// card. Changing that means vendoring the fatfs component and maintaining the
// patch, which is a fork rather than a setting.
//
// This is why a card over 32 GB, formatted by Windows or a camera, will not
// mount on either build and mountSD() offers to reformat it as FAT instead.

// A short description of the build, for the boot banner - so a log always says
// which variant produced it.
static inline const char *map_build_flavour() {
#if MAP_BUILD_IDF
    return MAP_HAVE_USB_MSC ? "idf+usb" : "idf";
#else
    return "arduino";
#endif
}

// Minimal M5Unified smoke test.
//
// Build with -DMAP_M5_SMOKE_TEST=1 to replace setup()/loop() with nothing but
// M5.begin() and a board report. The point is to take this project's ~3000
// lines out of the question: if a bare M5.begin() also reports a board id
// outside board_t with no panel, the problem is the components or how they are
// configured, not anything here. If it reports 22/22 and brings the panel up,
// the problem is something this project does before or around M5.begin().
//
// This is how the display bug was pinned on the build rather than the sketch.
// It reported 255/127 too - and a board id that the enum cannot hold is the
// signature of a layout mismatch between this file and the components. See
// DISPLAY_IDF_NOTES.md.
//
//   idf.py -DMAP_M5_SMOKE_TEST=1 build flash monitor
#ifndef MAP_M5_SMOKE_TEST
#  define MAP_M5_SMOKE_TEST 0
#endif
