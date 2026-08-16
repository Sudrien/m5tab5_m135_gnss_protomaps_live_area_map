// compass.h - magnetometer heading from the M135's BMM150.
//
// WHY THIS EXISTS
// The dial in the footer was drawn from GNSS course over ground, which is
// derived from successive fixes and therefore does not exist while the device
// is stationary. A parked receiver reports an empty course field, which parses
// to zero - so the choice was a needle that vanishes at rest or one that lies.
// A magnetometer measures the field itself and works standing still, which is
// what a compass is for.
//
// WHAT IT MEASURES
// Where the *device* is pointing, not where it is going. Those are the same
// thing in a car and quite different on foot, and the map is north-up either
// way, so this answers "which way am I facing relative to what is drawn".
//
// HARDWARE PATH
// The BMM150 is not on the internal I2C bus and never appears in a scan. It
// hangs off the M135 BMI270's auxiliary I2C master, so reaching it means:
//
//   1. EXT_5V_BUS on, or the module is unpowered and the bus scan is clean
//      and empty - indistinguishable from a module that is not seated.
//   2. The BMI270 at 0x69 initialised, including its 8 KB configuration
//      upload. Without it the part answers its chip ID and ignores everything
//      else, the aux interface included.
//   3. The aux master configured, and the BMM150 brought out of the suspend
//      mode it boots into - before which it does not even answer a chip-ID
//      probe.
//
// A failure at any of those looks identical from here: no magnetometer. The
// log lines distinguish them, which is why each step has one.
//
// BUS OWNERSHIP
// M5Unified owns the internal I2C bus in this project, so unlike the
// standalone example this is ported from, the Bosch drivers are wired to
// M5.In_I2C rather than to a raw i2c_master handle. Two owners of one bus is
// not a thing that half-works; see compass.cpp.
//
// OPTIONAL AT COMPILE TIME
// Everything here degrades to "no compass" when the Bosch drivers are absent,
// so a tree without them builds and runs. See tools/fetch_bosch_drivers.sh.

#ifndef COMPASS_H
#define COMPASS_H

#include <stdint.h>

// Was this build compiled with the Bosch drivers present? Compile-time; says
// nothing about whether the hardware answered.
bool compass_supported();

// Bring up EXT_5V, the BMI270 and the BMM150 behind it. Safe to call when
// unsupported (returns false quietly). Must run after M5.begin(), because
// Power_Class::begin() rewrites the expander direction register this touches.
//
// Call from the task that owns the internal I2C bus - loop()'s task. Every
// read afterwards must come from that same task.
bool compass_begin();

// Did bring-up succeed? False means every heading below is unavailable.
bool compass_ok();

// Sample the magnetometer and the module's accelerometer, and update the
// cached heading. Rate-limited internally, so calling it every loop is fine.
// Must be called from the same task as compass_begin().
void compass_update();

// Heading in degrees, 0 = true north (or magnetic north when declination is
// left at zero), increasing clockwise. Negative when no heading is available.
float compass_heading();

// Field magnitude in microtesla, for judging whether a heading is worth
// trusting. Should sit near the local field - 25 to 65 uT by latitude - and
// hold roughly constant as the device turns.
float compass_field_ut();

// Sixteen-point label for a heading: "N", "NNE", ... Returns "--" for a
// negative (unavailable) heading.
const char *compass_label(float deg);

// True north correction, degrees to add to the magnetic reading. Set from the
// current position so the needle agrees with the map, which is drawn in true
// north. Harmless to call often; recomputed only when the position has moved
// far enough to matter.
void compass_set_position(double lat, double lon);

// ---- calibration ----------------------------------------------------------
//
// The BMM150 sits centimetres from the Tab5's speaker magnet, which adds a
// fixed vector in sensor frame - comparable in size to the earth's field. Left
// uncorrected the heading can be wrong by a quarter turn and |B| swings wildly
// as the device rotates. The correction is per-board and has to be measured.
//
// Tumble the device slowly through as many orientations as possible; the
// offset is the centre of the sphere the samples trace out.
void compass_calibrate_start();
void compass_calibrate_cancel();
bool compass_calibrating();

// 0..100, how far through the calibration run. Progress is by coverage, not
// by time: a device left sitting still does not advance.
int  compass_calibrate_progress();

// A one-line description of calibration state for the UI, e.g.
// "turn the device over" or "calibrated, |B| 48 uT". Never null.
const char *compass_status();

#endif // COMPASS_H
