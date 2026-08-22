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
#include <stddef.h>     // size_t, for the calibration blob accessors below

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

// A single synchronised reading, for the log. Everything here comes from the
// same pass of compass_update(), which matters: pairing a magnetometer vector
// with an accelerometer vector read a moment later is how a tilt-compensated
// heading acquires an error that only shows up while the device is moving -
// which is exactly the condition being modelled.
//
// Both the raw and the corrected magnetometer vectors are carried. The raw one
// is what lets a fit be redone offline against different calibration
// parameters without collecting another drive; the corrected one is what the
// heading in this same struct was actually computed from.
struct CompassSample {
    uint32_t ms;              // millis() when the sample was taken
    uint32_t seq;             // increments once per accepted sample
    float    raw[3];          // magnetometer, uncorrected, sensor frame
    float    corrected[3];    // offset and gain applied, accelerometer frame
    float    acc[3];          // accelerometer, raw counts, same instant
    float    heading;         // degrees, -1 when uncalibrated
    float    field_ut;
    float    roll, pitch;     // degrees
    bool     calibrated;
};

// Copy the most recent sample. False when the compass is down, is calibrating,
// or has not produced one yet. Call from the same task as compass_update().
bool compass_sample(CompassSample *out);

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
// Device attitude in degrees, from the same accelerometer read the heading
// uses. Roll is rotation about the screen's long axis, pitch about the short
// one; both are zero with the screen face up. Meaningless when compass_ok()
// is false.
float compass_roll();
float compass_pitch();

// millis() when the device was last being handled, or 0 if it has not been
// since boot. Derived from the accelerometer alone - it needs no calibration
// and does not care whether the magnetometer answered, which is the point:
// it is available in exactly the cases where the heading is not.
//
// "Handled" means the attitude changed, not that the device is in motion. A
// device riding in a moving car sits at a constant attitude and will not
// register here; a stationary one being picked up will. Anything wanting
// motion in the travelling sense should ask GNSS for speed instead.
uint32_t compass_last_motion_ms();

// Largest acceleration departure seen since this was last called, in the same
// raw counts the detector works in, and cleared by the call. Reporting only:
// nothing decides anything from it. It exists because the detection threshold
// is the one number here that has to be set against a particular board in a
// particular mount, and it cannot be set against figures nobody can see.
//
// 16384 counts per g at the configured 2 g range, if a conversion is wanted.
float compass_motion_peak_take();

// millis() when the device was last disturbed at all, at a far lower bar than
// compass_last_motion_ms() uses, or 0 for never. Not a substitute for it: a
// stir is not enough to conclude anyone is present, only enough to conclude
// the device is not sitting undisturbed on a hard surface.
//
// The use is corroboration. A receiver indoors wanders its *position*, not
// just its speed, by tens of metres with a perfectly good HDOP, and no
// threshold on the GNSS side separates that from slow travel. Real travel is
// never silent here, so silence here is grounds to disbelieve it.
uint32_t compass_last_stir_ms();

void compass_calibrate_start();

// Restart calibration keeping the accumulated fit, so a second sweep improves
// on the first rather than replacing it. Falls back to a fresh start if
// nothing has been calibrated yet.
void compass_calibrate_start_refine();
void compass_calibrate_cancel();

// Opaque calibration blob for storage: compass.cpp owns the format, the
// caller owns the file. export returns 0 when there is nothing worth saving.
// import validates before applying and returns false on a stale or corrupt
// blob, leaving the current calibration untouched.
size_t compass_cal_blob_size();
size_t compass_cal_export(void *dst, size_t cap);
bool   compass_cal_import(const void *src, size_t len);

// Set when a calibration completes and worth writing to the card; the caller
// clears it once stored.
bool   compass_cal_dirty();
void   compass_cal_clear_dirty();
bool compass_calibrating();

// 0..100, how far through the calibration run. Progress is by coverage, not
// by time: a device left sitting still does not advance.
int  compass_calibrate_progress();

// A one-line description of calibration state for the UI, e.g.
// "turn the device over" or "calibrated, |B| 48 uT". Never null.
const char *compass_status();

#endif // COMPASS_H
