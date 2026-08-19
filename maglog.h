// maglog.h - CSV log of magnetometer against GNSS course, for later modelling.
//
// WHY
// The magnetometer answers "which way is the device pointing" and GNSS course
// answers "which way is it travelling". Those are the same thing only when the
// device is fixed to something moving in the direction it faces, and only when
// it is moving fast enough for course over ground to exist at all. The dial
// that used to show both is gone; what is left is the interesting part - the
// residual between the two - and that is a thing to model offline, not to read
// on a 110 px circle in a moving vehicle.
//
// WHAT IS WRITTEN
// One row per second, only while the GNSS course is trustworthy (see
// MAGLOG_MIN_KMH and the mode/HDOP gates in maglog.cpp), to /maglog.csv on
// whatever storage_fs() mounted. Both the raw and the calibration-corrected
// magnetometer vectors are written, along with the accelerometer, so a fit can
// be redone offline against different calibration parameters without needing
// another drive.
//
// WHAT IS NOT
// Nothing is written while the compass is calibrating (the offsets are
// half-measured), while the receiver is searching, or below walking pace -
// a parked receiver reports an empty course field, which parses to zero, so
// logging it would fill the file with confident southbound zeroes.

#ifndef MAGLOG_H
#define MAGLOG_H

#include <stdint.h>
#include "gnss.h"

// Open (or create) the log and write the header if it is new. Safe to call
// with no card: it fails quietly and every poll below becomes a no-op.
// Call after storage is mounted and after compass_begin().
void maglog_begin();

// Offer a fix. Gated and rate-limited internally, so calling it every loop is
// fine. Must be called from the task that owns compass_update() - it reads the
// sample compass.cpp last captured.
void maglog_poll(const GnssFix &fix);

// Runtime toggle, for the footer button. Enabled by default when the log
// opened successfully.
bool maglog_enabled();
void maglog_set_enabled(bool on);

// Is there somewhere to write at all? False when no filesystem mounted or the
// file could not be opened.
bool maglog_available();

// Rows written this session, and the total file size in bytes at the last
// flush. For the button label and the periodic serial line.
uint32_t maglog_rows();
uint32_t maglog_bytes();

// Push anything buffered if the flush interval has elapsed. Called from the
// idle path; maglog_flush() below is the unconditional form.
void maglog_flush_if_due();

// Push anything buffered to the card. Called on the same idle/power-button
// paths the tile cache uses, so a power cut loses seconds rather than minutes.
void maglog_flush();

#endif // MAGLOG_H
