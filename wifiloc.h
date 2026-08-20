// wifiloc.h - self-built Wi-Fi centroid database, for a position without sky.
//
// WHAT THIS IS
// Every access point the device hears while it has a good GNSS fix gets its
// observation folded into a running centroid: where, on average, this device
// was when it could hear that AP. Do that over enough travel and the database
// becomes a coarse map of the radio environment along the routes actually
// taken. When GNSS is unavailable - a garage, a tunnel approach, between tall
// buildings - a scan can be matched against it and the heard APs' centroids
// averaged into an estimate.
//
// WHAT IT IS NOT
// This is not a fix. The centroids are not AP positions: they are centroids of
// where *this device* stood, so they are biased onto the roads and paths it
// travels, and an AP heard only once from one direction has a centroid tens of
// metres from the transmitter. Nothing here is surveyed, calibrated, or
// checked against an external database, and the accuracy figure returned below
// is a spread estimate rather than a confidence interval.
//
// The map treats a position from here as 2D-quality at best - see the way
// tab5_map.cpp constructs the view fix - so nothing that requires a real fix
// (waypoint saving at full precision, the magnetometer log, the fine zoom
// gate) accepts one of these by accident.
//
// WHY CENTROIDS RATHER THAN TRILATERATION
// Turning RSSI into a distance needs a path loss exponent and each AP's
// transmit power, and neither is known. The exponent sits in an exponent, so
// a 6 dB error - a body between device and AP - is a factor of two in the
// derived distance. A weighted centroid makes no distance claim at all: it
// takes the received power as a weight and nothing more. With reference points
// that are themselves biased onto the same roads, the shared bias partly
// cancels in an average and does not cancel in a circle intersection.
//
// THE FILE
// /wifiloc.csv, plain text, one header line and one row per access point:
//
//   bssid,obs,lat,lon,min_lat,max_lat,min_lon,max_lon,best_rssi,mobile
//
// lat/lon are the centroid; the file carries the mean and the observation
// count, and the running sums the arithmetic uses are reconstructed on load.
//
// BSSIDs are stored as they are heard. An earlier version hashed them, on the
// reasoning that a lost card should not yield a usable list of networks - but
// /maglog.csv beside it is a plaintext per-second track log with lat, lon and
// UTC, /waypoints.bin holds named places, and /wifi.bin holds the credential
// for the home network. The hash protected nothing the volume did not already
// disclose, and it made the top five address bytes unavailable, which is what
// distinguishes several virtual BSSIDs on one radio from several real APs.
//
// So: everything on this card is a record of where the device has been, and
// this file is no different. If that matters, encrypt the volume or delete
// the files; obfuscating one of them was the wrong layer for it.

#ifndef WIFILOC_H
#define WIFILOC_H

#include <stdint.h>
#include <stddef.h>
#include "gnss.h"

// Load the database from the card and allocate the in-memory table in PSRAM.
// Safe to call with no storage or no PSRAM: it says so and every call below
// becomes a no-op. Call after storage is mounted and after the radio is up.
void wifiloc_begin();

// Drive the scan state machine. Learns while the fix is good, and attempts an
// estimate while it is not. Non-blocking: scans are asynchronous and this
// polls them. Call every loop from loop()'s task.
void wifiloc_poll(const GnssFix &fix);

// The most recent estimate, if there is one that is still fresh.
//
// `acc_m` is a rough spread of the contributing centroids, not a covariance -
// it says how much the APs being heard disagree about where they are, which is
// the best available signal for whether to believe the answer at all.
// `age_ms` is how long ago the scan behind it completed.
bool wifiloc_position(double *lat, double *lon, float *acc_m, uint32_t *age_ms);

// How many APs the last estimate was built from. Zero when there is no
// estimate. Shown in the status bar, because three is the floor and the
// difference between three and fifteen is the difference between a guess and
// something worth steering by.
int wifiloc_used();

// Database size, and how many records have changed since the last write.
uint32_t wifiloc_entries();
uint32_t wifiloc_dirty();

// Write the database if enough has changed or enough time has passed. This is
// what the idle path calls; wifiloc_flush() below is the unconditional form.
void wifiloc_flush_if_due();

// Write the database to the card. Called from the same idle and power-button
// paths the tile cache and the magnetometer log use.
void wifiloc_flush();

// Is there a table at all? False when PSRAM allocation failed or no storage
// mounted.
bool wifiloc_available();

// Runtime switch, for turning the whole thing off without reflashing. Learning
// and locating are both suppressed when off; the database is kept.
bool wifiloc_enabled();
void wifiloc_set_enabled(bool on);

#endif // WIFILOC_H
