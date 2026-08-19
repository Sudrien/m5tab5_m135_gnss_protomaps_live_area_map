// gpstrust.h - consistency checks on the GNSS solution.
//
// WHAT THIS IS FOR
// A GNSS receiver believes whatever reaches its antenna. A transmitter close
// by, or a recording of one replayed later, produces a solution the receiver
// reports with full confidence - correct checksums, plausible HDOP, a healthy
// satellite count. Nothing in the NMEA stream says "this is not real".
//
// What a transmitter cannot easily do is stay consistent with everything else
// the device knows. It has to lie about position, velocity and time at once,
// keep the lie continuous with where the device actually was a second ago, and
// do it without contradicting a clock, a radio environment and a magnetometer
// it has no control over. Each check below is one of those contradictions.
//
// WHAT THIS IS NOT
// Not detection, and it does not refuse anything. Every check here has an
// innocent explanation that is far more common than an attack: a tunnel exit
// looks like a position jump, a cold reacquisition looks like a clock step, a
// multipath-riddled urban canyon looks like velocity nonsense. The output is a
// confidence level for the status bar, and the map keeps drawing regardless.
//
// A device that stopped working because it was suspicious would be worse than
// one that was quietly lied to - the first fails every time you drive under a
// bridge, the second fails only when someone is actually attacking you.

#ifndef GPSTRUST_H
#define GPSTRUST_H

#include <stdint.h>
#include "gnss.h"

enum TrustLevel {
    TRUST_UNKNOWN = 0,   // nothing to say yet - no fix, or too few samples
    TRUST_OK,            // every check that could run, passed
    TRUST_ODD,           // one check failed, and one check fails all the time
    TRUST_BAD,           // several at once, which is the interesting case
};

// Individual flags, kept separate from the level so the log can say which.
// A single flag is a bridge; the combination is what matters.
enum {
    TRUST_F_JUMP    = 1 << 0,  // position moved faster than physics allows
    TRUST_F_SPEED   = 1 << 1,  // Doppler speed disagrees with position delta
    TRUST_F_CLOCK   = 1 << 2,  // GNSS time disagrees with the RTC
    TRUST_F_SNR     = 1 << 3,  // satellite SNRs implausibly uniform
    TRUST_F_PPS     = 1 << 4,  // pulse-per-second interval is not 1 Hz
    TRUST_F_WIFI    = 1 << 5,  // Wi-Fi centroids put us somewhere else
    TRUST_F_ALT     = 1 << 6,  // altitude impossible or frozen
};

// Feed every fix. Cheap; keeps its own history.
void gpstrust_update(const GnssFix &fix);

TrustLevel gpstrust_level();
uint32_t   gpstrust_flags();

// One short line naming what is wrong, for the status bar. Empty string when
// there is nothing to say.
const char *gpstrust_text();

// Reset the history - after a deliberate cold start, or anything else that
// makes a discontinuity expected rather than suspicious.
void gpstrust_reset();

#endif // GPSTRUST_H
