// waypoints.h - saved points, and "get me back there" guidance.
//
// WHAT THIS IS NOT
//
// It is not turn-by-turn. Routing needs a road graph with connectivity,
// one-ways and turn restrictions; the PMTiles archive here holds *rendering*
// geometry - road lines clipped at tile edges, no node identity across those
// edges, no topology at all. Building a router on top would mean re-stitching
// segments per tile and guessing junctions, at which point the answer is
// wrong in exactly the situations where it matters.
//
// So the guidance offered is what an offline device can honestly give: the
// straight-line bearing and distance to the point, drawn as an arrow relative
// to the current course, plus the pin on the map. That is what a handheld GPS
// has always done, and it fails safe - it can be off-road, never misleading
// about a turn.
//
// Storage is a flat file of fixed-size records on whatever storage_fs()
// mounted, rewritten whole on every change. WP_MAX records at ~48 bytes is
// under 2 KB; a partial-update scheme would be more code for no measurable
// gain, and rewriting whole means a torn write costs the list, not the
// interpretation of the list.

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <stdint.h>
#include <stddef.h>
#include "gnss.h"

#ifndef WP_MAX
#define WP_MAX 32
#endif

#define WP_NAME_MAX 24

struct Waypoint {
    double  lat, lon;
    char    name[WP_NAME_MAX];
    int64_t saved_utc;          // 0 when the clock was not set
};

// Read the file. Safe to call before storage is up - the list is then empty.
void wp_begin();

int  wp_count();

// Copy record `i` (0 .. wp_count()-1). False if out of range.
bool wp_get(int i, Waypoint *out);

// Add a point. `name` may be null or empty, in which case one is generated
// from the clock ("14:07") or the index ("pin 7"). Returns the new index, or
// -1 when the list is full or the write failed.
int  wp_add(double lat, double lon, const char *name);

// Drop the current fix. False without a valid fix - saving 0,0 because the
// receiver had not locked yet is the one failure that looks like success.
int  wp_add_fix(const GnssFix &fix);

bool wp_remove(int i);
void wp_clear();

// ---- navigation target -----------------------------------------------------
// At most one at a time: this is a "walk back to the car" feature, and a
// second simultaneous target would need a way to say which arrow is which.

// -1 for none.
int  wp_target();
void wp_set_target(int i);          // out of range clears it
static inline void wp_clear_target() { wp_set_target(-1); }

// Great-circle distance in metres, and initial bearing in degrees true.
// Both return false when there is no target or no fix.
bool wp_target_range(const GnssFix &fix, double *metres, double *bearing_deg);

// Pure geometry, exposed because the overlay wants it too.
double wp_distance_m(double lat1, double lon1, double lat2, double lon2);
double wp_bearing_deg(double lat1, double lon1, double lat2, double lon2);

// "1.4 km NE" / "230 m N", into `out`. Empty string when there is nothing to
// say. Metric only, matching the rest of the UI.
void wp_target_text(const GnssFix &fix, char *out, size_t cap);

#endif // WAYPOINTS_H
