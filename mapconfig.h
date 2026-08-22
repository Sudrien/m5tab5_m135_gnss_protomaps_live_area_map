// mapconfig.h - tunables shared across every translation unit.
//
// These cannot live in the .ino: Arduino compiles each .cpp separately, so a
// #define there never reaches tile_grid.c or mapengine.cpp.

#ifndef MAPCONFIG_H
#define MAPCONFIG_H

// ---- render resolution -----------------------------------------------------
// Pixels per subtile. Vector geometry has no inherent resolution, so this is
// purely a quality/memory trade: the same tile drawn larger is sharper, up to
// the point where the source simplification becomes visible.
//
// At z14 and latitude ~42:
//
//   px     m/px   screen span   grid RAM (10 buffers)
//   512    3.53      4520 m       5.0 MB
//   768    2.35      3010 m      11.2 MB
//   1024   1.77      2260 m      20.0 MB
//
// 1024 leaves roughly 3 MB spare of the ~30 MB PSRAM available after wifi
// comes up. If allocation fails at startup, drop to 768.
#ifndef SUBTILE_PX
#define SUBTILE_PX 1280
#endif

// ---- view following --------------------------------------------------------
// The marker is kept inside a band in the middle of the screen rather than
// pinned to the exact centre. Inside the band the view does not move at all,
// so GPS noise moves the dot instead of shuffling the whole map - which at
// HDOP 1.5 and 1.4 m/px is several pixels of jitter every fix.
//
// When the marker reaches the edge of the band the view follows exactly far
// enough to hold it there, so there is no lurch and nothing to tune about
// how it re-centres.
//
// Fraction of the visible area the band spans. 0.33 is the literal rule of
// thirds; smaller means the map moves more often but the marker stays more
// central.
#ifndef MARKER_BAND
#define MARKER_BAND 0.33f
#endif

// Radius reserved for the marker when repainting just its neighbourhood,
// large enough to cover the circle and the heading needle.
#ifndef MARKER_CLEAR_R
#define MARKER_CLEAR_R 36
#endif

// Radius the target guide is drawn within, and the radius reserved for it
// when repainting.
//
// The guide used to be a line all the way to the pin, which meant the marker
// repaint could not put back what it erased, which meant every marker move
// took the full-screen path - a whole-screen blit at up to 15 Hz, which is
// what the flashing was. Bounding the guide to a fixed radius puts it back
// inside the partial path: a hundred-odd pixels of blit per move instead of
// a million.
//
// The direction is all a bearing can honestly convey anyway; the distance is
// in the status bar, where a number belongs.
#ifndef GUIDE_R
#define GUIDE_R 74
#endif
#ifndef GUIDE_CLEAR_R
#define GUIDE_CLEAR_R (GUIDE_R + 18)
#endif

// ---- grid size -------------------------------------------------------------
// Tiles held resident, GRID_N x GRID_N. May be even.
//
// At 1280px a tile spans the full screen width, so a 1280x720 window touches
// at most 2x2 tiles however it is positioned - which makes a 2x2 grid enough
// to cover the screen with a full tile of margin in every direction. That is
// 5 buffers including the spare, 16.4 MB, against 32.8 MB for a 3x3.
//
// Even grids have no middle tile; the grid anchors on the junction where the
// four meet instead. tile_grid handles both.
#ifndef GRID_N
#define GRID_N 2
#endif

// Overview tile used to fill slots before their real render lands. Kept at
// 512 regardless of subtile size - it is a placeholder that survives on
// screen for under a second, and at 1024 it would cost another 1.5 MB for
// no lasting benefit.
#ifndef COARSE_PX
#define COARSE_PX 512
#endif

// ---- working zoom levels ---------------------------------------------------
// The grid switches between these two by speed. Setting both to the same
// value pins the map to one level, which is worth doing while judging
// fidelity: a level that only appears above 25 km/h is hard to evaluate.
//
// At SUBTILE_PX 1024, latitude ~42:
//
//   z    m/px   screen span   ground held by the 3x3 grid   overview span
//   13   3.53      4520 m            10.9 km                  28.9 km
//   14   1.77      2260 m             5.4 km                  14.5 km
//   15   0.88      1130 m             2.7 km                   7.2 km
//
// Zoom costs no memory - the buffers are the same size either way - so this
// trades sharpness against how far you can travel before leaving resident
// tiles. z15 is unusable in a car for that reason: 2.7 km is under two
// minutes at speed. z13 holds nearly 11 km and needs a quarter as many
// tiles cached per unit area as z14.
//
// Holding a single level also quarters the cache and prefetch time again,
// since each extra level is four times as many tiles.
#ifndef Z_FLOOR
#define Z_FLOOR 14
#endif

// ---- data zoom vs display zoom ---------------------------------------------
// How many zoom levels below Z_FLOOR the actual tile data comes from. Each
// data tile is then drawn as a 2^SPLIT x 2^SPLIT block of subtiles, one
// quadrant per subtile.
//
//   0   data at Z_FLOOR      1 tile  -> 1 subtile
//   1   data at Z_FLOOR-1    1 tile  -> 4 subtiles   (quarter the download)
//   2   data at Z_FLOOR-2    1 tile  -> 16 subtiles  (a sixteenth)
//
// Pixel scale is set by Z_FLOOR and SUBTILE_PX and does not change with this
// - only how much has to be fetched and stored. Verified exact: rendering a
// tile as quadrants reproduces a full render pixel for pixel.
//
// The cost is fidelity. Tile geometry is simplified for the zoom it was cut
// at, so at SPLIT 1 the z13 data is drawn at twice its intended density and
// small features it dropped are simply absent. SPLIT 2 is four times over
// and looks it.
// 0 here: z13 carries roughly a fifth of the buildings per unit area that
// z14 does - suburban housing is dropped almost entirely - and that showed
// immediately on screen. The four-fold saving in cache and bandwidth is not
// worth a map missing most of its houses.
#ifndef SUBTILE_SPLIT
#define SUBTILE_SPLIT 0
#endif

#define DATA_ZOOM_OF(z)  ((z) - SUBTILE_SPLIT)
#define SPLIT_N          (1 << SUBTILE_SPLIT)
#define SPLIT_MASK       (SPLIT_N - 1)

// Slow and fast working levels. Both at Z_FLOOR = one level, no switching.
#ifndef Z_LEVEL_CLOSE
#define Z_LEVEL_CLOSE Z_FLOOR
#endif
#ifndef Z_LEVEL_WIDE
#define Z_LEVEL_WIDE  Z_FLOOR
#endif

// Cache index entries, 16 bytes each. 80k covers a large regional cache and
// costs 1.3 MB; raising it competes directly with the render buffers.
#ifndef CACHE_MAX_ENTRIES_CFG
#define CACHE_MAX_ENTRIES_CFG 80000
#endif

// Deepest zoom to hold as a permanent offline floor. z0-6 is 5461 tiles and
// guarantees the map draws *something* anywhere on earth, which is what stops
// a drive out of cached territory ending in a white screen.
#ifndef WORLD_FLOOR_ZOOM
#define WORLD_FLOOR_ZOOM 6
#endif

#endif // MAPCONFIG_H
