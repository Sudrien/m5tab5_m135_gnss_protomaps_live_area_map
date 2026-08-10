// mapengine.h - tile grid, render worker, and screen composition.
//
// Threading model
// ---------------
//   render worker (core 1, low priority)  owns the PMTiles handle and the SD
//       file. It is the only task that touches either, which makes FatFs
//       thread-safety a non-issue. It writes pixels only into slots marked
//       PENDING.
//
//   UI task (core 0, loop())  reads the GNSS fix, decides when the grid must
//       shift, enqueues jobs, and composites READY slots to the panel.
//
// The one shared structure is the grid's metadata, guarded by a mutex held
// only for microseconds. Pixel buffers need no lock because of a single
// invariant: a PENDING slot is never blitted, and only PENDING slots are
// written. A shift that recycles a buffer mid-render is harmless - the
// worker's generation check fails, the result is dropped, and the slot stays
// PENDING until the replacement job lands.

#ifndef MAPENGINE_H
#define MAPENGINE_H

#include <stdint.h>
#include "gnss.h"

extern "C" {
  #include "tile_grid.h"
}

struct MapStats {
    uint32_t rendered = 0, dropped = 0, notfound = 0, failed = 0;
    uint32_t shifts = 0, queued = 0;

    // How often the coarse overview was actually drawn from, split by what
    // needed it. The overview costs a render of its own every few tile
    // crossings, and whether that is worth paying depends entirely on this
    // ratio - which is not something to guess at once local archives are in
    // play. coarse_gap counts slots the archive could not fill (the case that
    // is really about incomplete data); coarse_wait counts slots merely
    // waiting on the rasteriser.
    uint32_t coarse_renders = 0, coarse_gap = 0, coarse_wait = 0;
    uint32_t last_render_ms = 0;
    uint32_t queue_depth = 0;

    // Compositing cost, to separate "the map is slow to appear" (rendering)
    // from "the map is slow to update" (blitting).
    uint32_t last_draw_ms = 0, max_draw_ms = 0;
    uint32_t draw_total_ms = 0, draws = 0;
};

// Bring up buffers, open the archive, and start the render worker.
// `path` is the .pmtiles file on the mounted card.
bool map_begin(const char *path, uint8_t zoom, int worker_core, int worker_prio);

// Feed a fix. Recentres the grid and enqueues work when the marker leaves the
// centre tile. Safe to call every loop iteration; cheap when nothing changed.
// Tell the engine there is no display.
//
// Everything below that reads M5.Display.width()/height() faults when
// M5.begin() attached no panel - view_follow() is what crashed after the first
// GPS fix. Rather than have four functions here re-derive that, tab5_map.cpp
// states it once after panelBegin().
//
// Headless is not hypothetical on this hardware. The IDF build ran that way
// for a long time - see DISPLAY_IDF_NOTES.md - and a panel can still fail to
// come up on the boot after a sustained flash write, on either build. The rest
// of the program - GNSS, SD, tiles - runs perfectly well without it.
void map_set_headless(bool headless);

void map_update(const GnssFix &fix);

// Composite READY slots and the position marker onto the panel.
void map_draw(const GnssFix &fix);

// Force a full re-render, e.g. after a zoom change.
void map_set_zoom(uint8_t zoom, const GnssFix &fix);

// Pull every tile within `radius` tiles of the current position into the
// cache, at both working zooms, without rendering any of them. Runs in its
// own task and returns immediately.
//
// The grid only ever fetches tiles the device is standing on, so a cache
// warmed by normal use covers about three tiles in each direction - ten
// minutes of walking. This fills a usable area ahead of time, while there is
// still a network to fill it from.
bool map_prefetch_start(int radius, uint8_t z_wide, uint8_t z_close);

// 0 when idle, else 1..100.
// Populate the permanent world floor. Same task machinery as the radius
// prefetch, so only one runs at a time.
bool map_world_floor_start();

int  map_prefetch_progress();
bool map_prefetch_busy();

// Switch palettes. Colours are baked in when a tile is rasterised, so this
// discards every rendered tile and re-renders - roughly a second of coarse
// fallback before the map is sharp again.
void map_set_dark(bool dark);
bool map_is_dark();

// Suppress drawing entirely, for screen-off. The renderer and GNSS keep
// running, so waking is instant and the position stays current.
void map_set_visible(bool visible);

// Force the next map_draw to repaint. Needed after anything that alters the
// screen behind the engine's back.
void map_invalidate();

uint8_t  map_zoom();
void     map_stats(MapStats *out);
bool     map_has_fix_position();   // true once the grid has been centred

#endif // MAPENGINE_H
