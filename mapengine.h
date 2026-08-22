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
#include <stddef.h>
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

    // Compositing cost, split into work done and time merely elapsed.
    //
    // These used to be wall clock, and wall clock turned out to be measuring
    // the wrong thing most of the time. A Wi-Fi scan runs ESP-Hosted tasks at
    // priority 23 against loop() at priority 1, and a measured draw of 2773 ms
    // was 2550 ms of that spent descheduled - so the headline "blit" figure was
    // reporting the scheduler, on a parked device that had rendered nothing.
    // An average of 411 ms against 39 ms of actual work is not a number anyone
    // can act on, and it is the number the status bar puts on the panel.
    //
    // So the fields that get read as "how expensive is compositing" now carry
    // work: the task's own run time across the draw, which is what changing
    // blit_region could affect. They are named _work_ so that nothing silently
    // keeps its old meaning under the same identifier - a rename the compiler
    // enforces, rather than a redefinition a reader has to notice.
    //
    // Wall and stall are kept alongside, because the gap between them is the
    // preemption problem and it is worth watching in its own right. wall is
    // never less than work; stall is the difference.
    // Windowed, not since boot - see map_stats_reset_window().
    //
    // Cumulative was actively misleading here. On a three-minute parked log
    // the max froze at a single event 56 s in and never moved again, while
    // the average slid 30 -> 21 -> 16 -> 13 -> 11 -> 9 -> 8 -> 7 -> 6 ms as
    // marker-rect early-outs at 0 ms diluted the mean. By the end the line
    // read "avg 6 max 156" and neither number described anything that was
    // happening. A drive would be worse: one grid shift sets the max, and
    // everything after averages into a flatter and flatter mean, so twenty
    // minutes of driving yields one usable data point and no idea whether a
    // scan froze the UI once or forty times.
    //
    // These are therefore cleared each time the stats line prints, so every
    // interval describes that interval.
    uint32_t last_draw_work_ms = 0, max_draw_work_ms = 0;
    uint32_t draw_work_total_ms = 0;
    uint32_t last_draw_wall_ms = 0, max_draw_wall_ms = 0;
    uint32_t draw_wall_total_ms = 0;
    uint32_t max_stall_ms = 0;
    uint32_t draws = 0;

    // Since boot, never cleared. The window says what is happening now; these
    // say whether the worst thing seen so far has been seen again. The status
    // bar reads the peak rather than the window, so a number on the panel does
    // not silently reset itself every fifteen seconds while being looked at.
    uint32_t peak_draw_work_ms = 0, peak_stall_ms = 0;

    // False when the run-time counter is unavailable at build time, in which
    // case work is a copy of wall and stall is always zero. The stats line says
    // so rather than presenting the copy as a measurement.
    bool     cpu_time_valid = false;
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

// ---- panning ---------------------------------------------------------------
// Move the view one band width - one third of the visible area - in the given
// direction. dx and dy are each -1, 0 or +1, in screen terms: dx +1 is east,
// dy +1 is south.
//
// This is deliberately the same operation the follow logic performs when the
// marker leaves the band, applied to a synthetic anchor instead of the fix.
// The consequences are identical to having walked a third of a screen that
// way: the grid shifts when the anchor crosses a tile, the coarse overview is
// refreshed, and the same jobs are enqueued. Nothing here knows it is being
// driven by a finger rather than by movement, which is why there is no second
// code path to keep in step with the first.
//
// The marker keeps tracking the real fix throughout and simply leaves the
// screen when the view is panned away from it. That is the honest rendering:
// the map is showing somewhere the device is not.
//
// Returns false when there is nothing to pan (no fix has ever centred the
// grid, or headless).
bool map_pan_step(int dx, int dy);

// Back to following the fix. The next map_update() recentres.
void map_pan_reset();

// True while the view is somewhere other than where the device is. The footer
// uses this to light the recentre control, and it is worth surfacing rather
// than inferring: a panned view that nobody remembers panning is the failure
// mode this whole feature has.
bool map_panning();

// ---- boot seeding ----------------------------------------------------------
// Centre the grid on a remembered position so there is a map on screen during
// the thirty to ninety seconds a cold start takes, rather than a background
// colour and a status line.
//
// Explicitly does NOT place a marker. The remembered position is where the
// device was when it was last switched off, which may be a different city; a
// marker there would be indistinguishable from a live one and wrong by the
// whole distance travelled since. The marker appears when the receiver
// produces a fix, or when the Wi-Fi centroids produce an estimate - either
// way, when something has actually measured a position.
//
// Ignored once a real position has arrived, so it is safe to call late.
void map_seed_position(double lat, double lon);

// Whether the grid has an anchor at all, and whether anything drawable has
// landed in it yet. setup() orders the slow parts of bring-up behind the
// second of these; see the comments there.
// Advance the grid without painting: the placeholder fill that turns pending
// slots into drawable ones, and nothing else. For callers that are showing
// something other than the map (the boot backdrop) and must not have it
// painted over before there is a map to replace it with.
void map_pump();

// Drawable slots, and whether every slot is drawable. The boot handover waits
// on the second: one slot filled from the overview satisfies
// map_has_picture() and looks like a patch of map floating on an empty band.
int  map_picture_slots();

// True once no slot is still rendering - which is not the same as every slot
// being drawable, since a slot outside the archive's coverage never becomes
// drawable at all.
bool map_picture_complete();

// Whether the overview may stand in for a slot that is merely still
// rendering. Gaps in the data are always filled regardless. Off during boot,
// where the boot screen is already showing something better than a blurry
// z12 and the overview render is competing with the tiles it stands in for.
void map_set_coarse_fill_pending(bool on);

bool map_has_anchor();
bool map_has_picture();

// ---- world check -----------------------------------------------------------
// Render z0/0/0 - the whole earth in one tile - as an end-to-end proof that
// the archive on the card is readable, before anything touches the radio.
//
// map_begin() succeeding only proves the header parsed. This exercises the
// whole path against the one tile any complete archive must contain, so a
// failure here is unambiguously the card and not the network.
//
// Start it, poll the state (TILE_PENDING until the worker answers, then
// TILE_READY, TILE_NODATA or TILE_ERROR), optionally draw it centred at
// (cx, cy) at `size` pixels, then free the buffer.
void         map_world_check_start();
tile_state_t map_world_check_state();
bool         map_world_check_draw(int cx, int cy, int size);
void         map_world_check_free();

// Hold the render worker off the SD and PSRAM buses, and let it go again.
//
// For the few seconds of something that needs those buses on a deadline -
// SDIO enumeration of the Wi-Fi co-processor is the case this exists for,
// where a busy worker turned 47 ms of card init into 4.8 s.
//
// Cooperative: the worker parks between jobs, where it holds no lock and has
// no read open. pause() blocks until it has, or gives up after a few seconds
// and says so. It must not be done by suspending the task from outside -
// slots 0 and 1 share one sdmmc driver, and a worker frozen mid-read holds
// its lock forever, which hangs enumeration and the whole boot with it.
//
// The queue is untouched; work resumes where it stopped. Calls do not nest.
void map_worker_pause();
void map_worker_resume();

// False until a measured position has been seen this session. The status bar
// uses it to explain why a perfectly good map has nothing on it.
bool map_marker_valid();

// Pull every tile within `radius` tiles of the current position into the
// cache, at both working zooms, without rendering any of them. Runs in its
// own task and returns immediately.
//
// The grid only ever fetches tiles the device is standing on, so a cache
// warmed by normal use covers about three tiles in each direction - ten
// minutes of walking. This fills a usable area ahead of time, while there is
// still a network to fill it from.
bool map_prefetch_start(int radius, uint8_t z_wide, uint8_t z_close);

// How many tiles of that same square are not already held by a local archive.
// Zero means the whole area is offline already and map_prefetch_start() will
// decline. Cheap enough for the footer to call each repaint: header fields
// only, no directory reads.
int  map_prefetch_pending(int radius, uint8_t z_wide, uint8_t z_close);

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
// Tell the engine whether the map is on screen.
//
// False does not stop the render worker: it stops it *drawing*. Tiles are
// still fetched into the cache on the card, because that is the half that
// depends on a network which may be gone later, while the pixels can always
// be recomputed from bytes already stored. Slots fetched this way keep their
// id and stay PENDING.
//
// True re-queues every slot that has no pixels, so the map comes back from
// local cache rather than waiting for the grid to shift.
void map_set_visible(bool visible);

// Force the next map_draw to repaint. Needed after anything that alters the
// screen behind the engine's back.
void map_invalidate();

// POI and place labels. Drawn as an overlay at composite time, so this is a
// repaint rather than a re-render - unlike map_set_dark(), it is instant.
void map_set_labels(bool on);
bool map_labels_on();

// Saved-point overlay: the pins themselves, and the bearing line to whichever
// one is the current navigation target. Same cost as labels - drawn at
// composite time, so this is a repaint rather than a re-render.
//
// Note that a target set makes every marker move a full repaint: the guide
// line sweeps far outside MARKER_CLEAR_R, so the partial path cannot put back
// what it erased.
void map_set_pins(bool on);
bool map_pins_on();

// Roughly where the marker is, as "Locality, Region" - whichever of the two
// is known. False, and an empty string, until a named place turns up.
//
// Nearest-centroid, not point-in-polygon: near a boundary it can name the
// neighbour. See REGION_ZOOM in mapengine.cpp.
bool map_place_text(char *out, size_t cap);

uint8_t  map_zoom();
void     map_stats(MapStats *out);

// Clear the windowed draw counters. Call right after reporting them.
//
// UI-task only, and safe without the grid lock for that reason: every field it
// touches is written solely by map_draw(), which runs on the same task. The
// worker's fields - rendered, failed, last_render_ms - are deliberately not
// reset here, both because they are cumulative counts worth keeping and
// because clearing them from this task would be a genuine race.
void     map_stats_reset_window();
bool     map_has_fix_position();   // true once the grid has been centred

#endif // MAPENGINE_H
