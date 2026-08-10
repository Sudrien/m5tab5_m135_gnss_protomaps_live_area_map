// tile_grid.h - a 3x3 window of rasterised subtiles that follows a position.
//
// The grid holds GRID_N^2 slots. Slot pixel buffers are allocated once and
// never freed; a shift *rotates ownership* of those buffers between slots, so
// the allocation count is constant for the life of the program.
//
// Invariants (asserted by the test harness):
//   1. The set of pixel buffer pointers across all slots is constant and has
//      no duplicates, before and after any shift.
//   2. slots[i].id always equals tile_at(center, row-1, col-1).
//   3. Any slot whose id is unchanged by a shift keeps its pixels and its
//      READY state; every other slot becomes PENDING.
//   4. generation strictly increases on every shift and on every zoom change.
//
// Threading: the render worker writes pixels and flips PENDING->READY; the UI
// task reads. Take `lock` for any access to slots[], center or generation.
// The worker must re-check `generation` before committing, since a shift may
// have recycled the buffer it was handed.

#ifndef TILE_GRID_H
#define TILE_GRID_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "mapconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

// GRID_N comes from mapconfig.h, and may be even.
//
// An odd grid has a middle tile and the marker sits inside it. An even grid
// has no middle tile - it has a junction where the tiles meet - so the anchor
// is that junction instead, and the grid is described by its top-left tile
// rather than its centre.
//
// Both cases reduce to the same rule: the marker should stay within half a
// tile of the canvas centre, which sits at origin + GRID_N/2 in tile units.
// For a 3x3 that is the middle tile exactly, matching the old behaviour.
#define GRID_COUNT  (GRID_N * GRID_N)
// SUBTILE_PX comes from mapconfig.h so every file agrees on it.

typedef struct { uint8_t z; int32_t x, y; } tile_id_t;

typedef enum {
    TILE_EMPTY = 0,
    TILE_PENDING,   // queued, buffer holds nothing worth showing
    TILE_COARSE,    // upscaled from a lower zoom: approximate but drawable
    TILE_READY,     // rendered at this zoom
    TILE_NODATA,
    TILE_ERROR
} tile_state_t;

// Drawable states. TILE_COARSE exists so a slot can show *something* the
// instant it is created, rather than leaving a hole for the several hundred
// milliseconds a real render takes. Cold start and zoom changes invalidate
// all nine slots at once, which is where the difference is most visible.
static inline int tile_drawable(tile_state_t s) {
    return s == TILE_COARSE || s == TILE_READY;
}

typedef struct {
    tile_id_t    id;
    tile_state_t state;
    uint16_t    *pixels;      // RGB565, SUBTILE_PX^2
    uint32_t     generation;  // generation this slot's id was assigned in
} subtile_t;

typedef struct {
    subtile_t slots[GRID_COUNT];   // index = row*GRID_N + col; row 0 = north
    tile_id_t origin;              // top-left tile, == slots[0].id
    uint32_t  generation;
    int       initialised;
} tile_grid_t;

// The tile whose top-left corner the grid should be anchored at, for a
// marker at fractional tile position (fx, fy). Centres the canvas on the
// marker to within half a tile, for odd and even GRID_N alike.
static inline int32_t grid_origin_for(double f) {
    return (int32_t)floor(f - (double)GRID_N / 2.0 + 0.5);
}

// A render request handed to the worker queue.
typedef struct {
    tile_id_t id;
    uint8_t   slot;
    uint32_t  generation;
} render_job_t;

// Assign the (already allocated) pixel buffers and centre the grid.
// `bufs` must contain GRID_COUNT distinct pointers, each SUBTILE_PX^2 uint16.
void grid_init(tile_grid_t *g, uint16_t *const *bufs, tile_id_t center);

// The tile at (drow, dcol) from the grid origin. X wraps around the world,
// Y is clamped by the caller via grid_id_valid().
tile_id_t grid_tile_at(tile_id_t origin, int drow, int dcol);
int       grid_id_valid(tile_id_t id);

// Shift the window by (dx, dcol east-positive) / (dy, drow south-positive).
// Handles both axes at once: a diagonal shift reuses the 4 corner-adjacent
// slots and re-renders 5. Fills `jobs` with the slots that need rendering,
// centre-first, and returns how many.
int grid_shift(tile_grid_t *g, int dx, int dy,
               render_job_t *jobs, int max_jobs);

// Change zoom. Nothing is reusable across zoom levels, so all slots become
// PENDING. Returns the job count (always GRID_COUNT), centre-first.
int grid_set_zoom(tile_grid_t *g, tile_id_t new_center,
                  render_job_t *jobs, int max_jobs);

// Given the marker's fractional tile position, how far has it left the centre
// tile? Returns -1/0/+1 per axis. This is the shift trigger.
void grid_drift(tile_grid_t *g, double frac_x, double frac_y,
                int *dx, int *dy);

// Worker-side commit, swapping in a freshly rendered buffer.
//
// The worker renders into a spare buffer and hands it over here; the slot's
// previous buffer is returned through *recycled to become the next spare.
// Swapping pointers rather than copying keeps the handover atomic under the
// lock, and means a slot showing a coarse placeholder is never partially
// overwritten by an in-progress render.
//
// Returns 0 if the job was stale, in which case *recycled is set to the
// worker's own buffer and the slot is left untouched.
int grid_commit_swap(tile_grid_t *g, const render_job_t *job,
                     tile_state_t result, uint16_t *fresh, uint16_t **recycled);

// Commit without a buffer swap, for results that produce no pixels
// (TILE_NODATA, TILE_ERROR).
int grid_commit(tile_grid_t *g, const render_job_t *job, tile_state_t result);

#ifdef __cplusplus
}
#endif
#endif // TILE_GRID_H
