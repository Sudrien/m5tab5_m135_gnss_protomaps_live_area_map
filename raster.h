// raster.h - scanline rasteriser for MVT geometry into an RGB565 subtile.
//
// Fixed point throughout: the P4 has no FPU worth using for this, and the
// coordinate range (tile extent, typically 4096, occasionally spilling past
// the tile edge into the buffer zone) fits comfortably in 16.8 fixed.
//
// Polygons use the nonzero winding rule, which requires every ring of a
// feature at once - an outer ring and its holes only make sense together.
// The MVT decoder hands over one ring at a time, so this module accumulates
// *edges* (not points) and flushes when the feature index changes. Edges are
// far more compact than points and bound the memory cleanly.
//
// Lines are drawn immediately per part; they need no accumulation.

#ifndef RASTER_H
#define RASTER_H

#include <stdint.h>
#include "mvt.h"

#ifdef __cplusplus
extern "C" {
#endif

// Subpixel precision for edge coordinates. 8 fractional bits gives 1/256 px,
// far finer than the antialiasing can resolve, with no overflow risk.
#define RS_FRAC_BITS 8
#define RS_ONE       (1 << RS_FRAC_BITS)

// Vertical supersampling for antialiasing. Each output row is sampled this
// many times and the coverage averaged.
//
// Measured on a dense z14 tile at 512 px: 1x costs 12.7 ms, 2x 14.1 ms,
// 4x 17.5 ms (desktop). The visible gain from 2x to 4x is small at 1:1 on a
// 720p panel, so 2x is the default - it keeps diagonals and coastlines clean
// without paying for samples you cannot see.
#ifndef RS_SUBSAMPLES
#define RS_SUBSAMPLES 2
#endif

typedef struct {
    uint16_t fill;        // RGB565
    uint16_t stroke;      // RGB565
    uint8_t  stroke_w;    // in output pixels, 0 = no stroke
    uint8_t  has_fill;
    uint8_t  has_stroke;
} rs_style_t;

typedef struct {
    int32_t y0, y1;       // fixed point, y0 < y1
    int32_t x;            // fixed point x at y0
    int32_t dxdy;         // fixed point slope
    int8_t  dir;          // +1 downward, -1 upward (winding contribution)
} rs_edge_t;

typedef struct {
    uint16_t *px;         // RGB565 target, w*h
    int32_t   w, h;

    // Source coordinate span, i.e. the MVT layer extent.
    int32_t   extent;

    // Sub-rectangle of the tile's coordinate space to draw, in extent units.
    // Defaults (0, 0, extent) render the whole tile, which is the ordinary
    // case. Setting a smaller span renders one quadrant - or sixteenth - of
    // the tile across the full output surface.
    //
    // This is what lets a lower-zoom tile supply several subtiles: one z13
    // tile drawn as four quadrants covers the same ground at the same pixel
    // density as four z14 tiles, while being a quarter of the download. The
    // limit is that the geometry was simplified for z13, so past about 2x the
    // intended density the simplification starts to show.
    int32_t   src_x0, src_y0, src_span;

    // Edge accumulator for the polygon currently being assembled.
    rs_edge_t *edges;
    uint32_t   edge_cap, edge_n;

    // Active edge indices for the scanline currently being filled.
    uint16_t  *active;
    uint32_t   active_cap;

    // Crossing list, reused per scanline: x position and winding direction.
    int32_t   *xs;
    int8_t    *dirs;
    uint32_t   xs_cap;

    // Per-row coverage accumulator, 0..RS_SUBSAMPLES*256 per pixel.
    uint16_t  *cov;
    int32_t    cov_lo, cov_hi;  // pixel range dirtied in the current row

    // Style table, indexed by the style byte the MVT decoder produced.
    const rs_style_t *styles;
    uint32_t          n_styles;

    // Feature currently being accumulated, for flush detection.
    int32_t   cur_feature;
    uint8_t   cur_style;
    int       cur_valid;
    int       cur_is_stroke;   // batch is stroked line quads, not a fill

    uint32_t  stat_spans, stat_edges, stat_lines;
} rs_t;

// Fill the whole target with a colour.
void rs_clear(rs_t *r, uint16_t color);

// Feed one decoded MVT part. Suitable as an mvt_part_fn with ctx = rs_t*.
int  rs_part(void *ctx, const mvt_part_t *part);

// Flush any polygon still accumulating. Call after each layer.
void rs_flush(rs_t *r);

// Direct primitives, exposed for tests and for the position marker overlay.
void rs_fill_poly(rs_t *r);                       // consumes accumulated edges
void rs_line(rs_t *r, int32_t x0, int32_t y0,
             int32_t x1, int32_t y1, int32_t width, uint16_t color);

// RGB565 helper.
static inline uint16_t rs_rgb(uint8_t rr, uint8_t gg, uint8_t bb) {
    return (uint16_t)(((rr & 0xF8) << 8) | ((gg & 0xFC) << 3) | (bb >> 3));
}

#ifdef __cplusplus
}
#endif
#endif // RASTER_H
