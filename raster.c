// raster.c

#include "raster.h"
#include <string.h>
#include <stdlib.h>
#include <limits.h>

// ---- colour ----------------------------------------------------------------
// Exact replacement for round(v / 255) with no division.
//
// This matters more than it looks: blend565 runs once per composited pixel,
// and a hardware divide on the P4 costs an order of magnitude more than the
// shifts and adds below. Verified identical to round(v/255) across the whole
// 0..65025 input range.
static inline uint32_t div255(uint32_t v) {
    v += 128;
    return (v + (v >> 8)) >> 8;
}

// Blend src over dst with coverage 0..255, channels unpacked and repacked.
static inline uint16_t blend565(uint16_t dst, uint16_t src, uint32_t a) {
    if (a >= 255) return src;
    if (a == 0)   return dst;
    uint32_t dr = (dst >> 11) & 0x1F, dg = (dst >> 5) & 0x3F, db = dst & 0x1F;
    uint32_t sr = (src >> 11) & 0x1F, sg = (src >> 5) & 0x3F, sb = src & 0x1F;
    uint32_t ia = 255 - a;
    uint32_t rr = div255(sr * a + dr * ia);
    uint32_t gg = div255(sg * a + dg * ia);
    uint32_t bb = div255(sb * a + db * ia);
    return (uint16_t)((rr << 11) | (gg << 5) | bb);
}

void rs_clear(rs_t *r, uint16_t color) {
    int32_t n = r->w * r->h;
    for (int32_t i = 0; i < n; i++) r->px[i] = color;
    // The fillers now leave the coverage buffer zeroed and rely on finding it
    // that way; establish that once per tile.
    if (r->cov) memset(r->cov, 0, (size_t)r->w * sizeof(uint16_t));
    r->cov_lo = INT32_MAX; r->cov_hi = INT32_MIN;
}

// ---- coordinate transform --------------------------------------------------
// MVT tile coords -> output pixels, in RS_FRAC_BITS fixed point, mapping the
// source sub-rectangle onto the full output surface. Separate x and y because
// the sub-rectangle has an origin on each axis.
static inline int32_t to_fx_x(const rs_t *r, int32_t v) {
    return (int32_t)(((int64_t)(v - r->src_x0) * r->w << RS_FRAC_BITS) / r->src_span);
}
static inline int32_t to_fx_y(const rs_t *r, int32_t v) {
    return (int32_t)(((int64_t)(v - r->src_y0) * r->h << RS_FRAC_BITS) / r->src_span);
}

// ---- edge accumulation -----------------------------------------------------
static void add_edge(rs_t *r, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    if (y0 == y1) return;                     // horizontal edges contribute nothing
    if (r->edge_n >= r->edge_cap) return;     // silently drop; stat_edges reveals it

    rs_edge_t *e = &r->edges[r->edge_n++];
    if (y0 < y1) {
        e->y0 = y0; e->y1 = y1; e->x = x0; e->dir = +1;
        e->dxdy = (int32_t)(((int64_t)(x1 - x0) << RS_FRAC_BITS) / (y1 - y0));
    } else {
        e->y0 = y1; e->y1 = y0; e->x = x1; e->dir = -1;
        e->dxdy = (int32_t)(((int64_t)(x0 - x1) << RS_FRAC_BITS) / (y0 - y1));
    }
    r->stat_edges++;
}

static void add_ring(rs_t *r, const int32_t *pts, uint32_t n) {
    if (n < 2) return;
    int32_t px = to_fx_x(r, pts[0]), py = to_fx_y(r, pts[1]);
    int32_t fx = px, fy = py;
    for (uint32_t i = 1; i < n; i++) {
        int32_t cx = to_fx_x(r, pts[i * 2]), cy = to_fx_y(r, pts[i * 2 + 1]);
        add_edge(r, px, py, cx, cy);
        px = cx; py = cy;
    }
    // The decoder already appends the closing vertex, but a ring that was
    // truncated by a buffer limit may not be closed. Cheap insurance.
    if (px != fx || py != fy) add_edge(r, px, py, fx, fy);
}

// ---- scanline fill ---------------------------------------------------------
static int cmp_edge_y(const void *a, const void *b) {
    const rs_edge_t *ea = a, *eb = b;
    return (ea->y0 > eb->y0) - (ea->y0 < eb->y0);
}

// Sort the edge list by y0.
//
// qsort was costing more than the fill it was preparing for: a building is a
// four-edge quad, and dispatching through a function pointer per comparison
// dwarfs the comparison itself at that size. Insertion sort is used up to the
// point where its quadratic term starts to bite, which is well above any
// feature that is not a coastline. Same reasoning as sort_crossings, applied
// one level up.
#define RS_INSERTION_MAX 48

static void sort_edges_y(rs_edge_t *e, uint32_t n) {
    if (n < 2) return;
    if (n > RS_INSERTION_MAX) {
        qsort(e, n, sizeof(rs_edge_t), cmp_edge_y);
        return;
    }
    for (uint32_t i = 1; i < n; i++) {
        rs_edge_t tmp = e[i];
        uint32_t j = i;
        while (j > 0 && e[j - 1].y0 > tmp.y0) {
            e[j] = e[j - 1];
            j--;
        }
        e[j] = tmp;
    }
}

// Insertion sort the crossing list. Counts are small (typically 2-8 per
// scanline, occasionally dozens) so this beats qsort's overhead comfortably.
static void sort_crossings(int32_t *xs, int8_t *dirs, uint32_t n) {
    for (uint32_t i = 1; i < n; i++) {
        int32_t x = xs[i]; int8_t d = dirs[i];
        uint32_t j = i;
        while (j > 0 && xs[j - 1] > x) {
            xs[j] = xs[j - 1]; dirs[j] = dirs[j - 1];
            j--;
        }
        xs[j] = x; dirs[j] = d;
    }
}

// Accumulate horizontal coverage for one span into the row buffer. x0/x1 are
// fixed point; partial pixels at each end get proportional coverage, which is
// what removes the staircase on near-vertical polygon edges.
static void span_coverage(rs_t *r, int32_t x0, int32_t x1) {
    if (x1 <= x0) return;
    if (x0 < 0) x0 = 0;
    if (x1 > (r->w << RS_FRAC_BITS)) x1 = r->w << RS_FRAC_BITS;
    if (x1 <= x0) return;

    int32_t p0 = x0 >> RS_FRAC_BITS;
    int32_t p1 = (x1 - 1) >> RS_FRAC_BITS;

    // Remember what was dirtied so the caller can composite and re-zero
    // exactly that, instead of sweeping the whole shape's bounding box.
    if (p0 < r->cov_lo) r->cov_lo = p0;
    if (p1 > r->cov_hi) r->cov_hi = p1;

    if (p0 == p1) {
        r->cov[p0] += (uint16_t)(((x1 - x0) * 255) >> RS_FRAC_BITS);
        return;
    }
    // leading partial pixel
    int32_t lead = RS_ONE - (x0 & (RS_ONE - 1));
    r->cov[p0] += (uint16_t)((lead * 255) >> RS_FRAC_BITS);
    // solid interior
    for (int32_t p = p0 + 1; p < p1; p++) r->cov[p] += 255;
    // trailing partial pixel
    int32_t tail = x1 & (RS_ONE - 1);
    if (tail == 0) tail = RS_ONE;
    r->cov[p1] += (uint16_t)((tail * 255) >> RS_FRAC_BITS);
}

void rs_fill_poly(rs_t *r) {
    if (r->edge_n == 0) return;
    const rs_style_t *st = (r->cur_style < r->n_styles)
                           ? &r->styles[r->cur_style] : NULL;
    if (!st) { r->edge_n = 0; return; }
    uint16_t color;
    if (r->cur_is_stroke) {
        if (!st->has_stroke) { r->edge_n = 0; return; }
        color = st->stroke;
    } else {
        if (!st->has_fill) { r->edge_n = 0; return; }
        color = st->fill;
    }

    sort_edges_y(r->edges, r->edge_n);

    // Vertical bounds of the whole polygon, clipped to the target.
    int32_t ymin = r->edges[0].y0 >> RS_FRAC_BITS;
    int32_t ymax = 0;
    for (uint32_t i = 0; i < r->edge_n; i++) {
        int32_t e1 = (r->edges[i].y1 + RS_ONE - 1) >> RS_FRAC_BITS;
        if (e1 > ymax) ymax = e1;
    }
    if (ymin < 0) ymin = 0;
    if (ymax > r->h) ymax = r->h;
    if (ymin >= ymax) { r->edge_n = 0; return; }

    // Horizontal bounds too. Clearing and compositing the whole row for a
    // shape a few pixels wide is what made line rendering pathological: a
    // road segment spanning 3 rows was memsetting 3 KB to touch 12 pixels.
    int32_t xlo = INT32_MAX, xhi = INT32_MIN;
    for (uint32_t i = 0; i < r->edge_n; i++) {
        const rs_edge_t *e = &r->edges[i];
        int32_t xa = e->x;
        int32_t xb = e->x + (int32_t)(((int64_t)(e->y1 - e->y0) * e->dxdy) >> RS_FRAC_BITS);
        if (xa > xb) { int32_t t = xa; xa = xb; xb = t; }
        if (xa < xlo) xlo = xa;
        if (xb > xhi) xhi = xb;
    }
    xlo >>= RS_FRAC_BITS;
    xhi = (xhi >> RS_FRAC_BITS) + 1;
    if (xlo < 0) xlo = 0;
    if (xhi > r->w) xhi = r->w;
    if (xlo >= xhi) { r->edge_n = 0; return; }

    uint32_t next_edge = 0;
    uint32_t n_active = 0;

    // Skip edges entirely above the clip region.
    while (next_edge < r->edge_n &&
           (r->edges[next_edge].y1 >> RS_FRAC_BITS) < ymin) next_edge++;

    const int32_t SUB = RS_SUBSAMPLES;
    const int32_t sub_step = RS_ONE / SUB;

    // The coverage buffer is kept zeroed between rows rather than cleared
    // over the bounding box each time. A polygon's bbox is a poor guide to
    // what any one row touches - the top row of a circle spans a few pixels
    // of a box the full width of the shape - so both the clear and the
    // composite scan were doing bbox-sized work for span-sized output.
    for (int32_t y = ymin; y < ymax; y++) {
        r->cov_lo = INT32_MAX; r->cov_hi = INT32_MIN;

        for (int32_t s = 0; s < SUB; s++) {
            // Sample at the centre of each sub-row.
            int32_t sy = (y << RS_FRAC_BITS) + s * sub_step + sub_step / 2;

            // Admit newly started edges.
            while (next_edge < r->edge_n && r->edges[next_edge].y0 <= sy) {
                if (r->edges[next_edge].y1 > sy && n_active < r->active_cap)
                    r->active[n_active++] = (uint16_t)next_edge;
                next_edge++;
            }
            // Retire finished edges.
            for (uint32_t i = 0; i < n_active; ) {
                if (r->edges[r->active[i]].y1 <= sy)
                    r->active[i] = r->active[--n_active];
                else i++;
            }
            if (n_active == 0) continue;

            // Crossing x for each active edge at this sub-row.
            uint32_t nx = 0;
            for (uint32_t i = 0; i < n_active && nx < r->xs_cap; i++) {
                const rs_edge_t *e = &r->edges[r->active[i]];
                if (sy < e->y0 || sy >= e->y1) continue;
                int64_t dx = (int64_t)(sy - e->y0) * e->dxdy >> RS_FRAC_BITS;
                r->xs[nx]   = e->x + (int32_t)dx;
                r->dirs[nx] = e->dir;
                nx++;
            }
            if (nx < 2) continue;
            sort_crossings(r->xs, r->dirs, nx);

            // Nonzero winding: a span is inside wherever the running winding
            // count is non-zero. This is what makes holes work, and why every
            // ring of a feature has to be present together.
            int wind = 0;
            for (uint32_t i = 0; i + 1 < nx; i++) {
                wind += r->dirs[i];
                if (wind != 0) {
                    span_coverage(r, r->xs[i], r->xs[i + 1]);
                    r->stat_spans++;
                }
            }
        }

        // Composite the row, then hand the buffer back zeroed.
        if (r->cov_lo > r->cov_hi) continue;
        int32_t c0 = r->cov_lo, c1 = r->cov_hi + 1;
        if (c0 < xlo) c0 = xlo;
        if (c1 > xhi) c1 = xhi;

        uint16_t *row = r->px + (size_t)y * r->w;
        for (int32_t x = c0; x < c1; x++) {
            uint32_t c = r->cov[x];
            if (!c) continue;
            c /= SUB;
            if (c > 255) c = 255;
            row[x] = blend565(row[x], color, c);
        }
        memset(r->cov + r->cov_lo, 0,
               (size_t)(r->cov_hi - r->cov_lo + 1) * sizeof(uint16_t));
    }
    r->edge_n = 0;
}

// ---- lines -----------------------------------------------------------------
// Thick lines are quads. Joins are left to the overlap of consecutive
// segments, which at road widths on this display is indistinguishable from
// proper round joins and costs nothing.
// Fill a convex quad directly, bypassing the generic polygon path.
//
// This is the hot path: a tile has thousands of road segments and each one is
// a quad. The generic filler sorts an edge list, maintains an active table and
// applies the nonzero winding rule - all of which are wasted on a shape that
// has exactly two crossings per scanline. Going direct removed the single
// largest cost in the rasteriser.
static void fill_quad(rs_t *r, const int32_t *qx, const int32_t *qy,
                      uint16_t color)
{
    int32_t ylo = qy[0], yhi = qy[0];
    int32_t xlo = qx[0], xhi = qx[0];
    for (int i = 1; i < 4; i++) {
        if (qy[i] < ylo) ylo = qy[i];
        if (qy[i] > yhi) yhi = qy[i];
        if (qx[i] < xlo) xlo = qx[i];
        if (qx[i] > xhi) xhi = qx[i];
    }
    int32_t y0 = ylo >> RS_FRAC_BITS;
    int32_t y1 = (yhi >> RS_FRAC_BITS) + 1;
    int32_t px0 = xlo >> RS_FRAC_BITS;
    int32_t px1 = (xhi >> RS_FRAC_BITS) + 1;
    if (y0 < 0) y0 = 0;
    if (y1 > r->h) y1 = r->h;
    if (px0 < 0) px0 = 0;
    if (px1 > r->w) px1 = r->w;
    if (y0 >= y1 || px0 >= px1) return;

    const int32_t SUB = RS_SUBSAMPLES;
    const int32_t sub_step = RS_ONE / SUB;

    // Precompute each edge's slope once. The crossing test used to divide
    // inside the scanline loop - four 64-bit divides per subsample per row,
    // per segment - which on a core without a fast divider dominated
    // everything else the rasteriser did.
    int32_t eslope[4];
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        int32_t dy4 = qy[j] - qy[i];
        eslope[i] = dy4 ? (int32_t)((((int64_t)(qx[j] - qx[i])) << RS_FRAC_BITS) / dy4) : 0;
    }

    const int32_t XMAX_FX = r->w << RS_FRAC_BITS;

    for (int32_t y = y0; y < y1; y++) {
        // Find this row's crossings first, for every subsample, before
        // touching the coverage buffer.
        //
        // The quad's bounding box is the wrong extent to clear and composite.
        // A road segment running diagonally has an L x L bbox while covering
        // only L x width pixels, so clearing and scanning the full box makes
        // a long segment cost O(L^2) for O(L*w) of actual work - the same
        // pathology already fixed in the generic filler, just hidden here
        // behind a shape that looked small.
        int32_t slo[RS_SUBSAMPLES], shi[RS_SUBSAMPLES];
        int32_t rlo = INT32_MAX, rhi = INT32_MIN;

        for (int32_t s = 0; s < SUB; s++) {
            int32_t sy = (y << RS_FRAC_BITS) + s * sub_step + sub_step / 2;
            int32_t lo = INT32_MAX, hi = INT32_MIN;

            // Two of the four edges straddle any given scanline; taking the
            // min and max crossing finds them without knowing which.
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) & 3;
                int32_t ay = qy[i], by = qy[j];
                if (ay == by) continue;
                if ((sy < ay) == (sy < by)) continue;   // does not straddle
                int32_t x = qx[i] +
                    (int32_t)(((int64_t)(sy - ay) * eslope[i]) >> RS_FRAC_BITS);
                if (x < lo) lo = x;
                if (x > hi) hi = x;
            }
            slo[s] = lo; shi[s] = hi;
            if (lo < hi) {
                if (lo < rlo) rlo = lo;
                if (hi > rhi) rhi = hi;
            }
        }
        if (rlo >= rhi) continue;

        // Clip to the surface exactly as span_coverage would, so the pixel
        // range cleared is the range that can actually be written.
        if (rlo < 0) rlo = 0;
        if (rhi > XMAX_FX) rhi = XMAX_FX;
        if (rlo >= rhi) continue;

        int32_t cx0 = rlo >> RS_FRAC_BITS;
        int32_t cx1 = ((rhi - 1) >> RS_FRAC_BITS) + 1;
        if (cx0 < px0) cx0 = px0;
        if (cx1 > px1) cx1 = px1;
        if (cx0 >= cx1) continue;

        r->cov_lo = INT32_MAX; r->cov_hi = INT32_MIN;
        for (int32_t s = 0; s < SUB; s++) {
            if (slo[s] >= shi[s]) continue;
            span_coverage(r, slo[s], shi[s]);
            r->stat_spans++;
        }
        if (r->cov_lo > r->cov_hi) continue;
        if (cx0 < r->cov_lo) cx0 = r->cov_lo;
        if (cx1 > r->cov_hi + 1) cx1 = r->cov_hi + 1;

        uint16_t *row = r->px + (size_t)y * r->w;
        for (int32_t x = cx0; x < cx1; x++) {
            uint32_t c = r->cov[x];
            if (!c) continue;
            c /= SUB;
            if (c > 255) c = 255;
            row[x] = blend565(row[x], color, c);
        }
        memset(r->cov + r->cov_lo, 0,
               (size_t)(r->cov_hi - r->cov_lo + 1) * sizeof(uint16_t));
    }
}

void rs_line(rs_t *r, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
             int32_t width, uint16_t color)
{
    int32_t dx = x1 - x0, dy = y1 - y0;
    if (dx == 0 && dy == 0) return;

    int64_t len2 = (int64_t)dx * dx + (int64_t)dy * dy;
    int64_t len = 0;
    {   // integer sqrt, no FPU
        int64_t v = len2, g = 0, b = 1LL << 40;
        while (b > v) b >>= 2;
        while (b) {
            if (v >= g + b) { v -= g + b; g = (g >> 1) + b; }
            else g >>= 1;
            b >>= 2;
        }
        len = g;
    }
    if (len == 0) return;

    int32_t hw = width / 2;
    if (hw < RS_ONE / 2) hw = RS_ONE / 2;      // never thinner than one pixel
    int32_t ox = (int32_t)(-(int64_t)dy * hw / len);
    int32_t oy = (int32_t)( (int64_t)dx * hw / len);

    int32_t qx[4] = { x0 + ox, x1 + ox, x1 - ox, x0 - ox };
    int32_t qy[4] = { y0 + oy, y1 + oy, y1 - oy, y0 - oy };
    fill_quad(r, qx, qy, color);
    r->stat_lines++;
}

static void stroke_path(rs_t *r, const int32_t *pts, uint32_t n,
                        int32_t width, uint16_t color)
{
    if (n < 2) return;
    const int32_t MIN_STEP = RS_ONE / 2;      // half a pixel, fixed point

    int32_t px = to_fx_x(r, pts[0]), py = to_fx_y(r, pts[1]);
    for (uint32_t i = 1; i < n; i++) {
        int32_t cx = to_fx_x(r, pts[i * 2]), cy = to_fx_y(r, pts[i * 2 + 1]);
        int32_t adx = cx > px ? cx - px : px - cx;
        int32_t ady = cy > py ? cy - py : py - cy;
        if (i + 1 < n && adx < MIN_STEP && ady < MIN_STEP) continue;
        rs_line(r, px, py, cx, cy, width << RS_FRAC_BITS, color);
        px = cx; py = cy;
    }
}

// ---- MVT part sink ---------------------------------------------------------
void rs_flush(rs_t *r) {
    if (r->cur_valid && r->edge_n) rs_fill_poly(r);
    r->edge_n = 0;
    r->cur_valid = 0;
    r->cur_is_stroke = 0;
    r->cur_feature = -1;
}

int rs_part(void *ctx, const mvt_part_t *part) {
    rs_t *r = ctx;
    r->extent = (int32_t)part->layer->extent;
    // A caller that has not asked for a sub-rectangle gets the whole tile.
    if (r->src_span <= 0) { r->src_x0 = 0; r->src_y0 = 0; r->src_span = r->extent; }

    const rs_style_t *st = (part->style < r->n_styles)
                           ? &r->styles[part->style] : NULL;
    if (!st) return 0;                       // unstyled: skip silently

    if (part->geom == MVT_POLYGON) {
        if (r->cur_is_stroke && r->edge_n) rs_fill_poly(r);
        r->cur_is_stroke = 0;
        // A new feature means the previous polygon is complete. Polygons are
        // filled per feature, not batched: holes only make sense within one
        // feature, and batching would merge them across features.
        if (!r->cur_valid || (int32_t)part->feature_index != r->cur_feature) {
            if (r->cur_valid && r->edge_n) rs_fill_poly(r);
            r->cur_feature = (int32_t)part->feature_index;
            r->cur_style   = part->style;
            r->cur_valid   = 1;
            r->edge_n      = 0;
        }
        add_ring(r, part->pts, part->n_pts);
        return 0;
    }

    if (part->geom == MVT_LINESTRING && st->has_stroke) {
        if (r->cur_valid && r->edge_n) { rs_fill_poly(r); r->edge_n = 0; }
        r->cur_valid = 0;
        r->cur_is_stroke = 0;
        stroke_path(r, part->pts, part->n_pts,
                    st->stroke_w ? st->stroke_w : 1, st->stroke);
        return 0;
    }

    if (part->geom == MVT_POINT && st->has_fill) {
        if (r->edge_n) rs_fill_poly(r);
        r->cur_is_stroke = 0;
        r->cur_style = part->style;
        r->cur_valid = 1;
        r->edge_n = 0;
        int32_t rad = (st->stroke_w ? st->stroke_w : 3) << RS_FRAC_BITS;
        for (uint32_t i = 0; i < part->n_pts; i++) {
            int32_t cx = to_fx_x(r, part->pts[i * 2]);
            int32_t cy = to_fx_y(r, part->pts[i * 2 + 1]);
            add_edge(r, cx, cy - rad, cx + rad, cy);
            add_edge(r, cx + rad, cy, cx, cy + rad);
            add_edge(r, cx, cy + rad, cx - rad, cy);
            add_edge(r, cx - rad, cy, cx, cy - rad);
        }
        return 0;
    }
    return 0;
}
