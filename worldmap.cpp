#include "worldmap.h"
#include "worldmap_data.h"

#include <M5Unified.h>
#include <stdlib.h>

namespace {

// Palette. Muted on purpose: the boot panel sits on top of this and its text
// has to stay readable, so the backdrop is closer to a chart than to a
// satellite photo. The night pair is the same hues at roughly a third of the
// brightness.
struct Palette { uint8_t sea[3], land[3]; };
const Palette PAL_DAY   = { { 18, 38,  66 }, { 46, 72,  54 } };
const Palette PAL_NIGHT = { {  8, 14,  26 }, { 18, 30,  22 } };

// Room for every crossing on one scanline. A horizontal line across the
// widest part of the world at 1:110m crosses far fewer edges than this, but
// the fill below has to be correct rather than lucky, so overflow drops the
// extra crossings in pairs (see fill_row) instead of writing past the end.
const int MAX_X = 256;

int cmp_u16(const void *a, const void *b) {
    int ia = *(const uint16_t *)a, ib = *(const uint16_t *)b;
    return ia - ib;
}

// One scanline of one layer, even-odd. `sy` is the row in world units
// (0..65535); crossings come back as x in the same units.
//
// Even-odd rather than nonzero winding, and that is what makes lakes-inside-
// land and land-inside-lakes (Manitoulin, the islands in the Caspian) come
// out right without carrying ring orientation through the generator.
int row_spans(const uint16_t *xy, const uint16_t *ring, int rings,
              int32_t sy, uint16_t *out) {
    int n = 0;
    for (int r = 0; r < rings; r++) {
        const int a = ring[r], b = ring[r + 1];
        for (int i = a; i < b; i++) {
            const int j = (i + 1 < b) ? i + 1 : a;   // ring closes on itself
            int32_t y0 = xy[i * 2 + 1], y1 = xy[j * 2 + 1];
            if (y0 == y1) continue;
            // Half-open in y: a vertex exactly on the scanline counts for the
            // edge below it and not the one above, so crossings stay paired.
            if (!((y0 <= sy && sy < y1) || (y1 <= sy && sy < y0))) continue;
            int32_t x0 = xy[i * 2], x1 = xy[j * 2];
            int32_t x = x0 + (int32_t)(((int64_t)(sy - y0) * (x1 - x0)) / (y1 - y0));
            if (x < 0) x = 0;
            if (x > 65535) x = 65535;
            if (n < MAX_X) out[n++] = (uint16_t)x;
        }
    }
    if (n & 1) n--;                                  // never fill to the edge
    if (n > 1) qsort(out, n, sizeof out[0], cmp_u16);
    return n;
}

}  // namespace

uint16_t worldmap_sea(bool dark) {
    const Palette &p = dark ? PAL_NIGHT : PAL_DAY;
    return M5.Display.color565(p.sea[0], p.sea[1], p.sea[2]);
}

void worldmap_draw(int x0, int y0, int w, int h, bool dark) {
    if (w <= 0 || h <= 0) return;

    const Palette &p = dark ? PAL_NIGHT : PAL_DAY;
    const uint16_t sea  = M5.Display.color565(p.sea[0],  p.sea[1],  p.sea[2]);
    const uint16_t land = M5.Display.color565(p.land[0], p.land[1], p.land[2]);

    // Cover, not fit: the square world is sized to the longer edge and
    // centred, so the shorter one is cropped.
    const int side = w > h ? w : h;
    const int ox = x0 + (w - side) / 2;
    const int oy = y0 + (h - side) / 2;

    // One row at a time. A full-screen buffer would be 1.8 MB and this runs
    // before the allocator is under any obligation to have it.
    static uint16_t row[1280];
    const int rw = w > (int)(sizeof row / sizeof row[0])
                 ? (int)(sizeof row / sizeof row[0]) : w;

    uint16_t xs[MAX_X];

    for (int y = 0; y < h; y++) {
        for (int i = 0; i < rw; i++) row[i] = sea;

        // Screen row -> world row. Rows off the top or bottom of the world
        // (only possible when the rectangle is taller than it is wide) stay
        // open ocean, which is what is up there anyway.
        const int64_t sy = ((int64_t)(y0 + y - oy) * 65535) / side;
        if (sy >= 0 && sy <= 65535) {
            // Land first, then lakes painted back over it in sea colour: two
            // even-odd passes are simpler than one combined pass and the
            // second one touches a few hundred pixels.
            int n = row_spans(WM_LAND_XY, WM_LAND_RING, WM_LAND_RINGS,
                              (int32_t)sy, xs);
            for (int k = 0; k + 1 < n; k += 2) {
                int a = (int)(((int64_t)xs[k]     * side) / 65535) + ox - x0;
                int b = (int)(((int64_t)xs[k + 1] * side) / 65535) + ox - x0;
                if (b < 0 || a >= rw) continue;
                if (a < 0) a = 0;
                if (b > rw) b = rw;
                // A feature narrower than a pixel still gets one: an island
                // that rounds away entirely is worse than one a pixel wide.
                if (b == a && a < rw) b = a + 1;
                for (int i = a; i < b; i++) row[i] = land;
            }

            n = row_spans(WM_LAKE_XY, WM_LAKE_RING, WM_LAKE_RINGS,
                          (int32_t)sy, xs);
            for (int k = 0; k + 1 < n; k += 2) {
                int a = (int)(((int64_t)xs[k]     * side) / 65535) + ox - x0;
                int b = (int)(((int64_t)xs[k + 1] * side) / 65535) + ox - x0;
                if (b < 0 || a >= rw) continue;
                if (a < 0) a = 0;
                if (b > rw) b = rw;
                for (int i = a; i < b; i++) row[i] = sea;
            }
        }

        M5.Display.pushImage(x0, y0 + y, rw, 1, row);
    }
}
