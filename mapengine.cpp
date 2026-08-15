// mapengine.cpp

#include "mapengine.h"
#include "style.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <SD_MMC.h>
#include <WiFi.h>          // world_task checks the link before each fetch
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <math.h>

#include "netsource.h"

extern "C" {
  #include "pmtiles.h"
  #include "mvt.h"
  #include "inflate.h"
  #include "raster.h"
}
#include "mercator.h"
#include "mapconfig.h"
#include "waypoints.h"

// ---- configuration ---------------------------------------------------------
// Scratch for one tile, compressed and inflated.
//
// These were sized against an 84 KB z14 tile. A z13 tile covers four times
// the ground and can be several times larger, so both grow on demand: gzip
// records its uncompressed size in the trailer, which makes the requirement
// exact rather than a guess.
// No panel attached: skip anything that would ask the display for geometry.
// See map_set_headless() in mapengine.h.
static bool g_headless = false;

void map_set_headless(bool headless) { g_headless = headless; }

static uint32_t TILE_CAP = 192 * 1024;
static uint32_t MVT_CAP  = 384 * 1024;
static const uint32_t MVT_CAP_MAX = 3 * 1024 * 1024;
static const uint32_t PT_CAP    =  2048;
static const uint32_t VAL_CAP   =  1024;
static const uint32_t EDGE_CAP  = 16384;
static const uint32_t XS_CAP    =  4096;

// Screen geometry. The status bar owns the top strip and the buttons the
// bottom one; the map is clipped out of both so they are not fighting over
// the same pixels.
static const int STATUS_H = 52;
static const int FOOTER_H = 82;

// pushImage handled pixel format for us; writePixels is lower level and takes
// byte order explicitly. If red and blue come out exchanged, flip this.
#ifndef BLIT_SWAP_BYTES
#define BLIT_SWAP_BYTES false
#endif

// ---- state -----------------------------------------------------------------
static tile_grid_t      g_grid;
static SemaphoreHandle_t g_glock = nullptr;
static QueueHandle_t     g_jobs  = nullptr;

static uint8_t           g_zoom = 14;
static bool              g_centred = false;
static MapStats          g_stats;

// Archive access lives entirely in netsource now: it owns the local floor,
// the remote build, and the SD cache between them. Nothing here touches
// pmtiles directly.

// Worker-owned scratch. Only the render task reads or writes any of this,
// which is why none of it needs a lock.
static uint8_t   *w_tile = nullptr;    // compressed tile, straight from the source
static uint8_t   *w_mvt  = nullptr;    // inflated MVT
static int32_t   *w_pts  = nullptr;    // decoded points for one geometry part
static uint8_t   *w_val  = nullptr;    // one style byte per value-table entry
static rs_edge_t *w_edges = nullptr;
static uint16_t  *w_active = nullptr;
static int32_t   *w_xs = nullptr;
static int8_t    *w_dirs = nullptr;
static uint16_t  *w_cov = nullptr;
static const char **w_val_name = nullptr;   // value-table strings, for labels
static uint16_t  *w_val_name_len = nullptr;

// ---- labels ----------------------------------------------------------------
// Labels are collected during the render but drawn at composite time, not
// baked into the tile bitmap.
//
// Baking was the obvious thing and it is wrong twice. A subtile is a window
// onto the world, not a self-contained picture: a name whose box straddles
// the seam between two subtiles would be sliced in half by whichever one
// rendered it, and neither has the other's pixels to finish it. And because
// colours are baked at rasterise time, a POI toggle would then be a full
// re-render of every resident tile - the same second-of-coarse-fallback cost
// as map_set_dark - for what the user experiences as a checkbox.
//
// Collecting instead gives real fonts (LovyanGFX, rather than an embedded
// bitmap face in raster.c), lets a label overhang a tile edge, allows
// collision culling across the whole screen rather than per tile, and makes
// the toggle free. The price is that map_draw must repaint labels whenever it
// repaints pixels underneath them - see draw_labels().
#ifndef LABEL_TEXT_MAX
#define LABEL_TEXT_MAX 40
#endif
// Per subtile. A dense city z14 tile carries a few hundred POIs; there is no
// point keeping labels that collision culling will drop anyway, and the cap
// bounds the memory at GRID_COUNT * 48 * 48 B, about 9 KB for a 2x2.
#ifndef LABELS_PER_TILE
#define LABELS_PER_TILE 48
#endif

struct MapLabel {
    float   fx, fy;        // position within the subtile, 0..1
    uint8_t style;         // S_POI or one of the S_PLACE_* ranks
    char    text[LABEL_TEXT_MAX];
};

struct LabelSet {
    uint16_t n;
    MapLabel v[LABELS_PER_TILE];
};

// One per slot plus a spare, handed round by pointer swap rather than copied.
//
// This is the pixel buffers' invariant again, and for the same reason: the
// worker fills the spare while the UI task draws from the slots, so the two
// never touch the same set. A memcpy at commit would instead put a 2 KB write
// in the middle of whatever draw_labels was halfway through reading, and the
// visible result - one frame of a city's name spliced onto a cafe's - is
// exactly the class of bug the pixel side already avoids.
static LabelSet  g_labelbuf[GRID_COUNT + 1];
static LabelSet *g_labels[GRID_COUNT];
static LabelSet *w_labels = nullptr;   // the spare; worker-owned
static bool      g_labels_on = true;
static bool      g_pins_on   = true;

static void labels_init() {
    for (int i = 0; i < GRID_COUNT; i++) { g_labelbuf[i].n = 0; g_labels[i] = &g_labelbuf[i]; }
    g_labelbuf[GRID_COUNT].n = 0;
    w_labels = &g_labelbuf[GRID_COUNT];
}

// The place names behind the status bar readout. Written by the worker's
// region pass and by label collection; read by the UI task. Both are short
// and written whole, and a torn read costs one stale frame of a city name, so
// they are not worth a lock.
// The chain, finest first: neighbourhood, locality, region, country. Each is
// the nearest named point of its own rank, resolved independently - a
// neighbourhood is not looked up "inside" its city, because the basemap gives
// points and not containment, so there is no hierarchy to walk. Two of them
// come from the working grid and two from the region tile.
enum { PL_HOOD = 0, PL_LOCALITY, PL_REGION, PL_COUNTRY, PL_COUNT };
static char g_place[PL_COUNT][LABEL_TEXT_MAX] = { "", "", "", "" };

// How near a point has to be, in tiles at the working zoom, before its name
// is claimed as "where you are".
//
// Without this the chain reads as confidently wrong: a neighbourhood point
// two tiles away is a different neighbourhood, and naming it is worse than
// naming none. At z14 and latitude ~42 a tile is about 2.4 km, so 0.4 is
// roughly a kilometre - about the radius over which a neighbourhood name
// still describes you - and 3.0 covers a town from its outskirts.
//
// The region and country ranks get no radius. Their points are centroids of
// something enormous, so distance carries no information about whether you
// are inside; nearest is the only answer available. See PLACE_ZOOM.
#ifndef PLACE_HOOD_RADIUS
#define PLACE_HOOD_RADIUS 0.4
#endif
// 4.0 tiles is about 7 km at z14. A township's centroid can sit that far from
// its edge, which is the case that failed before - Canton is named from a
// point several kilometres from where you cross into it.
#ifndef PLACE_LOCALITY_RADIUS
#define PLACE_LOCALITY_RADIUS 4.0
#endif

// Country is the least useful line most of the time - you generally know
// which one you are in - so it is easy to switch off without touching the
// rest of the chain.
#ifndef PLACE_SHOW_COUNTRY
#define PLACE_SHOW_COUNTRY 1
#endif

// Where the collector should put what it finds. Set by render_tile's caller.
static LabelSet *w_label_dst = nullptr;

static void label_reset(LabelSet *ls) { if (ls) ls->n = 0; }

static void label_add(LabelSet *ls, float fx, float fy, uint8_t style,
                      const char *txt, uint32_t len)
{
    if (!ls || ls->n >= LABELS_PER_TILE) return;
    if (len == 0) return;
    if (len > LABEL_TEXT_MAX - 1) len = LABEL_TEXT_MAX - 1;
    MapLabel *m = &ls->v[ls->n++];
    m->fx = fx; m->fy = fy; m->style = style;
    memcpy(m->text, txt, len);
    m->text[len] = 0;
}

// ---- coarse overview -------------------------------------------------------
// One tile several zoom levels below the working grid, covering far more
// ground than the screen. New slots are filled by upscaling from it, so the
// map is never blank - only soft until the real tile lands.
//
// COARSE_STEP sets how far below the working zoom that tile is drawn, and with
// it both the softness and how often it has to be re-rendered:
//
//   step  overview  span  src px/tile  upscale to SUBTILE_PX 1280
//     3      z11     8x8       64          20.0x
//     2      z12     4x4      128          10.0x   <- current
//
// (src px/tile is COARSE_PX / span, so COARSE_PX is the other lever: doubling
// it to 1024 halves the upscale again, at 2 MB of PSRAM instead of 0.5 MB.)
//
// This was 3, and the placeholder was 64 source pixels stretched to 1280 -
// blocky enough to be distracting rather than merely soft. The comment here
// used to describe 128 -> 1024, which was never what mapconfig.h set.
//
// The cost of step 2 is re-rendering: one coarse tile now covers 4x4 working
// tiles instead of 8x8, so the grid leaves it four times as often. A z12 tile
// carries less geometry than a z11 one covering the same ground, which offsets
// part of that but not all of it - expect roughly twice the overview render
// work overall.
#ifndef COARSE_STEP_CFG
#define COARSE_STEP_CFG 2
#endif
static const int COARSE_STEP = COARSE_STEP_CFG;
static const uint8_t COARSE_SLOT = 0xFE;   // sentinel job slot

static uint16_t *g_coarse_px = nullptr;
static tile_id_t g_coarse_id = { 0, 0, 0 };
static bool      g_coarse_ok = false;
static tile_id_t g_coarse_want = { 0, 0, 0 };
static int64_t   g_coarse_retry_at = 0;

// Spare buffer the worker renders into before swapping it into a slot.
static uint16_t *g_spare = nullptr;

// Marker position in world coordinates (fractional tiles at the working
// zoom), and the view's top-left corner in the same units.
//
// The view is held in world coordinates rather than canvas pixels so that a
// grid shift - which moves the canvas origin by a whole tile - does not move
// the picture. Converting to canvas pixels happens at draw time against the
// current origin.
static double g_marker_wx = 0, g_marker_wy = 0;
static double g_view_wx = 0, g_view_wy = 0;
static bool   g_view_set = false;

// ---- rendering one tile ----------------------------------------------------
static const char *g_want_layer = nullptr;
static int rl_layer(void *ctx, const mvt_layer_t *l) {
    (void)ctx;
    return g_want_layer && l->name_len == strlen(g_want_layer) &&
           memcmp(l->name, g_want_layer, l->name_len) == 0;
}

// Geometry sink that also harvests label text.
//
// rs_part is called for every part either way - the dot for a POI is still
// drawn by the rasteriser, and the label sits on top of it - so this wraps
// rather than replaces it. Non-point and unnamed features fall straight
// through at the cost of two compares.
static int rl_part(void *ctx, const mvt_part_t *part) {
    if (w_label_dst && part->name && part->n_pts >= 1 &&
        part->geom == MVT_POINT && style_is_labelled(part->style)) {
        rs_t *r = (rs_t *)ctx;
        // Tile-local coords to a fraction of *this subtile*, which is only
        // the whole tile when SUBTILE_SPLIT is 0. Points outside the source
        // rectangle belong to a sibling subtile and are that subtile's to
        // label; dropping them here is what stops a name appearing four
        // times when one data tile feeds four subtiles.
        float fx = (float)(part->pts[0] - r->src_x0) / (float)r->src_span;
        float fy = (float)(part->pts[1] - r->src_y0) / (float)r->src_span;
        if (fx >= 0.0f && fx < 1.0f && fy >= 0.0f && fy < 1.0f)
            label_add(w_label_dst, fx, fy, part->style, part->name, part->name_len);
    }
    return rs_part(ctx, part);
}

// `split` is how many levels below id.z the data is taken from. The tile id
// stays in display-zoom space; only the fetch and the source rectangle move.
static tile_state_t render_tile(tile_id_t id, uint16_t *px, int size, int split) {
    // One data tile covers 2^split subtiles per axis, so the id's low bits
    // select which quadrant of it this subtile shows.
    uint8_t  dz = (uint8_t)(id.z - split);
    uint32_t dx = (uint32_t)(id.x >> split);
    uint32_t dy = (uint32_t)(id.y >> split);
    uint32_t qx = (uint32_t)(id.x & ((1 << split) - 1));
    uint32_t qy = (uint32_t)(id.y & ((1 << split) - 1));

    // Cache, then network, then the offline floor - netsource decides.
    uint32_t got = TILE_CAP;
    bool from_net = false;
    if (!netsource_get(dz, dx, dy, w_tile, &got, &from_net))
        return TILE_NODATA;
    if (got == 0) return TILE_NODATA;

    // The fetch/inflate fingerprint that used to sit here has been removed.
    // It hashed the whole compressed payload twice on every single render -
    // two scattered passes over as much as 192 KB of PSRAM per tile - to
    // catch a clobber that the buffer-growth path between them can no longer
    // cause. Define MAP_CHECK_TILE_BUFFER to bring it back while chasing one.
#ifdef MAP_CHECK_TILE_BUFFER
    uint32_t sum_after_fetch = 0;
    for (uint32_t i = 0; i < got; i++) sum_after_fetch = sum_after_fetch * 31 + w_tile[i];
#endif

    // Sanity-check the payload before trusting it. A tile that is not gzip at
    // all means the fetch returned something other than this tile - a
    // different range, or another response's body - which is a transport
    // problem wearing an inflate problem's clothes.
    if (got < 18 || w_tile[0] != 0x1F || w_tile[1] != 0x8B) {
        // %ld, not %d: these are int32_t, which is 'long int' on RISC-V and
        // 'int' on xtensa. IDF compiles with -Werror=format and catches it;
        // Arduino does not, which is why it survived this long.
        Serial.printf("map: %u/%ld/%ld payload is not gzip "
                      "(%lu bytes, starts %02X %02X)\n",
                      (unsigned)dz, (long)dx, (long)dy, (unsigned long)got,
                      got > 0 ? w_tile[0] : 0, got > 1 ? w_tile[1] : 0);
        return TILE_ERROR;
    }

    // Grow the inflate buffer to whatever this tile actually needs, before
    // trying. The gzip trailer states it exactly, so there is no reason to
    // fail first and find out afterwards.
    uint32_t need = gzip_isize(w_tile, got);
    if (need > MVT_CAP) {
        if (need > MVT_CAP_MAX) {
            Serial.printf("map: tile inflates to %lu KB, past the %lu KB limit\n",
                          (unsigned long)(need / 1024),
                          (unsigned long)(MVT_CAP_MAX / 1024));
            return TILE_ERROR;
        }
        uint32_t want = need + need / 4;          // headroom for the next one
        uint8_t *bigger = (uint8_t *)ps_malloc(want);
        if (!bigger) {
            Serial.printf("map: cannot grow inflate buffer to %lu KB "
                          "(%u KB PSRAM free) - lower SUBTILE_PX\n",
                          (unsigned long)(want / 1024),
                          (unsigned)(ESP.getFreePsram() / 1024));
            return TILE_ERROR;
        }
        free(w_mvt);
        w_mvt = bigger;
        MVT_CAP = want;
        Serial.printf("map: inflate buffer -> %lu KB (%u KB PSRAM left)\n",
                      (unsigned long)(want / 1024),
                      (unsigned)(ESP.getFreePsram() / 1024));
    }

#ifdef MAP_CHECK_TILE_BUFFER
    uint32_t sum_before_inflate = 0;
    for (uint32_t i = 0; i < got; i++) sum_before_inflate = sum_before_inflate * 31 + w_tile[i];
    if (sum_before_inflate != sum_after_fetch) {
        Serial.printf("map: BUFFER CLOBBERED between fetch and inflate "
                      "(%08lx -> %08lx) for %u/%ld/%ld\n",
                      (unsigned long)sum_after_fetch,
                      (unsigned long)sum_before_inflate,
                      (unsigned)dz, (long)dx, (long)dy);
    }
#endif

    uint32_t mlen = MVT_CAP;
    // Tile payloads skip the CRC: corruption here shows up as visibly wrong
    // geometry rather than a wrong offset, and the check costs ~3.5x.
    inf_err_t ie = inflate_auto_fast(w_tile, got, w_mvt, &mlen);
    if (ie != INF_OK) {
        Serial.printf("map: inflate %u/%ld/%ld failed: %s (%lu compressed, "
                      "%lu expected)\n", (unsigned)dz, (long)dx, (long)dy,
                      inflate_strerror(ie),
                      (unsigned long)got, (unsigned long)need);

        // Write the payload out so it can be examined off-device. A stream
        // that fails identically every time is a decoder bug, and the only
        // way to fix one is against the bytes that trigger it - the fuzzer
        // that cleared this decoder only ever fed it zlib's own output.
        // One file per tile id, and only if not already present.
        char p[64];
        snprintf(p, sizeof p, "/fail_%u_%lu_%lu.gz", (unsigned)dz,
                 (unsigned long)dx, (unsigned long)dy);
        fs::FS *fsp = (SD_MMC.cardType() != CARD_NONE) ? (fs::FS *)&SD_MMC
                                                       : (fs::FS *)&SD;
        File chk = fsp->open(p, FILE_READ);
        bool exists = (bool)chk;
        if (chk) chk.close();
        if (!exists) {
            File f = fsp->open(p, FILE_WRITE);
            if (f) {
                f.write(w_tile, got);
                f.close();
                Serial.printf("map: wrote %s (%lu bytes) for analysis\n",
                              p, (unsigned long)got);
            }
        }
        return TILE_ERROR;
    }

    rs_t r;
    memset(&r, 0, sizeof r);
    r.px = px; r.w = r.h = size; r.extent = 4096;
    r.src_span = 4096 >> split;
    r.src_x0 = (int32_t)(qx * r.src_span);
    r.src_y0 = (int32_t)(qy * r.src_span);
    r.edges = w_edges;  r.edge_cap = EDGE_CAP;
    r.active = w_active; r.active_cap = XS_CAP;
    r.xs = w_xs; r.dirs = w_dirs; r.xs_cap = XS_CAP;
    r.cov = w_cov;
    r.styles = STYLES; r.n_styles = S_COUNT;
    r.cur_feature = -1;

    mvt_decoder_t d;
    memset(&d, 0, sizeof d);
    d.layer_cb = rl_layer;
    d.style_cb = style_lookup;
    d.part_cb  = rl_part;
    d.ctx      = &r;
    d.pt_buf   = w_pts;  d.pt_cap  = PT_CAP;
    d.val_style = w_val; d.val_cap = VAL_CAP;
    d.val_name  = w_val_name;
    d.val_name_len = w_val_name_len;

    label_reset(w_label_dst);

    rs_clear(&r, style_background());
    // Layers are drawn bottom-up by decoding once per layer with the filter
    // set. Skipped layers cost only a length-walk, so total decode work stays
    // close to a single full pass.
    for (int i = 0; i < N_DRAW_ORDER; i++) {
        g_want_layer = DRAW_ORDER[i];
        // Names are resolved only for the two layers that have any. Setting
        // name_key unconditionally would build a value-string table for
        // roads and buildings as well - the layers with the most values in
        // the tile and no use at all for their names.
        bool wants_names = w_label_dst &&
            (strcmp(g_want_layer, "pois") == 0 ||
             strcmp(g_want_layer, "places") == 0);
        d.name_key = wants_names ? "name" : nullptr;
        mvt_decode(&d, w_mvt, mlen);
        rs_flush(&r);
    }
    return TILE_READY;
}

// ---- place lookup ----------------------------------------------------------
// What the status bar names as "where you are".
//
// None of this can come from the working grid, which is the mistake the first
// version made. A place's label point is a single centroid, and the grid is
// four z14 tiles - 3.6 km of ground at this latitude. Canton's centroid sits
// somewhere in a township several times that across, so whether the name
// appeared came down to whether you happened to be driving near the middle of
// it. Widening the radius does not help: the point is not in the tiles at all,
// so there is nothing to be within range of.
//
// So places are looked up from their own tiles, at zooms where one tile is
// large enough to actually contain the centroid, and a 3x3 block around it for
// the case where you are near an edge:
//
//   locality/neighbourhood   z12   7.2 km/tile   21.7 km across the block
//   region/country           z6    460 km/tile   1390 km across
//
// Both zooms have to be ones that are actually on the card, and this is worth
// stating plainly because getting it wrong is expensive rather than merely
// wrong. The floor archive is cut at exactly WORLD_FLOOR_ZOOM - z6, one level,
// not z0..6 - so an earlier REGION_ZOOM of 5 matched no local archive at all
// and sent all nine tiles of the block to the network, where they failed.
//
// So both defaults are tied to what exists: z12 because the world download
// produces z12 archives, and WORLD_FLOOR_ZOOM because that is the floor. A
// lookup at z10 would have nothing to read.
#ifndef PLACE_ZOOM
#define PLACE_ZOOM 12
#endif
#ifndef REGION_ZOOM
#define REGION_ZOOM WORLD_FLOOR_ZOOM
#endif
static const uint8_t PLACE_SLOT = 0xFD;    // sentinel job slot

// Nine z12 tiles of a dense metro carry a lot of named points, and unlike the
// per-subtile sets these are searched rather than drawn, so the cap only has
// to be generous enough not to lose the one nearest you.
#ifndef PLACE_INDEX_MAX
#define PLACE_INDEX_MAX 160
#endif

// Positions here are absolute world coordinates in fractional tiles at the
// index's own zoom, not the 0..1 within-tile fractions the drawing sets use -
// nine tiles' worth of points have to be comparable to each other. A float
// holds z12 world coordinates to about five metres, which is far finer than a
// centroid means anything to.
struct PlaceIndex {
    uint8_t  z;
    uint16_t n;
    MapLabel v[PLACE_INDEX_MAX];
};

// Double-buffered and swapped by pointer, same reason as the tile label sets:
// the worker rebuilds one while the UI task searches the other.
static PlaceIndex  g_pibuf[4];
static PlaceIndex *g_place_idx  = &g_pibuf[0];   // z12, localities and hoods
static PlaceIndex *w_place_idx  = &g_pibuf[1];
static PlaceIndex *g_region_idx = &g_pibuf[2];   // z5, regions and countries
static PlaceIndex *w_region_idx = &g_pibuf[3];

static tile_id_t g_place_centre  = { 0, 0, 0 };
static tile_id_t g_region_centre = { 0, 0, 0 };
static bool      g_place_ok = false, g_region_ok = false;

// Decode-only sink: no rasteriser and no pixel buffer. The offset turns
// tile-local coordinates into world ones as each tile is read.
//
// `want` is a bitmask of style bytes the index will keep, and it is not an
// optimisation - it is what makes the index usable at all. A z6 block spans
// 1390 km and carries every locality in it, thousands of them, so an
// unfiltered index fills its cap on the first tile or two and aborts the
// decode - taking the handful of region and country points it exists for
// down with it, since nothing orders them first. Each index keeps only the
// ranks it is consulted for.
struct PlaceSink { PlaceIndex *idx; double ox, oy; uint32_t want; };

static int place_part(void *ctx, const mvt_part_t *part) {
    PlaceSink *ps = (PlaceSink *)ctx;
    if (!part->name || part->n_pts < 1 || part->geom != MVT_POINT) return 0;
    if (!style_is_place(part->style)) return 0;
    if (!(ps->want & (1u << part->style))) return 0;
    if (ps->idx->n >= PLACE_INDEX_MAX) return 1;      // full, stop decoding
    MapLabel *m = &ps->idx->v[ps->idx->n];
    m->fx = (float)(ps->ox + (double)part->pts[0] / 4096.0);
    m->fy = (float)(ps->oy + (double)part->pts[1] / 4096.0);
    m->style = part->style;
    uint32_t len = part->name_len;
    if (len > LABEL_TEXT_MAX - 1) len = LABEL_TEXT_MAX - 1;
    memcpy(m->text, part->name, len);
    m->text[len] = 0;
    ps->idx->n++;
    return 0;
}

// Which ranks each index is built for. The region block is searched only for
// the two coarse ranks and the z12 block only for the two fine ones, so
// anything else read into either is dead weight.
static const uint32_t PLACE_WANT_FINE =
    (1u << S_PLACE_HOOD) | (1u << S_PLACE_LOCALITY);
static const uint32_t PLACE_WANT_COARSE =
    (1u << S_PLACE_REGION) | (1u << S_PLACE_COUNTRY);

static bool load_place_tile(tile_id_t id, PlaceIndex *out, uint32_t want) {
    // Local only: see netsource_get_local(). A place block is nine tiles, and
    // nine failing range requests in a row is what stalled the worker and
    // starved the wifi driver of DMA heap.
    uint32_t got = TILE_CAP;
    if (!netsource_get_local(id.z, id.x, id.y, w_tile, &got)) return false;
    if (got < 18 || w_tile[0] != 0x1F || w_tile[1] != 0x8B) return false;
    uint32_t need = gzip_isize(w_tile, got);
    if (need > MVT_CAP) return false;
    uint32_t mlen = MVT_CAP;
    if (inflate_auto_fast(w_tile, got, w_mvt, &mlen) != INF_OK) return false;

    PlaceSink sink = { out, (double)id.x, (double)id.y, want };
    mvt_decoder_t d;
    memset(&d, 0, sizeof d);
    d.layer_cb = rl_layer;
    d.style_cb = style_lookup;
    d.part_cb  = place_part;
    d.ctx      = &sink;
    d.pt_buf   = w_pts;  d.pt_cap  = PT_CAP;
    d.val_style = w_val; d.val_cap = VAL_CAP;
    d.val_name  = w_val_name;
    d.val_name_len = w_val_name_len;
    d.name_key = "name";

    // Only the places layer is decoded. Every other layer in the tile - and at
    // z12 that is most of its bytes - is rejected by rl_layer with a
    // length-walk, so this costs a fraction of what rendering the same tile
    // would.
    g_want_layer = "places";
    mvt_decode(&d, w_mvt, mlen);
    return true;
}

// The 3x3 block, built in one go so the index is never half-populated while
// something is searching it.
static bool load_place_block(tile_id_t centre, PlaceIndex *out, uint32_t want) {
    out->z = centre.z;
    out->n = 0;
    int world = 1 << centre.z;
    int ok = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            tile_id_t id;
            id.z = centre.z;
            id.y = centre.y + dy;
            if (id.y < 0 || id.y >= world) continue;        // past a pole
            id.x = (centre.x + dx + world) % world;         // wraps at the date line
            if (load_place_tile(id, out, want)) ok++;
        }
    }
    return ok > 0;
}

// ---- render worker ---------------------------------------------------------
static void worker_task(void *arg) {
    (void)arg;
    render_job_t job;
    for (;;) {
        if (xQueueReceive(g_jobs, &job, portMAX_DELAY) != pdTRUE) continue;

        // The overview tile is rendered by the same worker, into its own
        // buffer. Nothing reads it while it is being drawn except the coarse
        // fill, which tolerates a half-updated source - it is an
        // approximation either way.
        if (job.slot == PLACE_SLOT) {
            // No pixels and no grid interaction: this only ever rebuilds one
            // of the two place indexes and swaps it in.
            bool region = (job.id.z == REGION_ZOOM);
            PlaceIndex *scratch = region ? w_region_idx : w_place_idx;
            bool ok = load_place_block(job.id, scratch,
                                       region ? PLACE_WANT_COARSE : PLACE_WANT_FINE);
            if (ok) {
                xSemaphoreTake(g_glock, portMAX_DELAY);
                if (region) { PlaceIndex *t = g_region_idx; g_region_idx = scratch; w_region_idx = t;
                              g_region_centre = job.id; g_region_ok = true; }
                else        { PlaceIndex *t = g_place_idx;  g_place_idx  = scratch; w_place_idx  = t;
                              g_place_centre = job.id;  g_place_ok = true; }
                xSemaphoreGive(g_glock);
            }
            Serial.printf("map: %s block z%u/%ld/%ld %s (%u places)\n",
                          region ? "region" : "place",
                          (unsigned)job.id.z, (long)job.id.x, (long)job.id.y,
                          ok ? "read" : "FAILED",
                          (unsigned)(ok ? scratch->n : 0));
            // Hitting the cap means the decode was cut short and the index is
            // whatever happened to come first, which is not the same as the
            // nearest. Worth saying out loud rather than quietly answering
            // from a truncated list.
            if (ok && scratch->n >= PLACE_INDEX_MAX)
                Serial.println("map: place index is full - raise PLACE_INDEX_MAX");
            continue;
        }

        if (job.slot == COARSE_SLOT) {
            uint64_t t0 = esp_timer_get_time();
            g_coarse_ok = false;
            // The overview is a placeholder that gets painted over within a
            // second; labelling it would put names on screen that jump as
            // soon as the real tile lands.
            w_label_dst = nullptr;
            tile_state_t res = render_tile(job.id, g_coarse_px, COARSE_PX, 0);
            if (res == TILE_READY) { g_coarse_id = job.id; g_coarse_ok = true; }
            g_stats.coarse_renders++;
            Serial.printf("map: overview z%u %s in %lu ms\n", job.id.z,
                          res == TILE_READY ? "rendered" : "FAILED",
                          (unsigned long)((esp_timer_get_time() - t0) / 1000));
            // Everything on screen depends on this one tile when the working
            // level is missing, so a transient failure is worth retrying
            // rather than leaving the map with no fallback until the grid
            // happens to move far enough to request a different overview.
            if (res != TILE_READY) {
                g_coarse_retry_at = esp_timer_get_time() + 10000000;  // 10 s
                g_coarse_want = job.id;
            }
            continue;
        }

        // Cheap early-out: if the grid has already moved on, skip the work
        // entirely rather than rendering something that will be discarded.
        xSemaphoreTake(g_glock, portMAX_DELAY);
        uint32_t gen_now = g_grid.generation;
        xSemaphoreGive(g_glock);
        if (job.generation != gen_now) { g_stats.dropped++; continue; }

        // Render into the spare buffer, never into the slot: the slot may be
        // showing a coarse placeholder that the UI task is blitting right now.
        w_label_dst = w_labels;
        uint64_t t0 = esp_timer_get_time();
        tile_state_t res = render_tile(job.id, g_spare, SUBTILE_PX, SUBTILE_SPLIT);
        uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);

        int committed;
        xSemaphoreTake(g_glock, portMAX_DELAY);
        if (res == TILE_READY) {
            uint16_t *recycled = nullptr;
            committed = grid_commit_swap(&g_grid, &job, res, g_spare, &recycled);
            g_spare = recycled;              // O(1) handover, no 512 KB copy
            // Labels follow the pixels or they describe the wrong ground.
            // grid_commit_swap has already rejected a stale job, so this is
            // guarded by the same check and lands under the same lock.
            if (committed && job.slot < GRID_COUNT) {
                LabelSet *old = g_labels[job.slot];
                g_labels[job.slot] = w_labels;
                w_labels = old;              // becomes the next scratch set
            }
        } else {
            committed = grid_commit(&g_grid, &job, res);
            // NODATA and ERROR leave the previous pixels in place but they
            // are no longer this tile's, so the names must not survive
            // either - a stale label over a blank slot is the worst of both.
            if (committed && job.slot < GRID_COUNT) g_labels[job.slot]->n = 0;
        }
        xSemaphoreGive(g_glock);

        if (!committed)                  g_stats.dropped++;
        else if (res == TILE_READY)    { g_stats.rendered++; g_stats.last_render_ms = ms; }
        else if (res == TILE_NODATA)     g_stats.notfound++;
        else                             g_stats.failed++;

        // A tile that does not render is the interesting case, and the status
        // line has no room for the reason. Log the first few of each kind.
        if (res != TILE_READY || !committed) {
            static uint32_t logged = 0;
            if (logged < 12) {
                logged++;
                Serial.printf("map: tile %u/%ld/%ld (data %u/%ld/%ld) -> %s%s (%lu ms)\n",
                    (unsigned)job.id.z, (long)job.id.x, (long)job.id.y,
                    (unsigned)(job.id.z - SUBTILE_SPLIT),
                    (long)(job.id.x >> SUBTILE_SPLIT),
                    (long)(job.id.y >> SUBTILE_SPLIT),
                    res == TILE_NODATA ? "NODATA" :
                    res == TILE_ERROR  ? "ERROR"  : "READY",
                    committed ? "" : " [stale, dropped]",
                    (unsigned long)ms);
            }
        }
    }
}

// ---- coarse fill -----------------------------------------------------------
// Paint a slot from the overview tile so it has something to show while the
// real render is queued. Nearest-neighbour is deliberate: the result is a
// placeholder that will be replaced within a second, and a bilinear pass over
// 262k pixels would cost more than it buys.
//
// Runs on the UI task. It writes into a slot the worker is not touching -
// the worker renders into the spare buffer and only swaps on commit - so no
// lock is needed beyond the one already held for slot metadata.
static bool coarse_fill(subtile_t *s) {
    if (!g_coarse_ok || !g_coarse_px) return false;
    if (s->id.z <= g_coarse_id.z) return false;

    int step = s->id.z - g_coarse_id.z;
    int span = 1 << step;                    // working tiles per coarse tile

    // Where this tile sits inside the coarse tile, in coarse-tile pixels.
    int32_t base_x = g_coarse_id.x * span;
    int32_t base_y = g_coarse_id.y * span;
    int32_t ox = s->id.x - base_x;
    int32_t oy = s->id.y - base_y;
    if (ox < 0 || oy < 0 || ox >= span || oy >= span) return false;  // outside

    int sub = COARSE_PX / span;              // source pixels per working tile
    if (sub <= 0) return false;
    int32_t sx0 = ox * sub, sy0 = oy * sub;

    // Integer step through the source, 16.16 fixed point.
    uint32_t inc = ((uint32_t)sub << 16) / SUBTILE_PX;
    uint16_t *dst = s->pixels;
    uint32_t sy = 0;
    for (int y = 0; y < SUBTILE_PX; y++, sy += inc) {
        const uint16_t *row = g_coarse_px + (size_t)(sy0 + (sy >> 16)) * COARSE_PX + sx0;
        uint32_t sx = 0;
        for (int x = 0; x < SUBTILE_PX; x++, sx += inc)
            *dst++ = row[sx >> 16];
    }
    return true;
}

// Fill slots that have nothing to show yet.
//
// Two constraints shape this. The scaling loop writes 512 KB per slot, so
// doing all nine on one pass would stall the UI for a noticeable fraction of
// a second - hence the per-pass budget. And the loop must not run under the
// grid lock, or it would block the worker's commit for just as long.
//
// Dropping the lock during the fill is safe because of who touches what: the
// worker only ever writes to its spare buffer, and map_update and map_draw
// both run on the UI task, so nothing can reassign a slot's pixel pointer
// while this is using it. The lock is taken only around the state fields.
static const int COARSE_FILLS_PER_PASS = 2;

// The overview serves two jobs that are worth separating, because only one of
// them is about missing data.
//
//   NODATA / ERROR - the archive has no tile here, or it failed. Permanent:
//                    without the fill these slots stay blank forever, which is
//                    what turns driving past the edge of the extract into a
//                    white screen. This is the "we don't have comprehensive
//                    world files" case and is always on.
//
//   PENDING        - the tile exists and is being rendered. Temporary, and
//                    nothing to do with data availability: a local archive
//                    removes the fetch but not the rasteriser, and a render
//                    measures ~1400 ms. A band step turns over about two of
//                    the four slots, so switching this off means roughly
//                    2.8 s of background per step instead of a soft tile.
//
// Set COARSE_FILL_PENDING to 0 to keep only the missing-data insurance. Worth
// trying once local archives cover the area, where NODATA becomes rare and the
// question is purely whether a blocky placeholder beats a blank one for the
// second or two before the real tile lands - which is a matter of taste rather
// than correctness.
#ifndef COARSE_FILL_PENDING
#define COARSE_FILL_PENDING 1
#endif

static void coarse_fill_pending() {
    // Retry a failed overview on a timer.
    if (!g_coarse_ok && g_coarse_retry_at &&
        esp_timer_get_time() > g_coarse_retry_at) {
        g_coarse_retry_at = 0;
        render_job_t j;
        j.id = g_coarse_want;
        j.slot = COARSE_SLOT;
        j.generation = 0;
        xQueueSendToFront(g_jobs, &j, 0);
        Serial.println("map: retrying overview");
    }
    if (!g_coarse_ok) return;

    for (int done = 0; done < COARSE_FILLS_PER_PASS; done++) {
        subtile_t *target = nullptr;
        subtile_t snapshot;
        tile_state_t was = TILE_EMPTY;

        xSemaphoreTake(g_glock, portMAX_DELAY);
        for (int i = 0; i < GRID_COUNT; i++) {
            tile_state_t st = g_grid.slots[i].state;
            // NODATA and ERROR need this as much as PENDING does - arguably
            // more. A pending tile is about to be replaced anyway; a tile the
            // archive could not supply stays blank forever otherwise, which
            // is what turns driving out of cached territory into a white
            // screen rather than a coarse one.
            bool gap  = (st == TILE_NODATA || st == TILE_ERROR);
            bool wants = gap || (COARSE_FILL_PENDING && st == TILE_PENDING);
            if (wants) {
                if (gap) g_stats.coarse_gap++; else g_stats.coarse_wait++;
                target = &g_grid.slots[i];
                snapshot = *target;
                was = st;
                break;
            }
        }
        xSemaphoreGive(g_glock);
        if (!target) return;

        bool ok = coarse_fill(&snapshot);

        xSemaphoreTake(g_glock, portMAX_DELAY);
        // Re-check: the worker may have committed a real render in the gap,
        // in which case the placeholder is already obsolete and must not
        // overwrite the READY state.
        if (target->state == was && target->pixels == snapshot.pixels) {
            if (ok) target->state = TILE_COARSE;
            else if (was != TILE_PENDING) {
                // No overview to draw from and no tile of its own. Leave it
                // marked so the search does not spin on it every pass.
                target->state = TILE_EMPTY;
            }
        }
        xSemaphoreGive(g_glock);

        if (!ok) return;    // outside coverage; the rest will be too
    }
}

// Queue an overview render if the grid has moved outside what the current
// overview covers. At COARSE_STEP 2 one coarse tile spans 4x4 working tiles,
// so this fires roughly once per four tile crossings.
// Force the overview to be re-rendered even though its tile id has not
// changed - used when the palette changes, where the id is right but the
// pixels are the wrong colour.
static void invalidate_coarse() {
    g_coarse_id = (tile_id_t){ 0, 0, 0 };
}

static void ensure_coarse(tile_id_t origin) {
    int cz = (int)origin.z - COARSE_STEP;
    if (cz < 0) cz = 0;
    int step = origin.z - cz;
    // Centre of the grid, not its corner, so the overview stays useful for
    // the whole canvas rather than drifting off one edge of it.
    int32_t cx = origin.x + GRID_N / 2, cy = origin.y + GRID_N / 2;
    tile_id_t want = { (uint8_t)cz, cx >> step, cy >> step };

    if (g_coarse_ok && want.z == g_coarse_id.z &&
        want.x == g_coarse_id.x && want.y == g_coarse_id.y) return;

    render_job_t j;
    j.id = want;
    j.slot = COARSE_SLOT;
    j.generation = 0;                        // never stale; not a grid slot
    g_coarse_want = want;
    g_coarse_retry_at = 0;
    // Front of the queue: with the working level missing, this tile is the
    // only thing standing between the user and a blank screen.
    xQueueSendToFront(g_jobs, &j, 0);
}

// ---- setup -----------------------------------------------------------------
// Scratch placement.
//
// Internal SRAM is faster than PSRAM for the scattered, non-streaming access
// the rasteriser does - but it is also the only memory mbedTLS and the AES
// DMA engine can use, and http_range opens a fresh TLS connection per tile,
// so a handshake is the peak internal-heap moment in the whole program.
// Taking 47 KB here starved it: AES could not allocate a DMA descriptor and
// every remote fetch failed.
//
// So only the buffer that actually earns it goes internal. w_cov is read and
// written several times per output pixel; everything else is touched once per
// edge, crossing or point, which measured as noise against 570k spans. The
// ranking is deliberate, not incidental:
//
//   w_cov     2.5 KB   several times per pixel   <- worth internal SRAM
//   w_val     1.0 KB   per value-table entry     <- free, rides along
//   w_xs     16.0 KB   per crossing per subsample
//   w_dirs    4.0 KB   per crossing per subsample
//   w_active  8.0 KB   per edge per subsample
//   w_pts    16.0 KB   per point                 <- measured as noise
//
// MAP_SCRATCH_INTERNAL_KB raises the budget for a build with heap to spare;
// the reserve below is what stops it ever eating the TLS handshake again.
#ifndef MAP_SCRATCH_INTERNAL_KB
#define MAP_SCRATCH_INTERNAL_KB 4
#endif

// Internal free heap that must survive any scratch allocation, sized for a
// TLS handshake plus the AES DMA descriptors it needs underneath.
#ifndef MAP_INTERNAL_RESERVE
#define MAP_INTERNAL_RESERVE (110 * 1024)
#endif

static size_t g_scratch_internal = 0;

// Take internal SRAM only while the budget and the reserve both allow it.
// Falling back to PSRAM is always correct, only slower.
static void *alloc_fast(size_t n) {
    size_t budget = (size_t)MAP_SCRATCH_INTERNAL_KB * 1024;
    if (g_scratch_internal + n <= budget) {
        size_t freeInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (freeInt > n + MAP_INTERNAL_RESERVE) {
            void *p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (p) { g_scratch_internal += n; return p; }
        }
    }
    return ps_malloc(n);
}

static bool alloc_all() {
    w_tile   = (uint8_t *)ps_malloc(TILE_CAP);
    w_mvt    = (uint8_t *)ps_malloc(MVT_CAP);
    w_edges  = (rs_edge_t *)ps_malloc(EDGE_CAP * sizeof(rs_edge_t));

    // Requested hottest-first, so the budget is spent where it pays.
    w_cov    = (uint16_t *)alloc_fast(SUBTILE_PX * sizeof(uint16_t));
    w_val    = (uint8_t *)alloc_fast(VAL_CAP);
    w_xs     = (int32_t *)alloc_fast(XS_CAP * sizeof(int32_t));
    w_dirs   = (int8_t *)alloc_fast(XS_CAP);
    w_active = (uint16_t *)alloc_fast(XS_CAP * sizeof(uint16_t));
    w_pts    = (int32_t *)alloc_fast(PT_CAP * 2 * sizeof(int32_t));

    // Label value tables. Only pois, places and the region pass ever ask the
    // decoder to fill these, but they are sized like val_style because the
    // decoder indexes them identically.
    w_val_name     = (const char **)alloc_fast(VAL_CAP * sizeof(const char *));
    w_val_name_len = (uint16_t *)alloc_fast(VAL_CAP * sizeof(uint16_t));

    // rs_clear now leaves the coverage buffer zeroed and the fillers rely on
    // finding it that way, so it must not start life as uninitialised heap.
    if (w_cov) memset(w_cov, 0, SUBTILE_PX * sizeof(uint16_t));

    Serial.printf("map: scratch %u KB internal, %u KB internal heap free "
                  "(largest DMA block %u KB)\n",
                  (unsigned)(g_scratch_internal / 1024),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_DMA) / 1024));

    return w_tile && w_mvt && w_pts && w_val &&
           w_edges && w_active && w_xs && w_dirs && w_cov;
}

bool map_begin(const char *path, uint8_t zoom, int worker_core, int worker_prio) {
    if (!alloc_all()) { Serial.println("map: PSRAM alloc failed"); return false; }

    // 9 tile buffers, allocated once and never freed. Ownership rotates
    // between slots on every shift; the count stays constant for the life of
    // the program.
    // One extra buffer beyond the nine: the worker renders into it and swaps
    // it into a slot on commit, taking the slot's old buffer as the next
    // spare. Allocation count stays fixed at ten for the life of the program.
    labels_init();
    g_spare = (uint16_t *)ps_malloc((size_t)SUBTILE_PX * SUBTILE_PX * 2);
    g_coarse_px = (uint16_t *)ps_malloc((size_t)COARSE_PX * COARSE_PX * 2);
    if (!g_spare || !g_coarse_px) {
        Serial.printf("map: spare/overview alloc failed at SUBTILE_PX=%d\n",
                      SUBTILE_PX);
        Serial.println("     lower SUBTILE_PX in mapconfig.h (768, or 512)");
        return false;
    }

    static uint16_t *bufs[GRID_COUNT];
    for (int i = 0; i < GRID_COUNT; i++) {
        bufs[i] = (uint16_t *)ps_malloc((size_t)SUBTILE_PX * SUBTILE_PX * 2);
        if (!bufs[i]) {
            Serial.printf("map: tile buffer %d/%d failed at SUBTILE_PX=%d "
                          "(%u KB free)\n", i, GRID_COUNT, SUBTILE_PX,
                          (unsigned)(ESP.getFreePsram() / 1024));
            Serial.println("     lower SUBTILE_PX in mapconfig.h (768, or 512)");
            return false;
        }
        for (int p = 0; p < SUBTILE_PX * SUBTILE_PX; p++)
            bufs[i][p] = rs_rgb(228, 226, 220);
    }

    if (!netsource_begin(path)) { Serial.println("map: netsource init failed"); return false; }

    g_zoom = zoom;

    style_init(SUBTILE_PX, 0);

    g_glock = xSemaphoreCreateMutex();
    g_jobs  = xQueueCreate(32, sizeof(render_job_t));
    if (!g_glock || !g_jobs) return false;

    tile_id_t c = { g_zoom, 0, 0 };
    grid_init(&g_grid, bufs, c);
    g_centred = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        worker_task, "tilerender", 12288, nullptr, worker_prio, nullptr, worker_core);
    if (ok != pdPASS) { Serial.println("map: worker task failed"); return false; }

    Serial.printf("map: display z%u from z%u data, %dpx subtiles "
                  "(%dx%d per data tile), %u KB PSRAM left\n",
                  g_zoom, g_zoom - SUBTILE_SPLIT, SUBTILE_PX,
                  SPLIT_N, SPLIT_N, (unsigned)(ESP.getFreePsram() / 1024));
    return true;
}

// Hold the marker inside a band in the middle of the visible area.
//
// The view only moves when the marker would leave the band, and then only far
// enough to keep it on the boundary. Inside the band nothing moves at all,
// which is what stops fix noise from shuffling the map.
//
// Everything is in world units (fractional tiles); the band is converted from
// pixels once, here.
static void view_follow() {
    if (g_headless) return;
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;
    const double visH = visBot - visTop;

    // Band edges, as offsets from the top-left of the visible area.
    double bx = SW * (1.0 - MARKER_BAND) / 2.0;
    double by = visH * (1.0 - MARKER_BAND) / 2.0;
    double bxLo = bx, bxHi = SW - bx;
    double byLo = by, byHi = visH - by;

    if (!g_view_set) {
        g_view_wx = g_marker_wx - (SW / 2.0) / SUBTILE_PX;
        g_view_wy = g_marker_wy - (visH / 2.0) / SUBTILE_PX;
        g_view_set = true;
        return;
    }

    // Leaving the band moves the view one whole band width, so the marker
    // lands on the opposite third rather than on the line it just crossed.
    //
    // Clamping to loX/hiX instead - which is what this did - pins the marker
    // to the band edge and then drags the view by exactly its movement on
    // every subsequent fix. Walking a marker steadily across the map, that is
    // a view move on essentially every fix (1983 out of 2000 in simulation)
    // against 56 for a discrete step. Both cover the same ground; one does it
    // in ~12 px dribbles and the other in 422 px steps.
    //
    // The cost is not cosmetic. A view move means a repaint, and a full-region
    // blit has been measured at 100-220 ms - so the clamp spends most of every
    // GPS second redrawing the screen to shift it by a few pixels.
    //
    // Stepping across also puts a third of a screen of new map ahead of the
    // direction of travel, and leaves the marker a third from either line so
    // it takes real movement rather than noise to trigger the next step.
    double loX = g_marker_wx - bxHi / SUBTILE_PX;
    double hiX = g_marker_wx - bxLo / SUBTILE_PX;
    if (g_view_wx < loX) g_view_wx = hiX;     // crossed the far line, step over
    if (g_view_wx > hiX) g_view_wx = loX;

    double loY = g_marker_wy - byHi / SUBTILE_PX;
    double hiY = g_marker_wy - byLo / SUBTILE_PX;
    if (g_view_wy < loY) g_view_wy = hiY;
    if (g_view_wy > hiY) g_view_wy = loY;

    // The canvas is finite, so the view cannot wander past its edges however
    // the band would like it to. This takes priority - showing background is
    // worse than the marker briefly leaving the band, and the grid shift that
    // follows will restore it.
    xSemaphoreTake(g_glock, portMAX_DELAY);
    double ox = g_grid.origin.x, oy = g_grid.origin.y;
    xSemaphoreGive(g_glock);

    double maxX = ox + GRID_N - (double)SW / SUBTILE_PX;
    double maxY = oy + GRID_N - visH / SUBTILE_PX;
    if (g_view_wx < ox) g_view_wx = ox;
    if (g_view_wx > maxX) g_view_wx = maxX;
    if (g_view_wy < oy) g_view_wy = oy;
    if (g_view_wy > maxY) g_view_wy = maxY;
}

// ---- job dispatch ----------------------------------------------------------
static void enqueue(render_job_t *jobs, int n) {
    for (int i = 0; i < n; i++) {
        if (xQueueSend(g_jobs, &jobs[i], 0) == pdTRUE) g_stats.queued++;
        // A full queue means the worker is far behind; the tiles it is about
        // to finish are stale anyway, so dropping the newest is wrong.
        // Instead drain one stale entry and retry once.
        else {
            render_job_t junk;
            if (xQueueReceive(g_jobs, &junk, 0) == pdTRUE) {
                g_stats.dropped++;
                if (xQueueSend(g_jobs, &jobs[i], 0) == pdTRUE) g_stats.queued++;
            }
        }
    }
}

static void recentre(const GnssFix &fix) {
    merc_pt_t p = merc_from_ll(fix.lat, fix.lon, g_zoom);
    // Anchor by origin, which centres the canvas on the marker for odd and
    // even grids alike - an even grid has no middle tile to sit inside.
    tile_id_t c = { g_zoom, grid_origin_for(p.x), grid_origin_for(p.y) };

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_set_zoom(&g_grid, c, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    ensure_coarse(c);
    enqueue(jobs, n);
    g_centred = true;
    // A fresh grid means the previous view may sit outside it entirely.
    g_view_set = false;
    g_stats.shifts++;
}

// Ask the worker for whichever place blocks the marker has moved out of.
//
// The centre tile is what is tracked, not the block: as long as the marker is
// still inside the centre tile it is at least a full tile from any edge of the
// block, so everything within reach is already indexed.
static void ensure_place_blocks(double wx, double wy) {
    static int64_t next_try = 0;
    int64_t now = esp_timer_get_time();

    struct { uint8_t z; bool ok; tile_id_t have; } want[2] = {
        { PLACE_ZOOM,  g_place_ok,  g_place_centre  },
        { REGION_ZOOM, g_region_ok, g_region_centre },
    };

    for (int i = 0; i < 2; i++) {
        double scale = pow(2.0, (double)want[i].z - (double)g_zoom);
        tile_id_t c;
        c.z = want[i].z;
        c.x = (int32_t)floor(wx * scale);
        c.y = (int32_t)floor(wy * scale);
        if (want[i].ok && c.x == want[i].have.x && c.y == want[i].have.y &&
            c.z == want[i].have.z) continue;

        // One rebuild at a time, and not in a hot loop. The centre is only
        // recorded on success, so a block that cannot be read - no archive
        // covers it - would otherwise be requeued on every fix.
        if (now < next_try) return;
        next_try = now + 20000000;          // 20 s

        render_job_t j;
        j.id = c;
        j.slot = PLACE_SLOT;
        j.generation = 0;
        xQueueSend(g_jobs, &j, 0);
        return;
    }
}

// Nearest named place to the marker, per rank.
//
// Nearest-centroid, not point-in-polygon: near a boundary this can name the
// neighbour, and the only real fix is boundary geometry the basemap carries
// unnamed. Ranks are resolved independently - the schema gives points, not
// containment, so there is no hierarchy to walk and "nearest hood" and
// "nearest city" are separate answers that usually agree.
static void search_index(const PlaceIndex *idx, double wx, double wy,
                         double *best, const MapLabel **pick)
{
    if (!idx || idx->n == 0) return;
    double scale = pow(2.0, (double)idx->z - (double)g_zoom);
    double px = wx * scale, py = wy * scale;
    // Radii are quoted in tiles at the working zoom, so they have to come
    // across into this index's units too.
    double hood = PLACE_HOOD_RADIUS * scale, loc = PLACE_LOCALITY_RADIUS * scale;

    for (int k = 0; k < idx->n; k++) {
        const MapLabel *m = &idx->v[k];
        int slot; double lim;
        switch (m->style) {
        case S_PLACE_HOOD:     slot = PL_HOOD;     lim = hood; break;
        case S_PLACE_LOCALITY: slot = PL_LOCALITY; lim = loc;  break;
        case S_PLACE_REGION:   slot = PL_REGION;   lim = 0;    break;
        case S_PLACE_COUNTRY:  slot = PL_COUNTRY;  lim = 0;    break;
        default: continue;
        }
        double d2 = ((double)m->fx - px) * ((double)m->fx - px) +
                    ((double)m->fy - py) * ((double)m->fy - py);
        if (lim > 0 && d2 > lim * lim) continue;
        if (d2 < best[slot]) { best[slot] = d2; pick[slot] = m; }
    }
}

static void update_place_names(double wx, double wy) {
    double best[PL_COUNT] = { 1e18, 1e18, 1e18, 1e18 };
    const MapLabel *pick[PL_COUNT] = { nullptr, nullptr, nullptr, nullptr };

    xSemaphoreTake(g_glock, portMAX_DELAY);
    const PlaceIndex *pi = g_place_ok  ? g_place_idx  : nullptr;
    const PlaceIndex *ri = g_region_ok ? g_region_idx : nullptr;
    xSemaphoreGive(g_glock);

    // Both indexes are searched for every rank. A z12 block usually carries
    // the region point too, and when it does it is the better source - it is
    // the same feature, but a block that moves every 7 km beats one that
    // moves every 2000.
    search_index(pi, wx, wy, best, pick);
    search_index(ri, wx, wy, best, pick);

    // Each rank keeps its last value when nothing qualifies, so open country
    // between towns does not flicker between "Dexter, Michigan" and
    // "Michigan" every few tiles.
    for (int r = 0; r < PL_COUNT; r++) {
        if (!pick[r]) continue;
        strncpy(g_place[r], pick[r]->text, LABEL_TEXT_MAX - 1);
        g_place[r][LABEL_TEXT_MAX - 1] = 0;
    }
    // The neighbourhood is the exception, cleared rather than held: its radius
    // is small by design, so a stale one claims a district you have left.
    if (!pick[PL_HOOD]) g_place[PL_HOOD][0] = 0;
}

void map_update(const GnssFix &fix) {
    if (!gnss_coarse(fix)) return;

    merc_pt_t p = merc_from_ll(fix.lat, fix.lon, g_zoom);

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t centre = g_grid.origin;
    xSemaphoreGive(g_glock);

    if (!g_centred) { recentre(fix); return; }

    g_marker_wx = p.x;
    g_marker_wy = p.y;
    view_follow();
    ensure_place_blocks(p.x, p.y);
    update_place_names(p.x, p.y);

    int dx, dy;
    xSemaphoreTake(g_glock, portMAX_DELAY);
    grid_drift(&g_grid, p.x, p.y, &dx, &dy);
    xSemaphoreGive(g_glock);
    if (!dx && !dy) return;

    // A jump of more than one tile - tunnel exit, cold relocate - is cheaper
    // to handle as a full recentre than as repeated single-step shifts.
    double mid = (double)GRID_N / 2.0;
    double rx = p.x - ((double)centre.x + mid), ry = p.y - ((double)centre.y + mid);
    if (rx < -1.5 || rx > 1.5 || ry < -1.5 || ry > 1.5) { recentre(fix); return; }

    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    int n = grid_shift(&g_grid, dx, dy, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t nc = g_grid.origin;
    xSemaphoreGive(g_glock);
    ensure_coarse(nc);

    enqueue(jobs, n);
    g_stats.shifts++;
}

void map_set_zoom(uint8_t zoom, const GnssFix &fix) {
    if (zoom == g_zoom) return;
    g_zoom = zoom;
    if (gnss_coarse(fix)) recentre(fix);
}

// ---- compositing -----------------------------------------------------------
static bool g_visible = true;
static bool g_force_redraw = true;

// Anything that changes the image without changing the grid or the marker -
// a palette switch, a wake from screen-off, the status bar being cleared -
// has to say so, or the signature check will conclude nothing happened.
void map_invalidate() { g_force_redraw = true; }

// Toggling labels is a repaint, not a re-render. That is the whole reason the
// overlay exists: the tile bitmaps underneath are unchanged, so turning names
// off costs one blit rather than the ~1400 ms per tile a rasterise takes.
void map_set_labels(bool on) {
    if (g_labels_on == on) return;
    g_labels_on = on;
    g_force_redraw = true;
    Serial.printf("map: labels %s\n", on ? "on" : "off");
}
bool map_labels_on() { return g_labels_on; }

void map_set_pins(bool on) {
    if (g_pins_on == on) return;
    g_pins_on = on;
    g_force_redraw = true;
    Serial.printf("map: pins %s\n", on ? "on" : "off");
}
bool map_pins_on() { return g_pins_on; }

bool map_place_text(char *out, size_t cap) {
    if (!out || cap == 0) return false;
    out[0] = 0;
    size_t used = 0;
    for (int r = 0; r < PL_COUNT; r++) {
        if (r == PL_COUNTRY && !PLACE_SHOW_COUNTRY) continue;
        // Skip a rank that repeats the one before it. City-states and a
        // handful of regions carry the same name as their country, and
        // "Singapore, Singapore" reads as a bug.
        if (!g_place[r][0]) continue;
        bool dup = false;
        for (int q = 0; q < r; q++)
            if (g_place[q][0] && strcmp(g_place[q], g_place[r]) == 0) dup = true;
        if (dup) continue;

        int n = snprintf(out + used, cap - used, "%s%s",
                         used ? ", " : "", g_place[r]);
        if (n < 0 || (size_t)n >= cap - used) { out[cap - 1] = 0; break; }
        used += (size_t)n;
    }
    return out[0] != 0;
}

void map_set_visible(bool visible) {
    g_visible = visible;
    if (visible) g_force_redraw = true;
}

void map_set_dark(bool dark) {
    if ((bool)style_is_dark() == dark) return;
    style_init(SUBTILE_PX, dark ? 1 : 0);

    // Colours are chosen during rasterisation, so nothing already drawn can
    // be reused. Invalidate the whole grid and the overview; the coarse
    // fallback covers the gap, though it is itself stale until it re-renders.
    render_job_t jobs[GRID_COUNT];
    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t o = g_grid.origin;
    int n = grid_set_zoom(&g_grid, o, jobs, GRID_COUNT);
    xSemaphoreGive(g_glock);

    // Deliberately NOT clearing g_coarse_ok.
    //
    // The overview is the only thing standing between the user and an empty
    // screen while the working tiles re-render, and discarding it here meant
    // the palette switch had nothing at all to draw. A second of map in the
    // outgoing colours is a far better failure than a second of blank - and
    // it is longer than a second whenever a background fetch is holding the
    // archive lock.
    invalidate_coarse();
    ensure_coarse(o);
    enqueue(jobs, n);
    g_force_redraw = true;
    Serial.printf("map: switched to %s palette, re-rendering %d tiles\n",
                  dark ? "night" : "day", n);
}

bool map_is_dark() {
    // No screen to sample, so nothing to be dark about.
    if (g_headless) return false;
    return style_is_dark();
}

// Push one screen rectangle from whichever tiles cover it.
//
// Everything drawn from the tile buffers goes through here: a full repaint is
// just the whole visible area, and erasing the marker is the two small rects
// it moved between. Sending only the intersection matters - a 1280px tile
// pushed whole is 3.3 MB, and doing that for a 70-pixel dot would cost more
// than the entire visible area.
// Fill rect A clipped to rect B. Degenerate or non-overlapping inputs draw
// nothing, which lets callers pass computed bands without pre-checking them.
static inline void fill_isect(int32_t ax, int32_t ay, int32_t aw, int32_t ah,
                              int32_t bx, int32_t by, int32_t bw, int32_t bh,
                              uint16_t col)
{
    int32_t x0 = ax > bx ? ax : bx;
    int32_t y0 = ay > by ? ay : by;
    int32_t x1 = (ax + aw < bx + bw) ? ax + aw : bx + bw;
    int32_t y1 = (ay + ah < by + bh) ? ay + ah : by + bh;
    if (x0 < x1 && y0 < y1) M5.Display.fillRect(x0, y0, x1 - x0, y1 - y0, col);
}

static void blit_region(int32_t rx, int32_t ry, int32_t rw, int32_t rh,
                        int32_t canvas_x, int32_t canvas_y)
{
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    if (rx < 0) { rw += rx; rx = 0; }
    if (ry < visTop) { rh -= visTop - ry; ry = visTop; }
    if (rx + rw > SW) rw = SW - rx;
    if (ry + rh > visBot) rh = visBot - ry;
    if (rw <= 0 || rh <= 0) return;

    const uint16_t bg = style_background();

    // Background is painted into the gaps only, not under everything.
    //
    // This used to wipe the whole region whenever any slot was absent and
    // then paint the tiles back over it. That writes most pixels twice, and
    // because there is no framebuffer behind the panel the wipe is visible:
    // one missing tile made the entire map flash to background on every
    // repaint. Filling just the uncovered parts keeps the reason it existed -
    // gaps must not keep their previous contents, which is what made the
    // marker smear - without touching pixels a tile is about to cover.
    xSemaphoreTake(g_glock, portMAX_DELAY);
    M5.Display.startWrite();

    // The grid's footprint on screen. Anything in the region outside it is
    // covered by no tile at all and has to be background.
    const int32_t gx0 = -canvas_x;
    const int32_t gy0 = visTop - canvas_y;
    const int32_t gx1 = gx0 + (int32_t)GRID_N * SUBTILE_PX;
    const int32_t gy1 = gy0 + (int32_t)GRID_N * SUBTILE_PX;

    fill_isect(rx, ry, gx0 - rx, rh, rx, ry, rw, rh, bg);              // left
    fill_isect(gx1, ry, rx + rw - gx1, rh, rx, ry, rw, rh, bg);        // right
    fill_isect(rx, ry, rw, gy0 - ry, rx, ry, rw, rh, bg);              // above
    fill_isect(rx, gy1, rw, ry + rh - gy1, rx, ry, rw, rh, bg);        // below

    // Then the individual slots that have nothing to show yet.
    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            if (tile_drawable(g_grid.slots[i].state)) continue;
            fill_isect(c * SUBTILE_PX - canvas_x,
                       visTop + r * SUBTILE_PX - canvas_y,
                       SUBTILE_PX, SUBTILE_PX,
                       rx, ry, rw, rh, bg);
        }
    }

    for (int r = 0; r < GRID_N; r++) {
        for (int c = 0; c < GRID_N; c++) {
            int i = r * GRID_N + c;
            if (!tile_drawable(g_grid.slots[i].state)) continue;

            // Tile's top-left in screen coordinates.
            int32_t sx = c * SUBTILE_PX - canvas_x;
            int32_t sy = visTop + r * SUBTILE_PX - canvas_y;

            int32_t dx0 = sx > rx ? sx : rx;
            int32_t dy0 = sy > ry ? sy : ry;
            int32_t dx1 = sx + SUBTILE_PX < rx + rw ? sx + SUBTILE_PX : rx + rw;
            int32_t dy1 = sy + SUBTILE_PX < ry + rh ? sy + SUBTILE_PX : ry + rh;
            if (dx0 >= dx1 || dy0 >= dy1) continue;

            int32_t w = dx1 - dx0, h = dy1 - dy0;
            int32_t srcx = dx0 - sx, srcy = dy0 - sy;
            const uint16_t *px = g_grid.slots[i].pixels;

            M5.Display.setAddrWindow(dx0, dy0, w, h);
            for (int32_t y = 0; y < h; y++)
                M5.Display.writePixels(px + (size_t)(srcy + y) * SUBTILE_PX + srcx,
                                       w, BLIT_SWAP_BYTES);
        }
    }
    M5.Display.endWrite();
    xSemaphoreGive(g_glock);
}

static void draw_marker(const GnssFix &fix, int mx, int my) {
    if (g_headless) return;
    if (!gnss_coarse(fix)) return;
    uint16_t col = gnss_fine(fix) ? M5.Display.color565(30, 90, 220)
                                  : M5.Display.color565(150, 150, 160);
    if (fix.speedKmh > 3.0) {
        float a = (fix.course - 90.0f) * 0.017453292f;
        M5.Display.drawLine(mx, my, mx + (int)(26 * cosf(a)),
                            my + (int)(26 * sinf(a)), col);
    }
    M5.Display.fillCircle(mx, my, 9, col);
    M5.Display.drawCircle(mx, my, 9, TFT_WHITE);
    M5.Display.drawCircle(mx, my, 10, TFT_WHITE);
}

// ---- label overlay ---------------------------------------------------------
// Drawn after the tiles and before the marker, so a name never covers the
// position dot.
//
// Collision culling is a linear scan against the boxes already placed, which
// is O(n^2) in labels drawn - fine at the few dozen that survive, and the
// alternative (a grid index) would cost more to maintain than it saves. Rank
// order matters more than the algorithm: places are placed before POIs, so a
// city name wins its pixels over a petrol station rather than the other way
// round.
#ifndef LABEL_MAX_ON_SCREEN
#define LABEL_MAX_ON_SCREEN 40
#endif

struct LabelBox { int16_t x, y, w, h; };

static int label_rank(uint8_t st) {
    switch (st) {
    case S_PLACE_COUNTRY:  return 0;
    case S_PLACE_REGION:   return 1;
    case S_PLACE_LOCALITY: return 2;
    case S_PLACE_HOOD:     return 3;
    default:               return 4;    // S_POI
    }
}

// Real typefaces, not the 6x8 GLCD face scaled up.
//
// setTextSize() integer-multiplies a bitmap font: at size 2 every stem is
// exactly 2 px and every curve is a staircase. That is legible on a 320x240
// panel where the alternative is unreadably small, and it looks like exactly
// what it is on a 1280x720 one - which is the panel this runs on, at roughly
// 1:1 with SUBTILE_PX, so there is no scaling anywhere else in the pipeline
// to hide behind.
//
// The FreeSans faces ship with LovyanGFX and are true proportional fonts with
// their own hinted bitmaps per size, so stems stay one weight and the
// diagonals are properly shaped rather than stepped. Bold throughout: these
// sit on top of a busy map, and regular weight loses to a road casing.
static const lgfx::IFont *label_font(uint8_t st) {
    switch (st) {
    case S_PLACE_COUNTRY:  return &fonts::FreeSansBold18pt7b;
    case S_PLACE_REGION:   return &fonts::FreeSansBold18pt7b;
    case S_PLACE_LOCALITY: return &fonts::FreeSansBold12pt7b;
    case S_PLACE_HOOD:     return &fonts::FreeSans9pt7b;
    default:               return &fonts::FreeSans12pt7b;   // S_POI
    }
}

// Halo radius in pixels. Bigger text needs a thicker outline or the outline
// reads as a printing artefact rather than a deliberate edge.
static int label_halo_r(uint8_t st) { return style_is_place(st) ? 2 : 2; }

// Ink and halo, taken from the same STYLES table the rasteriser uses, so the
// dot beside a label is the exact colour the tile would have drawn.
static uint16_t label_ink(uint8_t st)  { return STYLES[st].fill; }
static uint16_t label_halo(uint8_t st) {
    (void)st;
    // Not the background colour: the halo has to work over water, parkland
    // and building fills as well as bare earth, and the background only
    // matches one of them. Near-white on the day palette and near-black on
    // the night one is the only pair that separates from all of them.
    return map_is_dark() ? (uint16_t)0x0000 : (uint16_t)0xFFFF;
}

// The POI marker itself, now that the rasteriser no longer draws it.
//
// A ringed dot rather than the rasteriser's plain diamond: at this pixel
// density a flat blob on a building fill has no edge, and the ring is what
// makes it read as a marker rather than as map furniture.
static void draw_poi_dot(int sx, int sy, uint8_t st) {
    const uint16_t ink = label_ink(st), halo = label_halo(st);
    M5.Display.fillCircle(sx, sy, 6, halo);
    M5.Display.fillCircle(sx, sy, 5, ink);
    M5.Display.fillCircle(sx, sy, 2, halo);
}

// One label with an outline.
//
// Eight offsets, not four. The four-way version leaves the diagonals of the
// glyph bare, which at 18pt shows as a nicked edge on every A and V. Eight
// costs eight more drawString calls on text that is already laid out and
// clipped, which is nothing against the blit underneath it.
static void draw_one_label(const MapLabel *m, int sx, int sy) {
    const uint16_t ink = label_ink(m->style), halo = label_halo(m->style);
    const int r = label_halo_r(m->style);

    M5.Display.setFont(label_font(m->style));
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(middle_center);

    M5.Display.setTextColor(halo);
    for (int dy = -r; dy <= r; dy += r) {
        for (int dx = -r; dx <= r; dx += r) {
            if (!dx && !dy) continue;
            M5.Display.drawString(m->text, sx + dx, sy + dy);
        }
    }
    M5.Display.setTextColor(ink);
    M5.Display.drawString(m->text, sx, sy);
}

// `clip` limits drawing to a rectangle, for the marker-only repaint path:
// blit_region has just overwritten that rectangle, so only the labels that
// intersect it need putting back. Pass a null clip to draw everything.
static void draw_labels(int32_t canvas_x, int32_t canvas_y,
                        const LabelBox *clip)
{
    if (!g_labels_on) return;
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    LabelBox placed[LABEL_MAX_ON_SCREEN];
    int n_placed = 0;

    M5.Display.setClipRect(0, visTop, SW, visBot - visTop);

    for (int rank = 0; rank <= 4 && n_placed < LABEL_MAX_ON_SCREEN; rank++) {
        for (int i = 0; i < GRID_COUNT && n_placed < LABEL_MAX_ON_SCREEN; i++) {
            // A COARSE slot's pixels come from the overview, which was never
            // labelled - but the label set belongs to whatever rendered last
            // and would sit over the wrong ground. Only READY slots label.
            xSemaphoreTake(g_glock, portMAX_DELAY);
            bool ready = g_grid.slots[i].state == TILE_READY;
            xSemaphoreGive(g_glock);
            if (!ready) continue;

            const LabelSet *ls = g_labels[i];
            int r = i / GRID_N, c = i % GRID_N;
            int32_t tx = (int32_t)c * SUBTILE_PX - canvas_x;
            int32_t ty = visTop + (int32_t)r * SUBTILE_PX - canvas_y;

            for (int k = 0; k < ls->n && n_placed < LABEL_MAX_ON_SCREEN; k++) {
                const MapLabel *m = &ls->v[k];
                if (label_rank(m->style) != rank) continue;

                int sx = tx + (int)(m->fx * SUBTILE_PX);
                int sy = ty + (int)(m->fy * SUBTILE_PX);

                M5.Display.setFont(label_font(m->style));
                M5.Display.setTextSize(1);
                int tw = (int)M5.Display.textWidth(m->text);
                int th = (int)M5.Display.fontHeight();

                // A POI's anchor is its dot, so the text sits above it
                // rather than on top of it. Places have no dot and stay
                // centred on their own point.
                const bool poi = (m->style == S_POI);
                const int DOT_R = 6;
                int ty = poi ? sy - DOT_R - th / 2 - 3 : sy;

                // The reserved box covers the dot as well, or a neighbouring
                // label would be free to land on it.
                int top = poi ? sy - DOT_R - th - 5 : sy - th / 2 - 3;
                int bot = poi ? sy + DOT_R + 2     : sy + th / 2 + 3;
                LabelBox b = { (int16_t)(sx - tw / 2 - 5), (int16_t)top,
                               (int16_t)(tw + 10), (int16_t)(bot - top) };

                if (b.x + b.w < 0 || b.x > SW ||
                    b.y + b.h < visTop || b.y > visBot) continue;

                bool hit = false;
                for (int j = 0; j < n_placed; j++) {
                    const LabelBox *o = &placed[j];
                    if (b.x < o->x + o->w && o->x < b.x + b.w &&
                        b.y < o->y + o->h && o->y < b.y + b.h) { hit = true; break; }
                }
                if (hit) continue;

                // Reserve the space even when clipped out of this pass, or a
                // partial repaint would lay out differently from a full one
                // and labels would shuffle as the marker moves.
                placed[n_placed++] = b;

                if (clip && (b.x + b.w < clip->x || clip->x + clip->w < b.x ||
                             b.y + b.h < clip->y || clip->y + clip->h < b.y))
                    continue;

                if (poi) draw_poi_dot(sx, sy, m->style);
                draw_one_label(m, sx, ty);
            }
        }
    }

    // Hand the panel back exactly as it was found. drawStatus() and
    // drawFooter() both set a size but neither sets a font, so leaving
    // FreeSansBold18pt7b selected here would silently restyle the status bar
    // - and only after the first label was ever drawn, which is a delightful
    // bug to track down.
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_left);
    M5.Display.clearClipRect();
}

// ---- saved-point overlay ---------------------------------------------------
//
// Same clip convention as draw_labels: a null clip draws everything, a
// non-null one restricts to the rectangle blit_region has just overwritten.
//
// Drawn after the labels and before the marker, so a pin may cover a place
// name but never the position dot. The dot is the one thing on this screen
// that has to be findable at all times.
static void draw_pins(int32_t canvas_x, int32_t canvas_y, const LabelBox *clip)
{
    // The pins are positioned from the view, not the canvas, exactly as the
    // marker is - a pin is a world position, not a feature of any tile.
    (void)canvas_x; (void)canvas_y;
    if (!g_pins_on || !g_view_set) return;
    const int n = wp_count();
    if (n <= 0) return;

    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;
    const int target = wp_target();

    M5.Display.setClipRect(0, visTop, SW, visBot - visTop);
    M5.Display.setFont(&fonts::FreeSansBold9pt7b);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(middle_center);

    for (int i = 0; i < n; i++) {
        Waypoint w;
        if (!wp_get(i, &w)) continue;

        merc_pt_t p = merc_from_ll(w.lat, w.lon, g_zoom);
        int sx = (int)((p.x - g_view_wx) * SUBTILE_PX);
        int sy = visTop + (int)((p.y - g_view_wy) * SUBTILE_PX);

        // Off-screen pins are the edge arrow's job, not this one's.
        if (sx < -60 || sx > SW + 60 || sy < visTop - 60 || sy > visBot + 60)
            continue;

        // The reserved box covers the stem, the head and the name above it.
        LabelBox b = { (int16_t)(sx - 50), (int16_t)(sy - 56),
                       (int16_t)100, (int16_t)62 };
        if (clip && (b.x + b.w < clip->x || clip->x + clip->w < b.x ||
                     b.y + b.h < clip->y || clip->y + clip->h < b.y))
            continue;

        const bool active = (i == target);
        const uint16_t ink  = active ? M5.Display.color565(230, 80, 40)
                                     : M5.Display.color565(200, 40, 140);
        // The same halo pair the labels use, and for the same reason: it has
        // to separate from water, parkland and building fills alike, not just
        // from the background.
        const uint16_t halo = map_is_dark() ? (uint16_t)0x0000 : (uint16_t)0xFFFF;

        // A teardrop anchored at the point rather than centred on it: the tip
        // is what marks the position, so a circle centred there would put the
        // pin half a diameter off in whichever direction it was drawn.
        M5.Display.drawLine(sx, sy, sx, sy - 16, halo);
        M5.Display.drawLine(sx + 1, sy, sx + 1, sy - 16, halo);
        M5.Display.fillCircle(sx, sy - 22, 10, halo);
        M5.Display.fillCircle(sx, sy - 22, 8, ink);
        if (active) M5.Display.drawCircle(sx, sy - 22, 12, ink);
        M5.Display.fillCircle(sx, sy, 2, ink);

        M5.Display.setTextColor(halo);
        for (int dy = -2; dy <= 2; dy += 2)
            for (int dx = -2; dx <= 2; dx += 2)
                if (dx || dy) M5.Display.drawString(w.name, sx + dx, sy - 42 + dy);
        M5.Display.setTextColor(ink);
        M5.Display.drawString(w.name, sx, sy - 42);
    }

    // Hand the panel back as it was found, for the same reason draw_labels
    // does: the status bar sets a size but never a font.
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_left);
    M5.Display.clearClipRect();
}

// A fixed-length arrow from the marker towards the target.
//
// Bounded to GUIDE_R deliberately - see mapconfig.h. A line drawn all the way
// to the pin cannot be erased by the marker's own repaint rectangles, so it
// forced the full-screen path on every fix; this fits inside GUIDE_CLEAR_R and
// costs the same as the marker itself.
//
// This is the whole of the "directions" claim, and it is deliberately modest.
// The archive holds rendering geometry - road lines clipped per tile, with no
// node identity across tile boundaries - so there is no graph to route on. A
// bearing cannot tell you which turn to take, but it is never wrong about
// which way the point is, which is the failure that matters when the
// alternative is a confident instruction into a field.
static void draw_target_guide(const GnssFix &fix, int mx, int my) {
    if (!g_pins_on || !g_view_set) return;
    if (wp_target() < 0 || !gnss_coarse(fix)) return;

    Waypoint w;
    if (!wp_get(wp_target(), &w)) return;

    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    merc_pt_t p = merc_from_ll(w.lat, w.lon, g_zoom);
    double dx = (p.x - g_view_wx) * SUBTILE_PX - mx;
    double dy = visTop + (p.y - g_view_wy) * SUBTILE_PX - my;
    double len = sqrt(dx * dx + dy * dy);

    // Standing on it. A bearing derived from a metre of consumer-receiver
    // noise spins, and a spinning arrow reads as a fault rather than as
    // arrival - so nothing is drawn and the status bar says "here".
    if (len < 12.0) return;
    dx /= len; dy /= len;

    const uint16_t col  = M5.Display.color565(230, 80, 40);
    const uint16_t halo = map_is_dark() ? (uint16_t)0x0000 : (uint16_t)0xFFFF;
    M5.Display.setClipRect(0, visTop, SW, visBot - visTop);

    // Start clear of the marker ring, or the shaft eats the dot it is drawn
    // from. Stop short of GUIDE_R so the head has room inside the reserved
    // rectangle.
    int x0 = mx + (int)(dx * 16),          y0 = my + (int)(dy * 16);
    int x1 = mx + (int)(dx * (GUIDE_R - 16)), y1 = my + (int)(dy * (GUIDE_R - 16));

    double px = -dy, py = dx;
    int hx = mx + (int)(dx * GUIDE_R), hy = my + (int)(dy * GUIDE_R);

    // Haloed, like the labels: an orange arrow over an orange roof casing is
    // otherwise invisible at exactly the moment it is being looked for.
    for (int o = -1; o <= 1; o++) {
        int ox = (int)(px * o), oy = (int)(py * o);
        M5.Display.drawLine(x0 + ox, y0 + oy, x1 + ox, y1 + oy, halo);
    }
    M5.Display.fillTriangle(hx + (int)(dx * 2),  hy + (int)(dy * 2),
                            x1 + (int)(px * 11), y1 + (int)(py * 11),
                            x1 - (int)(px * 11), y1 - (int)(py * 11), halo);

    M5.Display.drawLine(x0, y0, x1, y1, col);
    M5.Display.drawLine(x0 + (int)px, y0 + (int)py, x1 + (int)px, y1 + (int)py, col);
    M5.Display.fillTriangle(hx, hy,
                            x1 + (int)(px * 9), y1 + (int)(py * 9),
                            x1 - (int)(px * 9), y1 - (int)(py * 9), col);

    M5.Display.clearClipRect();
}

void map_draw(const GnssFix &fix) {
    if (g_headless) return;
    if (!g_visible || !g_view_set) return;
    const int SW = M5.Display.width(), SH = M5.Display.height();
    const int visTop = STATUS_H, visBot = SH - FOOTER_H;

    coarse_fill_pending();

    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t origin = g_grid.origin;
    uint32_t gen = g_grid.generation;
    uint32_t states = 0;
    for (int i = 0; i < GRID_COUNT; i++)
        states = states * 7 + (uint32_t)g_grid.slots[i].state;
    xSemaphoreGive(g_glock);

    // View position within the canvas, and the marker's screen position.
    int32_t canvas_x = (int32_t)((g_view_wx - (double)origin.x) * SUBTILE_PX);
    int32_t canvas_y = (int32_t)((g_view_wy - (double)origin.y) * SUBTILE_PX);
    int mx = (int)((g_marker_wx - g_view_wx) * SUBTILE_PX);
    int my = visTop + (int)((g_marker_wy - g_view_wy) * SUBTILE_PX);

    static int32_t last_cx = INT32_MIN, last_cy = INT32_MIN;
    static uint32_t last_gen = 0, last_states = 0;
    static int last_mx = INT32_MIN, last_my = INT32_MIN;
    static bool have_last = false;

    bool viewMoved  = (canvas_x != last_cx || canvas_y != last_cy);
    bool tilesMoved = (gen != last_gen || states != last_states);
    bool markerMoved = (mx != last_mx || my != last_my);

    // The guide is drawn around the marker, so it stays inside the partial
    // repaint path - it only needs a larger rectangle reserved for it.
    bool guiding = (g_pins_on && wp_target() >= 0);

    if (have_last && !viewMoved && !tilesMoved && !markerMoved && !g_force_redraw)
        return;

    uint64_t t0 = esp_timer_get_time();

    // A screen with nothing drawable looks identical to a crash, so say so.
    {
        bool anyDrawable = false;
        xSemaphoreTake(g_glock, portMAX_DELAY);
        for (int i = 0; i < GRID_COUNT; i++)
            if (tile_drawable(g_grid.slots[i].state)) { anyDrawable = true; break; }
        xSemaphoreGive(g_glock);

        static uint32_t blankSince = 0;
        static uint32_t lastMoan = 0;
        if (!anyDrawable) {
            if (!blankSince) blankSince = millis();
            if (millis() - blankSince > 5000 && millis() - lastMoan > 10000) {
                lastMoan = millis();
                MapStats st; map_stats(&st);
                Serial.printf("map: nothing drawable for %lus - overview %s, "
                              "queue %lu, rendered %lu, failed %lu\n",
                              (unsigned long)((millis() - blankSince) / 1000),
                              g_coarse_ok ? "ok" : "MISSING",
                              (unsigned long)st.queue_depth,
                              (unsigned long)st.rendered,
                              (unsigned long)st.failed);
            }
        } else {
            blankSince = 0;
        }
    }

    if (!have_last || viewMoved || tilesMoved || g_force_redraw) {
        // Everything can have changed; repaint the visible area.
        blit_region(0, visTop, SW, visBot - visTop, canvas_x, canvas_y);
        draw_labels(canvas_x, canvas_y, nullptr);
        draw_pins(canvas_x, canvas_y, nullptr);
    } else {
        // Only the marker moved, which is the common case now that the view
        // sits still inside the band. Repainting its old and new
        // neighbourhoods is a few thousand pixels instead of a million.
        const int R = guiding ? GUIDE_CLEAR_R : MARKER_CLEAR_R;
        blit_region(last_mx - R, last_my - R, R * 2, R * 2, canvas_x, canvas_y);
        blit_region(mx - R, my - R, R * 2, R * 2, canvas_x, canvas_y);
        // Those two rectangles have just erased any label crossing them.
        {
            LabelBox a = { (int16_t)(last_mx - R), (int16_t)(last_my - R),
                           (int16_t)(R * 2), (int16_t)(R * 2) };
            LabelBox b = { (int16_t)(mx - R), (int16_t)(my - R),
                           (int16_t)(R * 2), (int16_t)(R * 2) };
            draw_labels(canvas_x, canvas_y, &a);
            draw_labels(canvas_x, canvas_y, &b);
            draw_pins(canvas_x, canvas_y, &a);
            draw_pins(canvas_x, canvas_y, &b);
        }
    }

    draw_target_guide(fix, mx, my);
    M5.Display.setClipRect(0, visTop, SW, visBot - visTop);
    draw_marker(fix, mx, my);
    M5.Display.clearClipRect();

    last_cx = canvas_x; last_cy = canvas_y;
    last_gen = gen; last_states = states;
    last_mx = mx; last_my = my;
    have_last = true;
    g_force_redraw = false;

    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    g_stats.last_draw_ms = ms;
    g_stats.draw_total_ms += ms;
    g_stats.draws++;
    if (ms > g_stats.max_draw_ms) g_stats.max_draw_ms = ms;
}

// ---- background fetchers ---------------------------------------------------
// These hold the archive lock for a whole tile, so they can stall the
// renderer. Yielding while the render queue has work keeps the visible map
// responsive: a tile the user is looking at always beats one being stored
// for later.
static volatile int  g_pf_progress = 0;
static volatile bool g_pf_busy = false;

static void yield_to_renderer() {
    for (int i = 0; i < 200; i++) {                 // ~10 s ceiling
        if (uxQueueMessagesWaiting(g_jobs) == 0) return;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ---- radius prefetch -------------------------------------------------------
struct PrefetchArgs {
    double lat, lon;
    int radius;
    uint8_t z1, z2;
};

static void prefetch_task(void *arg) {
    PrefetchArgs a = *(PrefetchArgs *)arg;
    free(arg);

    // Its own scratch: the render worker owns w_tile and may be using it.
    uint8_t *buf = (uint8_t *)ps_malloc(TILE_CAP);
    if (!buf) { g_pf_busy = false; vTaskDelete(nullptr); return; }

    int side = a.radius * 2 + 1;
    // Fetch at the data zoom: those are the tiles that actually get stored,
    // and asking for display-zoom ids would cache four times as many of the
    // wrong thing.
    const uint8_t zs[2] = { (uint8_t)DATA_ZOOM_OF(a.z1),
                            (uint8_t)DATA_ZOOM_OF(a.z2) };
    int levels = (zs[0] == zs[1]) ? 1 : 2;
    int total = side * side * levels;

    int done = 0, fetched = 0, cached = 0, missing = 0;
    uint64_t t0 = esp_timer_get_time();

    for (int zi = 0; zi < levels; zi++) {
        merc_pt_t p = merc_from_ll(a.lat, a.lon, zs[zi]);
        int32_t cx = (int32_t)p.x, cy = (int32_t)p.y;

        for (int dy = -a.radius; dy <= a.radius; dy++) {
            for (int dx = -a.radius; dx <= a.radius; dx++) {
                done++;
                g_pf_progress = (done * 100) / total;

                uint32_t n = TILE_CAP;
                bool from_net = false;
                if (netsource_get(zs[zi], (uint32_t)(cx + dx), (uint32_t)(cy + dy),
                                  buf, &n, &from_net)) {
                    if (from_net) fetched++; else cached++;
                } else {
                    missing++;
                }

                vTaskDelay(pdMS_TO_TICKS(5));
                yield_to_renderer();
            }
        }
        Serial.printf("prefetch: z%u done (%d fetched, %d cached, %d empty)\n",
                      zs[zi], fetched, cached, missing);
    }

    free(buf);
    Serial.printf("prefetch: %d tiles in %lu s (%d from network)\n",
                  total, (unsigned long)((esp_timer_get_time() - t0) / 1000000),
                  fetched);
    g_pf_progress = 100;
    g_pf_busy = false;
    vTaskDelete(nullptr);
}

bool map_prefetch_start(int radius, uint8_t z_wide, uint8_t z_close) {
    if (g_pf_busy || !g_centred) return false;
    PrefetchArgs *a = (PrefetchArgs *)malloc(sizeof(PrefetchArgs));
    if (!a) return false;

    // Centre on the grid rather than a fix, so this works even if the fix
    // has momentarily dropped.
    xSemaphoreTake(g_glock, portMAX_DELAY);
    tile_id_t c = g_grid.origin;
    xSemaphoreGive(g_glock);
    merc_pt_t mid = { (double)c.x + GRID_N / 2.0, (double)c.y + GRID_N / 2.0, c.z };
    merc_to_ll(mid, &a->lat, &a->lon);

    a->radius = radius; a->z1 = z_wide; a->z2 = z_close;
    g_pf_busy = true;
    g_pf_progress = 0;

    // 10 KiB: inflate builds its Huffman tables on the stack (~3.5 KiB) on
    // top of this task's own frames.
    if (xTaskCreatePinnedToCore(prefetch_task, "prefetch", 10240, a, 1, nullptr, 1)
        != pdPASS) {
        free(a); g_pf_busy = false; return false;
    }
    Serial.printf("prefetch: %d x %d tiles at z%u around %.4f,%.4f\n",
                  radius * 2 + 1, radius * 2 + 1,
                  (unsigned)DATA_ZOOM_OF(z_close), a->lat, a->lon);
    return true;
}

// ---- world floor -----------------------------------------------------------
// z0-6 is only 5461 tiles but covers the entire planet, so there is always
// something to fall back to when the working level is missing. Checkpointed,
// because at roughly one tile a second the walk takes hours and will be
// interrupted.
static void world_task(void *arg) {
    (void)arg;
    uint8_t *buf = (uint8_t *)ps_malloc(TILE_CAP);
    if (!buf) { g_pf_busy = false; vTaskDelete(nullptr); return; }

    uint32_t total = 0;
    for (int z = 0; z <= WORLD_FLOOR_ZOOM; z++) total += 1u << (2 * z);
    uint32_t done = 0, fetched = 0, cached = 0;
    uint32_t done_this_run = 0;      // rate must not count the resumed part
    int64_t started = esp_timer_get_time();
    int64_t next_report = started;
    int64_t next_ckpt = started;

    uint8_t rz = 0; uint32_t rx = 0, ry = 0;
    bool resuming = netsource_world_load_pos(&rz, &rx, &ry);
    if (resuming) {
        for (int z = 0; z < rz; z++) done += 1u << (2 * z);
        // x is the outer loop and y the inner, so the linear index within a
        // level is x * n + y - not the other way round.
        done += rx * (1u << rz) + ry;
        Serial.printf("world: resuming at z%u %lu/%lu (%lu of %lu already done)\n",
                      rz, (unsigned long)rx, (unsigned long)ry,
                      (unsigned long)done, (unsigned long)total);
    }

    for (int z = 0; z <= WORLD_FLOOR_ZOOM; z++) {
        uint32_t n = 1u << z;
        if (resuming && z < rz) continue;
        for (uint32_t x = 0; x < n; x++) {
            if (resuming && z == rz && x < rx) continue;
            for (uint32_t y = 0; y < n; y++) {
                if (resuming && z == rz && x == rx && y < ry) continue;

                uint32_t len = TILE_CAP;
                bool from_net = false;
                if (netsource_get((uint8_t)z, x, y, buf, &len, &from_net)) {
                    if (from_net) fetched++; else cached++;
                }
                done++;
                done_this_run++;
                g_pf_progress = (int)((done * 100) / total);

                int64_t now = esp_timer_get_time();
                if (now >= next_report) {
                    next_report = now + 10000000;          // every 10 s
                    // Rate over work actually done in this run: on a resume
                    // `done` starts in the thousands while elapsed starts at
                    // zero, which reported six-figure tiles per second.
                    double elapsed = (now - started) / 1e6;
                    double rate = done_this_run / (elapsed > 0.5 ? elapsed : 0.5);
                    double left = (total - done) / (rate > 0.01 ? rate : 0.01);
                    Serial.printf("world: %lu/%lu (%d%%) z%d  %.1f tiles/s  "
                                  "~%.0f min left  [%lu new, %lu cached]\n",
                                  (unsigned long)done, (unsigned long)total,
                                  g_pf_progress, z, rate, left / 60.0,
                                  (unsigned long)fetched, (unsigned long)cached);
                }

                // Checkpoint on a timer rather than a tile count: the cost is
                // one small file write, and 30 s is a bounded amount of work
                // to redo after a power cut.
                if (now >= next_ckpt) {
                    next_ckpt = now + 30000000;
                    netsource_world_save_pos((uint8_t)z, x, y);
                }

                vTaskDelay(pdMS_TO_TICKS(3));
                yield_to_renderer();
                if (WiFi.status() != WL_CONNECTED) {
                    netsource_world_save_pos((uint8_t)z, x, y);
                    Serial.printf("world: link lost at %lu/%lu, resumes here "
                                  "next boot\n",
                                  (unsigned long)done, (unsigned long)total);
                    free(buf);
                    g_pf_busy = false;
                    vTaskDelete(nullptr);
                    return;
                }
            }
        }
    }

    free(buf);
    netsource_world_mark_done();
    Serial.printf("world: floor stored in %.0f min (%lu fetched, %lu already "
                  "cached) - the map will now draw anywhere offline\n",
                  (esp_timer_get_time() - started) / 6e7,
                  (unsigned long)fetched, (unsigned long)cached);
    g_pf_progress = 100;
    g_pf_busy = false;
    vTaskDelete(nullptr);
}

bool map_world_floor_start() {
    if (g_pf_busy) return false;
    g_pf_busy = true;
    g_pf_progress = 0;
    if (xTaskCreatePinnedToCore(world_task, "worldfloor", 10240, nullptr, 1,
                                nullptr, 1) != pdPASS) {
        g_pf_busy = false;
        return false;
    }
    Serial.printf("world: storing z0-%d floor in the background\n",
                  WORLD_FLOOR_ZOOM);
    return true;
}

int  map_prefetch_progress() { return g_pf_progress; }
bool map_prefetch_busy()     { return g_pf_busy; }

uint8_t map_zoom() { return g_zoom; }
bool map_has_fix_position() { return g_centred; }

void map_stats(MapStats *out) {
    *out = g_stats;
    out->queue_depth = g_jobs ? uxQueueMessagesWaiting(g_jobs) : 0;
}
