// mvt.c - Mapbox Vector Tile decoder.

#include "mvt.h"
#include <string.h>

// ---- protobuf wire format --------------------------------------------------
// Fields we care about, by message:
//   Tile:    3 = layers (message, repeated)
//   Layer:   1 = name (string)     2 = features (message, repeated)
//            3 = keys (string, repeated)  4 = values (message, repeated)
//            5 = extent (varint)   15 = version (varint)
//   Feature: 1 = id (varint)  2 = tags (packed varint)
//            3 = type (varint) 4 = geometry (packed varint)
//   Value:   1 = string  2 = float  3 = double
//            4 = int64   5 = uint64 6 = sint64  7 = bool

// src: Protocol Buffers encoding specification, "Message Structure" - the
//      three-bit wire type in each field tag. 3 and 4 (start/end group) are
//      deprecated and never appear in MVT, so they are not defined here.
#define WT_VARINT 0
#define WT_64BIT  1
#define WT_LEN    2
#define WT_32BIT  5

typedef struct { const uint8_t *b; uint32_t p, n; int bad; } rd_t;

static uint64_t rd_varint(rd_t *r) {
    uint64_t v = 0; int s = 0;
    while (r->p < r->n) {
        uint8_t c = r->b[r->p++];
        v |= (uint64_t)(c & 0x7F) << s;
        if (!(c & 0x80)) return v;
        s += 7;
        if (s > 63) break;
    }
    r->bad = 1;
    return 0;
}

// Returns 0 at end of buffer, else 1 with *field/*wt set.
static int rd_key(rd_t *r, uint32_t *field, uint32_t *wt) {
    if (r->p >= r->n) return 0;
    uint64_t k = rd_varint(r);
    if (r->bad) return 0;
    *field = (uint32_t)(k >> 3);
    *wt    = (uint32_t)(k & 7);
    return 1;
}

// Length-delimited payload: sets *s/*e to the byte range and advances past it.
static int rd_bytes(rd_t *r, uint32_t *s, uint32_t *e) {
    uint64_t n = rd_varint(r);
    if (r->bad || n > r->n - r->p) { r->bad = 1; return 0; }
    *s = r->p; *e = r->p + (uint32_t)n;
    r->p = *e;
    return 1;
}

static int rd_skip(rd_t *r, uint32_t wt) {
    switch (wt) {
    case WT_VARINT: rd_varint(r); return !r->bad;
    case WT_64BIT:  if (r->n - r->p < 8) { r->bad = 1; return 0; } r->p += 8; return 1;
    case WT_32BIT:  if (r->n - r->p < 4) { r->bad = 1; return 0; } r->p += 4; return 1;
    case WT_LEN: {
        uint32_t s, e;
        return rd_bytes(r, &s, &e);
    }
    default: r->bad = 1; return 0;
    }
}

static int32_t zigzag(uint64_t v) {
    return (int32_t)((v >> 1) ^ (~(v & 1) + 1));
}

// ---- geometry --------------------------------------------------------------
// Command integer: (id & 0x7) | (count << 3), where id 1 = MoveTo,
// 2 = LineTo, 7 = ClosePath. Parameters are zigzag deltas from a running
// cursor that persists across commands *and* across parts.

typedef struct {
    mvt_decoder_t *d;
    mvt_part_t     part;
    uint32_t       n;          // points currently in d->pt_buf
    int32_t        first_x, first_y;
    int            aborted;
} geom_ctx_t;

static int64_t shoelace2(const int32_t *p, uint32_t n) {
    // Twice the signed area. Accumulated in int64: at extent 4096 a ring
    // would need ~10^9 points to overflow, so this is safe by a wide margin.
    int64_t a = 0;
    for (uint32_t i = 0, j = n - 1; i < n; j = i++)
        a += (int64_t)p[j * 2] * p[i * 2 + 1] - (int64_t)p[i * 2] * p[j * 2 + 1];
    return a;
}

static int flush_part(geom_ctx_t *g) {
    if (g->n == 0) return 1;
    mvt_decoder_t *d = g->d;

    g->part.pts   = d->pt_buf;
    g->part.n_pts = g->n;

    if (g->part.geom == MVT_POLYGON && g->n >= 3) {
        g->part.area2 = shoelace2(d->pt_buf, g->n);
        // Exterior rings wind clockwise in y-down tile space -> positive sum.
        g->part.is_outer = g->part.area2 > 0;
    } else {
        g->part.area2 = 0;
        g->part.is_outer = 1;
    }

    d->stat_parts++;
    d->stat_points += g->n;
    if (g->n > d->stat_max_part_pts) d->stat_max_part_pts = g->n;

    int abort = d->part_cb ? d->part_cb(d->ctx, &g->part) : 0;

    g->part.part_index++;
    g->n = 0;
    if (abort) { g->aborted = 1; return 0; }
    return 1;
}

static mvt_err_t decode_geometry(geom_ctx_t *g, const uint8_t *buf,
                                 uint32_t s, uint32_t e)
{
    mvt_decoder_t *d = g->d;
    rd_t r = { buf, s, e, 0 };
    int32_t cx = 0, cy = 0;
    g->n = 0;
    g->part.part_index = 0;

    while (r.p < r.n) {
        uint64_t cmd = rd_varint(&r);
        if (r.bad) return MVT_EFORMAT;
        uint32_t id  = (uint32_t)(cmd & 7);
        uint32_t cnt = (uint32_t)(cmd >> 3);

        if (id == 7) {                       // ClosePath
            if (g->part.geom != MVT_POLYGON) return MVT_EFORMAT;
            // The closing vertex is implicit in MVT; make it explicit so the
            // rasteriser can treat every ring as a simple point list.
            if (g->n == 0) return MVT_EFORMAT;
            if (g->n + 1 > d->pt_cap) return MVT_ENOMEM;
            d->pt_buf[g->n * 2]     = g->first_x;
            d->pt_buf[g->n * 2 + 1] = g->first_y;
            g->n++;
            if (!flush_part(g)) return MVT_EABORT;
            continue;
        }

        if (id != 1 && id != 2) return MVT_EFORMAT;

        if (id == 1) {                       // MoveTo: starts a new part
            // For POINT geometry a single MoveTo may carry many points, which
            // together form one multipoint part.
            if (!flush_part(g)) return MVT_EABORT;
        } else if (g->n == 0) {
            return MVT_EFORMAT;              // LineTo with no current point
        }

        for (uint32_t i = 0; i < cnt; i++) {
            uint64_t a = rd_varint(&r);
            uint64_t b = rd_varint(&r);
            if (r.bad) return MVT_EFORMAT;
            cx += zigzag(a);
            cy += zigzag(b);

            if (id == 1 && i > 0 && g->part.geom != MVT_POINT) {
                // A multi-point MoveTo only makes sense for POINT features;
                // for lines and polygons each MoveTo begins a separate part.
                if (!flush_part(g)) return MVT_EABORT;
            }
            if (g->n >= d->pt_cap) return MVT_ENOMEM;
            d->pt_buf[g->n * 2]     = cx;
            d->pt_buf[g->n * 2 + 1] = cy;
            g->n++;
            if (g->n == 1) { g->first_x = cx; g->first_y = cy; }
        }
    }
    if (!flush_part(g)) return MVT_EABORT;
    return MVT_OK;
}

// ---- value table -----------------------------------------------------------
// Resolve one Value message to a style byte. Only string values participate:
// the Protomaps schema keys everything visual off string `kind`.
static uint8_t value_style(mvt_decoder_t *d, const mvt_layer_t *layer,
                           const uint8_t *buf, uint32_t s, uint32_t e)
{
    if (!d->style_cb) return 0;
    rd_t r = { buf, s, e, 0 };
    uint32_t f, wt;
    while (rd_key(&r, &f, &wt)) {
        if (f == 1 && wt == WT_LEN) {
            uint32_t vs, ve;
            if (!rd_bytes(&r, &vs, &ve)) return 0;
            return d->style_cb(d->ctx, layer, (const char *)buf + vs, ve - vs);
        }
        if (!rd_skip(&r, wt)) return 0;
    }
    return 0;
}

// ---- feature ---------------------------------------------------------------
static mvt_err_t decode_feature(mvt_decoder_t *d, const mvt_layer_t *layer,
                                const uint8_t *buf, uint32_t s, uint32_t e,
                                int32_t style_key_idx, uint32_t n_values,
                                uint32_t feature_index)
{
    rd_t r = { buf, s, e, 0 };
    uint32_t f, wt;
    uint32_t geom_s = 0, geom_e = 0;
    uint32_t tags_s = 0, tags_e = 0;
    uint64_t fid = 0;
    mvt_geom_t gt = MVT_UNKNOWN;

    while (rd_key(&r, &f, &wt)) {
        if (f == 1 && wt == WT_VARINT) {
            fid = rd_varint(&r);
        } else if (f == 2 && wt == WT_LEN) {
            if (!rd_bytes(&r, &tags_s, &tags_e)) return MVT_EFORMAT;
        } else if (f == 3 && wt == WT_VARINT) {
            gt = (mvt_geom_t)rd_varint(&r);
        } else if (f == 4 && wt == WT_LEN) {
            if (!rd_bytes(&r, &geom_s, &geom_e)) return MVT_EFORMAT;
        } else if (!rd_skip(&r, wt)) {
            return MVT_EFORMAT;
        }
        if (r.bad) return MVT_EFORMAT;
    }
    if (gt == MVT_UNKNOWN || geom_e == geom_s) return MVT_OK;  // nothing to draw

    // Resolve style by scanning the tag pairs for the styling key. Tags are
    // (key_index, value_index) pairs; the pre-resolved table turns the second
    // half of the lookup into an array index.
    uint8_t style = 0;
    if (style_key_idx >= 0 && tags_e > tags_s) {
        rd_t t = { buf, tags_s, tags_e, 0 };
        while (t.p < t.n) {
            uint64_t k = rd_varint(&t);
            if (t.bad || t.p >= t.n) break;
            uint64_t v = rd_varint(&t);
            if (t.bad) break;
            if ((int32_t)k == style_key_idx) {
                if (v < n_values) style = d->val_style[v];
                break;
            }
        }
    }

    geom_ctx_t g;
    memset(&g, 0, sizeof g);
    g.d = d;
    g.part.layer  = layer;
    g.part.geom   = gt;
    g.part.style  = style;
    g.part.feature_id    = fid;
    g.part.feature_index = feature_index;

    d->stat_features++;
    return decode_geometry(&g, buf, geom_s, geom_e);
}

// ---- layer -----------------------------------------------------------------
static mvt_err_t decode_layer(mvt_decoder_t *d, const uint8_t *buf,
                              uint32_t s, uint32_t e, int index)
{
    mvt_layer_t layer;
    memset(&layer, 0, sizeof layer);
    layer.extent  = 4096;      // spec default
    layer.version = 1;
    layer.index   = index;

    // Pass 1: header scalars. Protobuf permits any field order, so the name
    // and extent may follow the features; scan the whole layer first.
    {
        rd_t r = { buf, s, e, 0 };
        uint32_t f, wt;
        while (rd_key(&r, &f, &wt)) {
            if (f == 1 && wt == WT_LEN) {
                uint32_t ns, ne;
                if (!rd_bytes(&r, &ns, &ne)) return MVT_EFORMAT;
                layer.name = (const char *)buf + ns;
                layer.name_len = ne - ns;
            } else if (f == 5 && wt == WT_VARINT) {
                layer.extent = (uint32_t)rd_varint(&r);
            } else if (f == 15 && wt == WT_VARINT) {
                layer.version = (uint32_t)rd_varint(&r);
            } else if (!rd_skip(&r, wt)) {
                return MVT_EFORMAT;
            }
            if (r.bad) return MVT_EFORMAT;
        }
        if (layer.extent == 0) return MVT_EFORMAT;
    }

    if (d->layer_cb && !d->layer_cb(d->ctx, &layer)) return MVT_OK;
    d->stat_layers++;

    // Pass 2: find the index of the styling key.
    const char *skey = d->style_key ? d->style_key : "kind";
    uint32_t skey_len = (uint32_t)strlen(skey);
    int32_t style_key_idx = -1;
    {
        rd_t r = { buf, s, e, 0 };
        uint32_t f, wt, ki = 0;
        while (rd_key(&r, &f, &wt)) {
            if (f == 3 && wt == WT_LEN) {
                uint32_t ks, ke;
                if (!rd_bytes(&r, &ks, &ke)) return MVT_EFORMAT;
                if (style_key_idx < 0 && ke - ks == skey_len &&
                    memcmp(buf + ks, skey, skey_len) == 0)
                    style_key_idx = (int32_t)ki;
                ki++;
            } else if (!rd_skip(&r, wt)) {
                return MVT_EFORMAT;
            }
        }
    }

    // Pass 3: resolve every value to a style byte, once.
    uint32_t n_values = 0;
    {
        rd_t r = { buf, s, e, 0 };
        uint32_t f, wt;
        while (rd_key(&r, &f, &wt)) {
            if (f == 4 && wt == WT_LEN) {
                uint32_t vs, ve;
                if (!rd_bytes(&r, &vs, &ve)) return MVT_EFORMAT;
                if (n_values >= d->val_cap) return MVT_ENOMEM;
                d->val_style[n_values++] = value_style(d, &layer, buf, vs, ve);
            } else if (!rd_skip(&r, wt)) {
                return MVT_EFORMAT;
            }
        }
    }
    if (n_values > d->stat_max_values) d->stat_max_values = n_values;

    // Pass 4: features.
    {
        rd_t r = { buf, s, e, 0 };
        uint32_t f, wt, fi = 0;
        while (rd_key(&r, &f, &wt)) {
            if (f == 2 && wt == WT_LEN) {
                uint32_t fs, fe;
                if (!rd_bytes(&r, &fs, &fe)) return MVT_EFORMAT;
                mvt_err_t err = decode_feature(d, &layer, buf, fs, fe,
                                               style_key_idx, n_values, fi++);
                if (err != MVT_OK) return err;
            } else if (!rd_skip(&r, wt)) {
                return MVT_EFORMAT;
            }
        }
    }
    return MVT_OK;
}

// ---- entry point -----------------------------------------------------------
mvt_err_t mvt_decode(mvt_decoder_t *d, const uint8_t *tile, uint32_t len) {
    if (!d || !tile || !d->pt_buf || !d->val_style) return MVT_EFORMAT;
    d->stat_layers = d->stat_features = d->stat_parts = 0;
    d->stat_points = d->stat_max_part_pts = d->stat_max_values = 0;

    rd_t r = { tile, 0, len, 0 };
    uint32_t f, wt;
    int index = 0;
    while (rd_key(&r, &f, &wt)) {
        if (f == 3 && wt == WT_LEN) {
            uint32_t s, e;
            if (!rd_bytes(&r, &s, &e)) return MVT_EFORMAT;
            mvt_err_t err = decode_layer(d, tile, s, e, index++);
            if (err != MVT_OK) return err;
        } else if (!rd_skip(&r, wt)) {
            return MVT_EFORMAT;
        }
    }
    return r.bad ? MVT_EFORMAT : MVT_OK;
}

const char *mvt_strerror(mvt_err_t e) {
    switch (e) {
    case MVT_OK:      return "ok";
    case MVT_EFORMAT: return "malformed tile";
    case MVT_ENOMEM:  return "scratch buffer too small";
    case MVT_EABORT:  return "aborted by callback";
    }
    return "?";
}
