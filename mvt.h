// mvt.h - Mapbox Vector Tile decoder.
//
// Spec: https://github.com/mapbox/vector-tile-spec/tree/master/2.1
//
// Streaming and allocation-free: the caller hands in the inflated tile plus
// two scratch buffers, and receives geometry through a callback, one "part"
// at a time (a ring, a linestring, or a run of points). Nothing is retained
// after the callback returns, so the rasteriser can draw straight into the
// subtile buffer without a feature list ever existing in RAM.
//
// The style hook is the memory trick that makes this cheap. Rather than
// materialising each feature's tag dictionary, the decoder resolves the
// styling key once per *layer* into a byte per value-table entry. Feature
// styling is then a single array index.

#ifndef MVT_H
#define MVT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MVT_UNKNOWN    = 0,
    MVT_POINT      = 1,
    MVT_LINESTRING = 2,
    MVT_POLYGON    = 3
} mvt_geom_t;

typedef enum {
    MVT_OK        =  0,
    MVT_EFORMAT   = -1,   // malformed protobuf or geometry stream
    MVT_ENOMEM    = -2,   // a scratch buffer was too small
    MVT_EABORT    = -3    // callback asked to stop
} mvt_err_t;

// ---- layer callback --------------------------------------------------------
typedef struct {
    const char *name;
    uint32_t    name_len;
    uint32_t    extent;        // tile-local coordinate span, usually 4096
    uint32_t    version;
    int         index;         // 0-based order within the tile
} mvt_layer_t;

// Return non-zero to decode this layer, 0 to skip it entirely.
// Skipping is cheap: the layer body is never scanned.
typedef int (*mvt_layer_fn)(void *ctx, const mvt_layer_t *layer);

// Map a styling-key value to a style id. Return 0 to drop features carrying
// that value (a cheap way to filter, say, every road kind you don't draw).
// `s` is NOT null-terminated.
typedef uint8_t (*mvt_style_fn)(void *ctx, const mvt_layer_t *layer,
                                const char *s, uint32_t len);

// ---- geometry callback -----------------------------------------------------
typedef struct {
    const mvt_layer_t *layer;
    mvt_geom_t  geom;
    uint8_t     style;         // from mvt_style_fn, 0 if key absent
    uint64_t    feature_id;
    uint32_t    feature_index;  // 0-based within the layer

    const int32_t *pts;        // x,y interleaved, tile-local coords
    uint32_t       n_pts;
    uint32_t       part_index; // 0-based within the feature

    // Polygons only. MVT v2 orders exterior rings clockwise in the y-down
    // tile space, which makes the shoelace sum positive; interior rings are
    // the reverse. A polygon feature is a sequence of rings where each
    // exterior ring starts a new sub-polygon and the holes follow it.
    int64_t area2;             // twice the signed area, 0 for non-polygons
    int     is_outer;
} mvt_part_t;

// Return non-zero to abort the whole decode.
typedef int (*mvt_part_fn)(void *ctx, const mvt_part_t *part);

// ---- decoder ---------------------------------------------------------------
typedef struct {
    // callbacks; layer_cb and style_cb may be NULL
    mvt_layer_fn layer_cb;
    mvt_style_fn style_cb;
    mvt_part_fn  part_cb;
    void        *ctx;

    // Which key drives styling. Defaults to "kind" (the Protomaps basemap
    // schema) when left NULL.
    const char *style_key;

    // Scratch: decoded points for the feature currently being emitted.
    // Sized for the largest single part in the tile. 8192 points (64 KiB)
    // clears everything in the Protomaps basemap comfortably.
    int32_t  *pt_buf;
    uint32_t  pt_cap;

    // Scratch: one style byte per value-table entry, per layer.
    uint8_t  *val_style;
    uint32_t  val_cap;

    // Populated during decode, useful for diagnostics.
    uint32_t stat_layers, stat_features, stat_parts, stat_points;
    uint32_t stat_max_part_pts, stat_max_values;
} mvt_decoder_t;

mvt_err_t mvt_decode(mvt_decoder_t *d, const uint8_t *tile, uint32_t len);

const char *mvt_strerror(mvt_err_t e);

#ifdef __cplusplus
}
#endif
#endif // MVT_H
