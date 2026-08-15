// style.h - shared cartographic style table and draw order.
#ifndef STYLE_H
#define STYLE_H
#include "raster.h"
#ifdef __cplusplus
extern "C" {
#endif
enum {
    S_NONE = 0, S_EARTH, S_WATER, S_PARK, S_GRASS, S_URBAN, S_PIER,
    S_BUILDING, S_MAJOR, S_MINOR, S_PATH, S_RAIL, S_FERRY,
    S_POI,
    // Places are split by rank because the label layer treats them very
    // differently: a country name is drawn once and huge, a neighbourhood is
    // drawn last and dropped first. The dot styles stay identical - only the
    // label side cares - but the style byte is the only channel the decoder
    // gives us, so the distinction has to live here.
    S_PLACE_COUNTRY, S_PLACE_REGION, S_PLACE_LOCALITY, S_PLACE_HOOD,
    S_COUNT
};
extern rs_style_t STYLES[S_COUNT];
extern const char *DRAW_ORDER[];
extern const int   N_DRAW_ORDER;
// `dark` selects the night palette. Changing it invalidates every rendered
// tile, since the colours are baked in at rasterise time.
void    style_init(int px, int dark);
int     style_is_dark();

// Background behind the map, matching the active palette.
uint16_t style_background();
uint8_t style_lookup(void *ctx, const mvt_layer_t *l, const char *s, uint32_t n);

// Is this style byte a labelled point? Keeps the label collector from having
// to know the enum's shape.
static inline int style_is_place(uint8_t s) {
    return s >= S_PLACE_COUNTRY && s <= S_PLACE_HOOD;
}
static inline int style_is_labelled(uint8_t s) {
    return s == S_POI || style_is_place(s);
}
#ifdef __cplusplus
}
#endif
#endif
