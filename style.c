// style.c - Protomaps basemap schema -> render styles.
#include "style.h"
#include <string.h>

rs_style_t STYLES[S_COUNT];

// Bottom-up cartographic order. MVT stores layers alphabetically, which is
// not draw order, so each is decoded in its own filtered pass.
const char *DRAW_ORDER[] = {
    "earth", "landuse", "water", "roads", "buildings", "pois", "places"
};
const int N_DRAW_ORDER = (int)(sizeof DRAW_ORDER / sizeof DRAW_ORDER[0]);

static int g_dark = 0;

int style_is_dark() { return g_dark; }

uint16_t style_background() {
    return g_dark ? rs_rgb(18, 20, 24) : rs_rgb(228, 226, 220);
}

void style_init(int px, int dark) {
    // Stroke widths scale with output resolution, or the map looks
    // progressively thinner as the tile gets sharper.
    int k = px / 256; if (k < 1) k = 1;
    g_dark = dark;
    memset(STYLES, 0, sizeof STYLES);

    if (!dark) {
        STYLES[S_EARTH]    = (rs_style_t){ rs_rgb(243,240,232), 0, 0, 1, 0 };
        STYLES[S_WATER]    = (rs_style_t){ rs_rgb(160,200,225), 0, 0, 1, 0 };
        STYLES[S_PARK]     = (rs_style_t){ rs_rgb(200,224,190), 0, 0, 1, 0 };
        STYLES[S_GRASS]    = (rs_style_t){ rs_rgb(210,232,200), 0, 0, 1, 0 };
        STYLES[S_URBAN]    = (rs_style_t){ rs_rgb(236,232,224), 0, 0, 1, 0 };
        STYLES[S_PIER]     = (rs_style_t){ rs_rgb(228,224,214), 0, 0, 1, 0 };
        STYLES[S_BUILDING] = (rs_style_t){ rs_rgb(214,208,198), 0, 0, 1, 0 };
        STYLES[S_MAJOR] = (rs_style_t){ 0, rs_rgb(255,255,255), (uint8_t)(5*k), 0, 1 };
        STYLES[S_MINOR] = (rs_style_t){ 0, rs_rgb(255,255,255), (uint8_t)(3*k), 0, 1 };
        STYLES[S_PATH]  = (rs_style_t){ 0, rs_rgb(224,216,204), (uint8_t)(1*k), 0, 1 };
        STYLES[S_RAIL]  = (rs_style_t){ 0, rs_rgb(190,186,180), (uint8_t)(2*k), 0, 1 };
        STYLES[S_FERRY] = (rs_style_t){ 0, rs_rgb(150,180,210), (uint8_t)(1*k), 0, 1 };
        STYLES[S_POI]   = (rs_style_t){ rs_rgb(190,120,110), 0, (uint8_t)(2*k), 1, 0 };
        STYLES[S_PLACE] = (rs_style_t){ rs_rgb(90,90,100),   0, (uint8_t)(3*k), 1, 0 };
    } else {
        // Night palette.
        //
        // Not a simple inversion: roads must stay the brightest thing on the
        // map because they carry the information, while land and water drop
        // far enough that the panel is not a lamp. Water stays bluer than
        // land so the two remain distinguishable at low brightness, where
        // most of the contrast range is gone.
        STYLES[S_EARTH]    = (rs_style_t){ rs_rgb( 26, 28, 33), 0, 0, 1, 0 };
        STYLES[S_WATER]    = (rs_style_t){ rs_rgb( 18, 34, 54), 0, 0, 1, 0 };
        STYLES[S_PARK]     = (rs_style_t){ rs_rgb( 22, 38, 26), 0, 0, 1, 0 };
        STYLES[S_GRASS]    = (rs_style_t){ rs_rgb( 26, 42, 30), 0, 0, 1, 0 };
        STYLES[S_URBAN]    = (rs_style_t){ rs_rgb( 32, 35, 41), 0, 0, 1, 0 };
        STYLES[S_PIER]     = (rs_style_t){ rs_rgb( 38, 40, 45), 0, 0, 1, 0 };
        STYLES[S_BUILDING] = (rs_style_t){ rs_rgb( 46, 50, 58), 0, 0, 1, 0 };
        STYLES[S_MAJOR] = (rs_style_t){ 0, rs_rgb(196,204,214), (uint8_t)(5*k), 0, 1 };
        STYLES[S_MINOR] = (rs_style_t){ 0, rs_rgb(132,140,152), (uint8_t)(3*k), 0, 1 };
        STYLES[S_PATH]  = (rs_style_t){ 0, rs_rgb( 92, 96,104), (uint8_t)(1*k), 0, 1 };
        STYLES[S_RAIL]  = (rs_style_t){ 0, rs_rgb( 86, 90, 98), (uint8_t)(2*k), 0, 1 };
        STYLES[S_FERRY] = (rs_style_t){ 0, rs_rgb( 60, 84,112), (uint8_t)(1*k), 0, 1 };
        STYLES[S_POI]   = (rs_style_t){ rs_rgb(150, 82, 76),  0, (uint8_t)(2*k), 1, 0 };
        STYLES[S_PLACE] = (rs_style_t){ rs_rgb(140,144,156),  0, (uint8_t)(3*k), 1, 0 };
    }
}

static int keq(const char *s, uint32_t n, const char *lit) {
    return n == strlen(lit) && memcmp(s, lit, n) == 0;
}

uint8_t style_lookup(void *ctx, const mvt_layer_t *l, const char *s, uint32_t n) {
    (void)ctx;
    if (keq(l->name, l->name_len, "earth"))     return S_EARTH;
    if (keq(l->name, l->name_len, "water"))     return S_WATER;
    if (keq(l->name, l->name_len, "buildings")) return S_BUILDING;
    if (keq(l->name, l->name_len, "landuse")) {
        if (keq(s,n,"park") || keq(s,n,"forest") || keq(s,n,"wood"))   return S_PARK;
        if (keq(s,n,"grass") || keq(s,n,"farmland") || keq(s,n,"scrub")) return S_GRASS;
        if (keq(s,n,"pier")) return S_PIER;
        return S_URBAN;
    }
    if (keq(l->name, l->name_len, "roads")) {
        if (keq(s,n,"highway") || keq(s,n,"major_road")) return S_MAJOR;
        if (keq(s,n,"minor_road")) return S_MINOR;
        if (keq(s,n,"path"))  return S_PATH;
        if (keq(s,n,"rail"))  return S_RAIL;
        if (keq(s,n,"ferry")) return S_FERRY;
        return S_MINOR;
    }
    if (keq(l->name, l->name_len, "pois"))   return S_POI;
    if (keq(l->name, l->name_len, "places")) return S_PLACE;
    return S_NONE;
}
