// mercator.h - Web Mercator (EPSG:3857) tile math.
//
// Header-only, no allocation, no dependencies beyond libm. Safe to call from
// any task. All functions are pure.
//
// Conventions:
//   - XYZ scheme (y increases southward), which is what PMTiles/MVT use.
//   - "fractional tile coords" are the continuous form: integer part is the
//     tile index, fraction is the position inside that tile (0..1).
//   - Latitude is clamped to +/-85.0511287798 (the Mercator limit); beyond
//     that the projection diverges.

#ifndef MERCATOR_H
#define MERCATOR_H

#include <math.h>
#include <stdint.h>

// M_PI is POSIX, not ISO C - absent under strict -std=c11 and on some
// embedded toolchains. Define it ourselves rather than relying on the build.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// src: the Web Mercator convention - atan(sinh(pi)) in degrees, the latitude
//      at which the projection becomes square. Used by every slippy-map
//      implementation and by the EPSG:3857 bounds; not a chosen cutoff.
#define MERC_LAT_LIMIT 85.05112877980659

typedef struct {
    double x, y;   // fractional tile coordinates at some zoom
    uint8_t z;
} merc_pt_t;

static inline double merc_clamp_lat(double lat) {
    if (lat >  MERC_LAT_LIMIT) return  MERC_LAT_LIMIT;
    if (lat < -MERC_LAT_LIMIT) return -MERC_LAT_LIMIT;
    return lat;
}

// lat/lon (degrees) -> fractional tile coords at zoom z.
static inline merc_pt_t merc_from_ll(double lat, double lon, uint8_t z) {
    merc_pt_t p;
    double n = (double)(1u << z);
    lat = merc_clamp_lat(lat);
    // wrap longitude into [-180,180) so x stays in range near the antimeridian
    lon = fmod(lon + 180.0, 360.0);
    if (lon < 0) lon += 360.0;
    lon -= 180.0;

    double lat_rad = lat * M_PI / 180.0;
    p.x = (lon + 180.0) / 360.0 * n;
    p.y = (1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * n;
    p.z = z;
    return p;
}

// Inverse: fractional tile coords -> lat/lon (degrees).
static inline void merc_to_ll(merc_pt_t p, double *lat, double *lon) {
    double n = (double)(1u << p.z);
    *lon = p.x / n * 360.0 - 180.0;
    double t = M_PI * (1.0 - 2.0 * p.y / n);
    *lat = atan(sinh(t)) * 180.0 / M_PI;
}

// Ground resolution in metres per tile-pixel, given a tile rendered at
// `tile_px` pixels square. Useful for scale bars and for deciding whether a
// zoom level has enough detail for the display density.
static inline double merc_ground_res(double lat, uint8_t z, int tile_px) {
    // src: 2 * pi * 6378137, the WGS84 semi-major axis. EPSG:3857 treats the
    //      earth as a sphere of that radius, so this is the full width of the
    //      projected world in metres.
    const double EQUATOR_M = 40075016.685578488;
    lat = merc_clamp_lat(lat);
    return EQUATOR_M * cos(lat * M_PI / 180.0) / ((double)tile_px * (double)(1u << z));
}

// Clamp/wrap a tile index into valid range for zoom z.
// X wraps (the world is cyclic east-west); Y clamps (poles are hard edges).
static inline int32_t merc_wrap_x(int64_t x, uint8_t z) {
    int64_t n = (int64_t)1 << z;
    x %= n;
    if (x < 0) x += n;
    return (int32_t)x;
}
static inline int merc_y_valid(int64_t y, uint8_t z) {
    return y >= 0 && y < ((int64_t)1 << z);
}

// Pixel offset of a fractional point inside its own tile, for a tile
// rasterised at `tile_px` square.
static inline void merc_px_in_tile(merc_pt_t p, int tile_px, int *px, int *py) {
    *px = (int)((p.x - floor(p.x)) * tile_px);
    *py = (int)((p.y - floor(p.y)) * tile_px);
}

#endif // MERCATOR_H
