#!/usr/bin/env python3
"""Regenerate worldmap_data.h - the compiled-in z0 world outline.

Source is Natural Earth 1:110m, which is public domain, so the coordinates can
be embedded in the binary with nothing more than an acknowledgement. That is
the reason for Natural Earth rather than GSHHG (LGPL) or a z0 tile pulled out
of a pmtiles archive (whatever the archive's own basemap is licensed under -
OpenStreetMap data is ODbL, which is a licence to think about before baking
its geometry into firmware).

Usage:

    curl -sLO https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_land.geojson
    curl -sLO https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_110m_lakes.geojson
    python3 tools/gen_worldmap.py ne_110m_land.geojson ne_110m_lakes.geojson > worldmap_data.h

The output is deterministic, so regenerating without changing the inputs or
the constants below produces a byte-identical file.
"""

import json
import math
import sys

# What the outline is drawn at. Simplification and the drop threshold are both
# expressed in pixels of this, so the geometry is cut to the panel rather than
# to a round number of degrees.
TARGET_PX = 1280

# Douglas-Peucker tolerance, in pixels at TARGET_PX. Below one pixel, so the
# simplification cannot move a coastline somewhere the panel could show.
SIMPLIFY_PX = 0.7

# Rings smaller than this (square pixels, by the shoelace area) go. At 2 px the
# survivors are everything that would be more than a speck; the Azores stay,
# individual atolls do not.
MIN_AREA_PX = 2.0
MIN_AREA_PX_LAKES = 2.5


def mercator(lon, lat):
    """Web Mercator, normalised to the z0 tile: 0..1 across and down."""
    lat = max(-85.05112878, min(85.05112878, lat))
    s = math.sin(math.radians(lat))
    return (lon + 180.0) / 360.0, 0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)


def simplify(pts, eps):
    """Iterative Douglas-Peucker. Iterative rather than recursive because a
    few of these rings are thousands of points long."""
    if len(pts) < 3:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        ax, ay = pts[i]
        bx, by = pts[j]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        best, best_i = -1.0, -1
        for k in range(i + 1, j):
            px, py = pts[k]
            d = (math.hypot(px - ax, py - ay) if norm == 0 else
                 abs(dy * px - dx * py + bx * ay - by * ax) / norm)
            if d > best:
                best, best_i = d, k
        if best > eps:
            keep[best_i] = True
            stack.append((i, best_i))
            stack.append((best_i, j))
    return [p for p, k in zip(pts, keep) if k]


def rings_from(path, min_area_px):
    """Every ring of every polygon, projected, simplified and quantised to the
    16-bit grid. Holes come through as ordinary rings: the renderer fills
    even-odd, so a lake inside a landmass needs no marking."""
    doc = json.load(open(path))
    eps = SIMPLIFY_PX / TARGET_PX
    min_area = (min_area_px / TARGET_PX) ** 2
    out = []
    for feat in doc["features"]:
        geom = feat.get("geometry")
        if not geom:
            continue
        polys = (geom["coordinates"] if geom["type"] == "MultiPolygon"
                 else [geom["coordinates"]])
        for poly in polys:
            for ring in poly:
                pts = simplify([mercator(x, y) for x, y in ring], eps)
                if len(pts) < 4:
                    continue
                area = abs(sum(pts[i][0] * pts[i + 1][1] - pts[i + 1][0] * pts[i][1]
                               for i in range(len(pts) - 1))) / 2
                if area < min_area:
                    continue
                q = []
                for x, y in pts:
                    xi = int(round(max(0.0, min(1.0, x)) * 65535))
                    yi = int(round(max(0.0, min(1.0, y)) * 65535))
                    # Quantisation collapses neighbours; a repeated vertex is a
                    # zero-length edge, which the scanline fill would rather
                    # not see.
                    if not q or q[-1] != (xi, yi):
                        q.append((xi, yi))
                if len(q) >= 4:
                    out.append(q)
    out.sort(key=len, reverse=True)
    return out


def emit(name, rings, w):
    flat = [v for ring in rings for pt in ring for v in pt]
    offs, n = [0], 0
    for ring in rings:
        n += len(ring)
        offs.append(n)
    for label, vals in ((name + "_XY", flat), (name + "_RING", offs)):
        w(f"static const uint16_t {label}[] = {{\n")
        for i in range(0, len(vals), 12):
            w("    " + ", ".join(str(v) for v in vals[i:i + 12]) + ",\n")
        w("};\n\n")


def main(argv):
    if len(argv) != 3:
        sys.exit(__doc__)
    land = rings_from(argv[1], MIN_AREA_PX)
    lakes = rings_from(argv[2], MIN_AREA_PX_LAKES)
    w = sys.stdout.write
    w("// Generated file - do not edit by hand. See tools/gen_worldmap.py.\n"
      "//\n"
      '// Natural Earth 1:110m "land" and "lakes", public domain\n'
      "// (naturalearthdata.com), projected to Web Mercator and quantised to 16\n"
      "// bits over the span of the z0 tile: 0 is the left/top edge of the world,\n"
      "// 65535 the right/bottom. Rings are simplified to about 0.7 px at 1280,\n"
      "// which is the size this is drawn at, so the loss is below what the panel\n"
      "// can show.\n"
      "//\n"
      "// Coordinates only, no styling and no names: this is the boot backdrop,\n"
      "// and its whole job is to be a recognisable world without touching the SD\n"
      "// card.\n"
      "#ifndef WORLDMAP_DATA_H\n#define WORLDMAP_DATA_H\n\n#include <stdint.h>\n\n")
    emit("WM_LAND", land, w)
    emit("WM_LAKE", lakes, w)
    w(f"#define WM_LAND_RINGS {len(land)}\n#define WM_LAKE_RINGS {len(lakes)}\n\n")
    w("#endif // WORLDMAP_DATA_H\n")


if __name__ == "__main__":
    main(sys.argv)
