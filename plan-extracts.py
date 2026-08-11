#!/usr/bin/env python3
"""Plan .pmtiles extracts that each fit under the FAT32 4 GiB file limit.

FAT32 caps one file at 4 GiB, and a global z14 extract is about 32 GB, so world
coverage means splitting. Splitting by longitude works because the device
dispatches on the bounding box in each archive's own header - it only compares
longitude, so bands are exactly the shape it can use.

Equal-width bands would be badly unequal in size: an ocean band is nearly empty
and a European one is dense. So each band's east edge is found by binary search
on `pmtiles extract --dry-run`, which reports the size without downloading.

Usage:
  ./plan-extracts.py --zoom 14 --target 3.2
  ./plan-extracts.py --zoom 12 --target 3.2 --run

--dry-run passes cost real time (tens of seconds at z14), so a full z14 plan
takes a while. It is a one-time cost and much cheaper than downloading a band
that turns out to be 5 GB.
"""

import argparse, re, subprocess, sys

SRC_DEFAULT = "https://build.protomaps.com/20260807.pmtiles"

def size_gb(pmtiles, src, z, w, e, s, n):
    """Ask pmtiles what this band would weigh, without fetching it."""
    cmd = [pmtiles, "extract", src, "/dev/null",
           f"--minzoom={z}", f"--maxzoom={z}",
           f"--bbox={w},{s},{e},{n}", "--dry-run"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    blob = out.stdout + out.stderr
    m = re.search(r"archive size of ([\d.]+)\s*(B|KB|MB|GB|TB)", blob)
    if not m:
        print(blob.strip()[-500:], file=sys.stderr)
        raise SystemExit("could not parse a size from pmtiles output")
    val, unit = float(m.group(1)), m.group(2)
    return val * {"B":1e-9, "KB":1e-6, "MB":1e-3, "GB":1.0, "TB":1e3}[unit]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pmtiles", default="~/pmtiles")
    ap.add_argument("--src", default=SRC_DEFAULT)
    ap.add_argument("--zoom", type=int, required=True)
    ap.add_argument("--target", type=float, default=3.2,
                    help="GB per file. Keep under 4.29 (the FAT32 limit); the "
                         "default leaves margin because the search stops at a "
                         "band that is close enough, not exact.")
    ap.add_argument("--south", type=float, default=-85.0)
    ap.add_argument("--north", type=float, default=85.0)
    ap.add_argument("--prefix", default=None, help="default z<zoom>_")
    ap.add_argument("--run", action="store_true", help="run the extracts too")
    a = ap.parse_args()

    if a.target >= 4.29:
        raise SystemExit("--target must stay under the 4.29 GB FAT32 limit")
    prefix = a.prefix or f"z{a.zoom}_"

    bands, west, idx = [], -180.0, 0
    while west < 180.0:
        lo, hi = west, 180.0

        # Does everything remaining fit in one file? Then stop searching.
        whole = size_gb(a.pmtiles, a.src, a.zoom, west, 180.0, a.south, a.north)
        if whole <= a.target:
            bands.append((west, 180.0, whole)); break

        # Binary search the east edge. 8 iterations gets within ~1.4 degrees,
        # which is finer than the size variation between neighbouring bands.
        east = None
        for _ in range(8):
            mid = (lo + hi) / 2
            g = size_gb(a.pmtiles, a.src, a.zoom, west, mid, a.south, a.north)
            print(f"  probe {west:8.2f}..{mid:8.2f}  {g:6.2f} GB", file=sys.stderr)
            if g > a.target:
                hi = mid
            else:
                lo = mid; east = (mid, g)
        if east is None:
            # A single degree already exceeds the target - dense city band.
            # Emit it anyway and warn; the alternative is looping forever.
            east = (min(west + 1.0, 180.0),
                    size_gb(a.pmtiles, a.src, a.zoom, west, min(west+1.0,180.0),
                            a.south, a.north))
            print(f"  WARNING: {west:.2f}..{east[0]:.2f} is {east[1]:.2f} GB, "
                  f"over target and cannot be split further at this granularity",
                  file=sys.stderr)
        bands.append((west, east[0], east[1]))
        west = east[0]
        idx += 1
        if idx > 40:
            raise SystemExit("too many bands - is --target too small?")

    print(f"\n# z{a.zoom}: {len(bands)} files, "
          f"{sum(b[2] for b in bands):.1f} GB total\n")
    cmds = []
    for i, (w, e, g) in enumerate(bands):
        name = f"{prefix}{i:02d}.pmtiles"
        cmd = [a.pmtiles, "extract", a.src, name,
               f"--minzoom={a.zoom}", f"--maxzoom={a.zoom}",
               f"--bbox={w:.4f},{a.south},{e:.4f},{a.north}"]
        print(f"# {g:5.2f} GB  lon {w:8.2f} .. {e:8.2f}")
        print(" ".join(cmd))
        cmds.append(cmd)

    if a.run:
        for c in cmds:
            print(f"\n>>> {' '.join(c)}", file=sys.stderr)
            if subprocess.run(c).returncode != 0:
                raise SystemExit("extract failed")

if __name__ == "__main__":
    main()
