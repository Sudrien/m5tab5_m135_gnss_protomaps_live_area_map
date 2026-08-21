# Where the numbers came from

Every literal in this project should be traceable. A number with no stated
origin cannot be reviewed, cannot be safely changed, and cannot be told apart
from a number that was tuned once on one device and never questioned again.

## The convention

Above each constant, or each group of constants that share an origin, a comment
line beginning `src:`. Greppable:

```
grep -rn "src:" *.c *.cpp *.h
```

Wrap continuation lines to line up under the first word after `src:`.

Six kinds of origin, and the wording should make clear which one it is:

**`src: <standard or specification>`** — the number is the format or the
protocol, and getting it wrong produces wrong output rather than a failure.
Name the document and the section, not just the organisation.

```c
// src: RFC 1951 (DEFLATE) section 3.2.7 - Huffman code lengths in the
//      dynamic block header are encoded in 3 bits with a maximum of 15.
#define MAX_BITS   15
```

**`src: <datasheet>`** — a register address, a bit position, a timing minimum.

**`src: <upstream source file>`** — matched to another codebase on purpose.
Give the file, and say *why* it is matched, because the reason is what tells a
future reader whether they may diverge from it.

```c
// src: M5GFX M5GFX.cpp, Tab5 autodetect - every expander write there is
//      i2c_write_register8_array(..., 100000). Matched rather than chosen: the
//      two drivers talk to the same device on the same bus.
static const uint32_t EXP_FREQ = 100000;
```

**`src: local logging`** — the device told us. Quote the line, so the claim can
be re-checked by running the thing again.

```c
// src: local logging. esp_hosted echoes the pins it ends up using:
//        sdio_wrapper: GPIOs: CLK[12] CMD[13] D0[11] D1[10] D2[9] D3[8]
//                             Slave_Reset[15]
```

**`src: measured`** — arrived at by trying it on hardware. Say what was
measured, on what, and what the failure looked like on either side of the
value. A tuned number without its experiment is folklore.

**`src: chosen`** — arbitrary, and admitting so is the point. Magic numbers in
file headers, sentinel values, buffer sizes picked to be comfortably large. Say
what constrains it, even if the answer is "nothing".

## What is not acceptable

A restated value. `// 72 hours` above `AOP_MAX_AGE_H = 72` says nothing that
the line below it does not. The comment exists to answer "why 72, and who says
so", and if the honest answer is "nobody, it seemed right", then `src: chosen`
is the correct and useful thing to write.

## Outstanding

67 constants still have no `src:` line. They are listed below by file. Most
cannot be attributed by anyone but their author - only the person who wrote
`PIN_PPS = 51` knows whether it came from a schematic, a pinout diagram, or a
morning of trying pins.

Suggested order of work: the hardware constants first, because they are the
ones that will be wrong for somebody else's board, and the ones a person
porting this will have to re-derive from nothing if they are unattributed.

### Hardware and protocol - highest value

    tab5_map.cpp      48  PIN_GNSS_TX = 7
    tab5_map.cpp      49  PIN_GNSS_RX = 6
    tab5_map.cpp      50  PIN_PPS     = 51
    tab5_map.cpp      51  GNSS_BAUD   = 38400

The comment on the first two mentions "the DIP in position 1" - that is the
beginning of an attribution and should be finished. Whether these are from a
module silkscreen, a wiring diagram or measurement decides whether anyone with
different hardware can adapt them.

### Tuned thresholds - need the experiment recorded

    tab5_map.cpp     148  ZOOM_HOLD_MS = 8000
    tab5_map.cpp     233  PREFETCH_RADIUS = 7
    tab5_map.cpp     250  BRIGHT_NIGHT = 60
    tab5_map.cpp     396  BRIGHT_DUSK = 140
    tab5_map.cpp     448  IDLE_DIM_MS = 120000
    tab5_map.cpp     449  IDLE_DIM_PCT = 40
    tab5_map.cpp     450  IDLE_DIM_FLOOR = 30
    tab5_map.cpp     965  ANTENNA_SUSPECT_MS 90000
    tab5_map.cpp    1167  AOP_SAVE_INTERVAL_MS (30 * 60 * 1000)
    netsource.cpp     52  REFRESH_DAYS = 30
    netsource.cpp     62  MAX_PROBE_DAYS = 8
    netsource.cpp    316  NET_REQUEST_GAP_MS 150
    netsource.cpp    322  NET_KEEPALIVE_IDLE_MS 10000
    mapengine.cpp    429  COARSE_FILLS_PER_PASS = 2
    raster.c          97  RS_INSERTION_MAX 48
    gnss.h            34  hdop < 2.5 in gnss_fine()

`gnss_fine()` is not in the count above but belongs here: that threshold gates
the staged zoom-in, so a receiver that never reaches it leaves the map at the
coarse zoom forever.

### Capacities and layout - say what constrains them

    mapengine.cpp     45  MVT_CAP_MAX = 3 MiB
    mapengine.cpp     46  PT_CAP = 2048
    mapengine.cpp     47  VAL_CAP = 1024
    mapengine.cpp     48  EDGE_CAP = 16384
    mapengine.cpp     49  XS_CAP = 4096
    mapengine.cpp     54  STATUS_H = 52
    mapengine.cpp     55  FOOTER_H = 82
    mapengine.cpp    113  COARSE_STEP_CFG 2
    mapengine.cpp    116  COARSE_SLOT = 0xFE
    mapengine.cpp    574  MAP_SCRATCH_INTERNAL_KB 4
    mapengine.cpp    580  MAP_INTERNAL_RESERVE (110 * 1024)
    netsource.cpp     81  LOCAL_ARCHIVE_MAX 16
    netsource.cpp    131  MEMO_CAP 4096
    netsource.cpp    172  DIR_CAP_MIN = 256 KiB
    netsource.cpp    173  DIR_CAP_MAX = 4 MiB
    netsource.cpp    179  DIR_EXPAND = 4
    netsource.cpp    186  LOCAL_ROOT_CAP = 64 KiB
    netsource.cpp    337  NET_HANDSHAKE_MIN_DMA (32 * 1024)
    netsource.cpp    491  RANGE_CHUNK = 32 KiB
    tab5_map.cpp     984  AOP_CAP = 16 KiB
    tab5_map.cpp    1338  STATUS_LINE_MAX = 128
    tab5_map.cpp    1339  STATUS_STATS_MAX = 192
    inflate.c         10  FAST_BITS 9

`FAST_BITS` already has the reasoning above it - 512 entries, 1 KiB, fits in
cache on the P4 - and only needs the `src:` tag to say that is a choice rather
than a requirement of the format. Several others here are the same: the
explanation exists, the attribution does not.

### Geometry and UI

    mapconfig.h       24  SUBTILE_PX 1280
    mapconfig.h       41  MARKER_BAND 0.33f
    mapconfig.h       47  MARKER_CLEAR_R 36
    mapconfig.h       61  GRID_N 2
    mapconfig.h       69  COARSE_PX 512
    mapconfig.h       93  Z_FLOOR 14
    mapconfig.h      118  SUBTILE_SPLIT 0
    mapconfig.h      136  CACHE_MAX_ENTRIES_CFG 80000
    mapconfig.h      143  WORLD_FLOOR_ZOOM 6
    tab5_map.cpp     350  BTN_H = 54, BTN_M = 12
    tab5_map.cpp     381  UI_STATUS_H = 52
    tab5_map.cpp     392  BTN_PAD_TOP = 26, BTN_PAD_SIDE = 6
    tab5_map.cpp     471  KEY = 0xF81F
    raster.h          27  RS_FRAC_BITS 8
    raster.h          38  RS_SUBSAMPLES 2
    raster.c         444  MIN_STEP = RS_ONE / 2

`UI_STATUS_H = 52` carries "must match STATUS_H in mapengine", which is a
constraint rather than an origin - both are 52 and neither says where 52 came
from. If one is derived from a font metric or a touch-target minimum, that is
the attribution, and the other should say "must match" and point at it.

`BRIGHT_DAY`, `BRIGHT_DUSK` and `BRIGHT_NIGHT` are the three numbers with the
largest effect on battery life in the project, and two of the three are
judgements. The experiment that would attribute them is now cheap to run: the
`power:` line reports mean current against the brightness in force, so a few
minutes held at each level gives the milliamps each one costs. Until that is
recorded, changing any of them is guesswork in both directions - readability
and consumption.

The three `IDLE_DIM_*` values are the least examined numbers in this set and
the only ones whose failure is felt rather than measured. Too short a delay
and the screen dims while someone is reading it at a stop; too deep a step or
too low a floor and it reads as crashed rather than resting, which matters
because the way back is a touch and nobody touches a screen they believe has
died. None of the three has been tried on hardware. They want a session of
sitting with the device parked more than they want an instrument.

`DUSK_HALFWIDTH_MIN` is attributed, to civil twilight at mid latitudes, but the
attribution is honest about being a fixed stand-in for something that varies
with latitude and season. That is a different state from unattributed and does
not belong in the lists above; it is noted here so it is not mistaken for
settled.

`SUBTILE_PX 1280` and `Z_FLOOR 14` are probably the two most load-bearing
numbers in the project. They decide the whole tiling scheme and appear in the
boot log every run.
