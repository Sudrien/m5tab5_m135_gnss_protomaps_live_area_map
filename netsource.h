// netsource.h - tile acquisition: SD cache, network, and local fallback.
//
// Sources are tried in order, cheapest first:
//
//   1. SD cache      /t/<build>/<z>/<x>/<y>   - a tile fetched before
//   2. Network       HTTP range read straight out of the remote .pmtiles
//   3. world.pmtiles bundled low-zoom floor, always present, always offline
//
// The network source is not a separate protocol. PMTiles is designed for
// range access, so the remote archive is opened with the same pmt_t reader
// used for the local one - only the read callback differs, HTTP instead of
// SD. Everything above it is unchanged.
//
// FRESHNESS
// Protomaps keeps roughly one week of daily builds, so a hardcoded date goes
// stale almost immediately. The device discovers a live build by probing
// backwards from today's UTC date (from GNSS, no NTP required) and records
// what it adopted. Once that record is older than the refresh interval it
// probes again; a new build means a new cache directory, and the old one is
// deleted wholesale rather than being invalidated tile by tile.

#ifndef NETSOURCE_H
#define NETSOURCE_H

#include <stdint.h>

struct NetStats {
    uint32_t cache_hits = 0, net_hits = 0, local_hits = 0;
    uint32_t misses = 0, errors = 0;
    uint32_t bytes_fetched = 0;
    uint32_t last_fetch_ms = 0;
    char     build[16] = "";
    bool     online = false;
    // How many local .pmtiles are open. Not a diagnostic in itself - it tells
    // a caller which of the counters above can ever be non-zero. With a planet
    // archive mounted, tiles are served locally and cache_hits, net_hits and
    // the blob's entry count all stay at zero permanently, which reads as a
    // fault rather than as the intended offline path doing its job.
    uint8_t  locals = 0;
};

// `local_path` is the always-present low-zoom archive (world.pmtiles).
// Downloaded automatically on first run if absent and the network is up.
bool netsource_begin(const char *local_path);

// Give the module today's UTC date as DDMMYY (the NMEA RMC field) so it can
// judge whether the adopted build has aged out. Safe to call repeatedly.
void netsource_set_date(const char *ddmmyy);

// Same, from the system clock. Returns false if SNTP has not set it yet.
// Worth having both: GNSS needs a sky view, SNTP needs a network, and a
// device indoors on wifi has one but not the other.
bool netsource_set_date_from_clock();

// Fetch one tile's compressed payload. Returns true on success; sets
// *from_net when it came off the wire, so the caller can throttle.
bool netsource_get(uint8_t z, uint32_t x, uint32_t y,
                   uint8_t *dst, uint32_t *len, bool *from_net);

// Cache and local archives only - never the network, and never a miss that
// costs anything.
//
// For callers that want a tile if it is already to hand and would rather have
// nothing than wait: the place-name lookups, which read a 3x3 block at once.
// Going to the wire for those is a bad trade in both directions. A failed
// range request blocks the render worker for seconds and, on this hardware,
// the TLS handshake competes for the same DMA-capable heap the wifi driver
// needs - nine of them in a row is what drove the free block down to 23 KB
// and made the driver start dropping received frames. And the reward for all
// that is a place name, which is a nicety on a map that is already drawing.
bool netsource_get_local(uint8_t z, uint32_t x, uint32_t y,
                         uint8_t *dst, uint32_t *len);

// Is this tile inside a local archive's zoom range and bounding box?
//
// Header arithmetic only - it does not open a directory or read a tile, so it
// is cheap enough to ask about thousands of tiles in a row. Used to decide
// that a tile needs no caching at all: if a local archive holds it, a copy in
// the SD cache is a second copy of a file already sitting on the same card.
bool netsource_local_covers(uint8_t z, uint32_t x, uint32_t y);

// Reads and bytes issued against local archives since boot, counted at the
// single read callback every PMTiles lookup goes through - directory levels
// and tile bodies alike. Monotonic; sample either side of a piece of work and
// subtract. Reads per tile is the figure that shows whether the directory
// cache is being reused or thrashed.
void netsource_io_counters(uint32_t *reads, uint64_t *bytes);

// Directory decompression since boot: microseconds spent inflating, and the
// number of inflates. Both are directory-only - tile payloads are inflated by
// the caller, not in here - and every counted inflate is a directory cache
// miss, since a hit returns before the decompress. Monotonic, same sample-and
// -subtract use as netsource_io_counters().
//
// This is what separates a slow medium from a thrashing directory cache. Raw
// file time already shows up as seek and xfer; whatever is left inside a
// lookup is mostly this.
void netsource_dir_counters(uint32_t *inflate_us, uint32_t *loads);

// Populate the offline floor by pulling every tile from z0 to maxz into the
// cache. 5461 tiles at maxz 6; slow (one request each) but resumable, since
// anything already cached is skipped. `buf`/`cap` is scratch for one tile.
bool netsource_prefetch_world(uint8_t maxz, uint8_t *buf, uint32_t cap,
                              void (*progress)(uint32_t done, uint32_t total));

// True once the world floor has been stored. Recorded with a marker file
// rather than by counting tiles, since many low-zoom tiles are legitimately
// empty and would be indistinguishable from missing ones.
bool netsource_world_ready();
void netsource_world_mark_done();

// Checkpoint for the world floor walk.
//
// Without one, an interrupted run restarts at z0 and re-walks every position
// doing a cache lookup per tile - correct, but it re-reads thousands of
// index entries to discover it has nothing to do. The checkpoint records how
// far the walk got so a resume starts there.
//
// Tied to the build: a new build invalidates the cache, so a position within
// the old one means nothing.
void netsource_world_save_pos(uint8_t z, uint32_t x, uint32_t y);
bool netsource_world_load_pos(uint8_t *z, uint32_t *x, uint32_t *y);
void netsource_world_clear_pos();

// Re-probe for a current build and drop the old cache if one is found.
// Called automatically when the adopted build ages out.
bool netsource_refresh();

void netsource_stats(NetStats *out);

#endif // NETSOURCE_H
