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
