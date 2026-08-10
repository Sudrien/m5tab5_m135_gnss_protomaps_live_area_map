// tilecache.h - append-only tile cache.
//
// WHY NOT ONE FILE PER TILE
//
// The obvious layout - /t/<build>/<z>/<x>/<y> - is close to the worst access
// pattern an SD card can be given:
//
//   * Every create rewrites the FAT, which sits at a fixed logical address.
//     Lose power mid-update and the filesystem goes, not just the file. On a
//     portable device that is the realistic failure mode, well ahead of wear.
//
//   * 32 KB clusters against 3 KB tiles waste most of each allocation, and
//     the card's erase block is measured in megabytes - so each small
//     scattered write becomes a read-modify-write of a far larger region.
//
// Appending to a single blob instead makes every write sequential, keeps
// metadata updates to one file length change per flush, and lets the card's
// controller do what it is good at. The index lives in PSRAM and is written
// out periodically; if it is lost the blob can be rescanned, because each
// record carries its own header.
//
// Cards do wear-level internally - they are not raw NAND - so this is about
// power-loss integrity and write amplification rather than exhausting cells.

#ifndef TILECACHE_H
#define TILECACHE_H

#include <stdint.h>

struct CacheStats {
    uint32_t entries = 0;
    uint32_t hits = 0, misses = 0, writes = 0;
    uint64_t blob_bytes = 0;
    uint32_t index_flushes = 0;
    uint32_t rescans = 0;
};

// `build` names the archive vintage; changing it starts a fresh blob and
// discards the old one, since offsets from one build mean nothing in another.
bool tilecache_open(const char *build, uint32_t max_entries);
void tilecache_close();

// Returns true and fills dst/len on a hit. A zero-length hit is a negative
// marker: the archive genuinely has no tile there, so do not go back to the
// network for it.
bool tilecache_get(uint8_t z, uint32_t x, uint32_t y, uint8_t *dst, uint32_t *len);

// len == 0 records a negative marker.
bool tilecache_put(uint8_t z, uint32_t x, uint32_t y, const uint8_t *src, uint32_t len);

// Persist the index. Called automatically every FLUSH_INTERVAL writes; call
// directly before a planned shutdown.
void tilecache_flush();

// Delete the blob and index and start over. The only remedy for a blob
// damaged by concurrent writes: records after the damage are unreachable and
// their space is never reclaimed, so a partly-corrupt cache only grows.
bool tilecache_wipe();

void tilecache_stats(CacheStats *out);

// Writes appended since the last index flush. Zero means a power cut right
// now would cost nothing at all, not even rescan time.
uint32_t tilecache_pending();

#endif // TILECACHE_H
