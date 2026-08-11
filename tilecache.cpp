// tilecache.cpp

#include "tilecache.h"
#include "mapconfig.h"
#include <Arduino.h>
#include "storage.h"
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern "C" {
  #include "pmtiles.h"     // for pmt_zxy_to_tileid
}

// Where the tile cache lives on the card. Defined once: netsource.cpp used to
// carry its own unused copy, which is how two constants for one directory
// eventually drift apart. IDF's warning flags surfaced the duplicate that
// Arduino's did not.
static const char *CACHE_DIR = "/t";

// Flush the index after this many writes. Each flush is one small file
// rewrite, so the interval trades restart cost against metadata churn:
// 256 writes is a few seconds of prefetching and one FAT update.
// Early on, a handful of tiles is the entire cache and losing them to an
// unflushed index is the difference between a usable map and a blank one.
// Once enough are stored the marginal tile matters less, so the interval
// widens and metadata churn drops back down.
static uint32_t flush_interval() {
    extern uint32_t tilecache_entry_count();
    uint32_t n = tilecache_entry_count();
    if (n < 64)  return 1;
    if (n < 512) return 16;
    return 256;
}

// Record header written ahead of every blob entry, so a lost or stale index
// can be rebuilt by walking the file.
struct __attribute__((packed)) RecHdr {
    uint32_t magic;      // 'PMTC'
    uint64_t tile_id;
    uint32_t len;
};
// src: chosen. ASCII "CTMP" big-endian, so a hex dump of the cache file is
//      readable. No external meaning.
static const uint32_t REC_MAGIC = 0x43544D50;

struct Entry {
    uint64_t tile_id;
    uint32_t offset;     // of the payload, past its header
    uint32_t len;
};

static fs::FS *g_fs = nullptr;
static File     g_blob;
static Entry   *g_idx = nullptr;
static uint32_t g_n = 0, g_cap = 0;
static uint32_t g_since_flush = 0;
static char     g_build[16] = "";
static char     g_blobPath[64], g_idxPath[64];
static CacheStats g_st;

// The blob is one File handle with one position, and the index is one array.
// Three tasks reach this through netsource, so both need serialising - a
// concurrent read and append on the same handle silently returns the wrong
// bytes rather than failing.
static SemaphoreHandle_t g_clock = nullptr;

// Append offset, tracked here rather than asked of the file.
//
// File::size() on an "r+" handle reports what the filesystem has committed,
// which is not necessarily what has been written - buffered bytes may not be
// counted until a flush. Two appends between flushes then compute the same
// offset and land on top of each other, which shows up later as a record
// with bad magic partway through an otherwise healthy blob.
static uint32_t g_blob_len = 0;

struct Guard {
    bool held;
    Guard() : held(false) {
        if (g_clock) held = (xSemaphoreTake(g_clock, portMAX_DELAY) == pdTRUE);
    }
    ~Guard() { if (held) xSemaphoreGive(g_clock); }
};

// ---- index -----------------------------------------------------------------
// Kept sorted by tile_id so lookups are a binary search. Tiles arrive in
// roughly spatial order, which after the Hilbert mapping is roughly sorted,
// so the insertion memmove is usually short.
static int find_slot(uint64_t id, bool *found) {
    int lo = 0, hi = (int)g_n - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (g_idx[mid].tile_id == id) { *found = true; return mid; }
        if (g_idx[mid].tile_id < id) lo = mid + 1;
        else hi = mid - 1;
    }
    *found = false;
    return lo;
}

static bool idx_insert(uint64_t id, uint32_t off, uint32_t len) {
    bool found;
    int at = find_slot(id, &found);
    if (found) { g_idx[at].offset = off; g_idx[at].len = len; return true; }
    if (g_n >= g_cap) return false;
    memmove(&g_idx[at + 1], &g_idx[at], (g_n - at) * sizeof(Entry));
    g_idx[at].tile_id = id;
    g_idx[at].offset = off;
    g_idx[at].len = len;
    g_n++;
    return true;
}

static void idx_save() {
    File f = g_fs->open(g_idxPath, FILE_WRITE);
    if (!f) return;
    uint32_t n = g_n;
    uint64_t blobLen = g_blob_len;
    f.write((uint8_t *)&REC_MAGIC, 4);
    f.write((uint8_t *)&n, 4);
    f.write((uint8_t *)&blobLen, 8);
    storage_write(f, (uint8_t *)g_idx, n * sizeof(Entry));
    f.close();
    g_st.index_flushes++;
    g_since_flush = 0;
}

// Walk the blob rebuilding the index. Needed when the index is missing or was
// written before records that followed it - which is exactly what happens if
// power is lost between flushes. The record headers make this unambiguous.
static void rescan() {
    g_n = 0;
    if (!g_blob) return;
    uint32_t sz = g_blob_len, pos = 0;
    g_blob.seek(0);
    uint32_t stopped_at = 0;
    const char *why = "end of blob";
    while (pos + sizeof(RecHdr) <= sz && g_n < g_cap) {
        RecHdr h;
        if (g_blob.read((uint8_t *)&h, sizeof h) != (int)sizeof h) {
            stopped_at = pos; why = "read failed"; break;
        }
        if (h.magic != REC_MAGIC) {
            stopped_at = pos; why = "bad record magic"; break;
        }
        uint32_t payload = pos + sizeof(RecHdr);
        if (payload + h.len > sz) {
            stopped_at = pos; why = "record runs past end"; break;
        }
        idx_insert(h.tile_id, payload, h.len);
        pos = payload + h.len;
        g_blob.seek(pos);
    }
    // Stopping well short of the end means the blob is damaged, not merely
    // unflushed. Saying so matters: everything past that point is
    // unreachable, and the space is never reclaimed.
    if (stopped_at && sz - stopped_at > 64 * 1024) {
        Serial.printf("tilecache: CORRUPT - stopped at %lu of %lu bytes (%s)\n",
                      (unsigned long)stopped_at, (unsigned long)sz, why);
        Serial.printf("tilecache: %lu KB after that point is unreachable; "
                      "delete /t/%s.dat and .idx to reclaim it\n",
                      (unsigned long)((sz - stopped_at) / 1024), g_build);
    }
    g_st.rescans++;
    Serial.printf("tilecache: rescanned %lu records from %lu bytes\n",
                  (unsigned long)g_n, (unsigned long)sz);

    // Finding nothing in a blob that plainly has content means the read path
    // is broken, not that the data is gone. Saving an empty index at that
    // point would turn a recoverable situation into a permanent one.
    if (g_n == 0 && sz > sizeof(RecHdr)) {
        Serial.println("tilecache: blob is non-empty but unreadable - index left alone");
        return;
    }
    idx_save();
}

static bool idx_load() {
    File f = g_fs->open(g_idxPath, FILE_READ);
    if (!f) return false;
    uint32_t magic = 0, n = 0;
    uint64_t blobLen = 0;
    if (f.read((uint8_t *)&magic, 4) != 4 || magic != REC_MAGIC) { f.close(); return false; }
    f.read((uint8_t *)&n, 4);
    f.read((uint8_t *)&blobLen, 8);
    if (n > g_cap) { f.close(); return false; }
    if (storage_read(f, (uint8_t *)g_idx, n * sizeof(Entry)) != n * sizeof(Entry)) {
        f.close(); return false;
    }
    f.close();
    g_n = n;

    // If the blob grew past what the index knows about, records were appended
    // after the last flush. Rescanning recovers them instead of orphaning
    // however many tiles were fetched since.
    if (g_blob && g_blob_len != blobLen) {
        Serial.printf("tilecache: blob %lu bytes, index knows %lu - rescanning\n",
                      (unsigned long)g_blob_len, (unsigned long)blobLen);
        return false;
    }
    return true;
}

// ---- public ----------------------------------------------------------------
bool tilecache_open(const char *build, uint32_t max_entries) {
    // netsource opens the cache once at startup and again after build
    // discovery confirms the same build. Treating that as a reopen tore down
    // a populated index and rebuilt it from a blob whose records had not been
    // flushed yet, which is how entries went missing while the blob grew.
    if (g_blob && g_idx && build && strcmp(build, g_build) == 0) return true;

    if (!g_clock) g_clock = xSemaphoreCreateMutex();
    g_fs = storage_fs();
    tilecache_close();

    snprintf(g_build, sizeof g_build, "%s", build ? build : "");
    // The one place this path is defined. netsource.cpp used to carry its own
    // unused copy of it - harmless while it stayed unused, but two constants
    // for one directory is how they eventually disagree. IDF's warning flags
    // surfaced it; Arduino's did not.
    g_fs->mkdir(CACHE_DIR);
    snprintf(g_blobPath, sizeof g_blobPath, "%s/%s.dat", CACHE_DIR, g_build);
    snprintf(g_idxPath,  sizeof g_idxPath,  "%s/%s.idx", CACHE_DIR, g_build);

    g_cap = max_entries;
    g_idx = (Entry *)ps_malloc((size_t)g_cap * sizeof(Entry));
    if (!g_idx) { Serial.println("tilecache: index alloc failed"); return false; }
    g_n = 0;

    // "r+" - read and write, positioned anywhere, no truncation.
    //
    // FILE_APPEND is "a", which is write-only: appends land correctly but
    // every seek-and-read fails silently. That made the cache write-only,
    // and worse, made the recovery rescan find nothing in a perfectly good
    // blob and then overwrite a valid index with an empty one.
    // FILE_WRITE is "w" and would truncate, which is its own disaster here.
    g_blob = g_fs->open(g_blobPath, "r+");
    if (!g_blob) {
        g_blob = g_fs->open(g_blobPath, "w+");   // create
    }
    if (!g_blob) { Serial.printf("tilecache: cannot open %s\n", g_blobPath); return false; }

    g_blob_len = g_blob.size();
    if (!idx_load()) rescan();

    g_st.entries = g_n;
    g_st.blob_bytes = g_blob_len;
    Serial.printf("tilecache: %s, %lu entries, %lu KB\n",
                  g_blobPath, (unsigned long)g_n,
                  (unsigned long)(g_blob_len / 1024));
    return true;
}

void tilecache_close() {
    if (g_blob) { idx_save(); g_blob.close(); }
    if (g_idx) { free(g_idx); g_idx = nullptr; }
    g_n = g_cap = 0;
}

bool tilecache_get(uint8_t z, uint32_t x, uint32_t y, uint8_t *dst, uint32_t *len) {
    Guard g;
    if (!g_idx || !g_blob) return false;
    uint64_t id = pmt_zxy_to_tileid(z, x, y);
    bool found;
    int at = find_slot(id, &found);
    if (!found) { g_st.misses++; return false; }

    uint32_t n = g_idx[at].len;
    if (n == 0) { *len = 0; g_st.hits++; return true; }   // negative marker
    if (n > *len) { g_st.misses++; return false; }

    if (!g_blob.seek(g_idx[at].offset)) { g_st.misses++; return false; }
    if (storage_read(g_blob, dst, n) != n) { g_st.misses++; return false; }
    // Reads move the file position, and writes go wherever it happens to be
    // in "r+" mode. tilecache_put seeks explicitly, so this is belt and
    // braces rather than load-bearing.
    g_blob.seek(g_blob_len);
    *len = n;
    g_st.hits++;
    return true;
}

bool tilecache_put(uint8_t z, uint32_t x, uint32_t y, const uint8_t *src, uint32_t len) {
    Guard g;
    if (!g_idx || !g_blob) return false;
    uint64_t id = pmt_zxy_to_tileid(z, x, y);

    bool found;
    find_slot(id, &found);
    if (found) return true;                    // already cached, do not duplicate

    uint32_t off = g_blob_len;
    if (!g_blob.seek(off)) return false;

    RecHdr h = { REC_MAGIC, id, len };
    if (g_blob.write((uint8_t *)&h, sizeof h) != sizeof h) return false;
    if (len && storage_write(g_blob, src, len) != len) return false;
    g_blob_len = off + sizeof h + len;

    if (!idx_insert(id, off + sizeof(RecHdr), len)) {
        Serial.println("tilecache: index full");
        return false;
    }
    g_st.writes++;
    g_st.entries = g_n;
    g_st.blob_bytes = g_blob_len;

    // Batch the index writes: one small file rewrite per FLUSH_INTERVAL tiles
    // rather than per tile. The record headers mean anything appended since
    // the last flush is recoverable by rescan, so this costs restart time
    // rather than data.
    if (++g_since_flush >= flush_interval()) { g_blob.flush(); idx_save(); }
    return true;
}

bool tilecache_wipe() {
    char blobPath[64], idxPath[64];
    snprintf(blobPath, sizeof blobPath, "%s", g_blobPath);
    snprintf(idxPath,  sizeof idxPath,  "%s", g_idxPath);
    char build[16];
    snprintf(build, sizeof build, "%s", g_build);

    tilecache_close();
    if (g_fs) { g_fs->remove(blobPath); g_fs->remove(idxPath); }
    Serial.printf("tilecache: wiped %s\n", blobPath);
    return tilecache_open(build, g_cap ? g_cap : CACHE_MAX_ENTRIES_CFG);
}

void tilecache_flush() {
    Guard g;
    if (g_blob) { g_blob.flush(); idx_save(); }
}

void tilecache_stats(CacheStats *out) { *out = g_st; }

uint32_t tilecache_pending() { return g_since_flush; }
uint32_t tilecache_entry_count() { return g_n; }
