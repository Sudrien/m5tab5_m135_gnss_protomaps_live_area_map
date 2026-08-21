// pmtiles.h - PMTiles v3 archive reader.
//
// Spec: https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md
//
// Portable core: the archive is reached through two caller-supplied callbacks,
// so the same code serves a FILE* on desktop, an SD card on the ESP32-P4, and
// HTTP range requests over Wi-Fi. Nothing here allocates except the directory
// cache, and that is caller-sized at init.
//
// Thread safety: a pmt_t is NOT internally locked. Either confine it to one
// task or wrap calls in a mutex. On the Tab5 the render worker owns it.

#ifndef PMTILES_H
#define PMTILES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- enums from the spec ---------------------------------------------------
enum {
    PMT_COMPRESS_UNKNOWN = 0,
    PMT_COMPRESS_NONE    = 1,
    PMT_COMPRESS_GZIP    = 2,
    PMT_COMPRESS_BROTLI  = 3,
    PMT_COMPRESS_ZSTD    = 4,
};

enum {
    PMT_TYPE_UNKNOWN = 0,
    PMT_TYPE_MVT     = 1,
    PMT_TYPE_PNG     = 2,
    PMT_TYPE_JPEG    = 3,
    PMT_TYPE_WEBP    = 4,
    PMT_TYPE_AVIF    = 5,
};

// ---- return codes ----------------------------------------------------------
typedef enum {
    PMT_OK          =  0,
    PMT_NOTFOUND    =  1,   // no tile at this z/x/y (ocean, outside extract)
    PMT_EIO         = -1,   // read callback failed
    PMT_EFORMAT     = -2,   // bad magic / version / truncated structure
    PMT_EDECOMPRESS = -3,   // decompress callback failed or unsupported codec
    PMT_ENOMEM      = -4,   // scratch buffer too small
    PMT_ERANGE      = -5,   // zoom outside archive min/max
} pmt_err_t;

// ---- caller-supplied IO ----------------------------------------------------
// Read `len` bytes at absolute `off` into `dst`. Return 0 on success.
// Must be exact: short reads are an error, not a partial success.
typedef int (*pmt_read_fn)(void *ctx, uint64_t off, uint32_t len, uint8_t *dst);

// Decompress `src_len` bytes into `dst`, writing the produced size to
// *dst_len (which starts as the capacity). Return 0 on success.
// `codec` is one of PMT_COMPRESS_*. The reader only ever asks for the
// archive's internal-compression codec here; tile payloads are handed back
// still compressed, for the caller to inflate on its own schedule.
typedef int (*pmt_inflate_fn)(void *ctx, uint8_t codec,
                              const uint8_t *src, uint32_t src_len,
                              uint8_t *dst, uint32_t *dst_len);

// ---- header ----------------------------------------------------------------
typedef struct {
    uint64_t root_off,  root_len;
    uint64_t meta_off,  meta_len;
    uint64_t leaf_off,  leaf_len;
    uint64_t data_off,  data_len;
    uint64_t n_addressed, n_entries, n_contents;
    uint8_t  clustered;
    uint8_t  internal_compression;
    uint8_t  tile_compression;
    uint8_t  tile_type;
    uint8_t  min_zoom, max_zoom;
    int32_t  min_lon_e7, min_lat_e7, max_lon_e7, max_lat_e7;
    uint8_t  center_zoom;
    int32_t  center_lon_e7, center_lat_e7;
} pmt_header_t;

// ---- leaf directory cache slot ---------------------------------------------
// One decompressed directory, plus the identity of where it came from. The
// caller owns `buf` and sets `cap`; everything else belongs to the reader.
//
// Zero the struct to mean empty. A slot whose buf is NULL is skipped, so a
// caller that allocates lazily can hand over an array of zeroes and fill it
// in as sizes become known.
typedef struct {
    uint8_t  *buf;
    uint32_t  cap;
    uint64_t  off;         // source offset these contents came from
    uint32_t  srclen;      // source (compressed) length
    uint32_t  len;         // decompressed length held, 0 = empty
    uint32_t  used;        // LRU tick, highest is most recent
} pmt_dir_slot_t;

// ---- reader handle ---------------------------------------------------------
typedef struct {
    pmt_header_t   hdr;

    pmt_read_fn    read;
    pmt_inflate_fn inflate;
    void          *io_ctx;

    // Scratch for holding one *decompressed* directory. Sized by the caller;
    // must be large enough for the largest directory in the archive. Root
    // directories are capped at 16 KiB compressed by convention; leaves are
    // usually smaller. 64 KiB is a safe default, 32 KiB is usually plenty.
    uint8_t  *dir_buf;
    uint32_t  dir_cap;

    // Scratch for the raw (still compressed) directory bytes read from the
    // archive, before inflate. May alias nothing else.
    uint8_t  *raw_buf;
    uint32_t  raw_cap;

    // Single-entry root directory cache. The root is consulted on every
    // lookup, so caching it turns most lookups into one seek instead of two.
    uint8_t  *root_cache;      // optional; NULL disables
    uint32_t  root_cache_cap;
    uint32_t  root_cache_len;  // 0 = not yet populated

    // Identity of whatever dir_buf currently holds, so a directory that is
    // already decompressed in there is not fetched and inflated again.
    //
    // This costs nothing: dir_buf has to hold a directory anyway, and after
    // the first lookup the root is served from root_cache without touching
    // it, so what survives in dir_buf is the last *leaf*. On a planet archive
    // a leaf covers a contiguous run of tiles, so consecutive lookups hit it
    // - which matters enormously when a leaf is 129 KB over the network.
    //
    // dir_len == 0 means "holds nothing trustworthy".
    uint64_t  dir_off;         // source offset the contents came from
    uint32_t  dir_srclen;      // source (compressed) length
    uint32_t  dir_len;         // decompressed length held, 0 = empty

    // Decompressed size of the root directory, once it has been read at least
    // once. 0 until then.
    //
    // Recorded because whether it fits root_cache_cap decides how much work a
    // lookup costs, and nothing currently reports it. When it fits, the root
    // is copied into root_cache and every later lookup is served from memory.
    // When it does not, load_dir() takes neither that path nor the dir_off
    // identity path below it - the !is_root guard excludes the root - so the
    // root is re-read from the medium and re-inflated on every single lookup,
    // and the leaf that follows overwrites dir_buf so the next lookup does it
    // again. Two full read-and-inflate cycles per tile instead of none.
    uint32_t  root_dec_len;

    // ---- N-way leaf directory cache ---------------------------------------
    //
    // Optional and caller-owned, on the same terms as root_cache: this file
    // still allocates nothing. Leave `leaves` NULL and behaviour is exactly
    // the single dir_buf slot below.
    //
    // The single slot is enough while a lookup walks one leaf and stops. It
    // is not enough across a working set of several. Three leaves in play -
    // one for the z14 view, one for the z12 place block, one for the z6
    // region block - evict each other in rotation, and each eviction costs a
    // ~130 KB read and a full inflate on the next lookup that wants it back.
    // In one boot log the same leaf was read at #5 and again at #12, and
    // another at #3 and #22, purely because a third leaf landed in between.
    //
    // Slots are matched on (off, srclen) like the dir_buf slot, and chosen
    // for replacement by lowest `used`, which is a plain monotonic tick
    // rather than a timestamp so this stays free of any clock.
    //
    // A slot with a NULL buf, or one too small for the directory in hand, is
    // simply not used: the caller may grow it on demand and a cache that
    // cannot hold something must still not lie about holding it.
    // Slots are not owned per-archive by necessity: the caller may share one
    // array between several readers, provided it retires the previous owner
    // by clearing each slot's len. Offsets are only meaningful within one
    // archive, so a shared array that is not retired would match an offset
    // from a different file - the same hazard the dir_buf slot already has.
    pmt_dir_slot_t *leaves;
    uint32_t        leaf_n;        // slots; 0 or NULL leaves = disabled
    uint32_t        leaf_tick;     // LRU counter, bumped on every touch

    // Set whenever a call returns PMT_ENOMEM: the number of bytes that would
    // have been required. Leaf directory sizes are not described anywhere in
    // the header, so without this the caller has no way to size raw_buf
    // except by guessing and retrying.
    uint32_t  need_raw;        // bytes needed in raw_buf
    uint32_t  need_dir;        // bytes needed in dir_buf
} pmt_t;

// ---- API -------------------------------------------------------------------

// Read and validate the 127-byte header. Buffers must already be assigned.
pmt_err_t pmt_open(pmt_t *p);

// Read the root directory once, so root_dec_len is populated and the root
// cache is filled if it fits. Optional: lookups work without it, this just
// moves the first root read off the first tile fetch and makes the size
// available to report at open time.
pmt_err_t pmt_prime_root(pmt_t *p);

// Look up a tile. On PMT_OK, *off/*len describe the tile payload inside the
// archive; the bytes are still in the archive's tile_compression codec.
// Returns PMT_NOTFOUND for a legitimate gap (draw nothing, not an error).
pmt_err_t pmt_find(pmt_t *p, uint8_t z, uint32_t x, uint32_t y,
                   uint64_t *off, uint32_t *len);

// Convenience: look up and read the payload into `dst`. *dst_len starts as
// capacity, ends as the byte count. Still compressed.
//
// Note this repeats the pmt_find walk internally. A caller that has already
// called pmt_find should use pmt_read_blob instead - over a network reader,
// re-walking costs a second fetch of the leaf directory.
pmt_err_t pmt_get(pmt_t *p, uint8_t z, uint32_t x, uint32_t y,
                  uint8_t *dst, uint32_t *dst_len);

// Read a payload whose location is already known, as returned by pmt_find.
// *dst_len starts as capacity, ends as the byte count.
pmt_err_t pmt_read_blob(pmt_t *p, uint64_t off, uint32_t len,
                        uint8_t *dst, uint32_t *dst_len);

// Hilbert tile id, exposed for tests and for cache keys.
uint64_t  pmt_zxy_to_tileid(uint8_t z, uint32_t x, uint32_t y);
void      pmt_tileid_to_zxy(uint64_t id, uint8_t *z, uint32_t *x, uint32_t *y);

const char *pmt_strerror(pmt_err_t e);

#ifdef __cplusplus
}
#endif
#endif // PMTILES_H
