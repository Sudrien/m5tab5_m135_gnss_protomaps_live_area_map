// pmtiles.c - PMTiles v3 reader implementation.

#include "pmtiles.h"
#include <string.h>

// src: PMTiles v3 specification - the header is a fixed 127 bytes, followed
//      by the root directory. Fixed by the format, so reading fewer bytes
//      than this is never valid.
#define PMT_HEADER_LEN 127

// ---- little-endian scalar reads --------------------------------------------
static uint32_t rd32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static uint64_t rd64(const uint8_t *b) {
    return (uint64_t)rd32(b) | ((uint64_t)rd32(b + 4) << 32);
}

// ---- varint ----------------------------------------------------------------
// Protobuf-style base-128, little-endian groups. Returns 0 on success and
// advances *pos. Guards against running off the end and against >10-byte
// encodings (which cannot fit in a uint64).
static int varint(const uint8_t *buf, uint32_t len, uint32_t *pos, uint64_t *out) {
    uint64_t v = 0;
    int shift = 0;
    while (*pos < len) {
        uint8_t b = buf[(*pos)++];
        v |= (uint64_t)(b & 0x7F) << shift;
        if (!(b & 0x80)) { *out = v; return 0; }
        shift += 7;
        if (shift > 63) return -1;
    }
    return -1;   // truncated
}

// ---- Hilbert curve ---------------------------------------------------------
// PMTiles orders tiles along a Hilbert curve so that spatially adjacent tiles
// land near each other in the file. tile_id = (tiles below this zoom) +
// (Hilbert index within this zoom).

uint64_t pmt_zxy_to_tileid(uint8_t z, uint32_t x, uint32_t y) {
    // Sum of 4^t for t in [0,z) == ((4^z)-1)/3, the count of all tiles at
    // lower zooms. Computed iteratively to avoid overflow surprises.
    uint64_t acc = 0;
    for (uint8_t t = 0; t < z; t++) {
        uint64_t side = 1ULL << t;
        acc += side * side;
    }

    uint64_t n = 1ULL << z;
    uint64_t rx, ry, d = 0;
    uint64_t tx = x, ty = y;

    for (uint64_t s = n / 2; s > 0; s /= 2) {
        rx = (tx & s) ? 1 : 0;
        ry = (ty & s) ? 1 : 0;
        d += s * s * ((3 * rx) ^ ry);
        // rotate the quadrant
        if (ry == 0) {
            if (rx == 1) {
                tx = s - 1 - tx;
                ty = s - 1 - ty;
            }
            uint64_t tmp = tx; tx = ty; ty = tmp;
        }
    }
    return acc + d;
}

void pmt_tileid_to_zxy(uint64_t id, uint8_t *z, uint32_t *x, uint32_t *y) {
    uint64_t acc = 0;
    uint8_t zz = 0;
    for (;;) {
        uint64_t side = 1ULL << zz;
        uint64_t count = side * side;
        if (id < acc + count) break;
        acc += count;
        zz++;
        if (zz > 31) { *z = 0; *x = 0; *y = 0; return; }
    }
    uint64_t d = id - acc;
    uint64_t n = 1ULL << zz;
    uint64_t tx = 0, ty = 0, rx, ry, t = d;

    for (uint64_t s = 1; s < n; s *= 2) {
        rx = 1 & (t / 2);
        ry = 1 & (t ^ rx);
        if (ry == 0) {
            if (rx == 1) {
                tx = s - 1 - tx;
                ty = s - 1 - ty;
            }
            uint64_t tmp = tx; tx = ty; ty = tmp;
        }
        tx += s * rx;
        ty += s * ry;
        t /= 4;
    }
    *z = zz; *x = (uint32_t)tx; *y = (uint32_t)ty;
}

// ---- header ----------------------------------------------------------------
pmt_err_t pmt_open(pmt_t *p) {
    if (!p || !p->read || !p->inflate || !p->dir_buf || !p->raw_buf)
        return PMT_EFORMAT;

    uint8_t h[PMT_HEADER_LEN];
    if (p->read(p->io_ctx, 0, PMT_HEADER_LEN, h) != 0) return PMT_EIO;

    if (memcmp(h, "PMTiles", 7) != 0) return PMT_EFORMAT;
    if (h[7] != 3) return PMT_EFORMAT;   // v2 has a different layout entirely

    pmt_header_t *d = &p->hdr;
    d->root_off = rd64(h + 8);   d->root_len = rd64(h + 16);
    d->meta_off = rd64(h + 24);  d->meta_len = rd64(h + 32);
    d->leaf_off = rd64(h + 40);  d->leaf_len = rd64(h + 48);
    d->data_off = rd64(h + 56);  d->data_len = rd64(h + 64);
    d->n_addressed = rd64(h + 72);
    d->n_entries   = rd64(h + 80);
    d->n_contents  = rd64(h + 88);
    d->clustered            = h[96];
    d->internal_compression = h[97];
    d->tile_compression     = h[98];
    d->tile_type            = h[99];
    d->min_zoom = h[100];
    d->max_zoom = h[101];
    d->min_lon_e7 = (int32_t)rd32(h + 102);
    d->min_lat_e7 = (int32_t)rd32(h + 106);
    d->max_lon_e7 = (int32_t)rd32(h + 110);
    d->max_lat_e7 = (int32_t)rd32(h + 114);
    d->center_zoom = h[118];
    d->center_lon_e7 = (int32_t)rd32(h + 119);
    d->center_lat_e7 = (int32_t)rd32(h + 123);

    if (d->root_len == 0) return PMT_EFORMAT;
    if (p->root_cache) p->root_cache_len = 0;   // invalidate
    p->dir_len = 0;                             // dir_buf identity too
    return PMT_OK;
}

// ---- directory load --------------------------------------------------------
// Reads [off,len) from the archive, inflates it into p->dir_buf, and reports
// the decompressed length. Uses the root cache when the range is the root.
// Find a cached leaf holding exactly this directory, or NULL.
static pmt_dir_slot_t *leaf_find(pmt_t *p, uint64_t off, uint32_t len) {
    if (!p->leaves) return NULL;
    for (uint32_t i = 0; i < p->leaf_n; i++) {
        pmt_dir_slot_t *s = &p->leaves[i];
        if (s->buf && s->len && s->off == off && s->srclen == len) return s;
    }
    return NULL;
}

// Pick the slot to overwrite: the first empty one, else the least recently
// used. Slots too small for `need` are skipped rather than grown - this file
// does not allocate - so a cache of undersized slots simply never fills.
static pmt_dir_slot_t *leaf_victim(pmt_t *p, uint32_t need) {
    if (!p->leaves) return NULL;
    pmt_dir_slot_t *best = NULL;
    for (uint32_t i = 0; i < p->leaf_n; i++) {
        pmt_dir_slot_t *s = &p->leaves[i];
        if (!s->buf || s->cap < need) continue;
        if (!s->len) return s;                          // empty, take it
        if (!best || s->used < best->used) best = s;
    }
    return best;
}

static pmt_err_t load_dir(pmt_t *p, uint64_t off, uint32_t len,
                          const uint8_t **out, uint32_t *out_len) {
    int is_root = (off == p->hdr.root_off && len == p->hdr.root_len);

    if (is_root && p->root_cache && p->root_cache_len) {
        *out = p->root_cache;
        *out_len = p->root_cache_len;
        return PMT_OK;
    }

    // Already decompressed in dir_buf from a previous lookup.
    if (!is_root && p->dir_len && off == p->dir_off && len == p->dir_srclen) {
        *out = p->dir_buf;
        *out_len = p->dir_len;
        return PMT_OK;
    }

    // Held in the leaf cache from some lookup further back. Checked after
    // dir_buf because dir_buf is the same directory without the copy, and
    // before the read because that is the whole point.
    if (!is_root) {
        pmt_dir_slot_t *s = leaf_find(p, off, len);
        if (s) {
            s->used = ++p->leaf_tick;
            *out = s->buf;
            *out_len = s->len;
            return PMT_OK;
        }
    }

    // From here dir_buf is about to be overwritten, so what it held is no
    // longer valid - including if the read or inflate below fails.
    p->dir_len = 0;

    if (len > p->raw_cap) { p->need_raw = len; return PMT_ENOMEM; }
    if (p->read(p->io_ctx, off, len, p->raw_buf) != 0) return PMT_EIO;

    const uint8_t *src = p->raw_buf;
    uint32_t src_len = len;
    uint32_t dec_len = p->dir_cap;

    if (p->hdr.internal_compression == PMT_COMPRESS_NONE) {
        if (src_len > p->dir_cap) { p->need_dir = src_len; return PMT_ENOMEM; }
        memcpy(p->dir_buf, src, src_len);
        dec_len = src_len;
    } else {
        if (p->inflate(p->io_ctx, p->hdr.internal_compression,
                       src, src_len, p->dir_buf, &dec_len) != 0)
            return PMT_EDECOMPRESS;
    }

    if (is_root) p->root_dec_len = dec_len;

    if (is_root && p->root_cache && dec_len <= p->root_cache_cap) {
        memcpy(p->root_cache, p->dir_buf, dec_len);
        p->root_cache_len = dec_len;
        *out = p->root_cache;
        *out_len = dec_len;
        return PMT_OK;
    }

    // Remember what dir_buf now holds. Not done for the root: when the root
    // is served from root_cache it never reaches here, and when it is not,
    // leaving dir_len at 0 simply means no reuse rather than a wrong hit.
    if (!is_root) {
        p->dir_off = off;
        p->dir_srclen = len;
        p->dir_len = dec_len;

        // Copy into the leaf cache so the next eviction of dir_buf does not
        // lose it. The copy is the cost of this cache and it is worth it: it
        // is a PSRAM memcpy against a re-read and re-inflate of the same
        // directory, which is two orders of magnitude more work.
        //
        // Failure to place it is not an error anywhere - too few slots, or
        // none large enough - and leaves the reader exactly as it was before
        // this cache existed.
        pmt_dir_slot_t *s = leaf_victim(p, dec_len);
        if (s) {
            memcpy(s->buf, p->dir_buf, dec_len);
            s->off = off;
            s->srclen = len;
            s->len = dec_len;
            s->used = ++p->leaf_tick;
        }
    }

    *out = p->dir_buf;
    *out_len = dec_len;
    return PMT_OK;
}

// ---- directory search ------------------------------------------------------
// A serialised directory is five varint runs, each of `n` values, in order:
//   tile_id deltas, run_lengths, lengths, offsets
// (offset 0 is the sentinel meaning "contiguous with the previous entry").
//
// Entries are sorted by tile_id, so a binary search would be ideal - but the
// varint encoding is not random-access, so we stream it once, tracking the
// best candidate. Directories are small (hundreds to low thousands of
// entries), so this is a few microseconds even on the P4.
//
// Result semantics:
//   found_run == 0  -> the entry points at a LEAF directory, recurse
//   found_run  > 0  -> the entry covers [tile_id, tile_id+run) , tile data
typedef struct {
    int      found;
    uint64_t offset;
    uint32_t length;
    uint64_t run;
} dirhit_t;

// Walk `n` varints starting at *pos, discarding the values. Used to skip a
// whole run to reach the next section.
static int skip_run(const uint8_t *buf, uint32_t len, uint32_t *pos, uint32_t n) {
    uint64_t tmp;
    for (uint32_t i = 0; i < n; i++)
        if (varint(buf, len, pos, &tmp) != 0) return -1;
    return 0;
}

static pmt_err_t search_dir(const uint8_t *buf, uint32_t len,
                            uint64_t target, dirhit_t *hit)
{
    uint32_t pos = 0;
    uint64_t n64;
    if (varint(buf, len, &pos, &n64) != 0) return PMT_EFORMAT;
    hit->found = 0;
    if (n64 == 0) return PMT_OK;
    uint32_t n = (uint32_t)n64;

    // --- section 1: tile_id deltas ---
    // Walk all n even after finding the candidate, so `pos` lands cleanly on
    // the start of the next section.
    uint64_t id = 0, best_id = 0;
    uint32_t best_i = 0;
    int have_best = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t delta;
        if (varint(buf, len, &pos, &delta) != 0) return PMT_EFORMAT;
        id += delta;
        if (id <= target) { best_i = i; best_id = id; have_best = 1; }
    }
    if (!have_best) return PMT_OK;

    // --- section 2: run lengths ---
    uint64_t run = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t v;
        if (varint(buf, len, &pos, &v) != 0) return PMT_EFORMAT;
        if (i == best_i) run = v;
    }

    // --- sections 3 and 4: lengths, then offsets ---
    // The offset sentinel (0) means "immediately after the previous entry",
    // so resolving offset[best_i] needs the running chain of lengths and
    // offsets from entry 0. Two cursors walk the two sections in lockstep.
    uint32_t p_len = pos;
    if (skip_run(buf, len, &pos, n) != 0) return PMT_EFORMAT;
    uint32_t p_off = pos;

    uint64_t offset = 0, length = 0;
    uint64_t prev_off = 0, prev_len = 0;
    for (uint32_t i = 0; i <= best_i; i++) {
        uint64_t li, oi;
        if (varint(buf, len, &p_len, &li) != 0) return PMT_EFORMAT;
        if (varint(buf, len, &p_off, &oi) != 0) return PMT_EFORMAT;

        uint64_t this_off = (oi == 0) ? (prev_off + prev_len) : (oi - 1);
        if (i == best_i) { offset = this_off; length = li; break; }
        prev_off = this_off;
        prev_len = li;
    }

    // A leaf pointer (run == 0) is always followed. A data entry only matches
    // if the target falls inside its run of consecutive tile ids.
    if (run != 0 && target >= best_id + run) return PMT_OK;

    hit->found  = 1;
    hit->offset = offset;
    hit->length = (uint32_t)length;
    hit->run    = run;
    return PMT_OK;
}

pmt_err_t pmt_find(pmt_t *p, uint8_t z, uint32_t x, uint32_t y,
                   uint64_t *off, uint32_t *len)
{
    if (z < p->hdr.min_zoom || z > p->hdr.max_zoom) return PMT_ERANGE;

    uint64_t target = pmt_zxy_to_tileid(z, x, y);

    uint64_t d_off = p->hdr.root_off;
    uint32_t d_len = (uint32_t)p->hdr.root_len;

    // Depth is bounded: real archives use at most 3 levels. The cap stops a
    // corrupt file from spinning us forever.
    for (int depth = 0; depth < 4; depth++) {
        const uint8_t *buf; uint32_t blen;
        pmt_err_t e = load_dir(p, d_off, d_len, &buf, &blen);
        if (e != PMT_OK) return e;

        dirhit_t hit = {0};
        e = search_dir(buf, blen, target, &hit);
        if (e != PMT_OK) return e;
        if (!hit.found) return PMT_NOTFOUND;

        if (hit.run == 0) {
            // leaf directory pointer; offsets are relative to leaf_off
            d_off = p->hdr.leaf_off + hit.offset;
            d_len = hit.length;
            continue;
        }
        *off = p->hdr.data_off + hit.offset;
        *len = hit.length;
        return PMT_OK;
    }
    return PMT_EFORMAT;
}

pmt_err_t pmt_prime_root(pmt_t *p) {
    const uint8_t *buf; uint32_t blen;
    return load_dir(p, p->hdr.root_off, (uint32_t)p->hdr.root_len, &buf, &blen);
}

pmt_err_t pmt_read_blob(pmt_t *p, uint64_t off, uint32_t len,
                        uint8_t *dst, uint32_t *dst_len)
{
    if (len > *dst_len) { p->need_raw = len; return PMT_ENOMEM; }
    if (p->read(p->io_ctx, off, len, dst) != 0) return PMT_EIO;
    *dst_len = len;
    return PMT_OK;
}

pmt_err_t pmt_get(pmt_t *p, uint8_t z, uint32_t x, uint32_t y,
                  uint8_t *dst, uint32_t *dst_len)
{
    uint64_t off; uint32_t len;
    pmt_err_t e = pmt_find(p, z, x, y, &off, &len);
    if (e != PMT_OK) return e;
    return pmt_read_blob(p, off, len, dst, dst_len);
}

const char *pmt_strerror(pmt_err_t e) {
    switch (e) {
    case PMT_OK:          return "ok";
    case PMT_NOTFOUND:    return "no tile at that z/x/y";
    case PMT_EIO:         return "read failed";
    case PMT_EFORMAT:     return "bad or truncated archive";
    case PMT_EDECOMPRESS: return "decompress failed";
    case PMT_ENOMEM:      return "buffer too small";
    case PMT_ERANGE:      return "zoom outside archive range";
    }
    return "?";
}
