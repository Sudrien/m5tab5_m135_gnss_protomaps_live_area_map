// inflate.h - minimal, dependency-free DEFLATE / zlib / gzip decompressor.
//
// Exists because ESP-IDF does not reliably expose zlib through the Arduino
// layer. Single-shot only: whole input in, whole output out, no streaming,
// no allocation. That matches how PMTiles data is consumed - a directory or
// a tile is always inflated in one go into a caller-owned buffer.

#ifndef TC_INFLATE_H
#define TC_INFLATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    INF_OK       =  0,
    INF_EDATA    = -1,   // corrupt stream
    INF_EOUTPUT  = -2,   // output buffer too small
    INF_EINPUT   = -3,   // ran off the end of the input
    INF_EHEADER  = -4    // bad gzip/zlib wrapper
} inf_err_t;

// Raw DEFLATE (RFC 1951). *out_len starts as capacity, ends as byte count.
inf_err_t inflate_raw(const uint8_t *in, uint32_t in_len,
                      uint8_t *out, uint32_t *out_len);

// gzip (RFC 1952) or zlib (RFC 1950) wrapper, auto-detected.
// Verifies the gzip CRC-32 and length trailer.
inf_err_t inflate_auto(const uint8_t *in, uint32_t in_len,
                       uint8_t *out, uint32_t *out_len);

// Same, but skips the CRC check. Roughly 3.5x faster on large payloads.
//
// Which to use is a real tradeoff, not a default. Directories are small and
// a corrupt one produces wrong tile offsets, so verify those. Tile payloads
// are ~20x larger and corruption there shows up as visibly wrong geometry
// rather than a crash, so skipping the check is defensible when the frame
// budget is tight. SD cards carry their own ECC, so undetected corruption is
// already unlikely.
inf_err_t inflate_auto_fast(const uint8_t *in, uint32_t in_len,
                            uint8_t *out, uint32_t *out_len);

// Uncompressed size recorded in a gzip trailer, or 0 if the input is not
// gzip or is too short. Exact, and available before inflating - which turns
// "output buffer too small" from something you discover by failing into
// something you can size for.
uint32_t gzip_isize(const uint8_t *in, uint32_t in_len);

const char *inflate_strerror(inf_err_t e);

#ifdef __cplusplus
}
#endif
#endif
