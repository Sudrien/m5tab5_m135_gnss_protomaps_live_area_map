// inflate.c - DEFLATE / zlib / gzip decompressor (RFC 1950/1951/1952).

#include "inflate.h"
#include <string.h>

// A primary lookup table of this many bits resolves most symbols in one step;
// longer codes fall back to a bit-by-bit walk. 9 bits covers the overwhelming
// majority of real literal/length codes while keeping the table at 512
// entries (1 KiB), which fits in cache on the P4.
#define FAST_BITS  9
#define FAST_SIZE  (1 << FAST_BITS)
// src: RFC 1951 (DEFLATE) section 3.2.7 - Huffman code lengths in the
//      dynamic block header are encoded in 3 bits with a maximum of 15, and
//      no code in a DEFLATE stream may be longer than that.
#define MAX_BITS   15

typedef struct {
    uint16_t counts[MAX_BITS + 1];   // number of codes of each length
    uint16_t symbols[288];           // symbols ordered by code
    // fast[i] = (len << 12) | symbol, or 0 when the code is longer than
    // FAST_BITS and must be walked.
    uint16_t fast[FAST_SIZE];
} huff_t;

typedef struct {
    const uint8_t *in;
    uint32_t in_len, in_pos;
    uint32_t bitbuf;
    int      bitcnt;
    uint8_t *out;
    uint32_t out_cap, out_pos;
    int      over;      // phantom zero bits currently sitting in bitbuf
    int      err;
} inf_t;

// ---- bit reader (DEFLATE is LSB-first) -------------------------------------
// The reader runs up to 32 bits ahead of what the decoder has consumed, so
// reaching the end of input during a refill is normal for a well-formed
// stream. Zero bits are appended and counted in `over`; the error is raised
// only if the decoder actually *consumes* them, which means the stream really
// was truncated.
static void refill(inf_t *s) {
    while (s->bitcnt <= 24) {
        if (s->in_pos >= s->in_len) {
            s->bitcnt += 8;      // phantom zero byte, bitbuf already has 0s
            s->over   += 8;
            continue;
        }
        s->bitbuf |= (uint32_t)s->in[s->in_pos++] << s->bitcnt;
        s->bitcnt += 8;
    }
}

static inline uint32_t peek(inf_t *s, int n) {
    if (s->bitcnt < n) refill(s);
    return s->bitbuf & ((1u << n) - 1);
}
static inline void drop(inf_t *s, int n) {
    s->bitbuf >>= n;
    s->bitcnt  -= n;
    // Phantom bits live at the top of the buffer; if the real-bit count has
    // gone negative we have consumed past the end of the input.
    if (s->bitcnt < s->over) {
        s->over = s->bitcnt;
        if (!s->err) s->err = INF_EINPUT;
    }
}
static inline uint32_t bits(inf_t *s, int n) {
    if (n == 0) return 0;
    uint32_t v = peek(s, n);
    drop(s, n);
    return v;
}

// ---- Huffman ---------------------------------------------------------------
// Build canonical codes from a length vector, per RFC 1951 section 3.2.2.
static int huff_build(huff_t *h, const uint8_t *lens, int n) {
    memset(h->counts, 0, sizeof h->counts);
    for (int i = 0; i < n; i++) h->counts[lens[i]]++;
    h->counts[0] = 0;

    // Reject over-subscribed and (except for the degenerate single-code case)
    // incomplete tables.
    int left = 1;
    for (int l = 1; l <= MAX_BITS; l++) {
        left <<= 1;
        left -= h->counts[l];
        if (left < 0) return -1;
    }

    uint16_t offs[MAX_BITS + 2];
    offs[1] = 0;
    for (int l = 1; l <= MAX_BITS; l++) offs[l + 1] = offs[l] + h->counts[l];
    for (int i = 0; i < n; i++)
        if (lens[i]) h->symbols[offs[lens[i]]++] = (uint16_t)i;

    // Fast table: for every code of length <= FAST_BITS, fill every entry
    // whose low bits match the (bit-reversed) code.
    memset(h->fast, 0, sizeof h->fast);
    uint32_t code = 0;
    int si = 0;
    for (int l = 1; l <= MAX_BITS; l++) {
        for (int c = 0; c < h->counts[l]; c++, si++) {
            if (l <= FAST_BITS) {
                // reverse the l-bit code, since we read LSB-first
                uint32_t rev = 0;
                for (int b = 0; b < l; b++)
                    if (code & (1u << (l - 1 - b))) rev |= 1u << b;
                uint16_t entry = (uint16_t)((l << 12) | h->symbols[si]);
                for (uint32_t f = rev; f < FAST_SIZE; f += (1u << l))
                    h->fast[f] = entry;
            }
            code++;
        }
        code <<= 1;
    }
    return 0;
}

static int huff_decode(inf_t *s, const huff_t *h) {
    uint32_t look = peek(s, FAST_BITS);
    uint16_t e = h->fast[look];
    if (e) {
        int len = e >> 12;
        drop(s, len);
        return e & 0x0FFF;
    }
    // Slow path: accumulate one bit at a time, comparing against the first
    // code of each length.
    int code = 0, first = 0, index = 0;
    for (int l = 1; l <= MAX_BITS; l++) {
        code |= (int)bits(s, 1);
        int count = h->counts[l];
        if (code - first < count) return h->symbols[index + (code - first)];
        index += count;
        first  = (first + count) << 1;
        code <<= 1;
    }
    s->err = INF_EDATA;
    return -1;
}

// ---- length / distance tables ---------------------------------------------
//
// src: RFC 1951 section 3.2.5, the two tables headed "Extra Bits / Length"
//      and "Extra Bits / Dist". Transcribed, not derived - the values are
//      the format, so a mismatch here decodes garbage rather than failing.
static const uint16_t LEN_BASE[29] = {
    3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258
};
static const uint8_t  LEN_EXTRA[29] = {
    0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0
};
static const uint16_t DIST_BASE[30] = {
    1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,
    1025,1537,2049,3073,4097,6145,8193,12289,16385,24577
};
static const uint8_t  DIST_EXTRA[30] = {
    0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13
};

// ---- block decoding --------------------------------------------------------
static int inflate_block(inf_t *s, const huff_t *lit, const huff_t *dist) {
    for (;;) {
        int sym = huff_decode(s, lit);
        if (sym < 0 || s->err) return s->err ? s->err : INF_EDATA;

        if (sym < 256) {
            if (s->out_pos >= s->out_cap) return INF_EOUTPUT;
            s->out[s->out_pos++] = (uint8_t)sym;
            continue;
        }
        if (sym == 256) return INF_OK;              // end of block

        sym -= 257;
        if (sym >= 29) return INF_EDATA;
        uint32_t len = LEN_BASE[sym] + bits(s, LEN_EXTRA[sym]);

        int dsym = huff_decode(s, dist);
        if (dsym < 0 || dsym >= 30) return INF_EDATA;
        uint32_t d = DIST_BASE[dsym] + bits(s, DIST_EXTRA[dsym]);

        if (d > s->out_pos) return INF_EDATA;       // reference before start
        if (s->out_pos + len > s->out_cap) return INF_EOUTPUT;

        // Overlapping copies are legal and common (run-length encoding), so
        // this must be a byte loop, not memcpy.
        uint8_t *src = s->out + s->out_pos - d;
        uint8_t *dst = s->out + s->out_pos;
        for (uint32_t i = 0; i < len; i++) dst[i] = src[i];
        s->out_pos += len;
    }
}

static void build_fixed(huff_t *lit, huff_t *dist) {
    uint8_t l[288];
    int i = 0;
    for (; i < 144; i++) l[i] = 8;
    for (; i < 256; i++) l[i] = 9;
    for (; i < 280; i++) l[i] = 7;
    for (; i < 288; i++) l[i] = 8;
    huff_build(lit, l, 288);

    uint8_t d[30];
    for (i = 0; i < 30; i++) d[i] = 5;
    huff_build(dist, d, 30);
}

// src: RFC 1951 section 3.2.7 - the fixed permutation the code-length
//      alphabet is transmitted in, given there verbatim as
//      16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15.
static const uint8_t CLEN_ORDER[19] = {
    16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15
};

static int inflate_dynamic(inf_t *s, huff_t *lit, huff_t *dist) {
    uint32_t hlit  = bits(s, 5) + 257;
    uint32_t hdist = bits(s, 5) + 1;
    uint32_t hclen = bits(s, 4) + 4;
    if (hlit > 286 || hdist > 30) return INF_EDATA;

    uint8_t clens[19];
    memset(clens, 0, sizeof clens);
    for (uint32_t i = 0; i < hclen; i++) clens[CLEN_ORDER[i]] = (uint8_t)bits(s, 3);

    huff_t cl;
    if (huff_build(&cl, clens, 19) != 0) return INF_EDATA;

    uint8_t lens[288 + 30];
    memset(lens, 0, sizeof lens);
    uint32_t n = 0;
    while (n < hlit + hdist) {
        int sym = huff_decode(s, &cl);
        if (sym < 0) return INF_EDATA;
        if (sym < 16) {
            lens[n++] = (uint8_t)sym;
        } else if (sym == 16) {
            if (n == 0) return INF_EDATA;
            uint8_t prev = lens[n - 1];
            uint32_t r = 3 + bits(s, 2);
            while (r-- && n < hlit + hdist) lens[n++] = prev;
        } else if (sym == 17) {
            uint32_t r = 3 + bits(s, 3);
            while (r-- && n < hlit + hdist) lens[n++] = 0;
        } else {
            uint32_t r = 11 + bits(s, 7);
            while (r-- && n < hlit + hdist) lens[n++] = 0;
        }
        if (s->err) return s->err;
    }
    if (huff_build(lit, lens, (int)hlit) != 0) return INF_EDATA;
    // A single-symbol or empty distance tree is legal when no matches occur.
    huff_build(dist, lens + hlit, (int)hdist);
    return INF_OK;
}

inf_err_t inflate_raw(const uint8_t *in, uint32_t in_len,
                      uint8_t *out, uint32_t *out_len)
{
    inf_t s;
    memset(&s, 0, sizeof s);
    s.in = in; s.in_len = in_len;
    s.out = out; s.out_cap = *out_len;

    // On the stack, not static.
    //
    // These were static to keep ~3.2 KiB off the stack, which quietly made
    // the decoder non-reentrant. Two tasks inflating at once then shared one
    // set of Huffman tables, and the second call rebuilt them underneath the
    // first - producing a "corrupt stream" on input that is perfectly valid.
    //
    // The cost is that a caller needs roughly 3.5 KiB of stack headroom. That
    // is cheaper than the alternatives: a lock would make a portable core
    // depend on a threading API, and a per-call allocation would put malloc
    // in the hot path.
    huff_t lit, dist;
    int final = 0;

    do {
        final = (int)bits(&s, 1);
        uint32_t type = bits(&s, 2);
        if (s.err) return (inf_err_t)s.err;

        if (type == 0) {                     // stored
            drop(&s, s.bitcnt & 7);          // align to byte
            if (s.err) return (inf_err_t)s.err;
            // The bit buffer may still hold whole real bytes; rewind the
            // input cursor past them, ignoring any phantom padding.
            uint32_t real = (uint32_t)((s.bitcnt - s.over) / 8);
            uint32_t pos = s.in_pos - real;
            s.bitbuf = 0; s.bitcnt = 0; s.over = 0;
            if (pos + 4 > in_len) return INF_EINPUT;
            uint32_t len  = in[pos] | ((uint32_t)in[pos + 1] << 8);
            uint32_t nlen = in[pos + 2] | ((uint32_t)in[pos + 3] << 8);
            if ((len ^ 0xFFFF) != nlen) return INF_EDATA;
            pos += 4;
            if (pos + len > in_len) return INF_EINPUT;
            if (s.out_pos + len > s.out_cap) return INF_EOUTPUT;
            memcpy(s.out + s.out_pos, in + pos, len);
            s.out_pos += len;
            s.in_pos = pos + len;
        } else if (type == 1) {
            build_fixed(&lit, &dist);
            int r = inflate_block(&s, &lit, &dist);
            if (r != INF_OK) return (inf_err_t)r;
        } else if (type == 2) {
            int r = inflate_dynamic(&s, &lit, &dist);
            if (r != INF_OK) return (inf_err_t)r;
            r = inflate_block(&s, &lit, &dist);
            if (r != INF_OK) return (inf_err_t)r;
        } else {
            return INF_EDATA;
        }
    } while (!final);

    *out_len = s.out_pos;
    return INF_OK;
}

// CRC-32 (RFC 1952), nibble-table driven. A 16-entry table is 64 bytes of
// rodata and runs roughly 6x faster than the bitwise loop - the full 256-entry
// table is only marginally quicker again and costs 1 KiB, which is not worth
// it when this runs once per tile.
// src: the standard CRC-32 of RFC 1952 (gzip), polynomial 0xEDB88320
//      reflected. These sixteen entries are the nibble-at-a-time form: the
//      256-entry table with the low four index bits held at zero.
static const uint32_t CRC_TAB[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu
};

static uint32_t crc32_buf(const uint8_t *p, uint32_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        c = (c >> 4) ^ CRC_TAB[c & 0x0F];
        c = (c >> 4) ^ CRC_TAB[c & 0x0F];
    }
    return ~c;
}

static inf_err_t inflate_wrapped(const uint8_t *in, uint32_t in_len,
                                 uint8_t *out, uint32_t *out_len, int verify)
{
    if (in_len < 2) return INF_EHEADER;

    // gzip: 1f 8b 08, then flags decide what optional fields follow.
    if (in[0] == 0x1F && in[1] == 0x8B) {
        if (in_len < 18 || in[2] != 8) return INF_EHEADER;
        uint8_t flg = in[3];
        uint32_t p = 10;
        if (flg & 0x04) {                       // FEXTRA
            if (p + 2 > in_len) return INF_EHEADER;
            uint32_t xlen = in[p] | ((uint32_t)in[p + 1] << 8);
            p += 2 + xlen;
        }
        if (flg & 0x08) { while (p < in_len && in[p]) p++; p++; }   // FNAME
        if (flg & 0x10) { while (p < in_len && in[p]) p++; p++; }   // FCOMMENT
        if (flg & 0x02) p += 2;                                     // FHCRC
        if (p + 8 >= in_len) return INF_EHEADER;
        inf_err_t e = inflate_raw(in + p, in_len - p - 8, out, out_len);
        if (e != INF_OK) return e;

        // Trailer: CRC-32 then ISIZE, both little-endian. Checking these is
        // what turns a corrupt tile into a clean error instead of garbage
        // geometry drawn on screen.
        const uint8_t *t = in + in_len - 8;
        uint32_t want_crc = (uint32_t)t[0] | ((uint32_t)t[1] << 8) |
                            ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
        uint32_t want_len = (uint32_t)t[4] | ((uint32_t)t[5] << 8) |
                            ((uint32_t)t[6] << 16) | ((uint32_t)t[7] << 24);
        if (*out_len != want_len) return INF_EDATA;
        if (verify && crc32_buf(out, *out_len) != want_crc) return INF_EDATA;
        (void)want_crc;
        return INF_OK;
    }

    // zlib: CMF/FLG with CM=8 and a valid check value.
    if ((in[0] & 0x0F) == 8 && ((in[0] << 8) | in[1]) % 31 == 0) {
        if (in[1] & 0x20) return INF_EHEADER;   // preset dictionary
        return inflate_raw(in + 2, in_len - 2, out, out_len);
    }

    return inflate_raw(in, in_len, out, out_len);   // assume raw
}

inf_err_t inflate_auto(const uint8_t *in, uint32_t in_len,
                       uint8_t *out, uint32_t *out_len) {
    return inflate_wrapped(in, in_len, out, out_len, 1);
}

inf_err_t inflate_auto_fast(const uint8_t *in, uint32_t in_len,
                            uint8_t *out, uint32_t *out_len) {
    return inflate_wrapped(in, in_len, out, out_len, 0);
}

uint32_t gzip_isize(const uint8_t *in, uint32_t in_len) {
    if (in_len < 18 || in[0] != 0x1F || in[1] != 0x8B) return 0;
    const uint8_t *t = in + in_len - 4;
    return (uint32_t)t[0] | ((uint32_t)t[1] << 8) |
           ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
}

const char *inflate_strerror(inf_err_t e) {
    switch (e) {
    case INF_OK:      return "ok";
    case INF_EDATA:   return "corrupt stream";
    case INF_EOUTPUT: return "output buffer too small";
    case INF_EINPUT:  return "truncated input";
    case INF_EHEADER: return "bad wrapper header";
    }
    return "?";
}
