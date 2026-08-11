// netsource.cpp

#include "netsource.h"
#include <Arduino.h>
#include "storage.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_heap_caps.h>
#include <time.h>
#include <esp_sntp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

#include "tilecache.h"
#include "mapconfig.h"

extern "C" {
  #include "pmtiles.h"
  #include "inflate.h"
}

// WHERE TILES COME FROM
//
// Protomaps explicitly discourage hotlinking their build bucket and warn that
// the URLs may move: the documented guidance is to copy the tileset to your
// own cloud storage and serve it from there. A single hobby device is
// negligible traffic, but the right thing - and the thing that will not break
// when they reorganise - is to point this at storage you control.
//
// Any host that answers HTTP range requests works, since PMTiles needs
// nothing else. Cloudflare R2, S3, or a file on a home server all qualify.
//
// TILE_BASE is the directory; the archive name is appended. Leave it as the
// upstream bucket for evaluation, then change it once you have your own copy.
#ifndef TILE_BASE
#define TILE_BASE "https://build.protomaps.com/"
#endif

// When PINNED_BUILD is set, that archive is used verbatim and no date probing
// happens. Protomaps retain the latest build of each patch version
// indefinitely, unlike dailies which age out after about a week - so a pinned
// version is the stable choice, and the only sensible one when serving from
// your own storage.
//   e.g. #define PINNED_BUILD "v4.3.0"
#ifndef PINNED_BUILD
#define PINNED_BUILD ""
#endif

static const char *BUILD_HOST   = TILE_BASE;
static const char *MANIFEST     = "/t/build.txt";
static const uint32_t REFRESH_DAYS = 30;

// Index entries held in PSRAM: 16 bytes each, so 200k tiles costs 3.2 MB.
// A region at z15 comfortably fits; the whole planet would not, which is
// what the build-scoped cache is for.
static const uint32_t CACHE_MAX_ENTRIES = CACHE_MAX_ENTRIES_CFG;

// Protomaps retains about a week of daily builds, so probing more than that
// is pointless - if nothing in the last 8 days answers, the network or the
// service is down, not the date arithmetic.
static const int MAX_PROBE_DAYS = 8;

static fs::FS *g_fs = nullptr;
// ---- local archives --------------------------------------------------------
// A set rather than a single file, because FAT32 caps one file at 4 GiB and
// the archive this needs is larger than that.
//
// Merging extracts into one world.pmtiles runs straight back into the same
// wall - one file is one file. Splitting is the way out: a PMTiles header
// carries min_zoom, max_zoom and a bounding box, so each archive already
// describes what it holds. Open several, ask each whether it covers the tile
// being requested, and the 4 GiB limit applies per archive instead of to the
// whole map.
//
// That also makes the split match how the tiles are used. Only z0-6, z11 and
// z14 are ever requested, and a contiguous extract would spend most of its
// size on levels nothing asks for. Three files, one per band, keeps each well
// clear of the limit and skips the rest entirely.
// Raise this if a plan needs more bands than it allows. plan-extracts.py will
// happily emit up to 40 - a global z14 split at 3.2 GB a file is around ten,
// but a smaller --target or a finer zoom gets there quickly. Past this many,
// try_open() simply stops, so the extra archives are silently not consulted
// and the tiles they hold come over the network instead. Each open archive
// costs LOCAL_ROOT_CAP (64 KB) of PSRAM for its root directory, held for as
// long as it is open, so the ceiling is memory rather than anything structural.
#ifndef LOCAL_ARCHIVE_MAX
#define LOCAL_ARCHIVE_MAX 16
#endif

typedef struct {
    pmt_t    pmt;
    File     file;
    char     path[40];
    bool     ok;
} local_src_t;

static local_src_t g_locals[LOCAL_ARCHIVE_MAX];
static int         g_local_n = 0;
static int         g_dir_owner = -1;   // which archive last used the shared dir_buf

// Does this archive claim the tile? Zoom is authoritative; the bbox is a
// cheap reject that avoids a directory walk for a tile the file cannot hold.
// A miss here is not an error - the caller simply tries the next archive.
static bool local_covers(const local_src_t *s, uint8_t z, uint32_t x, uint32_t y) {
    if (!s->ok) return false;
    if (z < s->pmt.hdr.min_zoom || z > s->pmt.hdr.max_zoom) return false;

    // Tile bounds in e7 degrees, compared against the archive's own bbox.
    // Only longitude is checked directly; latitude needs the inverse Mercator
    // and the zoom test has already done most of the work, so the extra
    // precision is not worth the transcendentals here.
    uint32_t n = 1u << z;
    int64_t lon0 = (int64_t)(-1800000000LL) + (int64_t)3600000000LL * x / n;
    int64_t lon1 = (int64_t)(-1800000000LL) + (int64_t)3600000000LL * (x + 1) / n;
    if (lon1 < s->pmt.hdr.min_lon_e7 || lon0 > s->pmt.hdr.max_lon_e7) return false;
    (void)y;
    return true;
}
static pmt_t   g_remote;                // current build, over HTTP
static bool    g_remote_ok = false;

// ---- deduplicated blob memo ------------------------------------------------
// PMTiles stores identical tiles once and points every one of them at the same
// (offset, length). Whole oceans at low zoom are a single 93-byte blob, and the
// world floor walk was paying a full HTTPS request - TLS handshake included -
// for each of the thousands of tiles that share it:
//
//   netsource: 6/31/37 at offset 14969040 len 93
//   netsource: 6/31/38 at offset 14969040 len 93
//   netsource: 6/31/39 at offset 14969040 len 93   ... and so on
//
// Two tiles resolving to the same (offset, length) are the same bytes by
// construction, so the second one never needs fetching. Holding just the last
// blob is enough: the walk is raster order, so identical tiles arrive in long
// runs. Sized for the small shared payloads this exists for - a real tile
// misses the memo and takes the normal path.
#define MEMO_CAP 4096
static uint8_t  g_memo[MEMO_CAP];
static uint64_t g_memo_off = 0;
static uint32_t g_memo_len = 0;
static bool     g_memo_valid = false;

static void memo_clear() { g_memo_valid = false; g_memo_off = 0; g_memo_len = 0; }

static void memo_store(uint64_t off, const uint8_t *src, uint32_t len) {
    if (len == 0 || len > MEMO_CAP) { return; }   // too big to be worth holding
    memcpy(g_memo, src, len);
    g_memo_off = off; g_memo_len = len; g_memo_valid = true;
}
static char    g_build[16] = "";
static uint32_t g_adopted_epochdays = 0;
static uint32_t g_today_epochdays = 0;
static NetStats g_stats;
// Serialises access to the pmt_t readers and their scratch buffers.
//
// A pmt_t is explicitly not thread-safe - it owns a directory buffer, a raw
// buffer and a root cache that a lookup writes through. Three tasks reach
// netsource_get: the render worker, the radius prefetch and the world floor.
// Without this they interleave inside a single lookup, which shows up as
// "decompress failed" (another task overwrote raw_buf mid-inflate) and
// "read failed" (the shared file position moved under a read).
//
// Held for a whole tile fetch, so a background task can delay the renderer by
// up to one tile. That is the right trade: the alternative is a second set of
// buffers per task, and there is not enough PSRAM left for it.
static SemaphoreHandle_t g_lock = nullptr;

// Scratch owned by this module; only the render worker calls in here.
static uint8_t *l_dir = nullptr, *l_raw = nullptr;   // shared local scratch
static uint8_t *r_dir = nullptr, *r_raw = nullptr, *r_root = nullptr;
static uint32_t l_cap = 0, r_cap = 0;

// The header describes the root directory but says nothing about leaf sizes,
// and on a planet-scale archive the leaves are what matter: the root loaded
// fine at 15 KB while every leaf lookup failed. Rather than guess again, the
// reader now reports how many bytes it needed, and the buffers grow on
// demand up to a ceiling.
static const uint32_t DIR_CAP_MIN = 256 * 1024;
static const uint32_t DIR_CAP_MAX = 4 * 1024 * 1024;

// Directories are varint runs under gzip, which typically compresses 2-3x, so
// 4x the stored size is comfortable headroom for the decompressed form.
// Larger multipliers cost real PSRAM here: this scratch is allocated twice,
// once for the local archive and once for the remote.
static const uint32_t DIR_EXPAND = 4;

// Root-directory cache per local archive. Held for every open archive at once,
// so it is sized for what a regional extract actually needs rather than for the
// planet - the remote planet's root is about 15 KB. pmt_open reports a larger
// requirement through need_raw if one ever turns up, which is visible in the
// log rather than silent.
static const uint32_t LOCAL_ROOT_CAP = 64 * 1024;

// Grow a reader's scratch to suit the archive it just opened. Returns false
// only if the allocation fails outright.
static bool fit_buffers(pmt_t *p, uint8_t **raw, uint8_t **dir, uint8_t **root,
                        uint32_t *cap, const char *what, uint32_t want)
{
    if (want < DIR_CAP_MIN) want = DIR_CAP_MIN;
    if (want > DIR_CAP_MAX) {
        Serial.printf("netsource: %s directory needs %lu bytes, beyond the %lu cap\n",
                      what, (unsigned long)want, (unsigned long)DIR_CAP_MAX);
        want = DIR_CAP_MAX;
    }
    if (want <= *cap) return true;

    uint32_t dcap = want * DIR_EXPAND;
    if (dcap > DIR_CAP_MAX * DIR_EXPAND) dcap = DIR_CAP_MAX * DIR_EXPAND;

    free(*raw); free(*dir); free(*root);
    *raw  = (uint8_t *)ps_malloc(want);
    *dir  = (uint8_t *)ps_malloc(dcap);
    *root = (uint8_t *)ps_malloc(dcap);
    if (!*raw || !*dir || !*root) {
        Serial.printf("netsource: %s buffer alloc failed (%lu / %lu)\n",
                      what, (unsigned long)want, (unsigned long)dcap);
        return false;
    }
    *cap = want;
    p->raw_buf = *raw; p->raw_cap = want;
    p->dir_buf = *dir; p->dir_cap = dcap;
    p->root_cache = *root; p->root_cache_cap = dcap;
    p->root_cache_len = 0;
    // dir_buf was just freed and replaced, so whatever it was recorded as
    // holding is gone with it. Without this a stale identity would match and
    // hand back the contents of a freed allocation.
    p->dir_len = 0;

    Serial.printf("netsource: %s scratch -> raw %lu KB, dir %lu KB\n",
                  what, (unsigned long)(want / 1024), (unsigned long)(dcap / 1024));
    return true;
}

// ---- date helpers ----------------------------------------------------------
// Days since an arbitrary epoch. Only differences matter, so the zero point
// is irrelevant as long as it is consistent.
static uint32_t epoch_days(int y, int m, int d) {
    if (m <= 2) { y -= 1; m += 12; }
    long era = (y >= 0 ? y : y - 399) / 400;
    long yoe = y - era * 400;
    long doy = (153 * (m - 3) + 2) / 5 + d - 1;
    long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (uint32_t)(era * 146097 + doe + 719468);
}

static void days_to_ymd(uint32_t z, int *y, int *m, int *d) {
    long zz = (long)z - 719468;
    long era = (zz >= 0 ? zz : zz - 146096) / 146097;
    long doe = zz - era * 146097;
    long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long yy = yoe + era * 400;
    long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    long mp = (5 * doy + 2) / 153;
    *d = (int)(doy - (153 * mp + 2) / 5 + 1);
    *m = (int)(mp < 10 ? mp + 3 : mp - 9);
    *y = (int)(yy + (*m <= 2));
}

// Take the date from the system clock - but only once SNTP has actually
// synchronised.
//
// A plausibility check on the value is not enough. M5Unified seeds system
// time from the on-board RTC at startup, and an RTC that has never been set
// can hold an arbitrary date: a battery-backed chip reading years into the
// future looks entirely valid to any range test, and would send build
// discovery probing for archives that will not exist for years.
//
// Asking the SNTP client whether it has completed a sync is the only
// trustworthy signal. The year bound below is a backstop against a malicious
// or broken time server, not the primary check.
bool netsource_set_date_from_clock() {
    if (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED) return false;

    time_t now = time(nullptr);
    struct tm t;
    gmtime_r(&now, &t);
    int year = t.tm_year + 1900;
    if (year < 2026 || year > 2036) {
        Serial.printf("netsource: SNTP returned an implausible year (%d), ignoring\n",
                      year);
        return false;
    }
    g_today_epochdays = epoch_days(year, t.tm_mon + 1, t.tm_mday);
    Serial.printf("netsource: date %04d-%02d-%02d from SNTP\n",
                  year, t.tm_mon + 1, t.tm_mday);
    return true;
}

void netsource_set_date(const char *ddmmyy) {
    if (!ddmmyy || strlen(ddmmyy) < 6) return;
    int d = (ddmmyy[0]-'0')*10 + (ddmmyy[1]-'0');
    int m = (ddmmyy[2]-'0')*10 + (ddmmyy[3]-'0');
    int y = 2000 + (ddmmyy[4]-'0')*10 + (ddmmyy[5]-'0');
    if (d < 1 || d > 31 || m < 1 || m > 12) return;
    g_today_epochdays = epoch_days(y, m, d);
}

// ---- HTTP range read -------------------------------------------------------
static String remote_url() { return String(BUILD_HOST) + g_build + ".pmtiles"; }

// ---- pooled HTTPS connection ----------------------------------------------
// Every fetch used to open a fresh TLS connection. The handshake dominated the
// cost of a small tile and was the peak internal-heap moment in the program,
// so keeping the socket between requests is the single biggest win available
// here.
//
// The reason it was originally disabled is real and has to be handled rather
// than assumed away: a response whose body is not fully drained leaves those
// bytes in the socket, and the next request reads them as its own reply. That
// surfaces as a tile which fetches "successfully" and then fails to inflate -
// a corruption bug, not a connection bug, and a miserable one to trace back
// here. So the rule below is strict: the connection survives only a request
// that completed exactly as expected. Anything else - wrong status, wrong
// length, short read, a single unread byte - tears it down. Reconnecting
// costs a handshake; guessing costs correctness.
//
// The gap keeps the request rate deliberately low. The handshake used to
// provide that spacing as a side effect, and removing it without replacing it
// would turn a polite client into a hostile one against a bucket we are asked
// not to hotlink.
#ifndef NET_REQUEST_GAP_MS
#define NET_REQUEST_GAP_MS 150
#endif

// Drop a pooled socket that has been idle this long rather than discover it
// closed. Servers expire keep-alives on their own schedule.
#ifndef NET_KEEPALIVE_IDLE_MS
#define NET_KEEPALIVE_IDLE_MS 10000
#endif

// Smallest contiguous DMA-capable block worth attempting a TLS handshake with.
//
// The handshake needs one for the AES engine, and that allocation failing is
// not a graceful error - it surfaces as `esp-aes: Failed to allocate memory
// for len descriptor` and takes the fetch down with it. A prefetch burst runs
// the largest free DMA block down to around 43 KB, which is *below* the level
// at which handshakes were failing back when every fetch made one. Pooling
// keeps that safe by handshaking once at the start of a burst, while memory is
// still plentiful - but a keep-alive dropped by the far end mid-burst would
// reconnect at exactly the low-water mark. Skipping one tile is much cheaper
// than a failed handshake.
#ifndef NET_HANDSHAKE_MIN_DMA
#define NET_HANDSHAKE_MIN_DMA (32 * 1024)
#endif

static HTTPClient g_http;
static bool       g_http_live = false;   // g_http holds a begun request/socket
static uint32_t   g_http_last = 0;       // millis at the end of the last request
static uint32_t   g_http_reused = 0, g_http_fresh = 0;

// Tear the pooled connection down for good.
static void http_pool_drop() {
    if (!g_http_live) return;
    g_http.setReuse(false);              // make end() actually close it
    g_http.end();
    g_http_live = false;
}

// One attempt. *pooled reports whether this went out on an already-open
// socket, which decides whether a failure is worth retrying.
static int http_range_once(const String &url, uint64_t off, uint32_t len,
                           uint8_t *dst, bool *pooled)
{
    *pooled = g_http_live;

    if (!g_http_live) {
        // About to handshake. Check there is room for it first.
        size_t dma = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        if (dma < NET_HANDSHAKE_MIN_DMA) {
            delay(250);                  // a transient dip is common mid-burst
            dma = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        }
        if (dma < NET_HANDSHAKE_MIN_DMA) {
            Serial.printf("netsource: deferring handshake, largest DMA block "
                          "%u KB\n", (unsigned)(dma / 1024));
            return -1;
        }
        g_http.setReuse(true);
        g_http.setTimeout(15000);
        if (!g_http.begin(url)) { g_http_live = false; return -1; }
        g_http_live = true;
        g_http_fresh++;
    } else {
        // Same host and scheme every time, so begin() re-arms the request on
        // the socket that is already open.
        if (!g_http.begin(url)) { http_pool_drop(); return -1; }
        g_http_reused++;
    }

    char range[64];
    snprintf(range, sizeof range, "bytes=%llu-%llu",
             (unsigned long long)off, (unsigned long long)(off + len - 1));
    g_http.addHeader("Range", range);

    uint64_t t0 = esp_timer_get_time();
    int code = g_http.GET();
    // 206 is the expected answer. A 200 means the server ignored the Range
    // header and is about to send the entire multi-gigabyte archive, which
    // must be refused rather than read.
    if (code != HTTP_CODE_PARTIAL_CONTENT) {
        if (code == HTTP_CODE_OK)
            Serial.println("netsource: server ignored Range header, refusing");
        http_pool_drop();                // body unread - socket is unusable
        return -1;
    }

    // A 206 must carry exactly the requested range; anything else means the
    // response does not correspond to this request.
    int clen = g_http.getSize();
    if (clen >= 0 && (uint32_t)clen != len) {
        Serial.printf("netsource: range asked %lu got %d bytes, discarding\n",
                      (unsigned long)len, clen);
        http_pool_drop();
        return -1;
    }

    WiFiClient *s = g_http.getStreamPtr();
    uint32_t got = 0;
    // Scale the budget with the request size instead of using a flat 15 s.
    // A 129 KB leaf directory was arriving at under 3 KB/s and getting cut
    // off at a third of the way through, every time, for every tile in its
    // range - which stalled the world floor permanently at 65%.
    uint32_t deadline = millis() + 10000 + len / 4;   // ~4 KB/s floor
    uint32_t reads = 0;
    while (got < len && millis() < deadline) {
        reads++;
        size_t avail = s->available();
        if (!avail) { delay(2); continue; }
        int n = s->readBytes(dst + got, min((size_t)(len - got), avail));
        if (n <= 0) break;
        got += n;
    }

    uint32_t leftover = 0;
    while (s->available()) { s->read(); leftover++; }

    g_stats.last_fetch_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    g_http_last = millis();

    if (got != len || leftover) {
        Serial.printf("netsource: range %llu+%lu -> got %lu in %lu reads, "
                      "%lu left over\n", (unsigned long long)off,
                      (unsigned long)len, (unsigned long)got,
                      (unsigned long)reads, (unsigned long)leftover);
        // Either the body was short or there was more of it than the headers
        // promised. In both cases the stream framing is no longer trustworthy,
        // so the socket does not survive.
        http_pool_drop();
        return -1;
    }

    g_http.end();                        // reuse is on, so this keeps the socket
    g_stats.bytes_fetched += got;

    // Periodic proof that pooling is actually happening. If `fresh` climbs in
    // step with `reused`, the socket is being torn down every request and the
    // handshake has not gone anywhere.
    if (((g_http_reused + g_http_fresh) % 25) == 0)
        Serial.printf("netsource: conn %lu reused, %lu fresh, last fetch %lu ms\n",
                      (unsigned long)g_http_reused, (unsigned long)g_http_fresh,
                      (unsigned long)g_stats.last_fetch_ms);
    return 0;
}

static int http_range(const String &url, uint64_t off, uint32_t len, uint8_t *dst) {
    if (WiFi.status() != WL_CONNECTED) { http_pool_drop(); return -1; }

    // Retire a socket the server has probably given up on already.
    if (g_http_live && (uint32_t)(millis() - g_http_last) > NET_KEEPALIVE_IDLE_MS)
        http_pool_drop();

    // Space requests out, measured from the end of the previous one.
    uint32_t since = millis() - g_http_last;
    if (g_http_last && since < NET_REQUEST_GAP_MS) delay(NET_REQUEST_GAP_MS - since);

    bool pooled = false;
    int r = http_range_once(url, off, len, dst, &pooled);
    if (r == 0 || !pooled) return r;

    // A pooled socket failing is expected occasionally: keep-alives get closed
    // from the far end with no warning, and the failure looks identical to a
    // real error. Retrying once on a fresh connection separates the two, and
    // without it every server-side timeout would surface as a lost tile.
    http_pool_drop();
    Serial.println("netsource: pooled connection failed, retrying fresh");
    return http_range_once(url, off, len, dst, &pooled);
}

// Large ranges are split into several requests.
//
// Small reads come back promptly; the 129 KB one did not, consistently, across
// separate boots and different tiles. Whatever the cause upstream - CDN
// behaviour on big ranges, or a window that never opens - a request the size
// of the ones that already work is the reliable shape. The extra round trips
// only apply to directory-sized reads, which the leaf cache below now makes
// rare.
static const uint32_t RANGE_CHUNK = 32 * 1024;

static int net_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
    (void)ctx;
    if (len <= RANGE_CHUNK) return http_range(remote_url(), off, len, dst);

    String url = remote_url();      // build once for the whole split read
    uint32_t done = 0;
    while (done < len) {
        uint32_t n = len - done;
        if (n > RANGE_CHUNK) n = RANGE_CHUNK;
        if (http_range(url, off + done, n, dst + done) != 0) {
            Serial.printf("netsource: split read failed at %lu/%lu bytes\n",
                          (unsigned long)done, (unsigned long)len);
            return -1;
        }
        done += n;
    }
    return 0;
}

// ---- SD-backed pmtiles read ------------------------------------------------

// Reads come from whichever archive asked, passed through io_ctx - there is no
// longer a single local file to reach for.
static int sd_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
    local_src_t *src = (local_src_t *)ctx;
    if (!src || !src->file) return -1;
    if (!src->file.seek((uint32_t)off)) return -1;
    // Chunked: a pmtiles blob or leaf directory can be tens of KB, and a
    // single read that large is what the USB driver cannot allocate a
    // transfer buffer for. See STORAGE_IO_CHUNK.
    return storage_read(src->file, dst, len) == len ? 0 : -1;
}

static int gz_inflate(void *ctx, uint8_t codec, const uint8_t *src,
                      uint32_t src_len, uint8_t *dst, uint32_t *dst_len) {
    (void)ctx;
    if (codec != PMT_COMPRESS_GZIP) return -1;
    return inflate_auto(src, src_len, dst, dst_len) == INF_OK ? 0 : -1;
}

// ---- cache -----------------------------------------------------------------
// Backed by tilecache: one append-only blob plus an in-PSRAM index, rather
// than a file per tile. See tilecache.h for why - briefly, thousands of small
// creates is the pattern most likely to lose a FAT32 filesystem to a power
// cut, and wastes most of each 32 KB cluster besides.

static bool cache_read(uint8_t z, uint32_t x, uint32_t y,
                       uint8_t *dst, uint32_t *len) {
    return tilecache_get(z, x, y, dst, len);
}

static void cache_write(uint8_t z, uint32_t x, uint32_t y,
                        const uint8_t *src, uint32_t len) {
    tilecache_put(z, x, y, src, len);
}

// Zero length records "the archive genuinely has no tile here", so ocean and
// out-of-coverage areas are not re-requested on every pass.
static void cache_write_empty(uint8_t z, uint32_t x, uint32_t y) {
    tilecache_put(z, x, y, nullptr, 0);
}

static bool cache_is_empty_marker(uint8_t z, uint32_t x, uint32_t y) {
    uint32_t n = 0;
    uint8_t dummy;
    // A hit with zero length is the marker; a hit with data returns false
    // here because n would exceed the zero capacity passed in.
    return tilecache_get(z, x, y, &dummy, &n) && n == 0;
}

// ---- build discovery -------------------------------------------------------
static bool probe_build(const char *date) {
    String url = String(BUILD_HOST) + date + ".pmtiles";
    uint8_t hdr[16];
    HTTPClient http;
    http.setTimeout(8000);
    if (!http.begin(url)) return false;
    http.addHeader("Range", "bytes=0-15");
    int code = http.GET();
    bool ok = false;
    if (code == HTTP_CODE_PARTIAL_CONTENT) {
        WiFiClient *s = http.getStreamPtr();
        uint32_t t = millis();
        int got = 0;
        while (got < 16 && millis() - t < 5000) {
            if (s->available()) got += s->readBytes(hdr + got, 16 - got);
            else delay(2);
        }
        ok = (got == 16 && memcmp(hdr, "PMTiles", 7) == 0 && hdr[7] == 3);
    }
    http.end();
    return ok;
}

static bool discover_build(char *out, size_t n) {
    if (!g_today_epochdays) netsource_set_date_from_clock();
    if (!g_today_epochdays) {
        // Once, not once per tile.
        //
        // maybe_refresh() is called on every tile that misses the cache, so
        // before the clock is set this pair of lines is the entire console -
        // hundreds of repetitions that say nothing the first one did not, and
        // bury everything that does. The state is latched rather than rate
        // limited by time: it only changes when a clock arrives, and when it
        // does the next line will say so.
        static bool said = false;
        if (!said) {
            said = true;
            Serial.println("netsource: no date yet (needs SNTP or a GNSS fix) "
                           "- offline until one arrives, local archives still work");
        }
        return false;
    }
    for (int back = 0; back < MAX_PROBE_DAYS; back++) {
        int y, m, d;
        days_to_ymd(g_today_epochdays - back, &y, &m, &d);

        // Reject an implausible conversion rather than format it.
        //
        // g_today_epochdays is a uint32_t and days_to_ymd does signed
        // arithmetic on it, so a corrupt value yields a year of several
        // million - which would overrun `date` and get silently truncated
        // into a probe for an archive that cannot exist. The stored value is
        // only ever set from SNTP or a GNSS fix, so this should not happen;
        // it fires only if that invariant has already broken.
        //
        // Bailing rather than continuing is deliberate: `back` shifts the day
        // but not the magnitude, so if one iteration is nonsense they all are.
        if (y < 1970 || y > 2100 || m < 1 || m > 12 || d < 1 || d > 31) {
            Serial.printf("netsource: implausible date from epoch-days %lu "
                          "(%d-%d-%d), not probing\n",
                          (unsigned long)g_today_epochdays, y, m, d);
            return false;
        }

        char date[16];
        snprintf(date, sizeof date, "%04d%02d%02d", y, m, d);
        Serial.printf("netsource: probing build %s ... ", date);
        if (probe_build(date)) {
            Serial.println("ok");
            snprintf(out, n, "%s", date);
            return true;
        }
        Serial.println("no");
    }
    return false;
}

static void manifest_save() {
    File f = g_fs->open(MANIFEST, FILE_WRITE);
    if (!f) return;
    f.printf("%s\n%lu\n", g_build, (unsigned long)g_adopted_epochdays);
    f.close();
}

static bool manifest_load() {
    File f = g_fs->open(MANIFEST, FILE_READ);
    if (!f) return false;
    String a = f.readStringUntil('\n'); a.trim();
    String b = f.readStringUntil('\n'); b.trim();
    f.close();
    if (a.length() != 8) return false;
    snprintf(g_build, sizeof g_build, "%s", a.c_str());
    g_adopted_epochdays = (uint32_t)b.toInt();
    return true;
}

// Defined below, after the build-discovery helpers it depends on.
static bool maybe_refresh();

// ---- world bootstrap -------------------------------------------------------
// There is no server-side extract, so the device cannot ask for "just the low
// zooms" as a single download. It does not need to: the whole pyramid from
// z0 to z6 is 5461 tiles, and pulling them through the ordinary cache path
// leaves exactly the offline floor that world.pmtiles was going to provide.
//
// One HTTP round trip per tile makes this slow - several minutes - so it runs
// once, in the background, and reports progress. Everything it fetches is a
// normal cache entry, so an interrupted run simply resumes where it stopped.
bool netsource_prefetch_world(uint8_t maxz, uint8_t *buf, uint32_t cap,
                              void (*progress)(uint32_t done, uint32_t total))
{
    if (!maybe_refresh() || !g_remote_ok) return false;

    uint32_t total = 0;
    for (uint8_t z = 0; z <= maxz; z++) total += 1u << (2 * z);

    uint32_t done = 0, fetched = 0, skipped = 0;
    for (uint8_t z = 0; z <= maxz; z++) {
        uint32_t n = 1u << z;
        for (uint32_t x = 0; x < n; x++) {
            for (uint32_t y = 0; y < n; y++) {
                done++;
                // Already cached, including negative markers, so a resumed
                // run costs an SD stat rather than a round trip.
                uint32_t got = cap;
                if (cache_is_empty_marker(z, x, y) ||
                    cache_read(z, x, y, buf, &got)) { skipped++; }
                else {
                    got = cap;
                    bool net = false;
                    if (netsource_get(z, x, y, buf, &got, &net)) fetched++;
                }
                if (progress && (done % 64) == 0) progress(done, total);
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("netsource: link lost, prefetch paused");
                    return false;
                }
            }
        }
        Serial.printf("netsource: z%u done (%lu fetched, %lu already cached)\n",
                      z, (unsigned long)fetched, (unsigned long)skipped);
    }
    if (progress) progress(total, total);
    return true;
}

// ---- public ----------------------------------------------------------------
bool netsource_begin(const char *local_path) {
    g_fs = storage_fs();
    g_lock = xSemaphoreCreateMutex();

    l_raw  = (uint8_t *)ps_malloc(DIR_CAP_MIN);
    l_dir  = (uint8_t *)ps_malloc(DIR_CAP_MIN * DIR_EXPAND);
    r_raw  = (uint8_t *)ps_malloc(DIR_CAP_MIN);
    r_dir  = (uint8_t *)ps_malloc(DIR_CAP_MIN * DIR_EXPAND);
    r_root = (uint8_t *)ps_malloc(DIR_CAP_MIN * DIR_EXPAND);
    if (!l_dir || !l_raw || !r_dir || !r_raw || !r_root) return false;
    l_cap = r_cap = DIR_CAP_MIN;

    // Local archives: every .pmtiles found, not a fixed list of names.
    //
    // Splitting by zoom is not enough once a level is bigger than FAT32 allows
    // a single file to be - z14 is 32 GB globally, so covering the world means
    // ten or more files, and naming them individually does not scale. Each one
    // declares its own zoom range and bounding box in its header, so the set
    // can just be whatever is on the card.
    //
    // The configured path is opened first so an explicit choice still wins,
    // then everything else in alphabetical order. Ordering only affects which
    // archive answers when two cover the same tile, and by then either would
    // do.
    auto try_open = [&](const char *path) -> bool {
        if (g_local_n >= LOCAL_ARCHIVE_MAX) return false;
        for (int i = 0; i < g_local_n; i++)          // already opened
            if (strcmp(g_locals[i].path, path) == 0) return false;

        local_src_t *src = &g_locals[g_local_n];
        memset(&src->pmt, 0, sizeof src->pmt);
        src->ok = false;
        snprintf(src->path, sizeof src->path, "%s", path);

        src->file = g_fs->open(path, FILE_READ);
        if (!src->file) return false;

        uint8_t *root = (uint8_t *)ps_malloc(LOCAL_ROOT_CAP);
        if (!root) { src->file.close(); return false; }

        src->pmt.read = sd_read;
        src->pmt.io_ctx = src;
        src->pmt.inflate = gz_inflate;
        src->pmt.dir_buf = l_dir;  src->pmt.dir_cap = DIR_CAP_MIN * DIR_EXPAND;
        src->pmt.raw_buf = l_raw;  src->pmt.raw_cap = DIR_CAP_MIN;
        src->pmt.root_cache = root; src->pmt.root_cache_cap = LOCAL_ROOT_CAP;

        if (pmt_open(&src->pmt) != PMT_OK) {
            Serial.printf("netsource: local %s FAILED to open\n", path);
            free(root); src->file.close();
            return false;
        }

        // Does the file contain what its header claims?
        //
        // A multi-GB archive copied to a card realistically fails by being
        // truncated - a full card, a yanked reader, a copy that reported
        // success it had not earned. The header still parses, so the archive
        // opens, and the damage only shows up later as tiles that fail to
        // inflate somewhere out on a drive. The header says where the data
        // ends, so this costs one size() call and no reading.
        uint64_t need = src->pmt.hdr.data_off + src->pmt.hdr.data_len;
        uint64_t have = (uint64_t)src->file.size();
        if (have < need) {
            Serial.printf("netsource: local %s TRUNCATED - header needs %llu B, "
                          "file is %llu B (%.1f%%). Ignoring it.\n",
                          path, (unsigned long long)need,
                          (unsigned long long)have, 100.0 * have / (double)need);
            free(root); src->file.close();
            return false;
        }

        src->ok = true;
        Serial.printf("netsource: local %s z%u..%u  lon %.2f..%.2f  %.1f MB\n",
                      path, src->pmt.hdr.min_zoom, src->pmt.hdr.max_zoom,
                      src->pmt.hdr.min_lon_e7 / 1e7, src->pmt.hdr.max_lon_e7 / 1e7,
                      have / 1048576.0);
        g_local_n++;
        return true;
    };

    if (local_path && *local_path && g_fs->exists(local_path)) try_open(local_path);

    // Everything else at the root. Subdirectories are not searched: a flat
    // convention is easier to explain than a hierarchy nobody can remember.
    {
        File dir = g_fs->open("/");
        if (dir) {
            for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
                if (e.isDirectory()) { e.close(); continue; }
                const char *n = e.name();
                size_t ln = n ? strlen(n) : 0;
                bool is_pmt = ln > 8 && strcmp(n + ln - 8, ".pmtiles") == 0;
                char full[48];
                if (is_pmt) snprintf(full, sizeof full, "%s%s", n[0] == '/' ? "" : "/", n);
                e.close();
                if (is_pmt) try_open(full);
                if (g_local_n >= LOCAL_ARCHIVE_MAX) break;
            }
            dir.close();
        }
    }

    if (g_local_n == 0)
        Serial.println("netsource: no local archives (no *.pmtiles at the card root)");
    else
    {
        Serial.printf("netsource: %d local archive(s) open", g_local_n);
        if (g_local_n >= LOCAL_ARCHIVE_MAX)
            Serial.printf(" - at the LOCAL_ARCHIVE_MAX limit, any further "
                          "*.pmtiles were skipped");
        Serial.println();
    }

    manifest_load();
    if (g_build[0]) {
        Serial.printf("netsource: cached build %s\n", g_build);
        tilecache_open(g_build, CACHE_MAX_ENTRIES);

        // A blob whose records account for almost none of its size was
        // damaged - by concurrent writes, or an interrupted one. Records past
        // the damage are unreachable and their space is never reclaimed, so
        // keeping it means a cache that only grows while holding nothing.
        CacheStats cs;
        tilecache_stats(&cs);
        if (cs.blob_bytes > 1024 * 1024 && cs.entries < 32) {
            Serial.printf("netsource: cache holds %lu entries in %lu KB - "
                          "discarding it\n", (unsigned long)cs.entries,
                          (unsigned long)(cs.blob_bytes / 1024));
            tilecache_wipe();
        }
    }
    return true;
}

static bool open_remote() {
    if (!g_build[0] || WiFi.status() != WL_CONNECTED) return false;
    memset(&g_remote, 0, sizeof g_remote);
    g_remote.read = net_read; g_remote.inflate = gz_inflate;
    g_remote.dir_buf = r_dir; g_remote.dir_cap = r_cap * DIR_EXPAND;
    g_remote.raw_buf = r_raw; g_remote.raw_cap = r_cap;
    g_remote.root_cache = r_root; g_remote.root_cache_cap = r_cap * DIR_EXPAND;
    memo_clear();   // offsets are only meaningful within one archive
    g_remote_ok = (pmt_open(&g_remote) == PMT_OK);
    if (g_remote_ok) {
        Serial.printf("netsource: remote build %s open, z%u..%u, root %lu B\n",
                      g_build, g_remote.hdr.min_zoom, g_remote.hdr.max_zoom,
                      (unsigned long)g_remote.hdr.root_len);
        g_remote_ok = fit_buffers(&g_remote, &r_raw, &r_dir, &r_root, &r_cap,
                                  "remote", (uint32_t)g_remote.hdr.root_len);
    }
    return g_remote_ok;
}

// The world floor is what stops a drive out of cached territory ending in a
// blank screen: z0-6 is only 5461 tiles but covers the entire planet, so
// there is always something to fall back to.
static const char *WORLD_MARK = "/t/world.ok";
static const char *WORLD_POS  = "/t/world.pos";

bool netsource_world_ready() {
    File f = g_fs->open(WORLD_MARK, FILE_READ);
    if (!f) return false;
    f.close();
    return true;
}

void netsource_world_mark_done() {
    File f = g_fs->open(WORLD_MARK, FILE_WRITE);
    if (f) { f.printf("%s\n", g_build); f.close(); }
    netsource_world_clear_pos();
}

void netsource_world_save_pos(uint8_t z, uint32_t x, uint32_t y) {
    File f = g_fs->open(WORLD_POS, FILE_WRITE);
    if (!f) return;
    f.printf("%s %u %lu %lu\n", g_build, z,
             (unsigned long)x, (unsigned long)y);
    f.close();
}

bool netsource_world_load_pos(uint8_t *z, uint32_t *x, uint32_t *y) {
    File f = g_fs->open(WORLD_POS, FILE_READ);
    if (!f) return false;
    char line[64] = {0};
    int n = f.read((uint8_t *)line, sizeof line - 1);
    f.close();
    if (n <= 0) return false;

    char build[16] = {0};
    unsigned zz = 0, xx = 0, yy = 0;
    if (sscanf(line, "%15s %u %u %u", build, &zz, &xx, &yy) != 4) return false;
    // A checkpoint from a different build points into a cache that no longer
    // exists, so it has to be discarded rather than trusted.
    if (strcmp(build, g_build) != 0) {
        Serial.printf("netsource: world checkpoint is from build %s, ignoring\n",
                      build);
        return false;
    }
    *z = (uint8_t)zz; *x = xx; *y = yy;
    return true;
}

void netsource_world_clear_pos() {
    if (g_fs) g_fs->remove(WORLD_POS);
}

bool netsource_refresh() {
    if (WiFi.status() != WL_CONNECTED) return false;

    // A pinned build never ages out, so there is nothing to discover.
    if (PINNED_BUILD[0]) {
        if (strcmp(g_build, PINNED_BUILD) != 0) {
            snprintf(g_build, sizeof g_build, "%s", PINNED_BUILD);
            manifest_save();
        }
        g_adopted_epochdays = g_today_epochdays;
        return open_remote();
    }

    char found[16] = "";
    if (!discover_build(found, sizeof found)) {
        // Same reasoning as the line above, and the same call frequency. This
        // one is latched on the day: a new day is a new set of builds to
        // probe, so it is worth saying again then and not before.
        static uint32_t said_for_day = 0;
        if (said_for_day != g_today_epochdays) {
            said_for_day = g_today_epochdays ? g_today_epochdays : 1;
            Serial.println("netsource: no live build found in the last 8 days");
        }
        return false;
    }
    if (strcmp(found, g_build) != 0) {
        if (g_build[0]) {
            // Offsets from one build mean nothing in another, so the whole
            // cache goes. With a blob that is two file deletes rather than a
            // recursive walk of thousands of entries.
            char p[64];
            Serial.printf("netsource: build %s -> %s, dropping old cache\n",
                          g_build, found);
            tilecache_close();
            snprintf(p, sizeof p, "/t/%s.dat", g_build); g_fs->remove(p);
            snprintf(p, sizeof p, "/t/%s.idx", g_build); g_fs->remove(p);
        }
        snprintf(g_build, sizeof g_build, "%s", found);
    }
    tilecache_open(g_build, CACHE_MAX_ENTRIES);
    g_adopted_epochdays = g_today_epochdays;
    manifest_save();
    return open_remote();
}

static bool maybe_refresh() {
    if (g_remote_ok) {
        // A pinned build is deliberately frozen; only the daily channel ages.
        if (PINNED_BUILD[0]) return true;
        // Age out on schedule even while working, so a device left running
        // does not quietly serve month-old geometry forever.
        if (g_today_epochdays && g_adopted_epochdays &&
            g_today_epochdays - g_adopted_epochdays >= REFRESH_DAYS) {
            Serial.println("netsource: adopted build has aged out, refreshing");
            g_remote_ok = false;
            return netsource_refresh();
        }
        return true;
    }
    if (WiFi.status() != WL_CONNECTED) return false;
    if (g_build[0] && open_remote()) return true;
    return netsource_refresh();
}

// Try every local archive that claims this tile.
//
// Split out of netsource_get_locked() so it can run before the network rather
// than after it. See NET_PREFER_LOCAL.
static bool local_try(uint8_t z, uint32_t x, uint32_t y,
                      uint8_t *dst, uint32_t cap, uint32_t *len) {
    for (int i = 0; i < g_local_n; i++) {
        local_src_t *src = &g_locals[i];
        if (!local_covers(src, z, x, y)) continue;

        // The archives share one dir_buf, and pmtiles.c reuses whatever that
        // buffer already holds to avoid re-reading a leaf directory. That
        // shortcut is keyed on an offset and length, which are only meaningful
        // within one archive - so switching archives must retire the previous
        // owner's claim, or the next lookup could match an offset from a
        // different file and read another archive's directory as its own.
        if (i != g_dir_owner) {
            if (g_dir_owner >= 0 && g_dir_owner < g_local_n)
                g_locals[g_dir_owner].pmt.dir_len = 0;
            g_dir_owner = i;
        }

        uint32_t n = cap;
        if (pmt_get(&src->pmt, z, x, y, dst, &n) == PMT_OK) {
            *len = n;
            g_stats.local_hits++;
            return true;
        }
    }
    return false;
}

static bool netsource_get_locked(uint8_t z, uint32_t x, uint32_t y,
                                 uint8_t *dst, uint32_t *len, bool *from_net)
{
    if (from_net) *from_net = false;
    uint32_t cap = *len;

    // 1. cache
    bool empty_marked = false;
    if (g_build[0]) {
        // The empty marker used to return a miss outright. That was right when
        // local archives came last - anything the remote did not have, nothing
        // else was going to either.
        //
        // With local first it becomes a trap. The marker records that *the
        // remote build* had no tile there, which is a different claim from "no
        // tile exists": drop a band extract on the drive covering ground the
        // device once probed and cached as empty, and every one of those tiles
        // would stay empty for as long as the cache lived, with the data
        // sitting in a file it never opened.
        //
        // So note it and keep going. It still suppresses the network request,
        // which is what it is for.
        empty_marked = cache_is_empty_marker(z, x, y);
        if (!empty_marked) {
            uint32_t n = cap;
            if (cache_read(z, x, y, dst, &n)) { *len = n; g_stats.cache_hits++; return true; }
        }
    }

    // 2. local archives, before the network.
    //
    // The order used to be the other way round: network first, local as the
    // floor for whatever the network could not supply. That made sense when
    // the only local file was a small low-zoom world archive - a fallback for
    // being offline, not a source in its own right.
    //
    // It is wrong once the working zoom is on the drive. Downloading a tile
    // that is sitting in a file two feet away costs a TLS handshake and a
    // range request per tile, on a link that may not exist, to produce bytes
    // already present - and it does it for every tile of every pan.
    //
    // The zoom test in local_covers() is what makes this safe to do
    // unconditionally. A z14-only band extract claims nothing at z12, so the
    // coarse overview and the world floor still reach the network exactly as
    // before, and a tile no archive claims falls straight through.
    //
    // Set NET_PREFER_LOCAL to 0 for the old order.
#ifndef NET_PREFER_LOCAL
#define NET_PREFER_LOCAL 1
#endif
#if NET_PREFER_LOCAL
    if (local_try(z, x, y, dst, cap, len)) return true;
#endif

    if (empty_marked) { g_stats.misses++; return false; }

    // 3. network
    if (maybe_refresh() && g_remote_ok) {
        g_stats.online = true;

        // Resolve the tile's location once. pmt_get would repeat this walk
        // internally, and for a working-zoom tile that walk pulls a leaf
        // directory over HTTP - so the old debug-only find was quietly
        // doubling the request count on exactly the tiles the user is
        // waiting for.
        uint64_t off = 0; uint32_t blob_len = 0;
        pmt_err_t fe = pmt_find(&g_remote, z, x, y, &off, &blob_len);

        // A leaf directory larger than the current scratch is not an error,
        // just a buffer sized before we knew what the archive contained. The
        // reader reports what it needed; grow once and retry. This belongs on
        // the lookup, because the lookup is what loads leaf directories.
        if (fe == PMT_ENOMEM && g_remote.need_raw > r_cap &&
            g_remote.need_raw <= DIR_CAP_MAX) {
            Serial.printf("netsource: leaf needs %lu B, growing from %lu KB\n",
                          (unsigned long)g_remote.need_raw,
                          (unsigned long)(r_cap / 1024));
            if (fit_buffers(&g_remote, &r_raw, &r_dir, &r_root, &r_cap,
                            "remote", g_remote.need_raw))
                fe = pmt_find(&g_remote, z, x, y, &off, &blob_len);
        }

        if (fe == PMT_OK) {
            static uint32_t shown = 0;
            if (shown < 12) {
                shown++;
                Serial.printf("netsource: %u/%lu/%lu at offset %llu len %lu%s\n",
                              z, (unsigned long)x, (unsigned long)y,
                              (unsigned long long)off, (unsigned long)blob_len,
                              (g_memo_valid && off == g_memo_off &&
                               blob_len == g_memo_len) ? " (memo)" : "");
            }

            // Same blob as last time: it is the same bytes, by construction.
            if (g_memo_valid && off == g_memo_off && blob_len == g_memo_len &&
                blob_len <= cap) {
                memcpy(dst, g_memo, blob_len);
                cache_write(z, x, y, dst, blob_len);
                *len = blob_len;
                g_stats.cache_hits++;
                return true;
            }
        }

        // Read the payload from the location already resolved above, rather
        // than pmt_get, which would walk the directory a second time - and
        // over the network that walk is another leaf fetch and another TLS
        // handshake for a tile we have already located.
        uint32_t n = cap;
        pmt_err_t e = (fe == PMT_OK)
                    ? pmt_read_blob(&g_remote, off, blob_len, dst, &n)
                    : fe;

        if (e == PMT_OK) {
            cache_write(z, x, y, dst, n);
            // Remember small payloads so the next tile sharing this blob
            // costs nothing.
            if (fe == PMT_OK && n == blob_len) memo_store(off, dst, n);
            *len = n;
            g_stats.net_hits++;
            if (from_net) *from_net = true;
            return true;
        }
        if (e == PMT_NOTFOUND) {
            cache_write_empty(z, x, y);
            g_stats.misses++;
            return false;
        }
        g_stats.errors++;
        {
            static uint32_t logged = 0;
            if (logged < 8) {
                logged++;
                Serial.printf("netsource: remote %u/%lu/%lu failed: %s",
                              z, (unsigned long)x, (unsigned long)y,
                              pmt_strerror(e));
                if (e == PMT_ENOMEM)
                    Serial.printf(" (needed %lu B)",
                                  (unsigned long)g_remote.need_raw);
                Serial.println();
            }
        }
        // Fall through to a miss rather than failing outright. There is
        // nothing left to try: the local archives were asked first.
    } else {
        if (g_stats.online) Serial.println("netsource: gone offline");
        g_stats.online = false;
    }

#if !NET_PREFER_LOCAL
    if (local_try(z, x, y, dst, cap, len)) return true;
#endif

    g_stats.misses++;
    return false;
}

bool netsource_get(uint8_t z, uint32_t x, uint32_t y,
                   uint8_t *dst, uint32_t *len, bool *from_net)
{
    if (!g_lock) return netsource_get_locked(z, x, y, dst, len, from_net);
    if (xSemaphoreTake(g_lock, portMAX_DELAY) != pdTRUE) return false;
    bool r = netsource_get_locked(z, x, y, dst, len, from_net);
    xSemaphoreGive(g_lock);
    return r;
}

void netsource_stats(NetStats *out) {
    *out = g_stats;
    snprintf(out->build, sizeof out->build, "%s", g_build);
}
