// bigfile.cpp - see bigfile.h for why the VFS cannot do this.

#include "bigfile.h"
#include "storage.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

#if MAP_HAVE_BIGFILE
#include "ff.h"

// FF_FS_EXFAT is what makes FSIZE_t 64-bit. Without it every offset below
// silently truncates, which is the exact failure this file exists to remove,
// so the 64-bit path is withdrawn rather than left looking available.
//
// Withdrawn, not fatal: a stock ESP-IDF hardcodes FF_FS_EXFAT to 0, and an
// #error here would mean nobody who has not vendored the component can build
// the IDF target at all - which is a poor trade for a feature that degrades
// perfectly well. The stubs at the bottom of this file take over, callers see
// bigfile_supported() == false, and the boot log says so.
#if !FF_FS_EXFAT
#warning "MAP_HAVE_BIGFILE is set but fatfs has FF_FS_EXFAT=0 - 64-bit reads disabled, see bigfile.h"
#undef  MAP_HAVE_BIGFILE
#define MAP_HAVE_BIGFILE 0
#endif
#endif

#if MAP_HAVE_BIGFILE

// FatFs volumes are numbered in registration order by ff_diskio_register(),
// which esp_vfs_fat_*_mount() calls. Two media at most here, but probing the
// full set costs four failed f_open() calls once at boot.
#ifndef FF_VOLUMES
#define FF_VOLUMES 2
#endif

struct bigfile_s {
    FIL      fp;
    uint64_t size;
    char     path[64];
    // Cluster link map, owned here so it outlives the call that built it and
    // is freed with the handle. Null when fast seek could not be set up, in
    // which case fp.cltbl is null too and f_lseek() walks the chain.
    DWORD   *clmt;
};

// Defined below bigfile_open() because it is long and this is where it is
// called from; declared here so the order reads open-then-detail.
static void clmt_build(bigfile_t *b);

bool bigfile_supported() { return sizeof(FSIZE_t) >= 8; }

int bigfile_volumes() { return FF_VOLUMES; }

int bigfile_volume_of(const char *name, uint64_t size) {
    if (!name || !*name) return -1;

    int found = -1;
    for (int drv = 0; drv < FF_VOLUMES; drv++) {
        char ff[80];
        snprintf(ff, sizeof ff, "%d:%s%s", drv, name[0] == '/' ? "" : "/", name);

        FILINFO fi;
        if (f_stat(ff, &fi) != FR_OK) continue;
        if (size && (uint64_t)fi.fsize != size) continue;

        // Two volumes answering for the same name and size is not something to
        // resolve by picking one. It means the caller needs a better probe
        // file, and quietly returning the lower drive number would produce a
        // boot line that is confidently wrong - which is worse than one that
        // is absent.
        if (found >= 0) return -1;
        found = drv;
    }
    return found;
}

bool bigfile_volume_info(int drv, char *out, size_t out_len) {
    if (!out || out_len == 0 || drv < 0 || drv >= FF_VOLUMES) return false;

    char vol[8];
    snprintf(vol, sizeof vol, "%d:", drv);

    // f_getfree is the cheapest call that hands back the FATFS object, which
    // is where fs_type lives. The free-cluster count it computes is a side
    // effect here - on exFAT it reads the allocation bitmap, so it is not
    // free, but it happens once at boot.
    FATFS *fs = nullptr;
    DWORD  free_clst = 0;
    if (f_getfree(vol, &free_clst, &fs) != FR_OK || !fs) return false;

    const char *type;
    switch (fs->fs_type) {
        case FS_FAT12: type = "FAT12"; break;
        case FS_FAT16: type = "FAT16"; break;
        case FS_FAT32: type = "FAT32"; break;
        case FS_EXFAT: type = "exFAT"; break;
        default:       type = "FAT?";  break;
    }

    // n_fatent counts the two reserved entries, so clusters is that minus 2.
    // Sector size is a variable on this build - FF_MAX_SS and FF_MIN_SS differ
    // once exFAT is on, which is what makes fs->ssize exist at all.
#if FF_MAX_SS != FF_MIN_SS
    uint64_t ssize = fs->ssize;
#else
    uint64_t ssize = FF_MAX_SS;
#endif
    uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * ssize;

    // Label and serial, when there is one. A blank label is normal and not
    // worth a line of its own, so it simply drops out of the string.
    char  label[24] = "";
    DWORD vsn = 0;
    bool  have_id = (f_getlabel(vol, label, &vsn) == FR_OK);

    int n = snprintf(out, out_len, "%s %.1f GB", type, total / 1073741824.0);
    if (have_id && n > 0 && (size_t)n < out_len) {
        if (label[0])
            snprintf(out + n, out_len - n, "  %s %04X-%04X", label,
                     (unsigned)(vsn >> 16), (unsigned)(vsn & 0xFFFF));
        else
            snprintf(out + n, out_len - n, "  %04X-%04X",
                     (unsigned)(vsn >> 16), (unsigned)(vsn & 0xFFFF));
    }
    return true;
}

bigfile_t *bigfile_open(const char *path) {
    if (!path || !*path) return nullptr;

    bigfile_t *b = (bigfile_t *)calloc(1, sizeof *b);
    if (!b) return nullptr;

    // Built straight into b->path rather than into scratch and copied. Two
    // buffers means two sizes, and the compiler is right to object when the
    // second is smaller than what the first can hold - a name that opened
    // successfully would then be recorded truncated.
    for (int drv = 0; drv < FF_VOLUMES; drv++) {
        int n = snprintf(b->path, sizeof b->path, "%d:%s%s",
                         drv, path[0] == '/' ? "" : "/", path);
        if (n < 0 || (size_t)n >= sizeof b->path) {
            Serial.printf("bigfile: %s is too long to name on volume %d\n",
                          path, drv);
            continue;
        }
        if (f_open(&b->fp, b->path, FA_READ) != FR_OK) continue;

        b->size = (uint64_t)f_size(&b->fp);
        clmt_build(b);
        return b;
    }
    b->path[0] = 0;

    free(b);
    return nullptr;
}

void bigfile_close(bigfile_t *b) {
    if (!b) return;
    f_close(&b->fp);
    // After f_close, so nothing can seek through a freed table.
#if FF_USE_FASTSEEK
    b->fp.cltbl = nullptr;
#endif
    free(b->clmt);
    free(b);
}

// Build a cluster link map so f_lseek() stops walking the FAT.
//
// Without one, f_lseek() follows the cluster chain entry by entry from
// wherever the file position happens to be, reading FAT sectors as it goes.
// That is fine for a few MB and ruinous for a 131 GB archive, because the
// access pattern here is the worst case for it: a lookup reads its leaf
// directory from near the end of the file and its tile body from the middle,
// so consecutive calls jump about 129 GB and back.
//
// Measured on the planet archive before this: a nine-tile place block spent
// 4984 ms in ten seeks and 71 ms transferring 488 KB. The card was doing
// about 6.9 MB/s; the seeks were costing roughly 498 ms each. A 3 GB extract
// on a slower USB stick beat it by more than 20x for exactly this reason -
// shorter chains, not faster hardware.
//
// The table is two entries per contiguous fragment plus a terminator, so a
// freshly copied archive needs very few and a badly fragmented one needs
// more. Rather than guess, this asks: FatFs returns the required item count
// in cltbl[0] when the buffer is too small, so a first attempt at a modest
// size either succeeds or says exactly what to allocate.
//
// Failure is not fatal anywhere. cltbl is left null and f_lseek() reverts to
// walking the chain - slow, which is what it was doing already. That matters
// because a fragmented multi-GB file could legitimately need a table too big
// to be worth holding, and refusing to open the archive over it would be a
// far worse outcome than reading it slowly.
// Said at compile time as well as at runtime, for the same reason the
// FF_FS_EXFAT check at the top of this file is: a Serial.println() only helps
// somebody who is watching the console at the moment the first archive opens,
// and the symptom otherwise is slowness, which looks like the card or the
// archive rather than the config. This is the line that names the option.
//
// A #warning and not an #error. Fast seek is a performance feature and a build
// without it works; refusing to compile would also punish anyone deliberately
// running -DMAP_FATFS_EXFAT=0 on small FAT32 media, where chains are short and
// the table is not worth its RAM.
#if !FF_USE_FASTSEEK
#warning "fatfs has FF_USE_FASTSEEK=0 - f_lseek walks the FAT chain, ~498 ms/seek on large archives. Set CONFIG_FATFS_USE_FASTSEEK=y and delete sdkconfig."
#endif

static void clmt_build(bigfile_t *b) {
#if !FF_USE_FASTSEEK
    // FIL has no cltbl member unless FatFs was built with fast seek, so this
    // has to compile out rather than merely do nothing - referring to the
    // field at all is a build error.
    //
    // Reaching here means CONFIG_FATFS_USE_FASTSEEK is off in the sdkconfig
    // this build actually used. Note that sdkconfig.defaults is only consulted
    // when there is no sdkconfig yet: an existing one takes precedence and
    // will silently keep the old value. Either delete sdkconfig and rebuild,
    // or set the option through menuconfig.
    (void)b;
    static bool said = false;
    if (!said) {
        said = true;
        Serial.println("bigfile: built without FF_USE_FASTSEEK - every seek "
                       "walks the FAT chain. Set CONFIG_FATFS_USE_FASTSEEK "
                       "(sdkconfig.defaults is ignored if sdkconfig exists).");
    }
    return;
#else
    // Fast seek requires the file be open read-only, which f_open above does.
    static const uint32_t CLMT_FIRST_TRY = 256;   // items, 1 KB
    static const uint32_t CLMT_MAX       = 16384; // items, 64 KB

    uint32_t items = CLMT_FIRST_TRY;
    for (int attempt = 0; attempt < 2; attempt++) {
        DWORD *tbl = (DWORD *)heap_caps_malloc(items * sizeof(DWORD),
                                               MALLOC_CAP_SPIRAM);
        if (!tbl) tbl = (DWORD *)malloc(items * sizeof(DWORD));
        if (!tbl) break;

        b->fp.cltbl = tbl;
        tbl[0] = items;
        FRESULT r = f_lseek(&b->fp, CREATE_LINKMAP);
        if (r == FR_OK) {
            b->clmt = tbl;
            Serial.printf("bigfile: %s fast seek on, %lu of %lu table items\n",
                          b->path, (unsigned long)tbl[0], (unsigned long)items);
            f_lseek(&b->fp, 0);
            return;
        }

        // Too small: cltbl[0] now holds what it actually needs.
        uint32_t need = (r == FR_NOT_ENOUGH_CORE) ? (uint32_t)tbl[0] : 0;
        b->fp.cltbl = nullptr;
        free(tbl);

        if (!need || need > CLMT_MAX || attempt == 1) {
            Serial.printf("bigfile: %s fast seek unavailable (%s) - seeks will "
                          "walk the FAT chain\n", b->path,
                          need ? "table too large" : "linkmap failed");
            return;
        }
        items = need;
    }
    b->fp.cltbl = nullptr;
#endif  // FF_USE_FASTSEEK
}

uint64_t bigfile_size(bigfile_t *b) { return b ? b->size : 0; }

// Seek and transfer time, accumulated separately. See bigfile_io_counters().
static uint64_t s_seek_us = 0, s_xfer_us = 0;
static uint32_t s_seeks   = 0;

void bigfile_io_counters(uint64_t *seek_us, uint64_t *xfer_us, uint32_t *seeks) {
    if (seek_us) *seek_us = s_seek_us;
    if (xfer_us) *xfer_us = s_xfer_us;
    if (seeks)   *seeks   = s_seeks;
}

int bigfile_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst) {
    bigfile_t *b = (bigfile_t *)ctx;
    if (!b) return -1;
    if (off + len > b->size) return -1;

    // The seek is timed apart from the transfer because they are suspected of
    // costing wildly different amounts, and nothing so far could tell them
    // apart.
    //
    // A traced place block read 489 KB in 5590 ms - about 87 KB/s on SDMMC
    // 4-bit, where tens of MB/s is normal, and roughly 91 ms per 8 KB chunk.
    // The read pattern is the reason to suspect the seek rather than the
    // transfer: a lookup takes its leaf directory from near the end of the
    // archive and its tile body from the middle, so consecutive calls jump
    // about 129 GB and back, over and over.
    //
    // That was measured, and the seek won: 4984 ms against 71 ms. The cause
    // was f_lseek() walking the FAT chain entry by entry, which
    // CONFIG_FATFS_USE_FASTSEEK and the cluster link map in clmt_build() now
    // avoid. The counters stay because they are how the fix is confirmed, and
    // because an archive whose linkmap could not be built still walks.
    uint64_t t_seek = esp_timer_get_time();
    if (f_lseek(&b->fp, (FSIZE_t)off) != FR_OK) return -1;
    if (f_tell(&b->fp) != (FSIZE_t)off) return -1;   // truncated: wrong bytes
    s_seek_us += (uint64_t)(esp_timer_get_time() - t_seek);
    s_seeks++;

    uint64_t t_xfer = esp_timer_get_time();

    // Chunked for the same reason storage_read() is: the USB MSC driver sizes
    // its DMA transfer buffer to whatever it is asked for, and a large ask
    // that fails to allocate leaves a dangling buffer behind. STORAGE_IO_CHUNK.
    uint32_t done = 0;
    while (done < len) {
        UINT want = (UINT)((len - done) > STORAGE_IO_CHUNK ? STORAGE_IO_CHUNK
                                                           : (len - done));
        UINT got = 0;
        if (f_read(&b->fp, dst + done, want, &got) != FR_OK) return -1;
        if (got == 0) return -1;
        done += got;
    }
    s_xfer_us += (uint64_t)(esp_timer_get_time() - t_xfer);
    return 0;
}

#else   // !MAP_HAVE_BIGFILE

bool       bigfile_supported() { return false; }
int        bigfile_volumes() { return 0; }
bool       bigfile_volume_info(int, char *, size_t) { return false; }

// Zeroed rather than absent: callers report these unconditionally, and a
// build without the 64-bit reader has simply never seeked through it.
void       bigfile_io_counters(uint64_t *seek_us, uint64_t *xfer_us,
                               uint32_t *seeks) {
    if (seek_us) *seek_us = 0;
    if (xfer_us) *xfer_us = 0;
    if (seeks)   *seeks   = 0;
}
int        bigfile_volume_of(const char *, uint64_t) { return -1; }
bigfile_t *bigfile_open(const char *) { return nullptr; }
void       bigfile_close(bigfile_t *) {}
uint64_t   bigfile_size(bigfile_t *) { return 0; }
int        bigfile_read(void *, uint64_t, uint32_t, uint8_t *) { return -1; }

#endif
