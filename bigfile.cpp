// bigfile.cpp - see bigfile.h for why the VFS cannot do this.

#include "bigfile.h"
#include "storage.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <esp_timer.h>

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
};

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
        return b;
    }
    b->path[0] = 0;

    free(b);
    return nullptr;
}

void bigfile_close(bigfile_t *b) {
    if (!b) return;
    f_close(&b->fp);
    free(b);
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
    // CONFIG_FATFS_USE_FASTSEEK is not set in this build and nothing here
    // builds a cluster link map, so f_lseek() walks the FAT chain entry by
    // entry from wherever the file position happens to be. On a 131 GB file
    // that chain is millions of entries long, and a 129 GB jump walks a large
    // part of it, reading FAT sectors along the way. That would be
    // proportional to file size, would fall on every read rather than only on
    // directory reads, and would be unaffected by the medium - which matches
    // a 3 GB extract on a slower USB stick beating this by more than 20x.
    //
    // If seek dominates, enabling fast seek and allocating a CLMT per open
    // archive turns the walk into a table lookup. If transfer dominates
    // instead, the seek theory is dead and the 8 KB STORAGE_IO_CHUNK is the
    // next suspect. Measuring says which; guessing has already been wrong
    // twice here.
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
