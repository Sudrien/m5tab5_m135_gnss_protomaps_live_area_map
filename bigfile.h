// bigfile.h - read an archive larger than 4 GiB.
//
// WHY THIS EXISTS
//
// Nothing in the normal read path can address past 4 GiB, and every layer
// fails silently rather than saying so:
//
//   fs::File::seek(uint32_t)      the API is 32-bit. An offset of 5 GiB
//                                 arrives as 705 MB and the read succeeds,
//                                 returning the wrong bytes.
//   esp_vfs_fat lseek(off_t)      off_t is long, 32 bits on this toolchain.
//   FatFs FSIZE_t                 DWORD unless FF_FS_EXFAT, then QWORD.
//
// So the ceiling is not one thing to fix; the VFS is 32-bit by its own type
// signatures and always will be. The way past it is to skip the VFS and call
// FatFs directly: f_lseek() takes FSIZE_t, which is 64-bit once exFAT is
// enabled, and the volume is already mounted - esp_vfs_fat_sdmmc_mount()
// registers a FatFs drive and then wraps it, so f_open("0:/planet.pmtiles")
// reaches the same filesystem the fs::FS object does.
//
// PREREQUISITES, both of them real
//
//   1. FF_FS_EXFAT = 1. ESP-IDF hardcodes it to 0 in
//      components/fatfs/src/ffconf.h and exposes no Kconfig for it, so this
//      means vendoring the fatfs component. Without it FSIZE_t stays 32-bit
//      and this file is no better than fs::File - which is why bigfile_open()
//      refuses at compile time rather than letting it look like it works.
//
//   2. The archive has to be on a volume that can hold it, which for 126 GB
//      means exFAT, which is the same switch. FAT32 caps one file at 4 GiB
//      regardless of how the file is read.
//
// Everything here is read-only. That is deliberate: a bypass of the VFS that
// could also write is a second path to the allocation table, and there is no
// reason to have one.

#pragma once
#include "features.h"
#include <stddef.h>
#include <stdint.h>

// Opaque by design - ff.h should not leak into netsource.cpp, which also
// compiles under Arduino where FatFs is not on the include path.
typedef struct bigfile_s bigfile_t;

// Open `path` ("/planet.pmtiles") by trying each mounted FatFs volume in turn.
// Returns NULL if the file is not found on any of them, or if this build
// cannot address 64-bit offsets - in which case it logs why.
bigfile_t *bigfile_open(const char *path);

void       bigfile_close(bigfile_t *b);
uint64_t   bigfile_size(bigfile_t *b);

// Signature matches pmt_read_fn exactly, so it drops straight into a pmt_t.
// `ctx` is the bigfile_t*. Returns 0 on success.
int        bigfile_read(void *ctx, uint64_t off, uint32_t len, uint8_t *dst);

// Which FatFs volume is this file on?
//
// The mapping from a medium to a FatFs drive number is not otherwise
// discoverable. Each mount takes the first free pdrv, so a lone medium is
// always drive 0 whichever medium it is, and with both present the numbering
// follows plug order. IDF exposes no way to ask which number a VFS mount point
// was given.
//
// The volume serial does not help: FatFs can read it, but nothing on the
// fs::FS side can, so it identifies a volume across boots rather than tying a
// volume to a medium.
//
// A filename does bridge the two, because it was found by enumerating the
// medium and so is definitionally on it. `name` is a root-relative path
// ("/planet.pmtiles"); this stats it on each drive and returns the one that
// has it, or -1. `size` guards the case where both media happen to hold a file
// of the same name - pass 0 to skip the check.
int bigfile_volume_of(const char *name, uint64_t size);

// Describe a mounted FatFs volume: "exFAT 119.2 GB  PLANET 1A2B-3C4D".
//
// Which filesystem is actually in play is not otherwise visible anywhere. It
// decides whether a file over 4 GiB can exist at all, and a drive that came
// exFAT from the factory and got reformatted FAT32 by a camera looks identical
// from the outside - so it belongs on the boot list, next to which medium won,
// rather than in a log nobody reads on the move.
//
// The label and serial are there for the other question a boot list gets asked
// - whether the drive in your hand is the one you populated last week - which
// a capacity does not answer and a name does.
//
// Returns false when that drive is not mounted. `drv` runs 0..bigfile_volumes().
bool bigfile_volume_info(int drv, char *out, size_t out_len);
int  bigfile_volumes();

// Can this build reach past 4 GiB at all? Checked once at boot so the log
// says so before an archive is rejected for it.
bool       bigfile_supported();
