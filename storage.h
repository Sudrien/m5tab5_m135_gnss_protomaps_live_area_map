// storage.h - one place that decides which filesystem everything else uses.
//
// The choice was previously made in three files independently, each with its
// own copy of the same SD_MMC-else-SD test. That works only while there is
// exactly one answer; the moment a second backing store exists, three copies
// are three chances to disagree about which one is mounted, and the failure
// would be a subset of the code silently reading from a different device.
//
// Everything here is deliberately thin: it does not mount anything, it only
// reports what the boot sequence already mounted.

#pragma once
#include <FS.h>

// The filesystem holding /t (tile cache, manifests) and /wifi.bin.
// Never null - falls back to the SPI SD object, whose calls fail cleanly if
// nothing is mounted, which is the same behaviour the callers had before.
fs::FS *storage_fs();

// Human-readable name of the active store, for the boot line.
const char *storage_name();

// Re-run the selection. Call after mounting or removing a device.
void storage_rescan();

// How many files the filesystem layer must be able to hold open at once.
//
// Both SD_MMC.begin() and esp_vfs_fat_mount_config_t default to 5, which is
// fewer than this project needs and fails in a way that does not name itself:
// the sixth open() returns "no free file descriptors", and everything that
// follows - archives past the fifth, the tile cache, the world-floor position,
// the remembered fix - fails as though the files were missing.
//
// LOCAL_ARCHIVE_MAX archives can be open at once, and each is held for the run.
// The rest is the small stuff opened alongside them: the tile cache blob and
// its build stamp, world.pos, lastfix.bin, wifi.bin, aopdb.bin, and room to
// spare. Each descriptor costs a FIL struct with a sector buffer, so this is
// tens of KB of internal heap, not megabytes.
#define STORAGE_MAX_OPEN_FILES 24

// Largest single read or write to hand the filesystem in one call.
//
// FatFs passes a multi-sector run straight through to the driver, and the USB
// MSC driver sizes its DMA-capable transfer buffer to whatever it is asked
// for (esp-usb msc_host.c):
//
//     if (xfer->data_buffer_size < transfer_size) {
//         MSC_RETURN_ON_ERROR( usb_host_transfer_free(xfer) );
//         MSC_RETURN_ON_ERROR( usb_host_transfer_alloc(transfer_size, 0, &device->xfer) );
//
// So a 66 KB write asks for a 66 KB internal DMA allocation. With the USB host
// stack, three extra tasks and 24 file descriptors already resident, internal
// DMA memory is thin - and when that alloc fails, the driver has *already
// freed* the old buffer and returns leaving device->xfer dangling. The next
// transfer uses it. That is the CORRUPT HEAP, and it is upstream's bug, but
// only reachable by asking for a transfer too big to allocate.
//
// The buffer only ever grows, so keeping every request small keeps it small.
// 8 KB is 16 sectors: large enough that the per-transfer overhead is
// negligible, small enough to allocate under pressure.
#define STORAGE_IO_CHUNK 8192

// Read/write in STORAGE_IO_CHUNK pieces. Same return convention as
// File::read/File::write - the byte count, short on failure.
size_t storage_read(fs::File &f, uint8_t *dst, size_t len);
size_t storage_write(fs::File &f, const uint8_t *src, size_t len);

// Is there actually a device behind storage_fs(), or is this the empty
// fallback? storage_fs() is never null by design, so "did anything mount"
// cannot be answered by comparing it against nullptr.
bool storage_available();

// Bring the USB-A port's VBUS up.
//
// The port is electrically dead until this runs: USB5V_EN is P3 of the second
// PI4IOE5V6416 at I2C 0x44, not a GPIO, and M5Unified's Power_Class::begin()
// leaves that pin configured as an input. Nothing reports the port as off -
// the host stack installs, the client task registers, and no device ever
// enumerates.
//
// Safe to call more than once, and a no-op on builds without USB support.
void storage_usb_power(bool on);

// Power the port and install the host stack, without waiting for a drive.
//
// Separate from storage_rescan() so it can be started early: enumeration runs
// on the USB driver's own task, so anything that gives it a head start before
// the first question is asked is time not spent waiting later. Idempotent, and
// a no-op without USB support.
void storage_usb_begin();

// True while a card is physically in the slot.
//
// Polled, because it has to be: the microSD connector's detect switch is not
// wired to the SoC on this board - M5's BSP passes GPIO_NUM_NC for it - so
// there is no edge to interrupt on.
bool storage_card_present();
