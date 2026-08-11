// idf_usb_msc.cpp - USB mass storage host, IDF builds only.
//
// Rewritten against the real headers. The first version used enum names taken
// from the C examples and they do not resolve in C++: the event enumerators
// live in an *anonymous enum nested inside* msc_host_event_t, so in C they
// land at file scope and in C++ they are members of the struct. Hence the
// msc_host_event_t:: qualification below.
//
// Every name and signature here comes from espressif/usb_host_msc's and IDF's
// own headers rather than from memory. Written without hardware to test on,
// and it showed: the first version brought the stack up, powered the port and
// then sat there while a drive was plugged in, because nothing was running the
// host library's event loop. See usb_lib_task below.
//
// Shape of the thing: three tasks, all required.
//
//   usb_lib_task            usb_host_lib_handle_events()  - the host library:
//                           enumeration, transfers, refcounting.
//   the MSC driver's own    msc_host_handle_events()      - the client, which
//   background task                                         calls msc_event().
//   usb_msc_task            installs and mounts a drive, off the back of a
//                           queue, because doing it inside msc_event() would
//                           block the task that has to complete the transfers
//                           it waits on.
//
// Enumeration is asynchronous, so usb_msc_begin() cannot report a mounted
// drive - it brings both loops up and returns, and storage_rescan() picks the
// drive up once s_mounted flips.

#include "../features.h"

#if MAP_HAVE_USB_MSC

#include <string.h>
#include <Arduino.h>
#include <FS.h>
#include <vfs_api.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"

static const char *USB_MOUNT = "/usb";

static bool                     s_installed = false;
static volatile bool            s_mounted   = false;
static msc_host_device_handle_t s_dev = nullptr;
static msc_host_vfs_handle_t    s_vfs = nullptr;

// fs::FS over the VFS mount, built the same way SD_MMC is - a VFSImpl pointed
// at a mount point. That is what lets the rest of the project treat a USB
// drive and an SD card identically.
// A bare fs::FS is not enough: VFSImpl resolves every path against a mount
// point, and nothing outside the class can set it - FS::_impl is protected.
// SD_MMC gets away with a plain member because it *is* a subclass and calls
// _impl->mountpoint() in begin(). The first version of this file constructed
// an fs::FS directly, so its mount point stayed empty and every open() would
// have resolved to a path with no prefix.
class UsbFS : public fs::FS {
public:
    UsbFS() : fs::FS(fs::FSImplPtr(new VFSImpl())) {}
    void setMount(const char *mp) { _impl->mountpoint(mp); }
};
static UsbFS s_fs;

// The host library's own event loop.
//
// This was missing, and its absence is silent in exactly the way that wastes
// an afternoon: usb_host_install() succeeds, msc_host_install() succeeds, the
// port has power, a drive is plugged in - and no callback ever fires, because
// nothing is driving enumeration.
//
// msc_host_install(create_backround_task = true) spawns a task for the *client*
// event loop, msc_host_handle_events(). That is a different loop. The host
// library underneath it does enumeration, transfer completion and the
// reference counting that decides when a device object can be freed, and it
// only runs while something calls usb_host_lib_handle_events().
//
// src: m5tab5_esp_idf_usb_host_example, usb_lib_task - "Nothing else works
//      until this is running", which is precisely accurate.
static void usb_lib_task(void *arg) {
    (void)arg;
    for (;;) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

// Runs on the MSC driver's own task. Does nothing but hand the event on.
//
// Everything this used to do inline - msc_host_install_device(),
// msc_host_vfs_register() - performs USB transfers, and a transfer cannot
// complete while the task that would deliver its completion is sitting inside
// the call waiting for it. The result is a deadlock with no error and no
// crash: the log stops after "mass storage connected at address 1" and the
// driver task never runs again.
//
// src: m5tab5_esp_idf_usb_host_example, msc_event_cb - "Runs on the MSC driver
//      task; mounting from here would block it."
static QueueHandle_t s_events = nullptr;

static void msc_event(const msc_host_event_t *e, void *arg) {
    (void)arg;
    if (s_events) xQueueSend(s_events, e, 0);
}

// Where the actual work happens, on a task of our own.
static void usb_msc_task(void *arg) {
    (void)arg;
    msc_host_event_t e;

    for (;;) {
        if (xQueueReceive(s_events, &e, portMAX_DELAY) != pdTRUE) continue;

        if (e.event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
            Serial.printf("usb: mass storage connected at address %u\n",
                          (unsigned)e.device.address);

            if (msc_host_install_device(e.device.address, &s_dev) != ESP_OK) {
                Serial.println("usb: install_device failed");
                s_dev = nullptr;
                continue;
            }

            msc_host_device_info_t info;
            if (msc_host_get_device_info(s_dev, &info) == ESP_OK) {
                const uint64_t bytes =
                    (uint64_t)info.sector_count * info.sector_size;
                Serial.printf("usb: %llu MB (%lu x %lu byte sectors)\n",
                              (unsigned long long)(bytes / (1024 * 1024)),
                              (unsigned long)info.sector_count,
                              (unsigned long)info.sector_size);
            }

            // format_if_mount_failed stays false deliberately, for the same
            // reason as the SD path: a drive that will not mount may hold a
            // filesystem this build cannot read rather than none at all, and
            // erasing someone's drive because we could not read it is the
            // wrong default.
            esp_vfs_fat_mount_config_t mcfg = {};
            mcfg.format_if_mount_failed = false;
            mcfg.max_files              = 5;
            mcfg.allocation_unit_size   = 0;

            esp_err_t err = msc_host_vfs_register(s_dev, USB_MOUNT, &mcfg, &s_vfs);
            if (err != ESP_OK) {
                // Print the code. The first version of this said only
                // "unreadable filesystem?", which is a guess dressed as a
                // diagnosis - the call can also fail on a busy mount point or
                // an out-of-memory, and those look nothing alike.
                Serial.printf("usb: vfs_register failed: %s (0x%x)\n",
                              esp_err_to_name(err), (unsigned)err);

                // The overwhelmingly likely cause on a drive this size, so say
                // it rather than making someone guess:
                //
                // ESP-IDF's FATFS is FAT12/16/32 only - FF_FS_EXFAT is off and
                // there is no Kconfig to turn it on - and it reads an MBR
                // partition table, not GPT. A 118 GB drive out of the box is
                // almost always exFAT, and often GPT with it. Neither is a
                // fault in the drive or in this code; the drive simply cannot
                // be read as it stands.
                Serial.println("usb: ESP-IDF reads FAT12/16/32 on an MBR "
                               "partition table only - not exFAT, not GPT");
                Serial.println("usb: either reformat (mkfs.vfat -F 32 on an "
                               "msdos label) or build with "
                               "idf.py -DMAP_FATFS_EXFAT=1, which vendors a "
                               "FatFs with exFAT and GPT turned on");

                msc_host_uninstall_device(s_dev);
                s_dev = nullptr;
                continue;
            }

            s_fs.setMount(USB_MOUNT);
            s_mounted = true;
            Serial.printf("usb: mounted at %s\n", USB_MOUNT);

        } else if (e.event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
            Serial.println("usb: mass storage removed");

            // Cleared first, so nothing can start a new read against a handle
            // that is about to go away. Callers re-run storage_rescan() rather
            // than holding on to the old fs::FS.
            s_mounted = false;
            if (s_vfs) { msc_host_vfs_unregister(s_vfs); s_vfs = nullptr; }
            if (s_dev) { msc_host_uninstall_device(s_dev); s_dev = nullptr; }
        }
    }
}

// Say yes to every device.
//
// Required, not optional, when CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK is
// set - and it is in this build. The relevant code in IDF's enum.c is:
//
//     bool enum_proceed = false;
//     if (p_enum_driver->constant.enum_filter_cb) {
//         enum_proceed = p_enum_driver->constant.enum_filter_cb(dev_desc, &bConfigurationValue);
//     }
//     if (!enum_proceed) { ... enum_cancel(...); return ESP_OK; }
//
// so a null callback is not "no filtering", it is "reject everything". The
// device is found, its descriptor is read, and then enumeration is cancelled:
//
//     W ENUM: [0:0] Abort request of enumeration process (%#x:%#x)
//
// and that warning is the only trace. The two numbers are idProduct then
// idVendor, in that order, which is worth knowing because it reads like a
// VID:PID and is not one.
//
// bConfigurationValue arrives as ENUM_DEFAULT_CONFIGURATION_VALUE and is left
// alone: configuration 1 is what a mass storage device has, and a drive with a
// stranger layout is better handled by the MSC driver rejecting it than by
// this guessing.
static bool usb_enum_filter(const usb_device_desc_t *dev_desc,
                            uint8_t *bConfigurationValue) {
    (void)bConfigurationValue;
    Serial.printf("usb: device %04x:%04x offered, accepting\n",
                  (unsigned)dev_desc->idVendor, (unsigned)dev_desc->idProduct);
    return true;
}

// Bring up the host stack and the MSC driver. Reports whether the stack came
// up, not whether a drive is present - enumeration happens afterwards, on the
// driver's own task.
bool usb_msc_begin() {
    if (s_installed) return true;

    usb_host_config_t host_cfg = {};
    host_cfg.skip_phy_setup = false;
    host_cfg.intr_flags     = ESP_INTR_FLAG_LEVEL1;
#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
    host_cfg.enum_filter_cb = usb_enum_filter;
#endif
    if (usb_host_install(&host_cfg) != ESP_OK) {
        Serial.println("usb: host_install failed");
        return false;
    }

    // Before the MSC driver, and at a higher priority than its client task -
    // the client cannot make progress on an event the library has not raised.
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, nullptr, 10, nullptr) != pdPASS) {
        Serial.println("usb: could not start the host library task");
        usb_host_uninstall();
        return false;
    }

    s_events = xQueueCreate(4, sizeof(msc_host_event_t));
    if (!s_events) {
        Serial.println("usb: could not create the event queue");
        usb_host_uninstall();
        return false;
    }
    if (xTaskCreate(usb_msc_task, "usb_msc", 4096, nullptr, 4, nullptr) != pdPASS) {
        Serial.println("usb: could not start the mass storage task");
        usb_host_uninstall();
        return false;
    }

    msc_host_driver_config_t drv_cfg = {};
    drv_cfg.create_backround_task = true;   // spelling is upstream's, not a typo
    drv_cfg.task_priority         = 5;
    drv_cfg.stack_size            = 4096;
    drv_cfg.core_id               = tskNO_AFFINITY;
    drv_cfg.callback              = msc_event;
    drv_cfg.callback_arg          = nullptr;
    if (msc_host_install(&drv_cfg) != ESP_OK) {
        Serial.println("usb: msc_host_install failed");
        usb_host_uninstall();
        return false;
    }

    s_installed = true;
    Serial.println("usb: host stack up, waiting for a drive");
    return true;
}

bool    usb_msc_mounted() { return s_mounted; }
fs::FS *usb_msc_fs()      { return &s_fs; }

#endif  // MAP_HAVE_USB_MSC
