// idf_usb_msc.cpp - USB mass storage host, IDF builds only.
//
// Rewritten against the real headers. The first version used enum names taken
// from the C examples and they do not resolve in C++: the event enumerators
// live in an *anonymous enum nested inside* msc_host_event_t, so in C they
// land at file scope and in C++ they are members of the struct. Hence the
// msc_host_event_t:: qualification below.
//
// Every name and signature here now comes from espressif/usb_host_msc's and
// IDF's own headers rather than from memory. It is still untested against real
// hardware - I have no drive to plug in.
//
// Shape of the thing: the MSC driver runs its own background task and calls
// msc_event() when a drive appears. Enumeration is asynchronous, so
// usb_msc_begin() cannot report a mounted drive - it brings the stack up and
// returns, and storage_rescan() picks the drive up once s_mounted flips.

#include "../features.h"

#if MAP_HAVE_USB_MSC

#include <string.h>
#include <Arduino.h>
#include <FS.h>
#include <vfs_api.h>

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
static fs::FS s_fs = fs::FS(fs::FSImplPtr(new VFSImpl()));

static void msc_event(const msc_host_event_t *e, void *arg) {
    (void)arg;

    if (e->event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
        Serial.printf("usb: mass storage connected at address %u\n",
                      (unsigned)e->device.address);

        if (msc_host_install_device(e->device.address, &s_dev) != ESP_OK) {
            Serial.println("usb: install_device failed");
            s_dev = nullptr;
            return;
        }

        // format_if_mount_failed stays false deliberately, for the same reason
        // as the SD path: a drive that will not mount may hold a filesystem
        // this build cannot read rather than none at all, and erasing someone's
        // drive because we could not read it is the wrong default.
        esp_vfs_fat_mount_config_t mcfg = {};
        mcfg.format_if_mount_failed = false;
        mcfg.max_files              = 5;
        mcfg.allocation_unit_size   = 0;

        if (msc_host_vfs_register(s_dev, USB_MOUNT, &mcfg, &s_vfs) != ESP_OK) {
            Serial.println("usb: vfs_register failed - unreadable filesystem?");
            msc_host_uninstall_device(s_dev);
            s_dev = nullptr;
            return;
        }

        s_mounted = true;
        Serial.printf("usb: mounted at %s\n", USB_MOUNT);

    } else if (e->event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
        Serial.println("usb: mass storage removed");

        // Cleared first, so nothing can start a new read against a handle that
        // is about to go away. Callers re-run storage_rescan() rather than
        // holding on to the old fs::FS.
        s_mounted = false;
        if (s_vfs) { msc_host_vfs_unregister(s_vfs); s_vfs = nullptr; }
        if (s_dev) { msc_host_uninstall_device(s_dev); s_dev = nullptr; }
    }
}

// Bring up the host stack and the MSC driver. Reports whether the stack came
// up, not whether a drive is present - enumeration happens afterwards, on the
// driver's own task.
bool usb_msc_begin() {
    if (s_installed) return true;

    usb_host_config_t host_cfg = {};
    host_cfg.skip_phy_setup = false;
    host_cfg.intr_flags     = ESP_INTR_FLAG_LEVEL1;
    if (usb_host_install(&host_cfg) != ESP_OK) {
        Serial.println("usb: host_install failed");
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
