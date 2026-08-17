// compass.cpp - see compass.h for what this is and why.
//
// Ported from Sudrien/m5tab5_esp_idf_m135_examples (main/bosch_aux.c and the
// heading half of main/imu_example.c), with one structural change that is the
// whole difficulty of the port:
//
//   THE EXAMPLE OWNS THE I2C BUS. THIS PROJECT DOES NOT.
//
// That example installs its own i2c_master bus handle and hands
// i2c_master_dev_handle_t to Bosch's read/write callbacks. It can, because
// nothing else in it touches the bus - it never starts M5Unified at all.
//
// Here M5Unified owns the internal bus: the touch controller, the RTC, the
// codec, the INA226 fuel gauge, both I/O expanders and the Tab5's own IMU are
// on those same two wires, and M5.begin() has already claimed them. Adding a
// second i2c_master owner for the same pins does not fail cleanly - it is two
// drivers arbitrating one peripheral, and the symptom is the touch controller
// or the fuel gauge going intermittent rather than anything pointing here.
//
// So the Bosch callbacks below go through M5.In_I2C instead. Same registers,
// same sequence, same driver; only the transport differs.
//
// THREADING
// The internal I2C bus belongs to loop()'s task, exactly as storage.cpp says
// for the USB VBUS write. The render worker must not reach in here. Enforced
// rather than documented - see the owner check in compass_update().

#include "compass.h"
#include "features.h"

#include <M5Unified.h>
#include <Arduino.h>
#include <math.h>
#include <string.h>

// Bosch's drivers are vendored by tools/fetch_bosch_drivers.sh (or by the
// CMake fetch that calls it). Absent them, every entry point below becomes a
// quiet no-op and the UI falls back to GNSS course. The guard is __has_include
// rather than a build flag so that a tree without the drivers builds without
// anyone having to know they are optional.
#if __has_include("bmi270.h")
#define COMPASS_HAVE_BOSCH 1
#include "bmi270.h"
#include "bmm150.h"
#else
#define COMPASS_HAVE_BOSCH 0
#endif

bool compass_supported() { return COMPASS_HAVE_BOSCH; }

#if !COMPASS_HAVE_BOSCH

// ---- stubs ----------------------------------------------------------------
// Deliberately not silent at boot: "no compass" with no reason given is the
// same output as a wiring fault, and the fix here is a one-line script.
bool compass_begin() {
    Serial.println("compass: built without Bosch's BMI270/BMM150 drivers - "
                   "run ./tools/fetch_bosch_drivers.sh and rebuild");
    Serial.println("compass: the dial will fall back to GNSS course, which "
                   "only exists while moving");
    return false;
}
bool  compass_ok()                  { return false; }
void  compass_update()              { }
float compass_heading()             { return -1.0f; }
float compass_field_ut()            { return 0.0f; }
float compass_roll()                { return 0.0f; }
float compass_pitch()               { return 0.0f; }
void  compass_set_position(double, double) { }
void  compass_calibrate_start()        { }
void  compass_calibrate_start_refine() { }
size_t compass_cal_blob_size()         { return 0; }
size_t compass_cal_export(void *, size_t)      { return 0; }
bool  compass_cal_import(const void *, size_t) { return false; }
bool  compass_cal_dirty()              { return false; }
void  compass_cal_clear_dirty()        { }
void  compass_calibrate_cancel()    { }
bool  compass_calibrating()         { return false; }
int   compass_calibrate_progress()  { return 0; }
const char *compass_status()        { return "not built in"; }

#else

// ---- board constants ------------------------------------------------------
//
// src: m5tab5_esp_idf_m135_examples/main/tab5_bus.h. The M135's BMI270 is at
//      0x69; 0x68 is the Tab5's own. The module's DIP switches can move it
//      onto 0x68, where it collides with the onboard part and both appear to
//      work while fighting over the address - so this refuses to fall back to
//      0x68 rather than "helpfully" finding a device there.
static const uint8_t BMI270_M135_ADDR = 0x69;

// EXT_5V_BUS gates the 5V rail feeding the rear M5-Bus, and with it the whole
// module. It is expander #1 at 0x43 pin P2 - a *different* expander from the
// 0x44 that storage.cpp uses for USB VBUS. Getting them backwards turns on the
// USB-A port while the module stays dark.
//
// src: m5tab5_esp_idf_m135_examples README, "EXT5V_EN is not a GPIO".
static const uint8_t PI4IOE1_ADDR   = 0x43;
static const uint8_t EXT5V_EN_BIT   = 1 << 2;

// Same three registers, same order, as storage.cpp's USB VBUS write:
// direction, out of high impedance, then drive. Writing OUT_SET alone leaves
// the pin floating and the rail off, with no error anywhere.
static const uint8_t PI4IOE_IO_DIR   = 0x03;
static const uint8_t PI4IOE_OUT_SET  = 0x05;
static const uint8_t PI4IOE_OUT_H_IM = 0x07;
static const uint32_t EXP_FREQ       = 100000;

// Matched to what M5GFX uses for this bus rather than chosen.
static const uint32_t I2C_FREQ = 400000;

// ---- state ----------------------------------------------------------------
static struct bmi2_dev   s_bmi;
static struct bmm150_dev s_bmm;
static bool  s_ok = false;
static bool  s_started = false;
static TaskHandle_t s_owner = nullptr;

static float s_heading = -1.0f;
static float s_field   = 0.0f;
static float s_declination = 0.0f;
static float s_roll  = 0.0f;
static float s_pitch = 0.0f;
static char  s_status[64] = "not started";

// Hard-iron offset, in microtesla, subtracted from every reading. Zero until
// calibrated, which is honest rather than useful: an uncalibrated heading on
// this hardware is wrong by up to a quarter turn, so compass_ok() stays true
// but the status line says so and the UI can decline to trust it.
static float s_offset[3] = { 0.0f, 0.0f, 0.0f };

// Per-axis gain from the ellipsoid fit. 1.0 means "no soft-iron correction",
// which is what the min/max path leaves behind and what a rejected fit falls
// back to - so every code path below stays valid with these untouched.
static float s_scale[3] = { 1.0f, 1.0f, 1.0f };
static bool  s_fitted   = false;

// Snapshot taken when a refine starts, so a refine that comes back worse can
// be undone. Refining is meant to add coverage to a calibration; it has no
// business replacing a good one with a worse one, which is exactly what a
// sweep taken next to a phone or a desk frame will otherwise do.
static float s_prev_offset[3], s_prev_scale[3];
static bool  s_prev_valid = false;

// Spread of the per-axis gains: 0 for a perfect sphere. The device's own soft
// iron is small and fixed, so a good fit lands within a few percent. A sweep
// contaminated by something external stretches the readings and shows up here
// as a large spread even when the residual looks respectable - the points are
// consistent, just with the wrong field.
static float gain_spread(const float g[3]) {
    float mn = g[0], mx = g[0];
    for (int i = 1; i < 3; i++) { if (g[i] < mn) mn = g[i]; if (g[i] > mx) mx = g[i]; }
    return (mn > 0.01f) ? (mx / mn - 1.0f) : 99.0f;
}
static bool  s_calibrated = false;

// ---- Bosch interface callbacks, over M5.In_I2C ----------------------------
//
// The signatures are Bosch's; the bodies are the ported part.

static BMI2_INTF_RETURN_TYPE bmi_read(uint8_t reg, uint8_t *data,
                                      uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    return M5.In_I2C.readRegister(BMI270_M135_ADDR, reg, data, len, I2C_FREQ)
           ? BMI2_OK : -1;
}

static BMI2_INTF_RETURN_TYPE bmi_write(uint8_t reg, const uint8_t *data,
                                       uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    return M5.In_I2C.writeRegister(BMI270_M135_ADDR, reg, data, len, I2C_FREQ)
           ? BMI2_OK : -1;
}

static void bmi_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    // Bosch asks in microseconds and the config upload issues thousands of
    // short ones. Rounding those all up to a 1 ms FreeRTOS tick turns an
    // 8 KB upload into minutes, so the short waits are busy-waited.
    if (period < 2000) delayMicroseconds(period);
    else               delay(period / 1000);
}

// The BMM150 is reached through the BMI270's aux master in manual mode, so
// its transport is not I2C at all from here - it is two register pokes at the
// part in front of it. bmi2_*_aux_man_mode handles the ordering that makes
// that work (data before address, poll aux_busy, drop power save around each
// transfer), which is exactly the part that defeated a hand-rolled version in
// the source project.
//
// Manual-mode reads are capped at the burst length in AUX_IF_CONF, and 8 bytes
// (BMI2_AUX_READ_LEN_3) is the largest the hardware offers. bmm150_init reads
// its factory trim as 2 bytes from 0x5D, 4 from 0x62 and *10* from 0x64 - and
// that last one is the first request in the entire bring-up sequence that does
// not fit. So a 1-byte chip ID read succeeds, every earlier step looks healthy,
// and the failure lands inside bmm150_init with nothing to say which register
// it was on.
//
// Splitting here rather than shortening the burst: the trim read is the only
// caller that needs it, and dropping every other transfer to 6 bytes to make
// one 10-byte read divide evenly would slow the 3-byte data path for nothing.
static const uint32_t AUX_MAX_BURST = 8;

static BMM150_INTF_RET_TYPE aux_read(uint8_t reg, uint8_t *data,
                                     uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    while (len > 0) {
        uint32_t n = len > AUX_MAX_BURST ? AUX_MAX_BURST : len;
        int8_t rc = bmi2_read_aux_man_mode(reg, data, n, &s_bmi);
        if (rc != BMI2_OK) return rc;
        reg  += (uint8_t)n;
        data += n;
        len  -= n;
    }
    return BMI2_OK;
}

static BMM150_INTF_RET_TYPE aux_write(uint8_t reg, const uint8_t *data,
                                      uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    // Writes have no equivalent burst register - the aux master pushes one
    // byte per transaction regardless - but the same chunking costs nothing
    // and keeps both directions bounded the same way.
    while (len > 0) {
        uint32_t n = len > AUX_MAX_BURST ? AUX_MAX_BURST : len;
        int8_t rc = bmi2_write_aux_man_mode(reg, data, n, &s_bmi);
        if (rc != BMI2_OK) return rc;
        reg  += (uint8_t)n;
        data += n;
        len  -= n;
    }
    return BMI2_OK;
}

// ---- module power ---------------------------------------------------------
static void ext5v_enable() {
    // Idempotent and quiet, same as storage_usb_power().
    static bool applied = false;
    if (applied) return;
    applied = true;

    M5.In_I2C.bitOn (PI4IOE1_ADDR, PI4IOE_IO_DIR,   EXT5V_EN_BIT, EXP_FREQ);
    M5.In_I2C.bitOff(PI4IOE1_ADDR, PI4IOE_OUT_H_IM, EXT5V_EN_BIT, EXP_FREQ);
    M5.In_I2C.bitOn (PI4IOE1_ADDR, PI4IOE_OUT_SET,  EXT5V_EN_BIT, EXP_FREQ);
    Serial.printf("compass: EXT_5V on (expander 0x%02X, P2)\n", PI4IOE1_ADDR);

    // The module's regulator and the BMI270 behind it need a moment before
    // they will answer. Without this the chip-ID read below fails on a cold
    // boot and succeeds on a warm one, which reads as a flaky module.
    delay(50);
}

// ---- bring-up -------------------------------------------------------------
bool compass_begin() {
    if (s_started) return s_ok;
    s_started = true;
    s_owner = xTaskGetCurrentTaskHandle();

    ext5v_enable();

    // Probe before initialising, so "module not fitted" is distinguishable
    // from "module fitted and misbehaving". bmi270_init on an absent device
    // fails several layers in with a code that says nothing about the cause.
    if (!M5.In_I2C.start(BMI270_M135_ADDR, false, I2C_FREQ)) {
        M5.In_I2C.stop();
        Serial.printf("compass: nothing at 0x%02X - M135 not fitted, or its "
                      "DIP has moved the IMU to 0x68 where it collides with "
                      "the Tab5's own\n", BMI270_M135_ADDR);
        snprintf(s_status, sizeof s_status, "no module");
        return false;
    }
    M5.In_I2C.stop();

    memset(&s_bmi, 0, sizeof s_bmi);
    // chip_id is an output field - bmi270_init overwrites it with what the
    // part reports. The I2C address reaches the callbacks as a compile-time
    // constant instead, so nothing reads this; left unset rather than
    // pre-loaded with an address, which invites the wrong conclusion.
    s_bmi.read            = bmi_read;
    s_bmi.write           = bmi_write;
    s_bmi.delay_us        = bmi_delay_us;
    s_bmi.intf            = BMI2_I2C_INTF;
    s_bmi.intf_ptr        = &s_bmi;       // liveness token; the address is a compile-time constant here
    s_bmi.read_write_len  = 32;
    s_bmi.config_file_ptr = nullptr;      // use the image built into bmi270.c

    // This is the 8 KB configuration upload. There is no usable image in ROM:
    // without it the BMI270 returns its chip ID and ignores everything else,
    // accelerometer and aux interface included, reporting no error at all.
    // It takes a moment; the log line is so a slow boot is not a mystery.
    Serial.println("compass: uploading BMI270 config (8 KB over I2C)");
    int8_t rc = bmi270_init(&s_bmi);
    if (rc != BMI2_OK) {
        Serial.printf("compass: bmi270_init failed (%d)\n", rc);
        snprintf(s_status, sizeof s_status, "IMU init failed (%d)", rc);
        return false;
    }

    // bmi270_init returning BMI2_OK is not the same as the config image having
    // landed. The driver reports OK on a scrambled upload; the part tells the
    // truth in INTERNAL_STATUS, where message[3:0] must read 0x1 (init_ok).
    // Anything else - 0x0 not_init, 0x2 init_err, 0x3 drv_err - means the
    // accelerometer and the aux master are both dead while every register
    // write below still ACKs, which is the whole reason a magnetometer failure
    // shows up three layers from its cause.
    //
    // src: BMI270 datasheet 5.2.5, INTERNAL_STATUS (0x21).
    uint8_t istat = 0;
    if (bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR, &istat, 1, &s_bmi) != BMI2_OK) {
        Serial.println("compass: INTERNAL_STATUS unreadable");
        snprintf(s_status, sizeof s_status, "IMU status unreadable");
        return false;
    }
    if ((istat & 0x0F) != 0x01) {
        Serial.printf("compass: BMI270 config did not take - INTERNAL_STATUS "
                      "0x%02X (want low nibble 0x1). The 8 KB image uploaded "
                      "and was rejected; nothing below can work.\n", istat);
        snprintf(s_status, sizeof s_status, "IMU config rejected (0x%02X)", istat);
        return false;
    }
    Serial.println("compass: BMI270 config uploaded, internal status ok");

    // Advanced power save gates the aux master. Bosch's own aux example
    // disables it before touching AUX_IF_CONF, and while the man-mode helpers
    // drop it around each transfer, they restore whatever they found - so
    // leaving it on turns the first transaction after every idle gap into an
    // aux_err. Cheap to disable outright; the part is awake anyway for the
    // accelerometer.
    if (bmi2_set_adv_power_save(BMI2_DISABLE, &s_bmi) != BMI2_OK)
        Serial.println("compass: adv_power_save disable failed (continuing)");

    // Internal pull-ups on the aux bus - the M135 has no external ones.
    uint8_t pupsel = BMI2_ASDA_PUPSEL_2K;
    if (bmi2_set_regs(BMI2_AUX_IF_TRIM, &pupsel, 1, &s_bmi) != BMI2_OK)
        Serial.println("compass: AUX_IF_TRIM write failed (continuing)");

    struct bmi2_sens_config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.type = BMI2_AUX;
    if (bmi270_get_sensor_config(&cfg, 1, &s_bmi) != BMI2_OK) {
        Serial.println("compass: aux get_sensor_config failed");
        snprintf(s_status, sizeof s_status, "aux config failed");
        return false;
    }
    cfg.cfg.aux.odr             = BMI2_AUX_ODR_100HZ;
    cfg.cfg.aux.aux_en          = BMI2_ENABLE;
    cfg.cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    cfg.cfg.aux.fcu_write_en    = BMI2_ENABLE;
    cfg.cfg.aux.man_rd_burst    = BMI2_AUX_READ_LEN_3;
    cfg.cfg.aux.read_addr       = BMM150_REG_DATA_X_LSB;
    cfg.cfg.aux.manual_en       = BMI2_ENABLE;
    if (bmi270_set_sensor_config(&cfg, 1, &s_bmi) != BMI2_OK) {
        Serial.println("compass: aux set_sensor_config failed");
        snprintf(s_status, sizeof s_status, "aux config failed");
        return false;
    }

    // The accelerometer is not optional here even though nothing displays it:
    // tilt compensation needs a gravity vector in the same frame as the
    // magnetometer, and only the module's own accel is rigidly fixed to the
    // BMM150 by the same PCB. The Tab5's onboard IMU would be wrong by
    // whatever angle the module sits at, silently.
    struct bmi2_sens_config acc;
    memset(&acc, 0, sizeof acc);
    acc.type = BMI2_ACCEL;
    if (bmi270_get_sensor_config(&acc, 1, &s_bmi) == BMI2_OK) {
        // 50 Hz is ample for a gravity vector sampled at 10 Hz, and the
        // averaging filter is the right trade here: this is measuring which
        // way is down, not tracking motion, so smoothing a hand tremor out is
        // exactly what is wanted. +-2 g because the device is held, not
        // thrown - a narrower range is finer resolution on the only signal
        // that matters.
        acc.cfg.acc.odr        = BMI2_ACC_ODR_50HZ;
        acc.cfg.acc.range      = BMI2_ACC_RANGE_2G;
        acc.cfg.acc.bwp        = BMI2_ACC_OSR4_AVG1;
        acc.cfg.acc.filter_perf = BMI2_POWER_OPT_MODE;
        if (bmi270_set_sensor_config(&acc, 1, &s_bmi) != BMI2_OK)
            Serial.println("compass: accel config failed (continuing on defaults)");
    }

    // Not "continuing" any more. With BMI2_AUX disabled the aux master never
    // clocks, every man-mode transfer times out, and the first symptom is
    // bmm150_init failing - which reads as a dead magnetometer. Enable the
    // accelerometer separately so a failure names which one.
    uint8_t sens_aux = BMI2_AUX;
    if (bmi270_sensor_enable(&sens_aux, 1, &s_bmi) != BMI2_OK) {
        Serial.println("compass: aux master would not enable - the BMM150 is "
                       "unreachable regardless of whether it is fitted");
        snprintf(s_status, sizeof s_status, "aux enable failed");
        return false;
    }
    uint8_t sens_acc = BMI2_ACCEL;
    if (bmi270_sensor_enable(&sens_acc, 1, &s_bmi) != BMI2_OK)
        Serial.println("compass: accel enable failed - heading will not be "
                       "tilt compensated (continuing)");

    // The aux master needs a poll interval or two after enable before its
    // first manual transfer lands. Without this the probe below fails on a
    // cold boot and passes on a warm one.
    delay(20);

    memset(&s_bmm, 0, sizeof s_bmm);
    s_bmm.read     = aux_read;
    s_bmm.write    = aux_write;
    s_bmm.delay_us = bmi_delay_us;
    s_bmm.intf     = BMM150_I2C_INTF;
    // Not nullptr, even though nothing reads it. Bosch's null_ptr_check in
    // bmm150.c rejects a null intf_ptr along with a null read/write/delay, so
    // bmm150_init returns BMM150_E_NULL_PTR (-1) before issuing a single
    // transaction - which is indistinguishable, from the outside, from a bus
    // that just failed. The aux address is not carried here at all; it lives
    // in the BMI270's AUX_DEV_ID, set by the aux config above, so this is a
    // liveness token and nothing more.
    //
    // The equivalent line for s_bmi is harmless only by accident: bmi2.c has
    // no intf_ptr check, so the same nullptr passes bmi270_init silently. Both
    // are pointed at s_bmi rather than leaving one of them to depend on which
    // vendor driver happens to validate what.
    s_bmm.intf_ptr = &s_bmi;
    s_bmm.chip_id  = BMM150_DEFAULT_I2C_ADDRESS;

    // The BMM150 boots into suspend and answers nothing - not even its chip
    // ID - until POWER_CONTROL (0x4B) bit 0 is set. bmm150_init does write it,
    // but it then reads the chip ID exactly once, and one failed read there is
    // indistinguishable from an absent part, a suspended part and a broken aux
    // path. Doing the wake and the probe here separately is what turns a
    // single boolean into a diagnosis.
    //
    // The retry is not superstition: the first man-mode transfer after enable
    // is the one that fails, and the second reliably does not.
    bool woke = false;
    for (int attempt = 0; attempt < 3 && !woke; attempt++) {
        uint8_t pwr = 0x01;
        if (aux_write(BMM150_REG_POWER_CONTROL, &pwr, 1, nullptr) != BMM150_OK) {
            Serial.printf("compass: aux write to BMM150 0x4B failed "
                          "(attempt %d) - the fault is in the BMI270's aux "
                          "master, not the magnetometer\n", attempt + 1);
            delay(10);
            continue;
        }
        delay(5);                       // BMM150 suspend-to-sleep is ~3 ms

        uint8_t id = 0;
        if (aux_read(BMM150_REG_CHIP_ID, &id, 1, nullptr) != BMM150_OK) {
            Serial.printf("compass: aux read of BMM150 chip ID failed "
                          "(attempt %d)\n", attempt + 1);
            delay(10);
            continue;
        }
        if (id != BMM150_CHIP_ID) {
            Serial.printf("compass: aux path works but chip ID is 0x%02X, "
                          "want 0x%02X - something else is answering at aux "
                          "address 0x%02X\n",
                          id, BMM150_CHIP_ID, BMM150_DEFAULT_I2C_ADDRESS);
            snprintf(s_status, sizeof s_status, "wrong part on aux (0x%02X)", id);
            return false;
        }
        woke = true;
    }
    if (!woke) {
        Serial.println("compass: no answer from the aux bus after three tries "
                       "- BMI270 config is good and the aux master is enabled, "
                       "so suspect the M135's BMM150 itself or its aux wiring");
        snprintf(s_status, sizeof s_status, "aux bus silent");
        return false;
    }

    int8_t mrc = bmm150_init(&s_bmm);
    if (mrc != BMM150_OK) {
        // The part answered its ID one line ago, so the aux path is not the
        // problem. What is left inside bmm150_init is the trim read.
        // -1 is BMM150_E_NULL_PTR and means the struct was rejected before
        // any transaction; -2 BMM150_E_DEV_NOT_FOUND; -4 BMM150_E_COM_FAIL is
        // the one that actually implicates the aux path.
        Serial.printf("compass: bmm150_init failed (%d)%s\n", mrc,
                      mrc == BMM150_E_NULL_PTR
                          ? " - struct rejected, no transaction attempted"
                          : " despite a good chip ID - trim read");
        snprintf(s_status, sizeof s_status, "mag init failed (%d)", mrc);
        return false;
    }

    // Trim of all zeroes compiles, initialises and produces a heading that is
    // smoothly, confidently wrong - the worst failure mode available here, and
    // invisible without this check.
    if (s_bmm.trim_data.dig_z4 == 0 && s_bmm.trim_data.dig_x1 == 0 &&
        s_bmm.trim_data.dig_xyz1 == 0) {
        Serial.println("compass: trim registers read back all zero - the "
                       "transfer succeeded and returned nothing real");
        snprintf(s_status, sizeof s_status, "trim empty");
        return false;
    }

    struct bmm150_settings set;
    memset(&set, 0, sizeof set);
    set.pwr_mode = BMM150_POWERMODE_NORMAL;
    if (bmm150_set_op_mode(&set, &s_bmm) != BMM150_OK)
        Serial.println("compass: set_op_mode failed (continuing)");
    set.preset_mode = BMM150_PRESETMODE_REGULAR;
    if (bmm150_set_presetmode(&set, &s_bmm) != BMM150_OK)
        Serial.println("compass: set_presetmode failed (continuing)");

    s_ok = true;
    Serial.printf("compass: BMM150 up behind the BMI270 at 0x%02X\n",
                  BMI270_M135_ADDR);
    snprintf(s_status, sizeof s_status, "uncalibrated");
    return true;
}

bool compass_ok() { return s_ok; }

// ---- calibration ----------------------------------------------------------
//
// Classic hard-iron: tumble through every orientation, take the midpoint of
// each axis's observed range, subtract it. The speaker magnet's contribution
// is fixed in sensor frame, so it moves the sphere of readings off centre by
// exactly that vector, and the midpoint recovers it.
//
// Progress is measured by how much of each axis's range has been seen rather
// than by elapsed time. A timer rewards waiting; this rewards turning, which
// is the thing that actually has to happen. A device left flat on a desk for
// the full minute would otherwise "complete" with an offset that biases every
// heading it later produces.
static bool  s_cal_on = false;
static bool  s_cal_refine = false;
static float s_cal_min[3], s_cal_max[3];

// Enough range on an axis to call it swept. The earth's field is 25-65 uT, so
// a fully turned axis spans twice that; 40 uT of range is a comfortable
// fraction of the smallest case without demanding a perfect tumble.
static const float CAL_RANGE_TARGET = 40.0f;

// ---- ellipsoid fit --------------------------------------------------------
//
// Min/max recovers a centre and nothing else, so it models the readings as a
// sphere that has been shifted. Nearby steel also *stretches* them, and a
// stretched surface has no centre that makes |B| constant - which is why the
// axis radii kept disagreeing by ~50% across sweeps with quite different
// coverage, and why |B| still swung 52-66 uT after a correction that had
// clearly worked in every other respect.
//
// Fitted model, axis-aligned:
//
//     a_x x^2 + a_y y^2 + a_z z^2 + b_x x + b_y y + b_z z = 1
//
// Completing the square gives centre c_i = -b_i / (2 a_i) and, with
// K = 1 + sum a_i c_i^2, radius_i = sqrt(K / a_i). Scaling each axis by
// mean_radius / radius_i is what actually removes the stretch.
//
// No cross terms. A full quadric needs nine parameters and an eigen
// decomposition to find rotated axes, and that is worth doing only when the
// distortion is not aligned with the sensor. The evidence here - one dominant
// axis, radii differing per-axis - does not call for it, and the six-parameter
// form solves with plain elimination and no eigen solver on the device.
static const int FIT_KEEP = 128;
static float  s_fit_pt[FIT_KEEP][3];
static int    s_fit_n = 0;          // points kept
static int    s_fit_seen = 0;       // points offered (for the stride)
static double s_ata[6][6], s_atb[6];

static void fit_reset() {
    s_fit_n = s_fit_seen = 0;
    for (int i = 0; i < 6; i++) {
        s_atb[i] = 0.0;
        for (int j = 0; j < 6; j++) s_ata[i][j] = 0.0;
    }
}

static void fit_accumulate(float x, float y, float z) {
    const double d[6] = { (double)x * x, (double)y * y, (double)z * z,
                          (double)x,     (double)y,     (double)z };
    for (int i = 0; i < 6; i++) {
        s_atb[i] += d[i];
        for (int j = 0; j < 6; j++) s_ata[i][j] += d[i] * d[j];
    }

    // Keep a thinned sample so the fit can be scored against real readings
    // afterwards. A fit with no residual check is the failure mode worth
    // avoiding here: given partial coverage it does not fail loudly, it
    // returns a confident wrong centre, which is strictly worse than min/max
    // because it looks rigorous.
    if (s_fit_n < FIT_KEEP) {
        s_fit_pt[s_fit_n][0] = x; s_fit_pt[s_fit_n][1] = y;
        s_fit_pt[s_fit_n][2] = z; s_fit_n++;
    } else if ((s_fit_seen % 4) == 0) {
        int slot = (s_fit_seen / 4) % FIT_KEEP;
        s_fit_pt[slot][0] = x; s_fit_pt[slot][1] = y; s_fit_pt[slot][2] = z;
    }
    s_fit_seen++;
}

// Gaussian elimination with partial pivoting on the 6x6 normal equations.
static bool solve6(double a[6][6], double b[6], double out[6]) {
    for (int col = 0; col < 6; col++) {
        int piv = col;
        for (int r = col + 1; r < 6; r++)
            if (fabs(a[r][col]) > fabs(a[piv][col])) piv = r;
        if (fabs(a[piv][col]) < 1e-9) return false;   // singular: too little coverage
        if (piv != col) {
            for (int c = 0; c < 6; c++) { double t = a[col][c]; a[col][c] = a[piv][c]; a[piv][c] = t; }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (int r = col + 1; r < 6; r++) {
            double f = a[r][col] / a[col][col];
            if (f == 0.0) continue;
            for (int c = col; c < 6; c++) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int r = 5; r >= 0; r--) {
        double v = b[r];
        for (int c = r + 1; c < 6; c++) v -= a[r][c] * out[c];
        out[r] = v / a[r][r];
    }
    return true;
}

// Returns true and fills centre/scale only if the fit is trustworthy.
// *why is set on every path, because "rejected" alone cannot be acted on:
// thin coverage wants another sweep, a bad residual wants the fallback, and
// an implausible mean wants the sensor checked. Same reason bmm150_init's
// single boolean cost three rebuilds to unpick.
static bool fit_solve(float centre[3], float scale[3], float *residual_out,
                      const char **why) {
    *why = "ok";
    *residual_out = -1.0f;
    if (s_fit_n < 32) { *why = "too few points"; return false; }

    double a[6][6], b[6], p[6];
    for (int i = 0; i < 6; i++) {
        b[i] = s_atb[i];
        for (int j = 0; j < 6; j++) a[i][j] = s_ata[i][j];
    }
    if (!solve6(a, b, p)) { *why = "singular - coverage too thin"; return false; }

    // An ellipsoid needs all three quadratic coefficients to share a sign, but
    // NOT to be positive. Normalising the equation to "= 1" divides through by
    // (1 - sum c_i^2 / r_i^2), which is negative whenever the origin lies
    // outside the surface - and a hard-iron offset large enough to be worth
    // correcting puts it there every time. Demanding positive coefficients
    // rejects exactly the calibrations this code exists to handle.
    // Mixed signs are the real failure: that is a hyperboloid, which is what
    // least squares returns when the points do not wrap the surface.
    bool all_pos = (p[0] > 0.0 && p[1] > 0.0 && p[2] > 0.0);
    bool all_neg = (p[0] < 0.0 && p[1] < 0.0 && p[2] < 0.0);
    if (!all_pos && !all_neg) {
        *why = "hyperboloid - points do not wrap the surface";
        return false;
    }

    double c[3], k = 1.0;
    for (int i = 0; i < 3; i++) {
        c[i] = -p[3 + i] / (2.0 * p[i]);
        k   += p[i] * c[i] * c[i];
    }

    // k and the coefficients must agree in sign too, or the radii are
    // imaginary - the same degenerate-fit case arriving by a different route.
    double rad[3], mean = 0.0;
    for (int i = 0; i < 3; i++) {
        double r2 = k / p[i];
        if (r2 <= 0.0) { *why = "imaginary radius"; return false; }
        rad[i] = sqrt(r2);
        mean += rad[i] / 3.0;
    }
    if (mean < 5.0 || mean > 200.0) {
        *why = "implausible mean radius";
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (rad[i] < mean / 3.0 || rad[i] > mean * 3.0) {
            *why = "one axis radius far from the others";
            return false;
        }
        centre[i] = (float)c[i];
        scale[i]  = (float)(mean / rad[i]);
    }

    // Score it: with a good fit every kept point lands on the unit sphere once
    // centred and scaled. This is the number that decides whether to trust it.
    double acc = 0.0;
    for (int n = 0; n < s_fit_n; n++) {
        double q = 0.0;
        for (int i = 0; i < 3; i++) {
            double v = ((double)s_fit_pt[n][i] - centre[i]) * scale[i] / mean;
            q += v * v;
        }
        double e = sqrt(q) - 1.0;
        acc += e * e;
    }
    *residual_out = (float)sqrt(acc / s_fit_n);
    if (*residual_out > 0.15f) { *why = "residual too high"; return false; }
    return true;
}

// Serialised form. The normal equations are what actually get refined - they
// are a running sum, so a later sweep adds to them rather than replacing them,
// and the fit improves with every calibration instead of starting over.
// Storing only the finished offsets would make each calibration independent
// and there would be nothing to refine.
static const uint32_t CAL_MAGIC   = 0x43414C32;   // 'CAL2'
static const uint16_t CAL_VERSION = 2;

struct CalBlob {
    uint32_t magic;
    uint16_t version;
    uint16_t fit_n;
    float    offset[3];
    float    scale[3];
    float    residual;
    uint8_t  fitted;
    uint8_t  pad[3];
    double   ata[6][6];
    double   atb[6];
    float    pt[FIT_KEEP][3];
};

static bool s_cal_dirty = false;

size_t compass_cal_blob_size() { return sizeof(CalBlob); }
bool   compass_cal_dirty()     { return s_cal_dirty; }
void   compass_cal_clear_dirty() { s_cal_dirty = false; }

size_t compass_cal_export(void *dst, size_t cap) {
    if (!s_calibrated || cap < sizeof(CalBlob)) return 0;
    CalBlob *b = (CalBlob *)dst;
    memset(b, 0, sizeof *b);
    b->magic   = CAL_MAGIC;
    b->version = CAL_VERSION;
    b->fit_n   = (uint16_t)s_fit_n;
    b->fitted  = s_fitted ? 1 : 0;
    for (int i = 0; i < 3; i++) { b->offset[i] = s_offset[i]; b->scale[i] = s_scale[i]; }
    memcpy(b->ata, s_ata, sizeof s_ata);
    memcpy(b->atb, s_atb, sizeof s_atb);
    memcpy(b->pt,  s_fit_pt, sizeof s_fit_pt);
    return sizeof(CalBlob);
}

bool compass_cal_import(const void *src, size_t len) {
    if (!s_ok || len != sizeof(CalBlob)) return false;
    const CalBlob *b = (const CalBlob *)src;
    if (b->magic != CAL_MAGIC || b->version != CAL_VERSION) return false;

    // A stored calibration is only as good as the field it was taken in, and
    // nothing on the card records where that was. Sanity-check rather than
    // trust: a gain far from 1 or an offset larger than any plausible bias
    // means the file is stale or corrupt, and a bad calibration loaded
    // silently at boot is worse than none - it produces confident headings.
    for (int i = 0; i < 3; i++) {
        if (!(b->scale[i] > 0.2f && b->scale[i] < 5.0f))   return false;
        if (!(fabsf(b->offset[i]) < 1000.0f))              return false;
    }

    for (int i = 0; i < 3; i++) { s_offset[i] = b->offset[i]; s_scale[i] = b->scale[i]; }
    memcpy(s_ata, b->ata, sizeof s_ata);
    memcpy(s_atb, b->atb, sizeof s_atb);
    memcpy(s_fit_pt, b->pt, sizeof s_fit_pt);
    s_fit_n      = b->fit_n > FIT_KEEP ? FIT_KEEP : b->fit_n;
    s_fit_seen   = s_fit_n;
    s_fitted     = b->fitted != 0;
    s_calibrated = true;
    s_cal_dirty  = false;

    Serial.printf("compass: restored calibration, offsets %+.1f %+.1f %+.1f uT, "
                  "gains %.2f %.2f %.2f (%s, %d stored points)\n",
                  s_offset[0], s_offset[1], s_offset[2],
                  s_scale[0], s_scale[1], s_scale[2],
                  s_fitted ? "ellipsoid" : "minmax", s_fit_n);
    return true;
}

// Refining keeps the accumulated normal equations and adds to them. Old data
// is halved first so a sweep taken somewhere better outweighs one taken next
// to a desk leg, without discarding the earlier coverage entirely - which is
// the whole point, since no single tumble covers the sphere well.
static void fit_decay(double factor) {
    for (int i = 0; i < 6; i++) {
        s_atb[i] *= factor;
        for (int j = 0; j < 6; j++) s_ata[i][j] *= factor;
    }
}

void compass_calibrate_start_refine() {
    if (!s_ok) return;
    if (!s_calibrated) { compass_calibrate_start(); return; }

    // min/max restarts from scratch: its extremes are per-sweep by definition
    // and carrying them over would report 100% coverage before the device has
    // been moved at all.
    for (int i = 0; i < 3; i++) {
        s_cal_min[i] = 1e9f; s_cal_max[i] = -1e9f;
        s_prev_offset[i] = s_offset[i];
        s_prev_scale[i]  = s_scale[i];
    }
    s_prev_valid = true;
    fit_decay(0.5);
    s_cal_refine = true;
    s_cal_on = true;
    Serial.printf("compass: refining existing calibration (%d points carried "
                  "over at half weight) - turn through every orientation\n",
                  s_fit_n);
}

void compass_calibrate_start() {
    if (!s_ok) return;
    for (int i = 0; i < 3; i++) { s_cal_min[i] = 1e9f; s_cal_max[i] = -1e9f; }
    fit_reset();
    s_cal_refine = false;
    s_cal_on = true;
    Serial.println("compass: calibrating - turn the device slowly through "
                   "every orientation, including upside down");
}

void compass_calibrate_cancel() {
    if (!s_cal_on) return;
    s_cal_on = false;
    Serial.println("compass: calibration cancelled, previous offsets kept");
}

bool compass_calibrating() { return s_cal_on; }

int compass_calibrate_progress() {
    if (!s_cal_on) return 0;
    float worst = 1.0f;
    for (int i = 0; i < 3; i++) {
        if (s_cal_max[i] < s_cal_min[i]) return 0;      // nothing sampled yet
        float f = (s_cal_max[i] - s_cal_min[i]) / CAL_RANGE_TARGET;
        if (f > 1.0f) f = 1.0f;
        if (f < worst) worst = f;
    }
    return (int)(worst * 100.0f);
}

static void calibrate_finish() {
    s_cal_on = false;

    // Try the ellipsoid first; keep min/max as the fallback rather than the
    // replacement. The fit is better when it works and worse when it does not,
    // so it has to earn its use on every calibration, not once.
    float fc[3], fs[3], resid = 0.0f;
    const char *why = "";
    if (fit_solve(fc, fs, &resid, &why)) {
        // A refine has to beat what it replaces, not merely converge. Residual
        // alone does not catch this: a sweep taken in a distorted field fits
        // its own ellipsoid tidily and still produces gains far from 1.
        if (s_cal_refine && s_prev_valid) {
            float now = gain_spread(fs), before = gain_spread(s_prev_scale);
            if (now > 0.15f && now > before) {
                Serial.printf("compass: refine rejected - gains %.2f %.2f %.2f "
                              "spread %.0f%% vs %.0f%% before. That is a "
                              "distorted field, not better coverage; keeping "
                              "the previous calibration\n",
                              fs[0], fs[1], fs[2], now * 100.0f, before * 100.0f);
                for (int i = 0; i < 3; i++) {
                    s_offset[i] = s_prev_offset[i];
                    s_scale[i]  = s_prev_scale[i];
                }
                // Drop the contaminated samples rather than leaving them to
                // poison the next refine as well.
                fit_reset();
                return;
            }
        }
        for (int i = 0; i < 3; i++) { s_offset[i] = fc[i]; s_scale[i] = fs[i]; }
        s_fitted     = true;
        s_calibrated = true;
        s_cal_dirty  = true;
        Serial.printf("compass: ellipsoid fit from %d points, offsets "
                      "%+.1f %+.1f %+.1f uT, gains %.2f %.2f %.2f, "
                      "residual %.1f%%\n",
                      s_fit_n, s_offset[0], s_offset[1], s_offset[2],
                      s_scale[0], s_scale[1], s_scale[2], resid * 100.0f);
        Serial.println("compass: |B| should now hold steady as you turn - if "
                       "it still swings, the distortion is not axis-aligned");
        return;
    }

    // A refine that does not converge must not demote a calibration that was
    // already good. Falling through to min/max here would replace a working
    // ellipsoid with a worse model built from one partial sweep.
    if (s_cal_refine && s_calibrated) {
        Serial.printf("compass: refine rejected (%s) - keeping the previous "
                      "calibration\n", why);
        return;
    }

    for (int i = 0; i < 3; i++) s_scale[i] = 1.0f;
    s_fitted = false;
    // The three sampled radii are printed alongside because they are what the
    // fit and the min/max path disagree about, and seeing them is the fastest
    // way to tell a missed axis from real distortion.
    Serial.printf("compass: ellipsoid fit rejected (%s) from %d points%s",
                  why, s_fit_n, "");
    if (resid >= 0.0f) Serial.printf(", residual %.1f%%", resid * 100.0f);
    Serial.printf("\n  raw ranges x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f uT\n",
                  s_cal_min[0], s_cal_max[0], s_cal_min[1], s_cal_max[1],
                  s_cal_min[2], s_cal_max[2]);
    Serial.println("compass: falling back to min/max, which cannot correct "
                   "stretch");

    float rad[3], mean = 0.0f;
    for (int i = 0; i < 3; i++) {
        s_offset[i] = (s_cal_max[i] + s_cal_min[i]) / 2.0f;
        rad[i]      = (s_cal_max[i] - s_cal_min[i]) / 2.0f;
        mean += rad[i] / 3.0f;
    }
    s_calibrated = true;

    s_cal_dirty = true;
    Serial.printf("compass: calibrated, offsets %+.1f %+.1f %+.1f uT, "
                  "mean radius %.1f uT\n",
                  s_offset[0], s_offset[1], s_offset[2], mean);

    // All three radii measure the same field, so they should agree. When they
    // do not, min/max calibration cannot say why: an axis that never reached
    // its extremes shrinks its own radius, and so does genuine soft-iron
    // distortion from nearby steel. Both look identical from here, and only
    // the first is fixed by tumbling again - so this reports the disagreement
    // rather than claiming to know which it is.
    float spread = 0.0f;
    for (int i = 0; i < 3; i++) {
        float d = fabsf(rad[i] - mean) / (mean > 0.1f ? mean : 0.1f);
        if (d > spread) spread = d;
    }
    if (spread > 0.15f)
        Serial.printf("compass: axis radii differ by %.0f%% - either an axis "
                      "missed its extremes, or soft-iron distortion, which "
                      "this correction cannot remove\n", spread * 100.0f);
}

// ---- position and declination --------------------------------------------
//
// The map is drawn in true north; the magnetometer reports magnetic north.
// The difference reaches 20 degrees over the continental US and far more at
// high latitudes, so ignoring it puts the needle visibly out of agreement
// with the streets underneath it.
//
// A full world magnetic model is several hundred KB of coefficients that
// expire on a five-year cycle, which is a poor trade for a footer widget. A
// tilted-dipole approximation is a few lines and is typically within a couple
// of degrees across the mid-latitudes - well inside what a hand-held compass
// can be trusted for anyway, and honest about being an approximation.
//
// src: the geomagnetic dipole's north pole, ~86.5N 164.0E (IGRF epoch 2025).
static const double MAG_POLE_LAT = 86.5, MAG_POLE_LON = 164.0;

void compass_set_position(double lat, double lon) {
    // Without this the declination line prints on every boot whether or not a
    // magnetometer exists, because it is pure arithmetic on the GNSS fix. In a
    // log where the compass has just failed, a plausible "declination -4.3 deg"
    // three lines later reads as evidence the compass is alive.
    if (!s_ok) return;

    static double last_lat = 1e9, last_lon = 1e9;
    // Declination changes on the scale of degrees per hundred kilometres, so
    // recomputing for every fix is arithmetic nobody reads.
    if (fabs(lat - last_lat) < 0.25 && fabs(lon - last_lon) < 0.25) return;
    last_lat = lat; last_lon = lon;

    const double D = M_PI / 180.0;
    double plat = MAG_POLE_LAT * D, plon = MAG_POLE_LON * D;
    double slat = lat * D,          slon = lon * D;
    double dlon = plon - slon;

    // Bearing from here to the dipole pole. True north is 0; that bearing is
    // where a compass needle points, so its negation is the correction.
    double y = sin(dlon) * cos(plat);
    double x = cos(slat) * sin(plat) - sin(slat) * cos(plat) * cos(dlon);
    double bearing = atan2(y, x) / D;

    s_declination = (float)bearing;
    Serial.printf("compass: declination %+.1f deg at %.3f,%.3f (dipole "
                  "approximation)\n", s_declination, lat, lon);
}

// ---- sampling -------------------------------------------------------------
static float tilt_compensated_heading(float mx, float my, float mz,
                                      float ax, float ay, float az) {
    // Flat-earth atan2(my, mx) is only correct while the device is level. Off
    // level, gravity leaks the vertical field component into the horizontal
    // axes and the reading swings with tilt rather than with heading - which
    // is the single most common "my compass is wrong" report against a sensor
    // working exactly as specified.
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    // Kept for the dial. A ball needs the same two angles the heading already
    // computes, so publishing them here costs nothing and avoids a second,
    // separately-wrong copy of the same trigonometry in the drawing code.
    s_roll  = roll  * 180.0f / (float)M_PI;
    s_pitch = pitch * 180.0f / (float)M_PI;

    float xh = mx * cosf(pitch) + mz * sinf(pitch);
    float yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll)
             - mz * sinf(roll) * cosf(pitch);

    float h = atan2f(yh, xh) * 180.0f / (float)M_PI + s_declination;
    while (h < 0.0f)     h += 360.0f;
    while (h >= 360.0f)  h -= 360.0f;
    return h;
}

void compass_update() {
    if (!s_ok) return;

    // The internal I2C bus belongs to loop()'s task. A read from the render
    // worker would interleave with the fuel gauge and the touch controller,
    // and the symptom would surface over there rather than here.
    if (s_owner && s_owner != xTaskGetCurrentTaskHandle()) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            Serial.println("compass: refusing an off-task read - the internal "
                           "I2C bus belongs to loop()");
        }
        return;
    }

    // 10 Hz. The BMM150 is configured for 100 Hz and the dial repaints at
    // most 15 times a second, so this is well clear of both while leaving the
    // bus alone the rest of the time.
    static uint32_t last = 0;
    if (millis() - last < 100) return;
    last = millis();

    struct bmm150_mag_data md;
    if (bmm150_read_mag_data(&md, &s_bmm) != BMM150_OK) return;

    // BMM150_USE_FLOATING_POINT is not defined, so Bosch's driver returns
    // fixed-point microtesla in int16_t, already corrected against the factory
    // trim values in the part's NVM. No further scaling.
    float mx = (float)md.x, my = (float)md.y, mz = (float)md.z;

    if (s_cal_on) {
        float v[3] = { mx, my, mz };
        for (int i = 0; i < 3; i++) {
            if (v[i] < s_cal_min[i]) s_cal_min[i] = v[i];
            if (v[i] > s_cal_max[i]) s_cal_max[i] = v[i];
        }
        fit_accumulate(mx, my, mz);
        if (compass_calibrate_progress() >= 100) calibrate_finish();
        snprintf(s_status, sizeof s_status, "calibrating %d%%",
                 compass_calibrate_progress());
        // Deliberately no heading while calibrating: the offsets are mid-flight
        // and anything derived from them would be worse than nothing.
        s_heading = -1.0f;
        return;
    }

    // Centre, then scale. s_scale is all-ones unless an ellipsoid fit was
    // accepted, so this is a no-op on the min/max path.
    mx = (mx - s_offset[0]) * s_scale[0];
    my = (my - s_offset[1]) * s_scale[1];
    mz = (mz - s_offset[2]) * s_scale[2];
    s_field = sqrtf(mx * mx + my * my + mz * mz);

    // Accelerometer straight out of the data registers rather than through
    // bmi2_get_sensor_data(), matching what the source project does. The
    // units cancel in the roll/pitch atan2s below - only the ratios matter -
    // so raw counts are exactly as good here as scaled g, and this avoids a
    // second Bosch call path that would need its own verification.
    //
    // src: BMI270 datasheet, DATA_8..DATA_13 at 0x0C, X/Y/Z little-endian.
    uint8_t raw[6];
    if (!M5.In_I2C.readRegister(BMI270_M135_ADDR, 0x0C, raw, sizeof raw, I2C_FREQ))
        return;
    float ax = (float)(int16_t)(raw[1] << 8 | raw[0]);
    float ay = (float)(int16_t)(raw[3] << 8 | raw[2]);
    float az = (float)(int16_t)(raw[5] << 8 | raw[4]);

    // Axis remap, magnetometer -> accelerometer frame.
    //
    // The BMM150 sits on the M135 module and the accelerometer is inside the
    // BMI270 next to it; the two parts are not mounted in the same
    // orientation, so tilt_compensated_heading was mixing axes that do not
    // correspond. Measured, in a field confirmed clean with phones and other
    // magnets well clear: flat, screen up, antenna connector true north reads
    // 179 where 0 is expected; held at 45 degrees facing south reads 7-9
    // where 180 is expected. A consistent 180 in two independent postures,
    // on top of the 90 already known, gives (x, y) <- (-y, x).
    //
    // An earlier attempt used (y, -x), from a single reading of 91 taken with
    // a phone beside the device. The two candidate remaps differ by exactly
    // 180, and roughly 50 uT of contamination was enough to make the wrong
    // one look right - hence two postures, and a clean field first.
    //
    // Deliberately after the offsets and gains rather than before. The stored
    // calibration was fitted in the raw magnetometer frame, so remapping
    // upstream of the correction would apply Y's hard-iron offset to X and
    // silently invalidate every /compasscal.bin written so far.
    float rx = -my;
    float ry =  mx;
    float rz =  mz;

    s_heading = tilt_compensated_heading(rx, ry, rz, ax, ay, az);

    if (!s_calibrated)
        snprintf(s_status, sizeof s_status, "uncalibrated, |B| %.0f uT", s_field);
    else
        snprintf(s_status, sizeof s_status, "|B| %.0f uT%s", s_field,
                 s_fitted ? "" : " (minmax)");
}

float compass_heading()  { return s_calibrated ? s_heading : -1.0f; }
float compass_roll()     { return s_roll; }
float compass_pitch()    { return s_pitch; }
float compass_field_ut() { return s_field; }
const char *compass_status() { return s_status; }

#endif // COMPASS_HAVE_BOSCH

// ---- shared, driver-independent ------------------------------------------
const char *compass_label(float deg) {
    if (deg < 0.0f) return "--";
    static const char *names[16] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    };
    int i = (int)((deg + 11.25f) / 22.5f) % 16;
    if (i < 0) i += 16;
    return names[i];
}
