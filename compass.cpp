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
void  compass_set_position(double, double) { }
void  compass_calibrate_start()     { }
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
static char  s_status[64] = "not started";

// Hard-iron offset, in microtesla, subtracted from every reading. Zero until
// calibrated, which is honest rather than useful: an uncalibrated heading on
// this hardware is wrong by up to a quarter turn, so compass_ok() stays true
// but the status line says so and the UI can decline to trust it.
static float s_offset[3] = { 0.0f, 0.0f, 0.0f };
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
static BMM150_INTF_RET_TYPE aux_read(uint8_t reg, uint8_t *data,
                                     uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    return bmi2_read_aux_man_mode(reg, data, len, &s_bmi);
}

static BMM150_INTF_RET_TYPE aux_write(uint8_t reg, const uint8_t *data,
                                      uint32_t len, void *intf_ptr) {
    (void)intf_ptr;
    return bmi2_write_aux_man_mode(reg, data, len, &s_bmi);
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
    s_bmi.chip_id         = BMI270_M135_ADDR;
    s_bmi.read            = bmi_read;
    s_bmi.write           = bmi_write;
    s_bmi.delay_us        = bmi_delay_us;
    s_bmi.intf            = BMI2_I2C_INTF;
    s_bmi.intf_ptr        = nullptr;      // the address is a compile-time constant here
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

    uint8_t sens[2] = { BMI2_ACCEL, BMI2_AUX };
    if (bmi270_sensor_enable(sens, 2, &s_bmi) != BMI2_OK)
        Serial.println("compass: sensor_enable failed (continuing)");

    memset(&s_bmm, 0, sizeof s_bmm);
    s_bmm.read     = aux_read;
    s_bmm.write    = aux_write;
    s_bmm.delay_us = bmi_delay_us;
    s_bmm.intf     = BMM150_I2C_INTF;
    s_bmm.intf_ptr = nullptr;
    s_bmm.chip_id  = BMM150_DEFAULT_I2C_ADDRESS;

    // The BMM150 boots into suspend and answers nothing - not even its chip
    // ID - until brought out of it, so a failure here is as likely to be the
    // aux path above as the part itself.
    if (bmm150_init(&s_bmm) != BMM150_OK) {
        Serial.println("compass: bmm150_init failed - the magnetometer is "
                       "behind the BMI270's aux bus, so this usually means "
                       "the aux config above did not take");
        snprintf(s_status, sizeof s_status, "no magnetometer");
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
static float s_cal_min[3], s_cal_max[3];

// Enough range on an axis to call it swept. The earth's field is 25-65 uT, so
// a fully turned axis spans twice that; 40 uT of range is a comfortable
// fraction of the smallest case without demanding a perfect tumble.
static const float CAL_RANGE_TARGET = 40.0f;

void compass_calibrate_start() {
    if (!s_ok) return;
    for (int i = 0; i < 3; i++) { s_cal_min[i] = 1e9f; s_cal_max[i] = -1e9f; }
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
    float rad[3], mean = 0.0f;
    for (int i = 0; i < 3; i++) {
        s_offset[i] = (s_cal_max[i] + s_cal_min[i]) / 2.0f;
        rad[i]      = (s_cal_max[i] - s_cal_min[i]) / 2.0f;
        mean += rad[i] / 3.0f;
    }
    s_calibrated = true;

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
        if (compass_calibrate_progress() >= 100) calibrate_finish();
        snprintf(s_status, sizeof s_status, "calibrating %d%%",
                 compass_calibrate_progress());
        // Deliberately no heading while calibrating: the offsets are mid-flight
        // and anything derived from them would be worse than nothing.
        s_heading = -1.0f;
        return;
    }

    mx -= s_offset[0]; my -= s_offset[1]; mz -= s_offset[2];
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

    s_heading = tilt_compensated_heading(mx, my, mz, ax, ay, az);

    if (!s_calibrated)
        snprintf(s_status, sizeof s_status, "uncalibrated, |B| %.0f uT", s_field);
    else
        snprintf(s_status, sizeof s_status, "|B| %.0f uT", s_field);
}

float compass_heading()  { return s_calibrated ? s_heading : -1.0f; }
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
