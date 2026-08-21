# ESP-IDF project definition.
#
# arduino-cli ignores this file, along with sdkconfig.defaults, main/ and
# components/ - it compiles the sketch folder and src/ only. So none of this
# disturbs the Arduino build.
#
# Build:
#   idf.py build flash monitor
#
# Status: display and wifi both work on ESP-IDF v5.5.5. Both were the same
# bug - see the ARDUINO section below and DISPLAY_IDF_NOTES.md. ESP-IDF v6 is
# rejected outright, just above; M5Unified does not compile against it.
#
# Remaining: panel detection fails on the occasional boot immediately after a
# flash write, on this build and the Arduino one alike. panelBegin() restarts
# once. DISPLAY_IDF_NOTES.md has why the two obvious retries cannot work.
#
# There are no manual setup steps. Everything happens below, before project():
#   - sunset is cloned into components/sunset/sunset
#   - the m5stack_tab5 board variant is copied into the managed arduino-esp32
#     component, which ships only generic target variants
#   - m5gfx and m5unified are patched to compile with ARDUINO defined, which is
#     what stopped the display coming up
#   - sdkconfig.defaults is generated from the configuration arduino-cli uses
#     for this board and chip revision, plus a short list of overrides
#
# On a completely fresh tree the first configure cannot install the variant,
# because the component manager has not fetched arduino-esp32 yet. That build
# still completes, using the generic variant and without a working SD card, and
# says so. The next one picks up the variant.

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ---------------------------------------------------------------------------
# ESP-IDF v5 only. Stop here rather than 128 compile errors deep.
#
# M5Unified does not build on v6 - not this pin, not master as of 0.2.19. The
# new I2S driver dropped the port enum:
#
#   v5.5  components/esp_driver_i2s/include/driver/i2s_types.h
#           typedef enum { I2S_NUM_0 = 0, ... } i2s_port_t;
#   v6.x  same file
#           #define I2S_NUM_0  0
#
# and Speaker_Class.hpp / Mic_Class.hpp both declare
#
#   i2s_port_t i2s_port = i2s_port_t::I2S_NUM_0;
#
# which on v6 is an unknown type initialised from a macro that expands to a
# bare 0 after a scope operator. `SOC_I2S_NUM` is gone too. Every member that
# follows then fails to parse, so the errors arrive as a wall of "no member
# named i2s_port" that looks like something else entirely.
#
# This is a source incompatibility in a third-party component. No requirements
# list, no Kconfig option and nothing in this project can fix it - only an
# upstream release, or a fork of M5Unified.
#
# The missing esp_driver_rmt requirement that shows up alongside it is real but
# secondary, and also a v6-only problem: on v5 the `driver` umbrella still
# re-exports the split-out drivers. It is patched below anyway.
if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    message(FATAL_ERROR
        "ESP-IDF ${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR} is not supported by "
        "M5Unified: the v6 I2S driver removed i2s_port_t and SOC_I2S_NUM, which "
        "Speaker_Class and Mic_Class both use. Build with ESP-IDF v5.5.\n"
        "  cd $ENV{IDF_PATH}/..  &&  git clone -b v5.5.5 --recursive "
        "https://github.com/espressif/esp-idf.git esp-idf-v5.5\n"
        "Use a fresh shell for export.sh, and wipe build/, sdkconfig, "
        "managed_components/ and dependencies.lock before rebuilding.\n"
        "See DISPLAY_IDF_NOTES.md.")
endif()

# Two defines the Arduino build gets from boards.txt that an IDF build does not.
#
# ARDUINO_M5STACK_TAB5 comes from build.board=M5STACK_TAB5. The IDF path cannot
# produce it: arduino-esp32 derives its board macro from CONFIG_ARDUINO_VARIANT
# (and appends _DEV), and the packaged component has no m5stack_tab5 variant to
# select - it ships only generic target variants, esp32 through esp32p4.
#
# BOARD_HAS_PSRAM is the PSRAM=enabled menu option from the FQBN.
#
# Set on the build rather than on `main`, so the M5Unified and M5GFX components
# see them too - which is the point, since they are what inspects the board.
# Harmless if nothing tests for either name.
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_M5STACK_TAB5" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "BOARD_HAS_PSRAM" APPEND)

# Tell M5GFX the board instead of relying on autodetect.
#
# M5GFX::init_impl does:
#     board_t b = board_t::board_unknown;
#   #if defined (M5GFX_BOARD)
#     b = M5GFX_BOARD;
#   #endif
#     _board = autodetect(use_reset, b);
#
# and the P4 branch of autodetect gates its Tab5 path on
#     if (board == 0 || board == board_t::board_M5Tab5)
# so passing the board in does not skip bring-up - it just stops detection
# being the thing that has to succeed first.
#
# Under Arduino this is unnecessary: the board arrives via ARDUINO_M5STACK_TAB5
# and detection works, reporting 22/22.
#
# Tested under IDF while the display was down, and it changed nothing - still
# 255/127, still no panel. That was the most useful negative result of the
# investigation. M5GFX_BOARD feeds into
#     _board = autodetect(use_reset, b);
# so if that line ran, _board would be 22 or 0, never 127. The same garbage
# coming back meant init_impl was not executing at all - which pointed away
# from detection and eventually to the ARDUINO layout mismatch patched below.
#
# Kept because it is harmless and correct, and because it removes detection as
# a variable.
idf_build_set_property(COMPILE_DEFINITIONS
                       "M5GFX_BOARD=m5gfx::board_t::board_M5Tab5" APPEND)

# CDCOnBoot=cdc and USBMode=hwcdc from the FQBN.
#
# Without these, Arduino's `Serial` is HardwareSerial on UART0, which leaves
# the chip on GPIO 38/37 - physical pins, not the USB port the monitor is
# attached to. Every Serial.print in this project was going somewhere nobody
# was listening: no boot banner, no "display:" diagnostics, nothing. IDF's own
# ESP_LOG output still arrived over USB-Serial/JTAG, which made the build look
# like it was running silently rather than talking to the wrong pins.
#
# With ARDUINO_USB_CDC_ON_BOOT, `Serial` is HWCDC on the same USB-Serial/JTAG
# peripheral the monitor uses, matching what arduino-cli builds.
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_CDC_ON_BOOT=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "ARDUINO_USB_MODE=1" APPEND)

# idf.py -DMAP_M5_SMOKE_TEST=1 sets a CMake cache variable, not a compile
# definition - CMake even warns that it went unused. Forward it explicitly.
if(MAP_M5_SMOKE_TEST)
    idf_build_set_property(COMPILE_DEFINITIONS "MAP_M5_SMOKE_TEST=1" APPEND)
    message(STATUS "MAP_M5_SMOKE_TEST=1: setup() is the bare M5.begin() probe")
endif()

# ---------------------------------------------------------------------------
# sunset, fetched on first configure.
#
# A plain Arduino library with no idf_component.yml, so the component manager
# cannot get it. components/sunset/CMakeLists.txt wraps it; this clones the
# sources it wraps.
#
# Doing it here rather than telling the user to run git: a fresh checkout of
# this project would otherwise stop at a message, and the failure looks like a
# build error rather than a missing optional step.
set(SUNSET_DIR "${CMAKE_CURRENT_LIST_DIR}/components/sunset/sunset")
if(NOT EXISTS "${SUNSET_DIR}/src/sunset.cpp")
    find_package(Git QUIET)
    if(GIT_FOUND)
        message(STATUS "sunset: cloning buelowp/sunset")
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" clone --depth 1 -q
                    https://github.com/buelowp/sunset.git "${SUNSET_DIR}"
            RESULT_VARIABLE SUNSET_RC
            ERROR_VARIABLE  SUNSET_ERR)
        if(NOT SUNSET_RC EQUAL 0)
            message(FATAL_ERROR
                "sunset: clone failed (${SUNSET_RC}): ${SUNSET_ERR}\n"
                "Clone it by hand and build again:\n"
                "  git clone --depth 1 https://github.com/buelowp/sunset.git "
                "${SUNSET_DIR}")
        endif()
        message(STATUS "sunset: cloned into ${SUNSET_DIR}")
    else()
        message(FATAL_ERROR
            "sunset sources are missing and git was not found.\n"
            "  git clone --depth 1 https://github.com/buelowp/sunset.git "
            "${SUNSET_DIR}")
    endif()
endif()

# ---------------------------------------------------------------------------
# Bosch BMI270 + BMM150 drivers, fetched on first configure.
#
# These are what compass.cpp needs to produce a heading while stationary. GNSS
# course over ground is derived from successive fixes, so it does not exist at
# a standstill - a parked receiver leaves the RMC course field empty. Only a
# magnetometer answers "which way am I facing" when nothing is moving.
#
# Fetched rather than committed, and fetched here rather than by a script the
# user has to remember, for the same two reasons as everything else in this
# file: they are byte-exact copies from a pinned upstream under a different
# licence (BSD-3-Clause), and a missing optional step whose failure mode is a
# silently absent feature is a bad trade against one network call.
#
# Unlike sunset this is NOT fatal when it fails. compass.cpp guards on
# __has_include("bmi270.h") and degrades to GNSS course, so a build behind a
# firewall, or offline, still produces working firmware - just without a
# standstill compass. It says so rather than failing.
#
#   idf.py -DMAP_HAVE_COMPASS=0 build     skip the fetch entirely
#   ./tools/fetch_bosch_drivers.sh --revert
#
# src: the driver set and the reason for vendoring them are from
#      Sudrien/m5tab5_esp_idf_m135_examples; see tools/fetch_bosch_drivers.sh.
if(NOT DEFINED MAP_HAVE_COMPASS)
    set(MAP_HAVE_COMPASS 1)
endif()

set(BOSCH_DIR "${CMAKE_CURRENT_LIST_DIR}/components/bosch_sensortec")
if(MAP_HAVE_COMPASS AND NOT EXISTS "${BOSCH_DIR}/bmi270.c")
    message(STATUS "compass: fetching Bosch BMI270/BMM150 drivers")
    execute_process(
        COMMAND "${CMAKE_CURRENT_LIST_DIR}/tools/fetch_bosch_drivers.sh"
                --keep-build
        # Both of these matter, and neither is tidiness.
        #
        # --keep-build: the script's normal ending is `rm -rf build/`, because
        # a stale build directory reuses old objects and leaves the sensor
        # dead with good sources on disk. That is right when a human runs it
        # from the project root, and fatal here - configure runs *with build/
        # as its working directory*, so deleting it removes cmake's own CWD
        # and the next thing to touch the filesystem fails with "Current
        # working directory cannot be established", pointing at
        # component.cmake rather than at this.
        #
        # WORKING_DIRECTORY: without it the child inherits that same build/
        # CWD. The script only uses absolute paths derived from BASH_SOURCE so
        # it would survive, but nothing should be running out of a directory
        # this project deletes and recreates.
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
        RESULT_VARIABLE BOSCH_RC
        OUTPUT_VARIABLE BOSCH_OUT
        ERROR_VARIABLE  BOSCH_ERR)
    if(NOT BOSCH_RC EQUAL 0)
        # Not fatal, deliberately. The build that follows is a working build
        # without a standstill compass, which is strictly better than no build
        # at all for anyone who does not have the M135 fitted anyway.
        message(WARNING
            "compass: fetching Bosch's drivers failed (${BOSCH_RC}).\n"
            "${BOSCH_ERR}"
            "Building without a magnetometer - the compass dial will fall back "
            "to GNSS course over ground, which only exists while moving.\n"
            "Re-run ./tools/fetch_bosch_drivers.sh when the network is "
            "available, or build with -DMAP_HAVE_COMPASS=0 to stop asking.")
    else()
        message(STATUS "compass: vendored into components/bosch_sensortec")
    endif()
endif()

if(NOT MAP_HAVE_COMPASS AND EXISTS "${BOSCH_DIR}")
    file(REMOVE_RECURSE "${BOSCH_DIR}")
    message(STATUS "compass: vendored drivers removed (MAP_HAVE_COMPASS=0)")
endif()

# ---------------------------------------------------------------------------
# Board variant for the managed arduino-esp32 component.
#
# The component the IDF registry ships carries only generic target variants -
# esp32, esp32c3, ... esp32p4 - and no board variants. The Arduino *package*
# has them, because boards.txt selects one per board.
#
# It is not cosmetic. variants/esp32p4/pins_arduino.h defines
#     BOARD_SDMMC_POWER_CHANNEL 4
#     BOARD_SDMMC_POWER_PIN     45
# so SD_MMC powers the card slot through an LDO channel the Tab5 does not use:
# "ldo: The voltage value 0 is out of the recommended range", then
# sdmmc_init_ocr fails 0x107 and no card is ever found. The m5stack_tab5
# variant omits both, because the slot is powered through the PI4IO expander
# that M5Unified sets up.
#
# Doing this in CMake rather than a script the user has to remember: the copy
# is wiped whenever managed_components is refreshed, and the resulting failure
# ("Include directory .../variants/m5stack_tab5 is not a directory") gives no
# hint about what to re-run.
set(ARDUINO_COMPONENT "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__arduino-esp32")
set(TAB5_VARIANT "${ARDUINO_COMPONENT}/variants/m5stack_tab5")

# Called twice, before and after project(), for the same reason the M5 patch is.
#
# The component manager rewrites managed_components/ during project() whenever
# a dependency resolves, and it re-resolves on every configure that changes
# anything - a new -D on the idf.py command line is enough. That wipes the
# variant this copied in, and the copy above ran before project(), so it cannot
# know.
#
# The failure is silent, which is what makes it worth guarding. arduino-esp32
# falls back to the generic esp32p4 variant, which defines
#     BOARD_SDMMC_POWER_CHANNEL 4
#     BOARD_SDMMC_POWER_PIN     45
# and SD_MMC then builds an sd_pwr_ctrl_ldo_config_t from it (SD_MMC.cpp:267,
# which sets .ldo_chan_id and leaves voltage_mv at zero). The only sign is one
# warning during mount:
#
#     W ldo: The voltage value 0 is out of the recommended range [500, 2700]
#
# The build succeeds, the app runs, and the card may or may not mount depending
# on whether the expander happened to have the rail up already. A one-line
# warning is not enough for something that decides whether storage works.
function(tab5_install_variant OUT_INSTALLED)
    set(_did FALSE)
    if(EXISTS "${ARDUINO_COMPONENT}" AND NOT EXISTS "${TAB5_VARIANT}")
        file(GLOB VARIANT_CANDIDATES
             "$ENV{HOME}/.arduino15/packages/esp32/hardware/esp32/*/variants/m5stack_tab5")
        if(VARIANT_CANDIDATES)
            list(SORT VARIANT_CANDIDATES)
            list(REVERSE VARIANT_CANDIDATES)          # highest core version first
            list(GET VARIANT_CANDIDATES 0 VARIANT_SRC)
            file(COPY "${VARIANT_SRC}" DESTINATION "${ARDUINO_COMPONENT}/variants")
            message(STATUS "arduino variant: installed m5stack_tab5 from ${VARIANT_SRC}")
            set(_did TRUE)
        else()
            message(WARNING
                "CONFIG_ARDUINO_VARIANT is m5stack_tab5 but no such variant was found "
                "under ~/.arduino15/packages/esp32/hardware/esp32/*/variants/. "
                "Install the esp32 core through the Arduino IDE or arduino-cli, or "
                "set CONFIG_ARDUINO_VARIANT=\"esp32p4\" - the SD card slot will be "
                "powered through an LDO channel this board does not use.")
        endif()
    endif()
    set(${OUT_INSTALLED} ${_did} PARENT_SCOPE)
endfunction()

if(NOT EXISTS "${ARDUINO_COMPONENT}")
    # First configure on a fresh tree: the component manager has not run yet,
    # so there is nothing to copy into. It downloads during project() below,
    # and the second pass installs the variant then.
    message(STATUS
        "arduino variant: managed_components not present yet - components will "
        "be fetched during this configure, and the variant installed afterwards.")
endif()

tab5_install_variant(TAB5_VARIANT_INSTALLED_EARLY)

# ---------------------------------------------------------------------------
# exFAT and GPT in FatFs, on by default.
#
#   idf.py -DMAP_FATFS_EXFAT=0 build      turn off
#   idf.py -DMAP_FATFS_EXFAT=1 build      turn back on
#
# ESP-IDF ships FatFs with exFAT compiled out and offers no menuconfig option,
# so the only way in is a local copy of the component with src/ffconf.h edited.
# A component in components/ shadows the one in IDF, so nothing in the IDF tree
# is touched and other projects are unaffected.
#
# Two formats are at stake, not one. A drive over 32 GB ships exFAT from the
# factory and often GPT with it, and FatFs reads GPT partition tables in
# find_volume() inside #if FF_LBA64 - which is one of the three switches this
# sets. So this fixes both, and neither can be had alone: ffconf.h's own note
# is "To enable the 64-bit LBA, also exFAT needs to be enabled".
#
# This is on by default, so read the first of these rather than discovering it
# later:
#
#   - Espressif's stated reason for shipping exFAT off is licensing, not
#     technical. Their docs ask you to review it for your product and markets,
#     and this project turning it on by default does not answer that question
#     for you. Microsoft published the specification and added it to the Open
#     Invention Network Linux System Definition in 2019, which is why exFAT is
#     in the mainline Linux kernel; that is context, not legal advice. Build
#     with -DMAP_FATFS_EXFAT=0 if the answer for your case is no.
#   - Even with GPT support, FatFs only looks at partitions whose type GUID is
#     Microsoft Basic Data. FAT32 in a partition typed as Linux filesystem is
#     invisible - right filesystem, wrong label.
#   - Only the first FAT volume on a drive is mounted, either way.
#
# Formatting the drive FAT32 on an MBR label avoids all of this and costs
# nothing if you control the drives.
#
# components/fatfs/ IS gitignored, which is the opposite of what the example's
# README advises - and the difference is the point of doing this in CMake.
#
# That advice is right for a script: a copy nobody re-runs the script for has
# to be committed, or a fresh clone silently builds FAT-only. But a committed
# copy is a fork of one of IDF's components, pinned to whichever IDF version
# happened to be installed the day it was made. Update IDF and the tree still
# compiles the old FatFs against the new headers and the new Kconfig, and
# nothing says so.
#
# Since this derives the copy automatically, committing it buys nothing and
# costs that. The stamp below records the IDF version and the settings the copy
# was made with; anything else and it is thrown away and re-derived. So the
# vendored FatFs always matches the IDF in use, and the only thing in git is
# the three-line patch that makes it, in tools/patch_ffconf.py.
#
# This is the same shape as the m5gfx/m5unified patch further down: take what
# the toolchain provides, change the one thing that needs changing, and do it
# every time rather than once.
#
# src: ported from tools/enable_exfat.sh in m5tab5_esp_idf_usb_host_example.
#      Done here rather than as a script because a script has to be remembered,
#      and the failure mode for forgetting it is a drive that does not mount.
set(TAB5_FATFS "${CMAKE_CURRENT_LIST_DIR}/components/fatfs")

# Default on. A drive bought today is exFAT on GPT, and a build that cannot
# read it is a build that cannot read the storage this project now prefers.
# The trade is a vendored component in the tree and a licensing question to
# answer - both stated above, neither silent.
if(NOT DEFINED MAP_FATFS_EXFAT)
    set(MAP_FATFS_EXFAT 1)
endif()

# What the current copy would have to say to be left alone. Any change here -
# a different IDF, a different setting - invalidates it.
set(TAB5_FATFS_STAMP "${TAB5_FATFS}/.tab5-exfat-stamp")
set(TAB5_FATFS_WANT
    "idf=${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH} exfat=1 lba64=1 trim=0")

set(TAB5_FATFS_STALE FALSE)
if(MAP_FATFS_EXFAT AND EXISTS "${TAB5_FATFS}")
    set(TAB5_FATFS_HAVE "")
    if(EXISTS "${TAB5_FATFS_STAMP}")
        file(READ "${TAB5_FATFS_STAMP}" TAB5_FATFS_HAVE)
        string(STRIP "${TAB5_FATFS_HAVE}" TAB5_FATFS_HAVE)
    endif()
    if(NOT TAB5_FATFS_HAVE STREQUAL TAB5_FATFS_WANT)
        message(STATUS "fatfs: vendored copy is stale (${TAB5_FATFS_HAVE}) - re-deriving")
        file(REMOVE_RECURSE "${TAB5_FATFS}")
        set(TAB5_FATFS_STALE TRUE)
    endif()
endif()

if(MAP_FATFS_EXFAT AND NOT EXISTS "${TAB5_FATFS}")
    set(TAB5_FATFS_SRC "$ENV{IDF_PATH}/components/fatfs")
    if(NOT EXISTS "${TAB5_FATFS_SRC}/src/ffconf.h")
        message(FATAL_ERROR
            "MAP_FATFS_EXFAT is set but there is no FatFs component at "
            "${TAB5_FATFS_SRC}. Is IDF_PATH right, and is export.sh sourced?")
    endif()

    # NO_SOURCE_PERMISSIONS because the copy has to be writable to be patched,
    # and an IDF installed read-only - a system package, a shared toolchain -
    # would otherwise produce a copy nothing can edit.
    file(COPY "${TAB5_FATFS_SRC}" DESTINATION "${CMAKE_CURRENT_LIST_DIR}/components"
         NO_SOURCE_PERMISSIONS)

    find_program(TAB5_PYTHON NAMES python3 python)
    if(NOT TAB5_PYTHON)
        file(REMOVE_RECURSE "${TAB5_FATFS}")
        message(FATAL_ERROR
            "MAP_FATFS_EXFAT needs python to patch ffconf.h and none was found "
            "on PATH. Source ESP-IDF's export.sh, which puts its own on there.")
    endif()

    execute_process(
        COMMAND "${TAB5_PYTHON}" "${CMAKE_CURRENT_LIST_DIR}/tools/patch_ffconf.py"
                "${TAB5_FATFS}/src/ffconf.h"
        RESULT_VARIABLE TAB5_FFCONF_RC
        OUTPUT_VARIABLE TAB5_FFCONF_OUT
        ERROR_VARIABLE  TAB5_FFCONF_ERR)

    if(NOT TAB5_FFCONF_RC EQUAL 0)
        # Leave nothing half-patched behind. A vendored fatfs that shadows
        # IDF's but was not successfully edited is strictly worse than none:
        # it builds, it is FAT-only, and it looks like it should not be.
        file(REMOVE_RECURSE "${TAB5_FATFS}")
        message(FATAL_ERROR "patching ffconf.h failed:\n${TAB5_FFCONF_ERR}")
    endif()

    string(STRIP "${TAB5_FFCONF_OUT}" TAB5_FFCONF_OUT)
    message(STATUS "${TAB5_FFCONF_OUT}")

    # Written last, so a copy interrupted halfway has no stamp and is
    # re-derived rather than trusted.
    file(WRITE "${TAB5_FATFS_STAMP}" "${TAB5_FATFS_WANT}\n")
    message(STATUS "fatfs: vendored from IDF "
                   "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

    # A re-derive for a new IDF does not need the sdkconfig dance: the
    # component already existed, so its Kconfig symbols are already in there.
    if(NOT TAB5_FATFS_STALE)
        set(TAB5_FATFS_CHANGED TRUE)
    endif()
endif()

if(NOT MAP_FATFS_EXFAT AND EXISTS "${TAB5_FATFS}")
    file(REMOVE_RECURSE "${TAB5_FATFS}")
    message(STATUS "fatfs: vendored copy removed, back to stock FAT-only")
    set(TAB5_FATFS_CHANGED TRUE)
endif()

if(TAB5_FATFS_CHANGED)
    # sdkconfig has to go, and this is not housekeeping.
    #
    # A sdkconfig generated before components/fatfs/ existed carries none of
    # that component's Kconfig symbols, so the build compiles the vendored
    # ffconf.h against a stale config header. It fails as
    # "'CONFIG_FATFS_USE_LABEL' undeclared", pointing at a line of Espressif's
    # code, with no hint that the real problem is a config file from thirty
    # seconds ago. The same applies in reverse when reverting.
    #
    # sdkconfig.defaults regenerates it, and pins CONFIG_IDF_TARGET, so the
    # target survives this.
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
    message(FATAL_ERROR
        "FatFs changed and sdkconfig has been removed so Kconfig is "
        "regenerated from sdkconfig.defaults.\n"
        "Run the same command again - it will succeed.\n"
        "If anything still refuses to build, delete build/ as well: a "
        "config header cached from before this change compiles the new "
        "ffconf.h against the old symbols.")
endif()


# ---------------------------------------------------------------------------
# Compile M5GFX and M5Unified with ARDUINO defined, as `main` already is.
#
# THIS IS THE FIX FOR THE HEADLESS DISPLAY. See DISPLAY_IDF_NOTES.md.
#
# LovyanGFX changes the layout of every one of its objects depending on a
# macro (src/lgfx/v1/LGFXBase.hpp):
#
#     class LGFXBase
#     #if defined (ARDUINO)
#       : public Print
#     #endif
#
# so with ARDUINO defined, LGFXBase - and therefore M5GFX, and therefore the
# M5 object - gains a base class, and every member sits at a different offset.
#
# arduino-esp32 publishes -DARDUINO=10812 as a PUBLIC compile option on its own
# target, which reaches only the components that require it. `main` requires it
# (through idf_component.yml) and so compiles with ARDUINO. The vendored
# CMakeLists.txt of both m5gfx and m5unified ship this line commented out:
#
#     ### If you use arduino-esp32 components, please activate next comment line.
#     # list(APPEND COMPONENT_REQUIRES arduino-esp32)
#
# so neither of them requires it, and both compile *without* ARDUINO. Two
# incompatible layouts of the same class in one binary.
#
# It is not merely a wrong-looking getBoard(). M5Unified::begin() is defined
# inline in M5Unified.hpp, so it is compiled into main's translation unit with
# main's layout, and its first statement is:
#
#     if (_board != m5gfx::board_t::board_unknown) { return; }
#
# Read at the wrong offset, _board is 255. Not zero, so begin() returns
# immediately, before Display.init() - which is why the panel never comes up,
# why no M5GFX log line has ever appeared at any level, why M5GFX_BOARD changed
# nothing, and why 255/127 are not values from the board_t enum.
#
# The Arduino build is unaffected because there is only one definition of
# ARDUINO there, applied to everything.
#
# Uncommenting the vendored line is the vendor's own documented remedy. It is
# done here rather than by hand because the component manager rewrites
# managed_components/ whenever a dependency resolves, which would silently undo
# an edit made in the tree - and undo it back into a headless build.
#
# Called twice: once here, and once after project(). On a fresh tree
# managed_components/ does not exist yet at this point - the component manager
# only runs inside project() - so the first configure of a new checkout would
# otherwise produce a headless binary with no indication why. The second call
# catches that, and the same case where a dependency re-resolves and the
# manager rewrites the tree underneath an already-applied patch.
function(tab5_patch_m5_components OUT_PATCHED)
set(M5_DID_PATCH FALSE)

# Locate the Arduino core component on disk and record its registered name.
#
# The name is not knowable up front: a managed copy is
# espressif__arduino-esp32, a hand-placed one in components/ is named after its
# directory. It was previously resolved inside the injected code from the
# BUILD_COMPONENTS build property, which does not work - see the comment on the
# injected guard below - so it is resolved here, against the filesystem, and
# baked in as a literal.
set(M5_ARDUINO_DIR "")
foreach(CAND "managed_components/espressif__arduino-esp32"
             "components/arduino-esp32"
             "components/arduino")
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${CAND}/CMakeLists.txt")
        set(M5_ARDUINO_DIR "${CMAKE_CURRENT_LIST_DIR}/${CAND}")
        get_filename_component(M5_ARDUINO_NAME "${CAND}" NAME)
        break()
    endif()
endforeach()

foreach(M5_COMPONENT m5stack__m5gfx m5stack__m5unified)
    set(M5_DIR "${CMAKE_CURRENT_LIST_DIR}/managed_components/${M5_COMPONENT}")
    set(M5_CMAKE "${M5_DIR}/CMakeLists.txt")

    # M5GFX's CMakeLists has an "IDF_VERSION_MAJOR GREATER_EQUAL 6" branch that
    # names the split-out esp_driver_* components. M5Unified's does not - it
    # still asks only for `driver` on anything past v5, and on v6 `driver` no
    # longer re-exports them. LED_Strip_Class.hpp includes driver/rmt_types.h
    # unconditionally on IDF >= 5.0, so the build stops with a requirements
    # error naming esp_driver_rmt. Fixing only that one gets the next one.
    #
    # Every name is checked against $IDF_PATH/components before use: naming a
    # component that does not exist is a hard configure error, and this list has
    # to stay safe across the v5.3 to v6.1 range this project supports. On v5
    # they are all already reachable through `driver` and adding them changes
    # nothing.
    set(M5_EXTRA_REQUIRES
"
# M5Unified never got the IDF v6 branch that m5gfx has: on v6 the `driver`
# umbrella it asks for no longer re-exports the split-out drivers, and
# LED_Strip_Class.hpp includes driver/rmt_types.h unconditionally. Applied to
# both components - m5gfx's own v6 branch may be short of one too, and finding
# that out one failed build at a time is expensive.
idf_build_get_property(_tab5_idf_path IDF_PATH)
foreach(_tab5_drv esp_driver_rmt esp_driver_i2s esp_driver_i2c
                  esp_driver_gpio esp_driver_ledc esp_driver_touch_sens
                  esp_adc hal esp_timer)
    if(EXISTS \"\${_tab5_idf_path}/components/\${_tab5_drv}\")
        list(APPEND COMPONENT_REQUIRES \${_tab5_drv})
    endif()
endforeach()
")

    if(EXISTS "${M5_CMAKE}" AND NOT M5_ARDUINO_DIR STREQUAL "")
        file(READ "${M5_CMAKE}" M5_SRC)
        string(FIND "${M5_SRC}" "tab5_map: ARDUINO" M5_PATCHED)
        string(FIND "${M5_SRC}" "register_component()" M5_HAS_REGISTER)
        if(M5_PATCHED EQUAL -1 AND NOT M5_HAS_REGISTER EQUAL -1)
            # Relative, so the patch survives the project directory being moved
            # or copied - an absolute path baked in here would silently stop
            # matching and quietly hand back a headless build.
            file(RELATIVE_PATH M5_ARDUINO_REL "${M5_DIR}" "${M5_ARDUINO_DIR}")

            # Injected ahead of register_component(), not appended: the legacy
            # registration API reads COMPONENT_REQUIRES as it runs, so anything
            # added after that call is ignored - and calling it twice is an
            # error rather than a no-op.
            string(REPLACE "register_component()"
"# --- patched by tab5_map: ARDUINO must be defined here too -------------------
# Without this, this component compiles without ARDUINO while main compiles
# with it, LGFXBase gains a Print base class on one side only, and every member
# of the M5 object is read at the wrong offset from main. M5Unified::begin() is
# inline, sees a non-zero _board, and returns before initialising the display.
#
# The guard is a filesystem check rather than a BUILD_COMPONENTS lookup, which
# is what an earlier version of this patch used and is a trap. ESP-IDF collects
# REQUIRES in an early pass that runs each component's CMakeLists in a separate
# cmake -P script (tools/cmake/scripts/component_get_requirements.cmake), and
# BUILD_COMPONENTS is not set until after that pass has finished. Reading it
# there yields an empty list, the append never happens, and the whole patch
# becomes a silent no-op that still prints as if it applied.
if(EXISTS \"\${CMAKE_CURRENT_LIST_DIR}/${M5_ARDUINO_REL}/CMakeLists.txt\")
    list(APPEND COMPONENT_REQUIRES ${M5_ARDUINO_NAME})
    message(STATUS \"${M5_COMPONENT}: requiring ${M5_ARDUINO_NAME} for -DARDUINO\")
endif()
${M5_EXTRA_REQUIRES}
register_component()"
                M5_SRC "${M5_SRC}")
            file(WRITE "${M5_CMAKE}" "${M5_SRC}")
            message(STATUS "${M5_COMPONENT}: patched to compile with ARDUINO")
            set(M5_DID_PATCH TRUE)
        endif()
    endif()
endforeach()
set(${OUT_PATCHED} ${M5_DID_PATCH} PARENT_SCOPE)
endfunction()

tab5_patch_m5_components(M5_PATCHED_EARLY)
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/managed_components/m5stack__m5gfx")
    message(STATUS
        "m5gfx: not fetched yet - the ARDUINO patch is applied after the "
        "component manager runs, and this build will stop and ask to be run "
        "again")
endif()

# ---------------------------------------------------------------------------
# Disable esp_hosted's auto-init constructor.
#
# port_esp_hosted_host_init.c does this unconditionally:
#     static void __attribute__((constructor)) esp_hosted_host_init(void)
#     { ESP_ERROR_CHECK(esp_hosted_init()); }
# so the SDIO transport comes up before any application code, and arduino-esp32
# then cannot configure it: esp_hosted_sdio_set_config() returns
# ESP_ERR_NOT_ALLOWED and WiFi.mode(STA) fails.
#
# An earlier attempt patched arduino-esp32 to tolerate that error instead. It
# was worse: hostedInit() went on to call esp_hosted_init() a second time,
# which resets the C6 over its GPIO and re-initialises the bus - and that
# second init fails, fifteen sdmmc_init_ocr retries deep, destroying the link
# the constructor had already brought up correctly.
#
# Removing the constructor lets arduino-esp32 own the sequence exactly as it
# does in the Arduino build, which works on this hardware.
#
# There is now a second and stronger reason to keep this. The C6's power enable
# is WLAN_PWR_EN, on the I/O expander at 0x44 pin P0, and M5Unified drives it
# from Power_Class::begin() - inside M5.begin(). A constructor runs before
# app_main, so it necessarily runs before M5.begin(): whatever it brings up, it
# is bringing up against an unpowered slave. Doing it from WiFi.mode(), after
# setup(), is the only ordering that can work. See DISPLAY_IDF_NOTES.md.
set(HOSTED_CTOR "${CMAKE_CURRENT_LIST_DIR}/managed_components/espressif__esp_hosted/host/port/esp/freertos/src/port_esp_hosted_host_init.c")
if(EXISTS "${HOSTED_CTOR}")
    file(READ "${HOSTED_CTOR}" CTOR_SRC)
    string(FIND "${CTOR_SRC}" "__attribute__((constructor))" CTOR_FOUND)
    if(NOT CTOR_FOUND EQUAL -1)
        string(REPLACE
            "static void __attribute__((constructor)) esp_hosted_host_init(void)"
            "static void __attribute__((unused)) esp_hosted_host_init(void)  // patched: constructor removed, arduino-esp32 initialises the transport"
            CTOR_PATCHED "${CTOR_SRC}")
        string(REPLACE
            "static void __attribute__((destructor)) esp_hosted_host_deinit(void)"
            "static void __attribute__((unused)) esp_hosted_host_deinit(void)  // patched: destructor removed with the constructor"
            CTOR_PATCHED "${CTOR_PATCHED}")
        file(WRITE "${HOSTED_CTOR}" "${CTOR_PATCHED}")
        message(STATUS "esp_hosted: removed the auto-init constructor")
    endif()
endif()

# ---------------------------------------------------------------------------
# sdkconfig.defaults, generated from the configuration arduino-cli uses.
#
# Hand-picking a few dozen options and leaving ~3000 at IDF defaults is what
# produced three separate early-boot memory failures here. This imports the
# whole thing from the reference and appends only deliberate overrides.
#
# The reference is chosen by chip revision: boards.txt maps the Tab5's
# ChipVariant menu to esp32p4_es for chips before v3.00 (360 MHz) and esp32p4
# for v3.00+. This board reports v1.3, so _es.
set(SDKCONFIG_OUT "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
file(GLOB SDKCONFIG_REFS
     "$ENV{HOME}/.arduino15/packages/esp32/tools/esp32p4_es-libs/*/sdkconfig")

if(SDKCONFIG_REFS)
    list(SORT SDKCONFIG_REFS)
    list(REVERSE SDKCONFIG_REFS)
    list(GET SDKCONFIG_REFS 0 SDKCONFIG_REF)

    # Dropped: read-only hardware capabilities, the target and toolchain
    # identity (restated below), the Arduino library-builder settings, and
    # component trees this project does not use.
    set(SDKCONFIG_DROP
        "^CONFIG_(SOC_|IDF_TARGET|IDF_CMAKE|IDF_INIT_VERSION|IDF_TOOLCHAIN|IDF_FIRMWARE_CHIP_ID|LIB_BUILDER_|CAMERA_|ESP_IPA_|SR_WN_|SR_MN_|SR_NSN|SR_VADN|ESP_SR_|NN_|DSP_|UVC_|AFE_|MODEL_IN_|CAM_)")

    # Only ask for m5stack_tab5 once it is actually present. On a fresh tree
    # the component manager has not run yet, so the copy above found nothing to
    # copy into; naming the variant anyway makes arduino-esp32 stop with
    # "variants/m5stack_tab5 is not a directory" and the first build fails
    # outright. Falling back to esp32p4 lets that build finish - without a
    # working SD card, which the message below says plainly - and the next
    # configure picks up the variant and the card with it.
    if(EXISTS "${TAB5_VARIANT}")
        set(VARIANT_SETTING "CONFIG_ARDUINO_VARIANT=\"m5stack_tab5\"\n")
    else()
        set(VARIANT_SETTING "CONFIG_ARDUINO_VARIANT=\"esp32p4\"\n")
        message(STATUS
            "sdkconfig.defaults: m5stack_tab5 variant not present yet, using "
            "esp32p4 for this configure. The SD card will not mount. Run "
            "idf.py build again once components are fetched and it switches "
            "over automatically.")
    endif()

    set(SDKCONFIG_BODY "")
    file(STRINGS "${SDKCONFIG_REF}" SDKCONFIG_LINES)
    foreach(line IN LISTS SDKCONFIG_LINES)
        if(line MATCHES "^CONFIG_" AND NOT line MATCHES "${SDKCONFIG_DROP}")
            string(APPEND SDKCONFIG_BODY "${line}\n")
        endif()
    endforeach()

    string(CONCAT SDKCONFIG_TEXT
"# GENERATED by CMakeLists.txt - do not hand-edit.\n"
"# Imported from: ${SDKCONFIG_REF}\n"
"# Overrides are at the end, after the import, so they win.\n"
"\n"
"${SDKCONFIG_BODY}"
"\n"
"# ---------------------------------------------------------------------------\n"
"# Project overrides.\n"
"# ---------------------------------------------------------------------------\n"
"\n"
"# The target. Filtered out of the import with the other identity symbols;\n"
"# without it the build silently defaults to esp32 and configures xtensa.\n"
"CONFIG_IDF_TARGET=\"esp32p4\"\n"
"\n"
"# Board variant.\n"
"${VARIANT_SETTING}"
"\n"
"# This project's partition table.\n"
"CONFIG_PARTITION_TABLE_CUSTOM=y\n"
"CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions.csv\"\n"
"\n"
"# ---- ESP-Hosted SDIO pins (deviation from the reference) -------------------\n"
"# The reference carries the generic P4 dev-board pinout. The Tab5 wires the\n"
"# C6 to clk 12, cmd 13, d0..d3 11/10/9/8, reset 15 - confirmed against\n"
"# M5Stack's own Tab5 firmware, whose sdkconfig carries exactly these values.\n"
"#\n"
"# These must be the PRIV_..._SLOT_1 symbols, not CONFIG_ESP_HOSTED_SDIO_PIN_*.\n"
"# The latter have no prompt:\n"
"#     config ESP_HOSTED_SDIO_PIN_CLK\n"
"#         int\n"
"#         default ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_1 if ESP_HOSTED_SDIO_SLOT_1\n"
"# so they are derived and setting them here is silently ignored - which is\n"
"# exactly what happened: the boot log still read CLK[18] CMD[19] D0[14].\n"
"#\n"
"# It matters because esp_hosted_host_init() runs from a global constructor,\n"
"# so these compile-time values are what the transport actually uses.\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_1=12\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_CMD_SLOT_1=13\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D0_SLOT_1=11\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D1_4BIT_BUS_SLOT_1=10\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D2_4BIT_BUS_SLOT_1=9\n"
"CONFIG_ESP_HOSTED_PRIV_SDIO_PIN_D3_4BIT_BUS_SLOT_1=8\n"
"CONFIG_ESP_HOSTED_SDIO_GPIO_RESET_SLAVE=15\n"
"CONFIG_ESP_HOSTED_GPIO_SLAVE_RESET_SLAVE=15\n"
"\n"
"# ---- ESP-Hosted mempool (deviation from the reference) ---------------------\n"
"# On this esp_hosted version sdio_mempool_create asserted during global\n"
"# constructors with 256 KB of internal RAM free, so it is off here.\n"
"#\n"
"# Worth revisiting: M5Stack's own Tab5 demo firmware runs with\n"
"# CONFIG_ESP_HOSTED_USE_MEMPOOL=y on the same silicon, so the pool is not\n"
"# inherently broken. The assert was most likely a side effect of whatever\n"
"# else was wrong at the time - the IDF version, or the pins, both since\n"
"# fixed. Try turning it back on; if it boots, delete this override.\n"
"CONFIG_ESP_HOSTED_USE_MEMPOOL=n\n"
"\n"
"# ---- FatFs fast seek -------------------------------------------------------\n"
"# Unlike exFAT and LBA64, which are hardcoded literals in IDF's ffconf.h and\n"
"# so need tools/patch_ffconf.py, this one is a real Kconfig option:\n"
"#\n"
"#     #define FF_USE_FASTSEEK\tCONFIG_FATFS_USE_FASTSEEK\n"
"#\n"
"# with no depends and no conflicts. It is simply `default n`, and the\n"
"# arduino-cli reference imported above carries no line for it, so nothing\n"
"# ever turns it on.\n"
"#\n"
"# It has to be set here rather than by hand, because this whole file is\n"
"# regenerated from the reference on any configure where the text differs -\n"
"# a hand-added line lasts exactly until the next `idf.py build`.\n"
"#\n"
"# What it costs to leave off: bigfile.cpp's clmt_build() is wrapped in\n"
"# #if FF_USE_FASTSEEK and compiles out entirely, so every f_lseek() walks\n"
"# the cluster chain a FAT sector at a time. Measured on the planet archive,\n"
"# a nine-tile place block spent 4984 ms in ten seeks against 71 ms actually\n"
"# transferring 488 KB - roughly 498 ms per seek.\n"
"#\n"
"# Independent of MAP_FATFS_EXFAT. FatFs builds a CLMT on exFAT volumes\n"
"# happily: exFAT signals end-of-chain as 0x7FFFFFFF rather than 0xFFFFFFFF,\n"
"# which is not the value CREATE_LINKMAP treats as a disk error, so the\n"
"# fragment loop terminates on `cl < fs->n_fatent` as intended.\n"
"#\n"
"# CONFIG_FATFS_FAST_SEEK_BUFFER_SIZE becomes visible once this is set and is\n"
"# deliberately left alone: it only sizes the CLMT that vfs_fat.c preallocates\n"
"# on VFS opens, and bigfile.cpp holds its own FIL and sizes its own table.\n"
"CONFIG_FATFS_USE_FASTSEEK=y\n"
"\n"
"# ---- Compiler optimization -------------------------------------------------\n"
"# -O2 rather than the reference's -Os.\n"
"#\n"
"# This is a rendering and parsing workload - inflate.c, mvt.c and raster.c\n"
"# run over every tile that reaches the screen - and those are exactly the\n"
"# loops -Os declines to unroll or inline. The seek work either side of this\n"
"# was I/O bound and the CPU was not the limit; once the CLMT removes the FAT\n"
"# walk, decode is what is left.\n"
"#\n"
"# The choice is imported as CONFIG_COMPILER_OPTIMIZATION_SIZE=y (line ~133),\n"
"# and the reference also carries two deprecated aliases for the same choice\n"
"# near the end of its legacy block:\n"
"#\n"
"#     CONFIG_OPTIMIZATION_LEVEL_RELEASE=y\n"
"#     CONFIG_COMPILER_OPTIMIZATION_LEVEL_RELEASE=y\n"
"#\n"
"# both of which sdkconfig.rename maps onto COMPILER_OPTIMIZATION_SIZE. That\n"
"# is three lines selecting -Os, so it is worth being explicit about why one\n"
"# line here beats them: kconfiglib resolves a renamed symbol inline as it\n"
"# reads each line, not in a pass afterwards, so all three land as\n"
"# COMPILER_OPTIMIZATION_SIZE=y at their own position in the stream and this,\n"
"# being last, takes the choice. Same mechanism that lets the log level below\n"
"# override the reference's LOG_DEFAULT_LEVEL_ERROR.\n"
"#\n"
"# Configure therefore prints, and this is correct rather than a problem:\n"
"#\n"
"#     NOTE: Choice COMPILER_OPTIMIZATION has multiple active selections.\n"
"#     The last one will be used: COMPILER_OPTIMIZATION_PERF.\n"
"#\n"
"# The bootloader is deliberately left at -Os. Its partition is 0x6000 and\n"
"# already tight enough that raising its log level overflows it - see the\n"
"# diagnostics note below. It runs once, so there is nothing to win there.\n"
"#\n"
"# Costs: roughly 10-20% more .text, against 0x300000 of app partition, so\n"
"# check `idf.py size` rather than assuming. Assertions are a separate axis\n"
"# and stay on (ASSERTION_LEVEL=2) - this changes speed, not checking.\n"
"CONFIG_COMPILER_OPTIMIZATION_PERF=y\n"
"\n"
"# ---- TEMPORARY: diagnostics ------------------------------------------------\n"
"# The reference sets LOG_DEFAULT_LEVEL_ERROR, right for shipping and wrong\n"
"# while the board is being brought up: it silences heap_init and esp_psram.\n"
"# Not the bootloader's level - raising that overflows its 0x6000 partition.\n"
"#\n"
"# MAXIMUM at VERBOSE, DEFAULT at INFO. The maximum is compile-time: at INFO\n"
"# every ESP_LOGV is discarded by the preprocessor, and M5GFX logs its entire\n"
"# I2C probe at that level -\n"
"#     ESP_LOGV(LIBRARY_NAME, \"[Autodetect] i2c addr:%02x = %s\", ...)\n"
"# which is exactly what says whether the bus works and what answered. The\n"
"# default stays at INFO so nothing else floods; setup() raises M5GFX alone\n"
"# with esp_log_level_set(\"M5GFX\", ESP_LOG_VERBOSE).\n"
"CONFIG_LOG_DEFAULT_LEVEL_INFO=y\n"
"CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE=y\n")

    # Only rewrite when the content actually changes, so CMake does not
    # reconfigure in a loop.
    set(SDKCONFIG_OLD "")
    if(EXISTS "${SDKCONFIG_OUT}")
        file(READ "${SDKCONFIG_OUT}" SDKCONFIG_OLD)
    endif()
    if(NOT SDKCONFIG_OLD STREQUAL SDKCONFIG_TEXT)
        file(WRITE "${SDKCONFIG_OUT}" "${SDKCONFIG_TEXT}")
        message(STATUS "sdkconfig.defaults: regenerated from ${SDKCONFIG_REF}")
        message(STATUS "sdkconfig.defaults: changed - run idf.py fullclean and "
                       "delete sdkconfig if this build behaves unexpectedly")
    endif()
else()
    message(WARNING
        "No reference sdkconfig under "
        "~/.arduino15/packages/esp32/tools/esp32p4_es-libs/*/sdkconfig - "
        "keeping the existing sdkconfig.defaults. Install the esp32 core "
        "through arduino-cli to have it generated.")
endif()

project(tab5_map)

# ---------------------------------------------------------------------------
# Second pass at the M5GFX/M5Unified ARDUINO patch.
#
# The component manager runs inside project(), so on a fresh checkout - or
# whenever a dependency re-resolves and managed_components/ is rewritten - the
# call above found nothing to patch and the components have just been
# configured without ARDUINO. Building on would produce a binary whose display
# never comes up, for a reason nothing in the log would name.
#
# Patching now fixes the file but not this build: the components were already
# registered. Stopping is the honest outcome, and the next run is clean.
tab5_install_variant(TAB5_VARIANT_INSTALLED_LATE)
tab5_patch_m5_components(M5_PATCHED_LATE)
if(TAB5_VARIANT_INSTALLED_LATE OR M5_PATCHED_LATE)
    if(TAB5_VARIANT_INSTALLED_LATE)
        message(STATUS "arduino variant: was missing after component resolution")
    endif()
    if(M5_PATCHED_LATE)
        message(STATUS "m5gfx/m5unified: were unpatched after component resolution")
    endif()
    message(FATAL_ERROR
        "managed_components/ was fetched or refreshed during this configure, "
        "which discards everything installed into it beforehand. The board "
        "variant and the ARDUINO patch have been restored, but neither can "
        "take effect in a build that has already configured those components.\n"
        "Run the same command again - it will succeed.\n"
        "Building on would give a display that never initialises and an SD "
        "slot powered through an LDO channel this board does not use. Both "
        "fail quietly. See DISPLAY_IDF_NOTES.md.")
endif()
