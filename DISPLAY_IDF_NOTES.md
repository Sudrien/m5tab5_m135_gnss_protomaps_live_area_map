# The display does not come up under ESP-IDF

Status: **resolved and confirmed on hardware.** The cause was a build
configuration problem in this project, not a bug in M5Unified or M5GFX.

Confirmed on ESP-IDF v5.5.5: `M5.getBoard()=22  M5.Display.getBoard()=22`,
`panel 1280x720 ok`, PSRAM 30963 KB free - the 1.8 MB framebuffer is allocated
- status and button canvases allocated, map rendering. Wifi associates too; see
below.

One thing remains, and it is a different problem: **panel detection is flaky on
the boot immediately after a flash write.** That is covered at the end.

Short version: `M5GFX` and `M5Unified` were compiled **without** `ARDUINO`
defined while `main` was compiled **with** it. `LovyanGFX` changes the class
layout of every one of its objects on that macro, so `main` read every member
of the `M5` object at the wrong offset. `M5Unified::begin()` is defined inline
in the header, which means it is compiled into `main`'s translation unit, and
its first statement is:

```cpp
if (_board != m5gfx::board_t::board_unknown) { return; }
```

`_board` read at the wrong offset was 255. Not zero, so **`begin()` returned
immediately, before `Display.init()`**, on every boot.

The fix is in `CMakeLists.txt`: both components are patched to require the
arduino-esp32 component, which is where `-DARDUINO=10812` comes from. This is
the vendor's own documented remedy - see "The line that was already there".

---

## Symptom

ESP-IDF build, on every boot, including plain resets with no reflash:

```
display: M5.getBoard()=255  M5.Display.getBoard()=127
smoke: panel=0x0  0x0
smoke: PSRAM free 32765 KB
reset: unknown, panel 0x0 DOWN
```

Arduino build, same sources, same hardware:

```
display: M5.getBoard()=22  M5.Display.getBoard()=22
reset: unknown, panel 1280x720 ok
PSRAM 30963 KB free
```

Two things to notice.

**255 and 127 are not board values.** From `m5gfx/boards.hpp`, `board_unknown`
is 0 and `board_M5Tab5` is 22; the enum runs 0..201 plus 512. Neither 255
(`0xFF`) nor 127 (`0x7F`) appears in it.

The original reading of this was "these fields were never written". That was
wrong, and it sent the investigation down a long dead end. `M5` is a global, so
its storage is zero-initialised before anything runs; unwritten fields read 0,
not 0xFF. A value that cannot exist in the enum and cannot be leftover memory
is neither - it is **the right object read at the wrong address**.

**The 1.8 MB PSRAM gap is the framebuffer.** 32765 KB free versus 30963 KB
is 1802 KB, and 1280 x 720 x 16bpp is 1800 KB. Under IDF it was never
allocated, because nothing ever asked for it.

---

## The cause

`src/lgfx/v1/LGFXBase.hpp`:

```cpp
#if defined (ARDUINO)
 #include <Print.h>
#endif

  class LGFXBase
#if defined (ARDUINO)
  : public Print
#endif
  {
```

With `ARDUINO` defined, `LGFXBase` gains a base class. `Print` contributes
`int write_error`, so every member of `LGFXBase` - and of everything derived
from it, which is `LGFX_Device`, `M5GFX`, `M5Canvas`, and the `Display` member
of `M5Unified` - moves by four bytes. The vtable changes too.

`ARDUINO` is not a global switch. arduino-esp32 publishes it as a **`PUBLIC`
compile option on its own CMake target**:

```cmake
target_compile_options(${COMPONENT_TARGET} PUBLIC
    -DARDUINO=10812
    ...
```

which reaches only the components that require arduino-esp32. `main` requires
it, through `main/idf_component.yml`. `m5gfx` and `m5unified` did not.

So the binary contained two incompatible layouts of the same classes: the real
one, used by the compiled component, and a four-byte-shifted one, used by every
inline function that `main` compiled for itself.

### Why that is fatal rather than cosmetic

`M5Unified::begin()` is inline in `M5Unified.hpp`, so `main` compiles its own
copy against the shifted layout:

```cpp
void begin(config_t cfg)
{
  // Allow begin execution only once.
  if (_board != m5gfx::board_t::board_unknown) { return; }
  ...
  res = Display.init();
```

`_board` reads 255. The guard treats that as "already begun" and returns. The
call to `Display.init()` is never reached, and `M5.begin()` becomes a no-op
that reports no error.

Everything else follows from that one line:

| Observation | Explanation |
|---|---|
| No M5GFX log line ever appeared, at any level | No M5GFX code ran. |
| `M5GFX_BOARD=board_M5Tab5` changed nothing | `autodetect()` was never called. |
| Framebuffer never allocated | `Display.init()` was never called. |
| Deterministic across resets | It is a compile-time layout, not a race. |
| Pinning m5unified 0.2.19 / m5gfx 0.2.26 changed nothing | Not version drift. |
| `MAP_M5_SMOKE_TEST` failed identically | Correct - the project was never the problem. |
| `setPanel(nullptr)` appeared not to take effect | `setPanel()` is out of line and wrote `_panel` at the component's offset; `getPanel()` is inline and read it at `main`'s. The note that the managed copy "does not behave that way" was wrong. |

Three independent symptoms, one cause.

---

## The line that was already there

`m5gfx/CMakeLists.txt` and `m5unified/CMakeLists.txt` both ship with this:

```cmake
### If you use arduino-esp32 components, please activate next comment line.
# list(APPEND COMPONENT_REQUIRES arduino-esp32)
```

The remedy was in the vendored files the whole time, commented out, two lines
long, and it is the thing an autodetect-shaped theory would never lead you to
look at.

---

## The fix

`CMakeLists.txt` patches both components before `project()`, in the same style
as the existing `esp_hosted` patch and for the same reason: the component
manager rewrites `managed_components/` whenever a dependency resolves, so an
edit made by hand would silently revert - and revert back into a headless
build.

The patch injects a block ahead of `register_component()`, which is required:
the legacy registration API reads `COMPONENT_REQUIRES` as it runs, so anything
appended after that call is ignored.

### Do not resolve the component name from BUILD_COMPONENTS

The first version of this patch looked the arduino component up at injection
time:

```cmake
idf_build_get_property(_tab5_components BUILD_COMPONENTS)   # WRONG
foreach(_tab5_c ${_tab5_components})
    if(_tab5_c MATCHES "arduino-esp32$")
```

That is a silent no-op, and it is worth understanding because the failure mode
is indistinguishable from success.

ESP-IDF processes each component's `CMakeLists.txt` twice. The first pass
exists only to collect requirements, and it runs in a **separate `cmake -P`
script** - `tools/cmake/scripts/component_get_requirements.cmake` - fed a
snapshot of the build properties as they stood at the time. `BUILD_COMPONENTS`
is not set until *after* that pass completes (`build.cmake` sets it during
requirement expansion, and unsets it before any retry). Read from a component
CMakeLists, it is an empty list.

So the `foreach` iterated over nothing, `COMPONENT_REQUIRES` was never
appended, and the components compiled without `ARDUINO` exactly as before -
while `idf.py` still printed `patched to compile with ARDUINO`, because the
file really had been rewritten.

The name is now resolved against the filesystem in the project's own
`CMakeLists.txt` and baked into the injected text as a literal, guarded by an
`EXISTS` check on a path relative to the component directory. Filesystem checks
and `IDF_PATH` both work in either pass; the relative path means moving or
copying the project directory does not quietly disable the patch.

### ESP-IDF v6 does not work at all

Build on **v5.5**. This is not a tuning problem, and the requirements error is
not the real one.

M5Unified does not compile against ESP-IDF v6 - not the pinned 0.2.19, not
master. The v6 I2S driver removed the port enum:

```c
/* v5.5  components/esp_driver_i2s/include/driver/i2s_types.h */
typedef enum { I2S_NUM_0 = 0, ... } i2s_port_t;

/* v6.x  same file */
#define I2S_NUM_0           0
```

and both `Speaker_Class.hpp` and `Mic_Class.hpp` declare:

```cpp
i2s_port_t i2s_port = i2s_port_t::I2S_NUM_0;
```

On v6 that is an unknown type initialised from a macro that expands to a bare
`0` after a scope operator. `SOC_I2S_NUM` is gone too. Because the member
declaration fails to parse, everything downstream reports as
`'struct m5::speaker_config_t' has no member named 'i2s_port'` - dozens of
lines that read like a different bug entirely.

This is a source incompatibility in a third-party component. Nothing in this
project can fix it; it needs an upstream release or a fork of M5Unified.
`CMakeLists.txt` now stops with an explanation instead, and
`main/idf_component.yml` pins `idf: ">=5.3,<6.0"` so the manager refuses a v6
toolchain rather than resolving against one.

### The v6 requirements error, for the record

Alongside the above, on v6 only:

```
Compilation failed because LED_Strip_Class.hpp (in "m5stack__m5unified"
component) includes driver/rmt_types.h, provided by esp_driver_rmt
component(s). However, esp_driver_rmt component(s) is not in the requirements
list of "m5stack__m5unified".
```

Real, but secondary, and nothing to do with M5GFX. `m5gfx`'s CMakeLists has an
`IDF_VERSION_MAJOR GREATER_EQUAL 6` branch naming the split-out `esp_driver_*`
components; `m5unified`'s does not - it asks only for the `driver` umbrella on
anything past v5, and on v6 `driver` no longer re-exports them.
`LED_Strip_Class.hpp` includes `driver/rmt_types.h` unconditionally whenever
`SOC_RMT_SUPPORTED` and IDF >= 5.0.

The patch appends the whole set - `esp_driver_rmt`, `esp_driver_i2s`,
`esp_driver_i2c`, `esp_driver_gpio`, `esp_driver_ledc`,
`esp_driver_touch_sens`, `esp_adc`, `hal`, `esp_timer` - to both components,
each name checked against `$IDF_PATH/components` first, because fixing only the
one the error names gets you the next one. It changes nothing on v5, where they
are all reachable through `driver` already. It is kept so that whenever
M5Unified does gain v6 support, this is one less thing in the way.

The legacy-I2C end-of-life banner that appears throughout a v6 build is a
warning, not a failure. It comes from M5GFX using `driver/i2c.h`, EOL in v6.0
and slated for removal in v7.0. Upstream's problem, eventually.

### Confirmation

At configure time:

```
-- m5stack__m5gfx: patched to compile with ARDUINO
-- m5stack__m5unified: patched to compile with ARDUINO
-- m5stack__m5gfx: requiring espressif__arduino-esp32 for -DARDUINO
```

The third line is the one that matters. The first two only say the file was
rewritten; the third is printed by the injected code itself, from inside the
component, and so is the only one that proves the requirement was actually
added. If it is missing, the patch is inert.

At runtime, the board line that has always been printed:

```
display: M5.getBoard()=22  M5.Display.getBoard()=22
```

`setup()` now also checks those two values against the range `board_t` can
hold and, if either is outside it, prints what to look at. That check is the
cheap general detector for this class of problem, which will recur if the
components are ever re-fetched into a tree whose patch has been lost.

## What this means for the esp_lcd route

M5Stack's own Tab5 firmware (`m5stack/M5Tab5-UserDemo`, and the
`BufferRoot/M5STACK-TAB5-DEMO-CLEAN` fork on ESP-IDF 5.5.3) drives the panel
through a BSP rather than M5GFX, and there is a standalone worked example of
the same thing in `m5tab5_esp_idf_display_example` - plain `esp_lcd`, no
M5Unified, no M5GFX, no BSP.

Porting onto that is **no longer necessary**, and doing it would be a net loss:
it permanently diverges the two builds and abandons a rendering layer this
project uses heavily - `M5Canvas` sprites, `drawString` with datums and fonts,
`setClipRect`, `writePixels` - for one that has none of it.

The example remains worth keeping for two things it establishes independently
of M5GFX, both of which M5GFX handles internally and neither of which is
obvious from the outside:

- **`LCD_RST` is not a GPIO.** It is on a PI4IOE5V6416 I/O expander at I2C
  `0x43`, pin `P4`. M5GFX does drive it - see the `reg_data_io1_1` /
  `reg_data_io1_2` sequences in its Tab5 autodetect branch - and passes
  `reset_gpio_num = -1` to mean "handled out of band", not "no reset line".
  Held in reset, the DSI host spins forever in
  `mipi_dsi_hal_host_gen_write_dcs_command()` on a FIFO that never drains.
- **The backlight starts off.** PWM on GPIO 22; M5GFX sets it up as LEDC
  channel 7 at 44.1 kHz, the example uses channel 1 at 5 kHz. Panel init can
  run start to finish with the screen still dark.

The two also disagree on panel parameters, worth knowing if the panel comes up
but looks wrong. M5GFX picks by touch controller firmware version: 900 Mbps and
70 MHz for the ST7121, 1040 Mbps and 80 MHz for the ST7123. The example uses
965 Mbps for the ST7121. Both use 2 lanes, LDO channel 3 at 2500 mV, and
720x1280 portrait-native.

---

## Wifi under IDF: fixed by the same change

It was the same bug, downstream. Wifi now associates on every boot:

```
I (6100) transport: Identified slave [esp32c6]
I (6100) transport: capabilities: 0xd
wifi: scan found 12 networks
wifi: connected, IP 192.168.5.62, RSSI -45 dBm
```

The ESP32-C6's power rail is not a GPIO. It is on the second PI4IOE5V6416, at
I2C `0x44`, pin `P0`, and M5Unified drives it in `Power_Class::begin()`
(`utility/Power_Class.cpp`, the Tab5 case):

```
///     +--------- CHG_EN
///     |+-------- CHG_STAT
///     ||+------- nCHG_QC_EN
///     |||+------ PWROFF_PLUSE
///     ||||+----- USB5V_EN
///     |||||+---- NC
///     ||||||+--- NC
///     |||||||+-- WLAN_PWR_EN
0x05, 0b10000001,   // OUT_SET   <- P0 high
0x03, 0b10110001,   // IO_DIR    <- P0 output
```

`Power.begin()` is called from `M5Unified::_begin()`, which is called from the
inline `begin()` at the line *after* the `_board` guard that was returning
early. So for the entire time wifi was being investigated, `WLAN_PWR_EN` was
never driven and the C6 had no power. `sdmmc_init_ocr` returning 0x107 is
`ESP_ERR_TIMEOUT` - nothing on the other end - while the host-side pins logged
correctly, because the host side was never the problem.

**Keep the esp_hosted constructor patch.** It is load-bearing for a second
reason now: a constructor runs before `app_main`, and therefore before
`M5.begin()` powers the C6. Bringing SDIO up from there is bringing it up
against an unpowered slave. Letting arduino-esp32 do it from `WiFi.mode()`,
after `setup()`, is the only ordering that can work - and the log confirms it,
with `H_API: ESP-Hosted starting` arriving at 4.4 s, well after `M5.begin()`.

Two cosmetic things in the wifi log, neither a fault:

- `Version mismatch: Host [2.12.0] > Co-proc [0.0.0]` - the C6 slave firmware
  does not report a version. It works; the warning is upstream's.
- `sdmmc_host_init: SDMMC host already initialized, skipping init flow` - the
  SD card was mounted first and shares the host. Expected.

---

## Open: panel detection is flaky on the boot after a flash write

Not the layout bug, and not new - the Arduino build has always had it, which is
what `panelBegin()` was originally written for.

Symptom, on the boot immediately following a flash write
(`rst:0x17 CHIP_USB_UART_RESET`):

```
[   397][E][M5GFX.cpp:2825] autodetect(): [M5GFX] M5Tab5 display panel was not detected
display: M5.getBoard()=22  M5.Display.getBoard()=22
reset: unknown, panel 0x0 DOWN
```

The next boot, same binary, brings the panel up normally. So the board is
found, the DSI bus initialises, and only the panel identification fails.

`M5GFX.cpp:2825` is reached when `_panel_last` is still null after all three
probes: the ST touch firmware version read over I2C (which selects ST7121 vs
ST7123), and the two DSI ID reads (`0xF4` for the ST, page-1 `0x00..0x02` for
the ILI9881C). All of them came back empty.

### Neither retry that looks like it should help can fire

Worth writing down, because both look available and neither is.

**M5GFX's own retry.** `init_impl` wraps autodetect in four attempts:

```cpp
int retry = 4;
do {
  if (retry == 1) use_reset = true;
  board = autodetect(use_reset, board);
} while (board_t::board_unknown == board && --retry >= 0);
```

It loops only while the board is *unknown*. The Tab5 branch sets
`board = board_M5Tab5` as soon as it sees the two I/O expanders on I2C - before
it probes the panel - and when the panel probe fails it takes
`goto init_clear`, which returns that board anyway. The loop sees a known board
and stops after one attempt. Those four retries are unreachable for this exact
failure.

**Calling `Display.init()` again.** `init_impl` opens with:

```cpp
if (getBoard() != board_t::board_unknown) { return true; }
```

`_board` is 22 by then, so it returns true immediately, having re-probed
nothing. `panelBegin()` used to do this and log it as a retry; the log line
"panel attached but not up, retrying init" followed by a failure with no second
M5GFX error between them is that call reaching no hardware at all.

Re-running autodetect by hand is not attractive either: it has already done
`_bus_last.reset(bus_dsi)` with `bus_dsi->init()` succeeding, so a second pass
re-enters a half-initialised DSI stack.

### What is done about it

`panelBegin()` now restarts once, counted in RTC memory, and runs headless if
the second boot fails too. Crude, but it is the only lever that is known to
work, since the next boot does come up. The bound is the important part: a
panel that is genuinely absent must not turn a working GPS logger into a boot
loop. The counter is cleared only when the panel is actually up, not merely
when setup() finishes - clearing it on a headless boot handed every subsequent
boot a fresh restart, which is the loop it exists to prevent.

### What has not been tried

Something in the call sequence around `M5.begin()` may fix this properly rather
than papering over it. The reset that fails is the one where the panel and
touch controller keep their state from the previous run, and M5GFX holds
`LCD_RST`/`TP_RST` low for only about 10 ms (`reg_data_io1_1`, `delay(10)`,
`reg_data_io1_2`, `delay(100)`). Asserting both for longer, over I2C through
`lgfx::i2c` so it uses the same driver M5GFX does, before `M5.begin()`, is the
obvious experiment.

Two cautions before trying it:

- `GPIO 23` (TP_INT) is the touch controller's I2C address strap, and M5GFX
  drives it high across the reset release for that reason. Releasing `TP_RST`
  while it is not high latches the wrong address, which would produce exactly
  the failed firmware read seen here.
- This project has a recorded regression from putting code before
  `M5.begin()`: three lines (two `esp_log_level_set` and `Serial.begin`) cost
  the Arduino build its framebuffer. Whatever goes in there gets tested on both
  builds, or not at all.

---

## What the IDF build did while headless


Everything else: SD at SDMMC 4-bit, credential store, GNSS with AssistNow
restore (assisted fixes in 6-8 s, hdop under 1), tile cache, map rendering,
stable heap over 30+ second runs. It also found several real bugs in the
shared sources - the `int32_t` format specifiers, the `snprintf` truncations,
and the missing `loop()` guard after a failed `setup()`.

The headless path stays, and is no longer hypothetical for a second reason:
detection still fails on the occasional post-flash boot, on either build. The
`g_panelOk` guards are load-bearing - every `M5.Display` call in this project
dereferences a pointer that is null when there is no panel.
