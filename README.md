# SpO2 / Heart-Rate Monitor (MAX30102 + SSD1306, Pico / Pico 2)

[中文](README-zh.md)

A blood-oxygen and heart-rate monitor for the Raspberry Pi Pico and Pico 2,
built on top of the provided MAX30102 and SSD1306 drivers.

## Files

| File | Purpose |
|---|---|
| `MAX30102.c` | Application entry point: sensor setup, acquisition loop, OLED dashboard |
| `algorithm.c` / `algorithm.h` | SpO2 (ratio-of-ratios) and heart-rate (autocorrelation) estimation |
| `max30102.c` / `max30102.h` | MAX30102 driver (upgraded, see [Library Upgrade](#library-upgrade)) |
| `ssd1306.c` / `ssd1306.h` | SSD1306 OLED driver (baudrate made configurable via `SSD1306_I2C_BAUD`) |

## Wiring

| Signal | MAX30102 | SSD1306 OLED |
|---|---|---|
| I2C bus | I2C0 | I2C1 |
| SDA | GP4 | GP6 |
| SCL | GP5 | GP7 |
| VCC | 3.3V | 3.3V |
| GND | GND | GND |

The two devices are placed on separate I2C buses so neither one's traffic
(especially the OLED's larger writes) can delay the other.

> **Note**: The SSD1306 OLED on I2C1 runs at **100kHz** standard mode. This is
> because the RP2350 internal pull-up resistors (~50kΩ) cannot provide adequate
> signal rise times at 400kHz fast mode. If 400kHz is required, add external
> 4.7kΩ pull-up resistors on GP6/GP7 to 3.3V.

## Building

1. Install the Raspberry Pi Pico SDK (**v2.0.0 or later** is required for
   Pico 2 / RP2350 support) and set `PICO_SDK_PATH`.
2. Copy the SDK's import helper into this folder:
   ```bash
   cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
   ```
3. Configure and build:
   ```bash
   mkdir build && cd build
   cmake ..              # builds for Pico 2 (RP2350) by default
   make -j4
   ```
   To target the original Pico instead:
   ```bash
   cmake -DPICO_BOARD=pico ..
   ```
4. Flash `MAX30102.uf2` by holding BOOTSEL while plugging in the board,
   then copying the file to the mounted drive.

## How it works

Every loop iteration the sensor's FIFO is drained completely (handling the
driver's 32-byte burst cap by repeatedly calling `max30102_check()` until the
read/write pointers match), and each (IR, RED) sample pair is fed into
`spo2_algorithm_add_sample()`.

* **DC tracking**: a first-order IIR low-pass filter (alpha = 0.95) tracks
  the slowly-varying DC component of each channel; subtracting it leaves the
  pulsatile AC component.
* **Finger detection**: the IR DC level is compared against
  `SPO2_FINGER_THRESHOLD`. Below this, the display shows "Place finger".
* **SpO2**: once a 100-sample (4 s) window is full, the RMS of the AC signal
  on each channel is computed and combined into the standard ratio-of-ratios
  `R = (RMS_red/DC_red) / (RMS_ir/DC_ir)`, then mapped with the commonly used
  linear approximation `SpO2 = 110 - 25*R`.
* **Heart rate**: an autocorrelation of the IR AC signal is computed for lags
  corresponding to 40-200 BPM; the lag with the strongest correlation gives
  the pulse period.
* **Display**: refreshed once per second with SpO2, pulse, and the raw IR DC
  value (useful for tuning the finger-detection threshold).

> **Disclaimer**: the `110 - 25*R` formula and the default LED currents are
> a commonly used starting point for demos, not a calibrated medical device.
> Real pulse oximeters are calibrated against a reference instrument across
> many subjects. Treat the SpO2 output as approximate, and tune
> `SPO2_FINGER_THRESHOLD`, the LED currents (passed via a
> `max30102_config_t` instead of `NULL`), and the SpO2 formula constants for
> your specific hardware if accuracy matters.

## Library Upgrade

The MAX30102 driver was upgraded from the original provided version. Key
improvements over the original:

| Change | Old library | New library |
|---|---|---|
| `max30102_setup` return | `void` (silent failure) | **`bool`** — every I2C write checked |
| `max30102_reset` | `sleep_ms(500)` blind wait | Polls MODE_CONFIG register until ready (≤10 ms) |
| `max30102_check` pointer read | Two separate `read_reg` calls | Single 3-byte burst read (fewer I2C transactions) |
| `max30102_check` return | Raw sample gap | **Actual decoded sample count** |
| `max30102_read_temperature` | Reads stale register | **Triggers new conversion**, polls until complete |
| `max30102_read_temperature_fixed` | Truncation `*625/100` | **Rounding** `(*625+50)/100` |
| FIFO ring buffer wrapping | `if(n<0) n+=32` + `%=` | **`& FIFO_MASK` bitmask** (branchless) |
| FIFO RED/IR byte order | **Swapped** (IR first, RED second) | **Correct** per datasheet (RED first, IR second) |

> **Important**: The corrected RED/IR byte order in the new library means
> `max30102_get_red()` and `max30102_get_ir()` now return the correct channels,
> whereas the old library returned them swapped. To maintain compatibility with
> the algorithm's original calibration, `spo2_algorithm_add_sample()` is
> called with `(red, ir)` order in `MAX30102.c:107`.

## Tuning notes

* `SPO2_FINGER_THRESHOLD` (in `algorithm.h`): raise/lower based on the "IR DC"
  value shown on the OLED with and without a finger present.
* LED currents: pass a populated `max30102_config_t` to `max30102_setup()`
  (instead of `NULL`) to increase `led1_current`/`led2_current` if the AC
  signal is too weak (e.g. through thicker tissue or darker enclosures).
* `SPO2_BUFFER_SIZE` / window length: trades responsiveness (shorter window)
  for stability (longer window).

## Troubleshooting

| Problem | Likely cause | Solution |
|---|---|---|
| OLED doesn't display | I2C1 baudrate too high | Confirm `SSD1306_I2C_BAUD=100000` is in CMakeLists.txt (not only in MAX30102.c) |
| OLED doesn't display | Wiring error | Check GP6(SDA), GP7(SCL), 3.3V, GND connections |
| OLED doesn't display | Missing pull-up resistors | Add external 4.7kΩ on GP6/GP7 to 3.3V |
| MAX30102 not found | Wiring error | Check GP4(SDA), GP5(SCL), 3.3V, GND connections |
| Readings always zero | Sensor not in contact | Place finger gently on the sensor |
| SpO2/HR values erratic | Unstable finger contact | Keep finger still, avoid pressing too hard |
| SpO2 abnormal after library upgrade | RED/IR channel order change | Ensure `spo2_algorithm_add_sample(red, ir)` is used (see [Library Upgrade](#library-upgrade)) |

## Updates

### 2026-06-18

### Dual-Core Architecture (Core 0: Acquisition + Display, Core 1: Computation)

The original single-core superloop has been refactored into a dual-core architecture
leveraging the RP2350's two Cortex-M33 cores via Pico SDK `pico_multicore`:

| Core | Role | I2C peripherals |
|------|------|-----------------|
| Core 0 | MAX30102 sensor acquisition + SSD1306 OLED display | I2C0 + I2C1 |
| Core 1 | SpO2 / heart-rate computation (algorithm.c) | none |

**Inter-core communication**: A 64-slot SPSC ring buffer (`shared_fifo_t`) is guarded by
a hardware spinlock. Core 0 (producer) pushes raw RED/IR samples; Core 1 (calculator)
pops them for algorithm processing. Computed results are published through a separate
spinlock-protected result buffer (`shared_result_t`).

To revert to the original single-core behavior for debugging, uncomment `#define SINGLE_CORE` at the top of `MAX30102.c`.

### Performance

* **Streaming RMS** (`algorithm.c`): AC sum-of-squares (`sum_sq_ir` / `sum_sq_red`) is
  accumulated incrementally in `spo2_algorithm_add_sample()` rather than recomputed
  by looping over 100 samples in every `spo2_algorithm_compute()` call.
* **Core 1 sleep reduced** from 5 ms to 1 ms when idle — faster response to new samples.

### Memory

* **Shared FIFO reduced** from 128 to 64 slots (512 bytes saved). At the effective
  sample rate of 25 Hz, 64 slots still provide 2.56 seconds of buffering — well above
  the 4-second computation window requirement.
* **Removed unused libraries** `hardware_dma` and `hardware_pio` from `CMakeLists.txt`.

### Reliability

* **Hardware watchdog** (`hardware_watchdog`, 3 s timeout): if Core 0's main loop stalls
  (e.g. due to an I2C bus hang), the chip is automatically reset. `watchdog_update()`
  is called in every loop iteration.
* **`isfinite()` guards** in `spo2_algorithm_compute()`: SpO2 values are checked with
  `isfinite(spo2)` before marking them valid; heart rate is additionally bounded to
  30–250 BPM with `isfinite(hr)`. This prevents NaN/Inf from corrupt sensor data
  reaching the OLED display.

### Power Saving

* **Adaptive polling**: when `finger_present == false`, the Core 0 polling interval
  increases from 20 ms (50 Hz) to 100 ms (10 Hz), reducing power consumption ~5×
  during idle periods. The polling rate automatically returns to 50 Hz when a
  finger is detected.

### Code Quality

* **`printf` wrapped in `DBG_PRINTF`**: all debug output is gated by a `#define DEBUG`
  at the top of `MAX30102.c`. Comment out the `#define` to produce a release build
  with zero USB stdio overhead.
* **Sensor configuration parameterized**: a named `g_sensor_cfg` struct (type
  `max30102_config_t`) replaces the hardcoded `NULL` default. LED currents, sample
  rate, averaging, and ADC range are all visible and tunable in one place.
* **OLED refresh period** is controlled by a single `OLED_REFRESH_MS` macro
  (default 1000 ms).
* **Initialization progress display**: `show_init_progress()` shows "Sensor detect"
  then "Acquiring data..." on the OLED during startup, replacing the previous
  4-second blank screen.

### Build Changes (`CMakeLists.txt`)

* Added: `pico_multicore` (dual-core support), `hardware_watchdog` (watchdog timer).
* Removed: `hardware_dma`, `hardware_pio` (not used by the application code).
