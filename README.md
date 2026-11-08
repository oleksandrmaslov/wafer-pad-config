# Wafer Pad

Wafer Pad is a Bluetooth LE HID mouse built from an Azoteq TPS65 (IQS550) multi-touch trackpad, an ST LSM6DSOX IMU and a nice!nano v2 (nRF52840). It runs on Zephyr v4.5.0 as a standalone application.

* On a desk it is a trackpad (move, tap, tap-drag, long-press drag, two-finger scroll, right click).
* The IMU detects which edge faces the user and rotates pointer and scroll axes accordingly.
* Picked up, it becomes an air mouse: a single still finger acts as a clutch and the gyroscope rate moves the cursor.

> **Status: nothing has run on real hardware.** The firmware compiles and the algorithm unit tests pass on `native_sim`. Every hardware-facing statement in this repository is unverified until it is ticked off in [docs/testing.md](docs/testing.md).

## Documentation

| Document | Content |
|---|---|
| [docs/wiring.md](docs/wiring.md) | Pin table, I2C addresses, pull-ups, TPS65 connector, battery notes. Hardware facts live only there. |
| [docs/architecture.md](docs/architecture.md) | Layers, threads, frames, orientation, state machine, HID, BLE, power, future keyboard dongle. |
| [docs/calibration.md](docs/calibration.md) | Bring-up and booth tuning procedure. |
| [docs/testing.md](docs/testing.md) | Automated tests, what was verified, hardware test plan. |

## Milestone status

"Implemented" means: code exists, it compiles for `nice_nano_v2`, and where the module is pure C there is a `native_sim` unit test. No milestone has hardware verification yet.

| Milestone | Scope | Implemented + compiled + unit-tested | Needs hardware verification |
|---|---|---|---|
| M1 bring-up | Board port, USB CDC console + shell, I2C, `wp` shell, west manifest | Compiled. No unit tests (board and shell code). | Everything: UF2 flash, console, `i2c scan`, LED |
| M2 touchpad | IQS5xx driver, touch source, touch tracker, gesture engine, pointer accel, HID composer, BLE HOG | Tracker, mount transform, gestures, HID descriptor/pack/composer: unit-tested. Driver and BLE: compiled only. | Touch frame rate, axis mounting, cursor latency, pairing, reconnect |
| M3 IMU | LSM6DSOX source, gyro bias estimator, motion features | Bias estimator and motion features: unit-tested. Sensor access: compiled only. | Axis mount map, bias convergence, read errors |
| M4 orientation | Gravity estimator (A), lock (B), calibration gesture (C), gyro assist (D, experimental) | Unit-tested | Tilt thresholds, lock behaviour, gesture reliability on the real pad |
| M5 air mouse | State machine, pickup detection, clutch, gyro-to-cursor | State machine and air mouse math: unit-tested | Pickup/put-down detection, feel, drift |
| M6 polish | Power states, System OFF, battery, LED patterns, bond handling, error recovery | State machine transitions: unit-tested. Power, LED, battery: compiled only. | Current per state, System OFF wake, soak test |

## Hardware

| Part | Notes |
|---|---|
| nice!nano v2 (nRF52840) | Adafruit UF2 bootloader, S140 SoftDevice region kept (not used by Zephyr) |
| Azoteq TPS65 trackpad (IQS550, B000 firmware) | I2C 0x74, RDY and NRST lines. End-of-life part: get spares. |
| ST LSM6DSOX breakout | I2C 0x6A (or 0x6B). Driver is Zephyr `st,lsm6dso`. |
| LiPo with protection circuit (optional) | Charger is on the nice!nano. See wiring.md section 6. |

Wiring: [docs/wiring.md](docs/wiring.md). Devicetree: [boards/nice_nano_v2.overlay](boards/nice_nano_v2.overlay).

## Quick start

### a) Fresh workspace

Needs git, west, CMake, Ninja and Zephyr SDK 1.0.1 (the SDK version of Zephyr v4.5.0).

```sh
mkdir wafer-ws && cd wafer-ws
git clone <repo> wafer-pad
west init -l wafer-pad
west update
west packages pip --install
```

`wafer-pad` is its own manifest repository ([west.yml](west.yml)). It pins Zephyr v4.5.0 and imports only the modules the firmware needs.

### b) Build

From the workspace directory (`wafer-ws`):

```sh
west build -p always -b nice_nano_v2 wafer-pad
```

Output: `build/zephyr/zephyr.uf2`.

To change a Kconfig symbol for one build, append `-- -DCONFIG_NAME=value`. To change it permanently, edit [prj.conf](prj.conf).

### c) Flash over UF2

1. Double-tap RST on the nice!nano. The USB drive `NICENANO` appears.
2. Copy `build/zephyr/zephyr.uf2` onto it. The board reboots into the application.

Alternative: `west flash -r uf2` (runner configured in `boards/nicekeyboards/nice_nano_v2/board.cmake`).

Flash layout (see the DTS comment in `boards/nicekeyboards/nice_nano_v2/nice_nano_v2_nrf52840.dts`):

| Range | Content |
|---|---|
| 0x00000-0x26000 | MBR + SoftDevice region. Never overwritten. |
| 0x26000-0xEA000 | Application (code partition, 784 KB). The image is linked at 0x26000 and must end below 0xEA000. |
| 0xEC000-0xF4000 | Storage: settings, BLE bonds, gyro bias (32 KB) |
| 0xF4000-0x100000 | Adafruit bootloader and its settings. Never overwritten. |

**Back to the bootloader:** double-tap RST again. The firmware has no command that reboots into the bootloader. If the application hangs or the console is gone, double-tap RST still works. SWD runners (`jlink`, `pyocd`) are configured in `board.cmake` for restoring the bootloader itself; this was not tried.

### d) Console and shell

The console and shell are on a USB CDC ACM serial port (on Linux usually `/dev/ttyACM0`):

```sh
picocom -b 115200 /dev/ttyACM0
```

The baud rate is irrelevant for CDC ACM. Commands in [src/router/router_shell.c](src/router/router_shell.c):

| Command | Effect |
|---|---|
| `wp status` | State, mode, orientation (source, confidence), tilt, heading, contacts, BLE link, IMU/gyro, counters, `max latency`, battery |
| `wp orient <0\|90\|180\|270\|auto>` | Force an orientation (manual) or return to automatic |
| `wp mode <auto\|handheld\|trackpad>` | Mode override |
| `wp pair [seconds 0..3600]` | Open the BLE pairing window (default 120 s) |
| `wp unpair` | Remove all BLE bonds and restart advertising |
| `wp bias` | Print the gyro bias estimate (milli-dps, body frame) |
| `wp touch` | Touch/IMU frame counters, rates since the last call, `max latency` |
| `wp imu` | Latest body-frame IMU sample, tilt and gravity heading |
| `i2c scan i2c@40003000` | I2C bus scan (`CONFIG_I2C_SHELL=y`). Expect 0x74 and 0x6a/0x6b. |

### e) Tests

```sh
west twister -T wafer-pad/tests -p native_sim/native/64
```

Details: [docs/testing.md](docs/testing.md).

### First pairing

1. Power the pad. A pairing window of `CONFIG_WAFER_PAD_BLE_PAIRING_WINDOW_S` (120 s) opens at every boot. While no bond exists, pairing is accepted at any time. You can reopen the window with `wp pair`. The window closes after one host has paired.
2. Pair from the host's Bluetooth settings: device name "Wafer Pad". Pairing is Just Works.
3. The host must support **LE Secure Connections** (Bluetooth 4.2 or newer). Legacy-only hosts are rejected (Zephyr default `CONFIG_BT_SMP_SC_PAIR_ONLY=y`).
4. Outside the window, new pairings are rejected so that visitors cannot evict the demo host's bond. Already bonded hosts reconnect normally. Up to 4 bonds are kept (`CONFIG_BT_MAX_PAIRED=4`).
5. If a host forgot the pad, open the window with `wp pair` and pair again.

LED patterns (in [src/power/led.c](src/power/led.c)): advertising 200/800 ms, connected short blip every 3 s, calibration armed fast blink, air mouse solid, error double blink.

## Repository structure

```
wafer-pad/
  west.yml, CMakeLists.txt, Kconfig, prj.conf
  boards/
    nice_nano_v2.overlay            app overlay: I2C, TPS65, LSM6DSOX, ADC, regulator
    nicekeyboards/nice_nano_v2/     out-of-tree board (UF2 flash map, board.cmake)
  drivers/azoteq/                   IQS5xx driver (iqs5xx.c, Kconfig)
  dts/bindings/input/               azoteq,iqs5xx.yaml
  include/drivers/azoteq_iqs5xx.h
  include/wafer_pad/                public headers (types.h, platform.h, ...)
  src/
    main.c                          bring-up
    touch/  imu/                    sources (hardware side)
    router/                         router thread, control work queue, wp shell
    orientation/ gestures/ air_mouse/ state/   algorithms (pure C)
    hid/                            report composer, BLE HOG, transport
    power/                          power, battery, LED
  tests/                            Twister suites for native_sim
  docs/                             wiring, architecture, calibration, testing
```

## Key Kconfig options

All defaults are from the `Kconfig` files; edit [prj.conf](prj.conf) to override.

| Symbol | Default | Meaning |
|---|---|---|
| `WAFER_PAD_TOUCH_SWAP_XY` / `_INVERT_X` / `_INVERT_Y` | n / n / n | Touch axis mounting |
| `WAFER_PAD_IMU_MOUNT_X` / `_Y` / `_Z` | 1 / 2 / 3 | IMU axis to body axis map |
| `WAFER_PAD_ORIENT_AUTO` | y | Automatic orientation from gravity |
| `WAFER_PAD_ORIENT_MIN_TILT_DEG` | 20 | Minimum tilt for an orientation fix |
| `WAFER_PAD_ORIENT_CONFIRM_MS` | 300 | Confirmation time before a change |
| `WAFER_PAD_ORIENT_GYRO_ASSIST` | n | Algorithm D (experimental) |
| `WAFER_PAD_CALIB_FINGERS` / `_HOLD_MS` / `_SWIPE_MIN_MM` | 3 / 1500 / 25 | Calibration gesture |
| `WAFER_PAD_POINTER_SENSITIVITY_CENTI` | 100 | Pointer gain (x0.01) |
| `WAFER_PAD_POINTER_ACCEL_MAX_MULT_CENTI` | 300 | Maximum acceleration multiplier |
| `WAFER_PAD_GEST_SCROLL_NATURAL` | y | Natural scrolling |
| `WAFER_PAD_GEST_JUMP_MAX_STEP_MM` / `_JUMP_MAX_SPEED_MM_S` | 15 / 2000 | Ignore implausible finger jumps (ghost contacts, slot reuse); 0 disables |
| `WAFER_PAD_IMU_DEAD_MS` / `_DEAD_ACCEL_MILLI_G` | 500 / 200 | Re-initialize the IMU if it silently returns zeros (rail glitch); 0 disables |
| `WAFER_PAD_AIR_COUNTS_PER_DEG_CENTI` | 1200 | Air mouse sensitivity (x0.01 counts/deg) |
| `WAFER_PAD_AIR_DEADZONE_DECI_DPS` | 15 | Air mouse deadzone (x0.1 deg/s) |
| `WAFER_PAD_AIR_LPF_CUTOFF_HZ` | 15 | Air mouse rate low-pass |
| `WAFER_PAD_SM_IDLE_TIMEOUT_MS` | 30000 | Inactivity before IDLE |
| `WAFER_PAD_SM_SLEEP_ENABLED` | n | Allow System OFF (off for the exhibition build) |
| `WAFER_PAD_SM_SLEEP_TIMEOUT_MS` | 900000 | IDLE time before System OFF |
| `WAFER_PAD_SM_GYRO_ALWAYS_ON` | n | Keep the gyro powered in every state |
| `WAFER_PAD_IMU_WAKEUP` | n | IMU INT1 as System OFF wake source |
| `WAFER_PAD_BLE_PAIRING_WINDOW_S` | 120 | Pairing window after boot |
| `WAFER_PAD_BLE_LL_INTERVAL_MIN_US` / `_MAX_US` | 15000 | Low-latency connection interval |
| `WAFER_PAD_BATTERY_INTERVAL_S` | 60 | Battery measurement interval |

## Known limitations

* **No hardware testing yet.** Pin assignments, I2C addresses, pull-ups, FPC pinout and power numbers are paper facts. See the VERIFY ON HARDWARE items in [docs/wiring.md](docs/wiring.md).
* **No yaw reference when flat.** A 6-axis IMU cannot tell which edge faces the user while the pad lies flat. Orientation is derived only while the pad is tilted, then locked. See [docs/architecture.md](docs/architecture.md).
* The air mouse uses gyro rate only. It drifts if the bias is wrong; there is no absolute pointing.
* System OFF is disabled by default (`CONFIG_WAFER_PAD_SM_SLEEP_ENABLED=n`). Wake is a reset.
* One BLE connection at a time. Bonds are stored for up to 4 hosts.
* Mouse only. Report IDs 2 and 3 are reserved for a future keyboard; `wp_router_post_key()` returns `-ENOTSUP`.
* The TPS65 is end-of-life (see wiring.md section 4).
* USB is used for console and charging only, there is no USB HID transport.

## License

New source files carry `Copyright (c) 2026 Wafer Pad contributors` and `SPDX-License-Identifier: Apache-2.0` headers. The repository has no top-level `LICENSE` file yet; add one before publishing. Third-party code is not vendored; Zephyr (Apache-2.0) and its modules are fetched by `west update` under their own licenses.

Provenance of derived code:

* `src/hid/ble.c` and `src/hid/ble_hog.c` are adapted from Zephyr's `samples/bluetooth/peripheral_hids` (Apache-2.0) and keep its original copyright line.
* The Azoteq driver (`drivers/azoteq/`) was written from the Azoteq IQS5xx-B000 datasheet and application notes. No code was copied from the AYM1607 ZMK driver (MIT) or from the GPL-licensed Linux and QMK drivers; those were used only to cross-check register facts.
