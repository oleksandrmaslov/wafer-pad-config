# Wafer Pad testing

## 1. What has been verified

| Item | Status |
|---|---|
| Firmware compiles for `nice_nano_v2` (Zephyr v4.5.0, SDK 1.0.1) | Done |
| Unit tests on `native_sim/native/64` (suites in section 2) | Done |
| Everything on real hardware (I2C, touch, IMU, BLE, power, UF2 flashing) | **Not done. Nothing has run on hardware.** |

The unit tests cover pure-C algorithm modules only. They say nothing about the IQS5xx driver, I2C timing, BLE behaviour with real hosts, current consumption or sensor mounting.

## 2. Automated tests

Location: `tests/<suite>/` (Twister `testcase.yaml`, tag `wafer_pad`, platform `native_sim/native/64`).

```sh
# from the workspace directory (wafer-ws)
west twister -T wafer-pad/tests -p native_sim/native/64
# one suite
west twister -T wafer-pad/tests/gestures -p native_sim/native/64
```

| Suite | Covers (ZTEST suites in the sources) |
|---|---|
| `touch` | `touch_tracker`: down/move/up events, id reuse, overflow, release-all; `touch_mount`: all 8 swap/invert combinations, clamping |
| `imu` | `imu_bias`: convergence while still, no estimate while rotating or moving, implausible bias rejected, restore of a persisted value, hand tremor, re-learning a wrong value; `imu_motion`: still/moving detection, hand tremor, tilt, bad timestamps |
| `orientation` | `orient_transform` (all four rotations, scroll mapping, IMU mount, convention checks), `orient_gravity` (tilt in each direction, hysteresis, confirmation, rejection of lifting/fast rotation, flat gives no candidate), `orient_mgr` (deferred change during touch, manual lock modes, force/allow-auto), `orient_calib` (four directions, rejects scroll-like gestures, timeouts, finger-count changes), `orient_calib_chain` (rest-then-flick scroll never calibrates; 3-finger hold and swipe calibrates), `orient_gyro` (algorithm D steps, sign convention, gating) |
| `gestures` | Tap, tap-drag, long-press drag, two/three finger tap, scroll (natural, axis lock), jitter deadband, smoothing, acceleration curve, drag release on reset, orientation-dependent pointer and scroll direction |
| `air_mouse` | Config defaults, yaw/pitch direction, same motion in all orientations, deadzone, low-pass, ramp-in, gravity compensation, clamp, invert, time gaps |
| `state_machine` | Boot sequence, error recovery back-off, pickup (tremor, continued motion), bump rejection, timeouts, air mouse engage/disengage, put-down, idle/wake, sleep (enabled/disabled, BLE-disconnected), calibration enter/return, mode override cycle, no-IMU fallback, output flags per state, robustness |
| `hid` | `hid_desc` (balanced collections, 56-bit report 1, usages), `hid_pack` (little-endian layout, range limits), `hid_composer` (fractional accumulation, sub-count jitter, large motion split, wheel/pan split, press/release, release-all, non-finite input) |

Not covered by any automated test: `drivers/azoteq`, `touch_source`, `imu_source`, `router`, the `wp` shell, `ble.c`, `ble_hog.c`, power, battery, LED.

## 3. Hardware test plan

Run in milestone order and record results with date, firmware commit and host OS. A step passes only if every pass criterion holds. Tooling: USB serial console, logic analyser (>= 4 channels, >= 10 MHz), a Nordic PPK2 for current, a BLE sniffer (for example nRF Sniffer for Bluetooth LE) if you want on-air timing, a multimeter.

### M1 bring-up

| Step | Action | Pass criterion |
|---|---|---|
| 1.1 | Wiring check per [wiring.md](wiring.md) section 7 (FFC pin 1, pull-ups, 3.3 V on VCC and VDDHI) | Continuity and voltages as listed; no short between VCC and GND |
| 1.2 | Double-tap RST, drive `NICENANO` appears, copy `zephyr.uf2` | Drive disappears and the board reboots; bootloader still reachable by double-tap afterwards |
| 1.3 | Open the CDC ACM port | Boot log appears; `wp status` works |
| 1.4 | `i2c scan i2c@40003000` | 0x74 (TPS65) and 0x6a or 0x6b (LSM6DSOX) listed. If 0x6b, change the overlay (wiring.md). |
| 1.5 | LED | Advertising pattern after boot (200 ms on, 800 ms off) |
| 1.6 | I2C bus quality (logic analyser or scope on SDA/SCL) | Clean edges at 400 kHz with the fitted pull-ups; otherwise add 4.7 kOhm or drop to 100 kHz |

### M2 touchpad

| Step | Action | Pass criterion |
|---|---|---|
| 2.1 | RDY activity (analyser on RDY, D8) while touching | RDY pulses during touch, idle when untouched |
| 2.2 | Touch frame rate: `wp touch`, touch continuously for 5 s, run it again | `touch` rate printed in Hz; record it. It must be steady and not zero. (The target rate is a result to record, not an assumption.) |
| 2.3 | Axis mounting per [calibration.md](calibration.md) section 1 | All four directions correct |
| 2.4 | Multi-touch | `wp status` `contacts` follows 1 to 5 fingers; ids stable while fingers stay down |
| 2.5 | Gestures: tap, double tap, tap-drag, long-press drag, right click, scroll both axes | Each produces the expected host event; no click on a slow touch; no accidental right click on scroll |
| 2.6 | Cursor latency, method A: analyser on RDY (D8) against a BLE sniffer capture of the HID notification | Time from RDY edge to the notification on air; record median and maximum per connection interval |
| 2.7 | Cursor latency, method B: `wp status` `max latency` (capture to report hand-off inside the firmware) after 60 s of continuous movement | Record the value; it excludes BLE connection interval and host. Reset the maximum by rebooting. |
| 2.8 | Connection interval actually used | With the sniffer, the interval matches the request (15 ms) on each host, or record what the host applies |

No numeric latency target is defined in this repository. Set one after the first measurements.

### M3 IMU

| Step | Action | Pass criterion |
|---|---|---|
| 3.1 | `wp imu` flat, face up, still | a_z about +9.8, a_x and a_y near 0 |
| 3.2 | Raise each edge (calibration.md section 2) | Sign table matches for top, bottom, left, right |
| 3.3 | Gyro sign: rotate slowly counter-clockwise seen from above | Positive z rate in the body frame |
| 3.4 | `wp bias` after 2+ s of stillness with the gyro on | Bias printed; stays stable over 5 minutes; survives reboot |
| 3.5 | Disconnect the IMU (unplug INT/SDA) while running | State goes to ERROR_RECOVERY or the IMU is reported absent; the touchpad keeps working; recovery after reconnect |
| 3.6 | Unplug the TPS65 while running | ERROR_RECOVERY, retries with back-off, recovers when plugged in again |

### M4 orientation

| Scenario | Pass criterion |
|---|---|
| Raise each edge, hold 1 s | Orientation 0/270/180/90 as in calibration.md; pointer and scroll follow |
| Pad flat, rotate on the table | No orientation change |
| Pad flat, manual calibration gesture in each of the 4 directions | Correct orientation and LED blink count; scroll was inhibited while armed; no accidental trigger by 20 two-finger scrolls |
| **Orientation change during an active gesture:** hold a finger down (drag), raise another edge | No change while the finger is down; change applied after lift plus settle time; no cursor jump |
| Pad carried and put down | No spurious orientation change |
| Manual lock modes 0, 1, 2 | Behaves as described in calibration.md section 4.2 |
| Algorithm D (`CONFIG_WAFER_PAD_ORIENT_GYRO_ASSIST=y`) | Experimental: record behaviour, do not use at the booth unless it passes repeatedly |

### M5 air mouse

| Step | Pass criterion |
|---|---|
| Lift the pad and hold it | `wp status` shows PICKUP_CANDIDATE, then HANDHELD, within about 0.5 s |
| Bump the pad on the table | Returns to TRACKPAD without entering HANDHELD |
| One finger still for 300 ms while handheld | AIR_MOUSE (LED solid) |
| Turn left / tilt the forward edge up | Cursor left / up |
| Hold still for 30 s | No cursor creep beyond the deadzone |
| Add a second finger or lift the finger | Leaves AIR_MOUSE, no button stuck |
| Put the pad down | TRACKPAD after about 1.2 s |
| Same air mouse motion in the four orientations | Same cursor direction |
| `wp mode handheld` and `wp mode trackpad` | Forced behaviour holds; `wp mode auto` restores |

### M6 BLE, polish and power

| Step | Pass criterion |
|---|---|
| Pairing with Windows, macOS and Linux (BlueZ) | Pairs on first attempt within the window; works as a mouse; record host versions |
| Reconnect: pad power cycle, host sleep/wake, host Bluetooth toggle | Reconnects without user action; time to first cursor movement recorded |
| Second host while bonded and window closed | Rejected. With `wp pair` accepted. |
| Fifth host with `BT_MAX_PAIRED=4` | Oldest bond dropped, others unaffected |
| Host that "forgot" the pad | Pad re-pairs only while the window is open |
| Legacy-only host | Rejected (Secure Connections required) |
| Range and interference at the booth (many 2.4 GHz devices) | Record dropouts per minute; compare connection interval 15 ms against a longer one |
| **Stuck buttons:** drop the link mid-drag; reboot mid-drag; lift fingers during tap-drag; cancel a long-press drag | No button remains held on the host in any case |
| Error recovery | TPS65 and IMU unplug/replug recover; a BLE failure reboots the pad and it advertises again |
| Battery | `wp status` battery voltage plausible against a multimeter at BAT+ |

### Power measurement (Nordic PPK2)

Measure; do not assume numbers. Power the nice!nano through the PPK2 in source or ampere mode with the sensors fitted, no USB, and record average current over at least 60 s per state. Compare with datasheet figures only afterwards.

| State | How to reach it | Record |
|---|---|---|
| TRACKPAD, connected, no touch | Pad on the table, BLE connected | Average and peak current |
| TRACKPAD, continuous touch | Moving finger | Average current |
| HANDHELD / AIR_MOUSE | Pad held, finger still | Average current (gyro on) |
| IDLE | No touch for `SM_IDLE_TIMEOUT_MS` (30 s) | Average current, connected with power-saving parameters |
| IDLE, BLE disconnected | Host off | Average current (advertising) |
| System OFF | Build with `CONFIG_WAFER_PAD_SM_SLEEP_ENABLED=y` and a short `CONFIG_WAFER_PAD_SM_SLEEP_TIMEOUT_MS` | Current, then touch wake and (with `CONFIG_WAFER_PAD_IMU_WAKEUP=y`) motion wake. Remember the sensor rail stays on, so the floor is set by the TPS65 and IMU. |

Also check the wake cause log line after the wake ("wake from System OFF") and that the pad reconnects.

### Exhibition soak test

Run the final configuration (same Kconfig, enclosure, host and table as the booth) for at least 8 hours, longer than a show day, in a busy 2.4 GHz environment if possible.

1. Script or manually exercise the pad hourly: tap, scroll, drag, pick up, air mouse, put down.
2. Log the console output (serial capture to a file) and note every ERROR_RECOVERY, unexpected reboot, disconnect and stuck button.
3. Leave it idle for 1 hour, then touch: it must respond immediately and reconnect if it had disconnected.
4. Run `wp status` at start and end: `drops` and `max latency` recorded; no growth of `queue_drops` while idle.
5. Pass criteria: no unexpected reboot, no stuck button, no loss of pairing, no manual intervention needed. Anything else goes on a defect list before the show.
6. Keep a spare pad (TPS65 is end-of-life) and a laptop with this firmware's UF2 at the booth.
