# Wafer Pad architecture

Scope: how the firmware is organised and why. Hardware facts are in [wiring.md](wiring.md). Nothing here has been verified on hardware; see [testing.md](testing.md).

## 1. Layers

```
 hardware           sources              router             algorithms            HID composer      transport
+-----------+     +--------------+     +-----------+     +-----------------+     +-------------+   +-----------+
| TPS65     |---->| touch_source |---->|           |---->| orientation     |     |             |   |           |
| (IQS5xx   |     | (+mount S->P)|     |           |     | transform       |     | hid_report  |   | BLE HOG   |
|  driver)  |     +--------------+     |  router   |     | gestures        |---->| composer    |-->| (ble.c,   |--> host
+-----------+                          |  thread   |     | pointer accel   |     | (accumulate,|   |  ble_hog) |
| LSM6DSOX  |     +--------------+     |  + shell  |     | air mouse       |     |  split,     |   |           |
| (Zephyr   |---->| imu_source   |---->|  + ctl    |     | state machine   |     |  buttons)   |   | hid_      |
|  sensor)  |     | (+mount, bias)|    |  work q   |     | calib gesture   |     +-------------+   | transport |
+-----------+     +--------------+     +-----------+     +-----------------+                       +-----------+
                                            ^ post_touch / post_imu / post_hid_* / post_key (future)
```

| Layer | Files | Rule |
|---|---|---|
| Driver | `drivers/azoteq/iqs5xx.c`, Zephyr `st,lsm6dso` | Talks to the chip only. Own work queue for the IQS5xx. |
| Sources | `src/touch/touch_source.c`, `src/imu/imu_source.c` | Convert to normalized frames/samples (types.h), apply mounting, post to the router. |
| Router | `src/router/router.c`, `router_shell.c` | Single thread that owns all algorithm state. Everything arrives as a message in one queue. |
| Algorithms | `src/orientation`, `src/gestures`, `src/air_mouse`, `src/state`, `src/imu/imu_algo.c`, `src/touch/touch_tracker.c` | Pure C, no Zephyr includes in their headers, unit-tested on native_sim. |
| HID composer | `src/hid/hid_report.c`, `include/wafer_pad/hid_report.h` | Fractional counts to integer reports, button state, splitting of large motion. |
| Transport | `src/hid/ble.c`, `ble_hog.c`, `hid_transport.c` | BLE HID over GATT only. The composer is transport-independent (USB HID is not implemented). |

The interface between sources, router, HID and power is [include/wafer_pad/platform.h](../include/wafer_pad/platform.h).

## 2. Data flow

```
 touch frame (S) --> mount swap/invert --> pad frame P --+
                                                         |
 IMU sample --> body-frame map --> bias correction ------+--> router queue (depth CONFIG_WAFER_PAD_ROUTER_QUEUE_DEPTH, 24)
                                                         |
 BLE events, ctl requests -------------------------------+
                                                         v
   router thread:
     P frame --> calibration gesture (needs P, not transformed)
             --> orientation transform P -> U --> gesture engine --> pointer / scroll / clicks
     IMU     --> motion features, gravity estimator --> orientation manager
             --> state machine (inputs: contacts, single-still, motion)
     state   --> outputs: gyro_needed, imu_low_rate, touch_low_power,
                 pointer_from_touch, air_mouse_active, ble_low_latency, request_sleep
     AIR_MOUSE: gyro rate --> air mouse --> pointer (touch pointer disabled)
     --> HID composer --> bt_gatt_notify (max CONFIG_WAFER_PAD_BLE_MAX_INFLIGHT = 2)
```

The router also measures, per touch frame, the time from capture to report hand-off (`max latency` in `wp status` and `wp touch`, field `max_latency_us`). It is not the on-air latency.

## 3. Threads and work queues

| Context | Priority | Stack | Source |
|---|---|---|---|
| IQS5xx work queue `iqs5xx_wq` | `CONFIG_AZOTEQ_IQS5XX_WORKQ_PRIORITY` = 2 | `CONFIG_AZOTEQ_IQS5XX_WORKQ_STACK_SIZE` = 1536 | drivers/azoteq/Kconfig |
| Router thread | `CONFIG_WAFER_PAD_ROUTER_PRIORITY` = 4 (preemptible) | `CONFIG_WAFER_PAD_ROUTER_STACK_SIZE` = 4096 | src/router/Kconfig |
| IMU thread | `CONFIG_WAFER_PAD_IMU_THREAD_PRIORITY` = 5 (preemptible) | `CONFIG_WAFER_PAD_IMU_THREAD_STACK_SIZE` = 2048 | src/imu/Kconfig |
| Router control work queue | router priority + 2 = 6 (preemptible, `K_PRIO_PREEMPT(...)` in router.c) | 3072 (`CTL_WQ_STACK_SIZE`, fixed in router.c) | touch/IMU re-init, power modes, bias save |
| System work queue | Zephyr default | `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE` = 3072 | BT host TX completions, bond storage, LED, battery |
| main | Zephyr default | `CONFIG_MAIN_STACK_SIZE` = 2048 | bring-up, then idle |

The IMU thread polls at `CONFIG_WAFER_PAD_IMU_ODR_HZ` (104 Hz) and at `CONFIG_WAFER_PAD_IMU_LOW_RATE_HZ` (26 Hz) in IDLE. The router has a housekeeping tick of `CONFIG_WAFER_PAD_ROUTER_TICK_MS` (10 ms) so that timers (long press, tap deferral, state machine) advance without frames. Stack headroom has not been measured on target.

## 4. Coordinate frames

Authoritative definition: [include/wafer_pad/types.h](../include/wafer_pad/types.h). Summary:

| Frame | Meaning |
|---|---|
| S (sensor) | Raw controller counts |
| P (pad) | S after the fixed swap/invert mounting correction. +x right edge, +y bottom edge, origin top-left. The edge at y = 0 is the reference ("top") edge. |
| B (body, 3D) | x_B = +x_P, y_B = -y_P (towards the top edge), z_B out of the surface. IMU axes are mapped into B by `CONFIG_WAFER_PAD_IMU_MOUNT_*`. Face up at rest: accel = (0, 0, +g). |
| U (user) | What the user perceives: +x right, +y towards the user (HID +Y). U = R(orientation) * P. |

Orientation is the clockwise rotation of the pad, seen from above, relative to the posture where the reference edge faces away from the user:

| Orientation | Edge facing away from user | Delta P to U |
|---|---|---|
| 0 | top (y = 0) | (dx, dy) |
| 90 | left (x = 0) | (-dy, dx) |
| 180 | bottom | (-dx, -dy) |
| 270 | right | (dy, -dx) |

## 5. Orientation

Physical limitation: **a 6-axis IMU has no yaw reference.** With the pad flat, gravity says nothing about which edge faces the user. Automatic detection therefore only works while the pad is tilted, under the assumption "the raised edge points away from the user".

| Algo | Name | Behaviour |
|---|---|---|
| A | Gravity | Heading = atan2(-a_x, a_y) from the in-plane gravity component. Accepted when tilt is between `ORIENT_MIN_TILT_DEG` (20) and `ORIENT_MAX_TILT_DEG` (75), acceleration magnitude is within `ORIENT_MAX_MAG_DEV_MILLI_G` (120) of 1 g, rotation is below `ORIENT_MAX_GYRO_DPS` (20), hysteresis `ORIENT_HYSTERESIS_DEG` (15), confidence at least `ORIENT_MIN_CONFIDENCE_CENTI` (60), and the candidate is stable for `ORIENT_CONFIRM_MS` (300; 800 ms while the gyro is off). |
| B | Lock | When flat or being moved the last confident orientation is kept. A change is never applied while a finger is down: it is deferred until the last finger lifted plus `ORIENT_TOUCH_SETTLE_MS` (150) and only applied if the candidate is still confirmed. |
| C | Manual | The calibration gesture (or `wp orient`) sets the orientation. Lock policy `ORIENT_MANUAL_LOCK_MODE`: 0 none, 1 timed (default, `ORIENT_MANUAL_LOCK_MS` = 30000), 2 until the next calibration. During the lock, algorithm A cannot override it. |
| D | Gyro assist | Experimental, `CONFIG_WAFER_PAD_ORIENT_GYRO_ASSIST` default off. After a confident fix, yaw about the pad normal is integrated; each `ORIENT_GYRO_STEP_DEG` (70) of accumulated yaw (within `ORIENT_GYRO_MAX_WINDOW_MS`, below `ORIENT_GYRO_MAX_TILT_DEG`) makes one 90 degree step. |

All symbols are `CONFIG_WAFER_PAD_ORIENT_*`. Sources of the current orientation are reported by `wp status` (auto/gravity, manual, gyro). Algorithm A can be disabled with `CONFIG_WAFER_PAD_ORIENT_AUTO=n`; the boot orientation is `CONFIG_WAFER_PAD_ORIENT_DEFAULT_DEG`.

Calibration gesture details and tuning: [calibration.md](calibration.md).

## 6. State machine

Source: [include/wafer_pad/state_machine.h](../include/wafer_pad/state_machine.h), implementation `src/state/state_machine.c`.

```
 BOOT -> INITIALIZING --init ok--> TRACKPAD <------------------------------------+
              |                      |  ^                                        |
         init fail                   |  | bump (still putdown_still_ms)          |
              v                      v  |                                        |
        ERROR_RECOVERY <---- PICKUP_CANDIDATE                                     |
        (retry 500 ms,            | confirmed (tremor / motion)                   |
         doubling to 8 s)         v                                               |
              ^             HANDHELD --still + no tremor for handheld_putdown_ms--+
              |               |  ^
   any operational state      |  | contacts != 1
   --EV_ERROR-->              v  |
                           AIR_MOUSE --still + no tremor for handheld_putdown_ms--> TRACKPAD

 TRACKPAD --no touch, no motion for idle_timeout_ms (30 s)--> IDLE
 IDLE --contacts > 0 or motion--> TRACKPAD
 IDLE --sleep enabled and inactive for sleep_timeout_ms, or BLE disconnected for disconnected_sleep_ms--> SLEEP
 SLEEP: System OFF, wake = reset
 TRACKPAD / HANDHELD / AIR_MOUSE --calibration armed--> CALIBRATING --done / cancel / timeout--> state it came from
                                                        (AIR_MOUSE returns to HANDHELD)
```

| Transition | Condition (Kconfig, default) |
|---|---|
| TRACKPAD to PICKUP_CANDIDATE | IMU motion "moving" while mode override is auto |
| PICKUP_CANDIDATE to HANDHELD | After `SM_PICKUP_CONFIRM_MS` (400), not still, and gyro hand tremor, or motion seen at least `SM_PICKUP_SETTLE_MS` (150) after entering |
| PICKUP_CANDIDATE to TRACKPAD | Still for `SM_PUTDOWN_STILL_MS` (300), or `SM_PICKUP_TIMEOUT_MS` (1500) without decision |
| HANDHELD to AIR_MOUSE | One finger still for `SM_AIR_HOLD_MS` (300), `SM_AIR_MOUSE_ENABLED` y |
| HANDHELD / AIR_MOUSE to TRACKPAD | Still and no hand tremor for `SM_HANDHELD_PUTDOWN_MS` (1200) |
| TRACKPAD to IDLE | `SM_IDLE_TIMEOUT_MS` (30000) |
| IDLE to SLEEP | `SM_SLEEP_ENABLED` (default n) and `SM_SLEEP_TIMEOUT_MS` (900000), or `SM_DISCONNECTED_SLEEP_MS` (0 = never) |
| CALIBRATING timeout | `SM_CALIB_TIMEOUT_MS` (5000) |
| Error retry | `SM_ERROR_RETRY_MS` (500) doubling to `SM_ERROR_RETRY_MAX_MS` (8000) |

"Flat" is never treated as "on the desk": put-down is recognised from the absence of hand tremor and motion. A hand holding the cursor steady is still but shows tremor. The knock that triggers PICKUP_CANDIDATE must not confirm the pickup by itself (settle time).

Mode override (cycled by a three-finger tap, or `wp mode`): auto, force handheld, force trackpad. It is the fallback if automatic pickup detection proves unreliable on hardware. LED blinks 1/2/3 times for auto/handheld/trackpad.

Per-state outputs: the gyro is powered only when needed (PICKUP_CANDIDATE, HANDHELD, AIR_MOUSE, or always with `CONFIG_WAFER_PAD_SM_GYRO_ALWAYS_ON`). In IDLE the IMU runs at the low rate and the touch controller is put in low-power mode. BLE low latency is requested in TRACKPAD, PICKUP_CANDIDATE, HANDHELD, AIR_MOUSE and CALIBRATING.

## 7. Gesture engine

Input: user-frame touch frames. Everything is computed in millimetres from the pad geometry (`CONFIG_WAFER_PAD_TOUCH_WIDTH_CENTI_MM` 6500, `_HEIGHT_CENTI_MM` 4900), so thresholds do not depend on the controller.

| Gesture | Result |
|---|---|
| 1 finger move | Pointer, with jitter deadband, speed-dependent smoothing and acceleration (`WAFER_PAD_POINTER_*`, `WAFER_PAD_GEST_*`) |
| 1 finger tap | Left click. Deferred by the tap-drag window so a tap-drag never sends "click + press". |
| Tap, then touch and move within `GEST_DOUBLE_TAP_WINDOW_MS` | Left-button drag |
| 1 finger long press, then move | Left-button drag (armed after `GEST_LONG_PRESS_MS`, button pressed on movement) |
| 2 finger tap | Right click |
| 2 finger move | Scroll (wheel and pan), axis lock, natural scrolling by default |
| 3 finger tap | Event used as mode toggle |
| 3 fingers hold, then deliberate swipe | Orientation calibration (algorithm C) |

Safety rules: motion is re-anchored whenever the finger set changes; a drag button is always released when the last finger lifts, on reset, and on contact loss.

## 8. Air mouse

Cursor motion is proportional to the angular rate. There is no linear displacement estimate and **no double integration of the accelerometer**.

```
 gyro (rad/s, body) --> axis projection --> 1-pole LPF --> soft deadzone --> clamp --> gain --> ramp-in --> counts
   yaw   about "up" (gravity-compensated by default)  -> cursor X  (dx = -yaw_rate   * gain)
   pitch about the horizontal right axis              -> cursor Y  (dy = -pitch_rate * gain)
```

The forward edge for the current orientation comes from the orientation module, so the air mouse behaves the same in all four orientations. While the air mouse is active, the single finger is a clutch and the touch pointer is off. Parameters: `CONFIG_WAFER_PAD_AIR_*` (see [calibration.md](calibration.md)). Gravity compensation (`AIR_GRAVITY_COMP`, default y) uses a low-passed accelerometer estimate with falls back to uncompensated axes in free fall.

## 9. HID report layout

The descriptor is in `src/hid/hid_report.c`; packing in [include/wafer_pad/hid_report.h](../include/wafer_pad/hid_report.h).

| Report ID | Content | Status |
|---|---|---|
| 1 | Mouse input report, 7 bytes payload | Implemented |
| 2 | Keyboard | Reserved, not in the descriptor |
| 3 | Consumer | Reserved, not in the descriptor |

Report 1 payload:

| Byte | Field |
|---|---|
| 0 | Buttons: bit0 left, bit1 right, bit2 middle, bit3 back, bit4 forward, bits 5-7 padding |
| 1-2 | X, int16 little endian, relative, -32767..32767 |
| 3-4 | Y, int16 little endian, relative, +down |
| 5 | Wheel, int8, +up |
| 6 | AC Pan, int8, +right |

The composer accumulates fractional counts, cancels sub-count jitter, and splits motion larger than the field range over several reports instead of clipping.

## 10. BLE behaviour

| Topic | Behaviour |
|---|---|
| Role | Peripheral, HID over GATT, appearance mouse (962), Device Information and Battery services |
| Pairing | Just Works with LE Secure Connections only. New pairings only while the pairing window is open (`CONFIG_WAFER_PAD_BLE_PAIRING_WINDOW_S` = 120 s after boot, or `wp pair`); while there is no bond pairing is always allowed. The window closes after one host paired. |
| Bonds | Up to `CONFIG_BT_MAX_PAIRED` = 4, stored via settings/NVS (storage partition). The least recently used bond is dropped for a fifth host. `wp unpair` removes all. |
| Link readiness | A connection that does not become "HID ready" within 60 s (`WP_BLE_READY_TIMEOUT_S` in ble.c) is disconnected. |
| Connection parameters | Low latency (active states): 15 ms interval, latency 0 (`WAFER_PAD_BLE_LL_*`). Power saving (IDLE): 30-60 ms, latency 4 (`WAFER_PAD_BLE_PS_*`). Supervision timeout 6000 ms. Parameters not applied by the central are re-requested with back-off. Which parameters real hosts accept is untested. |
| Flow control | At most `WAFER_PAD_BLE_MAX_INFLIGHT` = 2 notifications in flight, tracked per connection generation. The router never waits for an ATT buffer (`CONFIG_BT_ATT_TX_COUNT=5`). If no completion arrives for longer than the supervision timeout on a link that stays up, the link is dropped. |
| Failure recovery | A failed BLE bring-up reboots the pad (bounded, main.c). |
| Advertising | 30-60 ms interval (`WAFER_PAD_BLE_ADV_INTERVAL_*_US`). |

## 11. Power

| State | What runs |
|---|---|
| TRACKPAD / HANDHELD / AIR_MOUSE | Touch active, IMU at 104 Hz, BLE low latency. Gyro on in PICKUP_CANDIDATE, HANDHELD and AIR_MOUSE only (unless `SM_GYRO_ALWAYS_ON`). |
| IDLE | Touch low-power mode, IMU at 26 Hz, gyro off, BLE power-saving parameters. |
| SLEEP (System OFF) | Entered only if `CONFIG_WAFER_PAD_SM_SLEEP_ENABLED=y` (default n). Wake sources: touch RDY pin (always), IMU INT1 (only with `CONFIG_WAFER_PAD_IMU_WAKEUP=y`, threshold `WAFER_PAD_IMU_WAKEUP_THS_MILLI_G` = 63). If no wake source can be armed the pad does not enter System OFF. Wake is a reset. |

The 3.3 V sensor rail (`ext_vcc`, nice!nano P0.13 LDO) is `regulator-always-on` and is **never cut**, including in System OFF, so the trackpad can wake the MCU. Battery voltage is read on SAADC `VDDHDIV5` every `CONFIG_WAFER_PAD_BATTERY_INTERVAL_S` (60) and published to the Battery Service. Power figures are not measured; measure them with the plan in [testing.md](testing.md).

## 12. Future: Wafer keyboard dongle

The pad is the first producer of a planned Wafer keyboard system. Reserved hooks, none functional yet:

| Hook | Where | State |
|---|---|---|
| `struct wp_key_event` (source local/left/right half, usage page 0x07 or 0x0C, usage, pressed, time) and `enum wp_key_source` | include/wafer_pad/types.h | Defined |
| `wp_router_post_key()` | include/wafer_pad/platform.h, src/router/router.c | Stub, returns `-ENOTSUP`. The router message union already has a key member. |
| HID report IDs 2 (keyboard) and 3 (consumer) | include/wafer_pad/hid_report.h | Reserved, not in the descriptor |
| Transport abstraction | src/hid/hid_transport.c, composer shared by transports | Mouse only; BLE HOG is the only transport |

Plan for the integration point: a keyboard producer (for example a BLE central talking to keyboard halves, or a local matrix scanner) calls `wp_router_post_key()`. The router hands keyboard usages to a new keyboard composer for report ID 2 and consumer usages to one for report ID 3, flushed next to the mouse report. The HID descriptor then needs the two new collections; that changes the descriptor every bonded host has cached, so hosts must re-pair or refresh the GATT database.

**The split-keyboard BLE protocol must be designed and validated separately.** ZMK split peripherals speak ZMK's own GATT split service. They will not automatically work with a custom Zephyr central; either that service has to be reimplemented and tested against the ZMK versions in use, or a new protocol has to be defined for both ends. None of that exists in this repository.
