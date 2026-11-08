# Wafer Pad bring-up calibration

Do these steps in order on the first assembled prototype. Nothing here has been done on hardware yet; each step says what to expect, and the expected values are derived from the code, not measured. Wiring first: [wiring.md](wiring.md) section 7.

Prerequisites: firmware flashed ([README](../README.md)), USB console open, `i2c scan i2c@40003000` shows 0x74 and 0x6a (or 0x6b), pad paired or at least advertising.

Changing a Kconfig value means editing [prj.conf](../prj.conf) (or `west build ... -- -DCONFIG_NAME=value`) and reflashing. Names below omit the `CONFIG_` prefix only where stated.

## 1. Touch axis mounting

Symbols: `CONFIG_WAFER_PAD_TOUCH_SWAP_XY`, `CONFIG_WAFER_PAD_TOUCH_INVERT_X`, `CONFIG_WAFER_PAD_TOUCH_INVERT_Y` (all default n). They map the controller's axes (sensor frame S) into the pad frame P: swap first, then invert.

Goal: in orientation 0 the reference ("top") edge is the edge at P y = 0, +x points to the right edge, +y to the bottom edge. Pick the reference edge physically (for example the edge opposite the FPC) and mark it.

1. Pair the pad to a host, or watch `wp status` (`contacts`) while touching.
2. Force orientation 0: `wp orient 0`. Hold the pad with the reference edge away from you, pad flat on the table.
3. Move one finger slowly:

| Finger movement (reference edge away from you) | Cursor must move | If wrong |
|---|---|---|
| Left to right | Right | Invert X |
| Away from you | Up | Invert Y |
| Left to right moves the cursor up/down | (axes exchanged) | Enable SWAP_XY, then re-check X and Y |

4. Apply the fix, rebuild, repeat. Check all four directions plus a diagonal.
5. Check the corners: the cursor delta must not jump at the pad edges.

If the pad physical size differs from the defaults, set `WAFER_PAD_TOUCH_WIDTH_CENTI_MM` (6500) and `WAFER_PAD_TOUCH_HEIGHT_CENTI_MM` (4900); the sizes refer to the sensor X and Y axes before the swap. Gesture thresholds are in millimetres and depend on these.

## 2. IMU mounting map

Symbols: `CONFIG_WAFER_PAD_IMU_MOUNT_X`, `_Y`, `_Z`. Each is a signed IMU axis index: body = sign * imu[|n| - 1] (1 = IMU x, 2 = y, 3 = z). The three absolute values must be a permutation of 1, 2, 3. Defaults 1, 2, 3 (identity).

Body frame (types.h): x towards the right edge, y towards the top (reference) edge, z out of the touch surface.

Procedure with `wp imu` (prints the body-frame sample after mapping, tilt and heading):

1. Lay the pad flat, face up, still. Expect a_z about +9.8 m/s^2, a_x and a_y near 0.
2. Raise one edge by about 30 to 45 degrees (the opposite edge stays on the table), hold still, run `wp imu`. Expected sign:

| Raised edge | Axis that goes positive | Axis that goes negative |
|---|---|---|
| Top (reference) edge | a_y > 0 | |
| Bottom edge | | a_y < 0 |
| Right edge | a_x > 0 | |
| Left edge | | a_x < 0 |

   In every case a_z stays positive (it shrinks with cos of the tilt). The shell prints "raise an edge: that axis goes +".
3. If an axis is wrong, change the mapping. Examples: if raising the top edge makes a_x positive, then body y = IMU x, so set `IMU_MOUNT_Y=1` and choose `IMU_MOUNT_X` so the three absolute values stay a permutation of 1, 2, 3. If the sign is inverted, use a negative index (for example `IMU_MOUNT_Y=-2`).
4. If z is negative while face up, the sensor is mounted upside down: `IMU_MOUNT_Z=-3` (mirroring one axis changes handedness, so another axis must flip as well; verify all three edges again).
5. Also check the gyro sign conventions with a slow rotation: turning the pad counter-clockwise seen from above must give a positive gyro z in the body frame (right-hand rule about +z out of the surface). The air-mouse unit tests assume this convention; a wrong mapping shows up as an inverted cursor in section 5.
6. `wp status` should then show `heading` equal to the raised-edge direction: 0 deg top edge raised, 90 left, 180 bottom, 270 right (`wp imu` also prints the raised edge name).

## 3. Gyro bias

The gyro offset is estimated automatically while the pad is still: no calibration command exists.

| Item | Value (Kconfig default) |
|---|---|
| Still criterion | gyro within `IMU_BIAS_STILL_GYRO_CENTI_DPS` (100 = 1 dps) of the estimate and dynamic accel below `IMU_BIAS_STILL_ACCEL_DEV_MILLI_G` (30) |
| Still time before estimating | `IMU_BIAS_STILL_MIN_MS` (800) |
| Filter | EMA, `IMU_BIAS_ALPHA_MILLI` (20 per mille per still sample) |
| Plausibility limit | estimates above `IMU_BIAS_MAX_DECI_DPS` (100 = 10 dps) are rejected |
| Persistence | `IMU_BIAS_PERSIST` y, settings key `wp/imu/bias`, saved at most every `IMU_BIAS_SAVE_INTERVAL_S` (60) and only if changed by `IMU_BIAS_SAVE_MIN_CHANGE_MILLI_DPS` (50) |

(All symbols are `CONFIG_WAFER_PAD_*`.)

Check: the gyro is only on in PICKUP_CANDIDATE, HANDHELD and AIR_MOUSE (or always with `CONFIG_WAFER_PAD_SM_GYRO_ALWAYS_ON=y`). For the bring-up, enable that option so `wp bias` works at the desk, then:

1. Put the pad on a table, do not touch it for at least 2 s.
2. `wp bias`: prints `gyro bias [mdps] x .. y .. z ..` in the body frame, or "not estimated yet". Record the numbers; they are the baseline for judging drift.
3. Reboot and run `wp bias` again: the persisted value must come back (it is loaded from settings; the estimator re-learns it when the pad is still, so a wrong stored value is corrected).
4. Pick up the pad, hold the cursor with a still finger (air mouse): the cursor must not creep. If it does, the bias is wrong or the deadzone is too small (section 5).

## 4. Orientation

### 4.1 Automatic tilt detection

Algorithm A assumes the raised edge points away from the user. Procedure:

1. `wp orient auto`. Check `wp status`: `orientation ... (gravity ...)` after the first fix.
2. Raise the edge you want as "away": at least `ORIENT_MIN_TILT_DEG` (20) and at most `ORIENT_MAX_TILT_DEG` (75). Hold it for `ORIENT_CONFIRM_MS` (300) without rotating faster than `ORIENT_MAX_GYRO_DPS` (20 dps). (With the gyro off the time is `ORIENT_CONFIRM_NO_GYRO_MS`, 800.)
3. `wp status` shows the new `orientation`. Pointer and scroll must now follow the user's frame: finger away from you moves the cursor up.
4. Rotate the pad 90 degrees on the table, flat: nothing changes (no yaw reference). Raise the new far edge: it changes.

| Tuning | Symbol | Default | Effect |
|---|---|---|---|
| Too sensitive to small tilts | `ORIENT_MIN_TILT_DEG` | 20 | raise |
| Slow to change | `ORIENT_CONFIRM_MS` | 300 | lower |
| Flips at ambiguous corners | `ORIENT_HYSTERESIS_DEG` | 15 | raise |
| Rejected while sliding the pad | `ORIENT_MAX_MAG_DEV_MILLI_G`, `ORIENT_MAX_GYRO_DPS` | 120, 20 | raise |

Lock behaviour: when the pad is flat or moving, the last orientation is kept. An orientation change never happens while a finger is down; it is applied after the last finger lifted plus `ORIENT_TOUCH_SETTLE_MS` (150) and only if still confirmed. Do the "change during an active gesture" check on hardware (see [testing.md](testing.md)).

### 4.2 Manual calibration gesture

For a pad that lies flat (no tilt), set the orientation by hand.

Defaults (Kconfig, all `CONFIG_WAFER_PAD_CALIB_*`):

| Parameter | Symbol | Default |
|---|---|---|
| Fingers | `CALIB_FINGERS` | 3 |
| Hold time before arming | `CALIB_HOLD_MS` | 1500 ms |
| Max finger movement while holding | `CALIB_HOLD_MAX_MOVE_CENTI_MM` | 150 (1.5 mm) |
| Time to start the swipe after arming | `CALIB_ARM_TIMEOUT_MS` | 2500 ms |
| Movement that starts the swipe timer | `CALIB_SWIPE_START_CENTI_MM` | 200 (2 mm) |
| Minimum swipe distance | `CALIB_SWIPE_MIN_MM` | 25 mm |
| Maximum swipe duration | `CALIB_SWIPE_MAX_MS` | 450 ms |
| Max deviation from a pad axis | `CALIB_SWIPE_MAX_ANGLE_DEV_DEG` | 30 deg |
| Lift within | `CALIB_LIFT_MAX_MS` | 250 ms after the swipe |

Procedure: put 3 fingers down together (within `GEST_MULTI_DOWN_SKEW_MS`, 120 ms), keep them still for 1.5 s (the LED fast-blinks, scrolling is inhibited), then swipe all three fingers in one quick stroke towards the edge that is "away from you", at least 25 mm, and lift. The LED blinks 1 to 4 times for orientation 0, 90, 180, 270. A diagonal swipe, a slow swipe, adding or removing a finger or waiting longer than 2.5 s cancels it. The swipe direction maps as: towards the top edge gives 0, left edge 90, bottom edge 180, right edge 270.

Why 3 fingers and 1.5 s: the original idea (2 fingers, 700 ms hold) was changed because "rest two fingers, then flick-scroll" flipped the pad by 180 degrees. The unit test `test_rest_then_flick_scroll_never_calibrates` covers this. To restore the original behaviour: `CONFIG_WAFER_PAD_CALIB_FINGERS=2` and `CONFIG_WAFER_PAD_CALIB_HOLD_MS=700`. Expect accidental triggers while scrolling.

After a manual result the orientation is locked against algorithm A according to `ORIENT_MANUAL_LOCK_MODE` (0 none, 1 timed = default, 2 until the next calibration) and `ORIENT_MANUAL_LOCK_MS` (30000). For a booth with the pad always flat in front of the visitor, mode 2 is the stable choice.

`wp orient <0|90|180|270|auto>` does the same from the shell; `wp orient auto` returns to automatic.

## 5. Air mouse and pointer tuning

### Air mouse

Enter it by lifting the pad (state HANDHELD, see `wp status`), then put one finger down and keep it still for `SM_AIR_HOLD_MS` (300). Lifting the finger or adding a second one leaves AIR_MOUSE. If automatic pickup does not work, `wp mode handheld` forces it.

| Symbol | Default | Tuning |
|---|---|---|
| `WAFER_PAD_AIR_COUNTS_PER_DEG_CENTI` | 1200 (12 counts/deg) | Sensitivity. Lower if the cursor flies away. |
| `WAFER_PAD_AIR_DEADZONE_DECI_DPS` | 15 (1.5 dps) | Raise if the cursor creeps at rest, lower if small motions are lost. |
| `WAFER_PAD_AIR_LPF_CUTOFF_HZ` | 15 | Lower = smoother and laggier. 0 disables. |
| `WAFER_PAD_AIR_MAX_RATE_DPS` | 400 | Rate clamp. |
| `WAFER_PAD_AIR_RAMP_IN_MS` | 120 | Gain ramp after the clutch engages. |
| `WAFER_PAD_AIR_GRAVITY_COMP` | y | Gravity-compensated axes. |
| `WAFER_PAD_AIR_INVERT_X`, `_Y` | n | Fix inverted axes (also check the IMU mapping first). |

Check direction in the booth posture: turning the pad left moves the cursor left, tilting the forward edge up moves the cursor up. Unit-tested: `test_yaw_left_moves_cursor_left`, `test_pitch_forward_edge_up_moves_cursor_up`.

### Pointer

| Symbol | Default | Tuning |
|---|---|---|
| `WAFER_PAD_POINTER_COUNTS_PER_MM_CENTI` | 1600 | Base resolution |
| `WAFER_PAD_POINTER_SENSITIVITY_CENTI` | 100 | Overall gain |
| `WAFER_PAD_POINTER_ACCEL_LOW_SPEED` / `_HIGH_SPEED` | 20 / 200 mm/s | Acceleration ramp |
| `WAFER_PAD_POINTER_ACCEL_MAX_MULT_CENTI` | 300 | 100 turns acceleration off |
| `WAFER_PAD_GEST_JITTER_DEADBAND_CENTI_MM` | 30 | Dead band after touch-down |
| `WAFER_PAD_GEST_SMOOTHING_ALPHA_CENTI` | 60 | 100 = smoothing off |
| `WAFER_PAD_GEST_JUMP_MAX_STEP_MM` | 15 | A same-finger step larger than this in one frame is treated as a ghost jump and re-anchored (no cursor motion). Raise it if very fast flicks get swallowed; 0 disables. |
| `WAFER_PAD_GEST_JUMP_MAX_SPEED_MM_S` | 2000 | Same check by speed (fast human flicks peak around 1-1.5 m/s); 0 disables. |

### Taps and scroll

| Symbol | Default |
|---|---|
| `WAFER_PAD_GEST_TAP_MAX_MS` / `_TAP_MAX_MOVE_CENTI_MM` | 180 / 150 |
| `WAFER_PAD_GEST_DOUBLE_TAP_WINDOW_MS` | 250 |
| `WAFER_PAD_GEST_LONG_PRESS_MS` | 450 |
| `WAFER_PAD_GEST_TWO_FINGER_TAP_MAX_MS` | 220 |
| `WAFER_PAD_GEST_SCROLL_DETENTS_PER_MM_MILLI` | 250 |
| `WAFER_PAD_GEST_SCROLL_NATURAL` | y |
| `WAFER_PAD_GEST_SCROLL_AXIS_LOCK` | y |

Disable features with `WAFER_PAD_GEST_TAP`, `WAFER_PAD_GEST_TAP_DRAG`, `WAFER_PAD_GEST_LONG_PRESS_DRAG`, `WAFER_PAD_GEST_THREE_FINGER_TAP`.

## 6. Tuning at the booth: checklist

Do this on the actual table and host with the final enclosure, and record the values you changed.

- [ ] `wp status`: state TRACKPAD, `hid connected`, `imu present`, touch counters increasing, `drops 0`.
- [ ] `wp unpair` then pair the demo laptop; confirm reconnect after pad power cycle without touching the host.
- [ ] Touch axes correct in orientation 0 (section 1).
- [ ] IMU signs correct for all four edges (section 2).
- [ ] Decide the demo posture: pad flat on the table (use `wp orient` or the calibration gesture, lock mode 2) or tilted (automatic).
- [ ] Orientation: lift each edge, then verify pointer and scroll follow the user; verify no flips while scrolling on a flat pad.
- [ ] Calibration gesture works 5 of 5 times, and 10 two-finger scrolls with a pause beforehand never trigger it.
- [ ] Pick up the pad: HANDHELD, then AIR_MOUSE with a still finger; put it down: back to TRACKPAD within about 1.2 s.
- [ ] Air mouse does not drift for 30 s with a still finger; adjust deadzone and sensitivity.
- [ ] Tap, double tap, tap-drag, long-press drag, right click, scroll directions with the host's own "natural scrolling" setting.
- [ ] Stuck-button check: lift a finger mid-drag, disconnect/reconnect BLE mid-drag; no button remains pressed.
- [ ] Idle for `SM_IDLE_TIMEOUT_MS` (30 s), then touch: the pad wakes without a lost first movement.
- [ ] Decide whether System OFF stays disabled (`SM_SLEEP_ENABLED=n`, default).
- [ ] Pairing window: with a bond present, a second host must be rejected unless `wp pair` was issued.
- [ ] Note battery voltage in `wp status` if a cell is fitted.
