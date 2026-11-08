/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Device-level finite state machine (pure C, host-testable).
 *
 *   BOOT -> INITIALIZING -> TRACKPAD -> PICKUP_CANDIDATE -> HANDHELD
 *        -> AIR_MOUSE -> HANDHELD -> TRACKPAD -> IDLE -> SLEEP
 *   plus CALIBRATING and ERROR_RECOVERY.
 *
 * Transition table (evaluated in wp_sm_update(); events via wp_sm_event()):
 *
 *   BOOT            --EV_INIT_START-->                      INITIALIZING
 *   INITIALIZING    --EV_INIT_OK-->                         TRACKPAD
 *                   --EV_INIT_FAIL-->                       ERROR_RECOVERY
 *   ERROR_RECOVERY  --EV_INIT_OK / EV_RECOVERED-->          TRACKPAD
 *                   (request_reinit pulses on entry and every
 *                    error_retry_ms, doubling up to error_retry_max_ms)
 *   any operational --EV_ERROR-->                           ERROR_RECOVERY
 *
 *   TRACKPAD        --motion.moving (override AUTO)-->      PICKUP_CANDIDATE
 *                   --no touch & no motion for idle_timeout_ms--> IDLE
 *   PICKUP_CANDIDATE--still for putdown_still_ms-->         TRACKPAD   (bump)
 *                   --after pickup_confirm_ms, not still, and (GYRO hand
 *                     tremor, or moving seen in the last pickup_confirm_ms
 *                     but at least pickup_settle_ms after entering the
 *                     state)-->                             HANDHELD
 *                   --pickup_timeout_ms without decision--> TRACKPAD
 *   HANDHELD        --single finger still for air_hold_ms
 *                     (air_mouse_enabled)-->                AIR_MOUSE
 *                   --still AND no hand tremor for
 *                     handheld_putdown_ms-->                TRACKPAD
 *   AIR_MOUSE       --contacts != 1 -->                     HANDHELD
 *                   --still AND no hand tremor for
 *                     handheld_putdown_ms-->                TRACKPAD
 *   IDLE            --contacts > 0 or motion.moving-->      TRACKPAD
 *                   --sleep_enabled and inactive for sleep_timeout_ms,
 *                     or BLE disconnected for disconnected_sleep_ms-->  SLEEP
 *   SLEEP           terminal (request_sleep stays true; wake = reset)
 *   TRACKPAD/HANDHELD/AIR_MOUSE --EV_CALIB_ARMED--> CALIBRATING
 *   CALIBRATING     --EV_CALIB_DONE / EV_CALIB_CANCEL / calib_timeout_ms-->
 *                     state it came from (AIR_MOUSE returns to HANDHELD)
 *
 * CALIBRATING entered from a handheld state keeps the handheld context: the
 * gyro stays powered, long press stays disallowed, and accel-only samples
 * (motion.tremor_from_gyro == false) are not taken as stillness, so the
 * put-down timer cannot run up during the calibration and drop the device to
 * TRACKPAD right after the return.
 *
 * "Horizontal" is NOT treated as "on a desk": put-down is detected from the
 * absence of hand tremor/motion (stillness), never from tilt alone. A hand
 * holding the cursor steady is "still" (|w| below the still threshold) but
 * shows tremor, so it is never taken for a put-down.
 *
 * Knock rejection: the knock that triggers PICKUP_CANDIDATE (and the gyro
 * power-up after it) must not confirm the pickup by itself. Motion only
 * counts after pickup_settle_ms, accel-jitter "tremor" (gyro off) never
 * counts, and pickup_confirm_ms is longer than the motion module's stillness
 * window (still_window_ms, 300 ms) so that the bump branch can win.
 *
 * Mode override (EV_MODE_TOGGLE cycles AUTO -> FORCE_HANDHELD ->
 * FORCE_TRACKPAD -> AUTO) is the explicit fallback if automatic pickup
 * detection proves unreliable:
 *   FORCE_HANDHELD: TRACKPAD/PICKUP_CANDIDATE immediately become HANDHELD and
 *                   the put-down transition is disabled.
 *   FORCE_TRACKPAD: HANDHELD/AIR_MOUSE/PICKUP_CANDIDATE return to TRACKPAD and
 *                   pickup detection is disabled.
 */

#ifndef WAFER_PAD_STATE_MACHINE_H_
#define WAFER_PAD_STATE_MACHINE_H_

#include <wafer_pad/types.h>
#include <wafer_pad/imu_algo.h>

#ifdef __cplusplus
extern "C" {
#endif

enum wp_state {
	WP_STATE_BOOT = 0,
	WP_STATE_INITIALIZING,
	WP_STATE_TRACKPAD,
	WP_STATE_PICKUP_CANDIDATE,
	WP_STATE_HANDHELD,
	WP_STATE_AIR_MOUSE,
	WP_STATE_IDLE,
	WP_STATE_SLEEP,
	WP_STATE_CALIBRATING,
	WP_STATE_ERROR_RECOVERY,
	WP_STATE_COUNT,
};

enum wp_sm_event {
	WP_SM_EV_INIT_START = 0,
	WP_SM_EV_INIT_OK,
	WP_SM_EV_INIT_FAIL,
	WP_SM_EV_ERROR,
	WP_SM_EV_RECOVERED,
	WP_SM_EV_BLE_CONNECTED,
	WP_SM_EV_BLE_DISCONNECTED,
	WP_SM_EV_CALIB_ARMED,
	WP_SM_EV_CALIB_DONE,
	WP_SM_EV_CALIB_CANCEL,
	WP_SM_EV_MODE_TOGGLE,
};

enum wp_mode_override {
	WP_MODE_AUTO = 0,
	WP_MODE_FORCE_HANDHELD,
	WP_MODE_FORCE_TRACKPAD,
};

struct wp_sm_config {
	uint32_t pickup_confirm_ms;     /**< Default 400 (> still window + 100 ms). */
	uint32_t pickup_settle_ms;      /**< Motion in the first ms of PICKUP_CANDIDATE is the
					 *   triggering transient. Default 150. */
	uint32_t pickup_timeout_ms;     /**< Default 1500. */
	uint32_t putdown_still_ms;      /**< PICKUP_CANDIDATE -> TRACKPAD. Default 300. */
	uint32_t handheld_putdown_ms;   /**< HANDHELD/AIR_MOUSE -> TRACKPAD. Default 1200. */
	bool air_mouse_enabled;         /**< Default true. */
	uint32_t air_hold_ms;           /**< Still finger time to engage air mouse. Default 300. */
	uint32_t idle_timeout_ms;       /**< Default 30000. */
	bool sleep_enabled;             /**< Default false for the exhibition build. */
	uint32_t sleep_timeout_ms;      /**< Inactivity in IDLE before SLEEP. Default 900000. */
	uint32_t disconnected_sleep_ms; /**< 0 = never. Default 0. */
	uint32_t calib_timeout_ms;      /**< Default 5000. */
	uint32_t error_retry_ms;        /**< Default 500. */
	uint32_t error_retry_max_ms;    /**< Default 8000. */
	bool gyro_always_on;            /**< Keep gyro powered in TRACKPAD/IDLE. Default false. */
};

void wp_sm_config_default(struct wp_sm_config *cfg);

struct wp_sm_inputs {
	int64_t t_us;
	uint8_t contacts;               /**< Fingers currently down. */
	bool single_still;              /**< From wp_gesture_output. */
	uint32_t single_still_ms;
	bool motion_valid;              /**< false if no IMU data (IMU absent/failed). */
	struct wp_motion_features motion;
};

struct wp_sm_outputs {
	enum wp_state state;
	enum wp_state prev_state;
	bool state_changed;             /**< True on the update that changed state. */
	enum wp_mode_override mode;
	/* What the platform should do in this state: */
	bool gyro_needed;               /**< Power the gyroscope: PICKUP_CANDIDATE/HANDHELD/
					 *   AIR_MOUSE, CALIBRATING entered from HANDHELD or
					 *   AIR_MOUSE, any state but SLEEP if gyro_always_on. */
	bool imu_low_rate;              /**< Low IMU data rate (IDLE). */
	bool touch_low_power;           /**< Touch controller low-power (IDLE). */
	bool pointer_from_touch;        /**< false in AIR_MOUSE (finger is a clutch). */
	bool air_mouse_active;
	bool long_press_allowed;        /**< false in PICKUP_CANDIDATE/HANDHELD/AIR_MOUSE,
					 *   CALIBRATING entered from HANDHELD/AIR_MOUSE,
					 *   and FORCE_HANDHELD. */
	bool orientation_auto_allowed;  /**< false in AIR_MOUSE, CALIBRATING, ERROR_RECOVERY. */
	bool request_reinit;            /**< One-shot: re-run sensor initialization. */
	bool request_sleep;             /**< Enter System OFF. */
	bool ble_low_latency;           /**< Request short connection interval. */
};

struct wp_sm {
	struct wp_sm_config cfg;
	enum wp_state state;
	enum wp_state prev_state;
	enum wp_state calib_return;
	enum wp_mode_override mode;
	int64_t now_us;
	int64_t state_since_us;
	int64_t last_activity_us;
	int64_t last_moving_us;
	int64_t still_since_us;
	bool still_tracking;
	int64_t calm_since_us;          /**< still && !hand_tremor since (put-down). */
	bool calm_tracking;
	bool ble_connected;
	int64_t ble_disconnected_since_us;
	int64_t next_retry_us;
	uint32_t retry_delay_ms;
	bool reinit_pending;
	bool changed_pending;
	enum wp_state changed_from;
};

void wp_sm_init(struct wp_sm *sm, const struct wp_sm_config *cfg, int64_t t_us);

/** Deliver a discrete event. State change (if any) is reported by the next
 *  wp_sm_update(). */
void wp_sm_event(struct wp_sm *sm, enum wp_sm_event ev, int64_t t_us);

/** Evaluate inputs and time; fills @p out. Call at >= 50 Hz. */
void wp_sm_update(struct wp_sm *sm, const struct wp_sm_inputs *in, struct wp_sm_outputs *out);

enum wp_state wp_sm_state(const struct wp_sm *sm);

const char *wp_state_name(enum wp_state st);

const char *wp_mode_name(enum wp_mode_override mode);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_STATE_MACHINE_H_ */
