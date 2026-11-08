/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> wp_gesture_config bridge (see <wafer_pad/config.h>).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <wafer_pad/config.h>
#include <wafer_pad/gestures.h>

LOG_MODULE_REGISTER(wp_gestures, CONFIG_WAFER_PAD_LOG_LEVEL);

#define CENTI(v) ((float)(v) / 100.0f)
#define MILLI(v) ((float)(v) / 1000.0f)

void wp_config_gestures(struct wp_gesture_config *cfg)
{
	wp_gesture_config_default(cfg);

	cfg->tap_enabled = IS_ENABLED(CONFIG_WAFER_PAD_GEST_TAP);
	cfg->tap_max_ms = CONFIG_WAFER_PAD_GEST_TAP_MAX_MS;
	cfg->tap_max_move_mm = CENTI(CONFIG_WAFER_PAD_GEST_TAP_MAX_MOVE_CENTI_MM);
	cfg->tap_drag_enabled = IS_ENABLED(CONFIG_WAFER_PAD_GEST_TAP_DRAG);
	cfg->double_tap_window_ms = CONFIG_WAFER_PAD_GEST_DOUBLE_TAP_WINDOW_MS;

	cfg->long_press_drag_enabled = IS_ENABLED(CONFIG_WAFER_PAD_GEST_LONG_PRESS_DRAG);
	cfg->long_press_ms = CONFIG_WAFER_PAD_GEST_LONG_PRESS_MS;
	cfg->long_press_max_move_mm = CENTI(CONFIG_WAFER_PAD_GEST_LONG_PRESS_MAX_MOVE_CENTI_MM);

	cfg->multi_down_skew_ms = CONFIG_WAFER_PAD_GEST_MULTI_DOWN_SKEW_MS;
	cfg->two_finger_tap_max_ms = CONFIG_WAFER_PAD_GEST_TWO_FINGER_TAP_MAX_MS;
	cfg->two_finger_tap_max_move_mm =
		CENTI(CONFIG_WAFER_PAD_GEST_TWO_FINGER_TAP_MAX_MOVE_CENTI_MM);
	cfg->three_finger_tap_enabled = IS_ENABLED(CONFIG_WAFER_PAD_GEST_THREE_FINGER_TAP);

	cfg->scroll_detents_per_mm = MILLI(CONFIG_WAFER_PAD_GEST_SCROLL_DETENTS_PER_MM_MILLI);
	cfg->scroll_start_mm = CENTI(CONFIG_WAFER_PAD_GEST_SCROLL_START_CENTI_MM);
	cfg->scroll_axis_lock = IS_ENABLED(CONFIG_WAFER_PAD_GEST_SCROLL_AXIS_LOCK);
	cfg->scroll_axis_lock_ratio = CENTI(CONFIG_WAFER_PAD_GEST_SCROLL_AXIS_LOCK_RATIO_CENTI);
	cfg->scroll.natural = IS_ENABLED(CONFIG_WAFER_PAD_GEST_SCROLL_NATURAL);
	cfg->scroll.invert_pan = IS_ENABLED(CONFIG_WAFER_PAD_GEST_SCROLL_INVERT_PAN);

	cfg->jitter_deadband_mm = CENTI(CONFIG_WAFER_PAD_GEST_JITTER_DEADBAND_CENTI_MM);
	cfg->smoothing_alpha = CENTI(CONFIG_WAFER_PAD_GEST_SMOOTHING_ALPHA_CENTI);
	cfg->smoothing_full_speed_mm_s = (float)CONFIG_WAFER_PAD_GEST_SMOOTHING_FULL_SPEED_MM_S;
	cfg->jump_max_step_mm = (float)CONFIG_WAFER_PAD_GEST_JUMP_MAX_STEP_MM;
	cfg->jump_max_speed_mm_s = (float)CONFIG_WAFER_PAD_GEST_JUMP_MAX_SPEED_MM_S;

	cfg->accel.counts_per_mm = CENTI(CONFIG_WAFER_PAD_POINTER_COUNTS_PER_MM_CENTI);
	cfg->accel.sensitivity = CENTI(CONFIG_WAFER_PAD_POINTER_SENSITIVITY_CENTI);
	cfg->accel.low_speed_mm_s = (float)CONFIG_WAFER_PAD_POINTER_ACCEL_LOW_SPEED;
	cfg->accel.high_speed_mm_s = (float)CONFIG_WAFER_PAD_POINTER_ACCEL_HIGH_SPEED;
	cfg->accel.max_multiplier = CENTI(CONFIG_WAFER_PAD_POINTER_ACCEL_MAX_MULT_CENTI);

	if (CONFIG_WAFER_PAD_POINTER_ACCEL_HIGH_SPEED <= CONFIG_WAFER_PAD_POINTER_ACCEL_LOW_SPEED) {
		LOG_WRN("accel high speed <= low speed: acceleration becomes a step");
	}
	LOG_DBG("tap %d/%dms, drag tap %d lp %d/%dms, scroll %d milli-detent/mm natural %d",
		(int)cfg->tap_enabled, CONFIG_WAFER_PAD_GEST_TAP_MAX_MS, (int)cfg->tap_drag_enabled,
		(int)cfg->long_press_drag_enabled, CONFIG_WAFER_PAD_GEST_LONG_PRESS_MS,
		CONFIG_WAFER_PAD_GEST_SCROLL_DETENTS_PER_MM_MILLI, (int)cfg->scroll.natural);
	LOG_DBG("pointer %d centi-counts/mm sens %d accel %d..%d mm/s x%d centi",
		CONFIG_WAFER_PAD_POINTER_COUNTS_PER_MM_CENTI,
		CONFIG_WAFER_PAD_POINTER_SENSITIVITY_CENTI,
		CONFIG_WAFER_PAD_POINTER_ACCEL_LOW_SPEED,
		CONFIG_WAFER_PAD_POINTER_ACCEL_HIGH_SPEED,
		CONFIG_WAFER_PAD_POINTER_ACCEL_MAX_MULT_CENTI);
}
