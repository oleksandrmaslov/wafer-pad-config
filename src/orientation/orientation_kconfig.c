/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> orientation / calibration configuration bridge
 * (see include/wafer_pad/config.h). Starts from the pure-C defaults and
 * applies the CONFIG_WAFER_PAD_ORIENT_* / CONFIG_WAFER_PAD_CALIB_* values.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/config.h>
#include <wafer_pad/orientation_transform.h>

LOG_MODULE_REGISTER(wp_orient_cfg, CONFIG_WAFER_PAD_LOG_LEVEL);

BUILD_ASSERT((CONFIG_WAFER_PAD_ORIENT_DEFAULT_DEG % 90) == 0,
	     "CONFIG_WAFER_PAD_ORIENT_DEFAULT_DEG must be 0, 90, 180 or 270");
BUILD_ASSERT(CONFIG_WAFER_PAD_ORIENT_MIN_TILT_DEG < CONFIG_WAFER_PAD_ORIENT_MAX_TILT_DEG,
	     "CONFIG_WAFER_PAD_ORIENT_MIN_TILT_DEG must be below ..._MAX_TILT_DEG");
BUILD_ASSERT(CONFIG_WAFER_PAD_CALIB_SWIPE_START_CENTI_MM < CONFIG_WAFER_PAD_CALIB_SWIPE_MIN_MM * 100,
	     "CONFIG_WAFER_PAD_CALIB_SWIPE_START_CENTI_MM must be below the swipe distance");

void wp_config_orient_mgr(struct wp_orient_mgr_config *cfg)
{
	wp_orient_mgr_config_default(cfg);

	cfg->auto_enabled = IS_ENABLED(CONFIG_WAFER_PAD_ORIENT_AUTO);

	cfg->gravity.min_tilt_deg = (float)CONFIG_WAFER_PAD_ORIENT_MIN_TILT_DEG;
	cfg->gravity.max_tilt_deg = (float)CONFIG_WAFER_PAD_ORIENT_MAX_TILT_DEG;
	cfg->gravity.max_mag_dev_g = (float)CONFIG_WAFER_PAD_ORIENT_MAX_MAG_DEV_MILLI_G / 1000.0f;
	cfg->gravity.hysteresis_deg = (float)CONFIG_WAFER_PAD_ORIENT_HYSTERESIS_DEG;
	cfg->gravity.max_gyro_dps = (float)CONFIG_WAFER_PAD_ORIENT_MAX_GYRO_DPS;
	cfg->gravity.confirm_ms = (uint32_t)CONFIG_WAFER_PAD_ORIENT_CONFIRM_MS;
	cfg->gravity.confirm_ms_no_gyro = (uint32_t)CONFIG_WAFER_PAD_ORIENT_CONFIRM_NO_GYRO_MS;
	cfg->gravity.max_tilt_change_deg = (float)CONFIG_WAFER_PAD_ORIENT_MAX_TILT_CHANGE_DEG;
	cfg->gravity.min_confidence = (float)CONFIG_WAFER_PAD_ORIENT_MIN_CONFIDENCE_CENTI / 100.0f;
	cfg->gravity.lpf_tau_ms = (float)CONFIG_WAFER_PAD_ORIENT_LPF_TAU_MS;

	cfg->manual_lock_mode = (enum wp_manual_lock_mode)CONFIG_WAFER_PAD_ORIENT_MANUAL_LOCK_MODE;
	cfg->manual_lock_ms = (uint32_t)CONFIG_WAFER_PAD_ORIENT_MANUAL_LOCK_MS;
	cfg->touch_release_settle_ms = (uint32_t)CONFIG_WAFER_PAD_ORIENT_TOUCH_SETTLE_MS;

	cfg->gyro.enabled = IS_ENABLED(CONFIG_WAFER_PAD_ORIENT_GYRO_ASSIST);
#ifdef CONFIG_WAFER_PAD_ORIENT_GYRO_ASSIST
	cfg->gyro.step_deg = (float)CONFIG_WAFER_PAD_ORIENT_GYRO_STEP_DEG;
	cfg->gyro.min_rate_dps = (float)CONFIG_WAFER_PAD_ORIENT_GYRO_MIN_RATE_DPS;
	cfg->gyro.max_window_ms = (uint32_t)CONFIG_WAFER_PAD_ORIENT_GYRO_MAX_WINDOW_MS;
	cfg->gyro.max_tilt_deg = (float)CONFIG_WAFER_PAD_ORIENT_GYRO_MAX_TILT_DEG;
#endif

	LOG_DBG("auto=%d tilt=%d..%d deg confirm=%d ms lock mode=%d (%d ms) gyro=%d",
		(int)cfg->auto_enabled, CONFIG_WAFER_PAD_ORIENT_MIN_TILT_DEG,
		CONFIG_WAFER_PAD_ORIENT_MAX_TILT_DEG, CONFIG_WAFER_PAD_ORIENT_CONFIRM_MS,
		CONFIG_WAFER_PAD_ORIENT_MANUAL_LOCK_MODE, CONFIG_WAFER_PAD_ORIENT_MANUAL_LOCK_MS,
		(int)cfg->gyro.enabled);
}

void wp_config_calib(struct wp_calib_config *cfg)
{
	wp_calib_config_default(cfg);

	cfg->fingers = (uint8_t)CONFIG_WAFER_PAD_CALIB_FINGERS;
	cfg->hold_ms = (uint32_t)CONFIG_WAFER_PAD_CALIB_HOLD_MS;
	cfg->hold_max_move_mm = (float)CONFIG_WAFER_PAD_CALIB_HOLD_MAX_MOVE_CENTI_MM / 100.0f;
	cfg->arm_timeout_ms = (uint32_t)CONFIG_WAFER_PAD_CALIB_ARM_TIMEOUT_MS;
	cfg->swipe_start_mm = (float)CONFIG_WAFER_PAD_CALIB_SWIPE_START_CENTI_MM / 100.0f;
	cfg->swipe_min_mm = (float)CONFIG_WAFER_PAD_CALIB_SWIPE_MIN_MM;
	cfg->swipe_max_ms = (uint32_t)CONFIG_WAFER_PAD_CALIB_SWIPE_MAX_MS;
	cfg->swipe_max_angle_dev_deg = (float)CONFIG_WAFER_PAD_CALIB_SWIPE_MAX_ANGLE_DEV_DEG;
	cfg->lift_max_ms = (uint32_t)CONFIG_WAFER_PAD_CALIB_LIFT_MAX_MS;
}

enum wp_orientation wp_config_orient_default(void)
{
	return wp_orientation_norm(CONFIG_WAFER_PAD_ORIENT_DEFAULT_DEG / 90);
}
