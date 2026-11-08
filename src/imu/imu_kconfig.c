/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> IMU algorithm configuration bridge (see <wafer_pad/config.h>).
 * Starts from the documented C defaults and applies CONFIG_WAFER_PAD_IMU_*.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/config.h>
#include <wafer_pad/imu_algo.h>
#include <wafer_pad/orientation_transform.h>

#define WP_ABS(x) ((x) < 0 ? -(x) : (x))

/* The mount map must be a signed permutation of {1, 2, 3}. */
BUILD_ASSERT(CONFIG_WAFER_PAD_IMU_MOUNT_X != 0 && CONFIG_WAFER_PAD_IMU_MOUNT_Y != 0 &&
		     CONFIG_WAFER_PAD_IMU_MOUNT_Z != 0,
	     "CONFIG_WAFER_PAD_IMU_MOUNT_X/Y/Z must be +-1, +-2 or +-3");
BUILD_ASSERT(WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_X) != WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_Y) &&
		     WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_X) != WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_Z) &&
		     WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_Y) != WP_ABS(CONFIG_WAFER_PAD_IMU_MOUNT_Z),
	     "CONFIG_WAFER_PAD_IMU_MOUNT_X/Y/Z must use each IMU axis exactly once");

void wp_config_imu_mount(struct wp_imu_mount *mount)
{
	if (mount == NULL) {
		return;
	}
	mount->map[0] = (int8_t)CONFIG_WAFER_PAD_IMU_MOUNT_X;
	mount->map[1] = (int8_t)CONFIG_WAFER_PAD_IMU_MOUNT_Y;
	mount->map[2] = (int8_t)CONFIG_WAFER_PAD_IMU_MOUNT_Z;

	if (!wp_imu_mount_valid(mount)) {
		/* Unreachable thanks to the BUILD_ASSERTs; keep identity as a fallback. */
		mount->map[0] = 1;
		mount->map[1] = 2;
		mount->map[2] = 3;
	}
}

void wp_config_gyro_bias(struct wp_gyro_bias_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	wp_gyro_bias_config_default(cfg);
	cfg->still_gyro_dps = CONFIG_WAFER_PAD_IMU_BIAS_STILL_GYRO_CENTI_DPS / 100.0f;
	cfg->still_accel_dev_g = CONFIG_WAFER_PAD_IMU_BIAS_STILL_ACCEL_DEV_MILLI_G / 1000.0f;
	cfg->still_min_ms = (uint32_t)CONFIG_WAFER_PAD_IMU_BIAS_STILL_MIN_MS;
	cfg->alpha = CONFIG_WAFER_PAD_IMU_BIAS_ALPHA_MILLI / 1000.0f;
	cfg->max_bias_dps = CONFIG_WAFER_PAD_IMU_BIAS_MAX_DECI_DPS / 10.0f;
}

void wp_config_motion(struct wp_motion_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	wp_motion_config_default(cfg);
	cfg->still_gyro_dps = CONFIG_WAFER_PAD_IMU_MOTION_STILL_GYRO_CENTI_DPS / 100.0f;
	cfg->still_accel_dev_g = CONFIG_WAFER_PAD_IMU_MOTION_STILL_ACCEL_DEV_MILLI_G / 1000.0f;
	cfg->still_window_ms = (uint32_t)CONFIG_WAFER_PAD_IMU_MOTION_STILL_WINDOW_MS;
	cfg->move_gyro_dps = CONFIG_WAFER_PAD_IMU_MOTION_MOVE_GYRO_DECI_DPS / 10.0f;
	cfg->move_accel_dev_g = CONFIG_WAFER_PAD_IMU_MOTION_MOVE_ACCEL_DEV_MILLI_G / 1000.0f;
	cfg->tremor_tau_ms = (float)CONFIG_WAFER_PAD_IMU_MOTION_TREMOR_TAU_MS;
	cfg->accel_lp_tau_ms = (float)CONFIG_WAFER_PAD_IMU_MOTION_ACCEL_LP_TAU_MS;
	cfg->handheld_tremor_dps = CONFIG_WAFER_PAD_IMU_MOTION_HANDHELD_TREMOR_CENTI_DPS / 100.0f;
	cfg->handheld_accel_jitter_g =
		CONFIG_WAFER_PAD_IMU_MOTION_HANDHELD_JITTER_MILLI_G / 1000.0f;
}

void wp_config_imu_plaus(struct wp_imu_plaus_config *cfg)
{
	if (cfg == NULL) {
		return;
	}
	wp_imu_plaus_config_default(cfg);
	cfg->min_accel_g = CONFIG_WAFER_PAD_IMU_DEAD_ACCEL_MILLI_G / 1000.0f;
	cfg->dead_ms = (uint32_t)CONFIG_WAFER_PAD_IMU_DEAD_MS;
}
