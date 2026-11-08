/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> algorithm configuration bridge.
 *
 * The algorithm modules are pure C and know nothing about Kconfig; each one
 * provides wp_xxx_config_default() with the documented defaults. Every module
 * ALSO provides one Zephyr-side "bridge" function below (implemented in
 * src/<module>/<module>_kconfig.c) that starts from the defaults and applies
 * the CONFIG_WAFER_PAD_* values. Only the router calls these.
 *
 * Kconfig has no float type: fractional parameters are expressed as integers
 * with the scale in the symbol name (_MILLI = x1000, _CENTI = x100, _DECI =
 * x10), e.g. CONFIG_WAFER_PAD_ORIENT_MAX_MAG_DEV_MILLI_G=120 -> 0.120 g.
 */

#ifndef WAFER_PAD_CONFIG_H_
#define WAFER_PAD_CONFIG_H_

#include <wafer_pad/types.h>
#include <wafer_pad/orientation.h>
#include <wafer_pad/orientation_calib.h>
#include <wafer_pad/gestures.h>
#include <wafer_pad/air_mouse.h>
#include <wafer_pad/imu_algo.h>
#include <wafer_pad/state_machine.h>

#ifdef __cplusplus
extern "C" {
#endif

/* src/touch/touch_kconfig.c */
void wp_config_touch_mount(struct wp_touch_mount *mount);
/** Physical size of the touch surface in the SENSOR frame (before mount). */
void wp_config_touch_size_mm(float *width_mm, float *height_mm);

/* src/imu/imu_kconfig.c */
void wp_config_imu_mount(struct wp_imu_mount *mount);
void wp_config_gyro_bias(struct wp_gyro_bias_config *cfg);
void wp_config_motion(struct wp_motion_config *cfg);
void wp_config_imu_plaus(struct wp_imu_plaus_config *cfg);

/* src/orientation/orientation_kconfig.c */
void wp_config_orient_mgr(struct wp_orient_mgr_config *cfg);
void wp_config_calib(struct wp_calib_config *cfg);
/** Boot orientation (CONFIG_WAFER_PAD_ORIENT_DEFAULT_DEG / 90). */
enum wp_orientation wp_config_orient_default(void);

/* src/gestures/gestures_kconfig.c */
void wp_config_gestures(struct wp_gesture_config *cfg);

/* src/air_mouse/air_mouse_kconfig.c */
void wp_config_air_mouse(struct wp_air_mouse_config *cfg);

/* src/state/state_kconfig.c */
void wp_config_sm(struct wp_sm_config *cfg);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_CONFIG_H_ */
