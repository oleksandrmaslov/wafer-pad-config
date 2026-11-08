/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> struct wp_air_mouse_config bridge (see <wafer_pad/config.h>).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <wafer_pad/air_mouse.h>
#include <wafer_pad/config.h>

LOG_MODULE_REGISTER(wp_air_mouse, CONFIG_WAFER_PAD_LOG_LEVEL);

void wp_config_air_mouse(struct wp_air_mouse_config *cfg)
{
	if (cfg == NULL) {
		return;
	}

	wp_air_mouse_config_default(cfg);

	cfg->counts_per_deg = (float)CONFIG_WAFER_PAD_AIR_COUNTS_PER_DEG_CENTI / 100.0f;
	cfg->deadzone_dps = (float)CONFIG_WAFER_PAD_AIR_DEADZONE_DECI_DPS / 10.0f;
	cfg->lpf_cutoff_hz = (float)CONFIG_WAFER_PAD_AIR_LPF_CUTOFF_HZ;
	cfg->max_rate_dps = (float)CONFIG_WAFER_PAD_AIR_MAX_RATE_DPS;
	cfg->ramp_in_ms = (uint32_t)CONFIG_WAFER_PAD_AIR_RAMP_IN_MS;
	cfg->gravity_compensated = IS_ENABLED(CONFIG_WAFER_PAD_AIR_GRAVITY_COMP);
	cfg->accel_lp_tau_ms = (float)CONFIG_WAFER_PAD_AIR_ACCEL_LP_TAU_MS;
	cfg->invert_x = IS_ENABLED(CONFIG_WAFER_PAD_AIR_INVERT_X);
	cfg->invert_y = IS_ENABLED(CONFIG_WAFER_PAD_AIR_INVERT_Y);

	LOG_DBG("air mouse: %d centi-counts/deg, deadzone %d deci-dps, lpf %d Hz, ramp %d ms, "
		"gravity comp %d",
		CONFIG_WAFER_PAD_AIR_COUNTS_PER_DEG_CENTI, CONFIG_WAFER_PAD_AIR_DEADZONE_DECI_DPS,
		CONFIG_WAFER_PAD_AIR_LPF_CUTOFF_HZ, CONFIG_WAFER_PAD_AIR_RAMP_IN_MS,
		(int)cfg->gravity_compensated);
}
