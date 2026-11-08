/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> struct wp_sm_config bridge (see <wafer_pad/config.h>).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <wafer_pad/config.h>
#include <wafer_pad/state_machine.h>

LOG_MODULE_REGISTER(wp_state, CONFIG_WAFER_PAD_LOG_LEVEL);

BUILD_ASSERT(CONFIG_WAFER_PAD_SM_ERROR_RETRY_MAX_MS >= CONFIG_WAFER_PAD_SM_ERROR_RETRY_MS,
	     "WAFER_PAD_SM_ERROR_RETRY_MAX_MS must be >= WAFER_PAD_SM_ERROR_RETRY_MS");

void wp_config_sm(struct wp_sm_config *cfg)
{
	if (cfg == NULL) {
		return;
	}

	wp_sm_config_default(cfg);

	cfg->pickup_confirm_ms = (uint32_t)CONFIG_WAFER_PAD_SM_PICKUP_CONFIRM_MS;
	cfg->pickup_settle_ms = (uint32_t)CONFIG_WAFER_PAD_SM_PICKUP_SETTLE_MS;
	cfg->pickup_timeout_ms = (uint32_t)CONFIG_WAFER_PAD_SM_PICKUP_TIMEOUT_MS;
	cfg->putdown_still_ms = (uint32_t)CONFIG_WAFER_PAD_SM_PUTDOWN_STILL_MS;
	cfg->handheld_putdown_ms = (uint32_t)CONFIG_WAFER_PAD_SM_HANDHELD_PUTDOWN_MS;
	cfg->air_mouse_enabled = IS_ENABLED(CONFIG_WAFER_PAD_SM_AIR_MOUSE_ENABLED);
	cfg->air_hold_ms = (uint32_t)CONFIG_WAFER_PAD_SM_AIR_HOLD_MS;
	cfg->idle_timeout_ms = (uint32_t)CONFIG_WAFER_PAD_SM_IDLE_TIMEOUT_MS;
	cfg->sleep_enabled = IS_ENABLED(CONFIG_WAFER_PAD_SM_SLEEP_ENABLED);
	cfg->sleep_timeout_ms = (uint32_t)CONFIG_WAFER_PAD_SM_SLEEP_TIMEOUT_MS;
	cfg->disconnected_sleep_ms = (uint32_t)CONFIG_WAFER_PAD_SM_DISCONNECTED_SLEEP_MS;
	cfg->calib_timeout_ms = (uint32_t)CONFIG_WAFER_PAD_SM_CALIB_TIMEOUT_MS;
	cfg->error_retry_ms = (uint32_t)CONFIG_WAFER_PAD_SM_ERROR_RETRY_MS;
	cfg->error_retry_max_ms = (uint32_t)CONFIG_WAFER_PAD_SM_ERROR_RETRY_MAX_MS;
	cfg->gyro_always_on = IS_ENABLED(CONFIG_WAFER_PAD_SM_GYRO_ALWAYS_ON);

	LOG_DBG("sm: idle %u ms, sleep %d (%u ms), air mouse %d, gyro always on %d",
		cfg->idle_timeout_ms, (int)cfg->sleep_enabled, cfg->sleep_timeout_ms,
		(int)cfg->air_mouse_enabled, (int)cfg->gyro_always_on);
}
