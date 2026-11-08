/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(wp_main, CONFIG_WAFER_PAD_LOG_LEVEL);

int main(void)
{
	LOG_INF("Wafer Pad booting");
	return 0;
}
