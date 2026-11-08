/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Kconfig -> touch configuration bridge (see <wafer_pad/config.h>).
 */

#include <zephyr/sys/util_macro.h>

#include <wafer_pad/config.h>

void wp_config_touch_mount(struct wp_touch_mount *mount)
{
	if (mount == NULL) {
		return;
	}

	mount->swap_xy = IS_ENABLED(CONFIG_WAFER_PAD_TOUCH_SWAP_XY);
	mount->invert_x = IS_ENABLED(CONFIG_WAFER_PAD_TOUCH_INVERT_X);
	mount->invert_y = IS_ENABLED(CONFIG_WAFER_PAD_TOUCH_INVERT_Y);
}

void wp_config_touch_size_mm(float *width_mm, float *height_mm)
{
	if (width_mm != NULL) {
		*width_mm = (float)CONFIG_WAFER_PAD_TOUCH_WIDTH_CENTI_MM / 100.0f;
	}
	if (height_mm != NULL) {
		*height_mm = (float)CONFIG_WAFER_PAD_TOUCH_HEIGHT_CENTI_MM / 100.0f;
	}
}
