/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * IQS5xx driver frame -> wp_touch_frame. Spec: <wafer_pad/touch_iqs5xx.h>.
 * Controller facts: research notes iqs5xx.md §4.3 (System Info 1 bits) and
 * §6.2 (slot activity, 0xFFFF for slots not in touch, TOO_MANY_FINGERS /
 * PALM_DETECT invalidate the XY data).
 */

#include <stddef.h>
#include <string.h>

#include <wafer_pad/touch_iqs5xx.h>

#define XY_INVALID_FLAGS (IQS5XX_SYS_INFO1_TOO_MANY_FINGERS | IQS5XX_SYS_INFO1_PALM_DETECT)

uint8_t wp_touch_from_iqs5xx(const struct iqs5xx_frame *in, uint16_t max_x, uint16_t max_y,
			     struct wp_touch_frame *out)
{
	if (out == NULL) {
		return 0U;
	}
	memset(out, 0, sizeof(*out));
	out->max_x = max_x;
	out->max_y = max_y;
	if (in == NULL) {
		return 0U;
	}
	out->t_us = in->timestamp_us;

	/* XY cleared (too many fingers) or suppressed (palm): no contacts. */
	if ((in->sys_info1 & XY_INVALID_FLAGS) != 0U) {
		return 0U;
	}

	for (uint8_t i = 0; (i < IQS5XX_MAX_FINGERS) && (out->count < WP_TOUCH_MAX_CONTACTS);
	     i++) {
		const struct iqs5xx_finger *fg = &in->fingers[i];
		struct wp_touch_contact *c;

		if (fg->strength == 0U || fg->x == IQS5XX_COORD_NONE ||
		    fg->y == IQS5XX_COORD_NONE) {
			continue;
		}
		c = &out->contacts[out->count++];
		c->id = i;
		c->x = (fg->x < max_x) ? fg->x : max_x;
		c->y = (fg->y < max_y) ? fg->y : max_y;
		c->strength = fg->strength;
		c->area = fg->area;
	}
	return out->count;
}
