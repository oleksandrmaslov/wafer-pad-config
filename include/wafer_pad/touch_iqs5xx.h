/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * IQS5xx driver frame -> normalized touch frame (pure logic, host-testable;
 * uses only the types and constants of <drivers/azoteq_iqs5xx.h>).
 *
 *   - Slot active <=> strength != 0. The slot index is the tracking id, so
 *     walking the slots in order gives contacts sorted by ascending id.
 *   - A frame flagged TOO_MANY_FINGERS or PALM_DETECT (System Info 1) has no
 *     valid XY data: it is reported with ZERO contacts (all fingers up).
 *   - A slot whose X or Y is 0xFFFF (IQS5XX_COORD_NONE, "not in touch") is
 *     dropped even if its strength is non-zero.
 *   - Coordinates are clamped to the inclusive extent max_x / max_y.
 *
 * The result is in the SENSOR frame S (no mounting correction applied).
 */

#ifndef WAFER_PAD_TOUCH_IQS5XX_H_
#define WAFER_PAD_TOUCH_IQS5XX_H_

#include <drivers/azoteq_iqs5xx.h>
#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Convert @p in to @p out (fully overwritten). Returns out->count. */
uint8_t wp_touch_from_iqs5xx(const struct iqs5xx_frame *in, uint16_t max_x, uint16_t max_y,
			     struct wp_touch_frame *out);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_TOUCH_IQS5XX_H_ */
