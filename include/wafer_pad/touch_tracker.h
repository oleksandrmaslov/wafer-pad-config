/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frame -> per-finger event conversion (pure C, host-testable).
 *
 * The touch source delivers complete frames (struct wp_touch_frame). Consumers
 * that prefer an event stream (finger down / move / up) run frames through a
 * tracker. The tracker is controller-independent: it only relies on
 * wp_touch_contact.id being stable while a finger stays down.
 */

#ifndef WAFER_PAD_TOUCH_TRACKER_H_
#define WAFER_PAD_TOUCH_TRACKER_H_

#include <wafer_pad/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Upper bound on events produced by one wp_touch_tracker_update() call. */
#define WP_TOUCH_TRACKER_MAX_EVENTS (2 * WP_TOUCH_MAX_CONTACTS)

/** Highest tracking id accepted (ids >= this are ignored). */
#define WP_TOUCH_TRACKER_MAX_ID 16

struct wp_touch_tracker {
	bool active[WP_TOUCH_TRACKER_MAX_ID];
	uint16_t x[WP_TOUCH_TRACKER_MAX_ID];
	uint16_t y[WP_TOUCH_TRACKER_MAX_ID];
	int64_t last_t_us;
};

void wp_touch_tracker_init(struct wp_touch_tracker *t);

/**
 * Diff @p f against the previous frame and emit events.
 *
 * Order of emitted events: all UP events first (ascending id), then DOWN
 * events (ascending id), then MOVE events (ascending id). MOVE is emitted only
 * when x or y changed.  A contact whose id changes position by any amount is a
 * MOVE, never UP+DOWN.
 *
 * @return number of events written to @p events (<= max_events). If
 *         max_events is too small the surplus events are dropped (state is
 *         still updated).
 */
size_t wp_touch_tracker_update(struct wp_touch_tracker *t, const struct wp_touch_frame *f,
			       struct wp_touch_event *events, size_t max_events);

/**
 * Emit UP events for every active finger and clear the state. Used when the
 * controller resets or communication is lost, so that consumers never keep a
 * phantom finger (and therefore a stuck mouse button).
 */
size_t wp_touch_tracker_release_all(struct wp_touch_tracker *t, int64_t t_us,
				    struct wp_touch_event *events, size_t max_events);

/** Number of fingers currently tracked as down. */
uint8_t wp_touch_tracker_count(const struct wp_touch_tracker *t);

/** Apply the fixed mounting correction S -> P to a whole frame (in place
 *  is allowed: @p in == @p out). */
void wp_touch_mount_apply(const struct wp_touch_mount *m, const struct wp_touch_frame *in,
			  struct wp_touch_frame *out);

#ifdef __cplusplus
}
#endif

#endif /* WAFER_PAD_TOUCH_TRACKER_H_ */
