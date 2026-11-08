/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frame -> per-finger event conversion and the fixed mounting correction.
 * Pure C (no Zephyr includes), host-testable; see <wafer_pad/touch_tracker.h>.
 */

#include <string.h>

#include <wafer_pad/touch_tracker.h>

void wp_touch_tracker_init(struct wp_touch_tracker *t)
{
	if (t == NULL) {
		return;
	}
	memset(t, 0, sizeof(*t));
}

/* Append one event if there is room; surplus events are dropped. */
static void emit(struct wp_touch_event *events, size_t max_events, size_t *n,
		 enum wp_touch_event_type type, uint8_t id, uint16_t x, uint16_t y, int32_t dx,
		 int32_t dy, int64_t t_us)
{
	if ((events == NULL) || (*n >= max_events)) {
		return;
	}

	struct wp_touch_event *ev = &events[*n];

	ev->type = type;
	ev->id = id;
	ev->x = x;
	ev->y = y;
	ev->dx = dx;
	ev->dy = dy;
	ev->t_us = t_us;
	(*n)++;
}

size_t wp_touch_tracker_update(struct wp_touch_tracker *t, const struct wp_touch_frame *f,
			       struct wp_touch_event *events, size_t max_events)
{
	bool now[WP_TOUCH_TRACKER_MAX_ID] = {false};
	uint16_t nx[WP_TOUCH_TRACKER_MAX_ID] = {0};
	uint16_t ny[WP_TOUCH_TRACKER_MAX_ID] = {0};
	size_t count;
	size_t n = 0;

	if ((t == NULL) || (f == NULL)) {
		return 0;
	}

	count = f->count;
	if (count > WP_TOUCH_MAX_CONTACTS) {
		count = WP_TOUCH_MAX_CONTACTS;
	}

	for (size_t i = 0; i < count; i++) {
		const struct wp_touch_contact *c = &f->contacts[i];

		/* Out-of-range ids are ignored; a duplicated id keeps its first
		 * contact so that one id never maps to two positions.
		 */
		if ((c->id >= WP_TOUCH_TRACKER_MAX_ID) || now[c->id]) {
			continue;
		}
		now[c->id] = true;
		nx[c->id] = c->x;
		ny[c->id] = c->y;
	}

	/* 1. UP, ascending id, at the last known position. */
	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		if (t->active[id] && !now[id]) {
			emit(events, max_events, &n, WP_TOUCH_UP, id, t->x[id], t->y[id], 0, 0,
			     f->t_us);
		}
	}

	/* 2. DOWN, ascending id. */
	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		if (!t->active[id] && now[id]) {
			emit(events, max_events, &n, WP_TOUCH_DOWN, id, nx[id], ny[id], 0, 0,
			     f->t_us);
		}
	}

	/* 3. MOVE, ascending id, only if the position changed. */
	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		if (t->active[id] && now[id] && ((nx[id] != t->x[id]) || (ny[id] != t->y[id]))) {
			emit(events, max_events, &n, WP_TOUCH_MOVE, id, nx[id], ny[id],
			     (int32_t)nx[id] - (int32_t)t->x[id],
			     (int32_t)ny[id] - (int32_t)t->y[id], f->t_us);
		}
	}

	/* State follows the frame even if events were dropped. */
	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		t->active[id] = now[id];
		if (now[id]) {
			t->x[id] = nx[id];
			t->y[id] = ny[id];
		}
	}
	t->last_t_us = f->t_us;

	return n;
}

size_t wp_touch_tracker_release_all(struct wp_touch_tracker *t, int64_t t_us,
				    struct wp_touch_event *events, size_t max_events)
{
	size_t n = 0;

	if (t == NULL) {
		return 0;
	}

	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		if (t->active[id]) {
			emit(events, max_events, &n, WP_TOUCH_UP, id, t->x[id], t->y[id], 0, 0,
			     t_us);
			t->active[id] = false;
		}
	}
	t->last_t_us = t_us;

	return n;
}

uint8_t wp_touch_tracker_count(const struct wp_touch_tracker *t)
{
	uint8_t n = 0;

	if (t == NULL) {
		return 0;
	}

	for (uint8_t id = 0; id < WP_TOUCH_TRACKER_MAX_ID; id++) {
		if (t->active[id]) {
			n++;
		}
	}
	return n;
}

/* x' = max - x, with x clamped to max first so the result never wraps. */
static uint16_t mirror(uint16_t v, uint16_t max)
{
	return (v >= max) ? 0U : (uint16_t)(max - v);
}

void wp_touch_mount_apply(const struct wp_touch_mount *m, const struct wp_touch_frame *in,
			  struct wp_touch_frame *out)
{
	struct wp_touch_frame f;

	if ((in == NULL) || (out == NULL)) {
		return;
	}

	f = *in; /* copy first: in == out is allowed */
	if (f.count > WP_TOUCH_MAX_CONTACTS) {
		f.count = WP_TOUCH_MAX_CONTACTS;
	}

	if (m != NULL) {
		if (m->swap_xy) {
			uint16_t tmp = f.max_x;

			f.max_x = f.max_y;
			f.max_y = tmp;
		}

		for (uint8_t i = 0; i < f.count; i++) {
			struct wp_touch_contact *c = &f.contacts[i];

			if (m->swap_xy) {
				uint16_t tmp = c->x;

				c->x = c->y;
				c->y = tmp;
			}
			if (m->invert_x) {
				c->x = mirror(c->x, f.max_x);
			}
			if (m->invert_y) {
				c->y = mirror(c->y, f.max_y);
			}
		}
	}

	*out = f;
}
