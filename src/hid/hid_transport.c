/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * HID transport selection. Transports are listed in order of preference
 * (a wired transport, when added, goes first). The active transport is the
 * first initialized one whose is_ready() is true. Only the router thread calls
 * these functions, so no locking is needed here.
 *
 * v0.1 has only the BLE HID-over-GATT transport: USB HID would need its own
 * USB device context (the board's CDC ACM console context is created and
 * enabled at boot with only CDC ACM registered).
 */

#include <errno.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/platform.h>

#include "hid_internal.h"

LOG_MODULE_REGISTER(wp_hid, CONFIG_WAFER_PAD_LOG_LEVEL);

static const struct wp_hid_transport_api *const transports[] = {
	&wp_hid_ble_transport,
};

static bool transport_up[ARRAY_SIZE(transports)];

static const struct wp_hid_transport_api *active_transport(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(transports); i++) {
		if (transport_up[i] && transports[i]->is_ready()) {
			return transports[i];
		}
	}
	return NULL;
}

int wp_hid_init(void)
{
	int first_err = 0;
	int up = 0;

	for (size_t i = 0; i < ARRAY_SIZE(transports); i++) {
		const struct wp_hid_transport_api *t = transports[i];
		int err;

		if (transport_up[i]) {
			up++;
			continue;
		}

		err = t->init();
		if (err != 0) {
			LOG_ERR("%s transport init failed: %d", t->name, err);
			if (first_err == 0) {
				first_err = err;
			}
			continue;
		}

		transport_up[i] = true;
		up++;
		LOG_INF("%s transport up", t->name);
	}

	if (up == 0) {
		return (first_err != 0) ? first_err : -ENODEV;
	}
	return 0;
}

bool wp_hid_is_ready(void)
{
	return active_transport() != NULL;
}

uint32_t wp_hid_link_gen(void)
{
	uint32_t gen = 0U;

	/* Sum over the transports: changes whenever any of them got ready. */
	for (size_t i = 0; i < ARRAY_SIZE(transports); i++) {
		if (transport_up[i] && transports[i]->link_gen != NULL) {
			gen += transports[i]->link_gen();
		}
	}
	return gen;
}

int wp_hid_send_mouse(const struct wp_mouse_report *r)
{
	const struct wp_hid_transport_api *t;

	if (r == NULL) {
		return -EINVAL;
	}

	t = active_transport();
	if (t == NULL) {
		return -EAGAIN;
	}

	return t->send_mouse(r);
}
