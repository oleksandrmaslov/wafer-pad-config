/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2026 Wafer Pad contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BLE peripheral of the Wafer Pad: stack bring-up, advertising, the single
 * connection, security/bonding (Just Works), connection parameters, input
 * report flow control and the Battery Service level.
 *
 * Advertising data and the connect -> security request flow are adapted from
 * the Zephyr samples/bluetooth/peripheral_hids sample (main.c).
 *
 * Link state ("HID ready") = connected AND security >= L2 AND the input report
 * CCC has notifications enabled. Every transition is posted to the router with
 * wp_router_post_hid_connected().
 *
 * Flow control: at most CONFIG_WAFER_PAD_BLE_MAX_INFLIGHT notifications are
 * queued in the host/controller. A completion callback frees a slot and posts
 * wp_router_post_hid_ready(). Completions are tagged with a CONNECTION
 * generation: queued PDUs keep their ATT buffers (and their completions keep
 * coming) across CCC / security changes within one connection, so the
 * in-flight count is only reset on connect / disconnect. If no completion
 * arrives for longer than the supervision timeout (+ margin) on a link that
 * stays up (peer stopped acknowledging), the link is dropped instead of
 * reclaiming slots: reclaiming would over-commit the ATT buffers and block
 * the router thread in bt_gatt_notify_cb() (K_FOREVER allocation).
 *
 * Robustness for an unattended demo:
 *   - a connection that does not become "HID ready" within
 *     WP_BLE_READY_TIMEOUT_S (scanner app, failed encryption, ...) is
 *     disconnected, so the single connection slot and advertising come back;
 *   - new pairings are accepted only while a pairing window is open
 *     (no bond yet, CONFIG_WAFER_PAD_BLE_PAIRING_WINDOW_S after boot, or
 *     wp_ble_open_pairing_window()), so visitors cannot evict the demo
 *     host's bond; pairing requires LE Secure Connections (Zephyr default
 *     BT_SMP_SC_PAIR_ONLY);
 *   - connection parameters the central did not apply (rejected, or later
 *     changed) are re-requested with back-off, a few times per profile.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#if defined(CONFIG_BT_BAS)
#include <zephyr/bluetooth/services/bas.h>
#endif

#include <wafer_pad/hid_report.h>
#include <wafer_pad/platform.h>

#include "hid_internal.h"

LOG_MODULE_REGISTER(wp_ble, CONFIG_WAFER_PAD_LOG_LEVEL);

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */

#define US_TO_ADV_UNITS(us)  ((us) / 625)    /* 0.625 ms */
#define US_TO_CONN_UNITS(us) ((us) / 1250)   /* 1.25 ms */
#define MS_TO_TIMEOUT_UNITS(ms) ((ms) / 10)  /* 10 ms */

#define ADV_INT_MIN  US_TO_ADV_UNITS(CONFIG_WAFER_PAD_BLE_ADV_INTERVAL_MIN_US)
#define ADV_INT_MAX  US_TO_ADV_UNITS(CONFIG_WAFER_PAD_BLE_ADV_INTERVAL_MAX_US)
#define LL_INT_MIN   US_TO_CONN_UNITS(CONFIG_WAFER_PAD_BLE_LL_INTERVAL_MIN_US)
#define LL_INT_MAX   US_TO_CONN_UNITS(CONFIG_WAFER_PAD_BLE_LL_INTERVAL_MAX_US)
#define PS_INT_MIN   US_TO_CONN_UNITS(CONFIG_WAFER_PAD_BLE_PS_INTERVAL_MIN_US)
#define PS_INT_MAX   US_TO_CONN_UNITS(CONFIG_WAFER_PAD_BLE_PS_INTERVAL_MAX_US)
#define SUP_TIMEOUT  MS_TO_TIMEOUT_UNITS(CONFIG_WAFER_PAD_BLE_SUPERVISION_TIMEOUT_MS)

BUILD_ASSERT(CONFIG_WAFER_PAD_BLE_ADV_INTERVAL_MIN_US % 625 == 0 &&
	     CONFIG_WAFER_PAD_BLE_ADV_INTERVAL_MAX_US % 625 == 0,
	     "advertising intervals must be multiples of 625 us");
BUILD_ASSERT(ADV_INT_MIN <= ADV_INT_MAX, "advertising interval min > max");

BUILD_ASSERT(CONFIG_WAFER_PAD_BLE_LL_INTERVAL_MIN_US % 1250 == 0 &&
	     CONFIG_WAFER_PAD_BLE_LL_INTERVAL_MAX_US % 1250 == 0 &&
	     CONFIG_WAFER_PAD_BLE_PS_INTERVAL_MIN_US % 1250 == 0 &&
	     CONFIG_WAFER_PAD_BLE_PS_INTERVAL_MAX_US % 1250 == 0,
	     "connection intervals must be multiples of 1250 us");
BUILD_ASSERT(CONFIG_WAFER_PAD_BLE_SUPERVISION_TIMEOUT_MS % 10 == 0,
	     "supervision timeout must be a multiple of 10 ms");
BUILD_ASSERT(LL_INT_MIN <= LL_INT_MAX && PS_INT_MIN <= PS_INT_MAX, "interval min > max");
/* Core spec (Vol 6, Part B, 4.5.2): timeout > (1 + latency) * interval_max * 2,
 * i.e. timeout_units * 4 > (1 + latency) * interval_units (bt_le_conn_params_valid).
 * Apple ADG 57.6 is stricter (x3), which is what we check.
 */
BUILD_ASSERT(SUP_TIMEOUT * 8 > 3 * (1 + CONFIG_WAFER_PAD_BLE_LL_LATENCY) * LL_INT_MAX,
	     "supervision timeout too short for the low-latency parameters");
BUILD_ASSERT(SUP_TIMEOUT * 8 > 3 * (1 + CONFIG_WAFER_PAD_BLE_PS_LATENCY) * PS_INT_MAX,
	     "supervision timeout too short for the power-saving parameters");

#if defined(CONFIG_BT_ATT_TX_COUNT)
/* att_pool is shared with ATT responses (at most one, ATT is sequential),
 * the Service Changed indication and the BAS notification: the router's
 * notify must never wait for a buffer (K_FOREVER outside the system work
 * queue). */
BUILD_ASSERT(CONFIG_WAFER_PAD_BLE_MAX_INFLIGHT + 3 <= CONFIG_BT_ATT_TX_COUNT,
	     "BT_ATT_TX_COUNT must cover the HID in-flight reports + response + "
	     "Service Changed + BAS, so notify never blocks the router thread");
#endif

/* Added to the supervision timeout to get the in-flight stall limit. */
#define WP_BLE_TX_STALL_MARGIN_MS 1000
/* Retry period when advertising cannot be started. */
#define WP_BLE_ADV_RETRY_MS 1000
/* A connection must become HID ready (encrypted + subscribed) within this
 * time (first HOGP discovery plus a pairing dialog fit easily). */
#define WP_BLE_READY_TIMEOUT_S 60
/* ... or within this time after an encryption / pairing failure. */
#define WP_BLE_READY_TIMEOUT_FAIL_S 10
/* Connection parameter re-requests per profile change, first delay. */
#define WP_BLE_PARAM_RETRY_MAX 3
#define WP_BLE_PARAM_RETRY_S   5

static const struct bt_le_conn_param conn_param_low_latency =
	BT_LE_CONN_PARAM_INIT(LL_INT_MIN, LL_INT_MAX, CONFIG_WAFER_PAD_BLE_LL_LATENCY, SUP_TIMEOUT);
static const struct bt_le_conn_param conn_param_power_saving =
	BT_LE_CONN_PARAM_INIT(PS_INT_MIN, PS_INT_MAX, CONFIG_WAFER_PAD_BLE_PS_LATENCY, SUP_TIMEOUT);

static const struct bt_le_adv_param adv_param =
	BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN, ADV_INT_MIN, ADV_INT_MAX, NULL);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, (CONFIG_BT_DEVICE_APPEARANCE & 0xFF),
		      ((CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xFF)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

enum conn_profile {
	PROFILE_NONE = -1,
	PROFILE_POWER_SAVING = 0,
	PROFILE_LOW_LATENCY = 1,
};

static struct k_spinlock lock;
/* All of the following are guarded by @lock. */
static struct bt_conn *cur_conn;   /* our reference to the single connection */
static bool link_ready;            /* connected + encrypted + subscribed */
static uint32_t conn_gen;          /* bumped on connect / disconnect (completion tag) */
static uint32_t ready_gen;         /* bumped on every transition to link_ready */
static int inflight;               /* notifications not yet completed */
static int64_t tx_progress_ms;     /* uptime of the last send/completion */
static bool want_low_latency = true;
static enum conn_profile requested_profile = PROFILE_NONE;
static uint8_t param_attempts;     /* re-requests for the current profile */
static int64_t pairing_until_ms;   /* pairing window end (uptime) */

static atomic_t ble_ready; /* bt_enable() + settings_load() done */

static void adv_work_handler(struct k_work *work);
static void stall_work_handler(struct k_work *work);
static void ready_work_handler(struct k_work *work);
static void param_retry_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_handler);
static K_WORK_DELAYABLE_DEFINE(stall_work, stall_work_handler);
static K_WORK_DELAYABLE_DEFINE(ready_work, ready_work_handler);
static K_WORK_DELAYABLE_DEFINE(param_retry_work, param_retry_handler);

static struct bt_conn *get_conn_ref(void)
{
	struct bt_conn *conn = NULL;
	k_spinlock_key_t key = k_spin_lock(&lock);

	if (cur_conn != NULL) {
		conn = bt_conn_ref(cur_conn);
	}
	k_spin_unlock(&lock, key);
	return conn;
}

/* ------------------------------------------------------------------------ */
/* Advertising                                                               */
/* ------------------------------------------------------------------------ */

static void adv_work_handler(struct k_work *work)
{
	bool connected;
	k_spinlock_key_t key;
	int err;

	ARG_UNUSED(work);

	if (!atomic_get(&ble_ready)) {
		return;
	}

	key = k_spin_lock(&lock);
	connected = (cur_conn != NULL);
	k_spin_unlock(&lock, key);
	if (connected) {
		return; /* only one connection: restarted when it is recycled */
	}

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err == 0) {
		LOG_INF("advertising");
	} else if (err == -EALREADY) {
		LOG_DBG("already advertising");
	} else {
		/* -ENOMEM: connection object not recycled yet, retry. */
		LOG_WRN("advertising start failed (%d), retrying", err);
		k_work_reschedule(&adv_work, K_MSEC(WP_BLE_ADV_RETRY_MS));
	}
}

static void adv_restart_soon(void)
{
	k_work_reschedule(&adv_work, K_NO_WAIT);
}

/* ------------------------------------------------------------------------ */
/* Link state                                                                */
/* ------------------------------------------------------------------------ */

/* Re-evaluate "HID ready" from the stack and post the transition, if any. */
static void link_update(void)
{
	struct bt_conn *conn = get_conn_ref();
	bool ready = false;
	bool connected = (conn != NULL);
	bool changed;
	k_spinlock_key_t key;

	if (conn != NULL) {
		ready = bt_conn_get_security(conn) >= BT_SECURITY_L2 &&
			wp_ble_hog_is_subscribed(conn);
		bt_conn_unref(conn);
	}

	key = k_spin_lock(&lock);
	changed = (ready != link_ready);
	if (changed) {
		link_ready = ready;
		if (ready) {
			ready_gen++;
		}
		/* inflight is NOT reset: the queued PDUs still hold ATT buffers
		 * and complete normally on this connection. */
	}
	k_spin_unlock(&lock, key);

	if (changed) {
		LOG_INF("HID %s", ready ? "ready" : "not ready");
		if (ready) {
			(void)k_work_cancel_delayable(&ready_work);
		} else if (connected) {
			(void)k_work_reschedule(&ready_work, K_SECONDS(WP_BLE_READY_TIMEOUT_S));
		}
		(void)wp_router_post_hid_connected(ready);
	}
}

/* A connection that never becomes HID ready holds the only slot and keeps
 * advertising off: drop it. */
static void ready_work_handler(struct k_work *work)
{
	struct bt_conn *conn = get_conn_ref();
	bool ready;
	k_spinlock_key_t key;

	ARG_UNUSED(work);
	if (conn == NULL) {
		return;
	}
	key = k_spin_lock(&lock);
	ready = link_ready;
	k_spin_unlock(&lock, key);

	if (!ready) {
		LOG_WRN("connection not HID ready in time: disconnecting");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
	}
	bt_conn_unref(conn);
}

/* Shorten the ready timeout (encryption / pairing failed). */
static void ready_timeout_shorten(void)
{
	if (k_work_delayable_remaining_get(&ready_work) >
	    k_ms_to_ticks_ceil32(WP_BLE_READY_TIMEOUT_FAIL_S * 1000U)) {
		(void)k_work_reschedule(&ready_work, K_SECONDS(WP_BLE_READY_TIMEOUT_FAIL_S));
	}
}

static int apply_conn_profile(struct bt_conn *conn, bool low_latency)
{
	const struct bt_le_conn_param *param =
		low_latency ? &conn_param_low_latency : &conn_param_power_saving;
	int err;

	/* Before the stack's peripheral update timer (5 s after connecting)
	 * this only stores the request; the stack sends it when the timer fires.
	 */
	err = bt_conn_le_param_update(conn, param);
	if (err != 0) {
		LOG_WRN("conn param update (%s) failed: %d", low_latency ? "low latency" : "power save",
			err);
	} else {
		LOG_DBG("requested %s connection parameters",
			low_latency ? "low latency" : "power saving");
	}
	return err;
}

/* ------------------------------------------------------------------------ */
/* Connection callbacks                                                      */
/* ------------------------------------------------------------------------ */

static void connected(struct bt_conn *conn, uint8_t err)
{
	bool busy;
	bool low_latency = true;
	k_spinlock_key_t key;

	if (err != 0U) {
		LOG_WRN("connection failed: 0x%02x %s", err, bt_hci_err_to_str(err));
		adv_restart_soon();
		return;
	}

	key = k_spin_lock(&lock);
	busy = (cur_conn != NULL);
	if (!busy) {
		cur_conn = bt_conn_ref(conn);
		conn_gen++;
		inflight = 0;
		param_attempts = 0U;
		low_latency = want_low_latency;
		requested_profile = low_latency ? PROFILE_LOW_LATENCY : PROFILE_POWER_SAVING;
	}
	k_spin_unlock(&lock, key);

	if (busy) {
		/* Cannot normally happen with CONFIG_BT_MAX_CONN=1. */
		LOG_WRN("second connection rejected");
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	LOG_INF("connected");
	wp_ble_hog_reset_input();
	(void)k_work_reschedule(&ready_work, K_SECONDS(WP_BLE_READY_TIMEOUT_S));

	/* Just Works bonding: request encryption (L2) right away. */
	if (bt_conn_set_security(conn, BT_SECURITY_L2) != 0) {
		LOG_WRN("failed to request security");
	}

	(void)apply_conn_profile(conn, low_latency);
	link_update();
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *old = NULL;
	k_spinlock_key_t key;

	key = k_spin_lock(&lock);
	if (cur_conn == conn) {
		old = cur_conn;
		cur_conn = NULL;
		requested_profile = PROFILE_NONE;
		/* Queued PDUs are flushed and their callbacks dropped now. */
		conn_gen++;
		inflight = 0;
	}
	k_spin_unlock(&lock, key);

	if (old == NULL) {
		return;
	}

	LOG_INF("disconnected: 0x%02x %s", reason, bt_hci_err_to_str(reason));
	(void)k_work_cancel_delayable(&ready_work);
	(void)k_work_cancel_delayable(&param_retry_work);
	wp_ble_hog_reset_input();
	bt_conn_unref(old);
	link_update();
	/* Advertising restarts from recycled(); the retry loop covers the
	 * case where the object is not free yet.
	 */
}

static void recycled(void)
{
	adv_restart_soon();
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	if (err != BT_SECURITY_ERR_SUCCESS) {
		LOG_WRN("security failed: level %u err %d %s", level, err,
			bt_security_err_to_str(err));
		/* Some centrals re-pair on the same link: no immediate drop. */
		ready_timeout_shorten();
	} else {
		LOG_INF("security level %u", level);
	}
	link_update();
}

/* The central did not apply (or later changed) the parameters we want:
 * re-request a few times with back-off (ble.c owns no other retry). */
static void param_mismatch(void)
{
	uint8_t n;
	k_spinlock_key_t key = k_spin_lock(&lock);

	n = param_attempts;
	if (n < WP_BLE_PARAM_RETRY_MAX) {
		param_attempts++;
	}
	requested_profile = PROFILE_NONE; /* the next request is sent again */
	k_spin_unlock(&lock, key);

	if (n >= WP_BLE_PARAM_RETRY_MAX) {
		LOG_WRN("central keeps other connection parameters (until the next profile change)");
		return;
	}
	(void)k_work_reschedule_for_queue(wp_router_ctl_queue(), &param_retry_work,
					  K_SECONDS(WP_BLE_PARAM_RETRY_S << n));
}

static void param_retry_handler(struct k_work *work)
{
	bool ll;
	k_spinlock_key_t key = k_spin_lock(&lock);

	ARG_UNUSED(work);
	ll = want_low_latency;
	k_spin_unlock(&lock, key);
	(void)wp_ble_set_low_latency(ll);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
			     uint16_t timeout)
{
	const struct bt_le_conn_param *want;
	bool ll;
	k_spinlock_key_t key;

	ARG_UNUSED(conn);
	LOG_INF("conn params: interval %u.%02u ms, latency %u, timeout %u ms",
		(interval * 125U) / 100U, (interval * 125U) % 100U, latency, timeout * 10U);

	key = k_spin_lock(&lock);
	ll = want_low_latency;
	k_spin_unlock(&lock, key);

	/* The supervision timeout is the central's choice: not compared. */
	want = ll ? &conn_param_low_latency : &conn_param_power_saving;
	if (interval < want->interval_min || interval > want->interval_max ||
	    latency != want->latency) {
		param_mismatch();
	} else {
		key = k_spin_lock(&lock);
		param_attempts = 0U;
		k_spin_unlock(&lock, key);
	}
}

#if defined(CONFIG_BT_USER_CONN_PARAM_REJECTED)
static void le_param_update_rejected(struct bt_conn *conn, uint8_t hci_err)
{
	ARG_UNUSED(conn);
	LOG_WRN("conn param update rejected: 0x%02x %s", hci_err, bt_hci_err_to_str(hci_err));
	param_mismatch();
}
#endif

BT_CONN_CB_DEFINE(wp_ble_conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.security_changed = security_changed,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_USER_CONN_PARAM_REJECTED)
	.le_param_update_rejected = le_param_update_rejected,
#endif
};

/* ---- Pairing window ------------------------------------------------------ */

static void count_bond(const struct bt_bond_info *info, void *user_data)
{
	ARG_UNUSED(info);
	(*(int *)user_data)++;
}

static bool pairing_allowed(void)
{
	int bonds = 0;
	bool open;
	k_spinlock_key_t key = k_spin_lock(&lock);

	open = k_uptime_get() < pairing_until_ms;
	k_spin_unlock(&lock, key);
	if (open) {
		return true;
	}
	bt_foreach_bond(BT_ID_DEFAULT, count_bond, &bonds);
	return bonds == 0;
}

static enum bt_security_err pairing_accept(struct bt_conn *conn,
					   const struct bt_conn_pairing_feat *const feat)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(feat);

	if (pairing_allowed()) {
		return BT_SECURITY_ERR_SUCCESS;
	}
	LOG_WRN("pairing rejected: pairing window closed (open it with 'wp pair')");
	ready_timeout_shorten();
	return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
}

/* Only pairing_accept: IO capability stays NoInputNoOutput (Just Works). */
static struct bt_conn_auth_cb auth_cb = {
	.pairing_accept = pairing_accept,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	LOG_INF("pairing complete, %sbonded", bonded ? "" : "not ");
	if (bonded) {
		/* One host per window: close it. */
		(void)wp_ble_open_pairing_window(0U);
	}
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	LOG_WRN("pairing failed: %d %s", reason, bt_security_err_to_str(reason));
	ready_timeout_shorten();
}

static struct bt_conn_auth_info_cb auth_info_cb = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* Called by ble_hog.c (BT RX thread) when the input report CCC changes. */
void wp_ble_hog_ccc_changed(bool notify_enabled)
{
	LOG_INF("input report notifications %s", notify_enabled ? "on" : "off");
	link_update();
}

/* ------------------------------------------------------------------------ */
/* Flow control                                                              */
/* ------------------------------------------------------------------------ */

static void notify_complete(struct bt_conn *conn, void *user_data)
{
	uint32_t gen = POINTER_TO_UINT(user_data);
	bool current;
	k_spinlock_key_t key;

	ARG_UNUSED(conn);

	key = k_spin_lock(&lock);
	current = (gen == conn_gen);
	if (current) {
		if (inflight > 0) {
			inflight--;
		}
		tx_progress_ms = k_uptime_get();
	}
	k_spin_unlock(&lock, key);

	if (current) {
		(void)wp_router_post_hid_ready();
	}
}

/* Longest time a notification can stay uncompleted on a live link. */
static int64_t stall_limit_ms(void)
{
	struct bt_conn *conn = get_conn_ref();
	int64_t limit = CONFIG_WAFER_PAD_BLE_SUPERVISION_TIMEOUT_MS;
	struct bt_conn_info info;

	if (conn != NULL) {
		/* The central chooses the actual timeout. */
		if (bt_conn_get_info(conn, &info) == 0) {
			limit = MAX(limit, (int64_t)info.le.timeout * 10);
		}
		bt_conn_unref(conn);
	}
	return limit + WP_BLE_TX_STALL_MARGIN_MS;
}

static void stall_work_handler(struct k_work *work)
{
	int64_t limit_ms = stall_limit_ms();
	int64_t idle_ms = 0;
	bool stalled = false;
	bool pending = false;
	k_spinlock_key_t key;

	ARG_UNUSED(work);

	key = k_spin_lock(&lock);
	if (inflight > 0) {
		idle_ms = k_uptime_get() - tx_progress_ms;
		if (idle_ms >= limit_ms) {
			stalled = true;
		} else {
			pending = true;
		}
	}
	k_spin_unlock(&lock, key);

	if (stalled) {
		/* The PDUs still hold ATT buffers: reclaiming the slots would let
		 * the router block in bt_gatt_notify_cb(). Dropping the link
		 * flushes them; disconnected() resets the count. */
		struct bt_conn *conn = get_conn_ref();

		LOG_WRN("no notification completed for %lld ms: disconnecting",
			(long long)idle_ms);
		if (conn != NULL) {
			(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
			bt_conn_unref(conn);
		}
	} else if (pending) {
		k_work_schedule(&stall_work, K_MSEC(limit_ms - idle_ms));
	}
}

static uint32_t ble_hid_link_gen(void)
{
	uint32_t gen;
	k_spinlock_key_t key = k_spin_lock(&lock);

	gen = ready_gen;
	k_spin_unlock(&lock, key);
	return gen;
}

static bool ble_hid_is_ready(void)
{
	bool ready;
	k_spinlock_key_t key = k_spin_lock(&lock);

	ready = link_ready && cur_conn != NULL && inflight < CONFIG_WAFER_PAD_BLE_MAX_INFLIGHT;
	k_spin_unlock(&lock, key);
	return ready;
}

static int ble_hid_send_mouse(const struct wp_mouse_report *r)
{
	uint8_t payload[WP_HID_MOUSE_REPORT_LEN];
	struct bt_conn *conn = NULL;
	uint32_t gen = 0;
	int err = 0;
	k_spinlock_key_t key;

	if (wp_hid_mouse_report_pack(r, payload, sizeof(payload)) != WP_HID_MOUSE_REPORT_LEN) {
		return -EINVAL;
	}

	key = k_spin_lock(&lock);
	if (cur_conn == NULL) {
		err = -ENOTCONN;
	} else if (!link_ready) {
		err = -EAGAIN; /* e.g. CCC toggled: keep the report (may be a release) */
	} else if (inflight >= CONFIG_WAFER_PAD_BLE_MAX_INFLIGHT) {
		err = -EBUSY;
	} else {
		conn = bt_conn_ref(cur_conn);
		gen = conn_gen;
		if (inflight == 0) {
			tx_progress_ms = k_uptime_get();
		}
		inflight++;
	}
	k_spin_unlock(&lock, key);

	if (err != 0) {
		return err;
	}

	err = wp_ble_hog_notify(conn, payload, sizeof(payload), notify_complete,
				UINT_TO_POINTER(gen));
	bt_conn_unref(conn);

	if (err != 0) {
		key = k_spin_lock(&lock);
		if (gen == conn_gen && inflight > 0) {
			inflight--;
		}
		k_spin_unlock(&lock, key);
		LOG_DBG("notify failed: %d", err);
		return err;
	}

	/* No-op while already scheduled; the handler re-arms itself. */
	k_work_schedule(&stall_work, K_MSEC(CONFIG_WAFER_PAD_BLE_SUPERVISION_TIMEOUT_MS +
					    WP_BLE_TX_STALL_MARGIN_MS));
	return 0;
}

/* ------------------------------------------------------------------------ */
/* Init                                                                      */
/* ------------------------------------------------------------------------ */

static int ble_hid_init(void)
{
	int err;

	if (atomic_get(&ble_ready)) {
		return 0;
	}

	err = wp_ble_hog_init();
	if (err != 0) {
		return err;
	}

	err = bt_conn_auth_info_cb_register(&auth_info_cb);
	if (err != 0) {
		LOG_WRN("auth info callbacks not registered: %d", err);
	}

	/* Synchronous: returns when the stack is ready. -EALREADY can also mean
	 * "a previous bt_enable() failed half-way": trust only bt_is_ready(). */
	err = bt_enable(NULL);
	if (err == -EALREADY && !bt_is_ready()) {
		err = -EIO;
	}
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("bt_enable failed: %d", err);
		return err;
	}

	err = bt_conn_auth_cb_register(&auth_cb);
	if (err != 0 && err != -EALREADY) {
		LOG_ERR("pairing policy not registered: %d", err);
		return err;
	}

	/* Bonds, CCC and identity are restored from the settings backend. */
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		err = settings_load();
		if (err != 0) {
			LOG_WRN("settings_load failed: %d (bonds lost?)", err);
		}
	}

	(void)wp_ble_open_pairing_window(CONFIG_WAFER_PAD_BLE_PAIRING_WINDOW_S);
	atomic_set(&ble_ready, 1);
	adv_restart_soon();
	LOG_INF("BLE ready (\"%s\")", CONFIG_BT_DEVICE_NAME);
	return 0;
}

const struct wp_hid_transport_api wp_hid_ble_transport = {
	.name = "ble",
	.init = ble_hid_init,
	.is_ready = ble_hid_is_ready,
	.send_mouse = ble_hid_send_mouse,
	.link_gen = ble_hid_link_gen,
};

/* ------------------------------------------------------------------------ */
/* Public API (include/wafer_pad/platform.h)                                 */
/* ------------------------------------------------------------------------ */

bool wp_ble_is_connected(void)
{
	bool connected;
	k_spinlock_key_t key = k_spin_lock(&lock);

	connected = (cur_conn != NULL);
	k_spin_unlock(&lock, key);
	return connected;
}

int wp_ble_set_low_latency(bool low_latency)
{
	enum conn_profile want = low_latency ? PROFILE_LOW_LATENCY : PROFILE_POWER_SAVING;
	struct bt_conn *conn = NULL;
	bool needed = false;
	int err;
	k_spinlock_key_t key;

	key = k_spin_lock(&lock);
	if (want_low_latency != low_latency) {
		param_attempts = 0U; /* new profile: fresh re-request budget */
	}
	want_low_latency = low_latency;
	if (cur_conn != NULL && requested_profile != want) {
		conn = bt_conn_ref(cur_conn);
		requested_profile = want;
		needed = true;
	}
	k_spin_unlock(&lock, key);

	if (!needed) {
		return 0; /* already requested, or applied at the next connection */
	}

	err = apply_conn_profile(conn, low_latency);
	if (err != 0) {
		key = k_spin_lock(&lock);
		if (cur_conn == conn) {
			requested_profile = PROFILE_NONE; /* allow a retry */
		}
		k_spin_unlock(&lock, key);
	}
	bt_conn_unref(conn);
	return err;
}

int wp_ble_unpair_all(void)
{
	struct bt_conn *conn;
	int err;

	if (!atomic_get(&ble_ready)) {
		return -EAGAIN;
	}

	err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
	if (err != 0) {
		LOG_ERR("unpair failed: %d", err);
		return err;
	}
	LOG_INF("all bonds removed");

	conn = get_conn_ref();
	if (conn != NULL) {
		/* bt_unpair() already drops bonded links; also drop an unbonded
		 * one. Advertising restarts when the connection is recycled.
		 */
		(void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		bt_conn_unref(conn);
	} else {
		(void)bt_le_adv_stop();
		adv_restart_soon();
	}
	return 0;
}

int wp_ble_open_pairing_window(uint32_t seconds)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	pairing_until_ms = (seconds == 0U) ? 0 : k_uptime_get() + (int64_t)seconds * 1000;
	k_spin_unlock(&lock, key);
	if (seconds != 0U) {
		LOG_INF("pairing window open for %u s", seconds);
	} else {
		LOG_INF("pairing window closed");
	}
	return 0;
}

uint32_t wp_ble_pairing_window_left(void)
{
	int64_t left;
	k_spinlock_key_t key = k_spin_lock(&lock);

	left = pairing_until_ms - k_uptime_get();
	k_spin_unlock(&lock, key);
	return (left > 0) ? (uint32_t)((left + 999) / 1000) : 0U;
}

int wp_ble_set_battery_level(uint8_t percent)
{
#if defined(CONFIG_BT_BAS)
	if (!atomic_get(&ble_ready)) {
		return -EAGAIN;
	}
	return bt_bas_set_battery_level(MIN(percent, 100U));
#else
	ARG_UNUSED(percent);
	return -ENOTSUP;
#endif
}
