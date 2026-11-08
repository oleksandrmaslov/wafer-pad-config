/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Event router: the ONE consumer thread of the firmware.
 *
 * Producers (touch source, IMU source, HID transports, shell) post tagged
 * messages into a single k_msgq and never block. The router thread owns every
 * algorithm instance (state machine, motion features, orientation manager,
 * calibration gesture, gesture engine, air mouse, HID report composer), so no
 * algorithm needs locking. Everything that may block on a bus (sensor power
 * modes, re-initialization, connection parameters, System OFF) is handed to
 * a dedicated control work queue (NOT the system work queue, which the BT
 * host needs for TX completions) through a "desired state" bitmask, so the
 * cursor path never waits on I2C. Failed power-mode changes are retried
 * with back-off until the applied state matches the desired one.
 *
 * Loss protection when the queue is full:
 *   - status / connection / request messages fall back to a per-type latch
 *     (latest value wins). Every post carries a sequence number; a latched
 *     value is applied only after every message posted before it has been
 *     handled (never ahead of an older queued message of the same type);
 *   - the newest dropped touch frame is kept and replayed once the queue has
 *     drained, so a finger lift can never be lost (no stuck drag button);
 *     frames captured before the last touch status (reset / error / re-init)
 *     are discarded, so a replay cannot resurrect a lost contact;
 *   - IMU samples and "HID ready" kicks are simply dropped (stateless).
 *
 * HID output is gated by the router's own, in-order view of the link
 * (rt.hid_connected): nothing is accumulated while disconnected, and a new
 * transport link generation forces a full button-state resync.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/air_mouse.h>
#include <wafer_pad/config.h>
#include <wafer_pad/gestures.h>
#include <wafer_pad/hid_report.h>
#include <wafer_pad/imu_algo.h>
#include <wafer_pad/orientation.h>
#include <wafer_pad/orientation_calib.h>
#include <wafer_pad/orientation_transform.h>
#include <wafer_pad/platform.h>
#include <wafer_pad/state_machine.h>
#include <wafer_pad/types.h>

LOG_MODULE_REGISTER(wp_router, CONFIG_WAFER_PAD_LOG_LEVEL);

/* Nominal TPS65 output resolution (Azoteq TPS65 module datasheet, Table 1.1:
 * 3072 x 2048). Only used until the touch source reports the real extent. */
#define FALLBACK_MAX_X 3072U
#define FALLBACK_MAX_Y 2048U
/* TPS65 module outline (65 x 49 mm), used if the touch config gives no size. */
#define FALLBACK_W_MM  65.0f
#define FALLBACK_H_MM  49.0f

/* No IMU sample for this long -> motion features are invalid. Covers the
 * lowest IDLE data rate with a wide margin. */
#define IMU_STALE_US        (500LL * 1000LL)
/* Latencies above this are clock mismatches / stale stamps, not measured. */
#define LATENCY_SANE_MAX_US (2LL * 1000LL * 1000LL)
/* Max reports pushed per flush (the composer splits big motions). */
#define FLUSH_BUDGET        6
/* IMU recovery back-off (independent of the touch ERROR_RECOVERY state:
 * the trackpad keeps working without the IMU). */
#define IMU_RETRY_MIN_MS    1000
#define IMU_RETRY_MAX_MS    8000
/* Retry back-off for failed power-mode / connection-parameter changes. */
#define CTL_RETRY_MIN_MS    500
#define CTL_RETRY_MAX_MS    8000

/* Control work queue: below the router and the sensor threads. */
#define CTL_WQ_STACK_SIZE   3072
#define CTL_WQ_PRIO \
	K_PRIO_PREEMPT(MIN(CONFIG_WAFER_PAD_ROUTER_PRIORITY + 2, CONFIG_NUM_PREEMPT_PRIORITIES - 1))

/* ------------------------------------------------------------------------ */
/* Messages                                                                 */
/* ------------------------------------------------------------------------ */

enum msg_type {
	MSG_TOUCH = 0,
	MSG_TOUCH_STATUS,
	MSG_IMU,
	MSG_IMU_STATUS,
	MSG_HID_CONNECTED,
	MSG_HID_READY,
	MSG_KEY,          /* reserved: future keyboard halves (never queued in v0.1) */
	MSG_ORIENT_REQ,
	MSG_MODE_REQ,
	MSG_SLEEP_FAILED, /* internal: System OFF could not be entered */
};

struct wp_msg {
	uint8_t type;
	uint32_t seq;      /* post order (wraps; compare with seq_before()) */
	int64_t post_us;   /* control messages: time of the post */
	union {
		struct wp_touch_frame touch;
		struct wp_imu_sample imu;
		struct wp_key_event key;
		int32_t value; /* status enum, bool, degrees or mode */
	} u;
};

K_MSGQ_DEFINE(router_q, sizeof(struct wp_msg), CONFIG_WAFER_PAD_ROUTER_QUEUE_DEPTH, 4);

static atomic_t queue_drops;
static atomic_t post_seq;

/* Per-type latches for control messages that did not fit into the queue. */
enum latch_id {
	LATCH_NONE = -1,
	LATCH_TOUCH_STATUS = 0,
	LATCH_IMU_STATUS,
	LATCH_HID_CONN,
	LATCH_ORIENT_REQ,
	LATCH_MODE_REQ,
	LATCH_SLEEP_FAILED,
	LATCH_COUNT,
};

struct latch {
	bool set;
	int32_t value;
	uint32_t seq;
	int64_t post_us;
};

static struct k_spinlock latch_lock;
static struct latch latches[LATCH_COUNT];

/* Coalesced "HID ready" kick. */
#define POST_HID_READY_PENDING 0
static atomic_t post_flags;

/* Newest touch frame that did not fit into the queue. */
static struct k_spinlock drop_lock;
static struct wp_touch_frame dropped_frame;
static bool dropped_frame_valid;

/* ------------------------------------------------------------------------ */
/* Control work (control work queue): power modes, reinit, sleep           */
/* ------------------------------------------------------------------------ */

enum ctl_bit {
	CTL_WANT_GYRO = 0,
	CTL_WANT_IMU_LOW_RATE,
	CTL_WANT_TOUCH_LP,
	CTL_WANT_BLE_LL,
	CTL_REQ_TOUCH_REINIT,
	CTL_REQ_SLEEP,
	CTL_REAPPLY_IMU,
	CTL_REAPPLY_TOUCH,
	CTL_REAPPLY_BLE,
	CTL_IMU_READY,   /* IMU initialized: power mode calls allowed */
	CTL_TOUCH_READY, /* touch initialized: power mode calls allowed */
	CTL_BLE_READY,   /* connected: connection parameter requests allowed */
	CTL_GYRO_ON,     /* status: gyroscope powered */
};

static atomic_t ctl_bits;
static struct k_work ctl_work;
static struct k_work_delayable ctl_retry_work;
static struct k_work_delayable imu_retry_work;
static atomic_t imu_retry_ms = ATOMIC_INIT(IMU_RETRY_MIN_MS);
static uint32_t ctl_retry_ms = CTL_RETRY_MIN_MS; /* control queue thread only */

K_THREAD_STACK_DEFINE(ctl_wq_stack, CTL_WQ_STACK_SIZE);
static struct k_work_q ctl_wq;

/* Applied state, touched only by the control work queue thread. */
static struct {
	bool imu_valid;
	bool touch_valid;
	bool ble_valid;
	bool gyro;
	bool imu_low_rate;
	bool touch_lp;
	bool ble_ll;
} ctl_applied;

/* ------------------------------------------------------------------------ */
/* Router state (router thread only, after wp_router_init())                */
/* ------------------------------------------------------------------------ */

struct router {
	/* algorithm instances */
	struct wp_sm sm;
	struct wp_sm_config sm_cfg;
	struct wp_motion motion;
	struct wp_motion_features features;
	struct wp_orient_mgr orient;
	struct wp_calib calib;
	struct wp_calib_config calib_cfg;
	struct wp_gesture_engine gest;
	struct wp_air_mouse air;
	struct wp_hid_composer comp;

	/* derived / platform state */
	struct wp_pad_geometry pad_geom;   /* P frame */
	float fallback_w_mm;
	float fallback_h_mm;
	enum wp_orientation orientation;   /* orientation the gesture engine uses */
	struct wp_sm_outputs out;          /* last applied state machine outputs */
	bool out_valid;
	uint8_t contacts;
	uint8_t held;
	bool single_still;
	uint32_t single_still_ms;
	bool touch_ok;
	bool imu_present;
	bool imu_ok;
	int64_t last_imu_rx_us;            /* router clock */
	bool hid_connected;
	uint32_t flush_gen;                /* transport link generation seen */
	bool sleep_requested;
	int64_t touch_lost_us;             /* post time of the last touch status */
	struct wp_imu_sample last_imu;     /* for "wp imu" */
	bool last_imu_valid;
	int64_t algo_t;                    /* monotonic time for gesture/calib/sm */
	int64_t last_frame_t;
	int led_base;

	/* HID output */
	struct wp_mouse_report retry;
	bool retry_valid;
	bool stamp_valid;
	int64_t stamp_us;

	/* counters */
	uint32_t touch_frames;
	uint32_t imu_samples;
	uint32_t reports_sent;
	uint32_t max_latency_us;
};

static struct router rt;
static bool router_started;

static struct k_spinlock status_lock;
static struct wp_router_status status_snap;

K_THREAD_STACK_DEFINE(router_stack, CONFIG_WAFER_PAD_ROUTER_STACK_SIZE);
static struct k_thread router_thread;

static int64_t now_us(void)
{
	return (int64_t)k_ticks_to_us_floor64((uint64_t)k_uptime_ticks());
}

static int pct(float x)
{
	return (int)(x * 100.0f + (x >= 0.0f ? 0.5f : -0.5f));
}

/** a was posted before b (wrap-safe). */
static bool seq_before(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b) < 0;
}

static uint32_t next_seq(void)
{
	return (uint32_t)atomic_inc(&post_seq);
}

static void ctl_submit(void)
{
	(void)k_work_submit_to_queue(&ctl_wq, &ctl_work);
}

struct k_work_q *wp_router_ctl_queue(void)
{
	return &ctl_wq;
}

/* ------------------------------------------------------------------------ */
/* Posting (any thread, never blocks)                                       */
/* ------------------------------------------------------------------------ */

static int post_ctrl(uint8_t type, int32_t value, enum latch_id latch)
{
	struct wp_msg m;
	k_spinlock_key_t key;

	memset(&m, 0, sizeof(m));
	m.type = type;
	m.u.value = value;
	m.post_us = now_us();
	m.seq = next_seq();

	if (k_msgq_put(&router_q, &m, K_NO_WAIT) == 0) {
		/* A newer value is queued: an older latched one is obsolete. */
		if (latch != LATCH_NONE) {
			key = k_spin_lock(&latch_lock);
			if (latches[latch].set && seq_before(latches[latch].seq, m.seq)) {
				latches[latch].set = false;
			}
			k_spin_unlock(&latch_lock, key);
		}
		return 0;
	}

	atomic_inc(&queue_drops);
	if (latch != LATCH_NONE) {
		key = k_spin_lock(&latch_lock);
		latches[latch] = (struct latch){
			.set = true, .value = value, .seq = m.seq, .post_us = m.post_us};
		k_spin_unlock(&latch_lock, key);
		return 0; /* not lost: applied in post order by the router */
	}
	return -ENOMSG;
}

int wp_router_post_touch(const struct wp_touch_frame *pad_frame)
{
	struct wp_msg m;

	if (pad_frame == NULL) {
		return -EINVAL;
	}

	m.type = MSG_TOUCH;
	m.seq = next_seq();
	m.post_us = 0;
	m.u.touch = *pad_frame;
	if (m.u.touch.count > WP_TOUCH_MAX_CONTACTS) {
		m.u.touch.count = WP_TOUCH_MAX_CONTACTS;
	}

	if (k_msgq_put(&router_q, &m, K_NO_WAIT) == 0) {
		return 0;
	}

	atomic_inc(&queue_drops);

	k_spinlock_key_t key = k_spin_lock(&drop_lock);

	dropped_frame = m.u.touch;
	dropped_frame_valid = true;
	k_spin_unlock(&drop_lock, key);

	return -ENOMSG;
}

int wp_router_post_touch_status(enum wp_touch_status status)
{
	return post_ctrl(MSG_TOUCH_STATUS, (int32_t)status, LATCH_TOUCH_STATUS);
}

int wp_router_post_imu(const struct wp_imu_sample *sample)
{
	struct wp_msg m;

	if (sample == NULL) {
		return -EINVAL;
	}

	m.type = MSG_IMU;
	m.seq = next_seq();
	m.post_us = 0;
	m.u.imu = *sample;

	if (k_msgq_put(&router_q, &m, K_NO_WAIT) != 0) {
		atomic_inc(&queue_drops);
		return -ENOMSG;
	}
	return 0;
}

int wp_router_post_imu_status(enum wp_imu_status status)
{
	return post_ctrl(MSG_IMU_STATUS, (int32_t)status, LATCH_IMU_STATUS);
}

int wp_router_post_hid_connected(bool connected)
{
	return post_ctrl(MSG_HID_CONNECTED, connected ? 1 : 0, LATCH_HID_CONN);
}

int wp_router_post_hid_ready(void)
{
	struct wp_msg m;

	if (atomic_test_and_set_bit(&post_flags, POST_HID_READY_PENDING)) {
		return 0; /* a kick is already queued */
	}

	m.type = MSG_HID_READY;
	m.seq = next_seq();
	m.post_us = 0;
	m.u.value = 0;
	if (k_msgq_put(&router_q, &m, K_NO_WAIT) != 0) {
		/* Harmless: the router flushes the composer on every tick. */
		atomic_clear_bit(&post_flags, POST_HID_READY_PENDING);
		return -ENOMSG;
	}
	return 0;
}

int wp_router_post_key(const struct wp_key_event *ev)
{
	ARG_UNUSED(ev);
	/*
	 * FUTURE (Wafer split keyboard halves): key events will be queued here
	 * as MSG_KEY (the union member already exists) and handled in
	 * handle_msg(): keyboard usages (page 0x07) go to a keyboard report
	 * composer for WP_HID_REPORT_ID_KEYBOARD, consumer usages (page 0x0C)
	 * to WP_HID_REPORT_ID_CONSUMER, both flushed next to the mouse report in
	 * hid_flush(). Neither report exists in the v0.1 descriptor.
	 */
	return -ENOTSUP;
}

int wp_router_request_orientation(int deg)
{
	if (deg != -1 && (deg < 0 || deg >= 360 || (deg % 90) != 0)) {
		return -EINVAL;
	}
	return post_ctrl(MSG_ORIENT_REQ, deg, LATCH_ORIENT_REQ);
}

int wp_router_request_mode(int mode)
{
	if (mode < -1 || mode > (int)WP_MODE_FORCE_TRACKPAD) {
		return -EINVAL;
	}
	return post_ctrl(MSG_MODE_REQ, mode, LATCH_MODE_REQ);
}

void wp_router_get_status(struct wp_router_status *st)
{
	if (st == NULL) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&status_lock);

	*st = status_snap;
	k_spin_unlock(&status_lock, key);

	st->queue_drops = (uint32_t)atomic_get(&queue_drops);
	st->gyro_on = atomic_test_bit(&ctl_bits, CTL_GYRO_ON);
}

/* ------------------------------------------------------------------------ */
/* Control work handlers (control work queue)                               */
/* ------------------------------------------------------------------------ */

static bool err_is_absent(int err)
{
	return err == -ENODEV || err == -ENOTSUP;
}

/* A failed BLE parameter request is final only if it can never succeed on
 * this link (re-applied on the next connection through CTL_REAPPLY_BLE). */
static bool ble_err_is_final(int err)
{
	return err == 0 || err == -ENOTCONN || err == -EINVAL;
}

static void ctl_work_handler(struct k_work *work)
{
	bool retry = false;
	bool want;
	int err;

	ARG_UNUSED(work);

	if (atomic_test_and_clear_bit(&ctl_bits, CTL_REQ_SLEEP)) {
		/* Returns only if System OFF could not be entered. */
		err = wp_power_enter_sleep();
		LOG_ERR("System OFF not entered (%d), resuming", err);
		(void)post_ctrl(MSG_SLEEP_FAILED, err, LATCH_SLEEP_FAILED);
	}

	if (atomic_test_and_clear_bit(&ctl_bits, CTL_REQ_TOUCH_REINIT)) {
		err = wp_touch_source_reinit();
		if (err == 0) {
			ctl_applied.touch_valid = false;
			(void)wp_router_post_touch_status(WP_TOUCH_STATUS_OK);
		} else {
			LOG_WRN("touch re-init failed (%d)", err);
		}
	}

	if (atomic_test_and_clear_bit(&ctl_bits, CTL_REAPPLY_IMU)) {
		ctl_applied.imu_valid = false;
	}
	if (atomic_test_and_clear_bit(&ctl_bits, CTL_REAPPLY_TOUCH)) {
		ctl_applied.touch_valid = false;
	}
	if (atomic_test_and_clear_bit(&ctl_bits, CTL_REAPPLY_BLE)) {
		ctl_applied.ble_valid = false;
	}

	if (!atomic_test_bit(&ctl_bits, CTL_IMU_READY)) {
		ctl_applied.imu_valid = false;
		atomic_clear_bit(&ctl_bits, CTL_GYRO_ON);
	} else {
		/* Asynchronous requests to the IMU thread: only fail if absent. */
		want = atomic_test_bit(&ctl_bits, CTL_WANT_GYRO);
		if (!ctl_applied.imu_valid || want != ctl_applied.gyro) {
			err = wp_imu_source_set_gyro(want);
			if (err != 0 && !err_is_absent(err)) {
				LOG_WRN("gyro %s failed (%d)", want ? "on" : "off", err);
			}
			atomic_set_bit_to(&ctl_bits, CTL_GYRO_ON, want && err == 0);
			ctl_applied.gyro = want;
		}

		want = atomic_test_bit(&ctl_bits, CTL_WANT_IMU_LOW_RATE);
		if (!ctl_applied.imu_valid || want != ctl_applied.imu_low_rate) {
			err = wp_imu_source_set_low_rate(want);
			if (err != 0 && !err_is_absent(err)) {
				LOG_WRN("IMU low rate %d failed (%d)", want, err);
			}
			ctl_applied.imu_low_rate = want;
		}
		ctl_applied.imu_valid = true;
	}

	if (!atomic_test_bit(&ctl_bits, CTL_TOUCH_READY)) {
		ctl_applied.touch_valid = false;
	} else {
		want = atomic_test_bit(&ctl_bits, CTL_WANT_TOUCH_LP);
		if (!ctl_applied.touch_valid || want != ctl_applied.touch_lp) {
			err = wp_touch_source_set_low_power(want);
			if (err == 0 || err_is_absent(err)) {
				ctl_applied.touch_lp = want;
				ctl_applied.touch_valid = true;
			} else {
				/* Forced I2C can fail while the controller is busy:
				 * keep the desired state and retry. */
				LOG_WRN("touch low power %d failed (%d), retrying", want, err);
				ctl_applied.touch_valid = false;
				retry = true;
			}
		}
	}

	if (!atomic_test_bit(&ctl_bits, CTL_BLE_READY)) {
		ctl_applied.ble_valid = false;
	} else {
		want = atomic_test_bit(&ctl_bits, CTL_WANT_BLE_LL);
		if (!ctl_applied.ble_valid || want != ctl_applied.ble_ll) {
			err = wp_ble_set_low_latency(want);
			if (ble_err_is_final(err)) {
				LOG_DBG("BLE low latency %d (%d)", want, err);
				ctl_applied.ble_ll = want;
				ctl_applied.ble_valid = true;
			} else {
				/* Controller busy / no buffer / HCI disallowed. */
				LOG_WRN("BLE low latency %d failed (%d), retrying", want, err);
				ctl_applied.ble_valid = false;
				retry = true;
			}
		}
	}

	if (retry) {
		(void)k_work_reschedule_for_queue(&ctl_wq, &ctl_retry_work, K_MSEC(ctl_retry_ms));
		ctl_retry_ms = MIN(ctl_retry_ms * 2U, (uint32_t)CTL_RETRY_MAX_MS);
	} else {
		ctl_retry_ms = CTL_RETRY_MIN_MS;
	}
}

static void ctl_retry_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	ctl_submit();
}

/*
 * Backstop for an IMU that failed: the IMU thread itself retries every
 * second while failing and reports the outcome (OK on recovery, which
 * cancels this work and resets the back-off). wp_imu_source_reinit() is
 * asynchronous, so its 0 is NOT a recovery: keep backing off.
 */
static void imu_retry_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	int err = wp_imu_source_reinit();

	if (err == -ENODEV) {
		(void)wp_router_post_imu_status(WP_IMU_STATUS_ABSENT);
		return;
	}

	atomic_val_t ms = MIN(atomic_get(&imu_retry_ms) * 2, (atomic_val_t)IMU_RETRY_MAX_MS);

	atomic_set(&imu_retry_ms, ms);
	if (err != 0) {
		LOG_WRN("IMU re-init failed (%d), next try in %ld ms", err, (long)ms);
	} else {
		LOG_DBG("IMU re-init requested, next check in %ld ms", (long)ms);
	}
	(void)k_work_schedule_for_queue(&ctl_wq, dwork, K_MSEC(ms));
}

/* ------------------------------------------------------------------------ */
/* Output helpers (router thread)                                           */
/* ------------------------------------------------------------------------ */

static int64_t algo_time(int64_t t)
{
	/* Gesture / calibration / state machine timers must never run
	 * backwards (frames carry capture time, ticks carry "now"). */
	if (t > rt.algo_t) {
		rt.algo_t = t;
	}
	return rt.algo_t;
}

static void stamp_input(int64_t t_in)
{
	if (!rt.stamp_valid || t_in > rt.stamp_us) {
		rt.stamp_us = t_in;
	}
	rt.stamp_valid = true;
}

static void apply_gesture(const struct wp_gesture_output *go, int64_t t, int64_t t_in)
{
	bool input = false;

	rt.contacts = go->contacts;
	rt.single_still = go->single_still;
	rt.single_still_ms = go->single_still_ms;

	wp_hid_composer_set_held(&rt.comp, go->buttons);
	if (go->buttons != rt.held) {
		rt.held = go->buttons;
		input = true;
	}

	/* Nothing is accumulated while no host listens: it would be sent as a
	 * stale burst (cursor jump, old clicks) on reconnect. The held buttons
	 * above are state, re-asserted after the connect. */
	if (rt.hid_connected && rt.out_valid && rt.out.pointer_from_touch &&
	    (go->dx != 0.0f || go->dy != 0.0f)) {
		wp_hid_composer_add_motion(&rt.comp, go->dx, go->dy);
		input = true;
	}

	if (rt.hid_connected && (go->wheel != 0.0f || go->pan != 0.0f)) {
		wp_hid_composer_add_scroll(&rt.comp, go->wheel, go->pan);
		input = true;
	}

	if (rt.hid_connected && go->click != 0U) {
		wp_hid_composer_click(&rt.comp, go->click);
		input = true;
	}

	if ((go->events & WP_GEST_EV_THREE_FINGER_TAP) != 0U) {
		LOG_INF("three-finger tap: mode toggle");
		wp_sm_event(&rt.sm, WP_SM_EV_MODE_TOGGLE, t);
	}

	if (input) {
		stamp_input(t_in);
	}
}

/* Forget everything the host may have missed and re-send the full state. */
static void hid_resync(void)
{
	wp_hid_composer_release_all(&rt.comp);
	wp_hid_composer_force_sync(&rt.comp);
	rt.held = 0U;
	rt.retry_valid = false;
	rt.stamp_valid = false;
	rt.flush_gen = wp_hid_link_gen();
}

static void hid_flush(void)
{
	/* The router's in-order view of the link gates sending: the transport
	 * flag turns ready before MSG_HID_CONNECTED (with its release_all) has
	 * been handled. */
	if (!rt.hid_connected) {
		return;
	}
	if (wp_hid_link_gen() != rt.flush_gen) {
		/* The link was (re)established behind our back, e.g. a
		 * disconnect/connect pair merged in the latch. */
		LOG_DBG("HID link generation changed: resync");
		hid_resync();
	}

	for (int i = 0; i < FLUSH_BUDGET; i++) {
		struct wp_mouse_report r;
		int err;

		if (!wp_hid_is_ready()) {
			return;
		}

		if (rt.retry_valid) {
			r = rt.retry;
		} else if (!wp_hid_composer_next(&rt.comp, &r)) {
			return;
		}

		err = wp_hid_send_mouse(&r);
		if (err == 0) {
			rt.retry_valid = false;
			rt.reports_sent++;
			if (rt.stamp_valid) {
				int64_t lat = now_us() - rt.stamp_us;

				if (lat >= 0 && lat < LATENCY_SANE_MAX_US &&
				    (uint32_t)lat > rt.max_latency_us) {
					rt.max_latency_us = (uint32_t)lat;
				}
				rt.stamp_valid = false;
			}
			continue;
		}

		if (err == -EAGAIN || err == -ENOMEM || err == -ENOBUFS || err == -EBUSY) {
			/* Transient: keep this exact report (it may carry a button
			 * release) and retry it before anything newer. */
			rt.retry = r;
			rt.retry_valid = true;
			return;
		}

		LOG_DBG("report dropped (%d)", err);
		rt.retry_valid = false;
		return;
	}
}

/* ------------------------------------------------------------------------ */
/* Geometry                                                                 */
/* ------------------------------------------------------------------------ */

static void geometry_fallback(struct wp_pad_geometry *g)
{
	struct wp_touch_mount mount;
	float w = 0.0f;
	float h = 0.0f;

	wp_config_touch_mount(&mount);
	wp_config_touch_size_mm(&w, &h);
	if (!(w > 0.0f) || !(h > 0.0f)) {
		w = FALLBACK_W_MM;
		h = FALLBACK_H_MM;
	}

	if (mount.swap_xy) {
		g->max_x = FALLBACK_MAX_Y;
		g->max_y = FALLBACK_MAX_X;
		g->width_mm = h;
		g->height_mm = w;
	} else {
		g->max_x = FALLBACK_MAX_X;
		g->max_y = FALLBACK_MAX_Y;
		g->width_mm = w;
		g->height_mm = h;
	}
}

static void geometry_sanitize(struct wp_pad_geometry *g)
{
	if (g->max_x == 0U) {
		g->max_x = rt.pad_geom.max_x != 0U ? rt.pad_geom.max_x : FALLBACK_MAX_X;
	}
	if (g->max_y == 0U) {
		g->max_y = rt.pad_geom.max_y != 0U ? rt.pad_geom.max_y : FALLBACK_MAX_Y;
	}
	if (!(g->width_mm > 0.0f)) {
		g->width_mm = rt.fallback_w_mm;
	}
	if (!(g->height_mm > 0.0f)) {
		g->height_mm = rt.fallback_h_mm;
	}
}

static bool geometry_equal(const struct wp_pad_geometry *a, const struct wp_pad_geometry *b)
{
	return a->max_x == b->max_x && a->max_y == b->max_y && a->width_mm == b->width_mm &&
	       a->height_mm == b->height_mm;
}

static void geometry_apply(const struct wp_pad_geometry *g_in, int64_t t)
{
	struct wp_pad_geometry g = *g_in;
	struct wp_pad_geometry ug;
	struct wp_gesture_output go;

	geometry_sanitize(&g);
	if (geometry_equal(&g, &rt.pad_geom)) {
		return;
	}

	rt.pad_geom = g;

	if (wp_calib_inhibits_scroll(&rt.calib)) {
		wp_sm_event(&rt.sm, WP_SM_EV_CALIB_CANCEL, t);
	}
	wp_calib_init(&rt.calib, &rt.calib_cfg, &g);

	wp_xform_geometry(rt.orientation, &g, &ug);
	wp_gesture_set_geometry(&rt.gest, &ug, &go);
	apply_gesture(&go, t, t);
	wp_gesture_set_scroll_inhibit(&rt.gest, false);

	LOG_INF("pad geometry %ux%u counts, %d x %d um", g.max_x, g.max_y,
		(int)(g.width_mm * 1000.0f), (int)(g.height_mm * 1000.0f));
}

static void geometry_refresh(const struct wp_touch_frame *f, int64_t t)
{
	struct wp_pad_geometry g;
	bool have = wp_touch_source_get_geometry(&g) == 0 && g.max_x != 0U && g.max_y != 0U;

	if (f != NULL) {
		if (f->max_x == 0U || f->max_y == 0U) {
			return;
		}
		if (f->max_x == rt.pad_geom.max_x && f->max_y == rt.pad_geom.max_y) {
			return;
		}
		/* The frame is authoritative for the extent; keep the size. */
		if (!have || g.max_x != f->max_x || g.max_y != f->max_y) {
			g = rt.pad_geom;
			g.max_x = f->max_x;
			g.max_y = f->max_y;
		}
	} else if (!have) {
		return;
	}

	geometry_apply(&g, t);
}

/* ------------------------------------------------------------------------ */
/* Event handlers (router thread)                                           */
/* ------------------------------------------------------------------------ */

static void handle_calib(const struct wp_calib_result *cr, int64_t t)
{
	int deg;

	switch (cr->event) {
	case WP_CALIB_EV_ARMED:
		LOG_INF("orientation calibration armed");
		wp_sm_event(&rt.sm, WP_SM_EV_CALIB_ARMED, t);
		break;
	case WP_CALIB_EV_COMPLETED:
		deg = wp_orientation_deg(cr->orientation);
		LOG_INF("orientation calibration: swipe %d deg -> %d deg", (int)cr->angle_deg, deg);
		wp_orient_mgr_set_manual(&rt.orient, cr->orientation, t);
		wp_sm_event(&rt.sm, WP_SM_EV_CALIB_DONE, t);
		wp_led_set(WP_LED_CALIB_DONE, deg);
		break;
	case WP_CALIB_EV_CANCELLED:
		LOG_INF("orientation calibration cancelled");
		wp_sm_event(&rt.sm, WP_SM_EV_CALIB_CANCEL, t);
		break;
	default:
		break;
	}
}

static void handle_touch(const struct wp_touch_frame *pf)
{
	struct wp_touch_frame uf;
	struct wp_calib_result cr;
	struct wp_gesture_output go;
	int64_t t;

	if (pf->t_us <= rt.touch_lost_us) {
		/* Captured before the last controller reset / error / re-init
		 * (e.g. a replayed or late-queued frame): its contacts are gone. */
		return;
	}
	t = algo_time(pf->t_us);

	rt.touch_frames++;
	rt.last_frame_t = pf->t_us;

	if (!rt.touch_ok) {
		/* Frames prove the controller works even if the OK status was lost. */
		rt.touch_ok = true;
	}

	geometry_refresh(pf, t);

	wp_orient_mgr_on_touch(&rt.orient, pf->count, t);

	wp_calib_update(&rt.calib, pf, &cr);
	handle_calib(&cr, t);
	wp_gesture_set_scroll_inhibit(&rt.gest, wp_calib_inhibits_scroll(&rt.calib));

	wp_xform_frame(rt.orientation, pf, &uf);
	wp_gesture_process(&rt.gest, &uf, &go);
	apply_gesture(&go, t, pf->t_us);
}

static void touch_contact_lost(int64_t t)
{
	struct wp_gesture_output go;

	wp_gesture_reset(&rt.gest, &go);
	apply_gesture(&go, t, t);

	/* Never leave a button stuck: drop everything, release all buttons. */
	wp_hid_composer_release_all(&rt.comp);
	rt.held = 0U;

	if (wp_calib_inhibits_scroll(&rt.calib)) {
		wp_sm_event(&rt.sm, WP_SM_EV_CALIB_CANCEL, t);
	}
	wp_calib_reset(&rt.calib);
	wp_gesture_set_scroll_inhibit(&rt.gest, false);

	wp_orient_mgr_on_touch(&rt.orient, 0U, t);
	rt.contacts = 0U;
	rt.single_still = false;
	rt.single_still_ms = 0U;
}

static void handle_touch_status(int32_t status, int64_t t, int64_t post_us)
{
	enum wp_state st = wp_sm_state(&rt.sm);
	bool initializing = st == WP_STATE_BOOT || st == WP_STATE_INITIALIZING;

	/* Every status follows a controller (re)initialization or failure:
	 * frames captured before it are stale. */
	if (post_us > rt.touch_lost_us) {
		rt.touch_lost_us = post_us;
	}

	switch (status) {
	case WP_TOUCH_STATUS_OK:
		/* OK always follows a (re)initialization: no contact survives it
		 * (a RESET posted just before may have been merged away). */
		touch_contact_lost(t);
		rt.touch_ok = true;
		geometry_refresh(NULL, t);
		if (initializing) {
			wp_sm_event(&rt.sm, WP_SM_EV_INIT_OK, t);
		} else if (st == WP_STATE_ERROR_RECOVERY) {
			wp_sm_event(&rt.sm, WP_SM_EV_RECOVERED, t);
		}
		atomic_set_bit(&ctl_bits, CTL_TOUCH_READY);
		atomic_set_bit(&ctl_bits, CTL_REAPPLY_TOUCH);
		ctl_submit();
		LOG_INF("touch controller ready");
		break;
	case WP_TOUCH_STATUS_RESET:
	case WP_TOUCH_STATUS_ERROR:
		LOG_WRN("touch %s: contacts released", status == WP_TOUCH_STATUS_RESET ? "reset" : "error");
		touch_contact_lost(t);
		break;
	case WP_TOUCH_STATUS_FATAL:
		LOG_ERR("touch controller failed");
		touch_contact_lost(t);
		rt.touch_ok = false;
		atomic_clear_bit(&ctl_bits, CTL_TOUCH_READY);
		wp_sm_event(&rt.sm, initializing ? WP_SM_EV_INIT_FAIL : WP_SM_EV_ERROR, t);
		break;
	default:
		LOG_WRN("unknown touch status %d", (int)status);
		break;
	}
}

static void imu_invalidate(void)
{
	rt.imu_ok = false;
	atomic_clear_bit(&ctl_bits, CTL_IMU_READY);
	wp_motion_reset(&rt.motion);
	memset(&rt.features, 0, sizeof(rt.features));
}

static void handle_imu_status(int32_t status)
{
	switch (status) {
	case WP_IMU_STATUS_OK:
		if (!rt.imu_ok) {
			LOG_INF("IMU ready");
		}
		rt.imu_present = true;
		rt.imu_ok = true;
		atomic_set(&imu_retry_ms, IMU_RETRY_MIN_MS);
		(void)k_work_cancel_delayable(&imu_retry_work);
		atomic_set_bit(&ctl_bits, CTL_IMU_READY);
		atomic_set_bit(&ctl_bits, CTL_REAPPLY_IMU);
		ctl_submit();
		break;
	case WP_IMU_STATUS_ERROR:
		LOG_WRN("IMU error: motion features disabled, recovering");
		rt.imu_present = true;
		imu_invalidate();
		(void)k_work_schedule_for_queue(&ctl_wq, &imu_retry_work,
						K_MSEC(atomic_get(&imu_retry_ms)));
		break;
	case WP_IMU_STATUS_ABSENT:
		LOG_INF("no IMU: trackpad-only operation");
		rt.imu_present = false;
		imu_invalidate();
		break;
	default:
		LOG_WRN("unknown IMU status %d", (int)status);
		break;
	}
}

/* A dropped touch frame waiting for replay may be the finger lift. */
static bool touch_replay_pending(void)
{
	k_spinlock_key_t key = k_spin_lock(&drop_lock);
	bool pending = dropped_frame_valid;

	k_spin_unlock(&drop_lock, key);
	return pending;
}

static void handle_imu(const struct wp_imu_sample *s)
{
	float dx = 0.0f;
	float dy = 0.0f;

	rt.imu_samples++;
	rt.last_imu_rx_us = now_us();
	rt.last_imu = *s;
	rt.last_imu_valid = true;
	if (!rt.imu_ok) {
		/* Samples prove the IMU works even if its OK status was lost. */
		handle_imu_status(WP_IMU_STATUS_OK);
	}

	wp_motion_update(&rt.motion, s, &rt.features);
	wp_orient_mgr_on_imu(&rt.orient, s);

	/* Always fed: the gravity estimate must be ready when the clutch engages.
	 * The clutch is the finger: besides the state machine output, motion is
	 * only emitted while exactly one finger is known to be down and no
	 * (possibly lift) frame is waiting for replay after a queue overflow.
	 */
	wp_air_mouse_update(&rt.air, s, &dx, &dy);
	if (rt.hid_connected && rt.out_valid && rt.out.air_mouse_active &&
	    wp_air_mouse_active(&rt.air) &&
	    rt.contacts == 1U && !touch_replay_pending() && (dx != 0.0f || dy != 0.0f)) {
		wp_hid_composer_add_motion(&rt.comp, dx, dy);
		stamp_input(s->t_us);
	}
}

static void handle_hid_connected(bool connected, int64_t t)
{
	if (connected == rt.hid_connected) {
		return;
	}
	rt.hid_connected = connected;
	rt.retry_valid = false;
	rt.stamp_valid = false;

	if (connected) {
		LOG_INF("HID connected");
		/* Motion and clicks accumulated while disconnected are stale. A drag
		 * that is still in progress is re-asserted by the next gesture tick
		 * (wp_hid_composer_set_held() in housekeeping). The first report
		 * re-sends the full button state: a RELEASE dropped while the link
		 * flapped would otherwise leave the host with a stuck button.
		 */
		hid_resync();
		wp_sm_event(&rt.sm, WP_SM_EV_BLE_CONNECTED, t);
		atomic_set_bit(&ctl_bits, CTL_BLE_READY);
		atomic_set_bit(&ctl_bits, CTL_REAPPLY_BLE);
		ctl_submit();
	} else {
		LOG_INF("HID disconnected");
		atomic_clear_bit(&ctl_bits, CTL_BLE_READY);
		wp_hid_composer_release_all(&rt.comp);
		rt.held = 0U;
		wp_sm_event(&rt.sm, WP_SM_EV_BLE_DISCONNECTED, t);
	}
}

static void sm_step(int64_t t);

static void handle_mode_req(int32_t mode, int64_t t)
{
	if (mode < 0) {
		wp_sm_event(&rt.sm, WP_SM_EV_MODE_TOGGLE, t);
		return;
	}

	/* The state machine only cycles AUTO -> HANDHELD -> TRACKPAD -> AUTO. */
	for (int i = 0; i < 3 && rt.out_valid && (int32_t)rt.out.mode != mode; i++) {
		wp_sm_event(&rt.sm, WP_SM_EV_MODE_TOGGLE, t);
		sm_step(t);
	}
	if (!rt.out_valid || (int32_t)rt.out.mode != mode) {
		LOG_WRN("mode %d not reached", (int)mode);
	}
}

static void handle_orient_req(int32_t deg, int64_t t)
{
	if (deg < 0) {
		LOG_INF("orientation: automatic");
		wp_orient_mgr_set_auto(&rt.orient, true);
		return;
	}
	LOG_INF("orientation forced to %d deg", (int)deg);
	wp_orient_mgr_force(&rt.orient, wp_orientation_from_deg((float)deg), t);
}

static void handle_sleep_failed(int64_t t)
{
	/* SLEEP is terminal in the state machine; restart it so the device
	 * keeps working when System OFF is unavailable. */
	LOG_WRN("restarting state machine after failed sleep");
	rt.sleep_requested = false;
	wp_sm_init(&rt.sm, &rt.sm_cfg, t);
	wp_sm_event(&rt.sm, WP_SM_EV_INIT_START, t);
	wp_sm_event(&rt.sm, rt.touch_ok ? WP_SM_EV_INIT_OK : WP_SM_EV_INIT_FAIL, t);
	if (rt.hid_connected) {
		wp_sm_event(&rt.sm, WP_SM_EV_BLE_CONNECTED, t);
	}
	rt.out_valid = false; /* re-apply every output */
	/* The wake-up preparation changed the sensor power modes behind the
	 * control work's back: force it to re-apply them. */
	atomic_set_bit(&ctl_bits, CTL_REAPPLY_IMU);
	atomic_set_bit(&ctl_bits, CTL_REAPPLY_TOUCH);
}

static void handle_msg(const struct wp_msg *m)
{
	/* Data messages carry their own capture time; control messages use
	 * "now" (taken lazily so frame timestamps are not clamped to it). */
	switch (m->type) {
	case MSG_TOUCH:
		handle_touch(&m->u.touch);
		break;
	case MSG_TOUCH_STATUS:
		handle_touch_status(m->u.value, algo_time(now_us()), m->post_us);
		break;
	case MSG_IMU:
		handle_imu(&m->u.imu);
		break;
	case MSG_IMU_STATUS:
		handle_imu_status(m->u.value);
		break;
	case MSG_HID_CONNECTED:
		handle_hid_connected(m->u.value != 0, algo_time(now_us()));
		break;
	case MSG_HID_READY:
		atomic_clear_bit(&post_flags, POST_HID_READY_PENDING);
		break; /* the flush in housekeeping() does the work */
	case MSG_ORIENT_REQ:
		handle_orient_req(m->u.value, algo_time(now_us()));
		break;
	case MSG_MODE_REQ:
		handle_mode_req(m->u.value, algo_time(now_us()));
		break;
	case MSG_SLEEP_FAILED:
		handle_sleep_failed(algo_time(now_us()));
		break;
	case MSG_KEY:
	default:
		LOG_DBG("message %u ignored", m->type);
		break;
	}
}

static void latch_dispatch(enum latch_id id, const struct latch *l, int64_t t)
{
	switch (id) {
	case LATCH_TOUCH_STATUS:
		handle_touch_status(l->value, t, l->post_us);
		break;
	case LATCH_IMU_STATUS:
		handle_imu_status(l->value);
		break;
	case LATCH_HID_CONN:
		handle_hid_connected(l->value != 0, t);
		break;
	case LATCH_ORIENT_REQ:
		handle_orient_req(l->value, t);
		break;
	case LATCH_MODE_REQ:
		handle_mode_req(l->value, t);
		break;
	case LATCH_SLEEP_FAILED:
		handle_sleep_failed(t);
		break;
	default:
		break;
	}
}

/*
 * Apply latched values in post order: all of them (@p all), or only those
 * posted before @p before_seq, i.e. before the message about to be handled.
 * A latched value is thus never applied ahead of an older queued message.
 */
static void apply_latches(bool all, uint32_t before_seq, int64_t t)
{
	for (;;) {
		enum latch_id best = LATCH_NONE;
		struct latch l;
		k_spinlock_key_t key = k_spin_lock(&latch_lock);

		for (int i = 0; i < LATCH_COUNT; i++) {
			const struct latch *c = &latches[i];

			if (!c->set || (!all && !seq_before(c->seq, before_seq))) {
				continue;
			}
			if (best == LATCH_NONE || seq_before(c->seq, latches[best].seq)) {
				best = (enum latch_id)i;
			}
		}
		if (best != LATCH_NONE) {
			l = latches[best];
			latches[best].set = false;
		}
		k_spin_unlock(&latch_lock, key);

		if (best == LATCH_NONE) {
			return;
		}
		latch_dispatch(best, &l, t);
	}
}

static void process_latches(int64_t t)
{
	/* Queue drained: whatever is latched is newer than everything handled. */
	if (k_msgq_num_used_get(&router_q) == 0U) {
		apply_latches(true, 0U, t);
	}

	/* Replay the newest dropped touch frame once the queue has drained
	 * (keeps ordering; guarantees the final finger state is seen). */
	if (k_msgq_num_used_get(&router_q) == 0U) {
		struct wp_touch_frame f;
		bool have = false;
		k_spinlock_key_t key = k_spin_lock(&drop_lock);

		if (dropped_frame_valid) {
			f = dropped_frame;
			dropped_frame_valid = false;
			have = true;
		}
		k_spin_unlock(&drop_lock, key);

		if (have && f.t_us > rt.last_frame_t) {
			handle_touch(&f);
		}
	}
}

/* ------------------------------------------------------------------------ */
/* State machine                                                            */
/* ------------------------------------------------------------------------ */

static bool motion_valid(void)
{
	return rt.imu_ok && rt.features.valid && (now_us() - rt.last_imu_rx_us) < IMU_STALE_US;
}

static void ctl_request(const struct wp_sm_outputs *out)
{
	atomic_set_bit_to(&ctl_bits, CTL_WANT_GYRO, out->gyro_needed);
	atomic_set_bit_to(&ctl_bits, CTL_WANT_IMU_LOW_RATE, out->imu_low_rate);
	atomic_set_bit_to(&ctl_bits, CTL_WANT_TOUCH_LP, out->touch_low_power);
	atomic_set_bit_to(&ctl_bits, CTL_WANT_BLE_LL, out->ble_low_latency);
	ctl_submit();
}

static void prepare_sleep(void)
{
	/* Release everything the host may still hold, best effort. */
	wp_hid_composer_release_all(&rt.comp);
	rt.held = 0U;
	hid_flush();
	wp_led_set(WP_LED_OFF, 0);
	atomic_set_bit(&ctl_bits, CTL_REQ_SLEEP);
	ctl_submit();
}

/* Why the pad left IDLE. Power diagnostic: on a pad lying untouched on a
 * table, IDLE must not end by itself. */
static void log_idle_exit_reason(void)
{
	if (rt.contacts > 0U) {
		LOG_INF("left IDLE: touch (%u contacts)", rt.contacts);
	} else if (rt.features.valid && rt.features.moving) {
		LOG_INF("left IDLE: motion (accel dev %d mg, gyro %d dps)",
			(int)(rt.features.accel_dev_g * 1000.0f), (int)rt.features.gyro_mag_dps);
	} else {
		LOG_INF("left IDLE: other (mode, calibration or error)");
	}
}

static void apply_sm(const struct wp_sm_outputs *out, int64_t t)
{
	const struct wp_sm_outputs *prev = &rt.out;
	bool first = !rt.out_valid;

	if (first || out->state_changed) {
		LOG_INF("state %s -> %s (mode %s)", wp_state_name(out->prev_state),
			wp_state_name(out->state), wp_mode_name(out->mode));
	}
	if (!first && out->state_changed && out->prev_state == WP_STATE_IDLE) {
		log_idle_exit_reason();
	}

	if (first || out->gyro_needed != prev->gyro_needed ||
	    out->imu_low_rate != prev->imu_low_rate ||
	    out->touch_low_power != prev->touch_low_power ||
	    out->ble_low_latency != prev->ble_low_latency) {
		ctl_request(out);
	}

	/* Leaving the trackpad posture (pickup, handheld, air mouse) ends any
	 * drag: the button must never stay held while the gyro moves the
	 * cursor. */
	if (!first && ((prev->pointer_from_touch && !out->pointer_from_touch) ||
		       (prev->long_press_allowed && !out->long_press_allowed))) {
		struct wp_gesture_output go;

		wp_gesture_cancel_drag(&rt.gest, &go);
		apply_gesture(&go, t, t);
	}

	if (first || out->pointer_from_touch != prev->pointer_from_touch) {
		wp_gesture_set_pointer_enabled(&rt.gest, out->pointer_from_touch);
	}
	if (first || out->long_press_allowed != prev->long_press_allowed) {
		wp_gesture_set_long_press_allowed(&rt.gest, out->long_press_allowed);
	}
	if (first || out->orientation_auto_allowed != prev->orientation_auto_allowed) {
		wp_orient_mgr_allow_auto(&rt.orient, out->orientation_auto_allowed);
	}

	if (first || out->air_mouse_active != prev->air_mouse_active) {
		/* Steady aiming must never be learnt as gyro bias. */
		wp_imu_source_set_bias_learning(!out->air_mouse_active);
	}

	if (first ? out->air_mouse_active : out->air_mouse_active != prev->air_mouse_active) {
		if (out->air_mouse_active) {
			wp_air_mouse_start(&rt.air, t);
		} else {
			wp_air_mouse_stop(&rt.air);
		}
		/* No jump when switching between finger and gyro pointing. */
		wp_hid_composer_reset_motion(&rt.comp);
		LOG_INF("air mouse %s", out->air_mouse_active ? "on" : "off");
	} else if (!out->air_mouse_active && wp_air_mouse_active(&rt.air)) {
		wp_air_mouse_stop(&rt.air);
	}

	if (!first && out->mode != prev->mode) {
		LOG_INF("mode override -> %s", wp_mode_name(out->mode));
		wp_led_set(WP_LED_MODE_CHANGE, (int)out->mode);
	}

	if (out->request_reinit) {
		LOG_INF("re-initializing touch controller");
		atomic_set_bit(&ctl_bits, CTL_REQ_TOUCH_REINIT);
		ctl_submit();
	}

	rt.out = *out;
	rt.out_valid = true;

	if (out->request_sleep && !rt.sleep_requested) {
		LOG_INF("entering System OFF");
		rt.sleep_requested = true;
		prepare_sleep();
	}
}

static void sm_step(int64_t t)
{
	struct wp_sm_inputs in;
	struct wp_sm_outputs out;

	memset(&in, 0, sizeof(in));
	in.t_us = t;
	in.contacts = rt.contacts;
	in.single_still = rt.single_still;
	in.single_still_ms = rt.single_still_ms;
	in.motion_valid = motion_valid();
	if (in.motion_valid) {
		in.motion = rt.features;
	}

	memset(&out, 0, sizeof(out));
	wp_sm_update(&rt.sm, &in, &out);
	apply_sm(&out, t);
}

/* ------------------------------------------------------------------------ */
/* Orientation changes, LED, status                                         */
/* ------------------------------------------------------------------------ */

static void poll_orientation(int64_t t)
{
	enum wp_orientation o;
	struct wp_gesture_output go;
	struct wp_pad_geometry ug;
	struct wp_orient_status os;

	if (!wp_orient_mgr_poll_change(&rt.orient, &o)) {
		return;
	}

	/* Reset everything accumulated in the old frame (and release a drag). */
	wp_gesture_reset(&rt.gest, &go);
	apply_gesture(&go, t, t);

	rt.orientation = o;
	wp_xform_geometry(o, &rt.pad_geom, &ug);
	wp_gesture_set_geometry(&rt.gest, &ug, &go);
	apply_gesture(&go, t, t);

	wp_hid_composer_reset_motion(&rt.comp);
	wp_air_mouse_set_orientation(&rt.air, o);
	if (rt.out_valid && rt.out.air_mouse_active && !wp_air_mouse_active(&rt.air)) {
		wp_air_mouse_start(&rt.air, t);
	}

	wp_led_set(WP_LED_CALIB_DONE, wp_orientation_deg(o));

	wp_orient_mgr_get_status(&rt.orient, &os);
	LOG_INF("orientation -> %d deg (source %s, confidence %d%%)", wp_orientation_deg(o),
		wp_orient_source_name(os.source), pct(os.confidence));
}

static void update_led(void)
{
	enum wp_led_pattern p;

	if (!rt.out_valid) {
		p = WP_LED_ADVERTISING;
	} else if (rt.out.state == WP_STATE_SLEEP) {
		p = WP_LED_OFF;
	} else if (rt.out.state == WP_STATE_ERROR_RECOVERY) {
		p = WP_LED_ERROR;
	} else if (wp_calib_inhibits_scroll(&rt.calib)) {
		p = WP_LED_CALIB_ARMED;
	} else if (rt.out.state == WP_STATE_AIR_MOUSE) {
		p = WP_LED_AIR_MOUSE;
	} else {
		p = rt.hid_connected ? WP_LED_CONNECTED : WP_LED_ADVERTISING;
	}

	if ((int)p != rt.led_base) {
		rt.led_base = (int)p;
		wp_led_set(p, 0);
	}
}

static void publish_status(void)
{
	struct wp_orient_status os;
	struct wp_router_status s;

	wp_orient_mgr_get_status(&rt.orient, &os);

	memset(&s, 0, sizeof(s));
	s.state = rt.out_valid ? (int)rt.out.state : (int)wp_sm_state(&rt.sm);
	s.mode = rt.out_valid ? (int)rt.out.mode : (int)WP_MODE_AUTO;
	s.orientation_deg = wp_orientation_deg(rt.orientation);
	s.orientation_source = (int)os.source;
	s.orientation_confidence = os.confidence;
	s.tilt_deg = os.last_gravity.valid ? os.last_gravity.tilt_deg : rt.features.tilt_deg;
	s.heading_deg = os.last_gravity.heading_deg;
	s.contacts = rt.contacts;
	s.hid_connected = rt.hid_connected;
	s.imu_present = rt.imu_present;
	s.touch_frames = rt.touch_frames;
	s.imu_samples = rt.imu_samples;
	s.reports_sent = rt.reports_sent;
	s.max_latency_us = rt.max_latency_us;
	s.gravity_valid = os.last_gravity.valid;
	s.imu_sample_valid = rt.last_imu_valid;
	if (rt.last_imu_valid) {
		s.imu_t_us = rt.last_imu.t_us;
		s.gyro_valid = rt.last_imu.gyro_valid;
		for (int i = 0; i < 3; i++) {
			s.accel[i] = rt.last_imu.accel[i];
			s.gyro_dps[i] = rt.last_imu.gyro_valid ? rt.last_imu.gyro[i] * WP_RAD_TO_DEG
							      : 0.0f;
		}
	}

	k_spinlock_key_t key = k_spin_lock(&status_lock);

	status_snap = s;
	k_spin_unlock(&status_lock, key);
}

/* Runs after every message and on every tick. */
static void housekeeping(void)
{
	struct wp_gesture_output go;
	struct wp_calib_result cr;
	int64_t t = algo_time(now_us());

	process_latches(t);
	t = algo_time(now_us());

	wp_gesture_tick(&rt.gest, t, &go);
	apply_gesture(&go, t, t);

	wp_calib_tick(&rt.calib, t, &cr);
	handle_calib(&cr, t);
	wp_gesture_set_scroll_inhibit(&rt.gest, wp_calib_inhibits_scroll(&rt.calib));

	wp_orient_mgr_tick(&rt.orient, t);
	poll_orientation(t);

	sm_step(t);

	hid_flush();
	update_led();
	publish_status();
}

static void router_thread_fn(void *p1, void *p2, void *p3)
{
	struct wp_msg msg;
	int64_t next_tick = k_uptime_get() + CONFIG_WAFER_PAD_ROUTER_TICK_MS;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("router running (queue %d, tick %d ms)", CONFIG_WAFER_PAD_ROUTER_QUEUE_DEPTH,
		CONFIG_WAFER_PAD_ROUTER_TICK_MS);

	for (;;) {
		int64_t wait = next_tick - k_uptime_get();
		k_timeout_t timeout = wait > 0 ? K_MSEC(wait) : K_NO_WAIT;

		if (k_msgq_get(&router_q, &msg, timeout) == 0) {
			/* Latched values posted before this message come first. */
			apply_latches(false, msg.seq, algo_time(now_us()));
			handle_msg(&msg);
		}

		housekeeping();

		int64_t now = k_uptime_get();

		if (now >= next_tick) {
			next_tick += CONFIG_WAFER_PAD_ROUTER_TICK_MS;
			if (next_tick <= now) {
				next_tick = now + CONFIG_WAFER_PAD_ROUTER_TICK_MS;
			}
		}
	}
}

/* ------------------------------------------------------------------------ */
/* Init                                                                     */
/* ------------------------------------------------------------------------ */

int wp_router_init(void)
{
	struct wp_motion_config mcfg;
	struct wp_orient_mgr_config ocfg;
	struct wp_gesture_config gcfg;
	struct wp_air_mouse_config acfg;
	struct wp_pad_geometry g;
	struct wp_pad_geometry ug;
	int64_t t = now_us();

	if (router_started) {
		return -EALREADY;
	}

	memset(&rt, 0, sizeof(rt));
	rt.algo_t = t;
	rt.led_base = -1;

	{
		const struct k_work_queue_config wq_cfg = {.name = "wp_ctl"};

		k_work_queue_init(&ctl_wq);
		k_work_queue_start(&ctl_wq, ctl_wq_stack, K_THREAD_STACK_SIZEOF(ctl_wq_stack),
				   CTL_WQ_PRIO, &wq_cfg);
	}
	k_work_init(&ctl_work, ctl_work_handler);
	k_work_init_delayable(&ctl_retry_work, ctl_retry_handler);
	k_work_init_delayable(&imu_retry_work, imu_retry_handler);

	/* Geometry: real one if the touch source already knows it, else the
	 * configured size with the nominal resolution (fixed on first frame). */
	geometry_fallback(&g);
	rt.fallback_w_mm = g.width_mm;
	rt.fallback_h_mm = g.height_mm;
	{
		struct wp_pad_geometry src;

		if (wp_touch_source_get_geometry(&src) == 0 && src.max_x != 0U &&
		    src.max_y != 0U) {
			g.max_x = src.max_x;
			g.max_y = src.max_y;
			if (src.width_mm > 0.0f && src.height_mm > 0.0f) {
				g.width_mm = src.width_mm;
				g.height_mm = src.height_mm;
			}
		}
	}
	rt.pad_geom = g;

	wp_config_motion(&mcfg);
	wp_motion_init(&rt.motion, &mcfg);

	rt.orientation = wp_config_orient_default();
	wp_config_orient_mgr(&ocfg);
	wp_orient_mgr_init(&rt.orient, &ocfg, rt.orientation);

	wp_config_calib(&rt.calib_cfg);
	wp_calib_init(&rt.calib, &rt.calib_cfg, &rt.pad_geom);

	wp_config_gestures(&gcfg);
	wp_xform_geometry(rt.orientation, &rt.pad_geom, &ug);
	wp_gesture_init(&rt.gest, &gcfg, &ug);

	wp_config_air_mouse(&acfg);
	wp_air_mouse_init(&rt.air, &acfg);
	wp_air_mouse_set_orientation(&rt.air, rt.orientation);

	wp_hid_composer_init(&rt.comp);

	wp_config_sm(&rt.sm_cfg);
	wp_sm_init(&rt.sm, &rt.sm_cfg, t);
	wp_sm_event(&rt.sm, WP_SM_EV_INIT_START, t);
	sm_step(t); /* BOOT -> INITIALIZING, applies the initial outputs */
	publish_status();

	LOG_INF("orientation %d deg at boot, pad %ux%u", wp_orientation_deg(rt.orientation),
		rt.pad_geom.max_x, rt.pad_geom.max_y);

	router_started = true;
	k_thread_create(&router_thread, router_stack, K_THREAD_STACK_SIZEOF(router_stack),
			router_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_WAFER_PAD_ROUTER_PRIORITY), K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&router_thread, "wp_router");

	return 0;
}
