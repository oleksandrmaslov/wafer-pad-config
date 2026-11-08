/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU source: samples the LSM6DSOX (DT alias motion0, Zephyr "st,lsm6dso"
 * sensor driver) in a dedicated thread, rotates samples into the body frame,
 * removes the gyroscope bias and posts them to the event router.
 *
 *   - Pacing with absolute deadlines (no drift), timestamp right after the
 *     fetch on the same clock as the touch frames.
 *   - Gyro power gating via SENSOR_ATTR_SAMPLING_FREQUENCY: the driver maps
 *     0 Hz to ODR index 0 (power-down) and 104 Hz to ODR_104Hz
 *     (lsm6dso_freq_to_odr_val()). Units from the driver: accel m/s^2,
 *     gyro rad/s.
 *   - Requests from other threads (gyro on/off, low rate, reinit) are applied
 *     by the IMU thread, so callers never block on I2C.
 *   - Gyro bias persisted under the settings key "wp/imu/bias".
 *   - Plausibility watchdog (wp_imu_plaus_*): readings that are physically
 *     impossible for > CONFIG_WAFER_PAD_IMU_DEAD_MS (|accel| < 0.2 g, or the
 *     enabled and settled gyro exactly 0,0,0) mean a silent reset left the
 *     sensor in power-down: WP_IMU_STATUS_ERROR is posted and the sensor is
 *     re-initialized, as after read errors.
 *
 * wp_imu_source_prepare_wakeup() suspends sampling. If System OFF is aborted
 * afterwards, call wp_imu_source_reinit() to resume. The INT1 pin sense is
 * enabled only by wp_imu_source_arm_wake_pin(), with interrupts locked right
 * before sys_poweroff(): an enabled level interrupt without a consumer would
 * otherwise re-fire forever if INT1 rose before power-off.
 *
 * A sensor that failed its driver init at boot (e.g. SDA still held after a
 * warm reset in the middle of a transfer) is retried with device_init()
 * whenever wp_imu_source_reinit() is called; only a missing devicetree node
 * reports WP_IMU_STATUS_ABSENT.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <wafer_pad/config.h>
#include <wafer_pad/imu_algo.h>
#include <wafer_pad/orientation_transform.h>
#include <wafer_pad/platform.h>

LOG_MODULE_REGISTER(wp_imu, CONFIG_WAFER_PAD_LOG_LEVEL);

#define WP_IMU_NODE DT_ALIAS(motion0)

#if defined(CONFIG_WAFER_PAD_IMU) && DT_NODE_HAS_STATUS_OKAY(WP_IMU_NODE)

#if defined(CONFIG_WAFER_PAD_IMU_BIAS_PERSIST)
#include <zephyr/settings/settings.h>
#endif

#if defined(CONFIG_WAFER_PAD_IMU_WAKEUP) && DT_NODE_HAS_COMPAT(WP_IMU_NODE, st_lsm6dso) &&       \
	DT_ON_BUS(WP_IMU_NODE, i2c) && DT_NODE_HAS_PROP(WP_IMU_NODE, irq_gpios)
#define WP_IMU_HAS_WAKEUP 1
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#else
#define WP_IMU_HAS_WAKEUP 0
#endif

#define WP_IMU_BIAS_KEY        "wp/imu/bias"
#define WP_IMU_ERR_POLL_US     (100 * 1000)  /* polling period while failing */
#define WP_IMU_REINIT_RETRY_MS 1000
#define WP_IMU_SUSPEND_WAIT_MS 200

static const struct device *const imu_dev = DEVICE_DT_GET(WP_IMU_NODE);

K_THREAD_STACK_DEFINE(imu_stack, CONFIG_WAFER_PAD_IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;
static K_SEM_DEFINE(imu_kick, 0, 1);
static K_SEM_DEFINE(imu_suspend_ack, 0, 1);

/* ---- Cross-thread state (atomics) --------------------------------------- */

static atomic_t imu_started;     /* thread created */
static atomic_t imu_present;
static atomic_t req_gyro_on = ATOMIC_INIT(1);
static atomic_t req_low_rate;
static atomic_t req_suspend;
static atomic_t req_reinit;
static atomic_t bias_learning = ATOMIC_INIT(1);

/* ---- Owned by the IMU thread (initialized before it starts) ------------- */

static struct {
	bool gyro_on;              /* applied gyro state */
	bool low_rate;             /* applied rate state */
	int64_t gyro_valid_from_us;
	int64_t accel_valid_from_us; /* samples before this are discarded */
	bool restart_pending;      /* flag the next posted sample "restart" */
	uint32_t period_us;
	struct wp_imu_mount mount;
	struct wp_gyro_bias bias;
	uint32_t err_count;
	bool in_error;
	int64_t last_reinit_ms;
	struct wp_imu_plaus plaus; /* silent-reset watchdog (HW-8) */
	uint32_t plaus_trips;      /* consecutive watchdog re-inits */
} imu;

/* ---- Bias snapshot for wp_imu_source_get_bias() ------------------------- */

static struct k_spinlock bias_lock;
static float bias_snap[3];
static bool bias_snap_valid;

static inline int64_t now_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

static int set_odr(enum sensor_channel chan, uint16_t hz)
{
	const struct sensor_value v = {.val1 = hz, .val2 = 0};

	return sensor_attr_set(imu_dev, chan, SENSOR_ATTR_SAMPLING_FREQUENCY, &v);
}

static void publish_bias(void)
{
	k_spinlock_key_t key = k_spin_lock(&bias_lock);

	memcpy(bias_snap, imu.bias.bias, sizeof(bias_snap));
	bias_snap_valid = imu.bias.valid;
	k_spin_unlock(&bias_lock, key);
}

/* ---- Bias persistence ---------------------------------------------------- */

#if defined(CONFIG_WAFER_PAD_IMU_BIAS_PERSIST)

static bool persist_ok;
static bool stored_valid;      /* a value exists in storage */
static float stored_bias[3];   /* last stored / loaded value (rad/s) */
static bool saved_this_boot;
static int64_t last_save_ms;
static float bias_to_save[3];  /* guarded by bias_lock */
static struct k_work bias_save_work;

struct bias_load_ctx {
	float v[3];
	bool found;
};

static int bias_load_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
			void *param)
{
	struct bias_load_ctx *ctx = param;

	/* Exact key match only (key == NULL or "" after the subtree prefix). */
	if (key != NULL && key[0] != '\0') {
		return 0;
	}
	if (len != sizeof(ctx->v)) {
		LOG_WRN("stored bias has wrong size %u", (unsigned int)len);
		return 0;
	}
	if (read_cb(cb_arg, ctx->v, sizeof(ctx->v)) == (ssize_t)sizeof(ctx->v)) {
		ctx->found = true;
	}
	return 0;
}

static void bias_save_work_fn(struct k_work *work)
{
	float v[3];
	k_spinlock_key_t key = k_spin_lock(&bias_lock);

	ARG_UNUSED(work);
	memcpy(v, bias_to_save, sizeof(v));
	k_spin_unlock(&bias_lock, key);

	int err = settings_save_one(WP_IMU_BIAS_KEY, v, sizeof(v));

	if (err != 0) {
		LOG_WRN("bias save failed (%d)", err);
	} else {
		LOG_DBG("bias saved");
	}
}

static void bias_persist_init(void)
{
	struct bias_load_ctx ctx = {.found = false};
	int err;

	k_work_init(&bias_save_work, bias_save_work_fn);

	err = settings_subsys_init();
	if (err != 0) {
		LOG_WRN("settings unavailable (%d), bias not persisted", err);
		return;
	}
	persist_ok = true;

	(void)settings_load_subtree_direct(WP_IMU_BIAS_KEY, bias_load_cb, &ctx);
	if (!ctx.found) {
		return;
	}
	wp_gyro_bias_set(&imu.bias, ctx.v);
	if (wp_gyro_bias_valid(&imu.bias)) {
		memcpy(stored_bias, ctx.v, sizeof(stored_bias));
		stored_valid = true;
		LOG_INF("restored gyro bias (mdps) %d %d %d",
			(int)(ctx.v[0] * WP_RAD_TO_DEG * 1000.0f),
			(int)(ctx.v[1] * WP_RAD_TO_DEG * 1000.0f),
			(int)(ctx.v[2] * WP_RAD_TO_DEG * 1000.0f));
	} else {
		LOG_WRN("stored gyro bias rejected (implausible)");
	}
}

/* IMU thread: save at most once per interval, only on a significant change. */
static void bias_maybe_save(void)
{
	const float min_change = CONFIG_WAFER_PAD_IMU_BIAS_SAVE_MIN_CHANGE_MILLI_DPS / 1000.0f;
	int64_t now_ms;
	float maxd = 0.0f;

	if (!persist_ok || !imu.bias.valid || !imu.bias.still) {
		return;
	}
	now_ms = k_uptime_get();
	if (saved_this_boot &&
	    (now_ms - last_save_ms) < (int64_t)CONFIG_WAFER_PAD_IMU_BIAS_SAVE_INTERVAL_S * 1000) {
		return;
	}
	if (stored_valid) {
		for (int i = 0; i < 3; i++) {
			float d = (imu.bias.bias[i] - stored_bias[i]) * WP_RAD_TO_DEG;

			maxd = MAX(maxd, (d < 0.0f) ? -d : d);
		}
		if (maxd <= min_change) {
			return;
		}
	}

	k_spinlock_key_t key = k_spin_lock(&bias_lock);

	memcpy(bias_to_save, imu.bias.bias, sizeof(bias_to_save));
	k_spin_unlock(&bias_lock, key);

	/* Flash write happens on the application control work queue, not in
	 * this thread and not on the system work queue (BT TX completions). */
	(void)k_work_submit_to_queue(wp_router_ctl_queue(), &bias_save_work);
	memcpy(stored_bias, imu.bias.bias, sizeof(stored_bias));
	stored_valid = true;
	saved_this_boot = true;
	last_save_ms = now_ms;
}

#else /* !CONFIG_WAFER_PAD_IMU_BIAS_PERSIST */

static inline void bias_persist_init(void)
{
}

static inline void bias_maybe_save(void)
{
}

#endif /* CONFIG_WAFER_PAD_IMU_BIAS_PERSIST */

/* ---- Rate / power control (IMU thread, or init before the thread runs) --- */

static int apply_rates(bool force)
{
	const bool want_low = atomic_get(&req_low_rate) != 0;
	const bool want_gyro = (atomic_get(&req_gyro_on) != 0) && !want_low;
	int err = 0;
	int ret;

	if (force || want_low != imu.low_rate) {
		const uint16_t hz = want_low ? CONFIG_WAFER_PAD_IMU_LOW_RATE_HZ
					     : CONFIG_WAFER_PAD_IMU_ODR_HZ;

		ret = set_odr(SENSOR_CHAN_ACCEL_XYZ, hz);
		if (ret == 0) {
			imu.low_rate = want_low;
			imu.period_us = 1000000U / hz;
			/* Output right after an ODR change is a transition, not
			 * motion: drop it, then restart the motion filters. */
			imu.accel_valid_from_us =
				now_us() + (int64_t)CONFIG_WAFER_PAD_IMU_ACCEL_SETTLE_MS * 1000;
			imu.restart_pending = true;
			LOG_DBG("accel %u Hz", hz);
		} else {
			err = ret;
		}
	}

	if (force || want_gyro != imu.gyro_on) {
		ret = set_odr(SENSOR_CHAN_GYRO_XYZ, want_gyro ? CONFIG_WAFER_PAD_IMU_ODR_HZ : 0);
		if (ret == 0) {
			if (want_gyro) {
				/* Turn-on time: flag samples invalid for the settle period. */
				imu.gyro_valid_from_us =
					now_us() + (int64_t)CONFIG_WAFER_PAD_IMU_GYRO_SETTLE_MS * 1000;
				wp_gyro_bias_restart(&imu.bias);
			}
			imu.gyro_on = want_gyro;
			LOG_DBG("gyro %s", want_gyro ? "on" : "off");
		} else {
			err = ret;
		}
	}

	if (err != 0) {
		LOG_DBG("rate change failed (%d)", err);
	}
	return err;
}

static int restore_full_scale(void)
{
#if DT_NODE_HAS_COMPAT(WP_IMU_NODE, st_lsm6dso)
	/* DT enum index -> range, same tables as the st,lsm6dso driver
	 * (lsm6dso_accel_fs_map / lsm6dso_gyro_fs_map, LSM6DSO_DT_FS_*).
	 */
	static const uint16_t accel_g[] = {2, 16, 4, 8};
	static const uint16_t gyro_dps[] = {250, 125, 500, 0, 1000, 0, 2000};
	const int ai = DT_PROP(WP_IMU_NODE, accel_range);
	const int gi = DT_PROP(WP_IMU_NODE, gyro_range);
	struct sensor_value v;
	int err = 0;

	if (ai >= 0 && ai < (int)ARRAY_SIZE(accel_g)) {
		sensor_g_to_ms2(accel_g[ai], &v);
		err = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	}
	if (err == 0 && gi >= 0 && gi < (int)ARRAY_SIZE(gyro_dps) && gyro_dps[gi] != 0) {
		sensor_degrees_to_rad(gyro_dps[gi], &v);
		err = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &v);
	}
	return err;
#else
	return 0;
#endif
}

#if WP_IMU_HAS_WAKEUP
static atomic_t wake_armed;
static void disarm_wakeup(void);
#endif

/* Re-apply full scale and data rates (also recovers from a sensor reset). */
static int do_reinit(void)
{
	int err;

	if (!device_is_ready(imu_dev)) {
		return -ENODEV;
	}
#if WP_IMU_HAS_WAKEUP
	if (atomic_cas(&wake_armed, 1, 0)) {
		disarm_wakeup();
	}
#endif
	err = restore_full_scale();
	if (err == 0) {
		err = apply_rates(true);
	}
	wp_gyro_bias_restart(&imu.bias);
	wp_imu_plaus_reset(&imu.plaus);
	if (err == 0) {
		LOG_INF("reinit ok");
	} else {
		LOG_DBG("reinit failed (%d)", err); /* retried every second while failing */
	}
	return err;
}

/* ---- Sampling ------------------------------------------------------------ */

static void on_read_error(int err)
{
	if (imu.err_count < UINT32_MAX) {
		imu.err_count++;
	}
	if (imu.err_count == 1U) {
		LOG_WRN("read failed (%d)", err);
	}
	if (imu.err_count == CONFIG_WAFER_PAD_IMU_READ_ERROR_LIMIT) {
		LOG_ERR("%u consecutive read errors", imu.err_count);
		imu.in_error = true;
		(void)wp_router_post_imu_status(WP_IMU_STATUS_ERROR);
	}
	if (imu.in_error) {
		const int64_t now_ms = k_uptime_get();

		if (now_ms - imu.last_reinit_ms >= WP_IMU_REINIT_RETRY_MS) {
			imu.last_reinit_ms = now_ms;
			(void)do_reinit();
		}
	}
}

static void on_read_ok(void)
{
	if (imu.in_error) {
		LOG_INF("recovered");
		imu.in_error = false;
		(void)wp_router_post_imu_status(WP_IMU_STATUS_OK);
	}
	imu.err_count = 0U;
}

/*
 * Plausibility watchdog: a silent reset (rail glitch) puts the sensor in
 * power-down; reads still succeed but return zeros, so on_read_error() never
 * fires. Readings implausible for > CONFIG_WAFER_PAD_IMU_DEAD_MS (|accel| <
 * 0.2 g, or the settled gyro exactly 0,0,0) run the error/re-init path.
 * Returns true if the sample must be dropped.
 */
static bool plausibility_check(int64_t t_us, const float accel[3], const float *gyro_raw)
{
	const uint32_t dead = wp_imu_plaus_update(&imu.plaus, t_us, accel, gyro_raw);

	if (dead == 0U) {
		if (!imu.plaus.accel_bad && !imu.plaus.gyro_bad) {
			imu.plaus_trips = 0U;
		}
		return false;
	}

	if (imu.plaus_trips < UINT32_MAX) {
		imu.plaus_trips++;
	}
	if (imu.plaus_trips == 1U) {
		LOG_WRN("%s for > %u ms (silent sensor reset?): re-initializing",
			(dead & WP_IMU_PLAUS_ACCEL_DEAD) ? "|accel| < min (zeros)"
							 : "gyro stuck at 0,0,0",
			(unsigned int)CONFIG_WAFER_PAD_IMU_DEAD_MS);
	} else {
		LOG_DBG("still implausible (%u), re-initializing again", imu.plaus_trips);
	}
	(void)wp_router_post_imu_status(WP_IMU_STATUS_ERROR);
	imu.last_reinit_ms = k_uptime_get();
	(void)do_reinit(); /* also restarts the watchdog timers */
	return true;
}

static void sample_once(void)
{
	struct sensor_value acc[3];
	struct sensor_value gyr[3];
	struct wp_imu_sample s = {0};
	float raw[3] = {0};
	float v[3];
	int64_t t_us;
	int err;

	err = sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_ACCEL_XYZ);
	if (err == 0 && imu.gyro_on) {
		err = sensor_sample_fetch_chan(imu_dev, SENSOR_CHAN_GYRO_XYZ);
	}
	t_us = now_us();
	if (err == 0) {
		err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
	}
	if (err == 0 && imu.gyro_on) {
		err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
	}
	if (err != 0) {
		on_read_error(err);
		return;
	}
	on_read_ok();

	if (t_us < imu.accel_valid_from_us) {
		return; /* settling after a data-rate change */
	}

	s.t_us = t_us;
	s.restart = imu.restart_pending;
	imu.restart_pending = false;
	for (int i = 0; i < 3; i++) {
		v[i] = sensor_value_to_float(&acc[i]);
	}
	wp_imu_mount_apply(&imu.mount, v, s.accel);

	s.gyro_valid = imu.gyro_on && (t_us >= imu.gyro_valid_from_us);
	if (s.gyro_valid) {
		for (int i = 0; i < 3; i++) {
			v[i] = sensor_value_to_float(&gyr[i]);
		}
		wp_imu_mount_apply(&imu.mount, v, raw);
	}
	if (plausibility_check(t_us, s.accel, s.gyro_valid ? raw : NULL)) {
		return;
	}
	if (s.gyro_valid) {
		if (atomic_get(&bias_learning) != 0) {
			(void)wp_gyro_bias_update(&imu.bias, t_us, raw, s.accel);
		} else {
			/* Air mouse: aiming must never be learnt as bias. */
			wp_gyro_bias_restart(&imu.bias);
		}
		wp_gyro_bias_apply(&imu.bias, raw, s.gyro);
		publish_bias();
		bias_maybe_save();
	}

	(void)wp_router_post_imu(&s);
}

static void imu_thread_fn(void *p1, void *p2, void *p3)
{
	int64_t next_us;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	next_us = now_us();
	for (;;) {
		if (atomic_get(&req_suspend) != 0) {
			k_sem_give(&imu_suspend_ack);
			while (atomic_get(&req_suspend) != 0) {
				(void)k_sem_take(&imu_kick, K_FOREVER);
			}
			atomic_set(&req_reinit, 1);
			next_us = now_us();
		}
		if (atomic_cas(&req_reinit, 1, 0)) {
			/* While failing, keep counting: ERROR is reported once per
			 * fault, not again after every external re-init request. */
			if (!imu.in_error) {
				imu.err_count = 0U;
			}
			(void)do_reinit();
		}

		(void)apply_rates(false);
		sample_once();

		const uint32_t period = imu.in_error ? MAX(imu.period_us, WP_IMU_ERR_POLL_US)
						     : imu.period_us;
		int64_t now;

		next_us += period;
		now = now_us();
		if (now - next_us > (int64_t)period) {
			/* Fell behind by more than a period: skip, do not burst. */
			next_us = now;
		}
#if defined(CONFIG_TIMEOUT_64BIT)
		if (k_sem_take(&imu_kick, K_TIMEOUT_ABS_US(next_us)) == 0) {
#else
		if (k_sem_take(&imu_kick, K_USEC(MAX(next_us - now, 0))) == 0) {
#endif
			/* Request pending: apply it now and restart the schedule. */
			next_us = now_us();
		}
	}
}

/* ---- Wake-up interrupt (System OFF) ---------------------------------------- */

#if WP_IMU_HAS_WAKEUP
/*
 * LSM6DSOX registers and bit fields, taken from hal_st
 * sensor/stmemsc/lsm6dsox_STdC/driver/lsm6dsox_reg.h (bit 0 = first
 * bit-field member in the little-endian layout):
 */
#define WP_LSM6DSOX_WAKE_UP_SRC   0x1BU   /* LSM6DSOX_WAKE_UP_SRC (read clears latch) */
#define WP_LSM6DSOX_CTRL1_XL      0x10U   /* odr_xl[7:4] fs_xl[3:2] */
#define WP_LSM6DSOX_CTRL2_G       0x11U   /* odr_g[7:4] */
#define WP_LSM6DSOX_CTRL3_C       0x12U   /* pp_od[4] h_lactive[5] */
#define WP_LSM6DSOX_CTRL6_C       0x15U   /* xl_hm_mode[4] */
#define WP_LSM6DSOX_TAP_CFG0      0x56U   /* lir[0] slope_fds[4] */
#define WP_LSM6DSOX_TAP_CFG2      0x58U   /* inact_en[6:5] interrupts_enable[7] */
#define WP_LSM6DSOX_WAKE_UP_THS   0x5BU   /* wk_ths[5:0] usr_off_on_wu[6] */
#define WP_LSM6DSOX_WAKE_UP_DUR   0x5CU   /* sleep_dur[3:0] wake_ths_w[4] wake_dur[6:5] */
#define WP_LSM6DSOX_MD1_CFG       0x5EU   /* int1_wu[5] */

#define WP_ODR_MASK               0xF0U   /* odr_xl / odr_g */
#define WP_ODR_SHIFT              4
#define WP_XL_ODR_26HZ            2U      /* LSM6DSOX_XL_ODR_26Hz */
#define WP_GY_ODR_OFF             0U      /* LSM6DSOX_GY_ODR_OFF */
#define WP_FS_XL_MASK             0x0CU
#define WP_FS_XL_SHIFT            2       /* LSM6DSOX_2g=0, _16g=1, _4g=2, _8g=3 */
#define WP_CTRL3_PP_OD            BIT(4)
#define WP_CTRL3_H_LACTIVE        BIT(5)
#define WP_CTRL6_XL_HM_MODE       BIT(4)  /* 1 = high-performance disabled (low power) */
#define WP_TAP_CFG0_LIR           BIT(0)
#define WP_TAP_CFG0_SLOPE_FDS     BIT(4)  /* 0 = slope filter */
#define WP_TAP_CFG2_INT_EN        BIT(7)
#define WP_TAP_CFG2_INACT_MASK    (BIT(6) | BIT(5))
#define WP_WU_THS_MASK            0x3FU
#define WP_WU_THS_USR_OFF         BIT(6)
#define WP_WU_DUR_THS_W           BIT(4)  /* 1 = LSM6DSOX_LSb_FS_DIV_256 */
#define WP_WU_DUR_WAKE_DUR_MASK   (BIT(6) | BIT(5))
#define WP_MD1_INT1_WU            BIT(5)

static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(WP_IMU_NODE);
static const struct gpio_dt_spec imu_int1 = GPIO_DT_SPEC_GET(WP_IMU_NODE, irq_gpios);
static struct gpio_callback imu_int1_cb;
static bool imu_int1_cb_added;

static void imu_int1_isr(const struct device *port, struct gpio_callback *cb,
			 gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);

	if ((pins & BIT(imu_int1.pin)) != 0U) {
		/* Level trigger: mask it, otherwise the GPIOTE ISR spins. */
		(void)gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_DISABLE);
	}
}

static int arm_wakeup(void)
{
	static const uint16_t fs_mg[] = {2000, 16000, 4000, 8000};
	uint8_t ctrl1;
	uint8_t src;
	uint32_t ths;
	int err;

	if (DT_PROP_OR(WP_IMU_NODE, int_pin, 1) != 1) {
		return -ENOTSUP; /* only INT1 is supported */
	}
	if (!i2c_is_ready_dt(&imu_i2c) || !gpio_is_ready_dt(&imu_int1)) {
		return -ENODEV;
	}

	/* Gyro off, accel low-power 26 Hz (keep the configured full scale). */
	err = i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL2_G, WP_ODR_MASK,
				     WP_GY_ODR_OFF << WP_ODR_SHIFT);
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL6_C,
						 WP_CTRL6_XL_HM_MODE, WP_CTRL6_XL_HM_MODE);
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL1_XL, WP_ODR_MASK,
						 WP_XL_ODR_26HZ << WP_ODR_SHIFT);
	err = err ? err : i2c_reg_read_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL1_XL, &ctrl1);
	if (err != 0) {
		return err;
	}

	/* Threshold LSB = FS / 2^8 with wake_ths_w = 1. */
	const uint32_t lsb_ug = (uint32_t)fs_mg[(ctrl1 & WP_FS_XL_MASK) >> WP_FS_XL_SHIFT] *
				1000U / 256U;

	ths = ((uint32_t)CONFIG_WAFER_PAD_IMU_WAKEUP_THS_MILLI_G * 1000U + lsb_ug / 2U) / lsb_ug;
	ths = CLAMP(ths, 1U, WP_WU_THS_MASK);

	/* INT1 push-pull, polarity matching the devicetree flag. */
	err = i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL3_C,
				     WP_CTRL3_PP_OD | WP_CTRL3_H_LACTIVE,
				     (imu_int1.dt_flags & GPIO_ACTIVE_LOW) ? WP_CTRL3_H_LACTIVE : 0);
	/* Latched interrupt, slope filter. */
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_TAP_CFG0,
						 WP_TAP_CFG0_LIR | WP_TAP_CFG0_SLOPE_FDS,
						 WP_TAP_CFG0_LIR);
	/* No wake duration, fine threshold resolution. */
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_WAKE_UP_DUR,
						 WP_WU_DUR_THS_W | WP_WU_DUR_WAKE_DUR_MASK,
						 WP_WU_DUR_THS_W);
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_WAKE_UP_THS,
						 WP_WU_THS_MASK | WP_WU_THS_USR_OFF, (uint8_t)ths);
	/* Enable basic interrupts (no inactivity), route wake-up to INT1. */
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_TAP_CFG2,
						 WP_TAP_CFG2_INT_EN | WP_TAP_CFG2_INACT_MASK,
						 WP_TAP_CFG2_INT_EN);
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_MD1_CFG, WP_MD1_INT1_WU,
						 WP_MD1_INT1_WU);
	if (err != 0) {
		return err;
	}

	/* Let the slope filter settle at the new ODR, then clear any latched event. */
	k_msleep(120);
	err = i2c_reg_read_byte_dt(&imu_i2c, WP_LSM6DSOX_WAKE_UP_SRC, &src);
	if (err != 0) {
		return err;
	}

	/* Input only: the level sense is enabled by wp_imu_source_arm_wake_pin()
	 * with interrupts locked, immediately before sys_poweroff(). */
	err = gpio_pin_configure_dt(&imu_int1, GPIO_INPUT);
	if (err == 0) {
		err = gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_DISABLE);
	}
	if (err == 0 && !imu_int1_cb_added) {
		/* Defence in depth: if the level interrupt ever fires while the
		 * system still runs, mask it instead of re-entering forever. */
		gpio_init_callback(&imu_int1_cb, imu_int1_isr, BIT(imu_int1.pin));
		err = gpio_add_callback_dt(&imu_int1, &imu_int1_cb);
		imu_int1_cb_added = (err == 0);
	}
	if (err == 0) {
		LOG_INF("wake-up prepared: INT1, ths %u LSB (%u ug/LSB)", ths, lsb_ug);
	}
	return err;
}

/* System OFF was aborted: undo the wake-up routing and accel low-power mode. */
static void disarm_wakeup(void)
{
	int err;

	(void)gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_DISABLE);
	err = i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_MD1_CFG, WP_MD1_INT1_WU, 0);
	err = err ? err : i2c_reg_update_byte_dt(&imu_i2c, WP_LSM6DSOX_CTRL6_C,
						 WP_CTRL6_XL_HM_MODE, 0);
	if (err != 0) {
		LOG_WRN("wake-up disarm failed (%d)", err);
	}
}
#endif /* WP_IMU_HAS_WAKEUP */

/* ---- Public API --------------------------------------------------------- */

int wp_imu_source_init(void)
{
	struct wp_gyro_bias_config bcfg;
	struct wp_imu_plaus_config pcfg;
	int err;

	if (atomic_get(&imu_started) != 0) {
		return 0;
	}

	wp_config_imu_mount(&imu.mount);
	wp_config_gyro_bias(&bcfg);
	wp_gyro_bias_init(&imu.bias, &bcfg);
	wp_config_imu_plaus(&pcfg);
	wp_imu_plaus_init(&imu.plaus, &pcfg);

	if (!device_is_ready(imu_dev)) {
		/* The node exists: its init failed (bus glitch at boot). Retry the
		 * driver init; the router keeps calling reinit() with back-off.
		 * No explicit bus recovery here: the touch driver may be using
		 * the shared bus and i2c_recover_bus() does not take the TWIM
		 * lock (the IQS5xx driver recovers the bus once at boot, before
		 * the sensors probe). */
		err = device_init(imu_dev);
		if (err != 0 && err != -EALREADY) {
			LOG_WRN("%s not ready (%d), retrying later", imu_dev->name, err);
			(void)wp_router_post_imu_status(WP_IMU_STATUS_ERROR);
			return -EIO;
		}
		LOG_INF("%s initialized on retry", imu_dev->name);
	}

	imu.period_us = 1000000U / CONFIG_WAFER_PAD_IMU_ODR_HZ;
	err = apply_rates(true);
	if (err != 0) {
		LOG_WRN("initial configuration failed (%d), retrying in thread", err);
		atomic_set(&req_reinit, 1);
	}

	bias_persist_init();
	publish_bias();

	atomic_set(&imu_present, 1);
	atomic_set(&imu_started, 1);
	k_thread_create(&imu_thread_data, imu_stack, K_THREAD_STACK_SIZEOF(imu_stack),
			imu_thread_fn, NULL, NULL, NULL,
			K_PRIO_PREEMPT(CONFIG_WAFER_PAD_IMU_THREAD_PRIORITY), K_FP_REGS,
			K_NO_WAIT);
	k_thread_name_set(&imu_thread_data, "wp_imu");

	LOG_INF("%s: %u Hz, mount {%d,%d,%d}", imu_dev->name, CONFIG_WAFER_PAD_IMU_ODR_HZ,
		imu.mount.map[0], imu.mount.map[1], imu.mount.map[2]);
	(void)wp_router_post_imu_status(WP_IMU_STATUS_OK);
	return 0;
}

bool wp_imu_source_present(void)
{
	return atomic_get(&imu_present) != 0;
}

int wp_imu_source_set_gyro(bool enabled)
{
	atomic_set(&req_gyro_on, enabled ? 1 : 0);
	if (!wp_imu_source_present()) {
		return -ENODEV;
	}
	k_sem_give(&imu_kick);
	return 0;
}

int wp_imu_source_set_low_rate(bool low_rate)
{
	atomic_set(&req_low_rate, low_rate ? 1 : 0);
	if (!wp_imu_source_present()) {
		return -ENODEV;
	}
	k_sem_give(&imu_kick);
	return 0;
}

bool wp_imu_source_get_bias(float bias[3])
{
	k_spinlock_key_t key = k_spin_lock(&bias_lock);
	const bool valid = bias_snap_valid;

	if (valid) {
		memcpy(bias, bias_snap, sizeof(bias_snap));
	}
	k_spin_unlock(&bias_lock, key);
	return valid;
}

int wp_imu_source_reinit(void)
{
	if (atomic_get(&imu_started) == 0) {
		return wp_imu_source_init();
	}
	if (!device_is_ready(imu_dev)) {
		return -EIO;
	}
	/* Also resumes sampling after prepare_wakeup(). */
	atomic_set(&req_reinit, 1);
	atomic_set(&req_suspend, 0);
	k_sem_give(&imu_kick);
	return 0;
}

int wp_imu_source_prepare_wakeup(void)
{
	const struct sensor_value off = {0};
	int err = -ENOTSUP;

	if (!wp_imu_source_present()) {
		return -ENODEV;
	}

	/* Stop the sampling thread so it does not touch the sensor any more. */
	k_sem_reset(&imu_suspend_ack);
	atomic_set(&req_suspend, 1);
	k_sem_give(&imu_kick);
	if (k_sem_take(&imu_suspend_ack, K_MSEC(WP_IMU_SUSPEND_WAIT_MS)) != 0) {
		LOG_WRN("IMU thread did not acknowledge suspend");
	}

#if WP_IMU_HAS_WAKEUP
	atomic_set(&wake_armed, 1);
	err = arm_wakeup();
	if (err == 0) {
		return 0;
	}
	atomic_set(&wake_armed, 0);
	LOG_ERR("wake-up arming failed (%d)", err);
#endif

	/* Not a wake source: power the sensor down to save battery in System OFF. */
	(void)sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &off);
	(void)sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &off);
	return err;
}

void wp_imu_source_arm_wake_pin(void)
{
#if WP_IMU_HAS_WAKEUP
	/* Interrupts are locked by the caller: only PIN_CNF.SENSE is written.
	 * A level that is already active wakes the SoC from System OFF at once,
	 * which is the correct outcome (a reboot). */
	if (atomic_get(&wake_armed) != 0) {
		(void)gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_LEVEL_ACTIVE);
	}
#endif
}

void wp_imu_source_set_bias_learning(bool allowed)
{
	atomic_set(&bias_learning, allowed ? 1 : 0);
}

#else /* IMU disabled or motion0 missing: graceful stubs */

int wp_imu_source_init(void)
{
	LOG_INF("IMU not available (CONFIG_WAFER_PAD_IMU=n or no motion0 alias)");
	(void)wp_router_post_imu_status(WP_IMU_STATUS_ABSENT);
	return -ENODEV;
}

bool wp_imu_source_present(void)
{
	return false;
}

int wp_imu_source_set_gyro(bool enabled)
{
	ARG_UNUSED(enabled);
	return -ENODEV;
}

int wp_imu_source_set_low_rate(bool low_rate)
{
	ARG_UNUSED(low_rate);
	return -ENODEV;
}

bool wp_imu_source_get_bias(float bias[3])
{
	ARG_UNUSED(bias);
	return false;
}

int wp_imu_source_reinit(void)
{
	return -ENODEV;
}

int wp_imu_source_prepare_wakeup(void)
{
	return -ENODEV;
}

void wp_imu_source_arm_wake_pin(void)
{
}

void wp_imu_source_set_bias_learning(bool allowed)
{
	ARG_UNUSED(allowed);
}

#endif
