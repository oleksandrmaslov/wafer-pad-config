/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Azoteq IQS5xx-B000 trackpad controller driver (IQS550 on the TPS65 module).
 *
 * Written from the IQS5xx-B000 datasheet facts in the project research notes;
 * no code was taken from the AYM1607 ZMK driver (its known defects, research
 * notes §8, are avoided on purpose: full re-init on SHOW_RESET, read-modify-
 * write of SYS_CFG0/SYS_CFG1, factory XY_CFG0 kept, product check + retries,
 * ACK_RESET during init, RDY armed only after setup, level-triggered RDY,
 * one coherent burst per frame, the declared init priority is used).
 *
 * Concurrency model
 * - Everything that talks to the controller runs with data->lock held, either
 *   on the driver's own work queue (frames, bring-up, recovery, watchdog) or
 *   in the caller's thread (iqs5xx_reinit/set_low_power/prepare_wakeup).
 * - RDY is a level-high interrupt. The ISR masks it, timestamps and submits
 *   the frame work; the work reads one 45-byte burst, closes the window and
 *   re-arms the interrupt (a window that is still open re-triggers at once).
 * - Shared bus rule: outside bring-up/recovery the controller is only
 *   addressed while RDY is high, except for the documented forced accesses
 *   (profile changes, wake-up preparation, stale-contact watchdog), which
 *   mask RDY first.
 * - Callbacks run after the lock is released, from the work queue (or from
 *   the thread that called iqs5xx_reinit()).
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <drivers/azoteq_iqs5xx.h>

#include "iqs5xx_regs.h"

LOG_MODULE_REGISTER(azoteq_iqs5xx, CONFIG_AZOTEQ_IQS5XX_LOG_LEVEL);

BUILD_ASSERT(CONFIG_AZOTEQ_IQS5XX_INIT_PRIORITY > CONFIG_I2C_INIT_PRIORITY,
	     "IQS5xx must initialize after the I2C controller");
#if defined(CONFIG_REGULATOR_FIXED_INIT_PRIORITY)
BUILD_ASSERT(CONFIG_AZOTEQ_IQS5XX_INIT_PRIORITY > CONFIG_REGULATOR_FIXED_INIT_PRIORITY,
	     "IQS5xx must initialize after the fixed regulator powering the sensors");
#endif

/** Window-time failures in a row before the recovery sequence runs. */
#define IQS5XX_MAX_CONSECUTIVE_ERRORS 3

/** Largest register payload written in one transaction (report rates). */
#define IQS5XX_MAX_WRITE (2 * IQS5XX_RR_COUNT)

/** Back-off between failed bring-up attempts (boot or recovery). After the
 *  last one the driver reports IQS5XX_EVT_FATAL. Keeps an unplugged module
 *  from hogging the bus shared with the IMU.
 */
static const uint16_t iqs5xx_retry_delay_ms[] = {1000, 5000};

enum iqs5xx_state {
	IQS5XX_STATE_PENDING = 0, /* bring-up queued or running */
	IQS5XX_STATE_UP,          /* configured, RDY armed */
	IQS5XX_STATE_RECOVERING,  /* communication lost, recovery queued/running */
	IQS5XX_STATE_FATAL,       /* gave up, iqs5xx_reinit() required */
};

struct iqs5xx_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec rdy;
	struct gpio_dt_spec reset; /* .port == NULL when reset-gpios is absent */
	uint16_t active_rr_ms;     /* 0 = keep the NV default */
	bool event_mode;           /* DT default for the normal profile */
};

struct iqs5xx_data {
	const struct device *dev;
	struct k_mutex lock;     /* all controller I/O + the state below */
	struct k_spinlock slock; /* cb, user_data, info, stats, state snapshot */
	struct gpio_callback rdy_cb;
	struct k_work rdy_work;
	struct k_work_delayable init_work;
	struct k_work_delayable watchdog;

	iqs5xx_callback_t cb;
	void *user_data;
	struct iqs5xx_info info;
	struct iqs5xx_stats stats;

	int64_t rdy_ticks;    /* RDY interrupt time, written by the ISR */
	int64_t last_rdy_ms;  /* last serviced window */
	int64_t forced_ok_ms; /* no forced comms before this uptime (ATI) */

	uint16_t nv_rr[IQS5XX_RR_COUNT];
	bool nv_rr_valid;

	enum iqs5xx_state state;
	uint8_t init_attempt;
	uint8_t consecutive_errors;
	bool recovering;   /* the queued bring-up is a recovery */
	bool event_mode;   /* runtime EVENT_MODE */
	bool low_power;    /* low-power report-rate profile */
	bool wakeup_armed; /* prepare_wakeup() done: no watchdog traffic */
	bool fingers_down; /* last frame had at least one active slot */

	uint8_t buf[IQS5XX_FRAME_LEN];
};

/* Events collected under the lock and delivered after it is released. */
#define IQS5XX_MAX_NOTIFY 3

struct iqs5xx_notify {
	uint8_t count;
	enum iqs5xx_event evt[IQS5XX_MAX_NOTIFY];
	struct iqs5xx_frame frame;
};

static K_THREAD_STACK_DEFINE(iqs5xx_wq_stack, CONFIG_AZOTEQ_IQS5XX_WORKQ_STACK_SIZE);
static struct k_work_q iqs5xx_wq;
static bool iqs5xx_wq_started;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static void iqs5xx_notify_add(struct iqs5xx_notify *nt, enum iqs5xx_event evt)
{
	if (nt->count < IQS5XX_MAX_NOTIFY) {
		nt->evt[nt->count++] = evt;
	}
}

static void iqs5xx_notify_flush(const struct device *dev, const struct iqs5xx_notify *nt)
{
	struct iqs5xx_data *data = dev->data;
	iqs5xx_callback_t cb;
	void *user_data;
	k_spinlock_key_t key;

	if (nt->count == 0) {
		return;
	}

	key = k_spin_lock(&data->slock);
	cb = data->cb;
	user_data = data->user_data;
	k_spin_unlock(&data->slock, key);

	if (cb == NULL) {
		return;
	}

	for (uint8_t i = 0; i < nt->count; i++) {
		const struct iqs5xx_frame *frame =
			(nt->evt[i] == IQS5XX_EVT_FRAME) ? &nt->frame : NULL;

		cb(dev, nt->evt[i], frame, user_data);
	}
}

static void iqs5xx_set_state(struct iqs5xx_data *data, enum iqs5xx_state state)
{
	k_spinlock_key_t key = k_spin_lock(&data->slock);

	data->state = state;
	k_spin_unlock(&data->slock, key);
}

static void iqs5xx_count(struct iqs5xx_data *data, uint32_t *counter)
{
	k_spinlock_key_t key = k_spin_lock(&data->slock);

	(*counter)++;
	k_spin_unlock(&data->slock, key);
}

static int iqs5xx_read(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len)
{
	const struct iqs5xx_config *cfg = dev->config;
	uint8_t addr[2];

	sys_put_be16(reg, addr);
	return i2c_write_read_dt(&cfg->i2c, addr, sizeof(addr), buf, len);
}

/* One i2c_write_dt() with [addr_hi, addr_lo, data...]: never i2c_burst_write()
 * (8-bit register address, and the nRF TWIM would have to merge two messages
 * in its small concatenation buffer).
 */
static int iqs5xx_write(const struct device *dev, uint16_t reg, const uint8_t *val, size_t len)
{
	const struct iqs5xx_config *cfg = dev->config;
	uint8_t buf[2 + IQS5XX_MAX_WRITE];

	if ((len == 0U) || (len > IQS5XX_MAX_WRITE)) {
		return -EINVAL;
	}

	sys_put_be16(reg, buf);
	memcpy(&buf[2], val, len);
	return i2c_write_dt(&cfg->i2c, buf, len + 2U);
}

/* Every forced access and every TWIM stall holds the SHARED bus (and the IMU)
 * for up to this long. An upstream shield Kconfig raises the default to
 * 1000 ms whenever SENSOR=y, so the application sets it explicitly. */
#if defined(CONFIG_I2C_TRANSFER_TIMEOUT_MS)
BUILD_ASSERT(CONFIG_I2C_TRANSFER_TIMEOUT_MS > 0 && CONFIG_I2C_TRANSFER_TIMEOUT_MS <= 500,
	     "set CONFIG_I2C_TRANSFER_TIMEOUT_MS to 1..500 (longest legit clock stretch ~200 ms)");
#endif

/* Retries cover the documented NACK of the first addressing while the
 * controller is in a low-power state (retry >= 150 us after the STOP). A
 * TWIM transfer timeout already took CONFIG_I2C_TRANSFER_TIMEOUT_MS and
 * recovered the bus, so it is not retried here.
 */
static int iqs5xx_read_retry(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len,
			     int attempts)
{
	int ret = -EIO;

	for (int i = 0; i < attempts; i++) {
		if (i > 0) {
			k_busy_wait(IQS5XX_RETRY_DELAY_US);
		}
		ret = iqs5xx_read(dev, reg, buf, len);
		if ((ret == 0) || (ret == -ETIMEDOUT)) {
			break;
		}
	}
	return ret;
}

static int iqs5xx_write_retry(const struct device *dev, uint16_t reg, const uint8_t *val,
			      size_t len, int attempts)
{
	int ret = -EIO;

	for (int i = 0; i < attempts; i++) {
		if (i > 0) {
			k_busy_wait(IQS5XX_RETRY_DELAY_US);
		}
		ret = iqs5xx_write(dev, reg, val, len);
		if ((ret == 0) || (ret == -ETIMEDOUT)) {
			break;
		}
	}
	return ret;
}

static int iqs5xx_write_u8(const struct device *dev, uint16_t reg, uint8_t val, int attempts)
{
	return iqs5xx_write_retry(dev, reg, &val, 1, attempts);
}

/* Close the communication window: one 3-byte write EE EE 00 + STOP. */
static int iqs5xx_end_window(const struct device *dev)
{
	return iqs5xx_write_u8(dev, IQS5XX_REG_END_WINDOW, 0x00, IQS5XX_WINDOW_ATTEMPTS);
}

static void iqs5xx_irq_disable(const struct device *dev)
{
	const struct iqs5xx_config *cfg = dev->config;

	(void)gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_DISABLE);
}

static void iqs5xx_irq_arm(const struct device *dev)
{
	const struct iqs5xx_config *cfg = dev->config;
	int ret = gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_LEVEL_ACTIVE);

	if (ret < 0) {
		LOG_ERR("cannot arm RDY interrupt (%d)", ret);
	}
}

/* Prepare a forced access (controller addressed while RDY may be low).
 * Lock held. Masks RDY (addressing the device raises RDY) and honours the
 * quiet time after SETUP_COMPLETE, during which ATI runs and a forced access
 * would clock-stretch the shared bus for a long time.
 */
static void iqs5xx_forced_begin(const struct device *dev)
{
	struct iqs5xx_data *data = dev->data;
	int64_t wait_ms;

	iqs5xx_irq_disable(dev);
	wait_ms = data->forced_ok_ms - k_uptime_get();
	if (wait_ms > 0) {
		k_msleep((int32_t)wait_ms);
	}
}

static void iqs5xx_watchdog_kick(const struct device *dev, bool restart)
{
	struct iqs5xx_data *data = dev->data;
	int32_t period_ms;

	if ((data->state != IQS5XX_STATE_UP) || data->wakeup_armed) {
		return;
	}

	if (!data->event_mode) {
		period_ms = CONFIG_AZOTEQ_IQS5XX_RDY_TIMEOUT_MS;
	} else if (data->fingers_down) {
		period_ms = CONFIG_AZOTEQ_IQS5XX_STALE_MS;
	} else {
		return; /* event mode, nothing down: no RDY is expected */
	}

	if (restart) {
		(void)k_work_reschedule_for_queue(&iqs5xx_wq, &data->watchdog, K_MSEC(period_ms));
	} else {
		/* No-op if already scheduled; the handler re-checks the real age. */
		(void)k_work_schedule_for_queue(&iqs5xx_wq, &data->watchdog, K_MSEC(period_ms));
	}
}

/* ------------------------------------------------------------------------ */
/* Controller configuration                                                 */
/* ------------------------------------------------------------------------ */

static int iqs5xx_identify(const struct device *dev, struct iqs5xx_info *info)
{
	const struct iqs5xx_config *cfg = dev->config;
	uint8_t id[IQS5XX_ID_LEN];
	uint16_t prod;
	int ret;

	ret = iqs5xx_read_retry(dev, IQS5XX_REG_PROD_NUM, id, sizeof(id),
				IQS5XX_FORCED_ATTEMPTS);
	if (ret < 0) {
		LOG_ERR("no response at 0x%02x (%d)", cfg->i2c.addr, ret);
		return ret;
	}

	prod = sys_get_be16(&id[0]);
	switch (prod) {
	case IQS5XX_PRODUCT_IQS550:
		break;
	case IQS5XX_PRODUCT_IQS572:
	case IQS5XX_PRODUCT_IQS525:
		LOG_WRN("product %u is not an IQS550 (TPS65), continuing", prod);
		break;
	default:
		/* An A000 device answers 16-bit addressing shifted by one byte. */
		LOG_ERR("unsupported product number %u (not IQS5xx-B000?)", prod);
		return -ENODEV;
	}

	info->product_number = prod;
	info->project_number = sys_get_be16(&id[2]);
	info->version_major = id[4];
	info->version_minor = id[5];

	LOG_INF("IQS5xx product %u project %u fw %u.%u bootloader 0x%02x", prod,
		info->project_number, info->version_major, info->version_minor, id[6]);
	return 0;
}

static void iqs5xx_profile_rates(const struct device *dev, uint16_t rr[IQS5XX_RR_COUNT])
{
	static const uint16_t lp_rr[IQS5XX_RR_COUNT] = {
		IQS5XX_LP_RR_ACTIVE_MS, IQS5XX_LP_RR_IDLE_TOUCH_MS, IQS5XX_LP_RR_IDLE_MS,
		IQS5XX_LP_RR_LP1_MS,    IQS5XX_LP_RR_LP2_MS,
	};
	const struct iqs5xx_config *cfg = dev->config;
	struct iqs5xx_data *data = dev->data;

	for (int i = 0; i < IQS5XX_RR_COUNT; i++) {
		rr[i] = data->nv_rr[i];
	}
	if (cfg->active_rr_ms != 0U) {
		rr[0] = cfg->active_rr_ms;
	}
	if (data->low_power) {
		for (int i = 0; i < IQS5XX_RR_COUNT; i++) {
			rr[i] = MAX(rr[i], lp_rr[i]);
		}
	}
}

/* Report rates for the current profile. The first successful read is kept as
 * the controller's NV defaults; only differing values are written.
 */
static int iqs5xx_apply_rates(const struct device *dev, int attempts)
{
	struct iqs5xx_data *data = dev->data;
	uint8_t raw[2 * IQS5XX_RR_COUNT];
	uint16_t want[IQS5XX_RR_COUNT];
	bool dirty = false;
	int ret;

	ret = iqs5xx_read_retry(dev, IQS5XX_REG_RR_ACTIVE, raw, sizeof(raw), attempts);
	if (ret < 0) {
		return ret;
	}

	if (!data->nv_rr_valid) {
		for (int i = 0; i < IQS5XX_RR_COUNT; i++) {
			data->nv_rr[i] = sys_get_be16(&raw[2 * i]);
		}
		data->nv_rr_valid = true;
		LOG_INF("NV report rates [ms]: active %u idle-touch %u idle %u lp1 %u lp2 %u",
			data->nv_rr[0], data->nv_rr[1], data->nv_rr[2], data->nv_rr[3],
			data->nv_rr[4]);
	}

	iqs5xx_profile_rates(dev, want);
	for (int i = 0; i < IQS5XX_RR_COUNT; i++) {
		if (sys_get_be16(&raw[2 * i]) != want[i]) {
			sys_put_be16(want[i], &raw[2 * i]);
			dirty = true;
		}
	}

	if (!dirty) {
		return 0;
	}
	LOG_DBG("report rates: active %u ms (%s profile)", want[0],
		data->low_power ? "low-power" : "normal");
	return iqs5xx_write_retry(dev, IQS5XX_REG_RR_ACTIVE, raw, sizeof(raw), attempts);
}

/* SYS_CFG1 read-modify-write: TP_EVENT on, GESTURE_EVENT off (gestures are
 * recognized in software), EVENT_MODE per runtime setting, rest untouched.
 */
static int iqs5xx_apply_sys_cfg1(const struct device *dev, int attempts, uint8_t *out)
{
	struct iqs5xx_data *data = dev->data;
	uint8_t v;
	int ret;

	ret = iqs5xx_read_retry(dev, IQS5XX_REG_SYS_CFG1, &v, 1, attempts);
	if (ret < 0) {
		return ret;
	}

	v |= IQS5XX_TP_EVENT;
	v &= (uint8_t)~IQS5XX_GESTURE_EVENT;
	if (data->event_mode) {
		v |= IQS5XX_EVENT_MODE;
	} else {
		v &= (uint8_t)~IQS5XX_EVENT_MODE;
	}

	if (out != NULL) {
		*out = v;
	}
	return iqs5xx_write_u8(dev, IQS5XX_REG_SYS_CFG1, v, IQS5XX_WINDOW_ATTEMPTS);
}

/* Full configuration inside one window (opened by RDY or by forced access).
 * Lock held, RDY masked. Ends the window on success and (best effort) after
 * a failure once the controller has answered.
 */
static int iqs5xx_configure(const struct device *dev)
{
	struct iqs5xx_data *data = dev->data;
	struct iqs5xx_info info = {0};
	uint8_t cfg0, cfg1, xy_cfg0;
	uint8_t res[4];
	uint8_t gestures[2] = {0x00, 0x00};
	k_spinlock_key_t key;
	int ret;

	ret = iqs5xx_identify(dev, &info);
	if (ret < 0) {
		return ret;
	}

	/* Acknowledge the reset that brought us here (clears SHOW_RESET). */
	ret = iqs5xx_write_u8(dev, IQS5XX_REG_SYS_CTRL0, IQS5XX_ACK_RESET,
			      IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	/* SYS_CFG0 read-modify-write: keep MANUAL_CONTROL / SW input bits. */
	ret = iqs5xx_read_retry(dev, IQS5XX_REG_SYS_CFG0, &cfg0, 1, IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}
	cfg0 |= IQS5XX_SETUP_COMPLETE | IQS5XX_WDT | IQS5XX_ALP_REATI | IQS5XX_REATI;
	ret = iqs5xx_write_u8(dev, IQS5XX_REG_SYS_CFG0, cfg0, IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	ret = iqs5xx_apply_sys_cfg1(dev, IQS5XX_WINDOW_ATTEMPTS, &cfg1);
	if (ret < 0) {
		goto fail;
	}

	/* The controller's own single- and multi-finger gestures are disabled:
	 * Wafer Pad recognizes gestures in software from the raw contacts.
	 */
	ret = iqs5xx_write_retry(dev, IQS5XX_REG_SF_GESTURES, gestures, sizeof(gestures),
				 IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	/* Max multi-touches (0x066A, 1 byte, <= 5) = all 5 slots, instead of
	 * relying on the NV default. Research notes iqs5xx.md §4.2 (address
	 * verified: DS p35 §5.3, ARD) and §5.1 step 5. The register holds only
	 * this count (0x066B is a separate register), so a plain write is
	 * correct; volatile, re-applied after every reset with the rest. More
	 * fingers than this raise TOO_MANY_FINGERS (frame then carries no XY).
	 */
	ret = iqs5xx_write_u8(dev, IQS5XX_REG_MAX_MULTI_TOUCH, IQS5XX_MAX_FINGERS,
			      IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	ret = iqs5xx_apply_rates(dev, IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	/* XY_CFG0 is NOT written: the factory SWITCH_XY_AXIS / FLIP / PALM_REJECT
	 * bits stay. Axis correction for the mounting is done in software.
	 */
	ret = iqs5xx_read_retry(dev, IQS5XX_REG_XY_CFG0, &xy_cfg0, 1, IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	ret = iqs5xx_read_retry(dev, IQS5XX_REG_X_RES, res, sizeof(res), IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}
	info.resolution_x = sys_get_be16(&res[0]);
	info.resolution_y = sys_get_be16(&res[2]);

	ret = iqs5xx_read_retry(dev, IQS5XX_REG_I2C_TIMEOUT, &info.i2c_timeout_ms, 1,
				IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		goto fail;
	}

	ret = iqs5xx_end_window(dev);
	if (ret < 0) {
		return ret;
	}

	key = k_spin_lock(&data->slock);
	data->info = info;
	k_spin_unlock(&data->slock, key);

	LOG_INF("resolution %ux%u, i2c timeout %u ms, sys_cfg0 0x%02x sys_cfg1 0x%02x "
		"xy_cfg0 0x%02x (%s%s%s), %s mode",
		info.resolution_x, info.resolution_y, info.i2c_timeout_ms, cfg0, cfg1, xy_cfg0,
		(xy_cfg0 & IQS5XX_SWITCH_XY_AXIS) ? "switch-xy " : "",
		(xy_cfg0 & IQS5XX_FLIP_X) ? "flip-x " : "",
		(xy_cfg0 & IQS5XX_FLIP_Y) ? "flip-y" : "",
		data->event_mode ? "event" : "streaming");
	return 0;

fail:
	LOG_ERR("configuration failed (%d)", ret);
	(void)iqs5xx_end_window(dev);
	return ret;
}

/* Hardware reset through NRST, or a software reset (SYS_CTRL1.RESET, takes
 * effect when the window ends) when reset-gpios is absent.
 */
static void iqs5xx_reset(const struct device *dev)
{
	const struct iqs5xx_config *cfg = dev->config;
	int ret;

	if (cfg->reset.port != NULL) {
		(void)gpio_pin_set_dt(&cfg->reset, 1);
		k_msleep(IQS5XX_RESET_PULSE_MS);
		(void)gpio_pin_set_dt(&cfg->reset, 0);
		k_msleep(IQS5XX_HW_RESET_WAIT_MS);
		return;
	}

	ret = iqs5xx_write_u8(dev, IQS5XX_REG_SYS_CTRL1, IQS5XX_SW_RESET, IQS5XX_FORCED_ATTEMPTS);
	if (ret == 0) {
		ret = iqs5xx_end_window(dev);
	}
	if (ret < 0) {
		LOG_WRN("software reset failed (%d)", ret);
	}
	k_msleep(IQS5XX_SW_RESET_WAIT_MS);
}

static bool iqs5xx_wait_rdy(const struct device *dev, int timeout_ms)
{
	const struct iqs5xx_config *cfg = dev->config;

	for (int t = 0; t < timeout_ms; t++) {
		if (gpio_pin_get_dt(&cfg->rdy) > 0) {
			return true;
		}
		k_msleep(1);
	}
	return gpio_pin_get_dt(&cfg->rdy) > 0;
}

/* Reset + configure + arm. Lock held. On success the state is UP. */
static int iqs5xx_bringup(const struct device *dev)
{
	struct iqs5xx_data *data = dev->data;
	int ret;

	iqs5xx_irq_disable(dev);
	(void)k_work_cancel_delayable(&data->watchdog);
	data->fingers_down = false;
	data->consecutive_errors = 0;
	data->wakeup_armed = false;
	data->forced_ok_ms = 0;

	iqs5xx_reset(dev);

	if (!iqs5xx_wait_rdy(dev, IQS5XX_FIRST_RDY_WAIT_MS)) {
		LOG_WRN("RDY did not rise after reset, using forced communication");
	}

	ret = iqs5xx_configure(dev);
	if (ret < 0) {
		return ret;
	}

	k_busy_wait(IQS5XX_RDY_SETTLE_US);
	data->forced_ok_ms = k_uptime_get() + IQS5XX_ATI_SETTLE_MS;
	data->last_rdy_ms = k_uptime_get();
	iqs5xx_set_state(data, IQS5XX_STATE_UP);
	iqs5xx_irq_arm(dev);
	iqs5xx_watchdog_kick(dev, true);
	return 0;
}

/* Lock held. Mask RDY and queue the reset + re-init sequence.
 *
 * No explicit i2c_recover_bus() here (nor in iqs5xx_reinit()): on the nRF
 * TWIM it does not take the driver's transfer lock and would kill a
 * concurrent IMU transfer on the shared bus (a ~1 s IMU stall per retry).
 * An address NACK is not a stuck bus; a real stall ends in a TWIM transfer
 * timeout, which the TWIM driver already recovers; and the NRST pulse of the
 * bring-up makes the controller release SDA. The only explicit recovery is
 * the one in iqs5xx_init(), before any other bus user exists.
 */
static void iqs5xx_start_recovery(const struct device *dev, struct iqs5xx_notify *nt)
{
	struct iqs5xx_data *data = dev->data;

	LOG_ERR("communication lost, starting recovery");
	iqs5xx_irq_disable(dev);
	(void)k_work_cancel_delayable(&data->watchdog);
	iqs5xx_set_state(data, IQS5XX_STATE_RECOVERING);
	data->recovering = true;
	data->init_attempt = 0;
	data->fingers_down = false;
	iqs5xx_notify_add(nt, IQS5XX_EVT_ERROR);
	(void)k_work_reschedule_for_queue(&iqs5xx_wq, &data->init_work, K_NO_WAIT);
}

static void iqs5xx_comm_error(const struct device *dev, int err, struct iqs5xx_notify *nt)
{
	struct iqs5xx_data *data = dev->data;

	iqs5xx_count(data, &data->stats.i2c_errors);
	data->consecutive_errors++;
	LOG_WRN("I2C error %d (%u in a row)", err, data->consecutive_errors);

	if ((data->consecutive_errors >= IQS5XX_MAX_CONSECUTIVE_ERRORS) || (err == -ETIMEDOUT)) {
		iqs5xx_start_recovery(dev, nt);
		return;
	}

	/* Best effort: close the window so the controller keeps cycling. */
	(void)iqs5xx_end_window(dev);
	k_busy_wait(IQS5XX_RDY_SETTLE_US);
}

static void iqs5xx_parse_frame(const uint8_t *b, int64_t ts_us, struct iqs5xx_frame *f)
{
	f->timestamp_us = ts_us;
	f->prev_cycle_ms = b[IQS5XX_FRM_PREV_CYCLE];
	f->gesture_events0 = b[IQS5XX_FRM_GESTURE0];
	f->gesture_events1 = b[IQS5XX_FRM_GESTURE1];
	f->sys_info0 = b[IQS5XX_FRM_SYS_INFO0];
	f->sys_info1 = b[IQS5XX_FRM_SYS_INFO1];
	f->num_fingers = b[IQS5XX_FRM_NUM_FINGERS];
	f->rel_x = (int16_t)sys_get_be16(&b[IQS5XX_FRM_REL_X]);
	f->rel_y = (int16_t)sys_get_be16(&b[IQS5XX_FRM_REL_Y]);

	for (int i = 0; i < IQS5XX_MAX_FINGERS; i++) {
		const uint8_t *p = &b[IQS5XX_FRM_FINGER0 + (IQS5XX_FRM_FINGER_STRIDE * i)];

		f->fingers[i].x = sys_get_be16(&p[0]);
		f->fingers[i].y = sys_get_be16(&p[2]);
		f->fingers[i].strength = sys_get_be16(&p[4]);
		f->fingers[i].area = p[6];
	}
}

/* Read and handle one window. Lock held, RDY masked. The caller re-arms RDY
 * if the state is still UP afterwards.
 */
static void iqs5xx_service(const struct device *dev, int64_t ts_us, bool forced,
			   struct iqs5xx_notify *nt)
{
	struct iqs5xx_data *data = dev->data;
	uint8_t *b = data->buf;
	bool down = false;
	int ret;

	ret = iqs5xx_read_retry(dev, IQS5XX_FRAME_START, b, IQS5XX_FRAME_LEN,
				forced ? IQS5XX_FORCED_ATTEMPTS : IQS5XX_WINDOW_ATTEMPTS);
	if (ret < 0) {
		iqs5xx_comm_error(dev, ret, nt);
		return;
	}
	data->consecutive_errors = 0;
	data->last_rdy_ms = k_uptime_get();

	if ((b[IQS5XX_FRM_SYS_INFO0] & IQS5XX_SHOW_RESET) != 0U) {
		/* The controller reset (WDT, ESD, brown-out) and runs on its NV
		 * defaults: drop this frame and re-apply the full configuration
		 * inside the same window.
		 */
		iqs5xx_count(data, &data->stats.device_resets);
		LOG_WRN("controller reset detected, re-initializing");
		data->fingers_down = false;
		ret = iqs5xx_configure(dev);
		if (ret < 0) {
			iqs5xx_count(data, &data->stats.i2c_errors);
			iqs5xx_start_recovery(dev, nt);
			return;
		}
		k_busy_wait(IQS5XX_RDY_SETTLE_US);
		data->forced_ok_ms = k_uptime_get() + IQS5XX_ATI_SETTLE_MS;
		iqs5xx_notify_add(nt, IQS5XX_EVT_RESET);
		return;
	}

	ret = iqs5xx_end_window(dev);
	k_busy_wait(IQS5XX_RDY_SETTLE_US);

	iqs5xx_parse_frame(b, ts_us, &nt->frame);
	for (int i = 0; i < IQS5XX_MAX_FINGERS; i++) {
		if (nt->frame.fingers[i].strength != 0U) {
			down = true;
		}
	}
	data->fingers_down = down;
	iqs5xx_count(data, &data->stats.frames);
	iqs5xx_notify_add(nt, IQS5XX_EVT_FRAME);

	if (ret < 0) {
		/* The frame is valid, but a window that does not close would
		 * stall the controller until its I2C timeout.
		 */
		iqs5xx_comm_error(dev, ret, nt);
	}
}

/* ------------------------------------------------------------------------ */
/* ISR and work handlers                                                    */
/* ------------------------------------------------------------------------ */

static void iqs5xx_rdy_isr(const struct device *port, struct gpio_callback *cb,
			   gpio_port_pins_t pins)
{
	struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);
	const struct iqs5xx_config *cfg = data->dev->config;

	ARG_UNUSED(port);

	if ((pins & BIT(cfg->rdy.pin)) == 0U) {
		return;
	}

	/* Level interrupt: mask until the window has been serviced. */
	(void)gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_DISABLE);
	data->rdy_ticks = k_uptime_ticks();
	(void)k_work_submit_to_queue(&iqs5xx_wq, &data->rdy_work);
}

static void iqs5xx_rdy_work_handler(struct k_work *work)
{
	struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, rdy_work);
	const struct device *dev = data->dev;
	const struct iqs5xx_config *cfg = dev->config;
	struct iqs5xx_notify nt = {0};
	int64_t ts_us;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->state != IQS5XX_STATE_UP) {
		/* Bring-up / recovery owns the interrupt and re-arms it. */
		k_mutex_unlock(&data->lock);
		return;
	}

	if (gpio_pin_get_dt(&cfg->rdy) <= 0) {
		/* The window already closed (controller I2C timeout or a forced
		 * access serviced it): never address the device outside a window.
		 */
		iqs5xx_irq_arm(dev);
		k_mutex_unlock(&data->lock);
		return;
	}

	ts_us = (int64_t)k_ticks_to_us_floor64((uint64_t)data->rdy_ticks);
	iqs5xx_service(dev, ts_us, false, &nt);

	if (data->state == IQS5XX_STATE_UP) {
		iqs5xx_irq_arm(dev);
		iqs5xx_watchdog_kick(dev, false);
	}

	k_mutex_unlock(&data->lock);
	iqs5xx_notify_flush(dev, &nt);
}

static void iqs5xx_watchdog_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, watchdog);
	const struct device *dev = data->dev;
	struct iqs5xx_notify nt = {0};
	int64_t elapsed;

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((data->state != IQS5XX_STATE_UP) || data->wakeup_armed) {
		k_mutex_unlock(&data->lock);
		return;
	}

	elapsed = k_uptime_get() - data->last_rdy_ms;

	if (!data->event_mode) {
		if (elapsed >= CONFIG_AZOTEQ_IQS5XX_RDY_TIMEOUT_MS) {
			LOG_ERR("no RDY for %u ms in streaming mode", (uint32_t)elapsed);
			iqs5xx_start_recovery(dev, &nt);
		} else {
			(void)k_work_reschedule_for_queue(
				&iqs5xx_wq, &data->watchdog,
				K_MSEC(CONFIG_AZOTEQ_IQS5XX_RDY_TIMEOUT_MS - (int32_t)elapsed));
		}
	} else if (data->fingers_down) {
		if (elapsed >= CONFIG_AZOTEQ_IQS5XX_STALE_MS) {
			/* Event mode: a lost finger-up would leave a stuck
			 * contact. One forced read refreshes the state.
			 */
			LOG_DBG("stale contact after %u ms, forced read", (uint32_t)elapsed);
			iqs5xx_count(data, &data->stats.forced_reads);
			iqs5xx_forced_begin(dev);
			iqs5xx_service(dev, (int64_t)k_ticks_to_us_floor64(k_uptime_ticks()), true,
				       &nt);
			if (data->state == IQS5XX_STATE_UP) {
				iqs5xx_irq_arm(dev);
				iqs5xx_watchdog_kick(dev, true);
			}
		} else {
			(void)k_work_reschedule_for_queue(
				&iqs5xx_wq, &data->watchdog,
				K_MSEC(CONFIG_AZOTEQ_IQS5XX_STALE_MS - (int32_t)elapsed));
		}
	} else {
		/* Event mode without contacts: nothing to supervise. */
	}

	k_mutex_unlock(&data->lock);
	iqs5xx_notify_flush(dev, &nt);
}

static void iqs5xx_init_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, init_work);
	const struct device *dev = data->dev;
	struct iqs5xx_notify nt = {0};
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if ((data->state == IQS5XX_STATE_UP) || (data->state == IQS5XX_STATE_FATAL)) {
		/* iqs5xx_reinit() ran in the meantime. */
		k_mutex_unlock(&data->lock);
		return;
	}

	ret = iqs5xx_bringup(dev);
	if (ret == 0) {
		if (data->recovering) {
			iqs5xx_count(data, &data->stats.recoveries);
			LOG_INF("recovered");
			iqs5xx_notify_add(&nt, IQS5XX_EVT_RECOVERED);
		} else {
			LOG_INF("ready");
			iqs5xx_notify_add(&nt, IQS5XX_EVT_RESET);
		}
		data->recovering = false;
		data->init_attempt = 0;
	} else if (data->init_attempt < ARRAY_SIZE(iqs5xx_retry_delay_ms)) {
		uint16_t delay_ms = iqs5xx_retry_delay_ms[data->init_attempt];

		data->init_attempt++;
		LOG_WRN("bring-up failed (%d), retry %u in %u ms", ret, data->init_attempt,
			delay_ms);
		(void)k_work_reschedule_for_queue(&iqs5xx_wq, &data->init_work, K_MSEC(delay_ms));
	} else {
		LOG_ERR("bring-up failed (%d), giving up until iqs5xx_reinit()", ret);
		iqs5xx_set_state(data, IQS5XX_STATE_FATAL);
		data->recovering = false;
		data->init_attempt = 0;
		iqs5xx_notify_add(&nt, IQS5XX_EVT_FATAL);
	}

	k_mutex_unlock(&data->lock);
	iqs5xx_notify_flush(dev, &nt);
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

int iqs5xx_set_callback(const struct device *dev, iqs5xx_callback_t cb, void *user_data)
{
	struct iqs5xx_data *data;
	k_spinlock_key_t key;

	if (dev == NULL) {
		return -EINVAL;
	}
	data = dev->data;

	key = k_spin_lock(&data->slock);
	data->cb = cb;
	data->user_data = user_data;
	k_spin_unlock(&data->slock, key);
	return 0;
}

/* Always copies the last identification data (zeros before the first
 * successful bring-up). Returns 0 while the controller is up, -EAGAIN while
 * bring-up or recovery is pending, -EIO after a fatal failure.
 */
int iqs5xx_get_info(const struct device *dev, struct iqs5xx_info *info)
{
	struct iqs5xx_data *data;
	enum iqs5xx_state state;
	k_spinlock_key_t key;

	if ((dev == NULL) || (info == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	data = dev->data;

	key = k_spin_lock(&data->slock);
	*info = data->info;
	state = data->state;
	k_spin_unlock(&data->slock, key);

	switch (state) {
	case IQS5XX_STATE_UP:
		return 0;
	case IQS5XX_STATE_FATAL:
		return -EIO;
	default:
		return -EAGAIN;
	}
}

int iqs5xx_get_stats(const struct device *dev, struct iqs5xx_stats *stats)
{
	struct iqs5xx_data *data;
	k_spinlock_key_t key;

	if ((dev == NULL) || (stats == NULL)) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	data = dev->data;

	key = k_spin_lock(&data->slock);
	*stats = data->stats;
	k_spin_unlock(&data->slock, key);
	return 0;
}

int iqs5xx_reinit(const struct device *dev)
{
	struct iqs5xx_data *data;
	struct iqs5xx_notify nt = {0};
	struct k_work_sync sync;
	enum iqs5xx_state prev;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	data = dev->data;

	if (k_current_get() == k_work_queue_thread_get(&iqs5xx_wq)) {
		/* Called from a driver callback: nothing else runs on the queue. */
		(void)k_work_cancel_delayable(&data->init_work);
	} else {
		(void)k_work_cancel_delayable_sync(&data->init_work, &sync);
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	prev = data->state;
	iqs5xx_irq_disable(dev);
	iqs5xx_set_state(data, IQS5XX_STATE_PENDING);
	data->recovering = false;
	data->init_attempt = 0;
	/* No bus recovery here: see iqs5xx_start_recovery(). */
	LOG_DBG("re-init from state %d", (int)prev);

	ret = iqs5xx_bringup(dev);
	if (ret == 0) {
		LOG_INF("re-initialized");
		iqs5xx_notify_add(&nt, IQS5XX_EVT_RESET);
	} else {
		LOG_ERR("re-initialization failed (%d)", ret);
		iqs5xx_set_state(data, IQS5XX_STATE_FATAL);
		iqs5xx_notify_add(&nt, IQS5XX_EVT_FATAL);
	}

	k_mutex_unlock(&data->lock);
	iqs5xx_notify_flush(dev, &nt);
	return ret;
}

/* Forced access that applies SYS_CFG1 + report rates. Lock held. */
static int iqs5xx_apply_profile_forced(const struct device *dev)
{
	struct iqs5xx_data *data = dev->data;
	int ret, ret2;

	iqs5xx_forced_begin(dev);
	ret = iqs5xx_apply_sys_cfg1(dev, IQS5XX_FORCED_ATTEMPTS, NULL);
	if (ret == 0) {
		ret = iqs5xx_apply_rates(dev, IQS5XX_WINDOW_ATTEMPTS);
	}
	ret2 = iqs5xx_end_window(dev);
	k_busy_wait(IQS5XX_RDY_SETTLE_US);
	if (ret == 0) {
		ret = ret2;
	}
	if (ret < 0) {
		iqs5xx_count(data, &data->stats.i2c_errors);
		LOG_WRN("profile change failed (%d)", ret);
	}
	return ret;
}

int iqs5xx_set_low_power(const struct device *dev, bool low_power)
{
	const struct iqs5xx_config *cfg;
	struct iqs5xx_data *data;
	int ret = 0;

	if (dev == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	cfg = dev->config;
	data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	data->low_power = low_power;
	data->event_mode = low_power ? true : cfg->event_mode;
	data->wakeup_armed = false;

	if (data->state == IQS5XX_STATE_UP) {
		/* Otherwise the profile is applied by the next (re)init. */
		ret = iqs5xx_apply_profile_forced(dev);
		data->last_rdy_ms = k_uptime_get();
		iqs5xx_irq_arm(dev);
		if (data->event_mode) {
			(void)k_work_cancel_delayable(&data->watchdog);
		}
		iqs5xx_watchdog_kick(dev, true);
		LOG_INF("%s profile (%s mode)", low_power ? "low-power" : "normal",
			data->event_mode ? "event" : "streaming");
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

int iqs5xx_prepare_wakeup(const struct device *dev)
{
	struct iqs5xx_data *data;
	int ret = -EIO;

	if (dev == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	(void)k_work_cancel_delayable(&data->watchdog);
	data->low_power = true;
	data->event_mode = true;

	if (data->state == IQS5XX_STATE_UP) {
		ret = iqs5xx_apply_profile_forced(dev);
	} else {
		LOG_WRN("controller not initialized, touch wake-up unavailable");
	}

	/* No watchdog traffic from now on. A level-high RDY interrupt keeps the
	 * nRF GPIO SENSE configured, which is what wakes the SoC from System
	 * OFF; windows opened before power-off are still serviced and re-armed.
	 */
	data->wakeup_armed = true;
	iqs5xx_irq_arm(dev);

	k_mutex_unlock(&data->lock);
	return ret;
}

int iqs5xx_arm_wake_pin(const struct device *dev)
{
	const struct iqs5xx_config *cfg;
	const struct iqs5xx_data *data;

	if (dev == NULL) {
		return -EINVAL;
	}
	cfg = dev->config;
	data = dev->data;

	/* Runs with interrupts locked right before sys_poweroff(): no mutex,
	 * no sleeping. The RDY ISR masks the level sense while a window is
	 * serviced, and the work handler that re-arms it may not have run
	 * yet; re-apply it unconditionally. If RDY is already high, the SoC
	 * wakes from System OFF at once, which is the correct outcome.
	 */
	if (!data->wakeup_armed) {
		return -EAGAIN;
	}
	return gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_LEVEL_ACTIVE);
}

/* ------------------------------------------------------------------------ */
/* Device init                                                              */
/* ------------------------------------------------------------------------ */

static int iqs5xx_init(const struct device *dev)
{
	const struct iqs5xx_config *cfg = dev->config;
	struct iqs5xx_data *data = dev->data;
	int ret;

	data->dev = dev;
	data->event_mode = cfg->event_mode;
	data->state = IQS5XX_STATE_PENDING;
	k_mutex_init(&data->lock);
	k_work_init(&data->rdy_work, iqs5xx_rdy_work_handler);
	k_work_init_delayable(&data->init_work, iqs5xx_init_work_handler);
	k_work_init_delayable(&data->watchdog, iqs5xx_watchdog_handler);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&cfg->rdy)) {
		LOG_ERR("RDY GPIO not ready");
		return -ENODEV;
	}

	/* Hold the controller in reset until the bring-up work runs: it then
	 * cannot touch the shared bus while other drivers initialize.
	 */
	if (cfg->reset.port != NULL) {
		if (!gpio_is_ready_dt(&cfg->reset)) {
			LOG_ERR("reset GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("cannot configure reset GPIO (%d)", ret);
			return ret;
		}
	}

	/* The only explicit bus recovery: the controller is held in reset and
	 * the other drivers on the shared bus (IMU, priority SENSOR_INIT_PRIORITY)
	 * have not probed yet, so nothing can be disturbed. After a warm reset
	 * in the middle of a transfer a slave may still hold SDA low. */
	ret = i2c_recover_bus(cfg->i2c.bus);
	if ((ret < 0) && (ret != -ENOSYS)) {
		LOG_WRN("I2C bus recovery failed (%d)", ret);
	}

	ret = gpio_pin_configure_dt(&cfg->rdy, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("cannot configure RDY GPIO (%d)", ret);
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}
	gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_isr, BIT(cfg->rdy.pin));
	ret = gpio_add_callback_dt(&cfg->rdy, &data->rdy_cb);
	if (ret < 0) {
		LOG_ERR("cannot add RDY callback (%d)", ret);
		return ret;
	}

	if (!iqs5xx_wq_started) {
		const struct k_work_queue_config wq_cfg = {
			.name = "iqs5xx_wq",
		};

		k_work_queue_init(&iqs5xx_wq);
		k_work_queue_start(&iqs5xx_wq, iqs5xx_wq_stack,
				   K_THREAD_STACK_SIZEOF(iqs5xx_wq_stack),
				   CONFIG_AZOTEQ_IQS5XX_WORKQ_PRIORITY, &wq_cfg);
		iqs5xx_wq_started = true;
	}

	/* Controller bring-up (reset, identify, configure) runs asynchronously;
	 * its outcome is reported as IQS5XX_EVT_RESET or IQS5XX_EVT_FATAL.
	 */
	(void)k_work_schedule_for_queue(&iqs5xx_wq, &data->init_work, K_NO_WAIT);
	return 0;
}

#define IQS5XX_DEFINE(inst)                                                                        \
	BUILD_ASSERT(DT_INST_PROP(inst, active_report_rate_ms) <= UINT16_MAX,                      \
		     "active-report-rate-ms out of range");                                        \
	static const struct iqs5xx_config iqs5xx_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.rdy = GPIO_DT_SPEC_INST_GET(inst, rdy_gpios),                                     \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.active_rr_ms = DT_INST_PROP(inst, active_report_rate_ms),                         \
		.event_mode = DT_INST_PROP(inst, event_mode),                                      \
	};                                                                                         \
	static struct iqs5xx_data iqs5xx_data_##inst;                                              \
	DEVICE_DT_INST_DEFINE(inst, iqs5xx_init, NULL, &iqs5xx_data_##inst,                        \
			      &iqs5xx_config_##inst, POST_KERNEL,                                  \
			      CONFIG_AZOTEQ_IQS5XX_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_DEFINE)
