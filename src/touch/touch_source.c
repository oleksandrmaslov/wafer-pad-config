/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Touch source: wraps the Azoteq IQS5xx driver (DT alias touch0).
 *
 * Driver frames (sensor frame S, one slot per tracked finger) are turned into
 * normalized wp_touch_frame snapshots, corrected for the mechanical mounting
 * (S -> P, wp_touch_mount_apply) and posted to the event router. Frames
 * flagged PALM_DETECT / TOO_MANY_FINGERS are posted with zero contacts and
 * slots with 0xFFFF coordinates are dropped (wp_touch_from_iqs5xx()). Driver
 * events are mapped to enum wp_touch_status.
 *
 * The controller reset + initialization itself is started by the driver at
 * boot (asynchronously, on its own work queue); wp_touch_source_init() binds
 * to it and reports the current state, so no event is lost whichever of the
 * two finishes first.
 *
 * Coordinate range: the resolution registers (0x066E/0x0670) are used as the
 * INCLUSIVE maximum (as the Linux iqs5xx driver does with ABS_MT_POSITION_*);
 * coordinates are clamped to it, so 0..res is always a valid range.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <drivers/azoteq_iqs5xx.h>
#include <wafer_pad/config.h>
#include <wafer_pad/platform.h>
#include <wafer_pad/touch_iqs5xx.h>
#include <wafer_pad/touch_tracker.h>

LOG_MODULE_REGISTER(wp_touch, CONFIG_WAFER_PAD_LOG_LEVEL);

#define WP_TOUCH_NODE DT_ALIAS(touch0)

#if DT_NODE_HAS_STATUS_OKAY(WP_TOUCH_NODE) && DT_NODE_HAS_COMPAT(WP_TOUCH_NODE, azoteq_iqs5xx) && \
	defined(CONFIG_AZOTEQ_IQS5XX)
#define WP_TOUCH_HAVE_DEV 1
#else
#define WP_TOUCH_HAVE_DEV 0
#endif

#if WP_TOUCH_HAVE_DEV

/* Azoteq TPS65 default output resolution (module datasheet, 3072 x 2048),
 * used only until the controller has reported its real resolution.
 */
#define WP_TOUCH_TPS65_MAX_X 3072U
#define WP_TOUCH_TPS65_MAX_Y 2048U

static const struct device *const touch_dev = DEVICE_DT_GET(WP_TOUCH_NODE);

static struct wp_touch_mount mount;
static float sensor_w_mm;
static float sensor_h_mm;
static bool config_loaded;

/* Sensor-frame extent packed as (max_x << 16) | max_y: written from the
 * driver work queue, read by the frame path and get_geometry().
 */
static atomic_t sensor_extent = ATOMIC_INIT((WP_TOUCH_TPS65_MAX_X << 16) | WP_TOUCH_TPS65_MAX_Y);
static atomic_t post_drops;

static void load_config(void)
{
	if (!config_loaded) {
		wp_config_touch_mount(&mount);
		wp_config_touch_size_mm(&sensor_w_mm, &sensor_h_mm);
		config_loaded = true;
	}
}

/* Query the controller resolution; fall back to the TPS65 defaults while it
 * is not known yet. Returns true if the real resolution was used.
 */
static bool refresh_extent(uint16_t *max_x, uint16_t *max_y)
{
	struct iqs5xx_info info = {0};
	uint16_t x = WP_TOUCH_TPS65_MAX_X;
	uint16_t y = WP_TOUCH_TPS65_MAX_Y;
	bool real = false;

	(void)iqs5xx_get_info(touch_dev, &info);
	if ((info.resolution_x != 0U) && (info.resolution_y != 0U)) {
		x = info.resolution_x;
		y = info.resolution_y;
		real = true;
	}

	(void)atomic_set(&sensor_extent, (atomic_val_t)(((uint32_t)x << 16) | y));
	if (max_x != NULL) {
		*max_x = x;
	}
	if (max_y != NULL) {
		*max_y = y;
	}
	return real;
}

static void post_frame(const struct iqs5xx_frame *fr)
{
	struct wp_touch_frame f;
	uint32_t ext = (uint32_t)atomic_get(&sensor_extent);
	int ret;

	/* Palm / too-many-fingers frames carry zero contacts; slots with 0xFFFF
	 * coordinates are dropped (see <wafer_pad/touch_iqs5xx.h>). */
	(void)wp_touch_from_iqs5xx(fr, (uint16_t)(ext >> 16), (uint16_t)(ext & 0xFFFFU), &f);

	wp_touch_mount_apply(&mount, &f, &f);

	ret = wp_router_post_touch(&f);
	if (ret < 0) {
		atomic_val_t n = atomic_inc(&post_drops) + 1;

		/* Log the first drop and then every 256th. */
		if ((n & 0xFF) == 1) {
			LOG_WRN("router queue full, %u touch frames dropped", (uint32_t)n);
		}
	}
}

static void touch_event_cb(const struct device *dev, enum iqs5xx_event evt,
			   const struct iqs5xx_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt) {
	case IQS5XX_EVT_FRAME:
		if (frame != NULL) {
			post_frame(frame);
		}
		break;
	case IQS5XX_EVT_RESET:
		/* Contacts are lost; the controller is configured again. */
		(void)refresh_extent(NULL, NULL);
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_RESET);
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_OK);
		break;
	case IQS5XX_EVT_ERROR:
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_ERROR);
		break;
	case IQS5XX_EVT_RECOVERED:
		(void)refresh_extent(NULL, NULL);
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_OK);
		break;
	case IQS5XX_EVT_FATAL:
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_FATAL);
		break;
	default:
		break;
	}
}

int wp_touch_source_init(void)
{
	struct iqs5xx_info info = {0};
	int ret;

	load_config();

	if (!device_is_ready(touch_dev)) {
		LOG_ERR("touch controller %s not ready", touch_dev->name);
		return -ENODEV;
	}

	ret = iqs5xx_set_callback(touch_dev, touch_event_cb, NULL);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("touch source on %s (swap_xy %d invert_x %d invert_y %d, %u.%02u x %u.%02u mm)",
		touch_dev->name, mount.swap_xy, mount.invert_x, mount.invert_y,
		CONFIG_WAFER_PAD_TOUCH_WIDTH_CENTI_MM / 100,
		CONFIG_WAFER_PAD_TOUCH_WIDTH_CENTI_MM % 100,
		CONFIG_WAFER_PAD_TOUCH_HEIGHT_CENTI_MM / 100,
		CONFIG_WAFER_PAD_TOUCH_HEIGHT_CENTI_MM % 100);

	/* The driver brings the controller up asynchronously from boot. If that
	 * already finished before the callback was registered, report the
	 * outcome now; otherwise EVT_RESET / EVT_FATAL follows.
	 */
	ret = iqs5xx_get_info(touch_dev, &info);
	if (ret == 0) {
		(void)refresh_extent(NULL, NULL);
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_OK);
	} else if (ret == -EIO) {
		(void)wp_router_post_touch_status(WP_TOUCH_STATUS_FATAL);
	} else {
		LOG_DBG("controller bring-up still running (%d)", ret);
	}

	return 0;
}

int wp_touch_source_get_geometry(struct wp_pad_geometry *pad_geom)
{
	uint16_t max_x, max_y;

	if (pad_geom == NULL) {
		return -EINVAL;
	}
	if (!device_is_ready(touch_dev)) {
		return -ENODEV;
	}

	load_config();

	if (!refresh_extent(&max_x, &max_y)) {
		LOG_INF("controller not up yet, using TPS65 default %ux%u", max_x, max_y);
	}

	/* Sensor -> pad frame: a swap exchanges both the extents and the
	 * physical sizes; inversions change neither.
	 */
	if (mount.swap_xy) {
		pad_geom->max_x = max_y;
		pad_geom->max_y = max_x;
		pad_geom->width_mm = sensor_h_mm;
		pad_geom->height_mm = sensor_w_mm;
	} else {
		pad_geom->max_x = max_x;
		pad_geom->max_y = max_y;
		pad_geom->width_mm = sensor_w_mm;
		pad_geom->height_mm = sensor_h_mm;
	}
	return 0;
}

int wp_touch_source_set_low_power(bool low_power)
{
	if (!device_is_ready(touch_dev)) {
		return -ENODEV;
	}
	return iqs5xx_set_low_power(touch_dev, low_power);
}

int wp_touch_source_reinit(void)
{
	if (!device_is_ready(touch_dev)) {
		return -ENODEV;
	}
	return iqs5xx_reinit(touch_dev);
}

int wp_touch_source_prepare_wakeup(void)
{
	if (!device_is_ready(touch_dev)) {
		return -ENODEV;
	}
	return iqs5xx_prepare_wakeup(touch_dev);
}

int wp_touch_source_arm_wake_pin(void)
{
	/* Interrupts locked: no device_is_ready() bookkeeping beyond a read. */
	if (!device_is_ready(touch_dev)) {
		return -ENODEV;
	}
	return iqs5xx_arm_wake_pin(touch_dev);
}

#else /* !WP_TOUCH_HAVE_DEV */

int wp_touch_source_init(void)
{
	LOG_WRN("no touch0 (azoteq,iqs5xx) device, touch disabled");
	return -ENODEV;
}

int wp_touch_source_get_geometry(struct wp_pad_geometry *pad_geom)
{
	ARG_UNUSED(pad_geom);
	return -ENODEV;
}

int wp_touch_source_set_low_power(bool low_power)
{
	ARG_UNUSED(low_power);
	return -ENODEV;
}

int wp_touch_source_reinit(void)
{
	return -ENODEV;
}

int wp_touch_source_prepare_wakeup(void)
{
	return -ENODEV;
}

int wp_touch_source_arm_wake_pin(void)
{
	return -ENODEV;
}

#endif /* WP_TOUCH_HAVE_DEV */
