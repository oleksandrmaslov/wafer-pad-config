/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Azoteq IQS5xx-B000 register map (subset used by the driver).
 *
 * Written from the IQS5xx-B000 datasheet (Rev 2.1) facts collected in the
 * project research notes (registers §4, bits §4.3, frame layout §6.1).
 * All registers use 16-bit big-endian addresses; multi-byte values are
 * big-endian (high byte at the lower address).
 */

#ifndef WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_REGS_H_
#define WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_REGS_H_

#include <zephyr/sys/util_macro.h>

/* ---- Read-only status block ------------------------------------------- */
#define IQS5XX_REG_PROD_NUM        0x0000 /* u16: 40 IQS550, 58 IQS572, 52 IQS525 */
#define IQS5XX_REG_PROJ_NUM        0x0002 /* u16: 15 for generic B000 */
#define IQS5XX_REG_VER_MAJOR       0x0004
#define IQS5XX_REG_VER_MINOR       0x0005
#define IQS5XX_REG_BL_STATUS       0x0006 /* 0xA5 bootloader present, 0xEE none */
#define IQS5XX_ID_LEN              7      /* 0x0000..0x0006 */

#define IQS5XX_REG_PREV_CYCLE_TIME 0x000C
#define IQS5XX_REG_SYS_INFO0       0x000F

/* One burst 0x000C..0x0038 (inclusive) carries a complete coherent frame. */
#define IQS5XX_FRAME_START         0x000C
#define IQS5XX_FRAME_LEN           45

/* Offsets inside the frame burst (relative to 0x000C). */
#define IQS5XX_FRM_PREV_CYCLE      0
#define IQS5XX_FRM_GESTURE0        1
#define IQS5XX_FRM_GESTURE1        2
#define IQS5XX_FRM_SYS_INFO0       3
#define IQS5XX_FRM_SYS_INFO1       4
#define IQS5XX_FRM_NUM_FINGERS     5
#define IQS5XX_FRM_REL_X           6  /* s16 BE */
#define IQS5XX_FRM_REL_Y           8  /* s16 BE */
#define IQS5XX_FRM_FINGER0         10 /* + 7 * i: absX u16, absY u16, strength u16, area u8 */
#define IQS5XX_FRM_FINGER_STRIDE   7

/* ---- Control / configuration (volatile at runtime, NV defaults at reset) */
#define IQS5XX_REG_SYS_CTRL0       0x0431
#define IQS5XX_REG_SYS_CTRL1       0x0432
#define IQS5XX_REG_RR_ACTIVE       0x057A /* u16 ms; +2 IdleTouch, +4 Idle, +6 LP1, +8 LP2 */
#define IQS5XX_RR_COUNT            5
#define IQS5XX_REG_I2C_TIMEOUT     0x058A /* u8 ms */
#define IQS5XX_REG_SYS_CFG0        0x058E
#define IQS5XX_REG_SYS_CFG1        0x058F
#define IQS5XX_REG_XY_CFG0         0x0669
#define IQS5XX_REG_MAX_MULTI_TOUCH 0x066A /* u8, <= 5; own register (0x066B is
					   * the finger split factor). Notes §4.2 */
#define IQS5XX_REG_X_RES           0x066E /* u16 */
#define IQS5XX_REG_Y_RES           0x0670 /* u16 */
#define IQS5XX_REG_SF_GESTURES     0x06B7 /* single-finger gesture enables */
#define IQS5XX_REG_MF_GESTURES     0x06B8 /* multi-finger gesture enables */
#define IQS5XX_REG_END_WINDOW      0xEEEE /* write 1 byte (any value) + STOP */

/* ---- System Info 0 (0x000F) ------------------------------------------- */
#define IQS5XX_SHOW_RESET          BIT(7)
#define IQS5XX_ALP_REATI_OCC       BIT(6)
#define IQS5XX_ALP_ATI_ERROR       BIT(5)
#define IQS5XX_REATI_OCC           BIT(4)
#define IQS5XX_ATI_ERROR           BIT(3)
#define IQS5XX_CHARGING_MODE_MASK  0x07

/* ---- System Info 1 (0x0010) ------------------------------------------- */
#define IQS5XX_RR_MISSED           BIT(3)
#define IQS5XX_TOO_MANY_FINGERS    BIT(2)
#define IQS5XX_PALM_DETECT         BIT(1)
#define IQS5XX_TP_MOVEMENT         BIT(0)

/* ---- System Control 0 (0x0431) ---------------------------------------- */
#define IQS5XX_ACK_RESET           BIT(7)

/* ---- System Control 1 (0x0432) ---------------------------------------- */
#define IQS5XX_SW_RESET            BIT(1) /* takes effect after the window ends */
#define IQS5XX_SUSPEND             BIT(0)

/* ---- System Config 0 (0x058E) ----------------------------------------- */
#define IQS5XX_MANUAL_CONTROL      BIT(7)
#define IQS5XX_SETUP_COMPLETE      BIT(6)
#define IQS5XX_WDT                 BIT(5)
#define IQS5XX_ALP_REATI           BIT(3)
#define IQS5XX_REATI               BIT(2)

/* ---- System Config 1 (0x058F) ----------------------------------------- */
#define IQS5XX_TP_EVENT            BIT(2)
#define IQS5XX_GESTURE_EVENT       BIT(1)
#define IQS5XX_EVENT_MODE          BIT(0)

/* ---- XY Config 0 (0x0669), read for diagnostics only (factory value kept) */
#define IQS5XX_PALM_REJECT         BIT(3)
#define IQS5XX_SWITCH_XY_AXIS      BIT(2)
#define IQS5XX_FLIP_Y              BIT(1)
#define IQS5XX_FLIP_X              BIT(0)

/* ---- Timing (datasheet / reference drivers, see research notes §5) ----- */
#define IQS5XX_RESET_PULSE_MS      1   /* DS: >= 300 ns guaranteed reset */
#define IQS5XX_HW_RESET_WAIT_MS    20  /* 2 ms bootloader window + app start */
#define IQS5XX_SW_RESET_WAIT_MS    100 /* QMK waits 100 ms after a SW reset */
#define IQS5XX_FIRST_RDY_WAIT_MS   200 /* poll RDY high before the first access */
#define IQS5XX_ATI_SETTLE_MS       250 /* no forced comms after SETUP_COMPLETE */
#define IQS5XX_RETRY_DELAY_US      250 /* DS: retry >= 150 us after a NACK */
#define IQS5XX_FORCED_ATTEMPTS     10  /* first addressing may NACK in low power */
#define IQS5XX_WINDOW_ATTEMPTS     3   /* retries inside an open RDY window */
#define IQS5XX_RDY_SETTLE_US       100 /* let RDY drop before re-arming level IRQ */

/* ---- Low-power report-rate profile (ms). Azoteq TPS65 examples: Idle-Touch
 * 50-100, Idle 75, LP1 75-80, LP2 160-200 (EV02 Table 2.1, AZD087 §6.3.1).
 * The driver never makes a rate FASTER than the controller's NV default.
 */
#define IQS5XX_LP_RR_ACTIVE_MS     20
#define IQS5XX_LP_RR_IDLE_TOUCH_MS 100
#define IQS5XX_LP_RR_IDLE_MS       80
#define IQS5XX_LP_RR_LP1_MS        80
#define IQS5XX_LP_RR_LP2_MS        160

#endif /* WAFER_PAD_DRIVERS_AZOTEQ_IQS5XX_REGS_H_ */
