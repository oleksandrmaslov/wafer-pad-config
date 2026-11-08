/*
 * Copyright (c) 2026 Wafer Pad contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Storage erase tool for the nice!nano v2 (see ../README.md).
 *
 * Erases the settings partition (0xEC000-0xF4000, 32 KB) at boot if it holds
 * any data. That partition holds Zephyr settings: the BLE bonds and whatever
 * the previous firmware stored there (ZMK and other Zephyr firmware use the
 * same partition on the nice!nano). Wafer Pad keeps its own bonds and gyro
 * bias there, so leftovers from another firmware would be loaded as if they
 * were its own.
 *
 * Nothing else in flash or UICR is written (this tool's overlay also removes
 * gpio-as-nreset, so the startup code never programs UICR). Before erasing, the
 * tool checks that the MBR starts the bootloader at 0xF4000 and that the
 * SoftDevice ends at 0x26000; otherwise it refuses. The MBR + SoftDevice
 * (0x00000-0x26000) and the bootloader with its settings (0xF4000-0x100000)
 * are checksummed before and after to show that they did not change.
 *
 *   success: LED on, report on the USB console for 10 s, then reboot into the
 *            UF2 bootloader (the NICENANO drive appears again). Without USB
 *            power it stays in this tool instead (no reboot loop on battery).
 *   failure: LED blinks fast, the error repeats on the USB console, and the
 *            board stays in this tool (double-tap RST for the bootloader).
 */

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include <soc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_io.h>

/* nice!nano layout: Adafruit nRF52 bootloader with SoftDevice S140 v6.1.1. */
#define SD_START         0x00000U
#define SD_END           0x26000U
#define STORAGE_START    0xEC000U
#define STORAGE_SIZE     0x08000U
#define BOOTLOADER_START 0xF4000U
#define FLASH_END        0x100000U

#define PAGE_SIZE  DT_PROP(DT_CHOSEN(zephyr_flash), erase_block_size)
#define PAGE_COUNT (STORAGE_SIZE / PAGE_SIZE)

/* S140 6.1.1 nrf_mbr.h / nrf_sdm.h: the MBR reads the bootloader start from
 * flash word 0xFF8, or from UICR NRFFW[0] if that word is blank. The SoftDevice
 * size is at SD_SIZE_GET(MBR_SIZE) = 0x1000 + 0x2000 + 0x08.
 */
#define MBR_BOOTLOADER_ADDR 0xFF8U
#define SD_SIZE_ADDR        0x3008U

/* GPREGRET value that makes the bootloader start in UF2 mode
 * (Adafruit_nRF52_Bootloader src/main.c, DFU_MAGIC_UF2_RESET).
 */
#define UF2_DFU_MAGIC 0x57U

#define REPORT_SECONDS 10
#define CHUNK          256U

/* Refuse to build for any other layout: the erase must never reach the
 * application, the SoftDevice or the bootloader.
 */
BUILD_ASSERT(PARTITION_EXISTS(storage_partition), "no storage_partition in the devicetree");
BUILD_ASSERT(PARTITION_OFFSET(storage_partition) == STORAGE_START,
	     "storage_partition is not at 0xEC000");
BUILD_ASSERT(PARTITION_SIZE(storage_partition) == STORAGE_SIZE,
	     "storage_partition is not 32 KB");
BUILD_ASSERT(PARTITION_OFFSET(boot_partition) == BOOTLOADER_START,
	     "boot_partition is not at 0xF4000");
BUILD_ASSERT(PARTITION_OFFSET(code_partition) >= SD_END, "code_partition overlaps the SoftDevice");
BUILD_ASSERT(PARTITION_OFFSET(code_partition) + PARTITION_SIZE(code_partition) <= STORAGE_START,
	     "code_partition overlaps storage_partition");
BUILD_ASSERT(STORAGE_START + STORAGE_SIZE <= BOOTLOADER_START,
	     "storage_partition overlaps the bootloader");
BUILD_ASSERT(STORAGE_START % PAGE_SIZE == 0 && STORAGE_SIZE % PAGE_SIZE == 0,
	     "storage_partition is not page aligned");
BUILD_ASSERT(PAGE_SIZE % CHUNK == 0 && (SD_END - SD_START) % CHUNK == 0 &&
		     (FLASH_END - BOOTLOADER_START) % CHUNK == 0,
	     "ranges must be multiples of the read chunk");

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static bool led_ok;

static uint8_t buf[CHUNK];

static void led_set(bool on)
{
	if (led_ok) {
		(void)gpio_pin_set_dt(&led, on ? 1 : 0);
	}
}

static void blink_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (led_ok) {
		(void)gpio_pin_toggle_dt(&led);
	}
}

K_TIMER_DEFINE(blink_timer, blink_handler, NULL);

/* The CDC ACM console takes about 64 bytes per USB transfer and drops the
 * rest (no flow control), so the report goes out one short line at a time.
 */
static void line(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);
	k_msleep(5);
}

/* Checksum of a memory-mapped flash range (read only). The SoftDevice range
 * starts at address 0, so the flash is read through sys_read32() (volatile
 * word reads from an integer address) instead of a C pointer to 0.
 */
static uint32_t flash_crc(uint32_t start, uint32_t end)
{
	uint32_t crc = 0;
	uint32_t words[CHUNK / sizeof(uint32_t)];

	for (uint32_t addr = start; addr < end; addr += sizeof(words)) {
		for (size_t i = 0; i < ARRAY_SIZE(words); i++) {
			words[i] = sys_read32(addr + i * sizeof(uint32_t));
		}
		crc = crc32_ieee_update(crc, (const uint8_t *)words, sizeof(words));
	}
	return crc;
}

/* Bootloader start address as the MBR sees it. */
static uint32_t mbr_bootloader_addr(void)
{
	uint32_t addr = sys_read32(MBR_BOOTLOADER_ADDR);

	return addr != UINT32_MAX ? addr : NRF_UICR->NRFFW[0];
}

/* Number of pages in the area that are not fully erased, or a negative errno. */
static int count_used_pages(const struct flash_area *fa)
{
	const uint8_t erased = flash_area_erased_val(fa);
	int used = 0;

	for (size_t page = 0; page < fa->fa_size; page += PAGE_SIZE) {
		bool blank = true;

		for (size_t off = page; blank && off < page + PAGE_SIZE; off += sizeof(buf)) {
			int err = flash_area_read(fa, (off_t)off, buf, sizeof(buf));

			if (err != 0) {
				return err;
			}
			for (size_t i = 0; i < sizeof(buf); i++) {
				if (buf[i] != erased) {
					blank = false;
					break;
				}
			}
		}
		if (!blank) {
			used++;
		}
	}
	return used;
}

int main(void)
{
	const struct flash_area *fa = NULL;
	const char *step;
	uint32_t bl_addr;
	uint32_t sd_size;
	uint32_t sd_crc;
	uint32_t bl_crc;
	bool vbus;
	int used_before = 0;
	int used_after;
	int err = 0;

	if (led.port != NULL && gpio_is_ready_dt(&led) &&
	    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) == 0) {
		led_ok = true;
	}

	bl_addr = mbr_bootloader_addr();
	sd_size = sys_read32(SD_SIZE_ADDR);
	sd_crc = flash_crc(SD_START, SD_END);
	bl_crc = flash_crc(BOOTLOADER_START, FLASH_END);

	step = "MBR bootloader address";
	if (bl_addr != BOOTLOADER_START) {
		err = -EFAULT;
	}
	if (err == 0) {
		step = "SoftDevice size";
		if (sd_size != SD_END) {
			err = -EFAULT;
		}
	}
	if (err == 0) {
		step = "open storage_partition";
		err = flash_area_open(PARTITION_ID(storage_partition), &fa);
	}
	if (err == 0) {
		step = "partition bounds";
		if ((uint32_t)fa->fa_off != STORAGE_START || fa->fa_size != STORAGE_SIZE) {
			err = -EFAULT;
		}
	}
	if (err == 0) {
		step = "read before erase";
		used_before = count_used_pages(fa);
		if (used_before < 0) {
			err = used_before;
		}
	}
	if (err == 0 && used_before > 0) {
		/* Already blank: skip, so repeated runs cause no flash wear. */
		step = "erase";
		led_set(true);
		err = flash_area_erase(fa, 0, STORAGE_SIZE);
	}
	if (err == 0) {
		step = "verify blank";
		used_after = count_used_pages(fa);
		if (used_after != 0) {
			err = used_after < 0 ? used_after : -EIO;
		}
	}
	if (err == 0) {
		step = "SoftDevice/bootloader crc";
		if (flash_crc(SD_START, SD_END) != sd_crc ||
		    flash_crc(BOOTLOADER_START, FLASH_END) != bl_crc) {
			err = -EIO;
		}
	}

	if (err != 0) {
		k_timer_start(&blink_timer, K_MSEC(100), K_MSEC(100));
		for (;;) {
			line("storage erase FAILED\n");
			line("  step  %s\n", step);
			line("  error %d\n", err);
			line("  bootloader at 0x%08X\n", bl_addr);
			line("  SoftDevice size 0x%08X\n", sd_size);
			line("double-tap RST for the bootloader\n\n");
			k_sleep(K_SECONDS(2));
		}
	}

	led_set(true);
	vbus = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0U;

	for (int s = REPORT_SECONDS; s > 0; s--) {
		line("storage erase OK\n");
		if (used_before > 0) {
			line("  erased 0x%05X-0x%05X (%u KB)\n", STORAGE_START,
			     STORAGE_START + STORAGE_SIZE, STORAGE_SIZE / 1024U);
			line("  %d of %u pages had data, now blank\n", used_before,
			     (unsigned int)PAGE_COUNT);
		} else {
			line("  0x%05X-0x%05X already blank\n", STORAGE_START,
			     STORAGE_START + STORAGE_SIZE);
		}
		line("  unchanged: SoftDevice crc32 0x%08X\n", sd_crc);
		line("  unchanged: bootloader crc32 0x%08X\n", bl_crc);
		if (vbus) {
			line("  UF2 bootloader in %d s\n\n", s);
		} else {
			line("  no USB power: staying here\n\n");
		}
		k_sleep(K_SECONDS(1));
	}

	flash_area_close(fa);

	if (!vbus) {
		/* On battery the bootloader would time out after 3 s and start
		 * this tool again, forever. Stay here instead.
		 */
		led_set(false);
		for (;;) {
			k_sleep(K_FOREVER);
		}
	}

	NRF_POWER->GPREGRET = UF2_DFU_MAGIC;
	sys_reboot(SYS_REBOOT_COLD);

	return 0;
}
