# storage_erase

A small firmware for the nice!nano v2 that wipes the settings partition before Wafer Pad is installed. Use it when the board ran another firmware before (ZMK, your own Zephyr app, ...).

## Why

The UF2 bootloader only overwrites the application area. The settings partition at 0xEC000–0xF4000 survives every UF2 flash. ZMK and other Zephyr firmware keep their settings and Bluetooth bonds there, at the same address Wafer Pad uses. Without an erase, Wafer Pad would load the old firmware's bonds as its own. Pairing then misbehaves: the "no bond yet, pairing always open" case never happens.

## What it touches

| Range | Content | Action |
|---|---|---|
| 0x00000–0x26000 | MBR + SoftDevice S140 6.1.1 | read only (checksummed before and after) |
| 0x26000–0xEA000 | application | this tool runs here; the next UF2 overwrites it |
| 0xEC000–0xF4000 | settings / bonds (32 KB) | **erased**, then verified blank |
| 0xF4000–0x100000 | UF2 bootloader + its settings | read only (checksummed before and after) |

Nothing in UICR is written either: this tool's overlay removes `gpio-as-nreset`, so Zephyr's startup code never programs UICR PSELRESET. The bootloader has already set it, so pin reset keeps working.

Safety checks:
* **Build time:** the partition addresses are checked with `BUILD_ASSERT`.
* **Run time, before erasing:**
  * The MBR must start the bootloader at 0xF4000 (flash word 0xFF8, or UICR NRFFW[0]).
  * The SoftDevice size must be 0x26000.
  * The opened partition must be 0xEC000 / 32 KB.
* **On any mismatch** the tool refuses and erases nothing.

A partition that is already blank is not erased again, so repeated runs cause no flash wear.

## Build

From the workspace directory (`wafer-ws`):

```sh
west build -p always -b nice_nano_v2 -d build-erase wafer-pad/tools/storage_erase
```

Output: `build-erase/zephyr/zephyr.uf2`.

## Use

Keep the board on a USB data connection to a PC.

1. Double-tap RST. The `NICENANO` drive appears.
2. Copy `build-erase/zephyr/zephyr.uf2` onto it.
3. The tool erases at once and keeps the LED on for 10 s. It then reboots into the bootloader by itself, and the `NICENANO` drive appears again.
4. Copy the Wafer Pad `zephyr.uf2` onto it **within about 5 minutes**.

Bootloader 0.10.0 leaves UF2 mode after 6 minutes and starts the installed app. If that happens while a copy is running, the half-written app starts. Double-tap RST and copy again.

Optional: open the USB console during those 10 s (`tio /dev/ttyACM0`). It prints the result:

```
storage erase OK
  erased 0xEC000-0xF4000 (32 KB)
  N of 8 pages had data, now blank
  unchanged: SoftDevice crc32 0x........
  unchanged: bootloader crc32 0x........
  UF2 bootloader in N s
```

If a check or the erase fails, the LED blinks fast. The console then repeats the failed step, the error code, and the bootloader address and SoftDevice size it found. The board stays in the tool; double-tap RST to get back to the bootloader.

If the tool stays installed, every reset runs it again: it finds the partition blank, erases nothing, and returns to the bootloader. Without USB power (battery only) it does not reboot; it stays in the tool with the LED off, so it can't loop on battery. Flash Wafer Pad to end this.

## Afterwards

* On the host PC, remove the old device's pairing. The new firmware uses the chip's factory Bluetooth address, which is probably the same one the old firmware used. The host would then try to reconnect with keys the pad no longer has.
* The erase cannot be undone. A UF2 backup (`CURRENT.UF2` from the drive) contains the old application but not its settings: the bootloader only exports 0x01000–0xEA000.
