# Wafer Pad wiring (prototype v0.1)

Status: pin assignments and electrical facts below were checked against the
documents listed under **Sources**. **Nothing here has been verified on a
physical prototype yet.** Items marked **VERIFY ON HARDWARE** must be checked
with a multimeter, a logic analyser or `i2c scan` before the first power-up.

## 1. Connection table

| Signal | nice!nano v2 pin | nRF52840 GPIO | TPS65 (J1) | LSM6DSOX breakout |
|---|---|---|---|---|
| I²C SDA | D2 | P0.17 | pin 6 `SDA` | SDA |
| I²C SCL | D3 | P0.20 | pin 5 `SCL` | SCL |
| Trackpad RDY | D8 | P1.04 | pin 1 `RDY` | – |
| Trackpad reset | D9 | P1.06 | pin 2 `NRST` | – |
| IMU interrupt (optional) | D4 | P0.22 | – | INT1 |
| 3.3 V | `VCC` | (LDO output, enabled by P0.13) | pin 4 `VDDHI` | VIN / 3V3 |
| Ground | `GND` | – | pin 3 `GND` | GND |

The devicetree for this table is [`boards/nice_nano_v2.overlay`](../boards/nice_nano_v2.overlay).

Both peripherals share **one I²C bus (TWIM0, 400 kHz)**:

| Device | 7-bit address | Note |
|---|---|---|
| Azoteq TPS65 (IQS550, IQS5xx-B000 firmware) | **0x74** | Fixed in the controller firmware; not configurable at runtime. Not the Cirque Pinnacle (0x2A). |
| ST LSM6DSOX | **0x6A** (SA0 = GND) or **0x6B** (SA0 = VDD) | The overlay uses 0x6A. If `i2c scan` shows 0x6B, change `reg` and the node name in the overlay. |

## 2. nice!nano v2 facts that matter here

Checked against the nice!nano v2 schematic (nicekeyboards.com) and the
nice!nano bootloader board file:

* **The `VCC` pin is a switched 3.3 V rail.** U3 (XC6220B331MR, 3.3 V LDO,
  fed from VDDH) has its CE pin on **P0.13** (`POWER_PIN`) with a 10 MΩ
  pull-up to VDDH. The rail is **on by default** and turns **off when P0.13
  is driven low**. The firmware models it as a `regulator-fixed` node
  (`ext_vcc`) with `regulator-boot-on` and `regulator-always-on`, so it is
  never switched off, including in System OFF. That keeps the trackpad able
  to wake the MCU.
* **There are no on-board I²C pull-ups** on the nice!nano.
* LED: blue, **P0.15**, active high.
* P0.14 and P0.16 are tied to the RESET net, and P0.18 is nRESET. Never use
  them; they are listed in `gpio-reserved-ranges`.
* A 32.768 kHz crystal is fitted on P0.00/P0.01, so the BLE low-frequency
  clock uses the crystal (LFXO).
* DC/DC inductors are fitted for both REG0 (VDDH → VDD) and REG1 (VDD → 1.3 V).
  Both DC/DC stages are enabled in the board DTS.
* Battery: the nRF52840 runs in high-voltage mode from BAT+ (VDDH). The
  firmware measures the battery through the SAADC `VDDHDIV5` input. No
  external divider is needed.

## 3. I²C pull-ups (REQUIRED)

| Source | Pull-ups? |
|---|---|
| nice!nano v2 | none |
| TPS65 module | **unknown**. The PCB render shows resistors near J1, but no schematic is published. **VERIFY ON HARDWARE**: measure SDA→VDDHI and SCL→VDDHI on the unpowered module. |
| LSM6DSOX breakout | depends on the board. Adafruit #4438 has pull-ups on SDA/SCL; check yours. |
| nRF52840 internal | ~13 kΩ, enabled in the overlay (`bias-pull-up`) as a fallback only |

**Recommendation:** fit **4.7 kΩ from SDA and from SCL to 3.3 V** unless the
breakout already provides pull-ups (Azoteq recommends 4.7 kΩ). With only the
internal pull-ups, reduce the bus to 100 kHz (`clock-frequency =
<I2C_BITRATE_STANDARD>` in the overlay).

## 4. TPS65 connector

* 6-pin ZIF FPC connector J1 (module revision `TPS65-201A`).
* Pin order (Azoteq module datasheet v1.06 Table 3.1, identical in v1.02):
  **1 RDY, 2 NRST, 3 GND, 4 VDDHI, 5 SCL, 6 SDA**. The silkscreen near J1
  reads `RDY RST GND 3V3 SCL SDA`.
* **FPC pitch: not documented** in any Azoteq TPS65 document. **VERIFY ON
  HARDWARE**: measure the ZIF on the received module before ordering an
  FFC or a mating connector.
* **Ribbon orientation:** on "same-side" and "opposite-side" contact FFCs,
  pin 1 lands at opposite ends. Trace pin 1 (RDY) end to end with a
  continuity tester before connecting power. A reversed cable puts 3.3 V on
  SCL and GND on VDDHI.
* VDDHI: 1.65–3.6 V (3.3 V typical). Absolute maximum 3.6 V, so never connect
  it to RAW/BAT+.
* NRST has an internal pull-up and the controller can pull it low itself.
  The firmware drives it **open-drain, active-low**.
* RDY is active-high. The firmware enables a weak pull-down on P1.04, so an
  unplugged FFC reads as "no data".
* Supply current (module datasheet): about 2.2 mA while tracking (active
  mode), about 23 µA in low-power mode (LP2 at 160 ms).
* **Supply risk:** Azoteq's EOL notice (Oct 2026) lists TPS65-101x/201x as
  end-of-life since 2024-04-11 ("final order from stock: contact sales").
  Secure spare modules for the exhibition.

## 5. LSM6DSOX

* Driver: Zephyr `st,lsm6dso`. The LSM6DSOX has the same WHO_AM_I (0x6C) and
  identical accelerometer/gyroscope control and output registers. This was
  checked register by register against ST's `lsm6dso_reg.h` and
  `lsm6dsox_reg.h`.
* Configuration: ±4 g, ±500 dps, 104 Hz (devicetree).
* Supply: 3.3 V from `VCC`. Most breakouts have their own LDO and level
  shifting, which is harmless at 3.3 V.
* **Mounting orientation:** the IMU axes must be mapped into the pad's body
  frame (`CONFIG_WAFER_PAD_IMU_MOUNT_X/Y/Z`, see
  [calibration.md](calibration.md)). **VERIFY ON HARDWARE** with `wp imu`
  (tilt each edge up and check the sign).
* INT1 → D4/P0.22 is optional. The v0.1 firmware polls the IMU at 104 Hz.
  INT1 is only used as a System-OFF wake source when
  `CONFIG_WAFER_PAD_IMU_WAKEUP=y`.

## 6. Battery and charging (later stage)

* Charger: **TI BQ24075** with R_ISET = 10 kΩ. The fast-charge current is
  I = K_ISET / R_ISET = 890 AΩ / 10 kΩ ≈ **89 mA** (datasheet range 80–98 mA).
* The **"Charge Boost" jumper JP1** puts 2 kΩ in parallel with R_ISET
  (≈ 1.67 kΩ), raising the charge current to **≈ 530 mA**. **Leave JP1 open**
  unless the cell is rated for at least 0.5 A charge (≥ ~550 mAh at 1 C).
* With JP1 open, use a cell of **≥ 100 mAh** so that 89 mA stays at or below
  1 C. Cells below 100 mAh would be charged above 1 C; don't use them without
  checking the cell datasheet.
* The charger's TS input is tied to a fixed 10 kΩ, so **there is no battery
  temperature monitoring**. Use a LiPo with a built-in protection circuit
  (PCM).
* Input current limit: 500 mA (EN1 = high, EN2 = low on the schematic).

## 7. Bring-up checklist

1. Before connecting the TPS65, check the FFC pin 1 orientation (section 4).
2. Power over USB and open the CDC ACM console (see [testing.md](testing.md)).
3. `i2c scan i2c@40003000` should list `0x74` (TPS65) and `0x6a` or `0x6b` (IMU).
   (`i2c@40003000` is the devicetree name of TWIM0 on the nRF52840.)
4. Measure 3.3 V on `VCC`, and on VDDHI at J1-4.
5. Check RDY toggling with a scope or logic analyser while touching the pad.

## Sources

* nice!nano v2 schematic, nicekeyboards.com/docs/nice-nano (net names
  POWER_PIN, BLED, RESET; U2 BQ24075, U3 XC6220B331MR, Y2 32.768 kHz, L1/L3/L4).
* Adafruit_nRF52_Bootloader `src/boards/nice_nano/board.h` (LED P0.15,
  UF2 board id `nRF52840-nicenano`) and `src/usb/uf2/uf2cfg.h` (UF2 write limit).
* ZMK `app/boards/nicekeyboards/nice_nano/nice_nano_nrf52840_zmk_2_0_0.overlay`
  (EXT_POWER on P0.13 active high, battery via VDDH).
* Azoteq IQS5xx-B000 datasheet rev 2.1 (address 0x74, RDY, window protocol),
  Azoteq ProxSense trackpad module datasheet v1.06 (TPS65 J1 pin order, VDDHI,
  current), Azoteq EOL notice Oct 2026.
* TI BQ24075 datasheet (K_ISET = 890 AΩ).
* ST hal_st `lsm6dso_reg.h` / `lsm6dsox_reg.h` (Zephyr v4.5.0 module `hal_st`).
