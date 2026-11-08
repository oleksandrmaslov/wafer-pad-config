# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# UF2 mass-storage flashing: double-tap RST -> "NICENANO" drive appears.
# The board id matches UF2_BOARD_ID of the nice!nano bootloader
# (Adafruit_nRF52_Bootloader src/boards/nice_nano/board.h).
board_runner_args(uf2 "--board-id=nRF52840-nicenano")
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)

# SWD recovery through the pads on the back (only needed if the bootloader
# itself must be restored).
board_runner_args(jlink "--device=nRF52840_xxAA" "--speed=4000")
board_runner_args(pyocd "--target=nrf52840" "--frequency=4000000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
