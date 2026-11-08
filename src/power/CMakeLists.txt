# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# Each file provides stubs when its feature (WAFER_PAD_SLEEP / _BATTERY /
# _LED) is disabled, so the platform.h API always links.
target_sources(app PRIVATE power.c battery.c led.c)
