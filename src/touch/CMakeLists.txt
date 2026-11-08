# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# touch_tracker.c and touch_iqs5xx.c are pure logic (also compiled by
# tests/touch); touch_source.c and touch_kconfig.c are Zephyr glue.
target_sources(app PRIVATE touch_tracker.c touch_iqs5xx.c touch_source.c touch_kconfig.c)
