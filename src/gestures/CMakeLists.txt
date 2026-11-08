# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# gesture_engine.c and pointer_accel.c are pure C (also built by tests/gestures).
# gesture_engine.c links wp_scroll_map() from src/orientation/orientation_transform.c.
target_sources(app PRIVATE
  gesture_engine.c
  pointer_accel.c
  gestures_kconfig.c
)
