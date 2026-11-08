# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# air_mouse.c calls wp_orientation_axes_body() from
# src/orientation/orientation_transform.c, which src/orientation/CMakeLists.txt
# already adds to the app (do not add it here: duplicate symbols).
target_sources(app PRIVATE air_mouse.c air_mouse_kconfig.c)
