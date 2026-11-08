# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE main.c)

# Each subsystem adds its own sources with target_sources(app PRIVATE ...).
# Pure algorithm files (no Zephyr includes) are also compiled by tests/.
add_subdirectory(touch)
add_subdirectory(imu)
add_subdirectory(orientation)
add_subdirectory(gestures)
add_subdirectory(air_mouse)
add_subdirectory(state)
add_subdirectory(hid)
add_subdirectory(power)
add_subdirectory(router)
