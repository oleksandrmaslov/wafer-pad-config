# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE main.c)

# Each subsystem adds its own sources with target_sources(app PRIVATE ...).
# Pure algorithm files (no Zephyr includes) are also compiled by tests/.
add_subdirectory(orientation)
