# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0
#
# Zephyr module CMake entry point (zephyr/module.yml: build.cmake = drivers).

# Public driver headers: #include <drivers/azoteq_iqs5xx.h>
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

add_subdirectory_ifdef(CONFIG_AZOTEQ_IQS5XX azoteq)
