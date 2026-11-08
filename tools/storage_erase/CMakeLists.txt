# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# The nice_nano_v2 board is defined in this repository (two levels up).
get_filename_component(WAFER_PAD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
list(APPEND EXTRA_ZEPHYR_MODULES ${WAFER_PAD_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(storage_erase LANGUAGES C)

target_sources(app PRIVATE src/main.c)
