# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Register this repository as a Zephyr module (out-of-tree board, DT binding,
# Azoteq driver) even when the app is built outside its own west workspace.
# zephyr_module.py merges modules by name, so a duplicate entry from west is
# harmless.
list(APPEND EXTRA_ZEPHYR_MODULES ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wafer_pad LANGUAGES C)

target_include_directories(app PRIVATE include)

add_subdirectory(src)
