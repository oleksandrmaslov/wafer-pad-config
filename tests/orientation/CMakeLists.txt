# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(orientation_test LANGUAGES C)

set(WP ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_include_directories(app PRIVATE ${WP}/include)
target_sources(app PRIVATE
  src/test_transform.c
  src/test_gravity.c
  src/test_mgr.c
  src/test_gyro.c
  src/test_calib.c
  ${WP}/src/orientation/orientation_transform.c
  ${WP}/src/orientation/orientation_gravity.c
  ${WP}/src/orientation/orientation_mgr.c
  ${WP}/src/orientation/orientation_calib.c
)
