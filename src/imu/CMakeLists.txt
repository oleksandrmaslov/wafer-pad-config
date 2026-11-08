# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# imu_algo.c is pure C (also compiled by tests/imu). imu_kconfig.c provides the
# config bridge used by the router. imu_source.c degrades to stubs when
# CONFIG_WAFER_PAD_IMU=n or the motion0 alias is missing.
target_sources(app PRIVATE
	imu_algo.c
	imu_kconfig.c
	imu_source.c
)
