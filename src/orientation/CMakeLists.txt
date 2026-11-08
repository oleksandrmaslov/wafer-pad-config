# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# Pure algorithm files (also compiled by tests/orientation) + Kconfig bridge.
target_sources(app PRIVATE
  orientation_transform.c
  orientation_gravity.c
  orientation_mgr.c
  orientation_calib.c
  orientation_kconfig.c
)
