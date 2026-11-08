# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# Suppress duplicate unit-address warnings for nRF52840 power/clock/acl/flash
# nodes (same as upstream Nordic boards).
list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
