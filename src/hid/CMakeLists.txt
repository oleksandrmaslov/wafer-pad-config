# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

# hid_report.c is pure C (also compiled by tests/hid).
target_sources(app PRIVATE hid_report.c hid_transport.c ble.c ble_hog.c)
