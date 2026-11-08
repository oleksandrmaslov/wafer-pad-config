# Copyright (c) 2026 Wafer Pad contributors
# SPDX-License-Identifier: Apache-2.0

target_sources(app PRIVATE router.c)
target_sources_ifdef(CONFIG_SHELL app PRIVATE router_shell.c)
