/*
 * Pebble display helpers
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_PEBBLE_DISPLAY_H
#define HW_DISPLAY_PEBBLE_DISPLAY_H

#include "hw/qdev-core.h"

#define TYPE_PEBBLE_DISPLAY "pebble-display"
#define PEBBLE_DISPLAY_FORMAT_RGB332 332

void pbl_display_update_framebuffer(DeviceState *dev, uint32_t offset,
                                    const uint8_t *pixels, uint32_t length);

#endif
