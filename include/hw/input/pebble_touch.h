/*
 * Pebble touch input helpers
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INPUT_PEBBLE_TOUCH_H
#define HW_INPUT_PEBBLE_TOUCH_H

#include "hw/qdev-core.h"

#define TYPE_PEBBLE_TOUCH "pebble-touch"

typedef void (*PblTouchStateCallback)(void *opaque, bool down,
                                      uint32_t x, uint32_t y);

void pbl_touch_set_callback(DeviceState *dev, PblTouchStateCallback callback,
                            void *opaque);

#endif
