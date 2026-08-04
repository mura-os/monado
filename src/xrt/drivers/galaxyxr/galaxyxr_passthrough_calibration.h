// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

#pragma once

#include "xrt/xrt_passthrough.h"

/*!
 * Load the complete rgb-left_curved/rgb-right_curved family from one
 * per-device EFS profile. If @p path is NULL, search the device locations.
 */
int
galaxyxr_passthrough_calibration_load(struct xrt_passthrough_calibration *out, const char *path);
