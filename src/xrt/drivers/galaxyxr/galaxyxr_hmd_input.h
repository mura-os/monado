// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR headset inputs.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include "xrt/xrt_device.h"

#include "util/u_logging.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
#define GALAXYXR_HMD_INPUT_COUNT 4
#else
#define GALAXYXR_HMD_INPUT_COUNT 3
#endif

struct galaxyxr_hmd_input
{
	struct xrt_device *xdev;
	enum u_logging_level *log_level;

	//! Monotonic timestamp in bits 0-62, with presence in the sign bit.
	xrt_atomic_s64_t presence_state;

	//! Monotonic timestamp in bits 0-62, with the power-button state in the sign bit.
	xrt_atomic_s64_t power_button_state;
};

void
galaxyxr_hmd_input_init(struct galaxyxr_hmd_input *input, struct xrt_device *xdev, enum u_logging_level *log_level);

xrt_result_t
galaxyxr_hmd_input_update(struct galaxyxr_hmd_input *input);

void
galaxyxr_hmd_input_handle_proximity(struct galaxyxr_hmd_input *input, float state, float adc);

/*!
 * Open and exclusively grab the power-button evdev device. The returned fd is
 * owned by the caller and must be closed from the same worker that called this
 * function.
 */
int
galaxyxr_hmd_input_open_power_button(struct galaxyxr_hmd_input *input);

/*!
 * Drain pending power-button events without blocking. Returns false if the fd
 * can no longer be used.
 */
bool
galaxyxr_hmd_input_poll_power_button(struct galaxyxr_hmd_input *input, int fd);

void
galaxyxr_hmd_input_reset_power_button(struct galaxyxr_hmd_input *input);

#ifdef __cplusplus
}
#endif
