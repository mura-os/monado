// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pacer for the Samsung Galaxy XR dual DSI direct mode.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup comp_main
 */

#pragma once

#include <xf86drmMode.h>

#include "xrt/xrt_results.h"

#ifdef __cplusplus
extern "C" {
#endif

struct u_pacing_compositor;

/*!
 * Create the Galaxy XR compositor pacer.
 *
 * Predictions sit on the vsync lattice extrapolated with the exact mode
 * period from the latest hardware-timestamped flip; the present to display
 * offset is derived from the mode's vertical timing plus an emission lag
 * knob for the panel.
 */
xrt_result_t
comp_window_galaxyxr_pacer_create(const drmModeModeInfo *mode, struct u_pacing_compositor **out_upc);

#ifdef __cplusplus
}
#endif
