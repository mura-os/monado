// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR eye tracking.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include "math/m_relation_history.h"

#include "util/u_logging.h"

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_device.h"

#include <galaxyxr/eyetracking.h>

#include <limits.h>
#include <stdbool.h>


#ifndef PATH_MAX
#define PATH_MAX 4096
#endif


#ifdef __cplusplus
extern "C" {
#endif

struct galaxyxr_eye_tracking
{
	struct m_relation_history *relation_history;
	galaxyxr_eyetracking tracker;

	/*!
	 * How many users need gaze right now: XR clients (one use for all of
	 * them, the device feature lifecycle is refcounted upstream) and the
	 * foveation fovea. The cameras run while it is above zero.
	 */
	xrt_atomic_s32_t use_count;

	char model_path[PATH_MAX];
};

bool
galaxyxr_eye_tracking_init(struct galaxyxr_eye_tracking *et, struct xrt_device *xdev, enum u_logging_level *log_level);

void
galaxyxr_eye_tracking_destroy(struct galaxyxr_eye_tracking *et);

void
galaxyxr_eye_tracking_retain(struct galaxyxr_eye_tracking *et);

void
galaxyxr_eye_tracking_release(struct galaxyxr_eye_tracking *et);

bool
galaxyxr_eye_tracking_is_needed(struct galaxyxr_eye_tracking *et);

int
galaxyxr_eye_tracking_open(struct galaxyxr_eye_tracking *et, struct xrt_device *xdev, enum u_logging_level *log_level);

void
galaxyxr_eye_tracking_close(struct galaxyxr_eye_tracking *et);

bool
galaxyxr_eye_tracking_process_events(struct galaxyxr_eye_tracking *et,
                                     struct xrt_device *xdev,
                                     enum u_logging_level *log_level);

void
galaxyxr_eye_tracking_get_relation(struct galaxyxr_eye_tracking *et,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation);

#ifdef __cplusplus
}
#endif
