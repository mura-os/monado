// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Information reported by compositor targets.
 * @author Stanislav Aleksandrov <lightofmysoul@gmail.com>
 * @ingroup comp_util
 */

#pragma once

#include "xrt/xrt_compiler.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Information discovered by a compositor target during initialization.
 *
 * @ingroup comp_util
 */
struct comp_target_info
{
	//! DRM fd for the display device the target uses, -1 if unknown.
	int display_drm_fd;
};

/*!
 * Initializer for @ref comp_target_info with no information.
 *
 * @relates comp_target_info
 * @ingroup comp_util
 */
#define COMP_TARGET_INFO_INIT XRT_C11_COMPOUND(struct comp_target_info){.display_drm_fd = -1}


#ifdef __cplusplus
}
#endif
