// Copyright 2026, Stanislav Aleksandrov <lightofmysoul@gmail.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared @ref comp_target helpers.
 * @author Stanislav Aleksandrov <lightofmysoul@gmail.com>
 * @ingroup comp_main
 */

#include "main/comp_target.h"


void
comp_target_get_info_default(struct comp_target *ct, struct comp_target_info *out)
{
	(void)ct;
	*out = COMP_TARGET_INFO_INIT;
}

void
comp_target_set_output_enabled_default(struct comp_target *ct, bool enabled)
{
	(void)ct;
	(void)enabled;
}
