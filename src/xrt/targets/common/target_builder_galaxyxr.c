// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR builder.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "target_builder_helpers.h"
#include "target_builder_interface.h"

#include "galaxyxr/galaxyxr_interface.h"

#include <assert.h>


#ifndef XRT_BUILD_DRIVER_GALAXYXR
#error "Must only be built with XRT_BUILD_DRIVER_GALAXYXR set"
#endif

DEBUG_GET_ONCE_BOOL_OPTION(galaxyxr_enabled, "GALAXYXR_ENABLE", true)

static const char *driver_list[] = {
    "galaxyxr",
};

static xrt_result_t
galaxyxr_estimate_system(struct xrt_builder *xb,
                         cJSON *config,
                         struct xrt_prober *xp,
                         struct xrt_builder_estimate *estimate)
{
	if (!debug_get_bool_option_galaxyxr_enabled() || !galaxyxr_is_present()) {
		return XRT_SUCCESS;
	}

	estimate->certain.head = true;
	estimate->priority = 0;

	return XRT_SUCCESS;
}

static xrt_result_t
galaxyxr_open_system_impl(struct xrt_builder *xb,
                          cJSON *config,
                          struct xrt_prober *xp,
                          struct xrt_tracking_origin *origin,
                          struct xrt_system_devices *xsysd,
                          struct xrt_frame_context *xfctx,
                          struct t_builder_roles_helper *tbrh)
{
	struct xrt_device *head = galaxyxr_hmd_create();
	if (head == NULL) {
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	xsysd->static_xdevs[xsysd->static_xdev_count++] = head;
	tbrh->head = head;

	return XRT_SUCCESS;
}

static void
galaxyxr_destroy(struct xrt_builder *xb)
{
	free(xb);
}

struct xrt_builder *
galaxyxr_builder_create(void)
{
	struct t_builder *tb = U_TYPED_CALLOC(struct t_builder);

	tb->base.estimate_system = galaxyxr_estimate_system;
	tb->base.open_system = t_builder_open_system_static_roles;
	tb->base.destroy = galaxyxr_destroy;
	tb->base.identifier = "galaxyxr";
	tb->base.name = "Samsung Galaxy XR builder";
	tb->base.driver_identifiers = driver_list;
	tb->base.driver_identifier_count = ARRAY_SIZE(driver_list);

	tb->open_system_static_roles = galaxyxr_open_system_impl;

	return &tb->base;
}
