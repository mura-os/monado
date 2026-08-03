// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_vulkan_includes.h"

#include "render/render_interface.h"

struct comp_compositor;
struct comp_passthrough;

struct comp_passthrough *
comp_passthrough_create(struct comp_compositor *c);

void
comp_passthrough_destroy(struct comp_passthrough **ptr);

VkDescriptorSetLayout
comp_passthrough_get_descriptor_set_layout(struct comp_passthrough *p);

/*!
 * Called after the previous compositor GPU fence has completed and before
 * recording this frame. Acquires the newest provider frame if available and
 * fills the render-facing data. Passing enabled=false stops the device camera
 * stream via @ref comp_passthrough_set_enabled. Passing do_timewarp=false
 * preserves the calibrated camera model but disables all dynamic
 * capture-to-display reprojection.
 */
bool
comp_passthrough_prepare(struct comp_passthrough *p,
                         bool enabled,
                         bool do_timewarp,
                         const struct xrt_pose eye_poses[XRT_MAX_VIEWS],
                         const struct xrt_pose *head_pose_scanout_begin,
                         const struct xrt_pose *head_pose_scanout_end,
                         uint32_t view_count,
                         struct render_gfx_passthrough_data *out_data);

/*!
 * Tell the provider whether passthrough video will actually be rendered.
 * Disabling releases the current hold and destroys the Vulkan imports, so it
 * must be called at the same GPU-drained point as comp_passthrough_prepare.
 */
void
comp_passthrough_set_enabled(struct comp_passthrough *p, bool enabled);

//! Release the current provider hold after the renderer has drained the GPU.
void
comp_passthrough_release_current(struct comp_passthrough *p);
