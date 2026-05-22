// Copyright 2023-2024, Collabora, Ltd.
// Copyright 2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Compositor rendering code helpers.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_util
 */

#pragma once

#include "xrt/xrt_compositor.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "render/render_interface.h"

#include "util/comp_base.h"
#include "util/comp_render.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Swapchain helpers.
 *
 */

static inline VkImageView
get_image_view(const struct comp_swapchain_image *image, enum xrt_layer_composition_flags flags, uint32_t array_index)
{
	if (flags & XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT) {
		return image->views.alpha[array_index];
	}

	return image->views.no_alpha[array_index];
}


/*
 *
 * FOV helpers.
 *
 */

//! Does @p outer fully cover @p inner?
static inline bool
fov_covers(const struct xrt_fov *outer, const struct xrt_fov *inner)
{
	const float eps = 0.0001f;
	return outer->angle_left <= inner->angle_left + eps &&   //
	       outer->angle_right >= inner->angle_right - eps && //
	       outer->angle_up >= inner->angle_up - eps &&       //
	       outer->angle_down <= inner->angle_down + eps;     //
}


/*
 *
 * View index helpers.
 *
 */

static inline bool
is_view_index_right(uint32_t view_index)
{
	return view_index % 2 == 1;
}

static inline void
view_index_to_projection_data(uint32_t view_index,
                              const struct xrt_layer_data *data,
                              const struct xrt_layer_projection_view_data **out_vd)
{
	const struct xrt_layer_projection_data *proj = &data->proj;
	*out_vd = &proj->v[view_index];
}

static inline void
view_index_to_depth_data(uint32_t view_index,
                         const struct xrt_layer_data *data,
                         const struct xrt_layer_projection_view_data **out_vd,
                         const struct xrt_layer_depth_data **out_dvd)
{
	const struct xrt_layer_projection_depth_data *depth = &data->depth;
	*out_vd = &depth->v[view_index];
	*out_dvd = &depth->d[view_index];
}


/*
 *
 * Layer data helpers.
 *
 */

static inline bool
is_layer_view_visible(const struct xrt_layer_data *data, uint32_t view_index)
{
	enum xrt_layer_eye_visibility visibility;
	switch (data->type) {
	case XRT_LAYER_CUBE: visibility = data->cube.visibility; break;
	case XRT_LAYER_CYLINDER: visibility = data->cylinder.visibility; break;
	case XRT_LAYER_EQUIRECT1: visibility = data->equirect1.visibility; break;
	case XRT_LAYER_EQUIRECT2: visibility = data->equirect2.visibility; break;
	case XRT_LAYER_QUAD: visibility = data->quad.visibility; break;
	case XRT_LAYER_PROJECTION:
	case XRT_LAYER_PROJECTION_DEPTH: return true;
	default: return false;
	};

	switch (visibility) {
	case XRT_LAYER_EYE_VISIBILITY_LEFT_BIT: return !is_view_index_right(view_index);
	case XRT_LAYER_EYE_VISIBILITY_RIGHT_BIT: return is_view_index_right(view_index);
	case XRT_LAYER_EYE_VISIBILITY_BOTH: return true;
	case XRT_LAYER_EYE_VISIBILITY_NONE:
	default: return false;
	}
}

static inline bool
is_layer_view_space(const struct xrt_layer_data *data)
{
	return (data->flags & XRT_LAYER_COMPOSITION_VIEW_SPACE_BIT) != 0;
}

static inline bool
is_layer_unpremultiplied(const struct xrt_layer_data *data)
{
	return (data->flags & XRT_LAYER_COMPOSITION_UNPREMULTIPLIED_ALPHA_BIT) != 0;
}

static inline bool
is_layer_alpha_inverted(const struct xrt_layer_data *data)
{
#ifndef XRT_FEATURE_OPENXR_INVERTED_ALPHA
	return false;
#else
	// The layers without source alpha bit flag are sent to comp with alpha 1.0, so they don't need to be inverted.
	return (data->flags & XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT) != 0 &&
	       (data->flags & XRT_LAYER_COMPOSITION_INVERTED_ALPHA_BIT) != 0;
#endif
}

static inline void
set_post_transform_rect(const struct xrt_layer_data *data,
                        const struct xrt_normalized_rect *src_norm_rect,
                        bool invert_flip,
                        struct xrt_normalized_rect *out_norm_rect)
{
	struct xrt_normalized_rect rect = *src_norm_rect;

	if (data->flip_y ^ invert_flip) {
		float h = rect.h;

		rect.h = -h;
		rect.y = rect.y + h;
	}

	*out_norm_rect = rect;
}

static inline void
apply_bias_and_scale_from_layer(const struct xrt_layer_data *data,
                                struct xrt_colour_rgba_f32 *out_scale,
                                struct xrt_colour_rgba_f32 *out_bias)
{
	if (data->flags & XRT_LAYER_COMPOSITION_COLOR_BIAS_SCALE) {
		*out_scale = data->color_scale;
		*out_bias = data->color_bias;
	} else {
		// Use identity scale and zero bias when flag is not set
		*out_scale = XRT_C11_COMPOUND(struct xrt_colour_rgba_f32){1.0f, 1.0f, 1.0f, 1.0f};
		*out_bias = XRT_C11_COMPOUND(struct xrt_colour_rgba_f32){0.0f, 0.0f, 0.0f, 0.0f};
	}
}


/*
 *
 * N-layer fast path helpers, shared between the compute
 * (distortion_nlayer.comp) and the gfx (mesh_nlayer.frag) variant.
 *
 */

/// Returns true iff every layer in the array is a type the N-layer fast
/// paths' shaders know how to sample: projection (incl. depth variant), quad,
/// cylinder, or equirect2. Cube / equirect1 / FB_passthrough fall through to
/// the squasher (which doesn't actually render them either today).
static inline bool
all_layers_are_nlayer_eligible(const struct comp_layer *layers, uint32_t layer_count)
{
	for (uint32_t i = 0; i < layer_count; ++i) {
		enum xrt_layer_type t = layers[i].data.type;
		if (t != XRT_LAYER_PROJECTION && t != XRT_LAYER_PROJECTION_DEPTH && t != XRT_LAYER_QUAD &&
		    t != XRT_LAYER_CYLINDER && t != XRT_LAYER_EQUIRECT2) {
			return false;
		}
	}
	return true;
}

/// Per-layer eye-visibility bits for the `eye_hidden_mask` spec constant:
/// (XRT_LAYER_EYE_VISIBILITY_BOTH XOR visibility), so BOTH — the default,
/// the overwhelmingly common case, and the only value for projection layers
/// per OpenXR semantics — maps to 0. Cube and equirect1 are not on the
/// nlayer eligibility list, so they never reach this.
static inline uint32_t
nlayer_eye_hidden_bits(const struct xrt_layer_data *data)
{
	enum xrt_layer_eye_visibility vis = XRT_LAYER_EYE_VISIBILITY_BOTH;
	switch (data->type) {
	case XRT_LAYER_QUAD: vis = data->quad.visibility; break;
	case XRT_LAYER_CYLINDER: vis = data->cylinder.visibility; break;
	case XRT_LAYER_EQUIRECT2: vis = data->equirect2.visibility; break;
	default: break; // projection / depth: stays BOTH
	}
	return ((uint32_t)XRT_LAYER_EYE_VISIBILITY_BOTH ^ (uint32_t)vis) & 0x3u;
}

/// Compute the model→view inverse (mv_inverse = model_inv * view_inv) for a
/// cylinder/equirect2 layer at one (slot, view).
static inline void
fill_nlayer_wrap_mv_inverse(const struct xrt_pose *pose,
                            const struct xrt_matrix_4x4 *view_mat,
                            struct xrt_matrix_4x4 *out_mv_inverse)
{
	const struct xrt_vec3 unit_scale = {1.0f, 1.0f, 1.0f};
	struct xrt_matrix_4x4 model, model_inv, v_inv;
	math_matrix_4x4_model(pose, &unit_scale, &model);
	math_matrix_4x4_inverse(&model, &model_inv);
	math_matrix_4x4_inverse(view_mat, &v_inv);
	math_matrix_4x4_multiply(&model_inv, &v_inv, out_mv_inverse);
}

/// Fill the per-(slot, view) wrap UBO data for one cylinder layer. params
/// layout: (radius, central_angle, aspect_ratio, _).
static inline void
fill_nlayer_cylinder_data(const struct xrt_layer_data *layer_data,
                          const struct xrt_matrix_4x4 *view_mat,
                          struct render_compute_nlayer_wrap_data *out)
{
	const struct xrt_layer_cylinder_data *c = &layer_data->cylinder;
	fill_nlayer_wrap_mv_inverse(&c->pose, view_mat, &out->mv_inverse);
	// CPU passes 0 for "+INFINITY" so the shader can early-out into the
	// directional-sample branch.
	out->params[0] = (c->radius >= INFINITY) ? 0.0f : c->radius;
	out->params[1] = c->central_angle;
	out->params[2] = c->aspect_ratio;
	out->params[3] = 0.0f;
}

/// Fill the per-(slot, view) wrap UBO data for one equirect2 layer. params
/// layout: (radius, central_horizontal_angle, upper_vertical_angle,
/// lower_vertical_angle).
static inline void
fill_nlayer_equirect2_data(const struct xrt_layer_data *layer_data,
                           const struct xrt_matrix_4x4 *view_mat,
                           struct render_compute_nlayer_wrap_data *out)
{
	const struct xrt_layer_equirect2_data *eq2 = &layer_data->equirect2;
	fill_nlayer_wrap_mv_inverse(&eq2->pose, view_mat, &out->mv_inverse);
	out->params[0] = (eq2->radius >= INFINITY) ? 0.0f : eq2->radius;
	out->params[1] = eq2->central_horizontal_angle;
	out->params[2] = eq2->upper_vertical_angle;
	out->params[3] = eq2->lower_vertical_angle;
}

/// Fill the per-(slot, view) quad UBO data for one quad layer: transforms the
/// quad pose into view space, derives the view-space normal as the difference
/// between two view-space points, then inverts the plane transform so the
/// shader can map intersection points back to plane-local UVs.
static inline void
fill_nlayer_quad_data(const struct xrt_layer_data *layer_data,
                      const struct xrt_matrix_4x4 *view_mat,
                      struct render_compute_nlayer_quad_data *out)
{
	const struct xrt_pose *pose = &layer_data->quad.pose;
	const struct xrt_vec3 *origin = &pose->position;

	// Quad center in view space.
	struct xrt_vec3 quad_position = XRT_STRUCT_INIT;
	math_matrix_4x4_transform_vec3(view_mat, origin, &quad_position);

	// View-space normal: rotate +z by the quad's orientation, translate to
	// world space, transform to view space, then subtract the view-space
	// origin to recover the rotated normal direction.
	struct xrt_vec3 normal_world = {0.0f, 0.0f, 1.0f};
	math_quat_rotate_vec3(&pose->orientation, &normal_world, &normal_world);
	struct xrt_vec3 normal_view_space = normal_world;
	math_vec3_accum(origin, &normal_view_space);
	math_matrix_4x4_transform_vec3(view_mat, &normal_view_space, &normal_view_space);
	math_vec3_subtract(&quad_position, &normal_view_space);

	// Inverse of the view-space plane transform.
	const struct xrt_vec3 unit_scale = {1.0f, 1.0f, 1.0f};
	struct xrt_matrix_4x4 plane_transform_view_space, inverse_transform;
	math_matrix_4x4_model(pose, &unit_scale, &plane_transform_view_space);
	math_matrix_4x4_multiply(view_mat, &plane_transform_view_space, &plane_transform_view_space);
	math_matrix_4x4_inverse(&plane_transform_view_space, &inverse_transform);

	out->position = quad_position;
	out->_pad0 = 0.0f;
	out->normal = normal_view_space;
	out->_pad1 = 0.0f;
	out->inverse_transform = inverse_transform;
	out->extent = layer_data->quad.size;
	out->_pad2[0] = 0.0f;
	out->_pad2[1] = 0.0f;
}


/*
 *
 * Command helpers.
 *
 */

/*!
 * This inserts a barrier operation that effects all views[X].squash.image
 * fields (which are VkImages).
 */
static inline void
cmd_barrier_view_squash_images(struct vk_bundle *vk,
                               const struct comp_render_dispatch_data *d,
                               VkCommandBuffer cmd,
                               VkAccessFlags src_access_mask,
                               VkAccessFlags dst_access_mask,
                               VkImageLayout transition_from,
                               VkImageLayout transition_to,
                               VkPipelineStageFlags src_stage_mask,
                               VkPipelineStageFlags dst_stage_mask)
{
	VkImageSubresourceRange first_color_level_subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = 1,
	    .baseArrayLayer = 0,
	    .layerCount = 1,
	};

	for (uint32_t i = 0; i < d->squash_view_count; i++) {
		bool already_barriered = false;

		VkImage image = d->views[i].squash.image;

		uint32_t k = i;
		while (k > 0) {
			k--; // k is always greater then zero.

			if (d->views[k].squash.image == image) {
				already_barriered = true;
				break;
			}
		}

		if (already_barriered) {
			continue;
		}

		vk_cmd_image_barrier_locked(              //
		    vk,                                   // vk_bundle
		    cmd,                                  // cmd_buffer
		    image,                                // image
		    src_access_mask,                      // src_access_mask
		    dst_access_mask,                      // dst_access_mask
		    transition_from,                      // old_image_layout
		    transition_to,                        // new_image_layout
		    src_stage_mask,                       // src_stage_mask
		    dst_stage_mask,                       // dst_stage_mask
		    first_color_level_subresource_range); // subresource_range
	}
}


#ifdef __cplusplus
}
#endif
