// Copyright 2023-2024, Collabora, Ltd.
// Copyright 2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Compositor (gfx - graphics shader) rendering code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup comp_util
 */

#include "xrt/xrt_compositor.h"
#include "util/comp_swapchain.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h"

#include "util/u_trace_marker.h"

#include "vk/vk_helpers.h"

#include "render/render_interface.h"

#include "util/comp_render.h"
#include "util/comp_render_helpers.h"


/*
 *
 * Internal structs.
 *
 */

/**
 * Internal per-view for the layer squashing render step.
 */
struct gfx_layer_view_state
{
	/// Filled out descriptor sets.
	VkDescriptorSet descriptor_sets[RENDER_MAX_LAYERS];

	/// The type of layer.
	enum xrt_layer_type types[RENDER_MAX_LAYERS];

	/// Is the alpha premultipled, false means unpremultiplied.
	bool premultiplied_alphas[RENDER_MAX_LAYERS];

	/// To go to this view's tangent lengths.
	struct xrt_normalized_rect to_tangent;

	/// Number of layers filled in.
	/// TODO move to parent struct
	uint32_t layer_count;

	/// Full rotation and translation VP matrix, in world space.
	struct xrt_matrix_4x4 world_vp_full;
	/// Full rotation and translation VP matrix, in view space.
	struct xrt_matrix_4x4 eye_vp_full;

	/// Full rotation and translation inverse V matrix, in world space.
	struct xrt_matrix_4x4 world_v_inv_full;
	/// Full rotation and translation inverse V matrix, in view space.
	struct xrt_matrix_4x4 eye_v_inv_full;

	/// Only rotation and translation VP matrix, in world space.
	struct xrt_matrix_4x4 world_vp_rot_only;
	/// Only rotation and translation VP matrix, in view space.
	struct xrt_matrix_4x4 eye_vp_rot_only;
};

/**
 * Internal state for the layer squashing render step, contains all per-view state
 */
struct gfx_layer_state
{
	struct gfx_layer_view_state views[XRT_MAX_VIEWS];
};

/*
 * Internal state for the mesh rendering step.
 */
struct gfx_mesh_state
{
	VkDescriptorSet descriptor_sets[XRT_MAX_VIEWS];
};

/*
 * Per-view input data for the mesh rendering step.
 */
struct gfx_mesh_view_data
{
	struct xrt_pose src_pose;
	struct xrt_fov src_fov;
	struct xrt_normalized_rect src_norm_rect;
	VkSampler src_sampler;
	VkImageView src_image_view;
};

/*
 * Input data for the mesh rendering step,
 * combined with comp_render_dispatch_data.
 */
struct gfx_mesh_data
{
	struct gfx_mesh_view_data views[XRT_MAX_VIEWS];
};


/*
 *
 * Static data.
 *
 */

static const VkClearColorValue background_color_idle = {
    .float32 = {0.0f, 0.0f, 0.0f, 1.0f},
};

static const VkClearColorValue background_color_active = {
    .float32 = {0.0f, 0.0f, 0.0f, 1.0f},
};


/*
 *
 * Input builder functions.
 *
 */

inline static void
gfx_mesh_add_view(struct gfx_mesh_data *md,
                  uint32_t view_index,
                  const struct xrt_pose *src_pose,
                  const struct xrt_fov *src_fov,
                  const struct xrt_normalized_rect *src_norm_rect,
                  VkSampler src_sampler,
                  VkImageView src_image_view)
{
	md->views[view_index].src_pose = *src_pose;
	md->views[view_index].src_fov = *src_fov;
	md->views[view_index].src_norm_rect = *src_norm_rect;
	md->views[view_index].src_sampler = src_sampler;
	md->views[view_index].src_image_view = src_image_view;
}



/*
 *
 * Model view projection helper functions.
 *
 */

static inline void
calc_mvp_full(struct gfx_layer_view_state *state,
              const struct xrt_layer_data *layer_data,
              const struct xrt_pose *pose,
              const struct xrt_vec3 *scale,
              struct xrt_matrix_4x4 *result)
{
	struct xrt_matrix_4x4 model;
	math_matrix_4x4_model(pose, scale, &model);

	if (is_layer_view_space(layer_data)) {
		math_matrix_4x4_multiply(&state->eye_vp_full, &model, result);
	} else {
		math_matrix_4x4_multiply(&state->world_vp_full, &model, result);
	}
}

static inline void
calc_mv_inv_full(struct gfx_layer_view_state *state,
                 const struct xrt_layer_data *layer_data,
                 const struct xrt_pose *pose,
                 const struct xrt_vec3 *scale,
                 struct xrt_matrix_4x4 *result)
{
	struct xrt_matrix_4x4 model;
	math_matrix_4x4_model(pose, scale, &model);

	struct xrt_matrix_4x4 model_inv;
	math_matrix_4x4_inverse(&model, &model_inv);

	struct xrt_matrix_4x4 *v;
	if (is_layer_view_space(layer_data)) {
		v = &state->eye_v_inv_full;
	} else {
		v = &state->world_v_inv_full;
	}

	math_matrix_4x4_multiply(&model_inv, v, result);
}

static inline void
calc_mvp_rot_only(struct gfx_layer_view_state *state,
                  const struct xrt_layer_data *data,
                  const struct xrt_pose *pose,
                  const struct xrt_vec3 *scale,
                  struct xrt_matrix_4x4 *result)
{
	struct xrt_matrix_4x4 model;
	struct xrt_pose rot_only = {
	    .orientation = pose->orientation,
	    .position = XRT_VEC3_ZERO,
	};
	math_matrix_4x4_model(&rot_only, scale, &model);

	if (is_layer_view_space(data)) {
		math_matrix_4x4_multiply(&state->eye_vp_rot_only, &model, result);
	} else {
		math_matrix_4x4_multiply(&state->world_vp_rot_only, &model, result);
	}
}


/*
 *
 * Graphics layer data builders.
 *
 */

static inline const struct comp_swapchain_image *
get_layer_image(const struct comp_layer *layer, uint32_t swapchain_index, uint32_t image_index)
{

	const struct comp_swapchain *sc = (struct comp_swapchain *)(comp_layer_get_swapchain(layer, swapchain_index));
	return &sc->images[image_index];
}

static inline void
add_layer(struct gfx_layer_view_state *state, const struct xrt_layer_data *data, VkDescriptorSet descriptor_set)
{
	uint32_t cur_layer = state->layer_count++;
	state->descriptor_sets[cur_layer] = descriptor_set;
	state->types[cur_layer] = data->type;
	state->premultiplied_alphas[cur_layer] = !is_layer_unpremultiplied(data);
}

/// Data setup for a cylinder layer
/// Also allocates and writes a descriptor set!
static VkResult
do_cylinder_layer(struct render_gfx *render,
                  const struct comp_layer *layer,
                  uint32_t view_index,
                  VkSampler clamp_to_edge,
                  VkSampler clamp_to_border_black,
                  struct gfx_layer_view_state *state)
{
	const struct xrt_layer_data *layer_data = &layer->data;
	const struct xrt_layer_cylinder_data *c = &layer_data->cylinder;
	const uint32_t array_index = c->sub.array_index;
	const struct comp_swapchain_image *image = get_layer_image(layer, 0, c->sub.image_index);

	struct vk_bundle *vk = render->r->vk;
	VkResult ret;

	// Color
	VkSampler src_sampler = clamp_to_edge; // WIP: Is this correct?
	VkImageView src_image_view = get_image_view(image, layer_data->flags, array_index);

	// Fully initialised below.
	struct render_gfx_layer_cylinder_data data;

	// Used for Subimage and OpenGL flip.
	set_post_transform_rect(   //
	    layer_data,            // data
	    &c->sub.norm_rect,     // src_norm_rect
	    false,                 // invert_flip
	    &data.post_transform); // out_norm_rect

	// Shared scale for all paths.
	struct xrt_vec3 scale = {1, 1, 1};

	// Handle infinite radius.
	if (c->radius == 0 || c->radius == INFINITY) {
		// Use rotation only to center the cylinder on the eye.
		calc_mvp_rot_only(state, layer_data, &c->pose, &scale, &data.mvp);
		data.radius = 1.0; // Fixed radius at one.
		data.central_angle = c->central_angle;
		data.aspect_ratio = c->aspect_ratio;
	} else {
		calc_mvp_full(state, layer_data, &c->pose, &scale, &data.mvp);
		data.radius = c->radius;
		data.central_angle = c->central_angle;
		data.aspect_ratio = c->aspect_ratio;
	}

	apply_bias_and_scale_from_layer(layer_data, &data.color_scale, &data.color_bias);

	// Can fail if we have too many layers.
	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = render_gfx_layer_cylinder_alloc_and_write( //
	    render,                                      //
	    &data,                                       //
	    src_sampler,                                 //
	    src_image_view,                              //
	    &descriptor_set);                            // out_descriptor_set
	VK_CHK_AND_RET(ret, "render_gfx_layer_cylinder_alloc_and_write");

	VK_NAME_DESCRIPTOR_SET(vk, descriptor_set, "render_gfx layer quad descriptor set");

	add_layer(state, layer_data, descriptor_set);

	return VK_SUCCESS;
}

/// Data setup for an "equirect2" layer
/// Also allocates and writes a descriptor set!
static VkResult
do_equirect2_layer(struct render_gfx *render,
                   const struct comp_layer *layer,
                   uint32_t view_index,
                   VkSampler clamp_to_edge,
                   VkSampler clamp_to_border_black,
                   struct gfx_layer_view_state *state)
{
	const struct xrt_layer_data *layer_data = &layer->data;
	const struct xrt_layer_equirect2_data *eq2 = &layer_data->equirect2;
	const uint32_t array_index = eq2->sub.array_index;
	const struct comp_swapchain_image *image = get_layer_image(layer, 0, eq2->sub.image_index);

	struct vk_bundle *vk = render->r->vk;
	VkResult ret;

	// Color
	VkSampler src_sampler = clamp_to_edge;
	VkImageView src_image_view = get_image_view(image, layer_data->flags, array_index);

	// Fully initialised below.
	struct render_gfx_layer_equirect2_data data;

	// Used for Subimage and OpenGL flip.
	set_post_transform_rect(   //
	    layer_data,            // data
	    &eq2->sub.norm_rect,   // src_norm_rect
	    false,                 // invert_flip
	    &data.post_transform); // out_norm_rect

	struct xrt_vec3 scale = {1.f, 1.f, 1.f};
	calc_mv_inv_full(state, layer_data, &eq2->pose, &scale, &data.mv_inverse);

	// Make it possible to go tangent lengths.
	data.to_tangent = state->to_tangent;

	// Simplifies the shader.
	if (eq2->radius >= INFINITY) {
		data.radius = 0.0;
	} else {
		data.radius = eq2->radius;
	}

	data.central_horizontal_angle = eq2->central_horizontal_angle;
	data.upper_vertical_angle = eq2->upper_vertical_angle;
	data.lower_vertical_angle = eq2->lower_vertical_angle;

	apply_bias_and_scale_from_layer(layer_data, &data.color_scale, &data.color_bias);

	// Can fail if we have too many layers.
	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = render_gfx_layer_equirect2_alloc_and_write( //
	    render,                                       //
	    &data,                                        //
	    src_sampler,                                  //
	    src_image_view,                               //
	    &descriptor_set);                             // out_descriptor_set
	VK_CHK_AND_RET(ret, "render_gfx_layer_equirect2_alloc_and_write");

	VK_NAME_DESCRIPTOR_SET(vk, descriptor_set, "render_gfx layer quad descriptor set");

	add_layer(state, layer_data, descriptor_set);

	return VK_SUCCESS;
}

/// Data setup for a projection layer
/// Also allocates and writes a descriptor set!
static VkResult
do_projection_layer(struct render_gfx *render,
                    const struct comp_layer *layer,
                    uint32_t view_index,
                    VkSampler clamp_to_edge,
                    VkSampler clamp_to_border_black,
                    struct gfx_layer_view_state *state)
{
	const struct xrt_layer_data *layer_data = &layer->data;
	const struct xrt_layer_projection_view_data *vd = NULL;
	const struct xrt_layer_depth_data *dvd = NULL;

	if (layer_data->type == XRT_LAYER_PROJECTION) {
		view_index_to_projection_data(view_index, layer_data, &vd);
	} else {
		view_index_to_depth_data(view_index, layer_data, &vd, &dvd);
	}

	uint32_t sc_array_index = is_view_index_right(view_index) ? 1 : 0;
	uint32_t array_index = vd->sub.array_index;
	const struct comp_swapchain_image *image = get_layer_image(layer, sc_array_index, vd->sub.image_index);

	struct vk_bundle *vk = render->r->vk;
	VkResult ret;
	// Color
	VkSampler src_sampler = clamp_to_border_black;
	VkImageView src_image_view = get_image_view(image, layer_data->flags, array_index);

	// Fully initialised below.
	struct render_gfx_layer_projection_data data;

	// Used for Subimage and OpenGL flip.
	set_post_transform_rect(   //
	    layer_data,            // data
	    &vd->sub.norm_rect,    // src_norm_rect
	    false,                 // invert_flip
	    &data.post_transform); // out_norm_rect

	// Used to go from UV to tangent space.
	render_calc_uv_to_tangent_lengths_rect(&vd->fov, &data.to_tangent);

	// Create MVP matrix, rotation only so we get 3dof timewarp.
	struct xrt_vec3 scale = {1, 1, 1};
	calc_mvp_rot_only(state, layer_data, &vd->pose, &scale, &data.mvp);

	apply_bias_and_scale_from_layer(layer_data, &data.color_scale, &data.color_bias);

	// Can fail if we have too many layers.
	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = render_gfx_layer_projection_alloc_and_write( //
	    render,                                        //
	    &data,                                         //
	    src_sampler,                                   //
	    src_image_view,                                //
	    &descriptor_set);                              // out_descriptor_set
	VK_CHK_AND_RET(ret, "render_gfx_layer_projection_alloc_and_write");

	VK_NAME_DESCRIPTOR_SET(vk, descriptor_set, "render_gfx layer proj descriptor set");

	add_layer(state, layer_data, descriptor_set);

	return VK_SUCCESS;
}

/// Data setup for a quad layer
/// Also allocates and writes a descriptor set!
static VkResult
do_quad_layer(struct render_gfx *render,
              const struct comp_layer *layer,
              uint32_t view_index,
              VkSampler clamp_to_edge,
              VkSampler clamp_to_border_black,
              struct gfx_layer_view_state *state)
{
	const struct xrt_layer_data *layer_data = &layer->data;
	const struct xrt_layer_quad_data *q = &layer_data->quad;
	const uint32_t array_index = q->sub.array_index;
	const struct comp_swapchain_image *image = get_layer_image(layer, 0, q->sub.image_index);

	struct vk_bundle *vk = render->r->vk;
	VkResult ret;

	// Color
	VkSampler src_sampler = clamp_to_edge;
	VkImageView src_image_view = get_image_view(image, layer_data->flags, array_index);

	// Fully initialised below.
	struct render_gfx_layer_quad_data data;

	// Used for Subimage and OpenGL flip.
	set_post_transform_rect(   //
	    layer_data,            // data
	    &q->sub.norm_rect,     // src_norm_rect
	    false,                 // invert_flip
	    &data.post_transform); // out_norm_rect

	// Create MVP matrix, full 6dof mvp needed.
	struct xrt_vec3 scale = {q->size.x, q->size.y, 1};
	calc_mvp_full(state, layer_data, &q->pose, &scale, &data.mvp);

	apply_bias_and_scale_from_layer(layer_data, &data.color_scale, &data.color_bias);

	// Can fail if we have too many layers.
	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = render_gfx_layer_quad_alloc_and_write( //
	    render,                                  //
	    &data,                                   //
	    src_sampler,                             //
	    src_image_view,                          //
	    &descriptor_set);                        // out_descriptor_set
	VK_CHK_AND_RET(ret, "render_gfx_layer_quad_alloc_and_write");

	VK_NAME_DESCRIPTOR_SET(vk, descriptor_set, "render_gfx layer quad descriptor set");

	add_layer(state, layer_data, descriptor_set);

	return VK_SUCCESS;
}

static void
crg_clear_output(struct render_gfx *render, const struct comp_render_dispatch_data *d)
{
	render_gfx_begin_target(     //
	    render,                  //
	    d->target.gfx.rtr,       //
	    &background_color_idle); //

	render_gfx_end_target(render);
}

/*
 *
 * Graphics distortion helpers.
 *
 */

/// Used in both fast-path and layer-squashed routes
static void
crg_distortion_common(struct render_gfx *render,
                      bool do_timewarp,
                      const struct gfx_mesh_data *md,
                      const struct comp_render_dispatch_data *d)
{
	struct vk_bundle *vk = render->r->vk;
	VkResult ret;

	/*
	 * Reserve UBOs, create descriptor sets, and fill in any data ahead of
	 * time. If we ever want to copy UBO data this lets us do that easily:
	 * write a copy command before the other gfx commands.
	 */

	struct gfx_mesh_state ms = XRT_STRUCT_INIT;

	for (uint32_t i = 0; i < d->target.view_count; i++) {

		struct render_gfx_mesh_ubo_data data = {
		    .vertex_rot = d->views[i].target.gfx.vertex_rot,
		    .post_transform = md->views[i].src_norm_rect,
		};

		// Extra arguments for timewarp.
		if (do_timewarp) {
			data.pre_transform = d->views[i].pre_transform;

			render_calc_time_warp_matrix(              //
			    &md->views[i].src_pose,                //
			    &md->views[i].src_fov,                 //
			    &d->views[i].world_pose_scanout_begin, //
			    &data.transform_scanout_begin);        //

			render_calc_time_warp_matrix(            //
			    &md->views[i].src_pose,              //
			    &md->views[i].src_fov,               //
			    &d->views[i].world_pose_scanout_end, //
			    &data.transform_scanout_end);        //
		}

		ret = render_gfx_mesh_alloc_and_write( //
		    render,                            //
		    &data,                             //
		    md->views[i].src_sampler,          //
		    md->views[i].src_image_view,       //
		    &ms.descriptor_sets[i]);           //
		VK_CHK_WITH_GOTO(ret, "render_gfx_mesh_alloc_and_write", err_no_memory);

		VK_NAME_DESCRIPTOR_SET(vk, ms.descriptor_sets[i], "render_gfx mesh descriptor sets");
	}


	/*
	 * Do command writing here.
	 */

	render_gfx_begin_target(       //
	    render,                    //
	    d->target.gfx.rtr,         //
	    &background_color_active); //

	for (uint32_t i = 0; i < d->target.view_count; i++) {
		// Convenience.
		const struct render_viewport_data *viewport_data = &d->views[i].target.viewport_data;

		const render_scissor_data_t *scissor_data = &d->views[i].target.scissor_data;

		render_gfx_begin_view( //
		    render,            //
		    i,                 // view_index
		    viewport_data,     //
		    scissor_data);     //

		render_gfx_mesh_draw(      //
		    render,                //
		    i,                     // mesh_index
		    ms.descriptor_sets[i], //
		    do_timewarp);          //

		render_gfx_end_view(render);
	}

	render_gfx_end_target(render);

	return;

err_no_memory:
	// Allocator reset at end of frame, nothing to clean up.
	VK_ERROR(vk, "Could not allocate all UBOs for frame, that's really strange and shouldn't happen!");
}

/// For use after squashing layers
static void
crg_distortion_after_squash(struct render_gfx *render, const struct comp_render_dispatch_data *d)
{

	// Shared between all views.
	VkSampler clamp_to_border_black = render->r->samplers.clamp_to_border_black;

	struct gfx_mesh_data md = XRT_STRUCT_INIT;
	for (uint32_t i = 0; i < d->target.view_count; i++) {
		struct xrt_pose src_pose = d->views[i].world_pose_scanout_begin;
		struct xrt_fov src_fov = d->views[i].fov;
		VkImageView src_image_view = d->views[i].squash_as_src.sample_view;
		struct xrt_normalized_rect src_norm_rect = d->views[i].squash_as_src.norm_rect;

		gfx_mesh_add_view(         //
		    &md,                   //
		    i,                     // view_index
		    &src_pose,             //
		    &src_fov,              //
		    &src_norm_rect,        //
		    clamp_to_border_black, // src_sampler
		    src_image_view);       //
	}

	crg_distortion_common( //
	    render,            //
	    d->do_timewarp,    //
	    &md,               //
	    d);                //
}

/// Fast path
static void
crg_distortion_fast_path(struct render_gfx *render,
                         const struct comp_render_dispatch_data *d,
                         const struct comp_layer *layer,
                         const struct xrt_layer_projection_view_data *vds[XRT_MAX_VIEWS])
{
	const struct xrt_layer_data *data = &layer->data;

	VkSampler clamp_to_border_black = render->r->samplers.clamp_to_border_black;

	struct gfx_mesh_data md = XRT_STRUCT_INIT;
	for (uint32_t i = 0; i < d->target.view_count; i++) {
		const uint32_t array_index = vds[i]->sub.array_index;

		const struct comp_swapchain_image *image = get_layer_image(layer, i, vds[i]->sub.image_index);

		struct xrt_pose src_pose;
		struct xrt_fov src_fov;
		struct xrt_normalized_rect src_norm_rect;

		src_pose = vds[i]->pose;
		src_fov = vds[i]->fov;
		src_norm_rect = vds[i]->sub.norm_rect;
		VkImageView src_image_view = get_image_view(image, data->flags, array_index);

		if (data->flip_y) {
			src_norm_rect.y += src_norm_rect.h;
			src_norm_rect.h = -src_norm_rect.h;
		}

		gfx_mesh_add_view(         //
		    &md,                   // md
		    i,                     // view_index
		    &src_pose,             // src_pose
		    &src_fov,              // src_fov
		    &src_norm_rect,        // src_norm_rect
		    clamp_to_border_black, // src_sampler
		    src_image_view);       // src_image_view
	}

	crg_distortion_common( //
	    render,            //
	    d->do_timewarp,    //
	    &md,               //
	    d);                //
}


/*
 *
 * Gfx N-layer fast path.
 *
 */

/// N-layer fast path: all layers composited by a single fragment shader
/// (mesh_nlayer.frag), one draw per view, directly into the target — the gfx
/// twin of the compute path's crc_nlayer_fast_path. No scratch image round
/// trip, no per-layer draws.
///
/// Returns true on success. Returns false iff the per-variant pipeline build
/// or the per-frame allocations failed — in that case no command-buffer state
/// was recorded and the caller can route to the squasher fallback in the same
/// frame.
static bool
crg_nlayer_fast_path(struct render_gfx *render,
                     const struct comp_render_dispatch_data *d,
                     const struct comp_layer *layers,
                     uint32_t layer_count)
{
	struct render_resources *r = render->r;
	VkResult ret;

	assert(layer_count >= 1 && layer_count <= r->gfx.nlayer.effective_nlayer_max);

	const uint32_t view_count = d->target.view_count;
	VkSampler clamp_to_edge = r->samplers.clamp_to_edge;
	VkSampler clamp_to_border_black = r->samplers.clamp_to_border_black;
	VkSampler clamp_to_border_transparent = r->samplers.clamp_to_border_transparent;

	// Today only TOP_TO_BOTTOM uses the begin/end lerp; everything else is
	// treated as global flash (single matrix).
	bool needs_end_pose = d->scanout_direction == XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM;

	// Per-(layer, view) source arrays, flat-indexed [layer * view_count + view].
	VkSampler src_samplers[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	VkImageView src_image_views[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];

	uint32_t unpremultiplied_mask = 0;
	uint32_t inverted_alpha_mask = 0;
	uint32_t layer_types = 0;
	uint32_t eye_hidden_mask = 0;
	uint32_t view_space_slots = 0;
	uint32_t projection_bounds_test_mask = 0;

	// ~5.5 KiB; filled here, then copied into the sub-allocated mapped
	// range by render_gfx_mesh_nlayer_alloc_and_write.
	struct render_gfx_mesh_nlayer_ubo_data data = XRT_STRUCT_INIT;

	// Per-view view-from-pose matrices. eye_view for view-space layers
	// (HUD-style, attached to the head), world_view for world-space layers.
	struct xrt_matrix_4x4 eye_view_mats[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 world_view_mats[XRT_MAX_VIEWS];
	for (uint32_t v = 0; v < view_count; v++) {
		math_matrix_4x4_view_from_pose(&d->views[v].eye_pose, &eye_view_mats[v]);
		math_matrix_4x4_view_from_pose(&d->views[v].world_pose_scanout_begin, &world_view_mats[v]);

		data.vertex_rot[v] = d->views[v].target.gfx.vertex_rot;
		data.pre_transforms[v] = d->views[v].pre_transform;

		// Per-view rotation R_delta = R_begin^-1 * R_end (rotation only),
		// lerped from identity by scanout_t in the shader and applied to
		// the view-space ray directions of non-projection layers.
		// Identity when unused.
		if (needs_end_pose) {
			const struct xrt_vec3 unit_scale = {1.0f, 1.0f, 1.0f};
			struct xrt_pose rot_only_begin = {
			    .orientation = d->views[v].world_pose_scanout_begin.orientation,
			    .position = {0.0f, 0.0f, 0.0f},
			};
			struct xrt_pose rot_only_end = {
			    .orientation = d->views[v].world_pose_scanout_end.orientation,
			    .position = {0.0f, 0.0f, 0.0f},
			};
			struct xrt_matrix_4x4 R_begin, R_end, R_begin_inv;
			math_matrix_4x4_model(&rot_only_begin, &unit_scale, &R_begin);
			math_matrix_4x4_model(&rot_only_end, &unit_scale, &R_end);
			math_matrix_4x4_inverse(&R_begin, &R_begin_inv);
			math_matrix_4x4_multiply(&R_begin_inv, &R_end, &data.scanout_view_rot_delta[v]);
		} else {
			math_matrix_4x4_identity(&data.scanout_view_rot_delta[v]);
		}
	}

	for (uint32_t l = 0; l < layer_count; l++) {
		const struct comp_layer *layer = &layers[l];
		const struct xrt_layer_data *ld = &layer->data;

		// One bit per slot in each mask (see mesh_nlayer.frag).
		if (is_layer_unpremultiplied(ld)) {
			unpremultiplied_mask |= (1u << l);
		}
		if (is_layer_alpha_inverted(ld)) {
			inverted_alpha_mask |= (1u << l);
		}

		// Per-layer eye-visibility bits, BOTH-XOR polarity so all-visible
		// keeps the shader spec const at 0 for typical apps.
		eye_hidden_mask |= nlayer_eye_hidden_bits(ld) << (l * 2u);

		// View-space transform: layers with XRT_LAYER_COMPOSITION_VIEW_SPACE_BIT
		// (HUD-style, attached to the head) use the eye pose; everything
		// else uses the head/world pose.
		bool view_space = is_layer_view_space(ld);
		if (view_space) {
			view_space_slots |= (1u << l);
		}
		const struct xrt_matrix_4x4 *view_mats = view_space ? eye_view_mats : world_view_mats;

		if (ld->type == XRT_LAYER_QUAD) {
			layer_types |= ((uint32_t)RENDER_NLAYER_TYPE_QUAD << (l * RENDER_NLAYER_TYPE_BITS));

			const struct xrt_layer_quad_data *q = &ld->quad;
			const struct comp_swapchain_image *image = get_layer_image(layer, 0, q->sub.image_index);
			VkImageView image_view = get_image_view(image, ld->flags, q->sub.array_index);

			struct xrt_normalized_rect rect = XRT_STRUCT_INIT;
			set_post_transform_rect(ld, &q->sub.norm_rect, true, &rect);

			for (uint32_t v = 0; v < view_count; v++) {
				uint32_t src_i = l * view_count + v;
				uint32_t ubo_i = l * XRT_MAX_VIEWS + v;
				src_samplers[src_i] = clamp_to_edge;
				src_image_views[src_i] = image_view;
				data.post_transforms[ubo_i] = rect;
				fill_nlayer_quad_data(ld, &view_mats[v], &data.quads[ubo_i]);
			}
			continue;
		}

		if (ld->type == XRT_LAYER_CYLINDER || ld->type == XRT_LAYER_EQUIRECT2) {
			const uint32_t type_code = (ld->type == XRT_LAYER_CYLINDER)
			                               ? (uint32_t)RENDER_NLAYER_TYPE_CYLINDER
			                               : (uint32_t)RENDER_NLAYER_TYPE_EQUIRECT2;
			layer_types |= (type_code << (l * RENDER_NLAYER_TYPE_BITS));

			// Both types: 1 source image, view-invariant array_index,
			// same sampler/post-transform-rect treatment.
			uint32_t array_index;
			uint32_t image_index;
			const struct xrt_normalized_rect *src_rect;
			if (ld->type == XRT_LAYER_CYLINDER) {
				array_index = ld->cylinder.sub.array_index;
				image_index = ld->cylinder.sub.image_index;
				src_rect = &ld->cylinder.sub.norm_rect;
			} else {
				array_index = ld->equirect2.sub.array_index;
				image_index = ld->equirect2.sub.image_index;
				src_rect = &ld->equirect2.sub.norm_rect;
			}
			const struct comp_swapchain_image *image = get_layer_image(layer, 0, image_index);
			VkImageView image_view = get_image_view(image, ld->flags, array_index);

			struct xrt_normalized_rect rect = XRT_STRUCT_INIT;
			set_post_transform_rect(ld, src_rect, false, &rect);

			for (uint32_t v = 0; v < view_count; v++) {
				uint32_t src_i = l * view_count + v;
				uint32_t ubo_i = l * XRT_MAX_VIEWS + v;
				src_samplers[src_i] = clamp_to_edge;
				src_image_views[src_i] = image_view;
				data.post_transforms[ubo_i] = rect;
				if (ld->type == XRT_LAYER_CYLINDER) {
					fill_nlayer_cylinder_data(ld, &view_mats[v], &data.wraps[ubo_i]);
				} else {
					fill_nlayer_equirect2_data(ld, &view_mats[v], &data.wraps[ubo_i]);
				}
			}
			continue;
		}

		// Projection (incl. depth variant). The timewarp destination
		// must be in the same space as the submitted pose: world
		// scanout poses for world layers, the head-relative eye pose
		// for VIEW-space layers (mixing the two reads the basis
		// difference as head motion and warps the layer out of view).
		// With timewarp off the pose maps onto itself, which still
		// bakes the submitted source FOV into the matrix.
		bool slot_timewarp = d->do_timewarp && !view_space;
		bool slot_compensated = slot_timewarp && needs_end_pose;
		// The bounds test can be skipped when border sampling already
		// does the right thing outside the FOV. The rect must cover the
		// whole image, so that out of FOV means out of image. Then one
		// of three cases: plain source alpha keeps the transparent
		// border transparent; the bottom slot doesn't care since black
		// and transparent are the same over an empty accumulator; a
		// display-covering FOV wants the classic black pull-in anyway.
		// An alpha-less swapchain format disqualifies the source-alpha
		// case: missing alpha substitutes to one, border texels
		// included.
		bool alpha_safe = ((inverted_alpha_mask >> l) & 1u) == 0u &&
		                  (ld->flags & XRT_LAYER_COMPOSITION_BLEND_TEXTURE_SOURCE_ALPHA_BIT) != 0;
		bool full_image = true;
		bool covers_display = true;
		for (uint32_t v = 0; v < view_count; v++) {
			const struct xrt_layer_projection_view_data *vd = NULL;
			if (ld->type == XRT_LAYER_PROJECTION) {
				view_index_to_projection_data(v, ld, &vd);
			} else if (ld->type == XRT_LAYER_PROJECTION_DEPTH) {
				const struct xrt_layer_depth_data *dvd = NULL;
				view_index_to_depth_data(v, ld, &vd, &dvd);
			} else {
				assert(false && "Layer type not eligible for gfx nlayer");
				return false;
			}

			const struct comp_swapchain_image *image = get_layer_image(layer, v, vd->sub.image_index);

			const struct comp_swapchain *csc =
			    (struct comp_swapchain *)comp_layer_get_swapchain(layer, v);
			if (!comp_swapchain_format_has_alpha(csc)) {
				alpha_safe = false;
			}

			struct xrt_normalized_rect rect = vd->sub.norm_rect;
			if (rect.x != 0.0f || rect.y != 0.0f || rect.w != 1.0f || rect.h != 1.0f) {
				full_image = false;
			}

			if (!fov_covers(&vd->fov, &d->views[v].fov)) {
				covers_display = false;
			}
			if (ld->flip_y) {
				rect.y += rect.h;
				rect.h = -rect.h;
			}

			uint32_t src_i = l * view_count + v;
			uint32_t ubo_i = l * XRT_MAX_VIEWS + v;
			src_samplers[src_i] = clamp_to_border_transparent;
			src_image_views[src_i] = get_image_view(image, ld->flags, vd->sub.array_index);
			// The sampling rect is folded into the matrices below;
			// post_transforms instead carries the bounds-test transform.
			render_calc_proj_bounds_transform(&rect, &data.post_transforms[ubo_i]);

			const struct xrt_pose *dst_begin;
			if (view_space) {
				dst_begin = &d->views[v].eye_pose;
			} else if (slot_timewarp) {
				dst_begin = &d->views[v].world_pose_scanout_begin;
			} else {
				dst_begin = &vd->pose;
			}
			render_calc_time_warp_matrix(                       //
			    &vd->pose,                                      //
			    &vd->fov,                                       //
			    dst_begin,                                      //
			    &data.transform_timewarp_scanout_begin[ubo_i]); //
			render_time_warp_matrix_fold_remap_and_rect(&data.transform_timewarp_scanout_begin[ubo_i], &rect);
			if (slot_compensated) {
				render_calc_time_warp_matrix(                     //
				    &vd->pose,                                    //
				    &vd->fov,                                     //
				    &d->views[v].world_pose_scanout_end,          //
				    &data.transform_timewarp_scanout_end[ubo_i]); //
				render_time_warp_matrix_fold_remap_and_rect(&data.transform_timewarp_scanout_end[ubo_i], &rect);
			}
			// Uncompensated slots only read the begin matrix.
		}
		if (full_image && covers_display && alpha_safe) {
			// Black pull-in for a source-alpha display-covering layer.
			for (uint32_t v = 0; v < view_count; v++) {
				src_samplers[l * view_count + v] = clamp_to_border_black;
			}
		}
		if (!full_image || (l > 0 && !alpha_safe && !covers_display)) {
			projection_bounds_test_mask |= 1u << l;
		}
	}


	/*
	 * Pipeline build (lazy) and per-frame allocations, both before any
	 * command writing so a failure here returns cleanly.
	 */

	// A set bit disables scanout compensation for that slot: VIEW-space
	// slots never take it, and with timewarp off or a display treated as
	// global flash nothing does. Must agree with slot_compensated above.
	const uint32_t used_slots = (1u << layer_count) - 1u;
	uint32_t scanout_compensate_layers_mask;
	if (d->do_timewarp && needs_end_pose) {
		scanout_compensate_layers_mask = view_space_slots;
	} else {
		scanout_compensate_layers_mask = used_slots;
	}

	VkPipeline pipeline = VK_NULL_HANDLE;
	ret = render_gfx_render_pass_get_or_create_nlayer_pipeline( //
	    d->target.gfx.rtr->rgrp,                                //
	    layer_count,                                            //
	    layer_types,                                            //
	    unpremultiplied_mask,                                   //
	    inverted_alpha_mask,                                    //
	    eye_hidden_mask,                                        //
	    scanout_compensate_layers_mask,                         //
	    projection_bounds_test_mask,                            //
	    d->do_distortion,                                       //
	    d->do_cac,                                              //
	    d->scanout_direction,                                   //
	    &pipeline);                                             //
	if (ret != VK_SUCCESS) {
		U_LOG_E("Failed to build gfx nlayer pipeline (N=%u, types=0x%x, unpre=0x%x, inv=0x%x, hidden=0x%x, "
		        "sc_mask=0x%x, bt_mask=0x%x, d=%d, cac=%d, scanout=%d): %d",
		        layer_count, layer_types, unpremultiplied_mask, inverted_alpha_mask, eye_hidden_mask,
		        scanout_compensate_layers_mask, projection_bounds_test_mask, (int)d->do_distortion,
		        (int)d->do_cac, (int)d->scanout_direction, ret);
		return false;
	}

	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	ret = render_gfx_mesh_nlayer_alloc_and_write( //
	    render,                                   //
	    &data,                                    //
	    layer_count,                              //
	    src_samplers,                             //
	    src_image_views,                          //
	    &descriptor_set);                         //
	if (ret != VK_SUCCESS) {
		U_LOG_E("Failed to allocate gfx nlayer UBO/descriptor set: %d", ret);
		return false;
	}


	/*
	 * Command writing: one composite draw per view.
	 */

	render_gfx_begin_target(       //
	    render,                    //
	    d->target.gfx.rtr,         //
	    &background_color_active); //

	for (uint32_t v = 0; v < view_count; v++) {
		render_gfx_begin_view(                 //
		    render,                            //
		    v,                                 // view_index
		    &d->views[v].target.viewport_data, //
		    &d->views[v].target.scissor_data); //

		render_gfx_mesh_nlayer_draw( //
		    render,                  //
		    v,                       // view_index
		    layer_count,             //
		    descriptor_set,          //
		    pipeline);               //

		render_gfx_end_view(render);
	}

	render_gfx_end_target(render);

	return true;
}

/// The warp varying array of the N-layer shaders is exactly sized per
/// variant; returns whether this dispatch's element count fits the device
/// budget. Cheap enough to run every frame — the same layer walk the fast
/// path itself does. Uses the two-ray-class upper bound so it never grants
/// a dispatch the pipeline getter's exact count would reject.
static bool
crg_nlayer_varyings_fit(const struct render_resources *r,
                        const struct comp_render_dispatch_data *d,
                        const struct comp_layer *layers,
                        uint32_t layer_count)
{
	uint32_t proj_slot_count = 0;
	for (uint32_t i = 0; i < layer_count; i++) {
		enum xrt_layer_type t = layers[i].data.type;
		if (t == XRT_LAYER_PROJECTION || t == XRT_LAYER_PROJECTION_DEPTH) {
			proj_slot_count++;
		}
	}
	uint32_t nonproj_slot_count = layer_count - proj_slot_count;

	uint32_t warp_count = render_gfx_nlayer_warp_count(   //
	    d->do_distortion && d->do_cac,                    //
	    proj_slot_count,                                  //
	    nonproj_slot_count > 2 ? 2 : nonproj_slot_count); //

	return warp_count <= r->gfx.nlayer.max_warp_count;
}


/*
 *
 * 'Exported' function(s).
 *
 */

void
comp_render_gfx_layers(struct render_gfx *render,
                       const struct comp_layer *layers,
                       uint32_t layer_count,
                       const struct comp_render_dispatch_data *d,
                       VkImageLayout transition_to)
{
	COMP_TRACE_MARKER();

	struct vk_bundle *vk = render->r->vk;
	VkResult ret;

	struct gfx_layer_state ls = XRT_STRUCT_INIT;

	// Compute MVP matrices per eye: populates gfx_layer_view_state elements in `ls`
	// from `comp_render_dispatch_data *d`
	for (uint32_t view = 0; view < d->squash_view_count; view++) {

		// Data for this view, convenience.
		const struct xrt_pose world_pose = d->views[view].world_pose_scanout_begin;
		const struct xrt_pose eye_pose = d->views[view].eye_pose;
		const struct xrt_fov new_fov = d->views[view].fov;

		// Current state we are writing to.
		struct gfx_layer_view_state *state = &ls.views[view];

		// Used to go from UV to tangent space.
		render_calc_uv_to_tangent_lengths_rect(&new_fov, &state->to_tangent);

		// Projection
		struct xrt_matrix_4x4 p;
		math_matrix_4x4_projection_vulkan_infinite_reverse(&new_fov, 0.1, &p);

		// Reused view matrix.
		struct xrt_matrix_4x4 v;

		// World
		math_matrix_4x4_view_from_pose(&world_pose, &v);
		math_matrix_4x4_multiply(&p, &v, &state->world_vp_full);
		math_matrix_4x4_inverse(&v, &state->world_v_inv_full);

		struct xrt_pose world_rot_only = {world_pose.orientation, XRT_VEC3_ZERO};
		math_matrix_4x4_view_from_pose(&world_rot_only, &v);
		math_matrix_4x4_multiply(&p, &v, &state->world_vp_rot_only);

		// Eye
		math_matrix_4x4_view_from_pose(&eye_pose, &v);
		math_matrix_4x4_multiply(&p, &v, &state->eye_vp_full);
		math_matrix_4x4_inverse(&v, &state->eye_v_inv_full);

		struct xrt_pose eye_rot_only = {eye_pose.orientation, XRT_VEC3_ZERO};
		math_matrix_4x4_view_from_pose(&eye_rot_only, &v);
		math_matrix_4x4_multiply(&p, &v, &state->eye_vp_rot_only);
	}

	/*
	 * Reserve UBOs, create descriptor sets, and fill in any data ahead of
	 * time. If we ever want to copy UBO data this lets us do that easily:
	 * write a copy command before the other gfx commands.
	 */

	assert(layer_count <= RENDER_MAX_LAYERS && "Too many layers");

	VkSampler clamp_to_edge = render->r->samplers.clamp_to_edge;
	VkSampler clamp_to_border_black = render->r->samplers.clamp_to_border_black;

	for (uint32_t view = 0; view < d->squash_view_count; view++) {

		// Source for data and written to as well, read and write.
		struct gfx_layer_view_state *state = &ls.views[view];

		for (uint32_t i = 0; i < layer_count; i++) {
			const struct xrt_layer_data *data = &layers[i].data;
			if (!is_layer_view_visible(data, view)) {
				continue;
			}

			switch (data->type) {
			case XRT_LAYER_CYLINDER:
				ret = do_cylinder_layer(   //
				    render,                //
				    &layers[i],            //
				    view,                  // view_index
				    clamp_to_edge,         //
				    clamp_to_border_black, //
				    state);                //
				VK_CHK_WITH_GOTO(ret, "do_cylinder_layer", err_layer);
				break;
			case XRT_LAYER_EQUIRECT2:
				ret = do_equirect2_layer(  //
				    render,                //
				    &layers[i],            //
				    view,                  // view_index
				    clamp_to_edge,         //
				    clamp_to_border_black, //
				    state);                //
				VK_CHK_WITH_GOTO(ret, "do_equirect2_layer", err_layer);
				break;
			case XRT_LAYER_PROJECTION:
			case XRT_LAYER_PROJECTION_DEPTH:
				ret = do_projection_layer( //
				    render,                //
				    &layers[i],            //
				    view,                  // view_index
				    clamp_to_edge,         //
				    clamp_to_border_black, //
				    state);                //
				VK_CHK_WITH_GOTO(ret, "do_projection_layer", err_layer);
				break;
			case XRT_LAYER_QUAD:
				ret = do_quad_layer(       //
				    render,                //
				    &layers[i],            //
				    view,                  // view_index
				    clamp_to_edge,         //
				    clamp_to_border_black, //
				    state);                //
				VK_CHK_WITH_GOTO(ret, "do_quad_layer", err_layer);
				break;
			default: break;
			}
		}
	}


	/*
	 * Do command writing here.
	 */

	const VkClearColorValue *color = layer_count == 0 ? &background_color_idle : &background_color_active;

	for (uint32_t view = 0; view < d->squash_view_count; view++) {

		// Convenience.
		const struct render_viewport_data *viewport_data = &d->views[view].squash.viewport_data;

		render_gfx_begin_target(           //
		    render,                        //
		    d->views[view].squash.gfx.rtr, //
		    color);                        //

		render_gfx_begin_view( //
		    render,            //
		    view,              // view_index
		    viewport_data,     // viewport_data
		    viewport_data);    // scissor_data

		// Only source for data here, read only.
		const struct gfx_layer_view_state *state = &ls.views[view];

		for (uint32_t i = 0; i < state->layer_count; i++) {
			switch (state->types[i]) {
			case XRT_LAYER_CYLINDER:
				render_gfx_layer_cylinder(          //
				    render,                         //
				    state->premultiplied_alphas[i], //
				    state->descriptor_sets[i]);     //
				break;
			case XRT_LAYER_EQUIRECT2:
				render_gfx_layer_equirect2(         //
				    render,                         //
				    state->premultiplied_alphas[i], //
				    state->descriptor_sets[i]);     //
				break;
			case XRT_LAYER_PROJECTION:
			case XRT_LAYER_PROJECTION_DEPTH:
				render_gfx_layer_projection(        //
				    render,                         //
				    state->premultiplied_alphas[i], //
				    state->descriptor_sets[i]);     //
				break;
			case XRT_LAYER_QUAD:
				render_gfx_layer_quad(              //
				    render,                         //
				    state->premultiplied_alphas[i], //
				    state->descriptor_sets[i]);     //
				break;
			default: break;
			}
		}

		render_gfx_end_view(render);

		render_gfx_end_target(render);
	}


	cmd_barrier_view_squash_images(                    //
	    render->r->vk,                                 //
	    d,                                             //
	    render->r->cmd,                                // cmd
	    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,          // src_access_mask
	    VK_ACCESS_SHADER_READ_BIT,                     // dst_access_mask
	    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,      // transition_from
	    transition_to,                                 //
	    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // src_stage_mask
	    VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);        // dst_stage_mask

	return;

err_layer:
	// Allocator reset at end of frame, nothing to clean up.
	VK_ERROR(vk, "Layer processing failed, that shouldn't happen!");
}



void
comp_render_gfx_dispatch(struct render_gfx *render,
                         const struct comp_layer *layers,
                         const uint32_t layer_count,
                         const struct comp_render_dispatch_data *d)
{
	if (!d->target.initialized) {
		VK_ERROR(render->r->vk, "Target hasn't been initialized, not rendering anything.");
		assert(d->target.initialized);
		return;
	}

	// Convenience.
	bool fast_path = d->fast_path;

	// Only used if fast_path is true.
	const struct comp_layer *layer = &layers[0];

	// Consistency check.
	assert(!fast_path || layer_count >= 1);

	// We want to read from the images afterwards.
	VkImageLayout transition_to = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

	// Gfx N-layer fast path (XRT_GFX_NLAYER, experimental): all layers
	// composited by a single fragment shader, one draw per view, directly
	// into the target. The compositor's fast_path eligibility already
	// rejected features the shader can't represent.
	bool use_nlayer = d->target.gfx.rtr->rgrp->nlayer.enabled &&                   //
	                  fast_path &&                                                 //
	                  layer_count >= 1 &&                                          //
	                  layer_count <= render->r->gfx.nlayer.effective_nlayer_max && //
	                  all_layers_are_nlayer_eligible(layers, layer_count) &&       //
	                  crg_nlayer_varyings_fit(render->r, d, layers, layer_count);  //

	if (use_nlayer && crg_nlayer_fast_path(render, d, layers, layer_count)) {
		// Fast path, done.

	} else if (fast_path && layer_count == 1 && layer->data.type == XRT_LAYER_PROJECTION) {
		// Fast path.
		const struct xrt_layer_projection_data *proj = &layer->data.proj;
		const struct xrt_layer_projection_view_data *vds[XRT_MAX_VIEWS];
		for (uint32_t view = 0; view < d->target.view_count; ++view) {
			vds[view] = &proj->v[view];
		}
		crg_distortion_fast_path( //
		    render,               //
		    d,                    //
		    layer,                //
		    vds);                 //

	} else if (fast_path && layer_count == 1 && layer->data.type == XRT_LAYER_PROJECTION_DEPTH) {
		// Fast path.
		const struct xrt_layer_projection_depth_data *depth = &layer->data.depth;
		const struct xrt_layer_projection_view_data *vds[XRT_MAX_VIEWS];
		for (uint32_t view = 0; view < d->target.view_count; ++view) {
			vds[view] = &depth->v[view];
		}
		crg_distortion_fast_path( //
		    render,               //
		    d,                    //
		    layer,                //
		    vds);                 //

	} else if (layer_count > 0) {
		// Graphics layer squasher
		if (use_nlayer) {
			U_LOG_W("gfx nlayer fast path pipeline build failed; falling back to layer squasher.");
		} else if (fast_path) {
			U_LOG_W("Wanted fast path but no projection layer, falling back to layer squasher.");
		}

		/*
		 * Layer squashing.
		 */
		comp_render_gfx_layers( //
		    render,             //
		    layers,             //
		    layer_count,        //
		    d,                  //
		    transition_to);     //

		/*
		 * Distortion.
		 */
		crg_distortion_after_squash( //
		    render,                  //
		    d);

	} else {
		// Just clear the screen
		crg_clear_output( //
		    render,       //
		    d);           //
	}
}
