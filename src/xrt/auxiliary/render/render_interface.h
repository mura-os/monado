// Copyright 2019-2023, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The NEW compositor rendering code header.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_render
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"

#include "vk/vk_helpers.h"
#include "vk/vk_cmd_pool.h"

#include "shaders/render_shaders_interface.h"


struct render_distortion_pipeline_cache;
struct render_layer_pipeline_cache;
struct render_blit_ms_pipeline_cache;


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * @defgroup aux_render Compositor render code
 * @ingroup comp
 *
 * @brief Rendering helper that is used by the compositor to render.
 */

/*!
 * @addtogroup aux_render
 * @{
 */

/*
 *
 * Defines
 *
 */

/*!
 * The value `minUniformBufferOffsetAlignment` is defined by the Vulkan spec as
 * having a max value of 256. Use this value to safely figure out sizes and
 * alignment of UBO sub-allocation. It is also the max for 'nonCoherentAtomSize`
 * which if we need to do flushing is what we need to align UBOs to.
 *
 * https://registry.khronos.org/vulkan/specs/1.3-extensions/man/html/VkPhysicalDeviceLimits.html
 * https://registry.khronos.org/vulkan/specs/1.3-extensions/html/vkspec.html#limits-minmax
 */
#define RENDER_ALWAYS_SAFE_UBO_ALIGNMENT (256)

/*!
 * Max number of layers for layer squasher, can be different from
 * @ref XRT_MAX_LAYERS as the render module is separate from the compositor.
 * It has to match RENDER_MAX_LAYERS in the layer.comp shader.
 */
#define RENDER_MAX_LAYERS (128)

/*!
 * The maximum number samplers per view that can be used by the compute shader
 * for layer composition (layer.comp)
 */
#define RENDER_CS_MAX_SAMPLERS_PER_VIEW 2

/*!
 * Max number of images that can be given at a single time to the layer
 * squasher in a single dispatch.
 */
#define RENDER_MAX_IMAGES_SIZE (RENDER_MAX_LAYERS * RENDER_CS_MAX_SAMPLERS_PER_VIEW)

/*!
 * Maximum number of times that the layer squasher shader can run per
 * @ref render_compute. Since you run the layer squasher shader once per view
 * this is essentially the same as number of views. But if you were to do
 * two or more different compositions it is not the maximum number of views per
 * composition (which is this number divided by number of composition).
 */
#define RENDER_MAX_LAYER_RUNS_SIZE (XRT_MAX_VIEWS)
#define RENDER_MAX_LAYER_RUNS_COUNT(RENDER_RESOURCES) (RENDER_RESOURCES->view_count)

/*!
 * Compile-time upper bound on the N-layer fast paths' layer count, for both
 * the compute and the gfx variant. Sizes the static arrays of per-N
 * descriptor set layouts / pipeline layouts / descriptor sets, and is the
 * absolute ceiling for the shaders' UBOs. The actual runtime caps are the
 * `effective_nlayer_max` fields (see structs
 * `render_resources::compute::distortion_nlayer` and
 * `render_resources::gfx::nlayer`), which may be lower when device per-stage
 * descriptor limits would not host this many. Must match MAX_NLAYER in
 * distortion_nlayer.comp and mesh_nlayer.inc.glsl.
 */
#define RENDER_NLAYER_MAX (8)

/*!
 * Capacity of the per-variant pipeline caches for the N-layer fast paths
 * (the compute one in @ref render_resources, the gfx one per
 * @ref render_gfx_render_pass). Variant key is (layer_count, layer_types,
 * layer_unpremult_mask, layer_inverted_alpha_mask,
 * scanout_compensate_layers_mask, do_distortion, do_cac, scanout_direction);
 * apps typically cycle through <30 patterns per session
 * even with the alpha mask axes, so this is well over-
 * provisioned. The cap exists as a defensive guardrail against runaway variant
 * creation, not as a tight budget. Linear search; lookup at this size is still
 * sub-microsecond and dwarfed by per-frame Vulkan submit overhead. If it fills
 * up render_resources_get_or_create_nlayer_pipeline returns
 * VK_ERROR_OUT_OF_DEVICE_MEMORY.
 */
#define RENDER_NLAYER_PIPELINE_CACHE_CAP (1024)

//! Distortion image dimension in pixels
#define RENDER_DISTORTION_IMAGE_DIMENSIONS (128)

//! How many distortion images we have, one for each channel (3 rgb) and per view.
#define RENDER_DISTORTION_IMAGES_SIZE (3 * XRT_MAX_VIEWS)
#define RENDER_DISTORTION_IMAGES_COUNT(RENDER_RESOURCES) (3 * RENDER_RESOURCES->view_count)

//! The binding that the layer projection and quad shader have their UBO on.
#define RENDER_BINDING_LAYER_SHARED_UBO 0

//! The binding that the shared layer fragment shader has its source on.
#define RENDER_BINDING_LAYER_SHARED_SRC 1

/*!
 * Default inset blend edge width for layer projection edge blending.
 * Must match the default in layer.comp (k_inset_blend_edge).
 */
#define RENDER_LAYER_DEFAULT_INSET_BLEND_EDGE (0.05f)

/*!
 * The maximum number samplers per view that can be used by the compute shader
 * for layer composition (layer.comp)
 */
#define RENDER_CS_MAX_SAMPLERS_PER_VIEW 2


/*
 *
 * N-layer fast path shared types, used by both the compute
 * (distortion_nlayer.comp) and the gfx (mesh_nlayer.frag) variant.
 *
 */

/*!
 * Per-slot layer-type encoding for the N-layer fast paths' @c layer_types
 * spec constant. 2 bits per slot; must match the LAYER_TYPE_* defines in
 * distortion_nlayer.comp and mesh_nlayer.frag.
 */
enum render_nlayer_type
{
	RENDER_NLAYER_TYPE_PROJECTION = 0,
	RENDER_NLAYER_TYPE_QUAD = 1,
	RENDER_NLAYER_TYPE_CYLINDER = 2,
	RENDER_NLAYER_TYPE_EQUIRECT2 = 3,
};

//! Bits per slot in the packed @c layer_types spec constant.
#define RENDER_NLAYER_TYPE_BITS (2)

/*!
 * Per-(layer, view) quad data for the N-layer fast paths. Layout must match
 * the `QuadData` struct in distortion_nlayer.comp and mesh_nlayer.inc.glsl
 * under std140 (each vec3 padded to 16, mat4 at offset 32, vec2 + 8 bytes
 * tail padding → 112 bytes total).
 */
struct render_compute_nlayer_quad_data
{
	struct xrt_vec3 position;
	float _pad0;
	struct xrt_vec3 normal;
	float _pad1;
	struct xrt_matrix_4x4 inverse_transform;
	struct xrt_vec2 extent;
	float _pad2[2];
};

/*!
 * Per-(layer, view) cylinder / equirect2 ("wrap") data for the N-layer fast
 * paths. Layout must match the `WrapData` struct in distortion_nlayer.comp
 * and mesh_nlayer.inc.glsl under std140 (mat4 at offset 0, vec4 at offset 64
 * → 80 bytes total).
 *
 * `mv_inverse` = model_inv * view_inv. `params` is interpreted per-type:
 *   cylinder:  (radius, central_angle, aspect_ratio, _)
 *   equirect2: (radius, central_horizontal_angle, upper_vertical_angle,
 *               lower_vertical_angle)
 */
struct render_compute_nlayer_wrap_data
{
	struct xrt_matrix_4x4 mv_inverse;
	float params[4];
};


/*
 *
 * Util functions.
 *
 */

/*!
 * Determines the maximum number of compositor layers supported based on Vulkan
 * device limits and the composition path being used.
 *
 * @param vk                 Vulkan bundle containing device properties
 * @param use_compute        True if using compute pipeline path, false for graphics
 * @param desired_max_layers Maximum layers requested by the compositor
 * @return                   Actual maximum layers supported, clamped by device limits (minimum 16)
 *
 */
uint32_t
render_max_layers_capable(const struct vk_bundle *vk, bool use_compute, uint32_t desired_max_layers);

/*!
 * Create a simplified projection matrix for timewarp.
 */
void
render_calc_time_warp_projection(const struct xrt_fov *fov, struct xrt_matrix_4x4 *result);

/*!
 * Calculates a timewarp matrix which takes in NDC coords and gives out results
 * in [-1, 1] space that needs a perspective divide.
 */
void
render_calc_time_warp_matrix(const struct xrt_pose *src_pose,
                             const struct xrt_fov *src_fov,
                             const struct xrt_pose *new_pose,
                             struct xrt_matrix_4x4 *matrix);

/*!
 * Folds the [-1, 1] -> [0, 1] remap and a layer's sub-image rect into a
 * matrix from @ref render_calc_time_warp_matrix, so xy / w after the
 * multiply is already the final source UV.
 */
void
render_time_warp_matrix_fold_remap_and_rect(struct xrt_matrix_4x4 *matrix, const struct xrt_normalized_rect *rect);

/*!
 * Builds the scale/bias (out.w/h, out.x/y) mapping a projection source UV
 * onto [-1, 1]^2 over the layer's sub-image rect, for the shader bounds
 * test `max(|uv * scale + bias|) <= 1`. Sign-symmetric, so a flip_y-negated
 * height just works; a degenerate rect maps to always-outside.
 */
void
render_calc_proj_bounds_transform(const struct xrt_normalized_rect *rect, struct xrt_normalized_rect *out_transform);

/*!
 * This function constructs a transformation in the form of a normalized rect
 * that lets you go from a UV coordinate on a projection plane to the a point on
 * the tangent plane. An example is that the UV coordinate `(0, 0)` would be
 * transformed to `(tan(angle_left), tan(fov.angle_up))`. The tangent plane (aka
 * tangent space) is really the tangent of the angle, aka length at unit distance.
 *
 * For the trivial case of an fov with 45 degrees angles, that is where the
 * tangent length are `1` (aka `tan(45)`), the transformation would go from
 * `[0 .. 1]` to `[-1 .. 1]` the expected returns are `x = -1`, `y = -1`,
 * `w = 2` and `h = 2`.
 *
 * param      fov      The fov of the projection image.
 * param[out] out_rect Transformation from UV to tangent lengths.
 */
void
render_calc_uv_to_tangent_lengths_rect(const struct xrt_fov *fov, struct xrt_normalized_rect *out_rect);


/*
 *
 * Buffer
 *
 */

/*!
 * Helper struct holding a buffer and its memory.
 */
struct render_buffer
{
	//! Backing memory.
	VkDeviceMemory memory;

	//! Buffer.
	VkBuffer buffer;

	//! Size requested for the buffer.
	VkDeviceSize size;

	//! Size of the memory allocation.
	VkDeviceSize allocation_size;

	//! Alignment of the buffer.
	VkDeviceSize alignment;

	void *mapped;
};

/*!
 * Initialize a buffer.
 */
VkResult
render_buffer_init(struct vk_bundle *vk,
                   struct render_buffer *buffer,
                   VkBufferUsageFlags usage_flags,
                   VkMemoryPropertyFlags memory_property_flags,
                   VkDeviceSize size);

/*!
 * Initialize a buffer, making it exportable.
 */
VkResult
render_buffer_init_exportable(struct vk_bundle *vk,
                              struct render_buffer *buffer,
                              VkBufferUsageFlags usage_flags,
                              VkMemoryPropertyFlags memory_property_flags,
                              VkDeviceSize size);

/*!
 * Frees all resources that this buffer has, but does not free the buffer itself.
 */
void
render_buffer_fini(struct vk_bundle *vk, struct render_buffer *buffer);

/*!
 * Maps the memory, sets render_buffer::mapped to the memory.
 */
VkResult
render_buffer_map(struct vk_bundle *vk, struct render_buffer *buffer);

/*!
 * Unmaps the memory.
 */
void
render_buffer_unmap(struct vk_bundle *vk, struct render_buffer *buffer);

/*!
 * Maps the buffer, and copies the given data to the buffer.
 */
VkResult
render_buffer_map_and_write(struct vk_bundle *vk, struct render_buffer *buffer, void *data, VkDeviceSize size);

/*!
 * Writes the given data to the buffer, will map it temporarily if not mapped.
 */
VkResult
render_buffer_write(struct vk_bundle *vk, struct render_buffer *buffer, void *data, VkDeviceSize size);


/*
 *
 * Sub-alloc.
 *
 */

/*!
 * Per frame sub-allocation into a buffer, used to reduce the number of UBO
 * objects we need to create. There is no way to free a sub-allocation, this is
 * done implicitly at the end of the frame when @ref render_sub_alloc_tracker is
 * zeroed out.
 *
 * @see render_sub_alloc_tracker
 */
struct render_sub_alloc
{
	/*!
	 * The buffer this is allocated from, it is the caller's responsibility
	 * to keep it alive for as long as the sub-allocation is used.
	 */
	VkBuffer buffer;

	//! Size of sub-allocation.
	VkDeviceSize size;

	//! Offset into buffer.
	VkDeviceSize offset;
};

/*!
 * A per-frame tracker of sub-allocation out of a buffer, used to reduce the
 * number of UBO objects we need to create. This code is designed with one
 * constraint in mind, that the lifetime of a sub-allocation is only for one
 * frame and is discarded at the end of it, but also alive for the entire frame.
 * This removes the need to free individual sub-allocation, or even track them
 * beyond filling the UBO data and descriptor sets.
 *
 * @see render_sub_alloc
 */
struct render_sub_alloc_tracker
{
	/*!
	 * The buffer to allocate from, it is the caller's responsibility to keep
	 * it alive for as long as the sub-allocations are in used.
	 */
	VkBuffer buffer;

	//! Start of memory, if buffer was mapped with initialised.
	void *mapped;

	//! Total size of buffer.
	VkDeviceSize total_size;

	//! Currently used memory.
	VkDeviceSize used;
};

/*!
 * Init a @ref render_sub_alloc_tracker struct from a @ref render_buffer, the
 * caller is responsible for keeping @p buffer alive while the sub allocator
 * is being used.
 */
void
render_sub_alloc_tracker_init(struct render_sub_alloc_tracker *rsat, struct render_buffer *buffer);

/*!
 * Allocate enough memory (with constraints of UBOs) of @p size, return the
 * pointer to the mapped memory or null if the buffer wasn't allocated.
 */
XRT_CHECK_RESULT VkResult
render_sub_alloc_ubo_alloc_and_get_ptr(struct vk_bundle *vk,
                                       struct render_sub_alloc_tracker *rsat,
                                       VkDeviceSize size,
                                       void **out_ptr,
                                       struct render_sub_alloc *out_rsa);

/*!
 * Allocate enough memory (with constraints of UBOs) to hold the memory in @p ptr
 * and copy that memory to the buffer using the CPU.
 */
XRT_CHECK_RESULT VkResult
render_sub_alloc_ubo_alloc_and_write(struct vk_bundle *vk,
                                     struct render_sub_alloc_tracker *rsat,
                                     const void *ptr,
                                     VkDeviceSize size,
                                     struct render_sub_alloc *out_rsa);


/*
 *
 * Resources
 *
 */

/*!
 * Holds all pools and static resources for rendering.
 */
struct render_resources
{
	//! The count of views that we are rendering to.
	uint32_t view_count;

	//! Vulkan resources.
	struct vk_bundle *vk;

	/*
	 * Loaded resources.
	 */

	//! All shaders loaded.
	struct render_shaders *shaders;


	/*
	 * Shared pools and caches.
	 */

	//! Pool used for distortion image uploads.
	struct vk_cmd_pool distortion_pool;

	//! Shared for all rendering.
	VkPipelineCache pipeline_cache;

	VkCommandPool cmd_pool;

	VkQueryPool query_pool;


	/*
	 * Static
	 */

	//! Command buffer for recording everything.
	VkCommandBuffer cmd;

	struct
	{
		//! Sampler for mock/null images.
		VkSampler mock;

		//! Sampler that repeats the texture in all directions.
		VkSampler repeat;

		//! Sampler that clamps the coordinates to the edge in all directions.
		VkSampler clamp_to_edge;

		//! Sampler that clamps color samples to black in all directions.
		VkSampler clamp_to_border_black;

		/*!
		 * Same but with a transparent border, for gfx N-layer projection
		 * slots: an out-of-image fetch composites as nothing.
		 */
		VkSampler clamp_to_border_transparent;
	} samplers;

	struct
	{
		//! Pool for shaders that uses one ubo and sampler.
		VkDescriptorPool ubo_and_src_descriptor_pool;

		/*!
		 * Shared UBO buffer that we sub-allocate out of, this is to
		 * have fewer buffers that the kernel needs to validate on
		 * command submission time.
		 *
		 * https://registry.khronos.org/vulkan/site/guide/latest/memory_allocation.html
		 */
		struct render_buffer shared_ubo;

		struct
		{
			struct
			{
				//! For projection and quad layer.
				VkDescriptorSetLayout descriptor_set_layout;

				//! For projection and quad layer.
				VkPipelineLayout pipeline_layout;
			} shared;
		} layer;

		/*!
		 * Gfx N-layer fast path (XRT_GFX_NLAYER): composites up to
		 * effective_nlayer_max layers in a single fragment shader
		 * (mesh_nlayer.vert + mesh_nlayer.frag), one draw per view,
		 * mirroring the compute path's distortion_nlayer.comp.
		 *
		 * Like @ref render_resources::compute::distortion_nlayer this
		 * keeps one descriptor set layout per layer_count value:
		 * binding 0 holds exactly `layer_count * view_count` combined
		 * image samplers (fragment stage, flat-indexed as
		 * [layer * view_count + view]), binding 1 the shared UBO
		 * (vertex + fragment). Each pipeline layout adds a single uint
		 * push constant carrying the view index — the gfx stand-in for
		 * the compute shader's gl_GlobalInvocationID.z. Pipelines
		 * depend on the render pass and live in
		 * @ref render_gfx_render_pass::nlayer.
		 */
		struct
		{
			//! XRT_GFX_NLAYER env option, on by default.
			bool enabled;

			/*!
			 * Maximum N actually supported on this device, in
			 * [0, RENDER_NLAYER_MAX]. The fragment stage binds
			 * `N * view_count` combined image samplers, so the
			 * budget is the smaller per-stage sampler /
			 * sampled-image limit divided by view_count. Zero when
			 * the path is disabled; then no layouts/pool exist.
			 *
			 * The vertex->fragment interface bounds each variant
			 * too, but that need depends on the variant — check
			 * @ref render_gfx_nlayer_warp_count against
			 * max_warp_count per dispatch.
			 */
			uint32_t effective_nlayer_max;

			/*!
			 * Device budget for the warp varying array element
			 * count. One whole location (4 components) per array
			 * element — elements consume whole locations before
			 * any driver packing. One vertex location is reserved
			 * for gl_Position, which counts against the vertex
			 * output limit only; the fragment stage declares no
			 * built-in inputs.
			 */
			uint32_t max_warp_count;

			VkDescriptorSetLayout descriptor_set_layouts[RENDER_NLAYER_MAX];
			VkPipelineLayout pipeline_layouts[RENDER_NLAYER_MAX];

			//! Pool for the per-frame descriptor sets, reset each frame.
			VkDescriptorPool descriptor_pool;
		} nlayer;
	} gfx;

	struct
	{
		//! The binding index for the source texture.
		uint32_t src_binding;

		//! The binding index for the UBO.
		uint32_t ubo_binding;

		//! Descriptor set layout for mesh distortion.
		VkDescriptorSetLayout descriptor_set_layout;

		//! Pipeline layout used for mesh.
		VkPipelineLayout pipeline_layout;

		struct render_buffer vbo;
		struct render_buffer ibo;

		uint32_t vertex_count;
		uint32_t index_counts[XRT_MAX_VIEWS];
		uint32_t stride;
		uint32_t index_offsets[XRT_MAX_VIEWS];
		uint32_t index_count_total;

		//! Info UBOs.
		struct render_buffer ubos[XRT_MAX_VIEWS];
	} mesh;

	/*!
	 * Used as a default image empty image when none is given or to pad
	 * out fixed sized descriptor sets.
	 */
	struct
	{
		struct
		{
			VkImage image;
			VkImageView image_view;
			VkDeviceMemory memory;
		} color;
	} mock;

	struct
	{
		//! Descriptor pool for compute work.
		VkDescriptorPool descriptor_pool;

		//! The source projection view binding point.
		uint32_t src_binding;

		//! Image storing the distortion.
		uint32_t distortion_binding;

		//! Writing the image out too.
		uint32_t target_binding;

		//! Uniform data binding.
		uint32_t ubo_binding;

		struct
		{
			//! Descriptor set layout for compute.
			VkDescriptorSetLayout descriptor_set_layout;

			//! Pipeline layout used for compute distortion.
			VkPipelineLayout pipeline_layout;

			//! Doesn't depend on target so is static.
			VkPipeline non_timewarp_pipeline;

			//! Doesn't depend on target so is static.
			VkPipeline timewarp_pipeline;

			//! Get-or-create cache for layer.comp specialization variants.
			struct render_layer_pipeline_cache *pipeline_cache;

			//! Size of combined image sampler array
			uint32_t image_array_size;

			//! Target info.
			struct render_buffer ubos[RENDER_MAX_LAYER_RUNS_SIZE];
		} layer;

		struct
		{
			//! Descriptor set layout for compute distortion.
			VkDescriptorSetLayout descriptor_set_layout;

			//! Pipeline layout used for compute distortion, shared with clear.
			VkPipelineLayout pipeline_layout;

			//! Cached non-timewarp variant from @ref pipeline_cache.
			VkPipeline pipeline;

			//! Cached timewarp variant from @ref pipeline_cache.
			VkPipeline timewarp_pipeline;

			//! Get-or-create cache for distortion.comp specialization variants.
			struct render_distortion_pipeline_cache *pipeline_cache;

			//! Target info.
			struct render_buffer ubo;
		} distortion;

		//! N-layer fast path: composites up to effective_nlayer_max layers of
		//! projection / quad / cylinder / equirect2 type in one dispatch.
		//! Projection layers get per-layer timewarp + geometric distortion,
		//! optionally with chromatic-aberration correction; the others get a
		//! view-space ray-shape intersection against their primitive. Writes
		//! straight to the swapchain. At N=1 with a pure projection layer it
		//! reproduces the existing 1-layer fast path's output; everything else
		//! replaces the layer squasher's scratch round trip.
		//!
		//! Per-N descriptor set layouts: layer_count is a spec const that
		//! varies between pipeline variants, so each variant's shader declares
		//! a different `sources` array size. To avoid mock-filling unused
		//! slots in a max-sized descriptor set, we maintain one descriptor
		//! set layout per supported layer_count, with binding 0's
		//! `descriptorCount = layer_count * view_count`. Indices are
		//! [layer_count - 1] (so N=1 lives at [0]).
		struct
		{
			//! Maximum N actually supported on this device, in [0, RENDER_NLAYER_MAX].
			//! Computed at init from the device's per-stage descriptor limits:
			//! the path needs `N * view_count + 3 * view_count` combined image
			//! samplers per stage, so the budget is
			//! `(min(max_per_stage_descriptor_samplers,
			//!       max_per_stage_descriptor_sampled_images) - 3 * view_count)
			//!  / view_count`, clamped against RENDER_NLAYER_MAX. Variants with
			//! N > effective_nlayer_max are rejected by the eligibility check
			//! and fall through to the squasher. Zero means the device can't
			//! host even N=1; the path is disabled and only the static fields
			//! below are zero-initialised — pool/sets/layouts are not created.
			uint32_t effective_nlayer_max;

			VkDescriptorSetLayout descriptor_set_layouts[RENDER_NLAYER_MAX];
			VkPipelineLayout pipeline_layouts[RENDER_NLAYER_MAX];

			//! Per-variant pipeline cache, built lazily on first use.
			//! Linear search since the active set is small (typically <50
			//! variants over a session) and lookup is off the per-pixel
			//! hot path.
			struct
			{
				//! Two packed words, see nlayer_pipeline_key().
				uint64_t key[2];
				VkPipeline pipeline;
			} pipelines[RENDER_NLAYER_PIPELINE_CACHE_CAP];
			uint32_t pipeline_count;

			//! Single pool serving all effective_nlayer_max per-N descriptor sets.
			VkDescriptorPool descriptor_pool;
			VkDescriptorSet descriptor_sets[RENDER_NLAYER_MAX];

			struct render_buffer ubo;
		} distortion_nlayer;

		struct
		{
			//! Doesn't depend on target so is static.
			VkPipeline pipeline;

			//! Target info.
			struct render_buffer ubo;

			//! @todo other resources
		} clear;
	} compute;

	struct
	{
		//! Transform to go from UV to tangle angles.
		struct xrt_normalized_rect uv_to_tanangle[XRT_MAX_VIEWS];

		//! Backing memory to distortion images.
		VkDeviceMemory device_memories[RENDER_DISTORTION_IMAGES_SIZE];

		//! Distortion images.
		VkImage images[RENDER_DISTORTION_IMAGES_SIZE];

		//! The views into the distortion images.
		VkImageView image_views[RENDER_DISTORTION_IMAGES_SIZE];

		//! Whether distortion images have been pre-rotated 90 degrees.
		bool pre_rotated;
	} distortion;
};

/*!
 * Allocate pools and static resources.
 *
 * @ingroup comp_main
 *
 * @public @memberof render_resources
 */
bool
render_resources_init(struct render_resources *r,
                      struct render_shaders *shaders,
                      struct vk_bundle *vk,
                      struct xrt_device *xdev);

/*!
 * Free all pools and static resources, does not free the struct itself.
 *
 * @public @memberof render_resources
 */
void
render_resources_fini(struct render_resources *r);

/*!
 * Lazy-build helper for the N-layer fast path. Returns a compute pipeline
 * specialized to (@p layer_count, @p layer_types, @p unpremult_mask,
 * @p inverted_alpha_mask, @p eye_hidden_mask,
 * @p scanout_compensate_layers_mask, @p do_distortion, @p do_cac,
 * @p scanout_direction), creating it on first request and caching it under
 * @c r->compute.distortion_nlayer. Must be called between
 * @ref render_compute_begin and @ref render_compute_end for the active frame,
 * on the compositor thread.
 *
 * @p unpremult_mask / @p inverted_alpha_mask: 1 bit per slot, one spec-const mask
 * per alpha feature; a zero mask folds its path away.
 *
 * @p eye_hidden_mask: 2 bits per slot, set means HIDDEN in the corresponding
 * view. C side encodes (XRT_LAYER_EYE_VISIBILITY_BOTH XOR visibility) so
 * BOTH = 0 and the common case fits a single cache entry (mask = 0).
 *
 * @p scanout_compensate_layers_mask: 1 bit per slot, set = no scanout
 * (begin -> end pose) compensation for that slot; replaces the shader-level
 * do_timewarp. Canonicalized before lookup: bits above @p layer_count are
 * cleared and @p scanout_direction folds to NONE iff no slot is
 * compensated, so identical dispatches share one pipeline.
 *
 * A set bit in @p projection_bounds_test_mask makes that projection slot
 * bounds-test its source UVs (spec id 1). Only clear a bit when border
 * sampling already handles out-of-FOV correctly for the slot: the rect
 * covers the whole image, and it is bottom-most, blends with plain source
 * alpha, or covers the display FOV. Bits on non-projection slots are
 * canonicalized away.
 *
 * @public @memberof render_resources
 */
VkResult
render_resources_get_or_create_nlayer_pipeline(struct render_resources *r,
                                               uint32_t layer_count,
                                               uint32_t layer_types,
                                               uint32_t unpremult_mask,
                                               uint32_t inverted_alpha_mask,
                                               uint32_t eye_hidden_mask,
                                               uint32_t scanout_compensate_layers_mask,
                                               uint32_t projection_bounds_test_mask,
                                               bool do_distortion,
                                               bool do_cac,
                                               enum xrt_scanout_direction scanout_direction,
                                               VkPipeline *out_pipeline);

/*!
 * Creates or recreates the compute distortion textures if necessary.
 *
 * @see render_distortion_images_fini
 * @public @memberof render_resources
 */
bool
render_distortion_images_ensure(struct render_resources *r,
                                struct vk_bundle *vk,
                                struct xrt_device *xdev,
                                bool pre_rotate);

/*!
 * Free distortion images.
 *
 * @see render_distortion_images_ensure
 * @public @memberof render_resources
 */
void
render_distortion_images_fini(struct render_resources *r);

/*!
 * Returns the timestamps for when the latest GPU work started and stopped that
 * was submitted using @ref render_gfx or @ref render_compute cmd buf builders.
 *
 * Returned in the same time domain as returned by @ref os_monotonic_get_ns .
 * Behaviour for this function is undefined if the GPU has not completed before
 * calling this function, so make sure to call vkQueueWaitIdle or wait on the
 * fence that the work was submitted with have fully completed. See other
 * limitation mentioned for @ref vk_convert_timestamps_to_host_ns .
 *
 * @see vk_convert_timestamps_to_host_ns
 *
 * @public @memberof render_resources
 */
bool
render_resources_get_timestamps(struct render_resources *r, uint64_t *out_gpu_start_ns, uint64_t *out_gpu_end_ns);

/*!
 * Returns the duration for the latest GPU work that was submitted using
 * @ref render_gfx or @ref render_compute cmd buf builders.
 *
 * Behaviour for this function is undefined if the GPU has not completed before
 * calling this function, so make sure to call vkQueueWaitIdle or wait on the
 * fence that the work was submitted with have fully completed.
 *
 * @public @memberof render_resources
 */
bool
render_resources_get_duration(struct render_resources *r, uint64_t *out_gpu_duration_ns);


/*
 *
 * Scratch images.
 *
 */

/*!
 * Small helper struct to hold a scratch image, intended to be used with the
 * compute pipeline where both srgb and unorm views are needed.
 */
struct render_scratch_color_image
{
	VkDeviceMemory device_memory;
	VkImage image;
	VkImageView srgb_view;
	VkImageView unorm_view;
};

/*!
 * Helper struct to hold scratch images.
 */
struct render_scratch_images
{
	VkExtent2D extent;

	struct render_scratch_color_image color[XRT_MAX_VIEWS];
};

/*!
 * Ensure that the scratch images are created and have the given extent.
 *
 * @public @memberof render_scratch_images
 */
bool
render_scratch_images_ensure(struct render_resources *r, struct render_scratch_images *rsi, VkExtent2D extent);

/*!
 * Close all resources on the given @ref render_scratch_images.
 *
 * @public @memberof render_scratch_images
 */
void
render_scratch_images_fini(struct render_resources *r, struct render_scratch_images *rsi);


/*
 *
 * Shared between both gfx and compute.
 *
 */

/*!
 *  The pure data information about a view that the renderer is rendering to.
 */
struct render_viewport_data
{
	uint32_t x, y;
	uint32_t w, h;
};

typedef struct render_viewport_data render_scissor_data_t;

/*
 *
 * Render pass
 *
 */

/*!
 * A render pass, while not depending on a @p VkFramebuffer, does depend on the
 * format of the target image(s), and other options for the render pass. These
 * are used to create a @p VkRenderPass, all @p VkFramebuffer(s) and
 * @p VkPipeline depends on the @p VkRenderPass so hang off this struct.
 */
struct render_gfx_render_pass
{
	struct render_resources *r;

	//! The format of the image(s) we are rendering to.
	VkFormat format;

	//! Sample count for this render pass.
	VkSampleCountFlagBits sample_count;

	//! Load op used on the attachment(s).
	VkAttachmentLoadOp load_op;

	//! Final layout of the target image(s).
	VkImageLayout final_layout;

	//! Render pass used for rendering.
	VkRenderPass render_pass;

	struct
	{
		//! Pipeline layout used for mesh, without timewarp.
		VkPipeline pipeline;

		//! Pipeline layout used for mesh, with timewarp.
		VkPipeline pipeline_timewarp;
	} mesh;

	struct
	{
		VkPipeline cylinder_premultiplied_alpha;
		VkPipeline cylinder_unpremultiplied_alpha;

		VkPipeline equirect2_premultiplied_alpha;
		VkPipeline equirect2_unpremultiplied_alpha;

		VkPipeline proj_premultiplied_alpha;
		VkPipeline proj_unpremultiplied_alpha;

		VkPipeline quad_premultiplied_alpha;
		VkPipeline quad_unpremultiplied_alpha;
	} layer;

	/*!
	 * Gfx N-layer fast path: all layers composited by a single fragment
	 * shader (mesh_nlayer.vert + mesh_nlayer.frag), one draw per view,
	 * blending disabled — the gfx twin of the compute path's
	 * distortion_nlayer.comp. Only used when enabled (XRT_GFX_NLAYER env
	 * option, on by default).
	 *
	 * layer_count / layer_types / the alpha masks are spec constants, so
	 * pipelines are lazy-built per variant with the same key semantics as
	 * @ref render_resources_get_or_create_nlayer_pipeline; they depend on
	 * this render pass, which is why the cache lives here instead of in
	 * @ref render_resources.
	 */
	struct
	{
		bool enabled;

		//! Per-variant pipeline cache, built lazily on first use.
		struct
		{
			//! Two packed words, see gfx_nlayer_pipeline_key().
			uint64_t key[2];
			VkPipeline pipeline;
		} pipelines[RENDER_NLAYER_PIPELINE_CACHE_CAP];
		uint32_t pipeline_count;
	} nlayer;
};

/*!
 * Creates all resources held by the render pass.
 *
 * @public @memberof render_gfx_render_pass
 */
bool
render_gfx_render_pass_init(struct render_gfx_render_pass *rgrp,
                            struct render_resources *r,
                            VkFormat format,
                            VkAttachmentLoadOp load_op,
                            VkImageLayout final_layout);

/*!
 * Frees all resources held by the render pass, does not free the struct itself.
 *
 * @public @memberof render_gfx_render_pass
 */
void
render_gfx_render_pass_fini(struct render_gfx_render_pass *rgrp);


/*
 *
 * Rendering target
 *
 */

/*!
 * Each rendering (@ref render_gfx) render to one or more targets
 * (@ref render_gfx_target_resources), the target points to one render pass and
 * its pipelines (@ref render_gfx_render_pass). It is up to the code using
 * these to do reuse of render passes and ensure they match.
 *
 * @see comp_render_gfx
 */
struct render_gfx_target_resources
{
	//! Collections of static resources.
	struct render_resources *r;

	//! Render pass.
	struct render_gfx_render_pass *rgrp;

	//! The offset & extents of the framebuffer.
	VkRect2D render_area;

	//! Target image, used for the dynamic rendering layout barriers.
	VkImage image;

	//! Target image view, rendered into directly with dynamic rendering.
	VkImageView view;

	//! Framebuffer for this target, only used on the render pass fallback path.
	VkFramebuffer framebuffer;
};

/*!
 * Init a target resource struct, caller has to keep target alive until closed.
 *
 * @public @memberof render_gfx_target_resources
 */
bool
render_gfx_target_resources_init(struct render_gfx_target_resources *rtr,
                                 struct render_resources *r,
                                 struct render_gfx_render_pass *rgrp,
                                 VkImage target_image,
                                 VkImageView target,
                                 VkExtent2D extent);

/*!
 * Frees all resources held by the target, does not free the struct itself.
 *
 * @public @memberof render_gfx_target_resources
 */
void
render_gfx_target_resources_fini(struct render_gfx_target_resources *rtr);


/*
 *
 * Rendering
 *
 */

/*!
 * The low-level resources and operations to perform layer squashing and/or
 * mesh distortion for a single frame using graphics shaders.
 *
 * It uses a two-stage process to render a frame. This means
 * consumers iterate layers (or other operations) **twice**, within each target and view.
 * There is a preparation stage, where the uniform buffer is sub-allocated and written.
 * This must be completed for all layers before the actual draw stage begins.
 * The second stage is recording the draw commands into a command buffer.
 *
 * You must make equivalent calls in the same order between the two stages. The second stage
 * additionally has @ref render_gfx_begin_target, @ref render_gfx_end_target,
 * @ref render_gfx_begin_view, and @ref render_gfx_end_view lacked by the first stage,
 * but if you exclude those functions, the others must line up.
 *
 * Furthermore, the struct needs to be kept alive until the work has been waited on,
 * or you get validation warnings. Either wait on the `VkFence` for the submit, or call
 * `vkDeviceWaitIdle`/`vkQueueWaitIdle` on the device/queue.
 *
 * @see comp_render_gfx
 */
struct render_gfx
{
	//! Resources that we are based on.
	struct render_resources *r;

	//! Shared buffer that we sub-allocate UBOs from.
	struct render_sub_alloc_tracker ubo_tracker;

	//! The current target we are rendering to, can change during command building.
	struct render_gfx_target_resources *rtr;
};

/*!
 * Init struct and create resources needed for rendering.
 *
 * @public @memberof render_gfx
 */
bool
render_gfx_init(struct render_gfx *render, struct render_resources *r);

/*!
 * Begins the rendering, takes the vk_bundle's pool lock and leaves it locked.
 *
 * @public @memberof render_gfx
 */
bool
render_gfx_begin(struct render_gfx *render);

/*!
 * Frees any unneeded resources and ends the command buffer so it can be used,
 * also unlocks the vk_bundle's pool lock that was taken by begin.
 *
 * @public @memberof render_gfx
 */
bool
render_gfx_end(struct render_gfx *render);

/*!
 * Frees all resources held by the rendering, does not free the struct itself.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_fini(struct render_gfx *render);


/*
 *
 * Drawing
 *
 */

/*!
 * UBO data that is sent to the mesh shaders.
 *
 * @relates render_gfx
 */
struct render_gfx_mesh_ubo_data
{
	struct xrt_matrix_2x2 vertex_rot;
	struct xrt_normalized_rect post_transform;

	// Only used for timewarp.
	struct xrt_normalized_rect pre_transform;
	struct xrt_matrix_4x4 transform_scanout_begin;
	struct xrt_matrix_4x4 transform_scanout_end;
};

/*!
 * UBO for the gfx N-layer fast path (mesh_nlayer.vert + mesh_nlayer.frag):
 * per-view shared state plus per-(layer, view) transforms, flat-indexed as
 * [layer * XRT_MAX_VIEWS + view]. Must match the Config block in
 * mesh_nlayer.inc.glsl. Everything except vertex_rot mirrors
 * @ref render_compute_distortion_nlayer_ubo_data — see there for field docs,
 * including the projection-slot matrix and post_transforms semantics; the
 * compute UBO's views[] (target offsets/extents) has no gfx equivalent since
 * the viewport covers that.
 *
 * @relates render_gfx
 */
struct render_gfx_mesh_nlayer_ubo_data
{
	struct xrt_matrix_2x2 vertex_rot[XRT_MAX_VIEWS];
	struct xrt_normalized_rect pre_transforms[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 scanout_view_rot_delta[XRT_MAX_VIEWS];
	struct xrt_normalized_rect post_transforms[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_begin[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_end[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct render_compute_nlayer_quad_data quads[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct render_compute_nlayer_wrap_data wraps[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
};

/*!
 * UBO data that is sent to the layer cylinder shader.
 *
 * @relates render_gfx
 */
struct render_gfx_layer_cylinder_data
{
	struct xrt_normalized_rect post_transform;
	struct xrt_matrix_4x4 mvp;
	float radius;
	float central_angle;
	float aspect_ratio;
	float _pad;
	struct xrt_colour_rgba_f32 color_scale;
	struct xrt_colour_rgba_f32 color_bias;
};

/*!
 * UBO data that is sent to the layer equirect2 shader.
 *
 * @relates render_gfx
 */
struct render_gfx_layer_equirect2_data
{
	struct xrt_normalized_rect post_transform;
	struct xrt_matrix_4x4 mv_inverse;

	//! See @ref render_calc_uv_to_tangent_lengths_rect.
	struct xrt_normalized_rect to_tangent;

	float radius;
	float central_horizontal_angle;
	float upper_vertical_angle;
	float lower_vertical_angle;
	struct xrt_colour_rgba_f32 color_scale;
	struct xrt_colour_rgba_f32 color_bias;
};

/*!
 * UBO data that is sent to the layer projection shader.
 *
 * @relates render_gfx
 */
struct render_gfx_layer_projection_data
{
	struct xrt_normalized_rect post_transform;
	struct xrt_normalized_rect to_tangent;
	struct xrt_matrix_4x4 mvp;
	struct xrt_colour_rgba_f32 color_scale;
	struct xrt_colour_rgba_f32 color_bias;
};

/*!
 * UBO data that is sent to the layer quad shader.
 *
 * @relates render_gfx
 */
struct render_gfx_layer_quad_data
{
	struct xrt_normalized_rect post_transform;
	struct xrt_matrix_4x4 mvp;
	struct xrt_colour_rgba_f32 color_scale;
	struct xrt_colour_rgba_f32 color_bias;
};

/*!
 * @name Preparation functions - first stage
 * @{
 */

/*!
 * Allocate needed resources for one mesh shader dispatch, will also update the
 * descriptor set, UBO will be filled out with the given @p data argument.
 *
 * Uses the @ref render_sub_alloc_tracker of the @ref render_gfx and the
 * descriptor pool of @ref render_resources, both of which will be reset once
 * closed, so don't save any reference to these objects beyond the frame.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_mesh_alloc_and_write(struct render_gfx *render,
                                const struct render_gfx_mesh_ubo_data *data,
                                VkSampler src_sampler,
                                VkImageView src_image_view,
                                VkDescriptorSet *out_descriptor_set);

/*!
 * Allocate and write the UBO and descriptor set for one N-layer composite,
 * serving all views: binding 0 gets @p layer_count * view_count
 * (sampler, image view) pairs flat-indexed as [layer * view_count + view],
 * binding 1 the UBO. See @ref render_gfx_mesh_alloc_and_write for lifetime
 * details; the descriptor set comes from the dedicated
 * @ref render_resources::gfx::nlayer pool, also reset each frame.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_mesh_nlayer_alloc_and_write(struct render_gfx *render,
                                       const struct render_gfx_mesh_nlayer_ubo_data *data,
                                       uint32_t layer_count,
                                       const VkSampler *src_samplers,
                                       const VkImageView *src_image_views,
                                       VkDescriptorSet *out_descriptor_set);

/*!
 * Allocate and write a UBO and descriptor_set to be used for cylinder layer
 * rendering, the content of @p data need to be valid at the time of the call.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_layer_cylinder_alloc_and_write(struct render_gfx *render,
                                          const struct render_gfx_layer_cylinder_data *data,
                                          VkSampler src_sampler,
                                          VkImageView src_image_view,
                                          VkDescriptorSet *out_descriptor_set);

/*!
 * Allocate and write a UBO and descriptor_set to be used for equirect2 layer
 * rendering, the content of @p data need to be valid at the time of the call.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_layer_equirect2_alloc_and_write(struct render_gfx *render,
                                           const struct render_gfx_layer_equirect2_data *data,
                                           VkSampler src_sampler,
                                           VkImageView src_image_view,
                                           VkDescriptorSet *out_descriptor_set);

/*!
 * Allocate and write a UBO and descriptor_set to be used for projection layer
 * rendering, the content of @p data need to be valid at the time of the call.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_layer_projection_alloc_and_write(struct render_gfx *render,
                                            const struct render_gfx_layer_projection_data *data,
                                            VkSampler src_sampler,
                                            VkImageView src_image_view,
                                            VkDescriptorSet *out_descriptor_set);

/*!
 * Allocate and write a UBO and descriptor_set to be used for quad layer
 * rendering, the content of @p data need to be valid at the time of the call.
 *
 * @public @memberof render_gfx
 */
XRT_CHECK_RESULT VkResult
render_gfx_layer_quad_alloc_and_write(struct render_gfx *render,
                                      const struct render_gfx_layer_quad_data *data,
                                      VkSampler src_sampler,
                                      VkImageView src_image_view,
                                      VkDescriptorSet *out_descriptor_set);


/*!
 * @}
 */

/*!
 * @name Drawing functions - second stage
 * @{
 */

/*!
 * This function allocates everything to start a single rendering. This is the
 * first function you call when you start the drawiing stage, you follow up with a call
 * to @ref render_gfx_begin_view.
 *
 * @public @memberof render_gfx
 */
bool
render_gfx_begin_target(struct render_gfx *render,
                        struct render_gfx_target_resources *rtr,
                        const VkClearColorValue *color);

/*!
 * @pre successful @ref render_gfx_begin_target call,
 *   no @ref render_gfx_begin_view without matching @ref render_gfx_end_view
 * @public @memberof render_gfx
 */
void
render_gfx_end_target(struct render_gfx *render);

/*!
 * @pre successful @ref render_gfx_begin_target call
 * @public @memberof render_gfx
 */
void
render_gfx_clear_color_attachment(struct render_gfx *render, const VkClearColorValue *color);

/*!
 * @pre successful @ref render_gfx_begin_target call
 * @public @memberof render_gfx
 */
void
render_gfx_begin_view(struct render_gfx *render,
                      uint32_t view,
                      const struct render_viewport_data *viewport_data,
                      const render_scissor_data_t *scissor_data);

/*!
 * @pre successful @ref render_gfx_begin_view call without a matching call to this function
 * @public @memberof render_gfx
 */
void
render_gfx_end_view(struct render_gfx *render);

/*!
 * Dispatch one mesh shader instance, using the give @p mesh_index as source for
 * mesh geometry, timewarp selectable via @p do_timewarp.
 *
 * Must have successfully called @ref render_gfx_mesh_alloc_and_write
 * before @ref render_gfx_begin_target to allocate @p descriptor_set and UBO.
 *
 * @pre successful @ref render_gfx_mesh_alloc_and_write call, successful @ref render_gfx_begin_view call
 * @public @memberof render_gfx
 */
void
render_gfx_mesh_draw(struct render_gfx *render, uint32_t mesh_index, VkDescriptorSet descriptor_set, bool do_timewarp);

/*!
 * Element count of the gfx N-layer warp varying array for a pipeline variant
 * — the value of the warp_count spec constant (id 11). Must mirror the block
 * layout in mesh_nlayer_warp.inc.glsl: one element set per projection slot
 * (every projection carries its own source mapping, timewarped or static),
 * then one shared non-projection ray set per scanout class in use. Callers
 * route to another path when the result exceeds
 * @ref render_resources::gfx::nlayer::max_warp_count.
 *
 * @param do_chroma       Per-channel chromatic aberration sampling, i.e.
 *                        do_distortion && do_cac: 3 channels instead of 1.
 * @param proj_slot_count Projection layers in the dispatch.
 * @param ray_class_count Shared non-projection ray classes in use
 *                        (uncompensated and/or scanout-compensated), 0..2.
 *                        Eligibility checks running before the per-slot masks
 *                        exist pass the upper bound
 *                        MIN(2, nonproj_slot_count); the exact count never
 *                        exceeds it, so a granted fast path always fits.
 */
static inline uint32_t
render_gfx_nlayer_warp_count(bool do_chroma, uint32_t proj_slot_count, uint32_t ray_class_count)
{
	uint32_t channels = do_chroma ? 3 : 1;
	return channels * (proj_slot_count + ray_class_count);
}

/*!
 * Get or lazily create the N-layer composite pipeline variant for this render
 * pass. Same failure contract, @p scanout_compensate_layers_mask semantics
 * and mask canonicalization as
 * @ref render_resources_get_or_create_nlayer_pipeline. Requires
 * @ref render_gfx_render_pass::nlayer to be enabled (XRT_GFX_NLAYER) and the
 * variant's warp varyings to fit the device budget, see
 * @ref render_gfx_nlayer_warp_count.
 *
 * A set bit in @p projection_bounds_test_mask makes that projection slot
 * bounds-test its source UVs (spec id 1). Only clear a bit when border
 * sampling already handles out-of-FOV correctly for the slot: the rect
 * covers the whole image, and it is bottom-most, blends with plain source
 * alpha, or covers the display FOV. Bits on non-projection slots are
 * canonicalized away.
 *
 * @public @memberof render_gfx_render_pass
 */
XRT_CHECK_RESULT VkResult
render_gfx_render_pass_get_or_create_nlayer_pipeline(struct render_gfx_render_pass *rgrp,
                                                     uint32_t layer_count,
                                                     uint32_t layer_types,
                                                     uint32_t unpremult_mask,
                                                     uint32_t inverted_alpha_mask,
                                                     uint32_t eye_hidden_mask,
                                                     uint32_t scanout_compensate_layers_mask,
                                                     uint32_t projection_bounds_test_mask,
                                                     bool do_distortion,
                                                     bool do_cac,
                                                     enum xrt_scanout_direction scanout_direction,
                                                     VkPipeline *out_pipeline);

/*!
 * Draw the N-layer composite for one view: one indexed draw of the view's
 * distortion mesh; the vertex shader runs the warp stage, the fragment
 * shader samples and blends every layer. @p view_index doubles as the mesh
 * index and the shader's view selector (push constant). Get
 * @p pipeline from
 * @ref render_gfx_render_pass_get_or_create_nlayer_pipeline and
 * @p descriptor_set from @ref render_gfx_mesh_nlayer_alloc_and_write, both
 * with the same @p layer_count.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_mesh_nlayer_draw(struct render_gfx *render,
                            uint32_t view_index,
                            uint32_t layer_count,
                            VkDescriptorSet descriptor_set,
                            VkPipeline pipeline);

/*!
 * Dispatch a cylinder layer shader into the current target and view.
 *
 * Must have successfully called @ref render_gfx_layer_cylinder_alloc_and_write
 * before @ref render_gfx_begin_target to allocate @p descriptor_set and UBO.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_layer_cylinder(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set);

/*!
 * Dispatch a equirect2 layer shader into the current target and view.
 *
 * Must have successfully called @ref render_gfx_layer_equirect2_alloc_and_write
 * before @ref render_gfx_begin_target to allocate @p descriptor_set and UBO.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_layer_equirect2(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set);

/*!
 * Dispatch a projection layer shader into the current target and view.
 *
 * Must have successfully called @ref render_gfx_layer_projection_alloc_and_write
 * before @ref render_gfx_begin_target to allocate @p descriptor_set and UBO.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_layer_projection(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set);

/*!
 * Dispatch a quad layer shader into the current target and view.
 *
 * Must have successfully called @ref render_gfx_layer_quad_alloc_and_write
 * before @ref render_gfx_begin_target to allocate @p descriptor_set and UBO.
 *
 * @public @memberof render_gfx
 */
void
render_gfx_layer_quad(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set);

/*!
 * @}
 */


/*
 *
 * Compute distortion.
 *
 */

/*!
 * The semi-low level resources and operations required to squash layers and/or
 * apply distortion for a single frame using compute shaders.
 *
 * Unlike @ref render_gfx, this is a single stage process, and you pass all layers at a single time.
 *
 * @see comp_render_cs
 */
struct render_compute
{
	//! Shared resources.
	struct render_resources *r;

	//! Layer descriptor set.
	VkDescriptorSet layer_descriptor_sets[RENDER_MAX_LAYER_RUNS_SIZE];

	/*!
	 * Shared descriptor set, used for the clear and distortion shaders. It
	 * is used in the functions @ref render_compute_projection_timewarp,
	 * @ref render_compute_projection, and @ref render_compute_clear.
	 */
	VkDescriptorSet shared_descriptor_set;
};

/*!
 * Push data that is sent to the blit shader.
 *
 * @relates render_compute
 */
struct render_compute_blit_push_data
{
	struct xrt_normalized_rect source_rect;
	struct xrt_rect target_rect;
};

/*!
 * Color mode for blit/resolve operations, used to select the correct shader variant for the source and target
 * image formats.
 *
 * @relates render_compute
 */
enum render_compute_blit_resolve_color_mode
{
	//! Source is UNORM/linear but the submitted bytes are gamma-encoded.
	RENDER_BLIT_RESOLVE_COLOR_MODE_GAMMA_IN_LINEAR_FORMAT = 1,
	//! Source is sRGB but the submitted bytes are already linear.
	RENDER_BLIT_RESOLVE_COLOR_MODE_LINEAR_IN_SRGB_FORMAT = 2,
	RENDER_BLIT_RESOLVE_COLOR_MODE_COUNT = 2,
};

/*!
 * Chroma key parameters for std140 layout, using HSV min/max range.
 *
 * @relates render_compute
 */
struct render_chroma_key_info
{
	float hsv_min_h;
	float hsv_min_s;
	float hsv_min_v;
	float hsv_max_h;
	float hsv_max_s;
	float hsv_max_v;
	float curve;
	float despill;
};

/*!
 * UBO data that is sent to the compute layer shaders.
 *
 * @relates render_compute
 */
struct render_compute_layer_ubo_data
{
	struct render_viewport_data view;

	struct
	{
		uint32_t value;
		uint32_t padding0; // Padding up to a vec4.
		uint32_t padding1;
		uint32_t padding2;
	} layer_count;

	struct xrt_normalized_rect pre_transform;

	struct
	{
		struct xrt_normalized_rect post_transforms;

		/*!
		 * Corresponds to enum xrt_layer_type and unpremultiplied alpha.
		 *
		 * std140 uvec2, because it is an array it gets padded to vec4.
		 */
		struct
		{
			uint32_t layer_type;
			uint32_t unpremultiplied_alpha;
			uint32_t inverted_alpha;
			uint32_t _padding0;
		} layer_data;

		/*!
		 * Which image/sampler(s) correspond to each layer.
		 *
		 * std140 uvec2, because it is an array it gets padded to vec4.
		 */
		struct
		{
			uint32_t color_image_index;
			uint32_t depth_image_index;

			//! @todo Implement separated samplers and images (and change to samplers[2])
			uint32_t _padding0;
			uint32_t _padding1;
		} image_info;

		//! Shared between cylinder and equirect2.
		struct xrt_matrix_4x4 mv_inverse;


		/*!
		 * For cylinder layer
		 */
		struct
		{
			float radius;
			float central_angle;
			float aspect_ratio;
			float padding;
		} cylinder_data;


		/*!
		 * For equirect2 layers
		 */
		struct
		{
			float radius;
			float central_horizontal_angle;
			float upper_vertical_angle;
			float lower_vertical_angle;
		} eq2_data;


		/*!
		 * For projection layers
		 */

		//! FOV and timewarp matrix
		struct xrt_matrix_4x4 transforms;

		//! Chroma key parameters (per layer)
		struct render_chroma_key_info chroma_key;

		/*!
		 * For quad layers
		 */

		//! All quad transforms and coordinates are in view space
		struct
		{
			struct xrt_vec3 val;
			float padding;
		} quad_position;
		struct
		{
			struct xrt_vec3 val;
			float padding;
		} quad_normal;
		struct xrt_matrix_4x4 inverse_quad_transform;

		//! Quad extent in world scale
		struct
		{
			struct xrt_vec2 val;
			float padding0;
			float padding1;
		} quad_extent;

		/*!
		 * Color scale and bias for all layers
		 */
		struct xrt_colour_rgba_f32 color_scale;
		struct xrt_colour_rgba_f32 color_bias;
	} layers[RENDER_MAX_LAYERS];
};

/*!
 * UBO data that is sent to the compute distortion shaders.
 *
 * @relates render_compute
 */
struct render_compute_distortion_ubo_data
{
	struct render_viewport_data views[XRT_MAX_VIEWS];
	struct xrt_normalized_rect pre_transforms[XRT_MAX_VIEWS];
	struct xrt_normalized_rect post_transforms[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_begin[XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_end[XRT_MAX_VIEWS];
};

/*!
 * UBO for the N-layer fast path. `pre_transforms` is shared across layers
 * (one entry per view, display-FOV-derived); `post_transforms`, timewarp
 * matrices, `quads`, and `wraps` are flat arrays indexed as
 * `[layer * XRT_MAX_VIEWS + view]`. The alpha masks travel via the
 * `layer_unpremult_mask` (id 7) / `layer_inverted_alpha_mask` (id 8) spec consts.
 *
 * Projection slots: the matrices carry the folded [0, 1] remap and
 * sub-image rect (@ref render_time_warp_matrix_fold_remap_and_rect), the
 * begin matrix may hold a static source mapping, the end matrix is only
 * filled for scanout-compensated slots, and their `post_transforms` entries
 * hold the out-of-FOV bounds test transform
 * (@ref render_calc_proj_bounds_transform), not a sampling rect.
 *
 * @relates render_compute
 */
struct render_compute_distortion_nlayer_ubo_data
{
	struct render_viewport_data views[XRT_MAX_VIEWS];
	struct xrt_normalized_rect pre_transforms[XRT_MAX_VIEWS];
	/*!
	 * Per-view head-rotation delta R = R_pose_begin^-1 * R_pose_end as a 4x4
	 * (translation column = (0,0,0,1)). Lerped from identity by scanout_t in
	 * the shader, then applied to view-space ray directions of scanout-
	 * compensated non-projection layers (quad / cylinder / equirect2) so
	 * they get per-row scanout compensation matching what the squasher's
	 * pass-2 2D timewarp gives them. Only consumed when such a slot
	 * exists.
	 */
	struct xrt_matrix_4x4 scanout_view_rot_delta[XRT_MAX_VIEWS];
	struct xrt_normalized_rect post_transforms[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_begin[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct xrt_matrix_4x4 transform_timewarp_scanout_end[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct render_compute_nlayer_quad_data quads[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	struct render_compute_nlayer_wrap_data wraps[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
};

/*!
 * Init struct and create resources needed for compute rendering.
 *
 * @public @memberof render_compute
 */
bool
render_compute_init(struct render_compute *render, struct render_resources *r);

/*!
 * Frees all resources held by the compute rendering, does not free the struct itself.
 *
 * @public @memberof render_compute
 */
void
render_compute_fini(struct render_compute *render);

/*!
 * Begin the compute command buffer building, takes the vk_bundle's pool lock
 * and leaves it locked.
 *
 * @public @memberof render_compute
 */
bool
render_compute_begin(struct render_compute *render);

/*!
 * Frees any unneeded resources and ends the command buffer so it can be used,
 * also unlocks the vk_bundle's pool lock that was taken by begin.
 *
 * @public @memberof render_compute
 */
bool
render_compute_end(struct render_compute *render);

/*!
 * Updates the given @p descriptor_set and dispatches the layer shader. Unlike
 * other dispatch functions below this function doesn't do any layer barriers
 * before or after dispatching, this is to allow the callee to batch any such
 * image transitions.
 *
 * Expected layouts:
 * * Source images: VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
 * * Target image: VK_IMAGE_LAYOUT_GENERAL
 *
 * @public @memberof render_compute
 */
void
render_compute_layers(struct render_compute *render,
                      VkDescriptorSet descriptor_set,
                      VkBuffer ubo,
                      VkSampler src_samplers[RENDER_MAX_IMAGES_SIZE],
                      VkImageView src_image_views[RENDER_MAX_IMAGES_SIZE],
                      uint32_t num_srcs,
                      VkImageView target_image_view,
                      const struct render_viewport_data *view,
                      bool timewarp);

/*!
 * @public @memberof render_compute
 */
void
render_compute_projection_timewarp(struct render_compute *render,
                                   VkSampler src_samplers[XRT_MAX_VIEWS],
                                   VkImageView src_image_views[XRT_MAX_VIEWS],
                                   const struct xrt_normalized_rect src_rects[XRT_MAX_VIEWS],
                                   const struct xrt_pose src_poses[XRT_MAX_VIEWS],
                                   const struct xrt_fov src_fovs[XRT_MAX_VIEWS],
                                   const struct xrt_pose new_poses_scanout_begin[XRT_MAX_VIEWS],
                                   const struct xrt_pose new_poses_scanout_end[XRT_MAX_VIEWS],
                                   VkImage target_image,
                                   VkImageView target_image_view,
                                   VkImageLayout target_final_layout,
                                   const struct render_viewport_data views[XRT_MAX_VIEWS]);

/*!
 * @public @memberof render_compute
 */
void
render_compute_projection_scanout_compensation(struct render_compute *render,
                                               VkSampler src_samplers[XRT_MAX_VIEWS],
                                               VkImageView src_image_views[XRT_MAX_VIEWS],
                                               const struct xrt_normalized_rect src_rects[XRT_MAX_VIEWS],
                                               const struct xrt_fov src_fovs[XRT_MAX_VIEWS],
                                               const struct xrt_pose new_poses_scanout_begin[XRT_MAX_VIEWS],
                                               const struct xrt_pose new_poses_scanout_end[XRT_MAX_VIEWS],
                                               VkImage target_image,
                                               VkImageView target_image_view,
                                               VkImageLayout target_final_layout,
                                               const struct render_viewport_data views[XRT_MAX_VIEWS]);

/*!
 * @public @memberof render_compute
 */
void
render_compute_projection_no_timewarp(struct render_compute *render,
                                      VkSampler src_samplers[XRT_MAX_VIEWS],
                                      VkImageView src_image_views[XRT_MAX_VIEWS],
                                      const struct xrt_normalized_rect src_rects[XRT_MAX_VIEWS],
                                      VkImage target_image,
                                      VkImageView target_image_view,
                                      VkImageLayout target_final_layout,
                                      const struct render_viewport_data views[XRT_MAX_VIEWS]);

/*!
 * Composites up to @p layer_count projection / quad / cylinder / equirect2
 * layer sources over each other in submission order in a single dispatch
 * directly to the swapchain. Projection layers get per-layer timewarp +
 * geometric distortion, optionally with chromatic aberration correction;
 * non-projection layers get view-space ray-shape intersection. Generalises the
 * 1-layer fast path: at
 * @p layer_count == 1 with @p layer_types == 0 it produces the same output
 * as @ref render_compute_projection_timewarp.
 *
 * Per-layer arrays (samplers, views, rects, poses, fovs, quad_data,
 * wrap_data) are flat, indexed as [layer * XRT_MAX_VIEWS + view], length
 * layer_count * view_count.
 *
 * @p layer_types packs the per-slot @ref render_nlayer_type at
 * (slot * RENDER_NLAYER_TYPE_BITS). Bits past layer_count*BITS are ignored.
 *
 * @p view_space_slots: 1 bit per slot, set for layers submitted in VIEW
 * space. View-space projections warp against @p eye_poses instead of the
 * world scanout poses, and no view-space slot takes scanout compensation:
 * the content is locked to the device, head motion must not leak into it.
 *
 * @p unpremultiplied_mask / @p inverted_alpha_mask: 1 bit per slot, one per feature.
 *
 * @p projection_bounds_test_mask: 1 bit per slot, set = the projection slot
 * bounds-tests its source UVs. A cleared bit needs a border sampler in
 * @p src_samplers that composites out-of-image samples correctly on its
 * own; see @ref render_resources_get_or_create_nlayer_pipeline for when
 * that holds.
 *
 * For projection slots, @p src_poses, @p src_fovs, @p new_poses_scanout_*
 * and @p eye_poses drive the per-(layer, view) matrix. It is always built;
 * with @p do_timewarp off it holds the static source-FOV mapping.
 * For quad slots those entries are ignored; @p quad_data carries view-space
 * position/normal/inverse transform/extent instead. For cylinder/equirect2
 * slots, @p wrap_data carries view-space @c mv_inverse + per-type params.
 *
 * @p layer_count must be in [1, @c r->compute.distortion_nlayer.effective_nlayer_max].
 * RENDER_NLAYER_MAX is the compile-time upper bound; the actual runtime cap
 * may be lower on devices reporting smaller per-stage descriptor limits.
 *
 * @return true on success; false if the per-variant pipeline build failed
 * (cache full or shader compile error). On false, the function records no
 * command-buffer state — the caller can safely route to the squasher
 * fallback in the same frame.
 *
 * @public @memberof render_compute
 */
bool
render_compute_projection_nlayer_timewarp(struct render_compute *render,
                                          uint32_t layer_count,
                                          uint32_t layer_types,
                                          uint32_t view_space_slots,
                                          VkSampler *src_samplers,
                                          VkImageView *src_image_views,
                                          const struct xrt_normalized_rect *src_rects,
                                          const struct xrt_pose *src_poses,
                                          const struct xrt_fov *src_fovs,
                                          const struct render_compute_nlayer_quad_data *quad_data,
                                          const struct render_compute_nlayer_wrap_data *wrap_data,
                                          const struct xrt_pose new_poses_scanout_begin[XRT_MAX_VIEWS],
                                          const struct xrt_pose new_poses_scanout_end[XRT_MAX_VIEWS],
                                          const struct xrt_pose eye_poses[XRT_MAX_VIEWS],
                                          uint32_t unpremultiplied_mask,
                                          uint32_t inverted_alpha_mask,
                                          uint32_t eye_hidden_mask,
                                          uint32_t projection_bounds_test_mask,
                                          VkImage target_image,
                                          VkImageView target_image_view,
                                          const struct render_viewport_data views[XRT_MAX_VIEWS],
                                          bool do_timewarp,
                                          bool do_distortion,
                                          bool do_cac,
                                          enum xrt_scanout_direction scanout_direction);

/*!
 * @public @memberof render_compute
 */
void
render_compute_clear(struct render_compute *render,
                     VkImage target_image,
                     VkImageView target_image_view,
                     VkImageLayout target_final_layout,
                     const struct render_viewport_data views[XRT_MAX_VIEWS]);



/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
