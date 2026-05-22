// Copyright 2019-2023, Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared resources for rendering.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_render
 */

#include "xrt/xrt_device.h"

#include "math/m_api.h"
#include "math/m_matrix_2x2.h"
#include "math/m_vec2.h"

#include "vk/vk_mini_helpers.h"

#include "render/render_interface.h"

#include "cache/render_compute_pipeline_cache.h"
#include "cache/render_shader_specialization_helpers.h"


#include <stdio.h>


/*
 *
 * Gfx shared
 *
 */

XRT_CHECK_RESULT static VkResult
create_gfx_ubo_and_src_descriptor_set_layout(struct vk_bundle *vk,
                                             uint32_t ubo_binding,
                                             uint32_t src_binding,
                                             VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	VkDescriptorSetLayoutBinding set_layout_bindings[2] = {
	    {
	        .binding = src_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
	    },
	    {
	        .binding = ubo_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(set_layout_bindings),
	    .pBindings = set_layout_bindings,
	};

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout(vk->device,              //
	                                      &set_layout_info,        //
	                                      NULL,                    //
	                                      &descriptor_set_layout); //
	VK_CHK_AND_RET(ret, "vkCreateDescriptorSetLayout");

	*out_descriptor_set_layout = descriptor_set_layout;

	return VK_SUCCESS;
}


/*
 *
 * Mesh
 *
 */

XRT_CHECK_RESULT static bool
init_mesh_vertex_buffers(struct vk_bundle *vk,
                         struct render_buffer *vbo,
                         struct render_buffer *ibo,
                         uint32_t vertex_count,
                         uint32_t stride,
                         void *vertices,
                         uint32_t index_counts,
                         void *indices)
{
	VkResult ret;

	// Using the same flags for all vbos.
	VkBufferUsageFlags vbo_usage_flags = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
	VkBufferUsageFlags ibo_usage_flags = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
	VkMemoryPropertyFlags memory_property_flags =
	    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

	// Distortion vbo and ibo sizes.
	VkDeviceSize vbo_size = stride * vertex_count;
	VkDeviceSize ibo_size = sizeof(int) * index_counts;


	// Don't create vbo if size is zero.
	if (vbo_size == 0) {
		return true;
	}

	ret = render_buffer_init(  //
	    vk,                    // vk_bundle
	    vbo,                   // buffer
	    vbo_usage_flags,       // usage_flags
	    memory_property_flags, // memory_property_flags
	    vbo_size);             // size
	VK_CHK_WITH_RET(ret, "render_buffer_init", false);
	VK_NAME_BUFFER(vk, vbo->buffer, "mesh vbo");

	ret = render_buffer_write( //
	    vk,                    // vk_bundle
	    vbo,                   // buffer
	    vertices,              // data
	    vbo_size);             // size
	VK_CHK_WITH_RET(ret, "render_buffer_write", false);


	// Don't create index buffer if size is zero.
	if (ibo_size == 0) {
		return true;
	}

	ret = render_buffer_init(  //
	    vk,                    // vk_bundle
	    ibo,                   // buffer
	    ibo_usage_flags,       // usage_flags
	    memory_property_flags, // memory_property_flags
	    ibo_size);             // size
	VK_CHK_WITH_RET(ret, "render_buffer_init", false);
	VK_NAME_BUFFER(vk, ibo->buffer, "mesh ibo");

	ret = render_buffer_write( //
	    vk,                    // vk_bundle
	    ibo,                   // buffer
	    indices,               // data
	    ibo_size);             // size
	VK_CHK_WITH_RET(ret, "render_buffer_write", false);

	return true;
}

XRT_CHECK_RESULT static bool
init_mesh_ubo_buffers(struct vk_bundle *vk, struct render_buffer ubo[XRT_MAX_VIEWS], uint32_t view_count)
{
	VkResult ret;

	// Using the same flags for all ubos.
	VkBufferUsageFlags ubo_usage_flags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
	VkMemoryPropertyFlags memory_property_flags =
	    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

	// Distortion ubo size.
	VkDeviceSize ubo_size = sizeof(struct render_gfx_mesh_ubo_data);
	for (uint32_t i = 0; i < view_count; ++i) {
		ret = render_buffer_init(vk,                    //
		                         &ubo[i],               //
		                         ubo_usage_flags,       //
		                         memory_property_flags, //
		                         ubo_size);             // size
		VK_CHK_WITH_RET(ret, "render_buffer_init", false);
		char name[20];
		snprintf(name, sizeof(name), "mesh ubo %d", i);
		VK_NAME_BUFFER(vk, ubo[i].buffer, name);

		ret = render_buffer_map(vk, &ubo[i]);
		VK_CHK_WITH_RET(ret, "render_buffer_map", false);
	}
	return true;
}


/*
 *
 * Compute
 *
 */

XRT_CHECK_RESULT static VkResult
create_compute_layer_descriptor_set_layout(struct vk_bundle *vk,
                                           uint32_t src_binding,
                                           uint32_t target_binding,
                                           uint32_t ubo_binding,
                                           uint32_t source_images_count,
                                           VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	VkDescriptorSetLayoutBinding set_layout_bindings[3] = {
	    {
	        .binding = src_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = source_images_count,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = target_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = ubo_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(set_layout_bindings),
	    .pBindings = set_layout_bindings,
	};

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout( //
	    vk->device,                        //
	    &set_layout_info,                  //
	    NULL,                              //
	    &descriptor_set_layout);           //
	VK_CHK_AND_RET(ret, "vkCreateDescriptorSetLayout");

	*out_descriptor_set_layout = descriptor_set_layout;

	return VK_SUCCESS;
}

XRT_CHECK_RESULT static VkResult
create_compute_distortion_descriptor_set_layout(struct vk_bundle *vk,
                                                uint32_t src_binding,
                                                uint32_t distortion_binding,
                                                uint32_t target_binding,
                                                uint32_t ubo_binding,
                                                uint32_t view_count,
                                                VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	VkDescriptorSetLayoutBinding set_layout_bindings[4] = {
	    {
	        .binding = src_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = view_count,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = distortion_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 3 * view_count,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = target_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = ubo_binding,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(set_layout_bindings),
	    .pBindings = set_layout_bindings,
	};

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout( //
	    vk->device,                        //
	    &set_layout_info,                  //
	    NULL,                              //
	    &descriptor_set_layout);           //
	VK_CHK_AND_RET(ret, "vkCreateDescriptorSetLayout");

	*out_descriptor_set_layout = descriptor_set_layout;

	return VK_SUCCESS;
}

struct compute_distortion_nlayer_params
{
	uint32_t distortion_texel_count;
	// Bit i set = projection slot i bounds-tests its source UVs.
	uint32_t projection_bounds_test_mask; // id 1
	int32_t view_count;
	VkBool32 do_distortion;
	int32_t layer_count;
	// Mirrors enum xrt_scanout_direction; the shader treats it as int.
	int32_t scanout_direction;
	// Packed per-slot type, 2 bits per slot. Mirrors `layer_types` (id 6).
	uint32_t layer_types;
	// Per-slot unpremultiplied-alpha bitmap, 1 bit/slot. Mirrors `layer_unpremult_mask` (id 7).
	uint32_t layer_unpremult_mask;
	// Per-slot inverted-alpha bitmap, 1 bit/slot. Mirrors `layer_inverted_alpha_mask` (id 8).
	uint32_t layer_inverted_alpha_mask;
	// Packed per-slot eye-hidden bitmap, 2 bits per slot. Mirrors
	// `eye_hidden_mask` (id 9). Encoded as (XRT_LAYER_EYE_VISIBILITY_BOTH
	// XOR visibility) so all-BOTH (the default + projection slots) is 0.
	uint32_t eye_hidden_mask;
	VkBool32 do_cac;
	// Bit i set = slot i gets no scanout compensation.
	uint32_t scanout_compensate_layers_mask; // id 12
};

XRT_CHECK_RESULT static VkResult
create_compute_distortion_nlayer_pipeline(struct vk_bundle *vk,
                                          VkPipelineCache pipeline_cache,
                                          VkShaderModule shader,
                                          VkPipelineLayout pipeline_layout,
                                          const struct compute_distortion_nlayer_params *params,
                                          VkPipeline *out_compute_pipeline)
{
#define ENTRY(ID, FIELD)                                                                                               \
	{                                                                                                              \
	    .constantID = ID,                                                                                          \
	    .offset = offsetof(struct compute_distortion_nlayer_params, FIELD),                                        \
	    sizeof(params->FIELD),                                                                                     \
	}

	VkSpecializationMapEntry entries[12] = {
	    ENTRY(0, distortion_texel_count),
	    ENTRY(1, projection_bounds_test_mask),
	    ENTRY(2, view_count),
	    ENTRY(3, do_distortion),
	    ENTRY(4, layer_count),
	    ENTRY(5, scanout_direction),
	    ENTRY(6, layer_types),
	    ENTRY(7, layer_unpremult_mask),
	    ENTRY(8, layer_inverted_alpha_mask),
	    ENTRY(9, eye_hidden_mask),
	    ENTRY(10, do_cac),
	    ENTRY(12, scanout_compensate_layers_mask),
	};
#undef ENTRY

	VkSpecializationInfo specialization_info = {
	    .mapEntryCount = ARRAY_SIZE(entries),
	    .pMapEntries = entries,
	    .dataSize = sizeof(*params),
	    .pData = params,
	};

	return vk_create_compute_pipeline( //
	    vk,                            // vk_bundle
	    pipeline_cache,                // pipeline_cache
	    shader,                        // shader
	    pipeline_layout,               // pipeline_layout
	    &specialization_info,          // specialization_info
	    out_compute_pipeline);         // out_compute_pipeline
}

XRT_CHECK_RESULT static VkResult
create_compute_distortion_nlayer_descriptor_set_layout(struct vk_bundle *vk,
                                                       uint32_t layer_count,
                                                       uint32_t view_count,
                                                       VkDescriptorSetLayout *out_descriptor_set_layout)
{
	VkResult ret;

	// Binding 0 (sources) is sized exactly to the slots the matching pipeline
	// variant's shader declares (`sources[layer_count * view_count]`). One
	// layout per layer_count value means each layout's descriptorCount equals
	// the shader's array size — descriptor writes never touch unused slots.
	VkDescriptorSetLayoutBinding bindings[4] = {
	    {
	        .binding = 0,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = layer_count * view_count,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 3 * view_count,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = 2,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	    {
	        .binding = 3,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
	    },
	};

	VkDescriptorSetLayoutCreateInfo set_layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(bindings),
	    .pBindings = bindings,
	};

	VkDescriptorSetLayout layout = VK_NULL_HANDLE;
	ret = vk->vkCreateDescriptorSetLayout(vk->device, &set_layout_info, NULL, &layout);
	VK_CHK_AND_RET(ret, "vkCreateDescriptorSetLayout");

	*out_descriptor_set_layout = layout;
	return VK_SUCCESS;
}

/*
 *
 * Mock image.
 *
 */

XRT_CHECK_RESULT static VkResult
prepare_mock_image_locked(struct vk_bundle *vk, VkCommandBuffer cmd, VkImage dst)
{
	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	vk_cmd_image_barrier_gpu_locked(              //
	    vk,                                       //
	    cmd,                                      //
	    dst,                                      //
	    0,                                        //
	    VK_ACCESS_TRANSFER_WRITE_BIT,             //
	    VK_IMAGE_LAYOUT_UNDEFINED,                //
	    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, //
	    subresource_range);                       //

	return VK_SUCCESS;
}


/*
 *
 * Scratch image.
 *
 */

XRT_CHECK_RESULT static bool
create_scratch_image_and_view(struct vk_bundle *vk, VkExtent2D extent, struct render_scratch_color_image *rsci)
{
	VkResult ret;

	VkFormat srgb_format = VK_FORMAT_R8G8B8A8_SRGB;
	VkFormat unorm_format = VK_FORMAT_R8G8B8A8_UNORM;
	VkImageViewType view_type = VK_IMAGE_VIEW_TYPE_2D;

	VkDeviceMemory device_memory = VK_NULL_HANDLE;
	VkImage image = VK_NULL_HANDLE;
	VkImageView srgb_view = VK_NULL_HANDLE;
	VkImageView unorm_view = VK_NULL_HANDLE;

	// Both usages are common.
	VkImageUsageFlags unorm_usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

	// Very few cards support SRGB storage.
	VkImageUsageFlags srgb_usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	// Combination of both.
	VkImageUsageFlags image_usage = unorm_usage | srgb_usage;

	ret = vk_create_image_mutable_rgba( //
	    vk,                             // vk_bundle
	    extent,                         // extent
	    image_usage,                    // usage
	    &device_memory,                 // out_device_memory
	    &image);                        // out_image
	VK_CHK_WITH_RET(ret, "vk_create_image_mutable_rgba", false);

	VK_NAME_DEVICE_MEMORY(vk, device_memory, "render_scratch_color_image device_memory");
	VK_NAME_IMAGE(vk, image, "render_scratch_color_image image");

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = VK_REMAINING_MIP_LEVELS,
	    .baseArrayLayer = 0,
	    .layerCount = VK_REMAINING_ARRAY_LAYERS,
	};

	ret = vk_create_view_usage( //
	    vk,                     // vk_bundle
	    image,                  // image
	    view_type,              // type
	    srgb_format,            // format
	    srgb_usage,             // image_usage
	    subresource_range,      // subresource_range
	    &srgb_view);            // out_image_view
	VK_CHK_WITH_RET(ret, "vk_create_view_usage", false);

	VK_NAME_IMAGE_VIEW(vk, srgb_view, "render_scratch_color_image image view srgb");

	ret = vk_create_view_usage( //
	    vk,                     // vk_bundle
	    image,                  // image
	    view_type,              // type
	    unorm_format,           // format
	    unorm_usage,            // image_usage
	    subresource_range,      // subresource_range
	    &unorm_view);           // out_image_view
	VK_CHK_WITH_RET(ret, "vk_create_view_usage", false);

	VK_NAME_IMAGE_VIEW(vk, unorm_view, "render_scratch_color_image image view unorm");

	rsci->device_memory = device_memory;
	rsci->image = image;
	rsci->srgb_view = srgb_view;
	rsci->unorm_view = unorm_view;

	return true;
}

static void
teardown_scratch_color_image(struct vk_bundle *vk, struct render_scratch_color_image *rsci)
{
	D(ImageView, rsci->unorm_view);
	D(ImageView, rsci->srgb_view);
	D(Image, rsci->image);
	DF(Memory, rsci->device_memory);
}


/*
 *
 * 'Exported' renderer functions.
 *
 */

bool
render_resources_init(struct render_resources *r,
                      struct render_shaders *shaders,
                      struct vk_bundle *vk,
                      struct xrt_device *xdev)
{
	VkResult ret;
	xrt_result_t xret;
	bool bret;

	/*
	 * Main pointers.
	 */

	r->vk = vk;
	r->shaders = shaders;


	/*
	 * Constants
	 */

	r->view_count = xdev->hmd->view_count;
	r->mesh.src_binding = 0;
	r->mesh.ubo_binding = 1;
	struct xrt_hmd_parts *parts = xdev->hmd;
	r->mesh.vertex_count = parts->distortion.mesh.vertex_count;
	r->mesh.stride = parts->distortion.mesh.stride;
	r->mesh.index_count_total = parts->distortion.mesh.index_count_total;
	for (uint32_t i = 0; i < r->view_count; ++i) {
		r->mesh.index_counts[i] = parts->distortion.mesh.index_counts[i];
		r->mesh.index_offsets[i] = parts->distortion.mesh.index_offsets[i];
	}
	r->compute.src_binding = 0;
	r->compute.distortion_binding = 1;
	r->compute.target_binding = 2;
	r->compute.ubo_binding = 3;

	r->compute.layer.image_array_size =
	    MIN(vk->limits.max_per_stage_descriptor_sampled_images, RENDER_MAX_IMAGES_SIZE);


	/*
	 * Common samplers.
	 */

	ret = vk_create_sampler(                   //
	    vk,                                    // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, // clamp_mode
	    &r->samplers.mock);                    // out_sampler
	VK_CHK_WITH_RET(ret, "vk_create_sampler", false);

	VK_NAME_SAMPLER(vk, r->samplers.mock, "render_resources sampler mock");

	ret = vk_create_sampler(            //
	    vk,                             // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_REPEAT, // clamp_mode
	    &r->samplers.repeat);           // out_sampler
	VK_CHK_WITH_RET(ret, "vk_create_sampler", false);

	VK_NAME_SAMPLER(vk, r->samplers.repeat, "render_resources sampler repeat");

	ret = vk_create_sampler(                   //
	    vk,                                    // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, // clamp_mode
	    &r->samplers.clamp_to_edge);           // out_sampler
	VK_CHK_WITH_RET(ret, "vk_create_sampler", false);

	VK_NAME_SAMPLER(vk, r->samplers.clamp_to_edge, "render_resources sampler clamp_to_edge");

	ret = vk_create_sampler(                     //
	    vk,                                      // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER, // clamp_mode
	    &r->samplers.clamp_to_border_black);     // out_sampler
	VK_CHK_WITH_RET(ret, "vk_create_sampler", false);

	VK_NAME_SAMPLER(vk, r->samplers.clamp_to_border_black, "render_resources sampler clamp_to_border_black");

	ret = vk_create_sampler_border(                  //
	    vk,                                          // vk_bundle
	    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,     // clamp_mode
	    VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK,     // border_color
	    &r->samplers.clamp_to_border_transparent);   // out_sampler
	VK_CHK_WITH_RET(ret, "vk_create_sampler_border", false);

	VK_NAME_SAMPLER(vk, r->samplers.clamp_to_border_transparent,
	                "render_resources sampler clamp_to_border_transparent");


	/*
	 * Command buffer pool, needs to go first.
	 */

	ret = vk_cmd_pool_init(vk, &r->distortion_pool, VK_COMMAND_POOL_CREATE_TRANSIENT_BIT);
	VK_CHK_WITH_RET(ret, "vk_cmd_pool_init", false);

	VK_NAME_COMMAND_POOL(vk, r->distortion_pool.pool, "render_resources distortion command pool");

	VkCommandPoolCreateInfo command_pool_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
	    .flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT,
	    .queueFamilyIndex = vk->main_queue->family_index,
	};

	ret = vk->vkCreateCommandPool(vk->device, &command_pool_info, NULL, &r->cmd_pool);
	VK_CHK_WITH_RET(ret, "vkCreateCommandPool", false);

	VK_NAME_COMMAND_POOL(vk, r->cmd_pool, "render_resources command pool");


	/*
	 * Mock, used as a default image empty image.
	 */

	{
		VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;
		VkImageUsageFlags usage = VK_IMAGE_USAGE_SAMPLED_BIT;
		VkExtent2D extent = {1, 1};

		VkImageSubresourceRange subresource_range = {
		    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
		    .baseMipLevel = 0,
		    .levelCount = 1,
		    .baseArrayLayer = 0,
		    .layerCount = 1,
		};

		ret = vk_create_image_simple( //
		    vk,                       // vk_bundle
		    extent,                   // extent
		    format,                   // format
		    usage,                    // usage
		    &r->mock.color.memory,    // out_mem
		    &r->mock.color.image);    // out_image
		VK_CHK_WITH_RET(ret, "vk_create_image_simple", false);

		VK_NAME_DEVICE_MEMORY(vk, r->mock.color.memory, "render_resources mock color device memory");
		VK_NAME_IMAGE(vk, r->mock.color.image, "render_resources mock color image");

		ret = vk_create_view(           //
		    vk,                         // vk_bundle
		    r->mock.color.image,        // image
		    VK_IMAGE_VIEW_TYPE_2D,      // type
		    format,                     // format
		    subresource_range,          // subresource_range
		    &r->mock.color.image_view); // out_view
		VK_CHK_WITH_RET(ret, "vk_create_view", false);

		VK_NAME_IMAGE_VIEW(vk, r->mock.color.image_view, "render_resources mock color image view");


		VkCommandBuffer cmd = VK_NULL_HANDLE;
		ret = vk_cmd_create_and_begin_cmd_buffer_locked(vk, r->cmd_pool, 0, &cmd);
		VK_CHK_WITH_RET(ret, "vk_cmd_create_and_begin_cmd_buffer_locked", false);

		VK_NAME_COMMAND_BUFFER(vk, cmd, "render_resources mock command buffer");

		ret = prepare_mock_image_locked( //
		    vk,                          // vk_bundle
		    cmd,                         // cmd
		    r->mock.color.image);        // dst
		VK_CHK_WITH_RET(ret, "prepare_mock_image_locked", false);

		ret = vk_cmd_end_submit_wait_and_free_cmd_buffer_locked(vk, vk->main_queue, r->cmd_pool, cmd);
		VK_CHK_WITH_RET(ret, "vk_cmd_end_submit_wait_and_free_cmd_buffer_locked", false);

		// No need to wait, submit waits on the fence.
	}


	/*
	 * Shared
	 */

	ret = vk_create_pipeline_cache(vk, &r->pipeline_cache);
	VK_CHK_WITH_RET(ret, "vk_create_pipeline_cache", false);

	VK_NAME_PIPELINE_CACHE(vk, r->pipeline_cache, "render_resources pipeline cache");

	VkCommandBufferAllocateInfo cmd_buffer_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
	    .commandPool = r->cmd_pool,
	    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
	    .commandBufferCount = 1,
	};

	ret = vk->vkAllocateCommandBuffers( //
	    vk->device,                     // device
	    &cmd_buffer_info,               // pAllocateInfo
	    &r->cmd);                       // pCommandBuffers
	VK_CHK_WITH_RET(ret, "vkAllocateCommandBuffers", false);

	VK_NAME_COMMAND_BUFFER(vk, r->cmd, "render_resources command buffer");


	/*
	 * Gfx.
	 */

	{
		// Number of layer shader runs (views) times number of layers.
		const uint32_t layer_shader_count = RENDER_MAX_LAYER_RUNS_COUNT(r) * RENDER_MAX_LAYERS;

		// Two mesh distortion runs.
		const uint32_t mesh_shader_count = RENDER_MAX_LAYER_RUNS_COUNT(r);

		struct vk_descriptor_pool_info mesh_pool_info = {
		    .uniform_per_descriptor_count = 1,
		    .sampler_per_descriptor_count = 1,
		    .storage_image_per_descriptor_count = 0,
		    .storage_buffer_per_descriptor_count = 0,
		    .descriptor_count = layer_shader_count + mesh_shader_count,
		    .freeable = false,
		};

		ret = vk_create_descriptor_pool(          //
		    vk,                                   // vk_bundle
		    &mesh_pool_info,                      // info
		    &r->gfx.ubo_and_src_descriptor_pool); // out_descriptor_pool
		VK_CHK_WITH_RET(ret, "vk_create_descriptor_pool", false);

		VK_NAME_DESCRIPTOR_POOL(vk, r->gfx.ubo_and_src_descriptor_pool,
		                        "render_resources ubo and src descriptor pool");

		VkBufferUsageFlags usage_flags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
		VkMemoryPropertyFlags memory_property_flags = //
		    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT |    //
		    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;      //

		uint32_t buffer_count = 0;

		// One UBO per layer shader.
		buffer_count += layer_shader_count;

		// One UBO per mesh shader.
		buffer_count += RENDER_MAX_LAYER_RUNS_COUNT(r);

		// We currently use the aligmnent as max UBO size.
		static_assert(sizeof(struct render_gfx_mesh_ubo_data) <= RENDER_ALWAYS_SAFE_UBO_ALIGNMENT, "MAX");

		// Calculate size.
		VkDeviceSize size = buffer_count * RENDER_ALWAYS_SAFE_UBO_ALIGNMENT;

		ret = render_buffer_init(  //
		    vk,                    // vk_bundle
		    &r->gfx.shared_ubo,    // buffer
		    usage_flags,           // usage_flags
		    memory_property_flags, // memory_property_flags
		    size);                 // size
		VK_CHK_WITH_RET(ret, "render_buffer_init", false);
		VK_NAME_BUFFER(vk, r->gfx.shared_ubo.buffer, "render_resources gfx shared ubo");

		ret = render_buffer_map( //
		    vk,                  // vk_bundle
		    &r->gfx.shared_ubo); // buffer
		VK_CHK_WITH_RET(ret, "render_buffer_map", false);
	}


	/*
	 * Gfx layer.
	 */

	ret = create_gfx_ubo_and_src_descriptor_set_layout( //
	    vk,                                             // vk_bundle
	    RENDER_BINDING_LAYER_SHARED_UBO,                // ubo_binding
	    RENDER_BINDING_LAYER_SHARED_SRC,                // src_binding
	    &r->gfx.layer.shared.descriptor_set_layout);    // out_descriptor_set_layout
	VK_CHK_WITH_RET(ret, "create_gfx_ubo_and_src_descriptor_set_layout", false);

	VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, r->gfx.layer.shared.descriptor_set_layout,
	                              "render_resources gfx layer shared descriptor set layout");

	ret = vk_create_pipeline_layout(               //
	    vk,                                        // vk_bundle
	    r->gfx.layer.shared.descriptor_set_layout, // descriptor_set_layout
	    &r->gfx.layer.shared.pipeline_layout);     // out_pipeline_layout
	VK_CHK_WITH_RET(ret, "vk_create_pipeline_layout", false);

	VK_NAME_PIPELINE_LAYOUT(vk, r->gfx.layer.shared.pipeline_layout,
	                        "render_resources gfx layer shared pipeline layout");


	/*
	 * Mesh static.
	 */

	ret = create_gfx_ubo_and_src_descriptor_set_layout( //
	    vk,                                             // vk_bundle
	    r->mesh.ubo_binding,                            // ubo_binding
	    r->mesh.src_binding,                            // src_binding
	    &r->mesh.descriptor_set_layout);                // out_mesh_descriptor_set_layout
	VK_CHK_WITH_RET(ret, "create_gfx_ubo_and_src_descriptor_set_layout", false);

	VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, r->mesh.descriptor_set_layout, "render_resources mesh descriptor set layout");

	ret = vk_create_pipeline_layout(   //
	    vk,                            // vk_bundle
	    r->mesh.descriptor_set_layout, // descriptor_set_layout
	    &r->mesh.pipeline_layout);     // out_pipeline_layout
	VK_CHK_WITH_RET(ret, "vk_create_pipeline_layout", false);

	VK_NAME_PIPELINE_LAYOUT(vk, r->mesh.pipeline_layout, "render_resources mesh pipeline layout");

	bret = init_mesh_vertex_buffers(     //
	    vk,                              //
	    &r->mesh.vbo,                    //
	    &r->mesh.ibo,                    //
	    r->mesh.vertex_count,            //
	    r->mesh.stride,                  //
	    parts->distortion.mesh.vertices, //
	    r->mesh.index_count_total,       //
	    parts->distortion.mesh.indices); //
	if (!bret) {
		return false;
	}

	bret = init_mesh_ubo_buffers(     //
	    vk,                           //
	    r->mesh.ubos, r->view_count); //
	if (!bret) {
		return false;
	}


	/*
	 * Compute static.
	 */

	VkBufferUsageFlags ubo_usage_flags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
	VkMemoryPropertyFlags memory_property_flags =
	    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;

	const uint32_t compute_descriptor_count = //
	    1 +                                   // Shared/distortion run(s).
	    RENDER_MAX_LAYER_RUNS_COUNT(r);       // Layer shader run(s).

	struct vk_descriptor_pool_info compute_pool_info = {
	    .uniform_per_descriptor_count = 1,
	    // layer images
	    .sampler_per_descriptor_count = r->compute.layer.image_array_size + RENDER_DISTORTION_IMAGES_COUNT(r),
	    .storage_image_per_descriptor_count = 1,
	    .storage_buffer_per_descriptor_count = 0,
	    .descriptor_count = compute_descriptor_count,
	    .freeable = false,
	};

	ret = vk_create_descriptor_pool(  //
	    vk,                           // vk_bundle
	    &compute_pool_info,           // info
	    &r->compute.descriptor_pool); // out_descriptor_pool
	VK_CHK_WITH_RET(ret, "vk_create_descriptor_pool", false);

	VK_NAME_DESCRIPTOR_POOL(vk, r->compute.descriptor_pool, "render_resources compute descriptor pool");

	/*
	 * Layer pipeline
	 */

	ret = create_compute_layer_descriptor_set_layout( //
	    vk,                                           // vk_bundle
	    r->compute.src_binding,                       // src_binding,
	    r->compute.target_binding,                    // target_binding,
	    r->compute.ubo_binding,                       // ubo_binding,
	    r->compute.layer.image_array_size,            // source_images_count,
	    &r->compute.layer.descriptor_set_layout);     // out_descriptor_set_layout
	VK_CHK_WITH_RET(ret, "create_compute_layer_descriptor_set_layout", false);

	VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, r->compute.layer.descriptor_set_layout,
	                              "render_resources compute layer descriptor set layout");

	ret = vk_create_pipeline_layout(            //
	    vk,                                     // vk_bundle
	    r->compute.layer.descriptor_set_layout, // descriptor_set_layout
	    &r->compute.layer.pipeline_layout);     // out_pipeline_layout
	VK_CHK_WITH_RET(ret, "vk_create_pipeline_layout", false);

	VK_NAME_PIPELINE_LAYOUT(vk, r->compute.layer.pipeline_layout, "render_resources compute layer pipeline layout");

	xret = render_layer_pipeline_cache_create("render_resources compute layer", &r->compute.layer.pipeline_cache);
	XVK_CHK_WITH_RET(xret, "render_layer_pipeline_cache_create", false);

	xret = render_layer_pipeline_cache_init( //
	    r->compute.layer.pipeline_cache,     //
	    r->shaders->layer_comp,              //
	    r->compute.layer.pipeline_layout,    //
	    r->pipeline_cache);                  //
	XVK_CHK_WITH_RET(xret, "render_layer_pipeline_cache_init", false);

	const struct render_layer_spec layer_variants[] = {
	    render_make_layer_spec(VK_FALSE, VK_TRUE, r->compute.layer.image_array_size,
	                           RENDER_LAYER_DEFAULT_INSET_BLEND_EDGE),
	    render_make_layer_spec(VK_TRUE, VK_TRUE, r->compute.layer.image_array_size,
	                           RENDER_LAYER_DEFAULT_INSET_BLEND_EDGE),
	};

	xret = render_layer_pipeline_cache_prewarm( //
	    r->compute.layer.pipeline_cache,        //
	    vk,                                     //
	    layer_variants,                         //
	    ARRAY_SIZE(layer_variants));            //
	XVK_CHK_WITH_RET(xret, "render_layer_pipeline_cache_prewarm", false);

	xret = render_layer_pipeline_cache_get(       //
	    r->compute.layer.pipeline_cache,          //
	    vk,                                       //
	    &layer_variants[0],                       //
	    &r->compute.layer.non_timewarp_pipeline); //
	XVK_CHK_WITH_RET(xret, "render_layer_pipeline_cache_get", false);

	VK_NAME_PIPELINE(vk, r->compute.layer.non_timewarp_pipeline,
	                 "render_resources compute layer non timewarp pipeline");

	xret = render_layer_pipeline_cache_get(   //
	    r->compute.layer.pipeline_cache,      //
	    vk,                                   //
	    &layer_variants[1],                   //
	    &r->compute.layer.timewarp_pipeline); //
	XVK_CHK_WITH_RET(xret, "render_layer_pipeline_cache_get", false);

	VK_NAME_PIPELINE(vk, r->compute.layer.timewarp_pipeline, "render_resources compute layer timewarp pipeline");

	size_t layer_ubo_size = sizeof(struct render_compute_layer_ubo_data);

	for (uint32_t i = 0; i < r->view_count; i++) {
		ret = render_buffer_init(      //
		    vk,                        // vk_bundle
		    &r->compute.layer.ubos[i], // buffer
		    ubo_usage_flags,           // usage_flags
		    memory_property_flags,     // memory_property_flags
		    layer_ubo_size);           // size
		VK_CHK_WITH_RET(ret, "render_buffer_init", false);
		VK_NAME_BUFFER(vk, r->compute.layer.ubos[i].buffer, "render_resources compute layer ubo");

		ret = render_buffer_map(        //
		    vk,                         // vk_bundle
		    &r->compute.layer.ubos[i]); // buffer
		VK_CHK_WITH_RET(ret, "render_buffer_map", false);
	}


	/*
	 * Distortion pipeline
	 */

	ret = create_compute_distortion_descriptor_set_layout( //
	    vk,                                                // vk_bundle
	    r->compute.src_binding,                            // src_binding,
	    r->compute.distortion_binding,                     // distortion_binding,
	    r->compute.target_binding,                         // target_binding,
	    r->compute.ubo_binding,                            // ubo_binding,
	    r->view_count,                                     // view_count
	    &r->compute.distortion.descriptor_set_layout);     // out_descriptor_set_layout
	VK_CHK_WITH_RET(ret, "create_compute_distortion_descriptor_set_layout", false);

	VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, r->compute.distortion.descriptor_set_layout,
	                              "render_resources compute distortion descriptor set layout");

	ret = vk_create_pipeline_layout(                 //
	    vk,                                          // vk_bundle
	    r->compute.distortion.descriptor_set_layout, // descriptor_set_layout
	    &r->compute.distortion.pipeline_layout);     // out_pipeline_layout
	VK_CHK_WITH_RET(ret, "vk_create_pipeline_layout", false);

	VK_NAME_PIPELINE_LAYOUT(vk, r->compute.distortion.pipeline_layout,
	                        "render_resources compute distortion pipeline layout");

	xret = render_distortion_pipeline_cache_create("render_resources compute distortion",
	                                               &r->compute.distortion.pipeline_cache);
	XVK_CHK_WITH_RET(xret, "render_distortion_pipeline_cache_create", false);

	xret = render_distortion_pipeline_cache_init( //
	    r->compute.distortion.pipeline_cache,     //
	    r->shaders->distortion_comp,              //
	    r->compute.distortion.pipeline_layout,    //
	    r->pipeline_cache);                       //
	XVK_CHK_WITH_RET(xret, "render_distortion_pipeline_cache_init", false);

	const struct render_distortion_spec distortion_variants[] = {
	    render_make_distortion_spec(RENDER_DISTORTION_IMAGE_DIMENSIONS, VK_FALSE, r->view_count),
	    render_make_distortion_spec(RENDER_DISTORTION_IMAGE_DIMENSIONS, VK_TRUE, r->view_count),
	};

	xret = render_distortion_pipeline_cache_prewarm( //
	    r->compute.distortion.pipeline_cache,        //
	    vk,                                          //
	    distortion_variants,                         //
	    ARRAY_SIZE(distortion_variants));            //
	XVK_CHK_WITH_RET(xret, "render_distortion_pipeline_cache_prewarm", false);

	xret = render_distortion_pipeline_cache_get( //
	    r->compute.distortion.pipeline_cache,    //
	    vk,                                      //
	    &distortion_variants[0],                 //
	    &r->compute.distortion.pipeline);        //
	XVK_CHK_WITH_RET(xret, "render_distortion_pipeline_cache_get", false);

	VK_NAME_PIPELINE(vk, r->compute.distortion.pipeline, "render_resources compute distortion pipeline");

	xret = render_distortion_pipeline_cache_get(   //
	    r->compute.distortion.pipeline_cache,      //
	    vk,                                        //
	    &distortion_variants[1],                   //
	    &r->compute.distortion.timewarp_pipeline); //
	XVK_CHK_WITH_RET(xret, "render_distortion_pipeline_cache_get", false);

	VK_NAME_PIPELINE(vk, r->compute.distortion.timewarp_pipeline,
	                 "render_resources compute distortion timewarp pipeline");

	size_t distortion_ubo_size = sizeof(struct render_compute_distortion_ubo_data);

	ret = render_buffer_init(       //
	    vk,                         // vk_bundle
	    &r->compute.distortion.ubo, // buffer
	    ubo_usage_flags,            // usage_flags
	    memory_property_flags,      // memory_property_flags
	    distortion_ubo_size);       // size
	VK_CHK_WITH_RET(ret, "render_buffer_init", false);
	VK_NAME_BUFFER(vk, r->compute.distortion.ubo.buffer, "render_resources compute distortion ubo");
	ret = render_buffer_map(         //
	    vk,                          // vk_bundle
	    &r->compute.distortion.ubo); // buffer
	VK_CHK_WITH_RET(ret, "render_buffer_map", false);


	/*
	 * N-layer fast path resources. The shader handles projection / quad /
	 * cylinder / equirect2 layers in one dispatch; pipelines are lazy-built
	 * per (N, layer_types, masks, do_timewarp, do_distortion, do_cac, scanout_direction)
	 * variant. The per-N descriptor set layouts / pipeline layouts /
	 * descriptor sets are pre-allocated below, up to effective_nlayer_max.
	 */

	// Cap the path to whatever the device can actually host. Each pipeline
	// variant binds `N * view_count + 3 * view_count` combined image samplers
	// on the compute stage; the smaller of the per-stage sampler and sampled-
	// image limits gates this. Variants with N above the effective max are
	// rejected by the eligibility check and fall through to the squasher.
	{
		uint32_t per_stage_budget = MIN(vk->limits.max_per_stage_descriptor_samplers,
		                                vk->limits.max_per_stage_descriptor_sampled_images);
		uint32_t distortion_cost = 3 * r->view_count;
		uint32_t source_budget = (per_stage_budget > distortion_cost) ? (per_stage_budget - distortion_cost) : 0;
		uint32_t budget_n = source_budget / r->view_count;
		r->compute.distortion_nlayer.effective_nlayer_max = MIN((uint32_t)RENDER_NLAYER_MAX, budget_n);

		U_LOG_I("nlayer fast path: effective max N = %u (compile cap %u, device per-stage budget %u, "
		        "view_count %u)",
		        r->compute.distortion_nlayer.effective_nlayer_max, (uint32_t)RENDER_NLAYER_MAX,
		        per_stage_budget, r->view_count);
	}

	const uint32_t effective_max = r->compute.distortion_nlayer.effective_nlayer_max;

	// Per-N descriptor set layouts + pipeline layouts. Index [N-1] holds the
	// trio for a `layer_count == N` pipeline variant. Only the
	// effective_nlayer_max entries are created; the rest stay VK_NULL_HANDLE
	// and are never indexed because eligibility gates layer_count against
	// effective_max before any dispatch.
	for (uint32_t i = 0; i < effective_max; ++i) {
		uint32_t layer_count = i + 1;
		ret = create_compute_distortion_nlayer_descriptor_set_layout( //
		    vk,                                                       //
		    layer_count,                                              //
		    r->view_count,                                            //
		    &r->compute.distortion_nlayer.descriptor_set_layouts[i]); //
		VK_CHK_WITH_RET(ret, "create_compute_distortion_nlayer_descriptor_set_layout", false);

		VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, r->compute.distortion_nlayer.descriptor_set_layouts[i],
		                              "render_resources compute distortion nlayer descriptor set layout");

		ret = vk_create_pipeline_layout(                            //
		    vk,                                                     // vk_bundle
		    r->compute.distortion_nlayer.descriptor_set_layouts[i], // descriptor_set_layout
		    &r->compute.distortion_nlayer.pipeline_layouts[i]);     // out_pipeline_layout
		VK_CHK_WITH_RET(ret, "vk_create_pipeline_layout", false);

		VK_NAME_PIPELINE_LAYOUT(vk, r->compute.distortion_nlayer.pipeline_layouts[i],
		                        "render_resources compute distortion nlayer pipeline layout");
	}

	if (effective_max > 0) {
		// Single pool sized to hold effective_max sets. The helper multiplies
		// per-descriptor counts by descriptor_count, so we use the largest
		// per-set value (N=effective_max) — the pool over-allocates by a small
		// constant for smaller-N sets, which is fine.
		struct vk_descriptor_pool_info nlayer_pool_info = {
		    .uniform_per_descriptor_count = 1,
		    .sampler_per_descriptor_count = (effective_max * r->view_count) + (3 * r->view_count),
		    .storage_image_per_descriptor_count = 1,
		    .storage_buffer_per_descriptor_count = 0,
		    .descriptor_count = effective_max,
		    .freeable = false,
		};

		ret = vk_create_descriptor_pool(                    //
		    vk,                                             // vk_bundle
		    &nlayer_pool_info,                              // info
		    &r->compute.distortion_nlayer.descriptor_pool); // out_descriptor_pool
		VK_CHK_WITH_RET(ret, "vk_create_descriptor_pool", false);

		VK_NAME_DESCRIPTOR_POOL(vk, r->compute.distortion_nlayer.descriptor_pool,
		                        "render_resources compute distortion nlayer descriptor pool");

		// Allocate one descriptor set per layout. Vulkan permits a single
		// VkDescriptorSetAllocateInfo to allocate multiple sets from different
		// layouts in one call.
		VkDescriptorSetAllocateInfo alloc_info = {
		    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
		    .descriptorPool = r->compute.distortion_nlayer.descriptor_pool,
		    .descriptorSetCount = effective_max,
		    .pSetLayouts = r->compute.distortion_nlayer.descriptor_set_layouts,
		};

		ret = vk->vkAllocateDescriptorSets(vk->device, &alloc_info,
		                                   r->compute.distortion_nlayer.descriptor_sets);
		VK_CHK_WITH_RET(ret, "vkAllocateDescriptorSets", false);
	}

	size_t nlayer_ubo_size = sizeof(struct render_compute_distortion_nlayer_ubo_data);

	ret = render_buffer_init(              //
	    vk,                                // vk_bundle
	    &r->compute.distortion_nlayer.ubo, // buffer
	    ubo_usage_flags,                   // usage_flags
	    memory_property_flags,             // memory_property_flags
	    nlayer_ubo_size);                  // size
	VK_CHK_WITH_RET(ret, "render_buffer_init", false);
	VK_NAME_BUFFER(vk, r->compute.distortion_nlayer.ubo.buffer,
	               "render_resources compute distortion nlayer ubo");
	ret = render_buffer_map(                //
	    vk,                                 // vk_bundle
	    &r->compute.distortion_nlayer.ubo); // buffer
	VK_CHK_WITH_RET(ret, "render_buffer_map", false);

	// Pipelines are built lazily by render_resources_get_or_create_nlayer_pipeline;
	// no need to memset — render_resources_init's caller zeroes the struct.


	/*
	 * Clear pipeline.
	 */

	ret = vk_create_compute_pipeline(          //
	    vk,                                    // vk_bundle
	    r->pipeline_cache,                     // pipeline_cache
	    r->shaders->clear_comp,                // shader
	    r->compute.distortion.pipeline_layout, // pipeline_layout
	    NULL,                                  // specialization_info
	    &r->compute.clear.pipeline);           // out_compute_pipeline
	VK_CHK_WITH_RET(ret, "vk_create_compute_pipeline", false);

	VK_NAME_PIPELINE(vk, r->compute.clear.pipeline, "render_resources compute clear pipeline");

	size_t clear_ubo_size = sizeof(struct render_compute_distortion_ubo_data);

	ret = render_buffer_init(  //
	    vk,                    // vk_bundle
	    &r->compute.clear.ubo, // buffer
	    ubo_usage_flags,       // usage_flags
	    memory_property_flags, // memory_property_flags
	    clear_ubo_size);       // size
	VK_CHK_WITH_RET(ret, "render_buffer_init", false);
	VK_NAME_BUFFER(vk, r->compute.clear.ubo.buffer, "render_resources compute clear ubo");

	ret = render_buffer_map(    //
	    vk,                     // vk_bundle
	    &r->compute.clear.ubo); // buffer
	VK_CHK_WITH_RET(ret, "render_buffer_map", false);


	/*
	 * Compute distortion textures, not created until later.
	 */

	for (uint32_t i = 0; i < RENDER_DISTORTION_IMAGES_COUNT(r); i++) {
		r->distortion.image_views[i] = VK_NULL_HANDLE;
	}
	for (uint32_t i = 0; i < RENDER_DISTORTION_IMAGES_COUNT(r); i++) {
		r->distortion.images[i] = VK_NULL_HANDLE;
	}
	for (uint32_t i = 0; i < RENDER_DISTORTION_IMAGES_COUNT(r); i++) {
		r->distortion.device_memories[i] = VK_NULL_HANDLE;
	}


	/*
	 * Timestamp pool.
	 */

	VkQueryPoolCreateInfo poolInfo = {
	    .sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO,
	    .pNext = NULL,
	    .flags = 0, // Reserved.
	    .queryType = VK_QUERY_TYPE_TIMESTAMP,
	    .queryCount = 2,         // Start & end
	    .pipelineStatistics = 0, // Not used.
	};

	vk->vkCreateQueryPool( //
	    vk->device,        // device
	    &poolInfo,         // pCreateInfo
	    NULL,              // pAllocator
	    &r->query_pool);   // pQueryPool

	VK_NAME_QUERY_POOL(vk, r->query_pool, "render_resources query pool");

	/*
	 * Done
	 */

	U_LOG_I("New renderer initialized!");

	return true;
}

void
render_resources_fini(struct render_resources *r)
{
	// We were never initialised or already closed, always safe to call this function.
	if (r->vk == NULL) {
		return;
	}

	struct vk_bundle *vk = r->vk;

	D(Sampler, r->samplers.mock);
	D(Sampler, r->samplers.repeat);
	D(Sampler, r->samplers.clamp_to_edge);
	D(Sampler, r->samplers.clamp_to_border_black);
	D(Sampler, r->samplers.clamp_to_border_transparent);

	D(ImageView, r->mock.color.image_view);
	D(Image, r->mock.color.image);
	DF(Memory, r->mock.color.memory);

	render_buffer_fini(vk, &r->gfx.shared_ubo);
	D(DescriptorPool, r->gfx.ubo_and_src_descriptor_pool);

	D(DescriptorSetLayout, r->gfx.layer.shared.descriptor_set_layout);
	D(PipelineLayout, r->gfx.layer.shared.pipeline_layout);

	D(DescriptorSetLayout, r->mesh.descriptor_set_layout);
	D(PipelineLayout, r->mesh.pipeline_layout);
	D(PipelineCache, r->pipeline_cache);
	D(QueryPool, r->query_pool);
	render_buffer_fini(vk, &r->mesh.vbo);
	render_buffer_fini(vk, &r->mesh.ibo);
	for (uint32_t i = 0; i < r->view_count; ++i) {
		render_buffer_fini(vk, &r->mesh.ubos[i]);
	}

	D(DescriptorPool, r->compute.descriptor_pool);

	D(DescriptorSetLayout, r->compute.layer.descriptor_set_layout);
	if (r->compute.layer.pipeline_cache != NULL) {
		r->compute.layer.non_timewarp_pipeline = VK_NULL_HANDLE;
		r->compute.layer.timewarp_pipeline = VK_NULL_HANDLE;
		render_layer_pipeline_cache_destroy(&r->compute.layer.pipeline_cache, vk);
	}
	D(PipelineLayout, r->compute.layer.pipeline_layout);

	D(DescriptorSetLayout, r->compute.distortion.descriptor_set_layout);
	if (r->compute.distortion.pipeline_cache != NULL) {
		r->compute.distortion.pipeline = VK_NULL_HANDLE;
		r->compute.distortion.timewarp_pipeline = VK_NULL_HANDLE;
		render_distortion_pipeline_cache_destroy(&r->compute.distortion.pipeline_cache, vk);
	}
	D(PipelineLayout, r->compute.distortion.pipeline_layout);

	for (uint32_t i = 0; i < r->compute.distortion_nlayer.pipeline_count; ++i) {
		D(Pipeline, r->compute.distortion_nlayer.pipelines[i].pipeline);
	}
	r->compute.distortion_nlayer.pipeline_count = 0;
	D(DescriptorPool, r->compute.distortion_nlayer.descriptor_pool);
	for (uint32_t i = 0; i < r->compute.distortion_nlayer.effective_nlayer_max; ++i) {
		D(DescriptorSetLayout, r->compute.distortion_nlayer.descriptor_set_layouts[i]);
		D(PipelineLayout, r->compute.distortion_nlayer.pipeline_layouts[i]);
	}
	render_buffer_fini(vk, &r->compute.distortion_nlayer.ubo);

	D(Pipeline, r->compute.clear.pipeline);

	render_distortion_images_fini(r);
	render_buffer_fini(vk, &r->compute.clear.ubo);
	for (uint32_t i = 0; i < r->view_count; i++) {
		render_buffer_fini(vk, &r->compute.layer.ubos[i]);
	}
	render_buffer_fini(vk, &r->compute.distortion.ubo);

	vk_cmd_pool_destroy(vk, &r->distortion_pool);
	D(CommandPool, r->cmd_pool);

	// Finally forget about the vk bundle. We do not own it!
	r->vk = NULL;
}

// Pack the variant key for linear-search lookup. The scanout mask does not
// fit one uint64 anymore, so the key is two words; bit 1 of the first word
// is unused:
//   lo[0]      do_distortion       (1 bit)
//   lo[2..4]   scanout_direction   (3 bits, values 0..4)
//   lo[5..8]   layer_count         (4 bits, values 0..8)
//   lo[9..24]  layer_types         (16 bits, 2 bits/slot * 8 slots)
//   lo[25..32] unpremult_mask      (8 bits, 1 bit/slot * 8 slots)
//   lo[33..40] inverted_alpha_mask (8 bits, 1 bit/slot * 8 slots)
//   lo[41..56] eye_hidden_mask     (16 bits, 2 bits/slot * 8 slots)
//   lo[57]     do_cac              (1 bit; ignored when do_distortion == false)
//   hi[0..7]   scanout_compensate_layers_mask (8 bits, 1 bit/slot * 8 slots)
//   hi[8..15]  projection_bounds_test_mask    (8 bits, 1 bit/slot * 8 slots)
// The 2-bit/slot fields cap this layout at RENDER_NLAYER_MAX==8. Inputs
// must already be canonicalized by the getter.
static inline void
nlayer_pipeline_key(uint32_t layer_count,
                    uint32_t layer_types,
                    uint32_t unpremult_mask,
                    uint32_t inverted_alpha_mask,
                    uint32_t eye_hidden_mask,
                    uint32_t scanout_compensate_layers_mask,
                    uint32_t projection_bounds_test_mask,
                    bool do_distortion,
                    bool do_cac,
                    enum xrt_scanout_direction scanout_direction,
                    uint64_t out_key[2])
{
	do_cac = do_distortion && do_cac;

	out_key[0] = ((uint64_t)(do_cac ? 1u : 0u) << 57) |  //
	             ((uint64_t)eye_hidden_mask << 41) |     //
	             ((uint64_t)inverted_alpha_mask << 33) | //
	             ((uint64_t)unpremult_mask << 25) |      //
	             ((uint64_t)layer_types << 9) |          //
	             ((uint64_t)layer_count << 5) |          //
	             ((uint64_t)scanout_direction << 2) |    //
	             ((uint64_t)(do_distortion ? 1u : 0u));  //
	out_key[1] = ((uint64_t)projection_bounds_test_mask << 8) | //
	             ((uint64_t)scanout_compensate_layers_mask);    //
}

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
                                               VkPipeline *out_pipeline)
{
	if (layer_count < 1 || layer_count > r->compute.distortion_nlayer.effective_nlayer_max) {
		U_LOG_E("nlayer pipeline: layer_count %u out of range [1, %u]", layer_count,
		        r->compute.distortion_nlayer.effective_nlayer_max);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}
	if ((uint32_t)scanout_direction >= 5) {
		U_LOG_E("nlayer pipeline: scanout_direction %d out of range [0, 4]", (int)scanout_direction);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}
	// Mask off any bits the shader can't see (slots >= layer_count). Keeps
	// the cache key tight: callers passing different garbage in unused bits
	// would otherwise produce two pipeline variants for the same effective
	// specialisation. layer_types/eye_hidden are 2 bits per slot; the two alpha
	// masks are 1 bit per slot.
	const uint32_t type_used_bits = layer_count * RENDER_NLAYER_TYPE_BITS;
	const uint32_t type_used_mask = (type_used_bits >= 32) ? 0xFFFFFFFFu : ((1u << type_used_bits) - 1u);
	layer_types &= type_used_mask;
	eye_hidden_mask &= type_used_mask; // same 2-bits-per-slot width as layer_types
	const uint32_t slot_used_mask = (layer_count >= 32) ? 0xFFFFFFFFu : ((1u << layer_count) - 1u);
	unpremult_mask &= slot_used_mask;
	inverted_alpha_mask &= slot_used_mask;

	// Only projection slots consume the mask; dead bits must not fork variants.
	uint32_t proj_slots = 0;
	for (uint32_t i = 0; i < layer_count; i++) {
		uint32_t slot_type = (layer_types >> (i * RENDER_NLAYER_TYPE_BITS)) & 3u;
		if (slot_type == (uint32_t)RENDER_NLAYER_TYPE_PROJECTION) {
			proj_slots |= 1u << i;
		}
	}
	projection_bounds_test_mask &= proj_slots;

	// Mask canonicalization: a global-flash display compensates nothing,
	// and a variant compensating nothing is a global-flash variant.
	scanout_compensate_layers_mask &= slot_used_mask;
	if (scanout_direction == XRT_SCANOUT_DIRECTION_NONE) {
		scanout_compensate_layers_mask = slot_used_mask;
	}
	if (scanout_compensate_layers_mask == slot_used_mask) {
		scanout_direction = XRT_SCANOUT_DIRECTION_NONE;
	}

	do_cac = do_distortion && do_cac;

	uint64_t key[2];
	nlayer_pipeline_key(layer_count, layer_types, unpremult_mask, inverted_alpha_mask, eye_hidden_mask,
	                    scanout_compensate_layers_mask, projection_bounds_test_mask, do_distortion, do_cac,
	                    scanout_direction, key);

	for (uint32_t i = 0; i < r->compute.distortion_nlayer.pipeline_count; ++i) {
		if (r->compute.distortion_nlayer.pipelines[i].key[0] == key[0] &&
		    r->compute.distortion_nlayer.pipelines[i].key[1] == key[1]) {
			*out_pipeline = r->compute.distortion_nlayer.pipelines[i].pipeline;
			return VK_SUCCESS;
		}
	}

	if (r->compute.distortion_nlayer.pipeline_count >= RENDER_NLAYER_PIPELINE_CACHE_CAP) {
		U_LOG_E("nlayer pipeline cache full (%u variants); bump RENDER_NLAYER_PIPELINE_CACHE_CAP",
		        (uint32_t)RENDER_NLAYER_PIPELINE_CACHE_CAP);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	struct compute_distortion_nlayer_params params = {
	    .distortion_texel_count = RENDER_DISTORTION_IMAGE_DIMENSIONS,
	    .projection_bounds_test_mask = projection_bounds_test_mask,
	    .view_count = (int32_t)r->view_count,
	    .do_distortion = do_distortion ? VK_TRUE : VK_FALSE,
	    .layer_count = (int32_t)layer_count,
	    .scanout_direction = (int32_t)scanout_direction,
	    .layer_types = layer_types,
	    .layer_unpremult_mask = unpremult_mask,
	    .layer_inverted_alpha_mask = inverted_alpha_mask,
	    .eye_hidden_mask = eye_hidden_mask,
	    .do_cac = do_cac ? VK_TRUE : VK_FALSE,
	    .scanout_compensate_layers_mask = scanout_compensate_layers_mask,
	};

	VkPipeline pipeline = VK_NULL_HANDLE;
	VkResult ret = create_compute_distortion_nlayer_pipeline(           //
	    r->vk,                                                          // vk_bundle
	    r->pipeline_cache,                                              // pipeline_cache
	    r->shaders->distortion_nlayer_comp,                             // shader
	    r->compute.distortion_nlayer.pipeline_layouts[layer_count - 1], // pipeline_layout
	    &params,                                                        // params
	    &pipeline);                                                     // out_compute_pipeline
	if (ret != VK_SUCCESS) {
		return ret;
	}

	VK_NAME_PIPELINE(r->vk, pipeline, "render_resources compute distortion nlayer pipeline");

	uint32_t slot = r->compute.distortion_nlayer.pipeline_count++;
	r->compute.distortion_nlayer.pipelines[slot].key[0] = key[0];
	r->compute.distortion_nlayer.pipelines[slot].key[1] = key[1];
	r->compute.distortion_nlayer.pipelines[slot].pipeline = pipeline;

	*out_pipeline = pipeline;
	return VK_SUCCESS;
}

bool
render_resources_get_timestamps(struct render_resources *r, uint64_t *out_gpu_start_ns, uint64_t *out_gpu_end_ns)
{
	struct vk_bundle *vk = r->vk;
	VkResult ret = VK_SUCCESS;

	// Simple pre-check, needed by vk_convert_timestamps_to_host_ns.
	if (!vk->has_EXT_calibrated_timestamps && !vk->has_KHR_calibrated_timestamps) {
		return false;
	}


	/*
	 * Query how long things took.
	 */

	VkQueryResultFlags flags = VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT;
	uint64_t timestamps[2] = {0};

	vk->vkGetQueryPoolResults( //
	    vk->device,            // device
	    r->query_pool,         // queryPool
	    0,                     // firstQuery
	    2,                     // queryCount
	    sizeof(uint64_t) * 2,  // dataSize
	    timestamps,            // pData
	    sizeof(uint64_t),      // stride
	    flags);                // flags


	/*
	 * Convert from GPU context to CPU context, has to be
	 * done fairly quickly after timestamps has been made.
	 */
	ret = vk_convert_timestamps_to_host_ns(vk, 2, timestamps);
	if (ret != VK_SUCCESS) {
		return false;
	}

	uint64_t gpu_start_ns = timestamps[0];
	uint64_t gpu_end_ns = timestamps[1];


	/*
	 * Done
	 */

	*out_gpu_start_ns = gpu_start_ns;
	*out_gpu_end_ns = gpu_end_ns;

	return true;
}

bool
render_resources_get_duration(struct render_resources *r, uint64_t *out_gpu_duration_ns)
{
	struct vk_bundle *vk = r->vk;
	VkResult ret = VK_SUCCESS;

	/*
	 * Query how long things took.
	 */

	VkQueryResultFlags flags = VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT;
	uint64_t timestamps[2] = {0};

	ret = vk->vkGetQueryPoolResults( //
	    vk->device,                  // device
	    r->query_pool,               // queryPool
	    0,                           // firstQuery
	    2,                           // queryCount
	    sizeof(uint64_t) * 2,        // dataSize
	    timestamps,                  // pData
	    sizeof(uint64_t),            // stride
	    flags);                      // flags

	if (ret != VK_SUCCESS) {
		return false;
	}


	/*
	 * Convert from ticks to nanoseconds
	 */

	double duration_ticks = (double)(timestamps[1] - timestamps[0]);
	*out_gpu_duration_ns = (uint64_t)(duration_ticks * vk->features.timestamp_period);

	return true;
}


/*
 *
 * 'Exported' scratch functions.
 *
 */

bool
render_scratch_images_ensure(struct render_resources *r, struct render_scratch_images *rsi, VkExtent2D extent)
{
	bool bret;

	if (rsi->extent.width == extent.width &&         //
	    rsi->extent.height == extent.height &&       //
	    rsi->color[0].srgb_view != VK_NULL_HANDLE && //
	    rsi->color[0].unorm_view != VK_NULL_HANDLE) {
		return true;
	}

	render_scratch_images_fini(r, rsi);

	for (uint32_t i = 0; i < r->view_count; i++) {
		bret = create_scratch_image_and_view( //
		    r->vk,                            //
		    extent,                           //
		    &rsi->color[i]);                  //
		if (!bret) {
			break;
		}
	}

	if (!bret) {
		render_scratch_images_fini(r, rsi);
		return false;
	}

	rsi->extent = extent;

	return true;
}

void
render_scratch_images_fini(struct render_resources *r, struct render_scratch_images *rsi)
{
	struct vk_bundle *vk = r->vk;

	for (uint32_t i = 0; i < r->view_count; i++) {
		teardown_scratch_color_image(vk, &rsi->color[i]);
	}

	U_ZERO(&rsi->extent);
}
