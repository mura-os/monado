// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Generic device passthrough to fused gfx N-layer bridge.
 * @ingroup comp_main
 */

#include "comp_passthrough.h"

#include "comp_compositor.h"

#include "xrt/xrt_passthrough.h"

#include "math/m_api.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "vk/vk_helpers.h"

#include <math.h>
#include <string.h>
#include <unistd.h>

DEBUG_GET_ONCE_FLOAT_OPTION(passthrough_focus, "XRT_PASSTHROUGH_FOCUS_M", 2.0)
DEBUG_GET_ONCE_LOG_OPTION(passthrough_log, "XRT_PASSTHROUGH_LOG", U_LOGGING_INFO)

#define CPT_TRACE(p, ...) U_LOG_IFL_T((p)->log_level, __VA_ARGS__)
#define CPT_DEBUG(p, ...) U_LOG_IFL_D((p)->log_level, __VA_ARGS__)
#define CPT_INFO(p, ...) U_LOG_IFL_I((p)->log_level, __VA_ARGS__)
#define CPT_WARN(p, ...) U_LOG_IFL_W((p)->log_level, __VA_ARGS__)
#define CPT_ERROR(p, ...) U_LOG_IFL_E((p)->log_level, __VA_ARGS__)

struct passthrough_import
{
	VkImage image;
	VkDeviceMemory memory;
	VkImageView view;
};

struct comp_passthrough
{
	struct comp_compositor *c;
	struct vk_bundle *vk;
	struct xrt_passthrough_stream *provider;
	enum u_logging_level log_level;
	float focus_m;

	PFN_vkCreateSamplerYcbcrConversionKHR create_ycbcr;
	PFN_vkDestroySamplerYcbcrConversionKHR destroy_ycbcr;
	PFN_vkGetMemoryFdPropertiesKHR get_memory_fd_properties;
	VkSamplerYcbcrConversion ycbcr;
	VkSampler sampler;
	VkDescriptorSetLayout descriptor_set_layout;
	VkDescriptorPool descriptor_pool;
	VkDescriptorSet descriptor_set;
	struct render_buffer curve_buffer;

	uint64_t imports_generation;
	struct passthrough_import imports[XRT_PASSTHROUGH_MAX_VIEWS][XRT_PASSTHROUGH_MAX_SLOTS];

	bool provider_enabled;
	bool current_valid;
	struct xrt_passthrough_frame current;
	uint64_t frames_displayed;
	bool reprojection_logged;
};

static void
destroy_import(struct comp_passthrough *p, struct passthrough_import *import)
{
	if (import->view != VK_NULL_HANDLE) {
		p->vk->vkDestroyImageView(p->vk->device, import->view, NULL);
	}
	if (import->image != VK_NULL_HANDLE) {
		p->vk->vkDestroyImage(p->vk->device, import->image, NULL);
	}
	if (import->memory != VK_NULL_HANDLE) {
		p->vk->vkFreeMemory(p->vk->device, import->memory, NULL);
	}
	U_ZERO(import);
}

static void
destroy_imports(struct comp_passthrough *p)
{
	for (uint32_t view = 0; view < XRT_PASSTHROUGH_MAX_VIEWS; view++) {
		for (uint32_t slot = 0; slot < XRT_PASSTHROUGH_MAX_SLOTS; slot++) {
			destroy_import(p, &p->imports[view][slot]);
		}
	}
	p->imports_generation = 0;
}

static VkResult
import_nv12(struct comp_passthrough *p,
            const struct xrt_passthrough_frame *frame,
            xrt_graphics_buffer_handle_t handle,
            struct passthrough_import *import)
{
	struct vk_bundle *vk = p->vk;

	if (frame->format != XRT_PASSTHROUGH_FORMAT_NV12 || frame->plane_count != 2 ||
	    frame->drm_format_modifier != 0) {
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}
	uint64_t luma_end = (uint64_t)frame->offsets[0] + (uint64_t)frame->strides[0] * frame->height;
	uint64_t chroma_end = (uint64_t)frame->offsets[1] + (uint64_t)frame->strides[1] * ((frame->height + 1) / 2);
	if (frame->strides[0] < frame->width || frame->strides[1] < frame->width || luma_end > frame->buffer_size ||
	    chroma_end > frame->buffer_size) {
		CPT_ERROR(p, "passthrough: invalid NV12 plane geometry for %llu-byte buffer",
		          (unsigned long long)frame->buffer_size);
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}

	VkSubresourceLayout plane_layouts[2] = {
	    {.offset = frame->offsets[0], .rowPitch = frame->strides[0]},
	    {.offset = frame->offsets[1], .rowPitch = frame->strides[1]},
	};
	VkExternalMemoryImageCreateInfo external_info = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkImageDrmFormatModifierExplicitCreateInfoEXT modifier_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT,
	    .pNext = &external_info,
	    .drmFormatModifier = frame->drm_format_modifier,
	    .drmFormatModifierPlaneCount = frame->plane_count,
	    .pPlaneLayouts = plane_layouts,
	};
	VkImageCreateInfo image_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &modifier_info,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = VK_FORMAT_G8_B8R8_2PLANE_420_UNORM,
	    .extent = {frame->width, frame->height, 1},
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT,
	    .usage = VK_IMAGE_USAGE_SAMPLED_BIT,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	VkResult ret = p->vk->vkCreateImage(p->vk->device, &image_info, NULL, &import->image);
	VK_CHK_AND_RET(ret, "vkCreateImage");

	VkMemoryRequirements requirements;
	p->vk->vkGetImageMemoryRequirements(p->vk->device, import->image, &requirements);
	if (requirements.size > frame->buffer_size) {
		CPT_ERROR(p, "passthrough: Vulkan import needs %llu bytes, dma-buf has %llu",
		          (unsigned long long)requirements.size, (unsigned long long)frame->buffer_size);
		destroy_import(p, import);
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}
	VkMemoryFdPropertiesKHR fd_properties = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR,
	};
	ret = p->get_memory_fd_properties(p->vk->device, VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT, handle,
	                                  &fd_properties);
	if (ret != VK_SUCCESS) {
		destroy_import(p, import);
		return ret;
	}

	uint32_t type_bits = requirements.memoryTypeBits & fd_properties.memoryTypeBits;
	if (type_bits == 0) {
		destroy_import(p, import);
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}
	uint32_t type_index = 0;
	while ((type_bits & (1u << type_index)) == 0) {
		type_index++;
	}

	int import_fd = dup(handle);
	if (import_fd < 0) {
		destroy_import(p, import);
		return VK_ERROR_OUT_OF_HOST_MEMORY;
	}
	VkImportMemoryFdInfoKHR import_info = {
	    .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	    .fd = import_fd,
	};
	VkMemoryDedicatedAllocateInfo dedicated_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO,
	    .pNext = &import_info,
	    .image = import->image,
	};
	VkMemoryAllocateInfo allocation_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = &dedicated_info,
	    .allocationSize = requirements.size,
	    .memoryTypeIndex = type_index,
	};
	ret = p->vk->vkAllocateMemory(p->vk->device, &allocation_info, NULL, &import->memory);
	if (ret != VK_SUCCESS) {
		close(import_fd); // Vulkan consumes it only on successful import.
		destroy_import(p, import);
		return ret;
	}
	ret = p->vk->vkBindImageMemory(p->vk->device, import->image, import->memory, 0);
	if (ret != VK_SUCCESS) {
		destroy_import(p, import);
		return ret;
	}

	VkSamplerYcbcrConversionInfo conversion_info = {
	    .sType = VK_STRUCTURE_TYPE_SAMPLER_YCBCR_CONVERSION_INFO,
	    .conversion = p->ycbcr,
	};
	VkImageViewCreateInfo view_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
	    .pNext = &conversion_info,
	    .image = import->image,
	    .viewType = VK_IMAGE_VIEW_TYPE_2D,
	    .format = VK_FORMAT_G8_B8R8_2PLANE_420_UNORM,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .levelCount = 1,
	            .layerCount = 1,
	        },
	};
	ret = p->vk->vkCreateImageView(p->vk->device, &view_info, NULL, &import->view);
	if (ret != VK_SUCCESS) {
		destroy_import(p, import);
	}
	return ret;
}

static bool
update_frame_resources(struct comp_passthrough *p, const struct xrt_passthrough_frame *frame)
{
	if (frame->generation != p->imports_generation) {
		destroy_imports(p);
		p->imports_generation = frame->generation;
	}

	VkDescriptorImageInfo image_infos[XRT_MAX_VIEWS];
	for (uint32_t view = 0; view < frame->view_count; view++) {
		uint32_t slot = frame->views[view].slot;
		if (slot >= XRT_PASSTHROUGH_MAX_SLOTS || !xrt_graphics_buffer_is_valid(frame->views[view].handle)) {
			return false;
		}
		struct passthrough_import *import = &p->imports[view][slot];
		if (import->image == VK_NULL_HANDLE) {
			VkResult ret = import_nv12(p, frame, frame->views[view].handle, import);
			if (ret != VK_SUCCESS) {
				CPT_ERROR(p, "passthrough: failed to import view %u slot %u: %s", view, slot,
				          vk_result_string(ret));
				return false;
			}
			CPT_DEBUG(p, "passthrough: imported generation %llu view %u slot %u",
			          (unsigned long long)frame->generation, view, slot);
		}
		image_infos[view] = (VkDescriptorImageInfo){
		    .sampler = VK_NULL_HANDLE, // Immutable in the layout.
		    .imageView = import->view,
		    .imageLayout = VK_IMAGE_LAYOUT_GENERAL,
		};
	}

	VkWriteDescriptorSet writes[XRT_PASSTHROUGH_MAX_VIEWS];
	for (uint32_t view = 0; view < frame->view_count; view++) {
		writes[view] = (VkWriteDescriptorSet){
		    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
		    .dstSet = p->descriptor_set,
		    .dstBinding = view,
		    .descriptorCount = 1,
		    .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
		    .pImageInfo = &image_infos[view],
		};
	}
	p->vk->vkUpdateDescriptorSets(p->vk->device, frame->view_count, writes, 0, NULL);
	return true;
}

static void
fill_reprojection_transform(const struct xrt_pose *capture_pose,
                            const struct xrt_pose *display_pose,
                            const struct xrt_matrix_4x4 *eye_to_device,
                            struct xrt_matrix_4x4 *out_transform)
{
	// This is the exact R_capture^-1 * R_display rotation used by ordinary
	// projection-layer timewarp, before its rectilinear projection.
	struct xrt_matrix_4x4 timewarp_rotation;
	render_calc_time_warp_rotation(capture_pose, display_pose, &timewarp_rotation);
	math_matrix_4x4_multiply(&timewarp_rotation, eye_to_device, out_transform);
}

static void
fill_render_data(struct comp_passthrough *p,
                 bool do_timewarp,
                 const struct xrt_pose eye_poses[XRT_MAX_VIEWS],
                 const struct xrt_pose *head_pose_scanout_begin,
                 const struct xrt_pose *head_pose_scanout_end,
                 uint32_t view_count,
                 struct render_gfx_passthrough_data *out)
{
	const struct xrt_passthrough_calibration *calibration = &p->provider->calibration;
	struct xrt_vec3 midpoint = {
	    .x = 0.5f * (eye_poses[0].position.x + eye_poses[1].position.x),
	    .y = 0.5f * (eye_poses[0].position.y + eye_poses[1].position.y),
	    .z = 0.5f * (eye_poses[0].position.z + eye_poses[1].position.z),
	};

	out->active = true;
	out->descriptor_set = p->descriptor_set;
	for (uint32_t view = 0; view < view_count; view++) {
		const struct xrt_passthrough_camera_calibration *camera = &calibration->views[view];
		struct render_gfx_passthrough_ubo_data *data = &out->views[view];
		memcpy(data->r0, camera->imu_to_camera[0], 3 * sizeof(float));
		memcpy(data->r1, camera->imu_to_camera[1], 3 * sizeof(float));
		memcpy(data->r2, camera->imu_to_camera[2], 3 * sizeof(float));
		data->r0[3] = camera->window_axis[0];
		data->r1[3] = camera->window_axis[1];
		data->r2[3] = camera->window_axis[2];

		float scale_x = (float)p->current.width / (float)camera->width;
		float scale_y = (float)p->current.height / (float)camera->height;
		float inverse_width = 1.0f / (float)p->current.width;
		float inverse_height = 1.0f / (float)p->current.height;
		data->intr[0] = camera->fx * scale_x * inverse_width;
		data->intr[1] = camera->fy * scale_y * inverse_height;
		data->intr[2] = (camera->cx * scale_x + 0.5f) * inverse_width;
		data->intr[3] = (camera->cy * scale_y + 0.5f) * inverse_height;
		memcpy(data->dist, camera->k, sizeof(data->dist));
		data->misc[0] = camera->max_valid_undistorted_radius;
		data->misc[1] = camera->max_valid_distorted_radius;
		data->misc[2] = p->focus_m;
		data->misc[3] = camera->distortion_offset[0];
		data->camera_position[0] = camera->camera_position[0];
		data->camera_position[1] = camera->camera_position[1];
		data->camera_position[2] = camera->camera_position[2];
		data->camera_position[3] = camera->distortion_offset[1];

		/*
		 * The compositor eye poses are centred around their live midpoint,
		 * while camera extrinsics use the calibration's IMU origin.
		 */
		struct xrt_pose calibrated_eye_pose = {
		    .orientation = eye_poses[view].orientation,
		    .position =
		        {
		            .x = calibration->eye_midpoint[0] + eye_poses[view].position.x - midpoint.x,
		            .y = calibration->eye_midpoint[1] + eye_poses[view].position.y - midpoint.y,
		            .z = calibration->eye_midpoint[2] + eye_poses[view].position.z - midpoint.z,
		        },
		};
		struct xrt_matrix_4x4 eye_to_device;
		math_matrix_4x4_isometry_from_pose(&calibrated_eye_pose, &eye_to_device);

		const struct xrt_passthrough_frame_view *frame_view = &p->current.views[view];
		if (do_timewarp) {
			fill_reprojection_transform(&frame_view->capture_pose_begin, head_pose_scanout_begin,
			                            &eye_to_device, &data->transform_capture_begin_scanout_begin);
			fill_reprojection_transform(&frame_view->capture_pose_begin, head_pose_scanout_end,
			                            &eye_to_device, &data->transform_capture_begin_scanout_end);
			fill_reprojection_transform(&frame_view->capture_pose_end, head_pose_scanout_begin,
			                            &eye_to_device, &data->transform_capture_end_scanout_begin);
			fill_reprojection_transform(&frame_view->capture_pose_end, head_pose_scanout_end,
			                            &eye_to_device, &data->transform_capture_end_scanout_end);
		} else {
			// Preserve the calibrated static mapping when timewarp is disabled.
			data->transform_capture_begin_scanout_begin = eye_to_device;
			data->transform_capture_begin_scanout_end = eye_to_device;
			data->transform_capture_end_scanout_begin = eye_to_device;
			data->transform_capture_end_scanout_end = eye_to_device;
		}

		uint32_t slot = p->current.views[view].slot;
		out->images[view] = p->imports[view][slot].image;
	}

	if (do_timewarp && !p->reprojection_logged) {
		CPT_INFO(p, "passthrough: capture-to-display projection timewarp active");
		p->reprojection_logged = true;
	}
}

void
comp_passthrough_release_current(struct comp_passthrough *p)
{
	if (p == NULL || !p->current_valid) {
		return;
	}
	xrt_passthrough_stream_release_frame(p->provider, &p->current);
	p->current_valid = false;
}

void
comp_passthrough_set_enabled(struct comp_passthrough *p, bool enabled)
{
	if (p == NULL || p->provider_enabled == enabled) {
		return;
	}
	if (!enabled) {
		comp_passthrough_release_current(p);
		destroy_imports(p);
	}
	p->provider_enabled = enabled;
	xrt_passthrough_stream_set_enabled(p->provider, enabled);
	CPT_INFO(p, "passthrough: camera stream %s", enabled ? "enabled" : "disabled");
}

bool
comp_passthrough_prepare(struct comp_passthrough *p,
                         bool enabled,
                         bool do_timewarp,
                         const struct xrt_pose eye_poses[XRT_MAX_VIEWS],
                         const struct xrt_pose *head_pose_scanout_begin,
                         const struct xrt_pose *head_pose_scanout_end,
                         uint32_t view_count,
                         struct render_gfx_passthrough_data *out_data)
{
	U_ZERO(out_data);
	if (p == NULL) {
		return false;
	}
	bool use = enabled && view_count == 2;
	comp_passthrough_set_enabled(p, use);
	if (!use) {
		return false;
	}
	if (p->current_valid && !xrt_passthrough_stream_is_frame_valid(p->provider, &p->current)) {
		CPT_DEBUG(p, "passthrough: current camera frame is no longer valid, dropping imports");
		comp_passthrough_release_current(p);
		destroy_imports(p);
	}

	struct xrt_passthrough_frame next = {0};
	if (xrt_passthrough_stream_acquire_frame(p->provider, &next)) {
		bool valid = next.view_count == view_count && next.width > 0 && next.height > 0 &&
		             next.format == XRT_PASSTHROUGH_FORMAT_NV12;
		if (p->current_valid && next.generation != p->current.generation) {
			comp_passthrough_release_current(p);
		}
		if (valid && update_frame_resources(p, &next)) {
			comp_passthrough_release_current(p);
			p->current = next;
			p->current_valid = true;
		} else {
			xrt_passthrough_stream_release_frame(p->provider, &next);
		}
	}
	if (!p->current_valid) {
		return false;
	}

	fill_render_data(p, do_timewarp, eye_poses, head_pose_scanout_begin, head_pose_scanout_end, view_count,
	                 out_data);
	xrt_passthrough_stream_mark_frame_used(p->provider, &p->current);
	p->frames_displayed++;
	return true;
}

static bool
create_vk_resources(struct comp_passthrough *p)
{
	struct vk_bundle *vk = p->vk;
	if (!vk->features.sampler_ycbcr_conversion) {
		CPT_WARN(p, "passthrough: samplerYcbcrConversion is unavailable");
		return false;
	}
#if defined(VK_EXT_external_memory_dma_buf) && defined(VK_EXT_image_drm_format_modifier)
	if (!vk->has_EXT_external_memory_dma_buf || !vk->has_EXT_image_drm_format_modifier) {
		CPT_WARN(p, "passthrough: dma-buf DRM modifier import extensions are unavailable");
		return false;
	}
#else
	CPT_WARN(p, "passthrough: this Vulkan build lacks dma-buf DRM modifier import support");
	return false;
#endif
	if (!vk->has_EXT_queue_family_foreign) {
		CPT_WARN(p, "passthrough: foreign queue-family ownership transfers are unavailable");
		return false;
	}
	p->create_ycbcr = (PFN_vkCreateSamplerYcbcrConversionKHR)vk->vkGetDeviceProcAddr(
	    vk->device, "vkCreateSamplerYcbcrConversionKHR");
	p->destroy_ycbcr = (PFN_vkDestroySamplerYcbcrConversionKHR)vk->vkGetDeviceProcAddr(
	    vk->device, "vkDestroySamplerYcbcrConversionKHR");
	if (p->create_ycbcr == NULL) {
		p->create_ycbcr = (PFN_vkCreateSamplerYcbcrConversionKHR)vk->vkGetDeviceProcAddr(
		    vk->device, "vkCreateSamplerYcbcrConversion");
		p->destroy_ycbcr = (PFN_vkDestroySamplerYcbcrConversionKHR)vk->vkGetDeviceProcAddr(
		    vk->device, "vkDestroySamplerYcbcrConversion");
	}
	p->get_memory_fd_properties =
	    (PFN_vkGetMemoryFdPropertiesKHR)vk->vkGetDeviceProcAddr(vk->device, "vkGetMemoryFdPropertiesKHR");
	if (p->create_ycbcr == NULL || p->destroy_ycbcr == NULL || p->get_memory_fd_properties == NULL) {
		CPT_WARN(p, "passthrough: required YCbCr/dma-buf Vulkan entry points are unavailable");
		return false;
	}

	VkSamplerYcbcrConversionCreateInfo conversion_info = {
	    .sType = VK_STRUCTURE_TYPE_SAMPLER_YCBCR_CONVERSION_CREATE_INFO,
	    .format = VK_FORMAT_G8_B8R8_2PLANE_420_UNORM,
	    .ycbcrModel = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_601,
	    .ycbcrRange = VK_SAMPLER_YCBCR_RANGE_ITU_FULL,
	    .components =
	        {
	            .r = VK_COMPONENT_SWIZZLE_IDENTITY,
	            .g = VK_COMPONENT_SWIZZLE_IDENTITY,
	            .b = VK_COMPONENT_SWIZZLE_IDENTITY,
	            .a = VK_COMPONENT_SWIZZLE_IDENTITY,
	        },
	    .xChromaOffset = VK_CHROMA_LOCATION_MIDPOINT,
	    .yChromaOffset = VK_CHROMA_LOCATION_MIDPOINT,
	    .chromaFilter = VK_FILTER_LINEAR,
	};
	VkResult ret = p->create_ycbcr(vk->device, &conversion_info, NULL, &p->ycbcr);
	if (ret != VK_SUCCESS) {
		return false;
	}

	VkSamplerYcbcrConversionInfo sampler_conversion = {
	    .sType = VK_STRUCTURE_TYPE_SAMPLER_YCBCR_CONVERSION_INFO,
	    .conversion = p->ycbcr,
	};
	VkSamplerCreateInfo sampler_info = {
	    .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
	    .pNext = &sampler_conversion,
	    .magFilter = VK_FILTER_LINEAR,
	    .minFilter = VK_FILTER_LINEAR,
	    .mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST,
	    .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
	    .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
	    .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
	};
	ret = vk->vkCreateSampler(vk->device, &sampler_info, NULL, &p->sampler);
	if (ret != VK_SUCCESS) {
		return false;
	}

	VkDescriptorSetLayoutBinding bindings[3] = {
	    {
	        .binding = 0,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
	        .pImmutableSamplers = &p->sampler,
	    },
	    {
	        .binding = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT,
	        .pImmutableSamplers = &p->sampler,
	    },
	    {
	        .binding = 2,
	        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
	        .descriptorCount = 1,
	        .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
	    },
	};
	VkDescriptorSetLayoutCreateInfo layout_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
	    .bindingCount = ARRAY_SIZE(bindings),
	    .pBindings = bindings,
	};
	ret = vk->vkCreateDescriptorSetLayout(vk->device, &layout_info, NULL, &p->descriptor_set_layout);
	if (ret != VK_SUCCESS) {
		return false;
	}

	VkDescriptorPoolSize pool_sizes[2] = {
	    {.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = XRT_MAX_VIEWS * 4},
	    {.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, .descriptorCount = 1},
	};
	VkDescriptorPoolCreateInfo pool_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
	    .maxSets = 1,
	    .poolSizeCount = ARRAY_SIZE(pool_sizes),
	    .pPoolSizes = pool_sizes,
	};
	ret = vk->vkCreateDescriptorPool(vk->device, &pool_info, NULL, &p->descriptor_pool);
	if (ret != VK_SUCCESS) {
		return false;
	}
	VkDescriptorSetAllocateInfo allocate_info = {
	    .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
	    .descriptorPool = p->descriptor_pool,
	    .descriptorSetCount = 1,
	    .pSetLayouts = &p->descriptor_set_layout,
	};
	ret = vk->vkAllocateDescriptorSets(vk->device, &allocate_info, &p->descriptor_set);
	if (ret != VK_SUCCESS) {
		return false;
	}

	VkDeviceSize curve_size = sizeof(float) * XRT_PASSTHROUGH_CURVE_LUT_SIZE * p->provider->calibration.view_count;
	ret =
	    render_buffer_init(vk, &p->curve_buffer, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
	                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, curve_size);
	if (ret != VK_SUCCESS) {
		return false;
	}
	ret = render_buffer_map(vk, &p->curve_buffer);
	if (ret != VK_SUCCESS) {
		return false;
	}
	for (uint32_t view = 0; view < p->provider->calibration.view_count; view++) {
		memcpy((char *)p->curve_buffer.mapped + view * XRT_PASSTHROUGH_CURVE_LUT_SIZE * sizeof(float),
		       p->provider->calibration.views[view].curved_window_lut,
		       XRT_PASSTHROUGH_CURVE_LUT_SIZE * sizeof(float));
	}
	VkDescriptorBufferInfo curve_info = {
	    .buffer = p->curve_buffer.buffer,
	    .offset = 0,
	    .range = curve_size,
	};
	VkWriteDescriptorSet curve_write = {
	    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	    .dstSet = p->descriptor_set,
	    .dstBinding = 2,
	    .descriptorCount = 1,
	    .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
	    .pBufferInfo = &curve_info,
	};
	vk->vkUpdateDescriptorSets(vk->device, 1, &curve_write, 0, NULL);

	VK_NAME_DESCRIPTOR_SET_LAYOUT(vk, p->descriptor_set_layout, "comp_passthrough descriptor set layout");
	VK_NAME_DESCRIPTOR_POOL(vk, p->descriptor_pool, "comp_passthrough descriptor pool");
	VK_NAME_BUFFER(vk, p->curve_buffer.buffer, "comp_passthrough curved-window LUT");
	return true;
}

void
comp_passthrough_destroy(struct comp_passthrough **ptr)
{
	struct comp_passthrough *p = ptr != NULL ? *ptr : NULL;
	if (p == NULL) {
		return;
	}
	*ptr = NULL;
	comp_passthrough_release_current(p);
	destroy_imports(p);
	xrt_passthrough_stream_set_enabled(p->provider, false);
	render_buffer_fini(p->vk, &p->curve_buffer);
	if (p->descriptor_pool != VK_NULL_HANDLE) {
		p->vk->vkDestroyDescriptorPool(p->vk->device, p->descriptor_pool, NULL);
	}
	if (p->descriptor_set_layout != VK_NULL_HANDLE) {
		p->vk->vkDestroyDescriptorSetLayout(p->vk->device, p->descriptor_set_layout, NULL);
	}
	if (p->sampler != VK_NULL_HANDLE) {
		p->vk->vkDestroySampler(p->vk->device, p->sampler, NULL);
	}
	if (p->ycbcr != VK_NULL_HANDLE && p->destroy_ycbcr != NULL) {
		p->destroy_ycbcr(p->vk->device, p->ycbcr, NULL);
	}
	free(p);
}

VkDescriptorSetLayout
comp_passthrough_get_descriptor_set_layout(struct comp_passthrough *p)
{
	return p != NULL ? p->descriptor_set_layout : VK_NULL_HANDLE;
}

struct comp_passthrough *
comp_passthrough_create(struct comp_compositor *c)
{
	struct xrt_passthrough_stream *provider = c->xdev->passthrough;
	if (provider == NULL || provider->calibration.view_count != c->xdev->hmd->view_count ||
	    provider->calibration.view_count != 2) {
		return NULL;
	}

	struct comp_passthrough *p = U_TYPED_CALLOC(struct comp_passthrough);
	p->c = c;
	p->vk = &c->base.vk;
	p->provider = provider;
	p->log_level = debug_get_log_option_passthrough_log();
	p->focus_m = (float)debug_get_float_option_passthrough_focus();
	if (!isfinite(p->focus_m) || p->focus_m < 0.0f || p->focus_m > 1000.0f) {
		p->focus_m = 2.0f;
	}
	if (!create_vk_resources(p)) {
		CPT_WARN(p, "passthrough: Vulkan setup failed, disabling fused camera rendering");
		comp_passthrough_destroy(&p);
		return NULL;
	}
	CPT_INFO(p, "passthrough: fused nlayer camera path ready (focus %.2f m, calibration %s)", p->focus_m,
	         provider->calibration.source_path);
	return p;
}
