// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Foveation (variable rate shading) map handling.
 *
 * Applies a device-drawn foveation map to the gfx target render pass, using
 * a VK_KHR_fragment_shading_rate attachment. The device draws straight into
 * a persistently mapped staging ring slot in the hardware texel format (see
 * @ref xrt_foveation_map); the slot is GPU-copied into the rate attachment
 * image in front of the render pass.
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup aux_render
 */

#include "vk/vk_mini_helpers.h"

#include "util/u_logging.h"
#include "util/u_misc.h"

#include "render/render_interface.h"

#include "xrt/xrt_device.h"

#include <stdlib.h>
#include <string.h>


/*
 *
 * FSR mechanism specifics.
 *
 */

#ifdef VK_KHR_fragment_shading_rate
//! Attachment rate byte encoding: (log2(w) << 2) | log2(h); 1x1 -> 0, 2x2 -> 5.
static uint8_t
fsr_pack_rate(uint32_t log2_w, uint32_t log2_h)
{
	return (uint8_t)((log2_w << 2) | log2_h);
}

/*!
 * Precompute the attachment byte for each (2^i)x(2^j) fragment size: the
 * largest-area sample-count-1 GPU-supported size not exceeding the requested
 * one in either dimension.
 */
static bool
fsr_fill_rates(struct vk_bundle *vk, uint8_t rates_out[3][3])
{
	VkPhysicalDeviceFragmentShadingRateKHR rates[32];
	uint32_t count = 0;

	VkResult ret = vk->vkGetPhysicalDeviceFragmentShadingRates(vk->physical_device, &count, NULL);
	if (ret != VK_SUCCESS && ret != VK_INCOMPLETE) {
		VK_ERROR(vk, "vkGetPhysicalDeviceFragmentShadingRates: %s", vk_result_string(ret));
		return false;
	}

	if (count > ARRAY_SIZE(rates)) {
		count = ARRAY_SIZE(rates);
	}

	for (uint32_t i = 0; i < count; i++) {
		rates[i] = (VkPhysicalDeviceFragmentShadingRateKHR){
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADING_RATE_KHR,
		};
	}

	ret = vk->vkGetPhysicalDeviceFragmentShadingRates(vk->physical_device, &count, rates);
	if (ret != VK_SUCCESS && ret != VK_INCOMPLETE) {
		VK_ERROR(vk, "vkGetPhysicalDeviceFragmentShadingRates: %s", vk_result_string(ret));
		return false;
	}

	for (uint32_t lw = 0; lw < 3; lw++) {
		for (uint32_t lh = 0; lh < 3; lh++) {
			const uint32_t req_w = 1u << lw;
			const uint32_t req_h = 1u << lh;

			uint32_t best_w = 1, best_h = 1, best_area = 1;
			for (uint32_t i = 0; i < count; i++) {
				if ((rates[i].sampleCounts & VK_SAMPLE_COUNT_1_BIT) == 0) {
					continue;
				}

				uint32_t w = rates[i].fragmentSize.width;
				uint32_t h = rates[i].fragmentSize.height;
				if (w <= req_w && h <= req_h && (w * h) >= best_area) {
					best_area = w * h;
					best_w = w;
					best_h = h;
				}
			}
			rates_out[lw][lh] = fsr_pack_rate(best_w >= 4 ? 2u : (best_w >= 2 ? 1u : 0u),
			                                  best_h >= 4 ? 2u : (best_h >= 2 ? 1u : 0u));
		}
	}

	return true;
}

static bool
fsr_attachment_texel_size_valid(const VkPhysicalDeviceFragmentShadingRatePropertiesKHR *props, VkExtent2D ts)
{
	if (ts.width == 0 || ts.height == 0 || props->maxFragmentShadingRateAttachmentTexelSizeAspectRatio == 0) {
		return false;
	}
	if (props->maxFragmentShadingRateAttachmentTexelSize.width < ts.width ||
	    props->maxFragmentShadingRateAttachmentTexelSize.height < ts.height) {
		return false;
	}

	const uint32_t small = ts.width < ts.height ? ts.width : ts.height;
	const uint32_t large = ts.width > ts.height ? ts.width : ts.height;
	return (uint64_t)large <= (uint64_t)small * props->maxFragmentShadingRateAttachmentTexelSizeAspectRatio;
}
#endif


/*
 *
 * Mechanism-dependent constants.
 *
 */

static VkImageLayout
map_read_layout(const struct render_foveation *f)
{
	switch (f->mechanism) {
#ifdef VK_KHR_fragment_shading_rate
	case RENDER_FOVEATION_MECHANISM_FSR: return VK_IMAGE_LAYOUT_FRAGMENT_SHADING_RATE_ATTACHMENT_OPTIMAL_KHR;
#endif
	default: return VK_IMAGE_LAYOUT_UNDEFINED;
	}
}

static VkPipelineStageFlags
map_read_stage(const struct render_foveation *f)
{
	switch (f->mechanism) {
#ifdef VK_KHR_fragment_shading_rate
	case RENDER_FOVEATION_MECHANISM_FSR: return VK_PIPELINE_STAGE_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
#endif
	default: return VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	}
}

static VkAccessFlags
map_read_access(const struct render_foveation *f)
{
	switch (f->mechanism) {
#ifdef VK_KHR_fragment_shading_rate
	case RENDER_FOVEATION_MECHANISM_FSR: return VK_ACCESS_FRAGMENT_SHADING_RATE_ATTACHMENT_READ_BIT_KHR;
#endif
	default: return 0;
	}
}

//! The map texel rect covering a viewport, clamped to the map.
static VkRect2D
view_texel_rect(const struct render_foveation *f,
                const struct render_foveation_map *map,
                const struct render_viewport_data *vp)
{
	if (vp->w == 0 || vp->h == 0) {
		return (VkRect2D){{0, 0}, {0, 0}};
	}

	uint32_t tx0 = vp->x / f->texel_size.width;
	uint32_t ty0 = vp->y / f->texel_size.height;
	uint32_t tx1 = (vp->x + vp->w + f->texel_size.width - 1) / f->texel_size.width;
	uint32_t ty1 = (vp->y + vp->h + f->texel_size.height - 1) / f->texel_size.height;
	tx1 = tx1 < map->extent.width ? tx1 : map->extent.width;
	ty1 = ty1 < map->extent.height ? ty1 : map->extent.height;
	if (tx0 >= tx1 || ty0 >= ty1) {
		return (VkRect2D){{0, 0}, {0, 0}};
	}

	return (VkRect2D){{(int32_t)tx0, (int32_t)ty0}, {tx1 - tx0, ty1 - ty0}};
}


/*
 *
 * Upload recording.
 *
 */

static VkImageMemoryBarrier
map_image_barrier(const struct render_foveation *f, const struct render_foveation_map *map, VkImage image)
{
	return (VkImageMemoryBarrier){
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = 0,
	    .dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT,
	    .oldLayout = map->initialized ? map_read_layout(f) : VK_IMAGE_LAYOUT_UNDEFINED,
	    .newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = image,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .baseMipLevel = 0,
	            .levelCount = 1,
	            .baseArrayLayer = 0,
	            .layerCount = 1,
	        },
	};
}

/*!
 * Record barrier + the pending per-view buffer-to-image copies + barrier for
 * the map into @p cmd, and consume the pending state. The first barrier's
 * source is the map read stage, ordering the write after the previous
 * frame's in-flight reads on the same queue.
 */
static void
record_pending_upload(struct vk_bundle *vk,
                      const struct render_foveation *f,
                      struct render_foveation_map *map,
                      VkCommandBuffer cmd)
{
	const VkDeviceSize texel = f->texel_bytes;

	VkBufferImageCopy writes[XRT_MAX_VIEWS];
	uint32_t write_count = 0;

	for (uint32_t i = 0; i < XRT_MAX_VIEWS; i++) {
		const VkRect2D *wr = &map->pending.writes[i];
		if (wr->extent.width == 0 || wr->extent.height == 0) {
			continue;
		}

		writes[write_count++] = (VkBufferImageCopy){
		    .bufferOffset = (((VkDeviceSize)wr->offset.y * map->extent.width) + wr->offset.x) * texel,
		    .bufferRowLength = map->extent.width,
		    .bufferImageHeight = map->extent.height,
		    .imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1},
		    .imageOffset = {wr->offset.x, wr->offset.y, 0},
		    .imageExtent = {wr->extent.width, wr->extent.height, 1},
		};
	}

	const VkBuffer src = map->stagings[map->pending.slot].buffer;
	const VkImage image = map->images[map->image_index];
	map->pending.slot = -1;
	memset(map->pending.writes, 0, sizeof(map->pending.writes));

	if (write_count == 0) {
		return;
	}

	VkImageMemoryBarrier barrier = map_image_barrier(f, map, image);
	vk->vkCmdPipelineBarrier(           //
	    cmd,                            //
	    map_read_stage(f),              // srcStageMask
	    VK_PIPELINE_STAGE_TRANSFER_BIT, // dstStageMask
	    0,                              //
	    0, NULL,                        //
	    0, NULL,                        //
	    1, &barrier);                   //

	vk->vkCmdCopyBufferToImage(               //
	    cmd,                                  //
	    src,                                  //
	    image,                                //
	    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, //
	    write_count, writes);                 //

	barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	barrier.dstAccessMask = map_read_access(f);
	barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	barrier.newLayout = map_read_layout(f);
	vk->vkCmdPipelineBarrier(           //
	    cmd,                            //
	    VK_PIPELINE_STAGE_TRANSFER_BIT, // srcStageMask
	    map_read_stage(f),              // dstStageMask
	    0,                              //
	    0, NULL,                        //
	    0, NULL,                        //
	    1, &barrier);                   //
}

/*!
 * Fenced one-shot init: clear every map image to the coarsest-rate
 * background and move them into their resting readable layout, so texels no
 * view grid covers stay sane forever.
 */
XRT_CHECK_RESULT static VkResult
clear_images_now(struct render_resources *r, struct render_foveation_map *map)
{
	struct vk_bundle *vk = r->vk;
	struct render_foveation *f = &r->foveation;
	struct vk_cmd_pool *pool = &r->distortion_pool;
	VkResult ret;

	vk_cmd_pool_lock(pool);

	VkCommandBuffer cmd = VK_NULL_HANDLE;
	ret = vk_cmd_pool_create_and_begin_cmd_buffer_locked(vk, pool, 0, &cmd);
	if (ret != VK_SUCCESS) {
		vk_cmd_pool_unlock(pool);
		VK_ERROR(vk, "vk_cmd_pool_create_and_begin_cmd_buffer_locked: %s", vk_result_string(ret));
		return ret;
	}

	const VkImageSubresourceRange range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = 1,
	    .baseArrayLayer = 0,
	    .layerCount = 1,
	};

	for (uint32_t i = 0; i < map->image_count; i++) {
		// initialized == false: UNDEFINED -> TRANSFER_DST.
		VkImageMemoryBarrier barrier = map_image_barrier(f, map, map->images[i]);
		vk->vkCmdPipelineBarrier(              //
		    cmd,                               //
		    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, // srcStageMask
		    VK_PIPELINE_STAGE_TRANSFER_BIT,    // dstStageMask
		    0,                                 //
		    0, NULL,                           //
		    0, NULL,                           //
		    1, &barrier);                      //

		vk->vkCmdClearColorImage(cmd, map->images[i], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
		                         &f->background_color, 1, &range);

		barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		barrier.dstAccessMask = map_read_access(f);
		barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		barrier.newLayout = map_read_layout(f);
		vk->vkCmdPipelineBarrier(           //
		    cmd,                            //
		    VK_PIPELINE_STAGE_TRANSFER_BIT, // srcStageMask
		    map_read_stage(f),              // dstStageMask
		    0,                              //
		    0, NULL,                        //
		    0, NULL,                        //
		    1, &barrier);                   //
	}

	ret = vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked(vk, pool, cmd);
	vk_cmd_pool_unlock(pool);
	if (ret != VK_SUCCESS) {
		VK_ERROR(vk, "vk_cmd_pool_end_submit_wait_and_free_cmd_buffer_locked: %s", vk_result_string(ret));
		return ret;
	}

	map->initialized = true;

	return VK_SUCCESS;
}


/*
 *
 * 'Exported' functions.
 *
 */

void
render_foveation_setup(struct render_resources *r)
{
	struct vk_bundle *vk = r->vk;
	struct render_foveation *f = &r->foveation;

	U_ZERO(f);

	// The mechanism is only wired into the dynamic rendering path.
	if (!vk->features.dynamic_rendering) {
		VK_DEBUG(vk, "Foveation: no dynamic rendering, disabled");
		return;
	}

#ifdef VK_KHR_fragment_shading_rate
	if (vk->features.attachment_fragment_shading_rate) {
		VkPhysicalDeviceFragmentShadingRatePropertiesKHR props = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FRAGMENT_SHADING_RATE_PROPERTIES_KHR,
		};
		VkPhysicalDeviceProperties2 props2 = {
		    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2,
		    .pNext = &props,
		};
		vk->vkGetPhysicalDeviceProperties2(vk->physical_device, &props2);

		// The min texel size is the finest map granularity available.
		const VkExtent2D ts = props.minFragmentShadingRateAttachmentTexelSize;
		if (fsr_attachment_texel_size_valid(&props, ts) && fsr_fill_rates(vk, f->fsr_rates)) {
			f->mechanism = RENDER_FOVEATION_MECHANISM_FSR;
			f->xrt_mechanism = XRT_FOVEATION_MECHANISM_VK_FSR;
			f->map_format = VK_FORMAT_R8_UINT;
			f->texel_bytes = 1;
			f->texel_size = ts;
			f->background_color.uint32[0] = f->fsr_rates[2][2];

			VK_INFO(vk, "Foveation: fragment shading rate attachment, %ux%u px/texel", ts.width, ts.height);
			return;
		}
	}
#endif

	VK_DEBUG(vk, "Foveation: no hardware mechanism available");
}

XRT_CHECK_RESULT VkResult
render_foveation_map_init(struct render_resources *r,
                          VkExtent2D target_extent,
                          uint32_t frames_in_flight,
                          struct render_foveation_map *map)
{
	struct vk_bundle *vk = r->vk;
	struct render_foveation *f = &r->foveation;
	VkResult ret = VK_ERROR_FEATURE_NOT_PRESENT;

	assert(f->mechanism != RENDER_FOVEATION_MECHANISM_NONE);

	U_ZERO(map);
	map->pending.slot = -1;
	map->inflight.slot = -1;
	map->extent.width = (target_extent.width + f->texel_size.width - 1) / f->texel_size.width;
	map->extent.height = (target_extent.height + f->texel_size.height - 1) / f->texel_size.height;

	const VkFormat format = f->map_format;
	VkImageUsageFlags usage = 0;
	switch (f->mechanism) {
#ifdef VK_KHR_fragment_shading_rate
	case RENDER_FOVEATION_MECHANISM_FSR:
		usage = VK_IMAGE_USAGE_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		break;
#endif
	default: return VK_ERROR_FEATURE_NOT_PRESENT;
	}

	/*
	 * One staging slot per frame in flight so a slot is never written
	 * while a still pipelined frame's copy from it may execute; slot
	 * count and preserved-tracking bits are bounded by the bitfield.
	 */
	uint32_t in_flight_bound = frames_in_flight > 2 ? frames_in_flight : 2;
	in_flight_bound = in_flight_bound < 64 ? in_flight_bound : 64;
	map->staging_count = in_flight_bound;
	map->image_count = 1;

	map->memories = U_TYPED_ARRAY_CALLOC(VkDeviceMemory, map->image_count);
	map->images = U_TYPED_ARRAY_CALLOC(VkImage, map->image_count);
	map->views = U_TYPED_ARRAY_CALLOC(VkImageView, map->image_count);
	map->stagings = U_TYPED_ARRAY_CALLOC(struct render_buffer, map->staging_count);
	if (map->memories == NULL || map->images == NULL || map->views == NULL || map->stagings == NULL) {
		ret = VK_ERROR_OUT_OF_HOST_MEMORY;
		goto err;
	}

	for (uint32_t i = 0; i < map->image_count; i++) {
		ret = vk_create_image_simple(vk, map->extent, format, usage, &map->memories[i], &map->images[i]);
		VK_CHK_WITH_GOTO(ret, "vk_create_image_simple", err);
		VK_NAME_IMAGE(vk, map->images[i], "render_foveation_map image");

		VkImageViewCreateInfo view_info = {
		    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
		    .image = map->images[i],
		    .viewType = VK_IMAGE_VIEW_TYPE_2D,
		    .format = format,
		    .subresourceRange =
		        {
		            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
		            .baseMipLevel = 0,
		            .levelCount = 1,
		            .baseArrayLayer = 0,
		            .layerCount = 1,
		        },
		};
		ret = vk->vkCreateImageView(vk->device, &view_info, NULL, &map->views[i]);
		VK_CHK_WITH_GOTO(ret, "vkCreateImageView", err);
		VK_NAME_IMAGE_VIEW(vk, map->views[i], "render_foveation_map view");
	}

	for (uint32_t i = 0; i < map->staging_count; i++) {
		const VkDeviceSize byte_count = (VkDeviceSize)map->extent.width * map->extent.height * f->texel_bytes;
		ret = render_buffer_init(                                                       //
		    vk, &map->stagings[i], VK_BUFFER_USAGE_TRANSFER_SRC_BIT,                    //
		    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, //
		    byte_count);                                                                //
		VK_CHK_WITH_GOTO(ret, "render_buffer_init", err);

		ret = render_buffer_map(vk, &map->stagings[i]);
		VK_CHK_WITH_GOTO(ret, "render_buffer_map", err);
	}

	ret = clear_images_now(r, map);
	VK_CHK_WITH_GOTO(ret, "clear_images_now", err);

	return VK_SUCCESS;

err:
	render_foveation_map_fini(r, map);
	return ret;
}

void
render_foveation_map_fini(struct render_resources *r, struct render_foveation_map *map)
{
	struct vk_bundle *vk = r->vk;

	for (uint32_t i = 0; i < map->image_count; i++) {
		if (map->views != NULL) {
			D(ImageView, map->views[i]);
		}
		if (map->images != NULL) {
			D(Image, map->images[i]);
		}
		if (map->memories != NULL) {
			DF(Memory, map->memories[i]);
		}
	}
	free(map->views);
	free(map->images);
	free(map->memories);

	for (uint32_t i = 0; i < map->staging_count; i++) {
		render_buffer_fini(vk, &map->stagings[i]);
	}
	free(map->stagings);

	U_ZERO(map);
}

void
render_foveation_map_begin_info(struct render_resources *r,
                                const struct render_foveation_map *map,
                                const struct render_viewport_data *viewports,
                                uint32_t view_count,
                                struct xrt_foveation_begin_info *out_info)
{
	struct render_foveation *f = &r->foveation;

	U_ZERO(out_info);
	out_info->mechanism = f->xrt_mechanism;
	out_info->texel_extent.w = (int)f->texel_size.width;
	out_info->texel_extent.h = (int)f->texel_size.height;
	out_info->view_count = view_count < XRT_MAX_VIEWS ? view_count : XRT_MAX_VIEWS;
	for (uint32_t i = 0; i < out_info->view_count; i++) {
		VkRect2D rect = view_texel_rect(f, map, &viewports[i]);
		out_info->views[i].width = rect.extent.width;
		out_info->views[i].height = rect.extent.height;
	}
}

void
render_foveation_map_update_begin(struct render_resources *r,
                                  struct render_foveation_map *map,
                                  const struct render_viewport_data *viewports,
                                  uint32_t view_count,
                                  struct xrt_foveation_map *out_map)
{
	struct render_foveation *f = &r->foveation;
	const uint32_t texel = f->texel_bytes;

	assert(f->mechanism != RENDER_FOVEATION_MECHANISM_NONE && map->images != NULL);

	const uint32_t slot = map->update_count++ % map->staging_count;
	uint8_t *base = map->stagings[slot].mapped;
	const VkDeviceSize stride = (VkDeviceSize)map->extent.width * texel;

	U_ZERO(out_map);
	out_map->mechanism = f->xrt_mechanism;
	out_map->texel_extent.w = (int)f->texel_size.width;
	out_map->texel_extent.h = (int)f->texel_size.height;
	memcpy(out_map->fsr_rates, f->fsr_rates, sizeof(out_map->fsr_rates));
	out_map->buffer_id = slot;
	out_map->preserved = (map->filled_bits & (1ull << slot)) != 0;

	// The device fill about to happen makes the slot content unknown
	// until it completes; commit marks it filled again.
	map->filled_bits &= ~(1ull << slot);

	map->inflight.slot = (int32_t)slot;

	out_map->view_count = view_count < XRT_MAX_VIEWS ? view_count : XRT_MAX_VIEWS;
	for (uint32_t i = 0; i < out_map->view_count; i++) {
		VkRect2D rect = view_texel_rect(f, map, &viewports[i]);
		map->inflight.writes[i] = rect;
		if (rect.extent.width == 0 || rect.extent.height == 0) {
			continue;
		}

		out_map->views[i].data = base + (size_t)rect.offset.y * stride + (size_t)rect.offset.x * texel;
		out_map->views[i].width = rect.extent.width;
		out_map->views[i].height = rect.extent.height;
		out_map->views[i].stride = (uint32_t)stride;
	}
}

XRT_CHECK_RESULT VkResult
render_foveation_map_update_commit(struct render_resources *r, struct render_foveation_map *map)
{
	(void)r;

	if (map->inflight.slot < 0) {
		return VK_SUCCESS;
	}

	const uint32_t slot = (uint32_t)map->inflight.slot;
	map->filled_bits |= 1ull << slot;
	map->inflight.slot = -1;

	map->pending.slot = (int32_t)slot;
	memcpy(map->pending.writes, map->inflight.writes, sizeof(map->pending.writes));

	return VK_SUCCESS;
}

void
render_foveation_map_record_pending(struct render_resources *r, struct render_foveation_map *map, VkCommandBuffer cmd)
{
	if (map->images == NULL || map->pending.slot < 0) {
		return;
	}

	record_pending_upload(r->vk, &r->foveation, map, cmd);
}
