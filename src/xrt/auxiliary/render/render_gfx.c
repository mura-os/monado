// Copyright 2019-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The NEW compositor rendering code header.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_render
 */

#include "vk/vk_mini_helpers.h"

#include "util/u_logging.h"

#include "render/render_interface.h"

#include <stdio.h>


/*
 *
 * Common helpers
 *
 */

/*!
 * Get the @ref vk_bundle from @ref render_gfx_target_resources.
 */
static inline struct vk_bundle *
vk_from_rtr(struct render_gfx_target_resources *rtr)
{
	return rtr->r->vk;
}

/*!
 * Get the @ref vk_bundle from @ref render_gfx.
 */
static inline struct vk_bundle *
vk_from_render(struct render_gfx *render)
{
	return render->r->vk;
}

XRT_CHECK_RESULT static VkResult
create_implicit_render_pass(struct vk_bundle *vk,
                            VkFormat format,
                            VkAttachmentLoadOp load_op,
                            VkImageLayout final_layout,
                            VkRenderPass *out_render_pass)
{
	VkResult ret;

	VkAttachmentDescription attachments[1] = {
	    {
	        .format = format,
	        .samples = VK_SAMPLE_COUNT_1_BIT,
	        .loadOp = load_op,
	        .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
	        .stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE,
	        .stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
	        .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	        .finalLayout = final_layout,
	        .flags = 0,
	    },
	};

	VkAttachmentReference color_reference = {
	    .attachment = 0,
	    .layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
	};

	VkSubpassDescription subpasses[1] = {
	    {
	        .pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS,
	        .inputAttachmentCount = 0,
	        .pInputAttachments = NULL,
	        .colorAttachmentCount = 1,
	        .pColorAttachments = &color_reference,
	        .pResolveAttachments = NULL,
	        .pDepthStencilAttachment = NULL,
	        .preserveAttachmentCount = 0,
	        .pPreserveAttachments = NULL,
	    },
	};

	/*!
	 * Explicit subpass dependency required to synchronize the implicit layout
	 * transition at render pass begin with the swapchain acquire semaphore, which
	 * signals at VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT. Without this, the
	 * implicit dependency is not enough and results in a SYNC-HAZARD-WRITE-AFTER-READ
	 * (or WRITE-AFTER-WRITE with shared presentable images) validation errors.
	 */
	const VkSubpassDependency subpass_dependencies[1] = {
	    {
	        .srcSubpass = VK_SUBPASS_EXTERNAL,
	        .dstSubpass = 0,
	        .srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
	        .dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
	        .srcAccessMask = 0,
	        .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
	        .dependencyFlags = 0,
	    },
	};

	VkRenderPassCreateInfo render_pass_info = {
	    .sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
	    .attachmentCount = ARRAY_SIZE(attachments),
	    .pAttachments = attachments,
	    .subpassCount = ARRAY_SIZE(subpasses),
	    .pSubpasses = subpasses,
	    .dependencyCount = ARRAY_SIZE(subpass_dependencies),
	    .pDependencies = subpass_dependencies,
	};

	VkRenderPass render_pass = VK_NULL_HANDLE;
	ret = vk->vkCreateRenderPass( //
	    vk->device,               //
	    &render_pass_info,        //
	    NULL,                     //
	    &render_pass);            //
	VK_CHK_AND_RET(ret, "vkCreateRenderPass");

	*out_render_pass = render_pass;

	return VK_SUCCESS;
}

XRT_CHECK_RESULT static VkResult
create_framebuffer(struct vk_bundle *vk,
                   VkImageView image_view,
                   VkRenderPass render_pass,
                   uint32_t width,
                   uint32_t height,
                   VkFramebuffer *out_external_framebuffer)
{
	VkResult ret;

	VkImageView attachments[1] = {image_view};

	VkFramebufferCreateInfo frame_buffer_info = {
	    .sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO,
	    .renderPass = render_pass,
	    .attachmentCount = ARRAY_SIZE(attachments),
	    .pAttachments = attachments,
	    .width = width,
	    .height = height,
	    .layers = 1,
	};

	VkFramebuffer framebuffer = VK_NULL_HANDLE;
	ret = vk->vkCreateFramebuffer( //
	    vk->device,                //
	    &frame_buffer_info,        //
	    NULL,                      //
	    &framebuffer);             //
	VK_CHK_AND_RET(ret, "vkCreateFramebuffer");

	*out_external_framebuffer = framebuffer;

	return VK_SUCCESS;
}

#ifdef VK_KHR_dynamic_rendering
/*!
 * Transition @p image into COLOR_ATTACHMENT_OPTIMAL and begin a dynamic
 * rendering instance. The previous contents are discarded (old layout
 * UNDEFINED); the configured load op either clears or leaves them undefined.
 * srcStageMask matches the render-pass fallback's external subpass dependency,
 * so this orders after the swapchain acquire semaphore (signalled at
 * COLOR_ATTACHMENT_OUTPUT).
 */
static void
begin_dynamic_rendering(struct vk_bundle *vk,
                        VkCommandBuffer command_buffer,
                        const struct render_gfx_target_resources *rtr,
                        const VkClearColorValue *color)
{
	VkImageMemoryBarrier acquire_barrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = 0,
	    .dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
	    .oldLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	    .newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = rtr->image,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .baseMipLevel = 0,
	            .levelCount = 1,
	            .baseArrayLayer = 0,
	            .layerCount = 1,
	        },
	};
	vk->vkCmdPipelineBarrier(                          //
	    command_buffer,                                //
	    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // srcStageMask
	    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // dstStageMask
	    0,                                             // dependencyFlags
	    0, NULL,                                       // memory barriers
	    0, NULL,                                       // buffer barriers
	    1, &acquire_barrier);                          // image barriers

	VkRenderingAttachmentInfo color_attachment = {
	    .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
	    .imageView = rtr->view,
	    .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
	    .loadOp = rtr->rgrp->load_op,
	    .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
	    .clearValue = {.color = *color},
	};

	VkRenderingInfo rendering_info = {
	    .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
	    .renderArea = rtr->render_area,
	    .layerCount = 1,
	    .viewMask = 0,
	    .colorAttachmentCount = 1,
	    .pColorAttachments = &color_attachment,
	};

	// Attach the pass's foveation map, if the render pass is foveated.
#ifdef VK_KHR_fragment_shading_rate
	VkRenderingFragmentShadingRateAttachmentInfoKHR fsr_attachment = {
	    .sType = VK_STRUCTURE_TYPE_RENDERING_FRAGMENT_SHADING_RATE_ATTACHMENT_INFO_KHR,
	    .imageLayout = VK_IMAGE_LAYOUT_FRAGMENT_SHADING_RATE_ATTACHMENT_OPTIMAL_KHR,
	    .shadingRateAttachmentTexelSize = rtr->r->foveation.texel_size,
	};
	if (rtr->rgrp->foveation_mechanism == RENDER_FOVEATION_MECHANISM_FSR && rtr->foveation != NULL &&
	    render_foveation_map_current_view(rtr->foveation) != VK_NULL_HANDLE) {
		fsr_attachment.imageView = render_foveation_map_current_view(rtr->foveation);
		fsr_attachment.pNext = rendering_info.pNext;
		rendering_info.pNext = &fsr_attachment;
	}
#endif

	vk->vkCmdBeginRendering(command_buffer, &rendering_info);
}

/*!
 * End a dynamic rendering instance and, since dynamic rendering has no automatic
 * final layout transition, explicitly transition the target into its final
 * layout to match the render-pass fallback's finalLayout. Callers that keep the
 * image in COLOR_ATTACHMENT_OPTIMAL (scratch) do their own transition later.
 */
static void
end_dynamic_rendering(struct vk_bundle *vk,
                      VkCommandBuffer command_buffer,
                      const struct render_gfx_target_resources *rtr)
{
	vk->vkCmdEndRendering(command_buffer);

	VkImageLayout final_layout = rtr->rgrp->final_layout;
	if (final_layout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL) {
		return;
	}

	VkImageMemoryBarrier final_barrier = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER,
	    .srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
	    .dstAccessMask = 0,
	    .oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
	    .newLayout = final_layout,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image = rtr->image,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .baseMipLevel = 0,
	            .levelCount = 1,
	            .baseArrayLayer = 0,
	            .layerCount = 1,
	        },
	};
	vk->vkCmdPipelineBarrier(                          //
	    command_buffer,                                //
	    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, // srcStageMask
	    VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,          // dstStageMask
	    0,                                             // dependencyFlags
	    0, NULL,                                       // memory barriers
	    0, NULL,                                       // buffer barriers
	    1, &final_barrier);                            // image barriers
}
#endif // VK_KHR_dynamic_rendering

static void
begin_render_pass(struct vk_bundle *vk,
                  VkCommandBuffer command_buffer,
                  const struct render_gfx_target_resources *rtr,
                  const VkClearColorValue *color)
{
#ifdef VK_KHR_dynamic_rendering
	if (vk->features.dynamic_rendering) {
		begin_dynamic_rendering(vk, command_buffer, rtr, color);
		return;
	}
#endif

	VkClearValue clear_color[1] = {{
	    .color = *color,
	}};

	VkRenderPassBeginInfo render_pass_begin_info = {
	    .sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO,
	    .renderPass = rtr->rgrp->render_pass,
	    .framebuffer = rtr->framebuffer,
	    .renderArea = rtr->render_area,
	    .clearValueCount = ARRAY_SIZE(clear_color),
	    .pClearValues = clear_color,
	};

	vk->vkCmdBeginRenderPass(command_buffer, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
}

static void
end_render_pass(struct vk_bundle *vk, VkCommandBuffer command_buffer, const struct render_gfx_target_resources *rtr)
{
#ifdef VK_KHR_dynamic_rendering
	if (vk->features.dynamic_rendering) {
		end_dynamic_rendering(vk, command_buffer, rtr);
		return;
	}
#endif

	vk->vkCmdEndRenderPass(command_buffer);
}

/// Update descriptor set for a layer to reference the parameter UBO and the source (layer) image.
static void
update_ubo_and_src_descriptor_set(struct vk_bundle *vk,
                                  uint32_t ubo_binding,
                                  VkBuffer buffer,
                                  VkDeviceSize offset,
                                  VkDeviceSize size,
                                  uint32_t src_binding,
                                  VkSampler sampler,
                                  VkImageView image_view,
                                  VkDescriptorSet descriptor_set)
{
	VkDescriptorImageInfo image_info = {
	    .sampler = sampler,
	    .imageView = image_view,
	    .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
	};

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = buffer,
	    .offset = offset,
	    .range = size,
	};

	VkWriteDescriptorSet write_descriptor_sets[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = src_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = &image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	vk->vkUpdateDescriptorSets(            //
	    vk->device,                        //
	    ARRAY_SIZE(write_descriptor_sets), // descriptorWriteCount
	    write_descriptor_sets,             // pDescriptorWrites
	    0,                                 // descriptorCopyCount
	    NULL);                             // pDescriptorCopies
}

/// Sub-allocate a UBO for our layer-specific data,
/// and create a descriptor set for it and the layer image to sample.
XRT_CHECK_RESULT static VkResult
do_ubo_and_src_alloc_and_write(struct render_gfx *render,
                               uint32_t ubo_binding,
                               const void *ubo_ptr,
                               VkDeviceSize ubo_size,
                               uint32_t src_binding,
                               VkSampler src_sampler,
                               VkImageView src_image_view,
                               VkDescriptorPool descriptor_pool,
                               VkDescriptorSetLayout descriptor_set_layout,
                               VkDescriptorSet *out_descriptor_set)
{
	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	struct render_sub_alloc ubo = XRT_STRUCT_INIT;
	struct vk_bundle *vk = vk_from_render(render);

	VkResult ret;


	/*
	 * Allocate and upload data.
	 */
	ret = render_sub_alloc_ubo_alloc_and_write( //
	    vk,                                     //
	    &render->ubo_tracker,                   // rsat
	    ubo_ptr,                                //
	    ubo_size,                               //
	    &ubo);                                  // out_rsa
	VK_CHK_AND_RET(ret, "render_sub_alloc_ubo_alloc_and_write");


	/*
	 * Create and fill out descriptor.
	 */

	ret = vk_create_descriptor_set( //
	    vk,                         //
	    descriptor_pool,            //
	    descriptor_set_layout,      //
	    &descriptor_set);           //
	VK_CHK_AND_RET(ret, "vk_create_descriptor_set");

	update_ubo_and_src_descriptor_set( //
	    vk,                            //
	    ubo_binding,                   //
	    ubo.buffer,                    //
	    ubo.offset,                    //
	    ubo.size,                      //
	    src_binding,                   //
	    src_sampler,                   //
	    src_image_view,                //
	    descriptor_set);               //

	*out_descriptor_set = descriptor_set;

	return VK_SUCCESS;
}

static inline void
dispatch_no_vbo(struct render_gfx *render, uint32_t vertex_count, VkPipeline pipeline, VkDescriptorSet descriptor_set)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;


	VkDescriptorSet descriptor_sets[1] = {descriptor_set};
	vk->vkCmdBindDescriptorSets(             //
	    r->cmd,                              //
	    VK_PIPELINE_BIND_POINT_GRAPHICS,     // pipelineBindPoint
	    r->gfx.layer.shared.pipeline_layout, // layout
	    0,                                   // firstSet
	    ARRAY_SIZE(descriptor_sets),         // descriptorSetCount
	    descriptor_sets,                     // pDescriptorSets
	    0,                                   // dynamicOffsetCount
	    NULL);                               // pDynamicOffsets

	vk->vkCmdBindPipeline(               //
	    r->cmd,                          //
	    VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
	    pipeline);                       //

	// This pipeline doesn't have any VBO input or indices.

	vk->vkCmdDraw(    //
	    r->cmd,       //
	    vertex_count, // vertexCount
	    1,            // instanceCount
	    0,            // firstVertex
	    0);           // firstInstance
}


/*
 *
 * Layer
 *
 */

XRT_CHECK_RESULT static VkResult
create_layer_pipeline(struct vk_bundle *vk,
                      VkRenderPass render_pass,
                      VkFormat color_format,
                      enum render_foveation_mechanism foveation_mechanism,
                      VkPipelineLayout pipeline_layout,
                      VkPipelineCache pipeline_cache,
                      VkBlendFactor src_blend_factor,
                      VkShaderModule module_vert,
                      VkShaderModule module_frag,
                      VkPipeline *out_pipeline)
{
	VkResult ret;

	// Might be changed to line for debugging.
	VkPolygonMode polygonMode = VK_POLYGON_MODE_FILL;

	// Generate vertices inside of the vertex shader.
	const VkPipelineInputAssemblyStateCreateInfo input_assembly_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
	    .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP,
	    .primitiveRestartEnable = VK_FALSE,
	};

	const VkPipelineVertexInputStateCreateInfo vertex_input_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
	    .vertexAttributeDescriptionCount = 0,
	    .pVertexAttributeDescriptions = NULL,
	    .vertexBindingDescriptionCount = 0,
	    .pVertexBindingDescriptions = NULL,
	};


	/*
	 * Target and rasterisation.
	 */

	const VkPipelineViewportStateCreateInfo viewport_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
	    .viewportCount = 1,
	    .scissorCount = 1,
	};

	const VkPipelineMultisampleStateCreateInfo multisample_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
	    .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT,
	};

	const VkPipelineRasterizationStateCreateInfo rasterization_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
	    .depthClampEnable = VK_FALSE,
	    .rasterizerDiscardEnable = VK_FALSE,
	    .polygonMode = polygonMode,
	    .cullMode = VK_CULL_MODE_BACK_BIT,
	    .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
	    .lineWidth = 1.0f,
	};


	/*
	 * Blending.
	 */

	const VkColorComponentFlags all_components = //
	    VK_COLOR_COMPONENT_R_BIT |               //
	    VK_COLOR_COMPONENT_G_BIT |               //
	    VK_COLOR_COMPONENT_B_BIT |               //
	    VK_COLOR_COMPONENT_A_BIT;                //

	/*
	 * We are using VK_BLEND_FACTOR_ONE for the dst alpha write
	 * to make sure that there is a valid value there, makes
	 * the debug UI work when inspecting the scratch images.
	 */
	const VkPipelineColorBlendAttachmentState blend_attachment_state = {
	    .blendEnable = VK_TRUE,
	    .colorWriteMask = all_components,
	    .srcColorBlendFactor = src_blend_factor,
	    .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
	    .colorBlendOp = VK_BLEND_OP_ADD,
	    .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
	    .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
	    .alphaBlendOp = VK_BLEND_OP_ADD,
	};

	const VkPipelineColorBlendStateCreateInfo color_blend_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
	    .attachmentCount = 1,
	    .pAttachments = &blend_attachment_state,
	};

	const VkPipelineDepthStencilStateCreateInfo depth_stencil_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
	    .depthTestEnable = VK_FALSE,
	    .depthWriteEnable = VK_FALSE,
	    .depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL,
	    .front = {.compareOp = VK_COMPARE_OP_ALWAYS},
	    .back = {.compareOp = VK_COMPARE_OP_ALWAYS},
	};


	/*
	 * Dynamic state.
	 */

	const VkDynamicState dynamic_states[] = {
	    VK_DYNAMIC_STATE_VIEWPORT,
	    VK_DYNAMIC_STATE_SCISSOR,
	};

	const VkPipelineDynamicStateCreateInfo dynamic_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
	    .dynamicStateCount = ARRAY_SIZE(dynamic_states),
	    .pDynamicStates = dynamic_states,
	};


	/*
	 * Shaders.
	 */

	const VkPipelineShaderStageCreateInfo shader_stages[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
	        .stage = VK_SHADER_STAGE_VERTEX_BIT,
	        .module = module_vert,
	        .pName = "main",
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
	        .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
	        .module = module_frag,
	        .pName = "main",
	    },
	};


	/*
	 * Bringing it all together.
	 */

	// Dynamic rendering (render_pass is VK_NULL_HANDLE) needs the color
	// attachment format(s) supplied to the pipeline directly.
	const void *pipeline_pnext = NULL;
#ifdef VK_KHR_dynamic_rendering
	VkPipelineRenderingCreateInfo rendering_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
	    .colorAttachmentCount = 1,
	    .pColorAttachmentFormats = &color_format,
	};
	if (vk->features.dynamic_rendering) {
		pipeline_pnext = &rendering_info;
	}
#endif

	VkPipelineCreateFlags pipeline_flags = 0;

#ifdef VK_KHR_fragment_shading_rate
	/*
	 * Foveated pass: the attachment map replaces the pipeline rate
	 * (combiners {keep, replace}), and using a shading-rate attachment
	 * under dynamic rendering must be declared at pipeline create time.
	 */
	VkPipelineFragmentShadingRateStateCreateInfoKHR fsr_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_FRAGMENT_SHADING_RATE_STATE_CREATE_INFO_KHR,
	    .pNext = pipeline_pnext,
	    .fragmentSize = {1, 1},
	    .combinerOps = {VK_FRAGMENT_SHADING_RATE_COMBINER_OP_KEEP_KHR,
	                    VK_FRAGMENT_SHADING_RATE_COMBINER_OP_REPLACE_KHR},
	};
	if (foveation_mechanism == RENDER_FOVEATION_MECHANISM_FSR) {
		pipeline_pnext = &fsr_state;
		pipeline_flags |= VK_PIPELINE_CREATE_RENDERING_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
	}
#endif

	const VkGraphicsPipelineCreateInfo pipeline_info = {
	    .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
	    .pNext = pipeline_pnext,
	    .flags = pipeline_flags,
	    .stageCount = ARRAY_SIZE(shader_stages),
	    .pStages = shader_stages,
	    .pVertexInputState = &vertex_input_state,
	    .pInputAssemblyState = &input_assembly_state,
	    .pViewportState = &viewport_state,
	    .pRasterizationState = &rasterization_state,
	    .pMultisampleState = &multisample_state,
	    .pDepthStencilState = &depth_stencil_state,
	    .pColorBlendState = &color_blend_state,
	    .pDynamicState = &dynamic_state,
	    .layout = pipeline_layout,
	    .renderPass = render_pass,
	    .basePipelineHandle = VK_NULL_HANDLE,
	    .basePipelineIndex = -1,
	};

	VkPipeline pipeline = VK_NULL_HANDLE;
	ret = vk->vkCreateGraphicsPipelines( //
	    vk->device,                      //
	    pipeline_cache,                  //
	    1,                               //
	    &pipeline_info,                  //
	    NULL,                            //
	    &pipeline);                      //
	VK_CHK_AND_RET(ret, "vkCreateGraphicsPipelines");

	*out_pipeline = pipeline;

	return VK_SUCCESS;
}


/*
 *
 * Mesh
 *
 */

struct mesh_params
{
	uint32_t do_timewarp;
};

/*!
 * Pipeline state shared by the classic mesh distortion shaders and the
 * N-layer mesh composite variant; only shader modules and specialization
 * infos differ.
 */
XRT_CHECK_RESULT static VkResult
create_mesh_pipeline_internal(struct vk_bundle *vk,
                              VkRenderPass render_pass,
                              VkFormat color_format,
                              enum render_foveation_mechanism foveation_mechanism,
                              VkPipelineLayout pipeline_layout,
                              VkPipelineCache pipeline_cache,
                              uint32_t src_binding,
                              uint32_t mesh_index_count_total,
                              uint32_t mesh_stride,
                              const VkSpecializationInfo *vert_specialization_info,
                              const VkSpecializationInfo *frag_specialization_info,
                              VkShaderModule mesh_vert,
                              VkShaderModule mesh_frag,
                              VkPipeline *out_mesh_pipeline)
{
	VkResult ret;

	// Might be changed to line for debugging.
	VkPolygonMode polygonMode = VK_POLYGON_MODE_FILL;

	VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	if (mesh_index_count_total > 0) {
		topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
	}

	VkPipelineInputAssemblyStateCreateInfo input_assembly_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
	    .topology = topology,
	    .primitiveRestartEnable = VK_FALSE,
	};

	VkPipelineRasterizationStateCreateInfo rasterization_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
	    .depthClampEnable = VK_FALSE,
	    .rasterizerDiscardEnable = VK_FALSE,
	    .polygonMode = polygonMode,
	    .cullMode = VK_CULL_MODE_BACK_BIT,
	    .frontFace = VK_FRONT_FACE_CLOCKWISE,
	    .lineWidth = 1.0f,
	};

	VkPipelineColorBlendAttachmentState blend_attachment_state = {
	    .blendEnable = VK_FALSE,
	    .colorWriteMask = 0xf,
	};

	VkPipelineColorBlendStateCreateInfo color_blend_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
	    .attachmentCount = 1,
	    .pAttachments = &blend_attachment_state,
	};

	VkPipelineDepthStencilStateCreateInfo depth_stencil_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
	    .depthTestEnable = VK_FALSE,
	    .depthWriteEnable = VK_FALSE,
	    .depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL,
	    .front = {.compareOp = VK_COMPARE_OP_ALWAYS},
	    .back = {.compareOp = VK_COMPARE_OP_ALWAYS},
	};

	VkPipelineViewportStateCreateInfo viewport_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
	    .viewportCount = 1,
	    .scissorCount = 1,
	};

	VkPipelineMultisampleStateCreateInfo multisample_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
	    .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT,
	};

	VkDynamicState dynamic_states[] = {
	    VK_DYNAMIC_STATE_VIEWPORT,
	    VK_DYNAMIC_STATE_SCISSOR,
	};

	VkPipelineDynamicStateCreateInfo dynamic_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
	    .dynamicStateCount = ARRAY_SIZE(dynamic_states),
	    .pDynamicStates = dynamic_states,
	};

	// clang-format off
	VkVertexInputAttributeDescription vertex_input_attribute_descriptions[2] = {
	    {
	        .binding = src_binding,
	        .location = 0,
	        .format = VK_FORMAT_R32G32B32A32_SFLOAT,
	        .offset = 0,
	    },
	    {
	        .binding = src_binding,
	        .location = 1,
	        .format = VK_FORMAT_R32G32B32A32_SFLOAT,
	        .offset = 16,
	    },
	};

	VkVertexInputBindingDescription vertex_input_binding_description[1] = {
	    {
	        .binding = src_binding,
	        .inputRate = VK_VERTEX_INPUT_RATE_VERTEX,
	        .stride = mesh_stride,
	    },
	};

	VkPipelineVertexInputStateCreateInfo vertex_input_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
	    .vertexAttributeDescriptionCount = ARRAY_SIZE(vertex_input_attribute_descriptions),
	    .pVertexAttributeDescriptions = vertex_input_attribute_descriptions,
	    .vertexBindingDescriptionCount = ARRAY_SIZE(vertex_input_binding_description),
	    .pVertexBindingDescriptions = vertex_input_binding_description,
	};
	// clang-format on

	VkPipelineShaderStageCreateInfo shader_stages[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
	        .stage = VK_SHADER_STAGE_VERTEX_BIT,
	        .module = mesh_vert,
	        .pSpecializationInfo = vert_specialization_info,
	        .pName = "main",
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
	        .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
	        .module = mesh_frag,
	        .pSpecializationInfo = frag_specialization_info,
	        .pName = "main",
	    },
	};

	// Dynamic rendering (render_pass is VK_NULL_HANDLE) needs the color
	// attachment format(s) supplied to the pipeline directly.
	const void *pipeline_pnext = NULL;
#ifdef VK_KHR_dynamic_rendering
	VkPipelineRenderingCreateInfo rendering_info = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
	    .colorAttachmentCount = 1,
	    .pColorAttachmentFormats = &color_format,
	};
	if (vk->features.dynamic_rendering) {
		pipeline_pnext = &rendering_info;
	}
#endif

	VkPipelineCreateFlags pipeline_flags = 0;

#ifdef VK_KHR_fragment_shading_rate
	/*
	 * Foveated pass: the attachment map replaces the pipeline rate
	 * (combiners {keep, replace}), and using a shading-rate attachment
	 * under dynamic rendering must be declared at pipeline create time.
	 */
	VkPipelineFragmentShadingRateStateCreateInfoKHR fsr_state = {
	    .sType = VK_STRUCTURE_TYPE_PIPELINE_FRAGMENT_SHADING_RATE_STATE_CREATE_INFO_KHR,
	    .pNext = pipeline_pnext,
	    .fragmentSize = {1, 1},
	    .combinerOps = {VK_FRAGMENT_SHADING_RATE_COMBINER_OP_KEEP_KHR,
	                    VK_FRAGMENT_SHADING_RATE_COMBINER_OP_REPLACE_KHR},
	};
	if (foveation_mechanism == RENDER_FOVEATION_MECHANISM_FSR) {
		pipeline_pnext = &fsr_state;
		pipeline_flags |= VK_PIPELINE_CREATE_RENDERING_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
	}
#endif

	VkGraphicsPipelineCreateInfo pipeline_info = {
	    .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
	    .pNext = pipeline_pnext,
	    .flags = pipeline_flags,
	    .stageCount = ARRAY_SIZE(shader_stages),
	    .pStages = shader_stages,
	    .pVertexInputState = &vertex_input_state,
	    .pInputAssemblyState = &input_assembly_state,
	    .pViewportState = &viewport_state,
	    .pRasterizationState = &rasterization_state,
	    .pMultisampleState = &multisample_state,
	    .pDepthStencilState = &depth_stencil_state,
	    .pColorBlendState = &color_blend_state,
	    .pDynamicState = &dynamic_state,
	    .layout = pipeline_layout,
	    .renderPass = render_pass,
	    .basePipelineHandle = VK_NULL_HANDLE,
	    .basePipelineIndex = -1,
	};

	VkPipeline pipeline = VK_NULL_HANDLE;
	ret = vk->vkCreateGraphicsPipelines( //
	    vk->device,                      //
	    pipeline_cache,                  //
	    1,                               //
	    &pipeline_info,                  //
	    NULL,                            //
	    &pipeline);                      //
	VK_CHK_AND_RET(ret, "vkCreateGraphicsPipelines");

	*out_mesh_pipeline = pipeline;

	return VK_SUCCESS;
}

XRT_CHECK_RESULT static VkResult
create_mesh_pipeline(struct vk_bundle *vk,
                     VkRenderPass render_pass,
                     VkFormat color_format,
                     enum render_foveation_mechanism foveation_mechanism,
                     VkPipelineLayout pipeline_layout,
                     VkPipelineCache pipeline_cache,
                     uint32_t src_binding,
                     uint32_t mesh_index_count_total,
                     uint32_t mesh_stride,
                     const struct mesh_params *params,
                     VkShaderModule mesh_vert,
                     VkShaderModule mesh_frag,
                     VkPipeline *out_mesh_pipeline)
{
#define ENTRY(ID, FIELD)                                                                                               \
	{                                                                                                              \
	    .constantID = ID,                                                                                          \
	    .offset = offsetof(struct mesh_params, FIELD),                                                             \
	    .size = sizeof(params->FIELD),                                                                             \
	}

	VkSpecializationMapEntry vert_entries[] = {
	    ENTRY(0, do_timewarp),
	};
#undef ENTRY

	VkSpecializationInfo vert_specialization_info = {
	    .mapEntryCount = ARRAY_SIZE(vert_entries),
	    .pMapEntries = vert_entries,
	    .dataSize = sizeof(*params),
	    .pData = params,
	};

	return create_mesh_pipeline_internal( //
	    vk,                               //
	    render_pass,                      //
	    color_format,                     //
	    foveation_mechanism,              //
	    pipeline_layout,                  //
	    pipeline_cache,                   //
	    src_binding,                      //
	    mesh_index_count_total,           //
	    mesh_stride,                      //
	    &vert_specialization_info,        //
	    NULL,                             //
	    mesh_vert,                        //
	    mesh_frag,                        //
	    out_mesh_pipeline);               //
}


/*
 *
 * Mesh N-layer composite (single-shader fast path).
 *
 */

/*!
 * Specialization data for one mesh_nlayer.frag / mesh_nlayer.vert pipeline
 * variant. Field order defines the spec map offsets; the constant ids match
 * distortion_nlayer.comp's (see compute_distortion_nlayer_params in
 * render_resources.c), minus id 0 (the distortion LUT texel count; the mesh
 * carries the distortion here) plus the gfx-only id 11 (warp varying count,
 * see mesh_nlayer_warp.inc.glsl) and the gfx-only passthrough ids 30/31,
 * parked far above the shared range.
 */
struct mesh_nlayer_params
{
	uint32_t projection_bounds_test_mask; // id 1 (frag)
	int32_t view_count;                   // id 2 (frag)
	VkBool32 do_distortion;               // id 3 (vert identity-UV switch, frag chroma/mono select)
	int32_t layer_count;                  // id 4
	int32_t scanout_direction;            // id 5 (vert)
	uint32_t layer_types;                 // id 6
	uint32_t layer_unpremult_mask;        // id 7 (frag)
	uint32_t layer_inverted_alpha_mask;   // id 8 (frag)
	uint32_t eye_hidden_mask;             // id 9 (frag)
	VkBool32 do_cac;                      // id 10
	// Gfx-only ids: warp varying array element count (the result of
	// render_gfx_nlayer_warp_count) and the per-slot scanout mask.
	int32_t warp_count;                      // id 11
	uint32_t scanout_compensate_layers_mask; // id 12
	// Gfx-only ids consumed by the passthrough modules only.
	VkBool32 do_passthrough;       // id 30
	VkBool32 do_camera_distortion; // id 31 (passthrough vert only)
};

XRT_CHECK_RESULT static VkResult
create_mesh_nlayer_pipeline(struct vk_bundle *vk,
                            VkRenderPass render_pass,
                            VkFormat color_format,
                            enum render_foveation_mechanism foveation_mechanism,
                            VkPipelineLayout pipeline_layout,
                            VkPipelineCache pipeline_cache,
                            uint32_t src_binding,
                            uint32_t mesh_index_count_total,
                            uint32_t mesh_stride,
                            const struct mesh_nlayer_params *params,
                            VkShaderModule mesh_vert,
                            VkShaderModule mesh_frag,
                            VkPipeline *out_mesh_pipeline)
{
#define ENTRY(ID, FIELD)                                                                                               \
	{                                                                                                              \
	    .constantID = ID,                                                                                          \
	    .offset = offsetof(struct mesh_nlayer_params, FIELD),                                                      \
	    .size = sizeof(params->FIELD),                                                                             \
	}

	// One map for both stages; Vulkan ignores entries whose id a module
	// does not declare, and a single list can't go stale against the
	// shaders' per-stage declarations.
	VkSpecializationMapEntry entries[] = {
	    ENTRY(1, projection_bounds_test_mask),     //
	    ENTRY(2, view_count),                      //
	    ENTRY(3, do_distortion),                   //
	    ENTRY(4, layer_count),                     //
	    ENTRY(5, scanout_direction),               //
	    ENTRY(6, layer_types),                     //
	    ENTRY(7, layer_unpremult_mask),            //
	    ENTRY(8, layer_inverted_alpha_mask),       //
	    ENTRY(9, eye_hidden_mask),                 //
	    ENTRY(10, do_cac),                         //
	    ENTRY(11, warp_count),                     //
	    ENTRY(12, scanout_compensate_layers_mask), //
	    ENTRY(30, do_passthrough),                 //
	    ENTRY(31, do_camera_distortion),           //
	};
#undef ENTRY

	VkSpecializationInfo specialization_info = {
	    .mapEntryCount = ARRAY_SIZE(entries),
	    .pMapEntries = entries,
	    .dataSize = sizeof(*params),
	    .pData = params,
	};

	return create_mesh_pipeline_internal( //
	    vk,                               //
	    render_pass,                      //
	    color_format,                     //
	    foveation_mechanism,              //
	    pipeline_layout,                  //
	    pipeline_cache,                   //
	    src_binding,                      //
	    mesh_index_count_total,           //
	    mesh_stride,                      //
	    &specialization_info,             //
	    &specialization_info,             //
	    mesh_vert,                        //
	    mesh_frag,                        //
	    out_mesh_pipeline);               //
}

// Pack the variant key for linear-search lookup. The scanout mask does not
// fit the packed 64-bit form anymore, so the key is two words; the first
// keeps the compute cache's nlayer_pipeline_key bit layout, with bit 1
// unused:
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
//   hi[16]     do_passthrough       (1 bit)
//   hi[17]     do_camera_distortion (1 bit; ignored when do_passthrough == false)
// Inputs must already be canonicalized by the getter.
static inline void
gfx_nlayer_pipeline_key(uint32_t layer_count,
                        uint32_t layer_types,
                        uint32_t unpremult_mask,
                        uint32_t inverted_alpha_mask,
                        uint32_t eye_hidden_mask,
                        uint32_t scanout_compensate_layers_mask,
                        uint32_t projection_bounds_test_mask,
                        bool do_distortion,
                        bool do_cac,
                        bool do_passthrough,
                        bool do_camera_distortion,
                        enum xrt_scanout_direction scanout_direction,
                        uint64_t out_key[2])
{
	do_cac = do_distortion && do_cac;
	do_camera_distortion = do_passthrough && do_camera_distortion;

	out_key[0] = ((uint64_t)(do_cac ? 1u : 0u) << 57) |               //
	             ((uint64_t)eye_hidden_mask << 41) |                  //
	             ((uint64_t)inverted_alpha_mask << 33) |              //
	             ((uint64_t)unpremult_mask << 25) |                   //
	             ((uint64_t)layer_types << 9) |                       //
	             ((uint64_t)layer_count << 5) |                       //
	             ((uint64_t)scanout_direction << 2) |                 //
	             ((uint64_t)(do_distortion ? 1u : 0u));               //
	out_key[1] = ((uint64_t)(do_camera_distortion ? 1u : 0u) << 17) | //
	             ((uint64_t)(do_passthrough ? 1u : 0u) << 16) |       //
	             ((uint64_t)projection_bounds_test_mask << 8) |       //
	             ((uint64_t)scanout_compensate_layers_mask);          //
}


/*
 *
 * 'Exported' render pass functions.
 *
 */

bool
render_gfx_render_pass_init(struct render_gfx_render_pass *rgrp,
                            struct render_resources *r,
                            VkFormat format,
                            VkAttachmentLoadOp load_op,
                            VkImageLayout final_layout,
                            bool foveated)
{
	struct vk_bundle *vk = r->vk;
	VkResult ret;

	// Set first so a mid-init failure leaves a struct fini can clean up.
	rgrp->r = r;

	// Decided before pipeline creation, all pipelines depend on it. A
	// selected mechanism implies dynamic rendering, the only path the
	// foveation attachments are wired into.
	rgrp->foveation_mechanism = foveated ? r->foveation.mechanism : RENDER_FOVEATION_MECHANISM_NONE;

	// Dynamic rendering does not use a VkRenderPass object; the pipelines and
	// the target attachment carry the format instead.
	rgrp->render_pass = VK_NULL_HANDLE;
	if (!vk->features.dynamic_rendering) {
		ret = create_implicit_render_pass( //
		    vk,                            //
		    format,                        // target_format
		    load_op,                       //
		    final_layout,                  //
		    &rgrp->render_pass);           // out_render_pass
		VK_CHK_WITH_RET(ret, "create_implicit_render_pass", false);
		VK_NAME_RENDER_PASS(vk, rgrp->render_pass, "render_gfx_render_pass render pass");
	}

	struct mesh_params simple_params = {
	    .do_timewarp = false,
	};

	ret = create_mesh_pipeline(    //
	    vk,                        //
	    rgrp->render_pass,         //
	    format,                    //
	    rgrp->foveation_mechanism, //
	    r->mesh.pipeline_layout,   //
	    r->pipeline_cache,         //
	    r->mesh.src_binding,       //
	    r->mesh.index_count_total, //
	    r->mesh.stride,            //
	    &simple_params,            //
	    r->shaders->mesh_vert,     //
	    r->shaders->mesh_frag,     //
	    &rgrp->mesh.pipeline);     // out_mesh_pipeline
	VK_CHK_WITH_RET(ret, "create_mesh_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->mesh.pipeline, "render_gfx_render_pass mesh pipeline");

	struct mesh_params timewarp_params = {
	    .do_timewarp = true,
	};

	ret = create_mesh_pipeline(         //
	    vk,                             //
	    rgrp->render_pass,              //
	    format,                         //
	    rgrp->foveation_mechanism,      //
	    r->mesh.pipeline_layout,        //
	    r->pipeline_cache,              //
	    r->mesh.src_binding,            //
	    r->mesh.index_count_total,      //
	    r->mesh.stride,                 //
	    &timewarp_params,               //
	    r->shaders->mesh_vert,          //
	    r->shaders->mesh_frag,          //
	    &rgrp->mesh.pipeline_timewarp); // out_mesh_pipeline
	VK_CHK_WITH_RET(ret, "create_mesh_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->mesh.pipeline_timewarp, "render_gfx_render_pass mesh pipeline timewarp");

	const VkBlendFactor blend_factor_premultiplied_alpha = VK_BLEND_FACTOR_ONE;
	const VkBlendFactor blend_factor_unpremultiplied_alpha = VK_BLEND_FACTOR_SRC_ALPHA;

	// Cylinder
	ret = create_layer_pipeline(                    //
	    vk,                                         //
	    rgrp->render_pass,                          //
	    format,                                     //
	    rgrp->foveation_mechanism,                  //
	    r->gfx.layer.shared.pipeline_layout,        //
	    r->pipeline_cache,                          //
	    blend_factor_premultiplied_alpha,           // src_blend_factor
	    r->shaders->layer_cylinder_vert,            //
	    r->shaders->layer_cylinder_frag,            //
	    &rgrp->layer.cylinder_premultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.cylinder_premultiplied_alpha,
	                 "render_gfx_render_pass cylinder premultiplied alpha");

	ret = create_layer_pipeline(                      //
	    vk,                                           //
	    rgrp->render_pass,                            //
	    format,                                       //
	    rgrp->foveation_mechanism,                    //
	    r->gfx.layer.shared.pipeline_layout,          //
	    r->pipeline_cache,                            //
	    blend_factor_unpremultiplied_alpha,           // src_blend_factor
	    r->shaders->layer_cylinder_vert,              // module_vert
	    r->shaders->layer_cylinder_frag,              // module_frag
	    &rgrp->layer.cylinder_unpremultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.cylinder_unpremultiplied_alpha,
	                 "render_gfx_render_pass cylinder unpremultiplied alpha");

	// Equirect2
	ret = create_layer_pipeline(                     //
	    vk,                                          //
	    rgrp->render_pass,                           //
	    format,                                      //
	    rgrp->foveation_mechanism,                   //
	    r->gfx.layer.shared.pipeline_layout,         //
	    r->pipeline_cache,                           //
	    blend_factor_premultiplied_alpha,            // src_blend_factor
	    r->shaders->layer_equirect2_vert,            // module_vert
	    r->shaders->layer_equirect2_frag,            // module_frag
	    &rgrp->layer.equirect2_premultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.equirect2_premultiplied_alpha,
	                 "render_gfx_render_pass equirect2 premultiplied alpha");

	ret = create_layer_pipeline(                       //
	    vk,                                            //
	    rgrp->render_pass,                             //
	    format,                                        //
	    rgrp->foveation_mechanism,                     //
	    r->gfx.layer.shared.pipeline_layout,           //
	    r->pipeline_cache,                             //
	    blend_factor_unpremultiplied_alpha,            // src_blend_factor
	    r->shaders->layer_equirect2_vert,              // module_vert
	    r->shaders->layer_equirect2_frag,              // module_frag
	    &rgrp->layer.equirect2_unpremultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.equirect2_unpremultiplied_alpha,
	                 "render_gfx_render_pass equirect2 unpremultiplied alpha");

	// Projection.
	ret = create_layer_pipeline(                //
	    vk,                                     //
	    rgrp->render_pass,                      //
	    format,                                 //
	    rgrp->foveation_mechanism,              //
	    r->gfx.layer.shared.pipeline_layout,    //
	    r->pipeline_cache,                      //
	    blend_factor_premultiplied_alpha,       // src_blend_factor
	    r->shaders->layer_projection_vert,      // module_vert
	    r->shaders->layer_projection_frag,      // module_frag
	    &rgrp->layer.proj_premultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.proj_premultiplied_alpha,
	                 "render_gfx_render_pass projection premultiplied alpha");

	ret = create_layer_pipeline(                  //
	    vk,                                       //
	    rgrp->render_pass,                        //
	    format,                                   //
	    rgrp->foveation_mechanism,                //
	    r->gfx.layer.shared.pipeline_layout,      //
	    r->pipeline_cache,                        //
	    blend_factor_unpremultiplied_alpha,       // src_blend_factor
	    r->shaders->layer_projection_vert,        // module_vert
	    r->shaders->layer_projection_frag,        // module_frag
	    &rgrp->layer.proj_unpremultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.proj_unpremultiplied_alpha,
	                 "render_gfx_render_pass projection unpremultiplied alpha");

	// Quad
	ret = create_layer_pipeline(                //
	    vk,                                     //
	    rgrp->render_pass,                      //
	    format,                                 //
	    rgrp->foveation_mechanism,              //
	    r->gfx.layer.shared.pipeline_layout,    //
	    r->pipeline_cache,                      //
	    blend_factor_premultiplied_alpha,       // src_blend_factor
	    r->shaders->layer_quad_vert,            // module_vert
	    r->shaders->layer_quad_frag,            // module_frag
	    &rgrp->layer.quad_premultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.quad_premultiplied_alpha, "render_gfx_render_pass quad premultiplied alpha");

	ret = create_layer_pipeline(                  //
	    vk,                                       //
	    rgrp->render_pass,                        //
	    format,                                   //
	    rgrp->foveation_mechanism,                //
	    r->gfx.layer.shared.pipeline_layout,      //
	    r->pipeline_cache,                        //
	    blend_factor_unpremultiplied_alpha,       // src_blend_factor
	    r->shaders->layer_quad_vert,              // module_vert
	    r->shaders->layer_quad_frag,              // module_frag
	    &rgrp->layer.quad_unpremultiplied_alpha); // out_pipeline
	VK_CHK_WITH_RET(ret, "create_layer_pipeline", false);
	VK_NAME_PIPELINE(vk, rgrp->layer.quad_unpremultiplied_alpha,
	                 "render_gfx_render_pass quad unpremultiplied alpha");

	// Gfx N-layer fast path: pipelines are lazy-built per variant by
	// render_gfx_render_pass_get_or_create_nlayer_pipeline, nothing to
	// create eagerly. Enabled only when the static per-N layouts exist.
	rgrp->nlayer.enabled = r->gfx.nlayer.enabled && r->gfx.nlayer.effective_nlayer_max > 0;

	// Set fields.
	rgrp->format = format;
	rgrp->sample_count = VK_SAMPLE_COUNT_1_BIT;
	rgrp->load_op = load_op;
	rgrp->final_layout = final_layout;

	return true;
}

void
render_gfx_render_pass_fini(struct render_gfx_render_pass *rgrp)
{
	// Never initialized, or already finished.
	if (rgrp->r == NULL) {
		return;
	}

	struct vk_bundle *vk = rgrp->r->vk;

	D(RenderPass, rgrp->render_pass);
	D(Pipeline, rgrp->mesh.pipeline);
	D(Pipeline, rgrp->mesh.pipeline_timewarp);

	D(Pipeline, rgrp->layer.cylinder_premultiplied_alpha);
	D(Pipeline, rgrp->layer.cylinder_unpremultiplied_alpha);
	D(Pipeline, rgrp->layer.equirect2_premultiplied_alpha);
	D(Pipeline, rgrp->layer.equirect2_unpremultiplied_alpha);
	D(Pipeline, rgrp->layer.proj_premultiplied_alpha);
	D(Pipeline, rgrp->layer.proj_unpremultiplied_alpha);
	D(Pipeline, rgrp->layer.quad_premultiplied_alpha);
	D(Pipeline, rgrp->layer.quad_unpremultiplied_alpha);

	for (uint32_t i = 0; i < rgrp->nlayer.pipeline_count; ++i) {
		D(Pipeline, rgrp->nlayer.pipelines[i].pipeline);
	}
	rgrp->nlayer.pipeline_count = 0;

	U_ZERO(rgrp);
}

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
                                                     bool do_passthrough,
                                                     bool do_camera_distortion,
                                                     enum xrt_scanout_direction scanout_direction,
                                                     VkPipeline *out_pipeline)
{
	struct render_resources *r = rgrp->r;

	if (!rgrp->nlayer.enabled) {
		U_LOG_E("gfx nlayer pipeline: fast path not enabled");
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}
	if (layer_count > r->gfx.nlayer.effective_nlayer_max || (layer_count == 0 && !do_passthrough)) {
		U_LOG_E("gfx nlayer pipeline: layer_count %u out of range (zero requires passthrough, max %u)",
		        layer_count, r->gfx.nlayer.effective_nlayer_max);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}
	if ((uint32_t)scanout_direction >= 5) {
		U_LOG_E("gfx nlayer pipeline: scanout_direction %d out of range [0, 4]", (int)scanout_direction);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	// Mask off any bits the shader can't see (slots >= layer_count), so
	// callers passing garbage in unused bits don't fork extra pipeline
	// variants.
	const uint32_t type_used_bits = layer_count * RENDER_NLAYER_TYPE_BITS;
	const uint32_t type_used_mask = (type_used_bits >= 32) ? 0xFFFFFFFFu : ((1u << type_used_bits) - 1u);
	layer_types &= type_used_mask;
	eye_hidden_mask &= type_used_mask; // same 2-bits-per-slot width as layer_types
	const uint32_t slot_used_mask = (layer_count >= 32) ? 0xFFFFFFFFu : ((1u << layer_count) - 1u);
	unpremult_mask &= slot_used_mask;
	inverted_alpha_mask &= slot_used_mask;

	do_cac = do_distortion && do_cac;
	do_camera_distortion = do_passthrough && do_camera_distortion;
	if (do_passthrough && r->gfx.nlayer.passthrough_descriptor_set_layout == VK_NULL_HANDLE) {
		U_LOG_E("gfx nlayer pipeline: passthrough requested without a descriptor set layout");
		return VK_ERROR_FEATURE_NOT_PRESENT;
	}

	// Projection slots (type code 0) among the used slots; with the other
	// axes this decides the warp varying array size (id 11). Derived from
	// the already-masked layer_types so it can never disagree with what
	// the shader's per-slot dispatch sees.
	uint32_t proj_slots = 0;
	uint32_t proj_slot_count = 0;
	for (uint32_t i = 0; i < layer_count; i++) {
		uint32_t slot_type = (layer_types >> (i * RENDER_NLAYER_TYPE_BITS)) & 3u;
		if (slot_type == (uint32_t)RENDER_NLAYER_TYPE_PROJECTION) {
			proj_slots |= 1u << i;
			proj_slot_count++;
		}
	}

	// Only projection slots consume the mask; dead bits must not fork variants.
	projection_bounds_test_mask &= proj_slots;

	// Scanout-mask canonicalization (a set bit disables compensation for
	// that slot): a global-flash display compensates nothing, and a
	// variant compensating nothing is a global-flash variant — except
	// with an active passthrough, which is world-locked by definition and
	// reprojects whenever the panel rolls, so it keeps the scanout axis
	// live on its own. (With timewarp off the passthrough transforms all
	// hold the same matrix and the kept lerp degenerates correctly.)
	scanout_compensate_layers_mask &= slot_used_mask;
	if (scanout_direction == XRT_SCANOUT_DIRECTION_NONE) {
		scanout_compensate_layers_mask = slot_used_mask;
	}
	if (scanout_compensate_layers_mask == slot_used_mask && !do_passthrough) {
		scanout_direction = XRT_SCANOUT_DIRECTION_NONE;
	}

	// Shared non-projection ray classes actually present, mirroring the
	// shader's warp_have_uncomp_rays / warp_have_comp_rays derivation.
	const uint32_t nonproj_slots = slot_used_mask & ~proj_slots;
	const uint32_t ray_class_count = //
	    ((nonproj_slots & scanout_compensate_layers_mask) != 0 ? 1u : 0u) +
	    ((nonproj_slots & ~scanout_compensate_layers_mask) != 0 ? 1u : 0u);

	uint32_t warp_count = render_gfx_nlayer_warp_count( //
	    do_cac,                                         //
	    proj_slot_count,                                //
	    ray_class_count,                                //
	    do_passthrough);                                //
	if (warp_count > r->gfx.nlayer.max_warp_count) {
		U_LOG_E("gfx nlayer pipeline: variant needs %u warp elements, device budget is %u", warp_count,
		        r->gfx.nlayer.max_warp_count);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	uint64_t key[2];
	gfx_nlayer_pipeline_key(layer_count, layer_types, unpremult_mask, inverted_alpha_mask, eye_hidden_mask,
	                        scanout_compensate_layers_mask, projection_bounds_test_mask, do_distortion, do_cac,
	                        do_passthrough, do_camera_distortion, scanout_direction, key);

	for (uint32_t i = 0; i < rgrp->nlayer.pipeline_count; ++i) {
		if (rgrp->nlayer.pipelines[i].key[0] == key[0] && rgrp->nlayer.pipelines[i].key[1] == key[1]) {
			*out_pipeline = rgrp->nlayer.pipelines[i].pipeline;
			return VK_SUCCESS;
		}
	}

	if (rgrp->nlayer.pipeline_count >= RENDER_NLAYER_PIPELINE_CACHE_CAP) {
		U_LOG_E("gfx nlayer pipeline cache full (%u variants); bump RENDER_NLAYER_PIPELINE_CACHE_CAP",
		        (uint32_t)RENDER_NLAYER_PIPELINE_CACHE_CAP);
		return VK_ERROR_OUT_OF_DEVICE_MEMORY;
	}

	struct mesh_nlayer_params params = {
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
	    .warp_count = (int32_t)warp_count,
	    .scanout_compensate_layers_mask = scanout_compensate_layers_mask,
	    .do_passthrough = do_passthrough ? VK_TRUE : VK_FALSE,
	    .do_camera_distortion = do_camera_distortion ? VK_TRUE : VK_FALSE,
	};
	uint32_t descriptor_layer_count = layer_count > 0 ? layer_count : 1;

	VkPipeline pipeline = VK_NULL_HANDLE;
	VkResult ret = create_mesh_nlayer_pipeline( //
	    r->vk,                                  //
	    rgrp->render_pass,                      //
	    rgrp->format,                           //
	    rgrp->foveation_mechanism,              //
	    do_passthrough ? r->gfx.nlayer.passthrough_pipeline_layouts[descriptor_layer_count - 1]
	                   : r->gfx.nlayer.pipeline_layouts[descriptor_layer_count - 1],
	    r->pipeline_cache,                                        //
	    r->mesh.src_binding,                                      //
	    r->mesh.index_count_total,                                //
	    r->mesh.stride,                                           //
	    &params,                                                  //
	    do_passthrough ? r->shaders->mesh_nlayer_passthrough_vert //
	                   : r->shaders->mesh_nlayer_vert,            //
	    layer_count == 0
	        ? r->shaders->mesh_nlayer_passthrough_only_frag
	        : (do_passthrough ? r->shaders->mesh_nlayer_passthrough_frag : r->shaders->mesh_nlayer_frag),
	    &pipeline); //
	if (ret != VK_SUCCESS) {
		return ret;
	}

	VK_NAME_PIPELINE(r->vk, pipeline, "render_gfx_render_pass nlayer pipeline");

	uint32_t slot = rgrp->nlayer.pipeline_count++;
	rgrp->nlayer.pipelines[slot].key[0] = key[0];
	rgrp->nlayer.pipelines[slot].key[1] = key[1];
	rgrp->nlayer.pipelines[slot].pipeline = pipeline;

	*out_pipeline = pipeline;
	return VK_SUCCESS;
}


/*
 *
 * 'Exported' target resources functions.
 *
 */

bool
render_gfx_target_resources_init(struct render_gfx_target_resources *rtr,
                                 struct render_resources *r,
                                 struct render_gfx_render_pass *rgrp,
                                 struct render_foveation_map *foveation_map,
                                 VkImage target_image,
                                 VkImageView target,
                                 VkExtent2D extent)
{
	struct vk_bundle *vk = r->vk;

	// Foveated pass: all its pipelines declare the attachment, so the
	// caller must provide the shared map to attach.
	assert((rgrp->foveation_mechanism != RENDER_FOVEATION_MECHANISM_NONE) == (foveation_map != NULL));

	// Set fields.
	rtr->r = r;
	rtr->rgrp = rgrp;
	rtr->foveation = foveation_map;
	rtr->image = target_image;
	rtr->view = target;
	rtr->render_area = (VkRect2D){
	    .offset = {0, 0},
	    .extent = extent,
	};

	// Dynamic rendering renders straight into the image view; no framebuffer.
	if (!vk->features.dynamic_rendering) {
		VkResult ret = create_framebuffer( //
		    vk,                            //
		    target,                        // image_view
		    rgrp->render_pass,             //
		    extent.width,                  //
		    extent.height,                 //
		    &rtr->framebuffer);            // out_external_framebuffer
		VK_CHK_WITH_RET(ret, "create_framebuffer", false);
		VK_NAME_FRAMEBUFFER(vk, rtr->framebuffer, "render_gfx_target_resources framebuffer");
	}


	return true;
}

void
render_gfx_target_resources_fini(struct render_gfx_target_resources *rtr)
{
	// Never initialized, or already finished.
	if (rtr->r == NULL) {
		return;
	}

	struct vk_bundle *vk = vk_from_rtr(rtr);

	D(Framebuffer, rtr->framebuffer);

	U_ZERO(rtr);
}


/*
 *
 * 'Exported' rendering functions.
 *
 */

bool
render_gfx_init(struct render_gfx *render, struct render_resources *r)
{
	// Init fields.
	render->r = r;

	// Used to sub-allocate UBOs from, restart from scratch each frame.
	render_sub_alloc_tracker_init(&render->ubo_tracker, &r->gfx.shared_ubo);

	return true;
}

bool
render_gfx_begin(struct render_gfx *render)
{
	struct vk_bundle *vk = vk_from_render(render);
	VkResult ret;

	ret = vk->vkResetCommandPool(vk->device, render->r->cmd_pool, 0);
	VK_CHK_WITH_RET(ret, "vkResetCommandPool", false);


	VkCommandBufferBeginInfo begin_info = {
	    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
	    .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
	};

	ret = vk->vkBeginCommandBuffer( //
	    render->r->cmd,             //
	    &begin_info);               //
	VK_CHK_WITH_RET(ret, "vkBeginCommandBuffer", false);

	vk->vkCmdResetQueryPool(   //
	    render->r->cmd,        //
	    render->r->query_pool, //
	    0,                     // firstQuery
	    2);                    // queryCount

	vk->vkCmdWriteTimestamp(               //
	    render->r->cmd,                    //
	    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, // pipelineStage
	    render->r->query_pool,             //
	    0);                                // query

	return true;
}

bool
render_gfx_end(struct render_gfx *render)
{
	struct vk_bundle *vk = vk_from_render(render);
	VkResult ret;

	vk->vkCmdWriteTimestamp(                  //
	    render->r->cmd,                       //
	    VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, // pipelineStage
	    render->r->query_pool,                //
	    1);                                   // query

	ret = vk->vkEndCommandBuffer(render->r->cmd);
	VK_CHK_WITH_RET(ret, "vkEndCommandBuffer", false);

	return true;
}

void
render_gfx_fini(struct render_gfx *render)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;

	// Reclaim all descriptor sets.
	vk->vkResetDescriptorPool(              //
	    vk->device,                         //
	    r->gfx.ubo_and_src_descriptor_pool, //
	    0);                                 //

	// Also the gfx N-layer fast path's sets, pool only exists when enabled.
	if (r->gfx.nlayer.descriptor_pool != VK_NULL_HANDLE) {
		vk->vkResetDescriptorPool(         //
		    vk->device,                    //
		    r->gfx.nlayer.descriptor_pool, //
		    0);                            //
	}

	// This "reclaims" the allocated UBOs.
	U_ZERO(render);
}


/*
 *
 * 'Exported' draw functions.
 *
 */

bool
render_gfx_begin_target(struct render_gfx *render,
                        struct render_gfx_target_resources *rtr,
                        const VkClearColorValue *color)
{
	struct vk_bundle *vk = vk_from_render(render);

	assert(render->rtr == NULL);
	render->rtr = rtr;

	// A staged foveation map update must land before the pass reads it.
	if (rtr->foveation != NULL) {
		render_foveation_map_record_pending(render->r, rtr->foveation, render->r->cmd);
	}

	begin_render_pass(  //
	    vk,             //
	    render->r->cmd, //
	    rtr,            //
	    color);         //

	return true;
}

void
render_gfx_end_target(struct render_gfx *render)
{
	struct vk_bundle *vk = vk_from_render(render);

	struct render_gfx_target_resources *rtr = render->rtr;
	assert(rtr != NULL);
	render->rtr = NULL;

	// Stop the [shared] render pass.
	end_render_pass(vk, render->r->cmd, rtr);
}

void
render_gfx_clear_color_attachment(struct render_gfx *render, const VkClearColorValue *color)
{
	struct vk_bundle *vk = vk_from_render(render);

	assert(render->rtr != NULL);

	const VkClearAttachment clear_attachment = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .colorAttachment = 0,
	    .clearValue.color = *color,
	};
	const VkClearRect clear_rect = {
	    .rect = render->rtr->render_area,
	    .baseArrayLayer = 0,
	    .layerCount = 1,
	};
	vk->vkCmdClearAttachments(render->r->cmd, 1, &clear_attachment, 1, &clear_rect);
}

void
render_gfx_begin_view(struct render_gfx *render,
                      uint32_t view,
                      const struct render_viewport_data *viewport_data,
                      const render_scissor_data_t *scissor_data)
{
	struct vk_bundle *vk = vk_from_render(render);

	// We currently only support two views.
	assert(view == 0 || view == 1);
	assert(render->rtr != NULL);
	assert(viewport_data != NULL);
	assert(scissor_data != NULL);

	/*
	 * Viewport
	 */

	const VkViewport viewport = {
	    .x = (float)viewport_data->x,
	    .y = (float)viewport_data->y,
	    .width = (float)viewport_data->w,
	    .height = (float)viewport_data->h,
	    .minDepth = 0.0f,
	    .maxDepth = 1.0f,
	};

	vk->vkCmdSetViewport(render->r->cmd, //
	                     0,              // firstViewport
	                     1,              // viewportCount
	                     &viewport);     //

	/*
	 * Scissor
	 */

	const VkRect2D scissor = {
	    .offset =
	        {
	            .x = scissor_data->x,
	            .y = scissor_data->y,
	        },
	    .extent =
	        {
	            .width = scissor_data->w,
	            .height = scissor_data->h,
	        },
	};

	vk->vkCmdSetScissor(render->r->cmd, //
	                    0,              // firstScissor
	                    1,              // scissorCount
	                    &scissor);      //
}

void
render_gfx_end_view(struct render_gfx *render)
{
	//! Must have a current target.
	assert(render->rtr != NULL);
}

XRT_CHECK_RESULT VkResult
render_gfx_mesh_alloc_and_write(struct render_gfx *render,
                                const struct render_gfx_mesh_ubo_data *data,
                                VkSampler src_sampler,
                                VkImageView src_image_view,
                                VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;

	return do_ubo_and_src_alloc_and_write(  //
	    render,                             //
	    r->mesh.ubo_binding,                //
	    data,                               // ubo_ptr
	    sizeof(*data),                      // ubo_size
	    r->mesh.src_binding,                //
	    src_sampler,                        //
	    src_image_view,                     //
	    r->gfx.ubo_and_src_descriptor_pool, //
	    r->mesh.descriptor_set_layout,      //
	    out_descriptor_set);                //
}

XRT_CHECK_RESULT VkResult
render_gfx_mesh_nlayer_alloc_and_write(struct render_gfx *render,
                                       const struct render_gfx_mesh_nlayer_ubo_data *data,
                                       uint32_t layer_count,
                                       const VkSampler *src_samplers,
                                       const VkImageView *src_image_views,
                                       VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;
	struct vk_bundle *vk = vk_from_render(render);
	VkResult ret;

	assert(layer_count <= r->gfx.nlayer.effective_nlayer_max);
	uint32_t descriptor_layer_count = layer_count > 0 ? layer_count : 1;

	struct render_sub_alloc ubo = XRT_STRUCT_INIT;
	ret = render_sub_alloc_ubo_alloc_and_write( //
	    vk,                                     //
	    &render->ubo_tracker,                   //
	    data,                                   //
	    sizeof(*data),                          //
	    &ubo);                                  //
	VK_CHK_AND_RET(ret, "render_sub_alloc_ubo_alloc_and_write");

	VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
	VkDescriptorSetLayout descriptor_set_layout = r->gfx.nlayer.descriptor_set_layouts[descriptor_layer_count - 1];
	ret = vk_create_descriptor_set(    //
	    vk,                            //
	    r->gfx.nlayer.descriptor_pool, //
	    descriptor_set_layout,         //
	    &descriptor_set);              //
	VK_CHK_AND_RET(ret, "vk_create_descriptor_set");

	// Source slots: exactly layer_count * view_count entries, the layout
	// this set was allocated from declares the matching descriptorCount.
	VkDescriptorImageInfo src_image_info[RENDER_NLAYER_MAX * XRT_MAX_VIEWS];
	const uint32_t src_count = layer_count * r->view_count;
	for (uint32_t i = 0; i < src_count; ++i) {
		src_image_info[i] = (VkDescriptorImageInfo){
		    .sampler = src_samplers[i],
		    .imageView = src_image_views[i],
		    .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
		};
	}

	VkDescriptorBufferInfo buffer_info = {
	    .buffer = ubo.buffer,
	    .offset = ubo.offset,
	    .range = ubo.size,
	};

	VkWriteDescriptorSet write_descriptor_sets[2] = {
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = r->mesh.src_binding,
	        .descriptorCount = src_count,
	        .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
	        .pImageInfo = src_image_info,
	    },
	    {
	        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
	        .dstSet = descriptor_set,
	        .dstBinding = r->mesh.ubo_binding,
	        .descriptorCount = 1,
	        .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
	        .pBufferInfo = &buffer_info,
	    },
	};

	// The passthrough-only shader has no application-source binding.
	uint32_t first_write = layer_count == 0 ? 1 : 0;
	vk->vkUpdateDescriptorSets(                          //
	    vk->device,                                      //
	    ARRAY_SIZE(write_descriptor_sets) - first_write, // descriptorWriteCount
	    &write_descriptor_sets[first_write],             // pDescriptorWrites
	    0,                                               // descriptorCopyCount
	    NULL);                                           // pDescriptorCopies

	*out_descriptor_set = descriptor_set;

	return VK_SUCCESS;
}

void
render_gfx_mesh_draw(struct render_gfx *render, uint32_t mesh_index, VkDescriptorSet descriptor_set, bool do_timewarp)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;


	/*
	 * Descriptors and pipeline.
	 */

	VkDescriptorSet descriptor_sets[1] = {descriptor_set};
	vk->vkCmdBindDescriptorSets(         //
	    r->cmd,                          //
	    VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
	    r->mesh.pipeline_layout,         // layout
	    0,                               // firstSet
	    ARRAY_SIZE(descriptor_sets),     // descriptorSetCount
	    descriptor_sets,                 // pDescriptorSets
	    0,                               // dynamicOffsetCount
	    NULL);                           // pDynamicOffsets

	// Select which pipeline we want.
	VkPipeline pipeline =
	    do_timewarp ? render->rtr->rgrp->mesh.pipeline_timewarp : render->rtr->rgrp->mesh.pipeline;

	vk->vkCmdBindPipeline(               //
	    r->cmd,                          //
	    VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
	    pipeline);                       // pipeline


	/*
	 * Vertex buffer.
	 */

	VkBuffer buffers[1] = {r->mesh.vbo.buffer};
	VkDeviceSize offsets[1] = {0};
	static_assert(ARRAY_SIZE(buffers) == ARRAY_SIZE(offsets), "buffers and offsets array size mismatch");

	vk->vkCmdBindVertexBuffers( //
	    r->cmd,                 //
	    0,                      // firstBinding
	    ARRAY_SIZE(buffers),    // bindingCount
	    buffers,                // pBuffers
	    offsets);               // pOffsets


	/*
	 * Draw with indices or not?
	 */

	if (r->mesh.index_count_total > 0) {
		vk->vkCmdBindIndexBuffer(  //
		    r->cmd,                //
		    r->mesh.ibo.buffer,    // buffer
		    0,                     // offset
		    VK_INDEX_TYPE_UINT32); // indexType

		vk->vkCmdDrawIndexed(                  //
		    r->cmd,                            //
		    r->mesh.index_counts[mesh_index],  // indexCount
		    1,                                 // instanceCount
		    r->mesh.index_offsets[mesh_index], // firstIndex
		    0,                                 // vertexOffset
		    0);                                // firstInstance
	} else {
		vk->vkCmdDraw(            //
		    r->cmd,               //
		    r->mesh.vertex_count, // vertexCount
		    1,                    // instanceCount
		    0,                    // firstVertex
		    0);                   // firstInstance
	}
}

void
render_gfx_mesh_nlayer_draw(struct render_gfx *render,
                            uint32_t view_index,
                            uint32_t layer_count,
                            VkDescriptorSet descriptor_set,
                            VkDescriptorSet passthrough_descriptor_set,
                            VkPipeline pipeline,
                            bool do_passthrough)
{
	struct vk_bundle *vk = vk_from_render(render);
	struct render_resources *r = render->r;

	assert(pipeline != VK_NULL_HANDLE);
	assert(layer_count <= r->gfx.nlayer.effective_nlayer_max);
	assert(layer_count > 0 || do_passthrough);
	uint32_t descriptor_layer_count = layer_count > 0 ? layer_count : 1;

	VkPipelineLayout pipeline_layout = do_passthrough
	                                       ? r->gfx.nlayer.passthrough_pipeline_layouts[descriptor_layer_count - 1]
	                                       : r->gfx.nlayer.pipeline_layouts[descriptor_layer_count - 1];

	VkDescriptorSet descriptor_sets[2] = {descriptor_set, passthrough_descriptor_set};
	vk->vkCmdBindDescriptorSets(         //
	    r->cmd,                          //
	    VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
	    pipeline_layout,                 // layout
	    0,                               // firstSet
	    do_passthrough ? 2 : 1,          // descriptorSetCount
	    descriptor_sets,                 // pDescriptorSets
	    0,                               // dynamicOffsetCount
	    NULL);                           // pDynamicOffsets

	// The shader's view selector, the gfx gl_GlobalInvocationID.z.
	vk->vkCmdPushConstants(                                        //
	    r->cmd,                                                    //
	    pipeline_layout,                                           //
	    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, //
	    0,                                                         // offset
	    sizeof(view_index),                                        // size
	    &view_index);                                              // pValues

	vk->vkCmdBindPipeline(               //
	    r->cmd,                          //
	    VK_PIPELINE_BIND_POINT_GRAPHICS, // pipelineBindPoint
	    pipeline);                       // pipeline

	VkBuffer buffers[1] = {r->mesh.vbo.buffer};
	VkDeviceSize offsets[1] = {0};

	vk->vkCmdBindVertexBuffers( //
	    r->cmd,                 //
	    0,                      // firstBinding
	    ARRAY_SIZE(buffers),    // bindingCount
	    buffers,                // pBuffers
	    offsets);               // pOffsets

	if (r->mesh.index_count_total > 0) {
		vk->vkCmdBindIndexBuffer(  //
		    r->cmd,                //
		    r->mesh.ibo.buffer,    // buffer
		    0,                     // offset
		    VK_INDEX_TYPE_UINT32); // indexType

		vk->vkCmdDrawIndexed(                  //
		    r->cmd,                            //
		    r->mesh.index_counts[view_index],  // indexCount
		    1,                                 // instanceCount
		    r->mesh.index_offsets[view_index], // firstIndex
		    0,                                 // vertexOffset
		    0);                                // firstInstance
	} else {
		vk->vkCmdDraw(            //
		    r->cmd,               //
		    r->mesh.vertex_count, // vertexCount
		    1,                    // instanceCount
		    0,                    // firstVertex
		    0);                   // firstInstance
	}
}


/*
 *
 * 'Exported' layer functions.
 *
 */

XRT_CHECK_RESULT VkResult
render_gfx_layer_cylinder_alloc_and_write(struct render_gfx *render,
                                          const struct render_gfx_layer_cylinder_data *data,
                                          VkSampler src_sampler,
                                          VkImageView src_image_view,
                                          VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;

	return do_ubo_and_src_alloc_and_write(         //
	    render,                                    //
	    RENDER_BINDING_LAYER_SHARED_UBO,           // ubo_binding
	    data,                                      // ubo_ptr
	    sizeof(*data),                             // ubo_size
	    RENDER_BINDING_LAYER_SHARED_SRC,           // src_binding
	    src_sampler,                               //
	    src_image_view,                            //
	    r->gfx.ubo_and_src_descriptor_pool,        //
	    r->gfx.layer.shared.descriptor_set_layout, //
	    out_descriptor_set);                       //
}

XRT_CHECK_RESULT VkResult
render_gfx_layer_equirect2_alloc_and_write(struct render_gfx *render,
                                           const struct render_gfx_layer_equirect2_data *data,
                                           VkSampler src_sampler,
                                           VkImageView src_image_view,
                                           VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;

	return do_ubo_and_src_alloc_and_write(         //
	    render,                                    //
	    RENDER_BINDING_LAYER_SHARED_UBO,           // ubo_binding
	    data,                                      // ubo_ptr
	    sizeof(*data),                             // ubo_size
	    RENDER_BINDING_LAYER_SHARED_SRC,           // src_binding
	    src_sampler,                               //
	    src_image_view,                            //
	    r->gfx.ubo_and_src_descriptor_pool,        //
	    r->gfx.layer.shared.descriptor_set_layout, //
	    out_descriptor_set);                       //
}

XRT_CHECK_RESULT VkResult
render_gfx_layer_projection_alloc_and_write(struct render_gfx *render,
                                            const struct render_gfx_layer_projection_data *data,
                                            VkSampler src_sampler,
                                            VkImageView src_image_view,
                                            VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;

	return do_ubo_and_src_alloc_and_write(         //
	    render,                                    //
	    RENDER_BINDING_LAYER_SHARED_UBO,           // ubo_binding
	    data,                                      // ubo_ptr
	    sizeof(*data),                             // ubo_size
	    RENDER_BINDING_LAYER_SHARED_SRC,           // src_binding
	    src_sampler,                               //
	    src_image_view,                            //
	    r->gfx.ubo_and_src_descriptor_pool,        //
	    r->gfx.layer.shared.descriptor_set_layout, //
	    out_descriptor_set);                       //
}

XRT_CHECK_RESULT VkResult
render_gfx_layer_quad_alloc_and_write(struct render_gfx *render,
                                      const struct render_gfx_layer_quad_data *data,
                                      VkSampler src_sampler,
                                      VkImageView src_image_view,
                                      VkDescriptorSet *out_descriptor_set)
{
	struct render_resources *r = render->r;

	return do_ubo_and_src_alloc_and_write(         //
	    render,                                    //
	    RENDER_BINDING_LAYER_SHARED_UBO,           // ubo_binding
	    data,                                      // ubo_ptr
	    sizeof(*data),                             // ubo_size
	    RENDER_BINDING_LAYER_SHARED_SRC,           // src_binding
	    src_sampler,                               //
	    src_image_view,                            //
	    r->gfx.ubo_and_src_descriptor_pool,        //
	    r->gfx.layer.shared.descriptor_set_layout, //
	    out_descriptor_set);                       //
}

void
render_gfx_layer_cylinder(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set)
{
	VkPipeline pipeline =                                              //
	    premultiplied_alpha                                            //
	        ? render->rtr->rgrp->layer.cylinder_premultiplied_alpha    //
	        : render->rtr->rgrp->layer.cylinder_unpremultiplied_alpha; //

	// One per degree.
	uint32_t subdivisions = 360;

	// One edge on either endstop and one between each subdivision.
	uint32_t edges = subdivisions + 1;

	// With triangle strip we get 2 vertices per edge.
	uint32_t vertex_count = edges * 2;

	dispatch_no_vbo(     //
	    render,          //
	    vertex_count,    // vertex_count
	    pipeline,        //
	    descriptor_set); //
}

void
render_gfx_layer_equirect2(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set)
{
	VkPipeline pipeline =                                               //
	    premultiplied_alpha                                             //
	        ? render->rtr->rgrp->layer.equirect2_premultiplied_alpha    //
	        : render->rtr->rgrp->layer.equirect2_unpremultiplied_alpha; //

	// Hardcoded to 4 vertices.
	dispatch_no_vbo(     //
	    render,          //
	    4,               // vertex_count
	    pipeline,        //
	    descriptor_set); //
}

void
render_gfx_layer_projection(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set)
{
	VkPipeline pipeline =                                          //
	    premultiplied_alpha                                        //
	        ? render->rtr->rgrp->layer.proj_premultiplied_alpha    //
	        : render->rtr->rgrp->layer.proj_unpremultiplied_alpha; //

	// Hardcoded to 4 vertices.
	dispatch_no_vbo(     //
	    render,          //
	    4,               // vertex_count
	    pipeline,        //
	    descriptor_set); //
}

void
render_gfx_layer_quad(struct render_gfx *render, bool premultiplied_alpha, VkDescriptorSet descriptor_set)
{
	VkPipeline pipeline =                                          //
	    premultiplied_alpha                                        //
	        ? render->rtr->rgrp->layer.quad_premultiplied_alpha    //
	        : render->rtr->rgrp->layer.quad_unpremultiplied_alpha; //

	// Hardcoded to 4 vertices.
	dispatch_no_vbo(     //
	    render,          //
	    4,               // vertex_count
	    pipeline,        //
	    descriptor_set); //
}
