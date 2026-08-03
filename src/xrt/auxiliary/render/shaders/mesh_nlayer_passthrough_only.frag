// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

// Zero-app-layer passthrough: only the fused camera background is drawn.
// The vertex stage is mesh_nlayer_passthrough.vert specialized with
// layer_count = 0, whose sole output is the passthrough warp element
// (warp_count = 1); this shader has no application-source descriptors at
// all, so it replaces mesh_nlayer_frag.inc.glsl entirely instead of
// specializing it — a sources[0] array would be invalid GLSL.

#version 460
#extension GL_GOOGLE_include_directive : require

#define XRT_MESH_NLAYER_PASSTHROUGH 1
#include "mesh_nlayer.inc.glsl"
#include "mesh_nlayer_warp.inc.glsl"
#include "srgb.inc.glsl"

// Keep the YCbCr samplers in distinct bindings. Vulkan requires combined
// image samplers that enable YCbCr conversion to be indexed only by constant
// integral expressions.
layout(set = 1, binding = 0) uniform sampler2D passthrough_source_left;
layout(set = 1, binding = 1) uniform sampler2D passthrough_source_right;

layout(location = 0) out vec4 out_color;

void main()
{
	vec3 linear_rgb = vec3(0.0);
	if (warp[warp_pt_base].z >= 0.0) {
		vec2 uv = warp[warp_pt_base].xy;
		vec3 encoded;
		if (pc.view_index == 0u) {
			encoded = textureLod(passthrough_source_left, uv, 0.0).rgb;
		} else {
			encoded = textureLod(passthrough_source_right, uv, 0.0).rgb;
		}
		linear_rgb = from_srgb_to_linear(encoded);
	}

	// The render target format performs the sRGB encode.
	out_color = vec4(linear_rgb, 1.0);
}
