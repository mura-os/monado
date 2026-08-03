// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

// Vertex shader for the gfx N-layer fast path: rasterizes the distortion mesh
// and runs the whole warp stage per vertex — per-projection-slot timewarp or
// static source mapping with per-slot scanout compensation, plus up to two
// shared per-channel view-ray classes — via warp_emit in
// mesh_nlayer_warp.inc.glsl. The fragment stage only keeps the non-linear
// per-pixel tail (perspective divide, normalize, intersection, sampling,
// compositing). The passthrough variant additionally projects the fused
// camera background per vertex (passthrough_emit), the last warp element.
// The includer defines XRT_MESH_NLAYER_PASSTHROUGH to 0 or 1.

#define WARP_STAGE_VERT
#include "mesh_nlayer.inc.glsl"
#include "mesh_nlayer_warp.inc.glsl"

layout (location = 0) in vec4 in_pos_ruv;
layout (location = 1) in vec4 in_guv_buv;

out gl_PerVertex
{
	vec4 gl_Position;
};

void main()
{
	uint iz = pc.view_index;

	mat2x2 rot = {
		ubo.vertex_rot[iz].xy,
		ubo.vertex_rot[iz].zw,
	};

	gl_Position = vec4(rot * in_pos_ruv.xy, 0.0f, 1.0f);

	float scanout_t = compute_scanout_fraction(gl_Position.xy);

	// Per-channel distorted UVs from the mesh attributes (the baked
	// distortion LUT); the green/geometric channel feeds all three when
	// chromatic-aberration correction is off, identity UVs in the mesh's
	// pre-rotation space when distortion is off entirely.
	vec2 ruv;
	vec2 guv;
	vec2 buv;
	if (do_cac) {
		ruv = in_pos_ruv.zw;
		guv = in_guv_buv.xy;
		buv = in_guv_buv.zw;
	} else {
		vec2 uv = do_distortion ? in_guv_buv.xy : in_pos_ruv.xy * 0.5 + 0.5;
		ruv = uv;
		guv = uv;
		buv = uv;
	}

	warp_emit(ruv, guv, buv, scanout_t, iz);

#if XRT_MESH_NLAYER_PASSTHROUGH
	passthrough_emit(guv, scanout_t, iz);
#endif
}
