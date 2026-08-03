// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

// Gfx N-layer fast path: samples up to MAX_NLAYER projection / quad /
// cylinder / equirect2 layer sources and alpha composites them in submission
// order — one draw per view. The gfx twin of distortion_nlayer.comp; the
// ray-shape intersection and compositing code is kept identical, keep those
// in sync. Differences:
//  - Geometric distortion, optionally with per-channel chromatic-aberration
//    correction, comes from the rasterized mesh UVs instead of LUT images.
//  - The whole warp stage (per-layer projection timewarp with scanout
//    compensation, per-channel scanout-rotated view rays) runs per vertex,
//    see mesh_nlayer_warp.inc.glsl; this shader only keeps the non-linear
//    per-pixel tail: perspective divide, normalize and intersections.
//  - Output goes to the color attachment with blending disabled and alpha
//    1.0; the sRGB encode is done by the render target format, not in-shader.
//  - The passthrough variant (XRT_MESH_NLAYER_PASSTHROUGH, defined 0 or 1 by
//    the including module) composites the fused camera background under the
//    app layers, sampled at the per-vertex projected UV in the last warp
//    element.

#include "mesh_nlayer.inc.glsl"
// Shared spec constants (do_distortion, layer_count, scanout_direction,
// layer_types, do_cac, warp_count, scanout_compensate_layers_mask) and the
// warp varying array this shader consumes.
#include "mesh_nlayer_warp.inc.glsl"

// Spec constant ids and meanings match distortion_nlayer.comp (id 0, the
// distortion LUT texel count, has no gfx equivalent and is unused).
// Bit i set = projection slot i needs the out-of-FOV bounds test. The C
// side clears a bit when border sampling already handles out-of-FOV
// correctly for that slot, see crg_nlayer_fast_path for the rules.
layout(constant_id = 1) const uint projection_bounds_test_mask = 0;

// Number of views (1 = mono, 2 = stereo). Sizes the sources array so only
// the descriptors the device actually needs are allocated.
layout(constant_id = 2) const int view_count = 2;

// Per-slot unpremultiplied-alpha bitmap, 1 bit per slot.
layout(constant_id = 7) const uint layer_unpremult_mask = 0;

// Per-slot inverted-alpha bitmap, 1 bit per slot. The `!= 0u` gate (see decode)
// strips the whole inversion path when no layer is inverted.
layout(constant_id = 8) const uint layer_inverted_alpha_mask = 0;

// Packed per-slot eye-hidden bitmap: 2 bits per slot, `1` in bit (i*2+v)
// means slot i is hidden in view v. Encoded as (XRT_LAYER_EYE_VISIBILITY_BOTH
// XOR visibility) on the C side so that BOTH (the default and overwhelming
// common case, plus all projection slots) maps to all-zero bits. With
// layer_count and eye_hidden_mask both spec consts, the per-slot
// `(hidden_bits >> iz) & 1` check folds: BOTH slots compile away entirely;
// LEFT/RIGHT slots compile to a single uniform compare on iz, skipping the
// texture fetch for the wrong-eye draw.
layout(constant_id = 9) const uint eye_hidden_mask = 0;

// One sampler per (layer, view) pair, flat-indexed as [layer * view_count + view].
// The C-side descriptor set layout for this layer_count declares
// descriptorCount = layer_count * view_count, an exact match.
layout (binding = 0) uniform sampler2D sources[layer_count * view_count];
#if XRT_MESH_NLAYER_PASSTHROUGH
// Keep the YCbCr samplers in distinct bindings. Vulkan requires combined
// image samplers that enable YCbCr conversion to be indexed only by constant
// integral expressions, so a dynamically indexed per-view array is invalid.
layout(set = 1, binding = 0) uniform sampler2D passthrough_source_left;
layout(set = 1, binding = 1) uniform sampler2D passthrough_source_right;
#include "srgb.inc.glsl"
#endif

layout (location = 0) out vec4 out_color;

struct ChromaSample
{
	vec3 rgb;
	vec3 alpha;
};

struct ChromaAccum
{
	vec3 rgb;
};

// One per-channel view-space ray set; each non-projection slot reads the
// class its scanout_compensate_layers_mask bit selects.
struct ChromaRays
{
	vec3 r;
	vec3 g;
	vec3 b;
};

struct ChromaInputs
{
	ChromaRays uncomp;
	ChromaRays comp;
};

struct MonoInputs
{
	vec3 uncomp_ray;
	vec3 comp_ray;
};

struct QuadSampleData
{
	mat4 inverse_transform;
	vec4 post_transform;
	vec3 normal;
	vec3 position;
	vec2 extent;
	vec2 half_extent;
};

struct WrapSampleData
{
	mat4 mvi;
	vec4 post_transform;
	vec3 ray_origin;
	vec4 params;
};

struct WrapBounds
{
	vec2 lower;
	vec2 upper;
	vec2 span;
	vec2 local_to_uv_scale;
	vec4 post_transform;
};

// Perspective-divide tail of the per-vertex projection warp. The C side
// pre-multiplies the [-1, 1] -> [0, 1] remap and the layer's sub-image rect
// into the timewarp matrices (render_time_warp_matrix_fold_remap_and_rect),
// so the divide yields the final source UV directly. @p elem is the slot's first
// element in the warp proj block, tracked by the composite loops' running
// counter (see mesh_nlayer_warp.inc.glsl on why there is no index helper).
vec2 warp_proj_uv(int elem, uint chan)
{
	vec3 v = warp[elem + int(chan)];
	return v.xy * (1.0 / max(v.z, 0.00001));
}

// Out-of-FOV test for a projection slot's final source UV. bounds is the
// slot's post_transform entry, repurposed as a scale/bias that maps the
// sub-image rect onto [-1, 1]^2 (render_calc_proj_bounds_transform). The
// abs form handles a flip_y-negated rect height without min/max, and a
// behind-projection w <= 0 lands far outside via warp_proj_uv's clamp.
bool proj_uv_inside(vec2 uv, vec4 bounds)
{
	vec2 q = fma(uv, bounds.zw, bounds.xy);
	return max(abs(q.x), abs(q.y)) <= 1.0;
}

vec2 apply_post_transform(vec2 uv, vec4 post)
{
	return uv * post.zw + post.xy;
}

ChromaSample chroma_sample_zero()
{
	return ChromaSample(vec3(0.0), vec3(0.0));
}

#if XRT_MESH_NLAYER_PASSTHROUGH
// Camera background at this pixel: the vertex stage projected the camera
// UV and validity margin into the last warp element (passthrough_emit); a
// negative margin means no camera coverage and interpolation fades partially
// valid triangles out at the zero crossing.
vec3 sample_passthrough(uint iz)
{
	if (!do_passthrough || warp[warp_pt_base].z < 0.0) {
		return vec3(0.0);
	}
	vec3 encoded;
	if (iz == 0u) {
		encoded = textureLod(passthrough_source_left, warp[warp_pt_base].xy, 0.0).rgb;
	} else {
		encoded = textureLod(passthrough_source_right, warp[warp_pt_base].xy, 0.0).rgb;
	}
	// The YCbCr conversion produces gamma-encoded UNORM RGB. The target is
	// sRGB, so composite in linear space and let the attachment encode once.
	return from_srgb_to_linear(encoded);
}
#endif

ChromaSample chroma_sample_from_rgb(vec4 r, vec4 g, vec4 b)
{
	return ChromaSample(vec3(r.r, g.g, b.b), vec3(r.a, g.a, b.a));
}

// Per-feature mask decode; folds to a literal per unrolled slot. Unpremult keeps
// the pre-feature bit position, so a no-inversion mask is bit-identical to it.
bool slot_unpremultiplied_alpha(uint slot)
{
	return ((layer_unpremult_mask >> slot) & 1u) != 0u;
}

bool slot_inverted_alpha(uint slot)
{
	// `!= 0u` folds to false when no layer is inverted, stripping the path.
	return layer_inverted_alpha_mask != 0u &&
	       ((layer_inverted_alpha_mask >> slot) & 1u) != 0u;
}

bool slot_needs_projection_bounds_test(uint slot)
{
	return projection_bounds_test_mask != 0u &&
	       ((projection_bounds_test_mask >> slot) & 1u) != 0u;
}

// Inverted alpha (spec 12.28): call only on in-bounds samples (out-of-bounds keep
// alpha 0). In-bounds-only for ALL bounded types here — deliberately unlike
// layer.comp's unconditional cylinder/equirect2 invert. Don't "fix" it.
float invert_alpha(float a, uint slot)
{
	if (slot_inverted_alpha(slot)) {
		a = 1.0 - a;
	}
	return a;
}

ChromaSample chroma_sample_premultiply_if_needed(ChromaSample s, uint layer)
{
	if (slot_unpremultiplied_alpha(layer)) {
		s.rgb *= s.alpha;
	}
	return s;
}

ChromaAccum chroma_over(ChromaAccum accum, ChromaSample s)
{
	accum.rgb = accum.rgb * (1.0 - s.alpha) + s.rgb;
	return accum;
}

// Hybrid wrap chroma tail: 3 per-channel texture samples gated on the per-
// channel bounds checks. Used by sample_cylinder_chroma and
// sample_equirect2_chroma, which derive R/B layer UVs from G via a Jacobian
// and need explicit per-channel in-bounds flags. Quad chroma doesn't use
// this — it just calls sample_quad_prepared three times (the prepared
// helper already returns vec4(0) on miss) — and projection chroma uses
// plain selects, see sample_projection_chroma.
ChromaSample apply_chroma_sample(uint src_idx,
                                 vec2 g_uv, vec2 r_uv, vec2 b_uv,
                                 bool g_inside, bool r_inside, bool b_inside,
                                 uint slot)
{
	if (!g_inside && !r_inside && !b_inside) {
		return chroma_sample_zero();
	}
	vec4 r = r_inside ? texture(sources[src_idx], r_uv) : vec4(0.0);
	vec4 g = g_inside ? texture(sources[src_idx], g_uv) : vec4(0.0);
	vec4 b = b_inside ? texture(sources[src_idx], b_uv) : vec4(0.0);
	ChromaSample s = chroma_sample_from_rgb(r, g, b);
	// Inverted alpha, in-bounds channels only (branchless select; out-of-bounds
	// channels stay 0). Outer check folds away on non-inverted slots.
	if (slot_inverted_alpha(slot)) {
		s.alpha = mix(s.alpha, 1.0 - s.alpha, bvec3(r_inside, g_inside, b_inside));
	}
	return s;
}

WrapBounds make_wrap_bounds(vec2 lower, vec2 upper, vec4 post_transform)
{
	vec2 span = upper - lower;
	return WrapBounds(lower, upper, span, post_transform.zw / span, post_transform);
}

bool wrap_coord_inside(vec2 coord, WrapBounds bounds)
{
	return coord.x >= bounds.lower.x && coord.x <= bounds.upper.x &&
	       coord.y >= bounds.lower.y && coord.y <= bounds.upper.y;
}

vec2 wrap_coord_to_uv(vec2 coord, WrapBounds bounds)
{
	vec2 local = (coord - bounds.lower) / bounds.span;
	return apply_post_transform(local, bounds.post_transform);
}

vec4 sample_wrap_coord(uint src_idx, vec2 coord, WrapBounds bounds, uint slot)
{
	if (!wrap_coord_inside(coord, bounds)) {
		return vec4(0.0);
	}
	vec4 c = texture(sources[src_idx], wrap_coord_to_uv(coord, bounds));
	c.a = invert_alpha(c.a, slot);  // in-bounds only
	return c;
}

ChromaSample sample_wrap_chroma_offsets(uint src_idx, vec2 g_coord, vec2 r_delta, vec2 b_delta, WrapBounds bounds, uint slot)
{
	vec2 r_coord = g_coord + r_delta;
	vec2 b_coord = g_coord + b_delta;

	bool g_inside = wrap_coord_inside(g_coord, bounds);
	bool r_inside = wrap_coord_inside(r_coord, bounds);
	bool b_inside = wrap_coord_inside(b_coord, bounds);
	if (!g_inside && !r_inside && !b_inside) {
		return chroma_sample_zero();
	}

	vec2 g_uv = wrap_coord_to_uv(g_coord, bounds);
	vec2 r_uv = g_uv + r_delta * bounds.local_to_uv_scale;
	vec2 b_uv = g_uv + b_delta * bounds.local_to_uv_scale;
	return apply_chroma_sample(src_idx, g_uv, r_uv, b_uv, g_inside, r_inside, b_inside, slot);
}

vec4 mono_sample_premultiply_if_needed(vec4 s, uint layer)
{
	if (slot_unpremultiplied_alpha(layer)) {
		s.rgb *= s.a;
	}
	return s;
}

vec3 mono_over(vec3 accum_rgb, vec4 s)
{
	return accum_rgb * (1.0 - s.a) + s.rgb;
}

// The vertex stage already applied distortion and the scanout rotation;
// only the (exactly interpolating) normalize is per pixel, once per
// existing ray class.
ChromaInputs build_chroma_inputs()
{
	ChromaInputs inputs;

	if (warp_have_uncomp_rays) {
		inputs.uncomp.r = normalize(warp[warp_uncomp_ray_base + 0]);
		inputs.uncomp.g = normalize(warp[warp_uncomp_ray_base + 1]);
		inputs.uncomp.b = normalize(warp[warp_uncomp_ray_base + 2]);
	}

	if (warp_have_comp_rays) {
		inputs.comp.r = normalize(warp[warp_comp_ray_base + 0]);
		inputs.comp.g = normalize(warp[warp_comp_ray_base + 1]);
		inputs.comp.b = normalize(warp[warp_comp_ray_base + 2]);
	}

	return inputs;
}

MonoInputs build_mono_inputs()
{
	MonoInputs inputs;

	if (warp_have_uncomp_rays) {
		inputs.uncomp_ray = normalize(warp[warp_uncomp_ray_base]);
	}

	if (warp_have_comp_rays) {
		inputs.comp_ray = normalize(warp[warp_comp_ray_base]);
	}

	return inputs;
}

// The condition folds, so each unrolled slot statically reads one class.
ChromaRays select_chroma_rays(ChromaInputs inputs, uint slot)
{
	return slot_scanout_compensated(slot) ? inputs.comp : inputs.uncomp;
}

vec3 select_mono_ray(MonoInputs inputs, uint slot)
{
	return slot_scanout_compensated(slot) ? inputs.comp_ray : inputs.uncomp_ray;
}

QuadSampleData load_quad_sample_data(uint idx)
{
	vec2 extent = ubo.quads[idx].extent;
	return QuadSampleData(ubo.quads[idx].inverse_transform, ubo.post_transform[idx],
	                      normalize(ubo.quads[idx].normal.xyz), ubo.quads[idx].position.xyz, extent, extent * 0.5);
}

WrapSampleData load_wrap_sample_data(uint idx)
{
	mat4 mvi = ubo.wraps[idx].mv_inverse;
	// mvi * vec4(0,0,0,1) is by definition the 4th column of mvi; GLSL's
	// column-major indexing gives it directly. Saves a mat4×vec4 per wrap
	// layer per pixel (ACO doesn't reliably fold the constant-vec form).
	return WrapSampleData(mvi, ubo.post_transform[idx], mvi[3].xyz, ubo.wraps[idx].params);
}

// Sample one quad layer at this output pixel. View-space ray-plane
// intersection matches layer.comp's do_quad(); off-plane / behind-camera
// pixels return zero alpha so the over blend treats them as transparent.
//
// Caller passes the pre-computed view-space ray direction (mesh-corrected,
// scanout-compensated, pre-normalised in build_*_inputs). The "mesh-corrected"
// part is essential: using raw display UVs would skip lens distortion and
// place the quad at the wrong angular position (visibly closer/larger
// off-axis).
vec4 sample_quad_prepared(uint src_idx, vec3 ray_dir_view, QuadSampleData q, uint slot)
{
	float denominator = dot(ray_dir_view, q.normal);
	// Front-facing plane only; positive or near-zero denominator means the
	// ray is parallel or hitting the back face.
	if (denominator >= -0.00001) {
		return vec4(0.0);
	}

	vec3 camera = vec3(0.0);
	float dist = dot(camera - q.position, q.normal);
	float t = (dot(camera, q.normal) + dist) / -denominator;
	if (t < 0.0) {
		return vec4(0.0);
	}

	vec3 intersection = camera + t * ray_dir_view;
	vec2 ps = (q.inverse_transform * vec4(intersection, 1.0)).xy;

	if (abs(ps.x) > q.half_extent.x || abs(ps.y) > q.half_extent.y) {
		return vec4(0.0);
	}

	vec2 plane_uv = ps / q.extent + 0.5;
	plane_uv = apply_post_transform(plane_uv, q.post_transform);
	vec4 c = texture(sources[src_idx], plane_uv);
	c.a = invert_alpha(c.a, slot);  // in-bounds only (matches do_quad)
	return c;
}

vec4 sample_quad(uint idx, uint src_idx, vec3 ray_dir_view, uint slot)
{
	QuadSampleData q = load_quad_sample_data(idx);
	return sample_quad_prepared(src_idx, ray_dir_view, q, slot);
}

// Quad chroma: per-channel ray-plane intersection via sample_quad_prepared.
// The intersection math for a plane is cheap (no transcendentals); we just
// call it three times. sample_quad_prepared already returns vec4(0) for
// off-quad rays, so chroma_sample_from_rgb naturally drops those channels'
// contribution — same chroma fringe at geometric edges as per-ray. No
// Jacobian needed for quad.
ChromaSample sample_quad_chroma(uint idx, uint src_idx, ChromaRays rays, uint slot)
{
	QuadSampleData q = load_quad_sample_data(idx);
	vec4 r = sample_quad_prepared(src_idx, rays.r, q, slot);
	vec4 g = sample_quad_prepared(src_idx, rays.g, q, slot);
	vec4 b = sample_quad_prepared(src_idx, rays.b, q, slot);
	return chroma_sample_from_rgb(r, g, b);
}

const float PI = 3.14159265358979323846;

// dUV/dRay_view chain rule for the longitude axis on a cylinder or sphere,
// evaluated at the G intersection's direction. Approximation: treats
// δ_dir_model ≈ mat3(mvi) × δ_ray_view. Shared between
// sample_cylinder_chroma and sample_equirect2_chroma: both have
// lon = atan(dir.x, -dir.z) / (2π) + 0.5 with the same derivative shape.
vec3 wrap_lon_jacobian_dray(vec3 dir, mat3 mvi3)
{
	float x = dir.x;
	float z = dir.z;
	float xz2 = max(x * x + z * z, 1e-8);
	vec3 dlon_ddir = vec3(-z, 0.0, x) / (xz2 * 2.0 * PI);
	return dlon_ddir * mvi3;
}

// Ray vs Y-axis-aligned cylinder of given radius centred at origin in model
// space. Returns true if the ray meets the cylinder geometry; `out dir` is the
// unit direction from the cylinder origin to the far intersection point. Used
// by both sample_cylinder_prepared (mono) and sample_cylinder_chroma. Caller
// still bounds-checks the resulting layer-UV against the angular extent.
bool cylinder_intersect_dir(vec3 ray_m, vec3 ray_origin, float radius, out vec3 dir)
{
	if (radius == 0.0) {
		// CPU passes 0 for "+INFINITY" radius; directional skybox cylinder.
		dir = ray_m;
		return true;
	}
	const vec3 axis = vec3(0.0, 1.0, 0.0);
	float card = dot(axis, ray_m);
	float caoc = dot(axis, ray_origin);
	float a = 1.0 - card * card;
	float b = dot(ray_origin, ray_m) - caoc * card;
	float c = dot(ray_origin, ray_origin) - caoc * caoc - radius * radius;
	float h = b * b - a * c;
	if (h < 0.0) {
		return false;
	}
	h = sqrt(h);
	// Far intersection (inside the cylinder shell) — matches layer.comp's
	// behaviour for the typical skydome-style use case.
	float t = (-b + h) / a;
	if (t < 0.0) {
		return false;
	}
	dir = normalize(ray_origin + ray_m * t);
	return true;
}

// Ray vs sphere of given radius centred at origin in model space. Returns
// true if the ray meets the sphere; `out dir` is the unit direction from the
// sphere origin to the far intersection point. Used by both
// sample_equirect2_prepared (mono) and sample_equirect2_chroma.
bool sphere_intersect_dir(vec3 ray_m, vec3 ray_origin, float radius, out vec3 dir)
{
	if (radius == 0.0) {
		dir = ray_m;
		return true;
	}
	float B = dot(ray_origin, ray_m);
	vec3 QC = ray_origin - B * ray_m;
	float H = radius * radius - dot(QC, QC);
	if (H < 0.0) {
		return false;
	}
	H = sqrt(H);
	float t = -B + H;
	if (t < 0.0) {
		return false;
	}
	dir = normalize(ray_origin + ray_m * t);
	return true;
}

// Sample one cylinder layer at this output pixel. View-space ray is
// transformed into the cylinder's model space via mv_inverse, intersected
// with the cylinder (Y-axis aligned in model space), and mapped to
// longitude / Y-tan for texture-UV lookup. Mirrors layer.comp's do_cylinder.
// Caller passes the pre-computed view-space ray direction (pre-normalised in
// build_*_inputs; mv_inverse is rigid with scale = 1 so length is preserved
// through the matrix multiply). The view-space ray origin is always (0, 0, 0).
vec4 sample_cylinder_prepared(uint src_idx, vec3 ray_dir_view, WrapSampleData wrap, uint slot)
{
	// w=0 means the translation column contributes nothing; mat3(mvi) is the
	// equivalent transform on a direction. Saves ~13 ops per call.
	vec3 ray_m = mat3(wrap.mvi) * ray_dir_view;
	float radius = wrap.params.x;
	float central_angle = wrap.params.y;
	float aspect_ratio = wrap.params.z;

	vec3 dir_from_cyl;
	if (!cylinder_intersect_dir(ray_m, wrap.ray_origin, radius, dir_from_cyl)) {
		return vec4(0.0);
	}

	float lon = atan(dir_from_cyl.x, -dir_from_cyl.z) / (2.0 * PI) + 0.5;  // [0, 1]
	// Y-axis: project to tangent at unit radius, equivalent to -y/sqrt(1-y²).
	// max(0.0, ...) guards against FP roundoff after normalize pushing |y|
	// slightly past 1 — a negative radicand would NaN, and the bounds check
	// below doesn't catch NaN (NaN < x is false). At true poles the sqrt
	// returns 0 and y → ±inf, which the bounds check correctly rejects.
	float y = -dir_from_cyl.y / sqrt(max(0.0, 1.0 - dir_from_cyl.y * dir_from_cyl.y));

	float chan = central_angle / (2.0 * PI);
	float uhan = 0.5 + chan / 2.0;
	float lhan = 0.5 - chan / 2.0;
	float height = central_angle / aspect_ratio;
	float ymin = -height / 2.0;
	float ymax = height / 2.0;
	WrapBounds bounds = make_wrap_bounds(vec2(lhan, ymin), vec2(uhan, ymax), wrap.post_transform);

	return sample_wrap_coord(src_idx, vec2(lon, y), bounds, slot);
}

vec4 sample_cylinder(uint idx, uint src_idx, vec3 ray_dir_view, uint slot)
{
	WrapSampleData wrap = load_wrap_sample_data(idx);
	return sample_cylinder_prepared(src_idx, ray_dir_view, wrap, slot);
}

// Merged cyl/equirect2 chroma: one full intersection with the G ray, then
// derive R/B layer UVs via a first-order Taylor expansion of (lon, second)(dir)
// around G's intersection. is_sphere=false → cylinder, is_sphere=true →
// equirect2. The two paths share lon math, mvi3 transform, dispatch, and
// bounds structure; they differ only in (intersection helper, second-axis map,
// second-axis partial derivative, bounds definition).
//
// Approximation layers (apply to both):
//   (1) δ_dir ≈ mat3(mvi) × δ_ray_view — exact at radius=0, drops the
//       intersection-depth derivative at radius>0.
//   (2) δ(lon, second) ≈ ∂(lon, second)/∂dir · δ_dir — first-order regardless
//       of radius, since lon and the second axis are nonlinear in dir.
// Combined error is typically sub-pixel at real-lens chroma magnitudes.
//
// Codegen shape: `is_sphere` is intentionally a bool parameter rather than
// a struct/helper abstraction. ACO folds it through the inlined wrappers
// (sample_cylinder_chroma → ..., false; sample_equirect2_chroma → ..., true),
// so each call site compiles to only the relevant branch and the output is
// identical to having two separate functions. Earlier refactors that pulled
// the shared scaffolding into helper-structs or chained helper functions
// regressed throughput ~0.4% in measurements (ACO inlines but doesn't fully
// erase the function-call boundary). Keep this merged-function shape.
ChromaSample sample_wrap_layer_chroma(uint idx, uint src_idx, ChromaRays rays, bool is_sphere, uint slot)
{
	WrapSampleData wrap = load_wrap_sample_data(idx);

	// CPU passes unit_scale to math_matrix_4x4_model, so mat3(mvi) is rigid.
	mat3 mvi3 = mat3(wrap.mvi);
	vec3 ray_m = mvi3 * rays.g;
	float radius = wrap.params.x;

	// Silhouette divergence: if G ray misses geometry we drop the whole
	// layer's chroma; R/B might still hit but the divergence band is sub-pixel.
	vec3 dir;
	bool hit = is_sphere ? sphere_intersect_dir(ray_m, wrap.ray_origin, radius, dir)
	                     : cylinder_intersect_dir(ray_m, wrap.ray_origin, radius, dir);
	if (!hit) {
		return chroma_sample_zero();
	}

	float x = dir.x;
	float y = clamp(dir.y, -1.0, 1.0);  // guard FP roundoff after normalize()
	float z = dir.z;
	float lon = atan(x, -z) / (2.0 * PI) + 0.5;
	float one_minus_y2 = max(1.0 - y * y, 1e-8);
	float sqrt_omy2 = sqrt(one_minus_y2);

	float second;
	float dsecond_dy;
	vec2 lower, upper;
	float chan_x = wrap.params.y / (2.0 * PI);  // y == central[_horizontal]_angle in both
	float lhan = 0.5 - chan_x / 2.0;
	float uhan = 0.5 + chan_x / 2.0;
	if (is_sphere) {
		// equirect2: lat = acos(y)/π; ∂lat/∂y = -1/(π·√(1-y²))
		second = acos(y) / PI;
		dsecond_dy = -1.0 / (PI * sqrt_omy2);
		float lvan = wrap.params.w / PI + 0.5;
		float uvan = wrap.params.z / PI + 0.5;
		lower = vec2(lhan, lvan);
		upper = vec2(uhan, uvan);
	} else {
		// cylinder: y_proj = -y/√(1-y²); ∂y_proj/∂y = -1/(1-y²)^1.5
		second = -y / sqrt_omy2;
		dsecond_dy = -1.0 / (one_minus_y2 * sqrt_omy2);
		float height = wrap.params.y / wrap.params.z;
		lower = vec2(lhan, -height / 2.0);
		upper = vec2(uhan, height / 2.0);
	}
	WrapBounds bounds = make_wrap_bounds(lower, upper, wrap.post_transform);

	vec3 dlon_dray = wrap_lon_jacobian_dray(dir, mvi3);
	vec3 dsecond_dray = vec3(0.0, dsecond_dy, 0.0) * mvi3;
	vec3 dr = rays.r - rays.g;
	vec3 db = rays.b - rays.g;

	float dlon_r = dot(dlon_dray, dr);
	float dlon_b = dot(dlon_dray, db);
	float dsecond_r = dot(dsecond_dray, dr);
	float dsecond_b = dot(dsecond_dray, db);

	return sample_wrap_chroma_offsets(src_idx, vec2(lon, second),
	                                  vec2(dlon_r, dsecond_r), vec2(dlon_b, dsecond_b),
	                                  bounds, slot);
}

ChromaSample sample_cylinder_chroma(uint idx, uint src_idx, ChromaRays rays, uint slot)
{
	return sample_wrap_layer_chroma(idx, src_idx, rays, false, slot);
}

// Sample one equirect2 layer at this output pixel. View-space ray transformed
// into the layer's model space, intersected with a sphere, mapped to lat/lon
// for texture-UV lookup. Mirrors layer.comp's do_equirect2. Same calling
// convention as sample_cylinder.
vec4 sample_equirect2_prepared(uint src_idx, vec3 ray_dir_view, WrapSampleData wrap, uint slot)
{
	// w=0 means the translation column contributes nothing; mat3(mvi) is the
	// equivalent transform on a direction. Saves ~13 ops per call.
	vec3 ray_m = mat3(wrap.mvi) * ray_dir_view;
	float radius = wrap.params.x;
	float central_horizontal_angle = wrap.params.y;
	float upper_vertical_angle = wrap.params.z;
	float lower_vertical_angle = wrap.params.w;

	vec3 dir_from_sph;
	if (!sphere_intersect_dir(ray_m, wrap.ray_origin, radius, dir_from_sph)) {
		return vec4(0.0);
	}

	float lon = atan(dir_from_sph.x, -dir_from_sph.z) / (2.0 * PI) + 0.5;
	// clamp guards against FP roundoff after normalize pushing |y| past 1
	// (acos is undefined outside [-1, 1], can return NaN — and the bounds
	// check below doesn't catch NaN since NaN comparisons are always false).
	float lat = acos(clamp(dir_from_sph.y, -1.0, 1.0)) / PI;

	float chan = central_horizontal_angle / (2.0 * PI);
	float uhan = 0.5 + chan / 2.0;
	float lhan = 0.5 - chan / 2.0;
	float uvan = upper_vertical_angle / PI + 0.5;
	float lvan = lower_vertical_angle / PI + 0.5;
	WrapBounds bounds = make_wrap_bounds(vec2(lhan, lvan), vec2(uhan, uvan), wrap.post_transform);

	return sample_wrap_coord(src_idx, vec2(lon, lat), bounds, slot);
}

vec4 sample_equirect2(uint idx, uint src_idx, vec3 ray_dir_view, uint slot)
{
	WrapSampleData wrap = load_wrap_sample_data(idx);
	return sample_equirect2_prepared(src_idx, ray_dir_view, wrap, slot);
}

ChromaSample sample_equirect2_chroma(uint idx, uint src_idx, ChromaRays rays, uint slot)
{
	return sample_wrap_layer_chroma(idx, src_idx, rays, true, slot);
}

// The [0, 1] remap and sub-image rect are pre-folded into the per-slot
// matrices, so the warp elements already carry the final source UVs; only
// the bounds test is left per channel. Deliberately branchless — fetch,
// then select-zero: gating the fetches like apply_chroma_sample measured
// 0.3-1.7% slower, and the skipped fetches are border-clamped anyway.
// Selects rather than a multiply so a NaN texel can't leak through as
// NaN * 0. With the mask bit clear everything but the fetches folds away.
ChromaSample sample_projection_chroma(uint idx, uint src_idx, int proj_elem, uint slot)
{
	bool need_bounds = slot_needs_projection_bounds_test(slot);
	vec4 bounds = ubo.post_transform[idx];
	vec2 r_uv = warp_proj_uv(proj_elem, 0u);
	vec2 g_uv = warp_proj_uv(proj_elem, 1u);
	vec2 b_uv = warp_proj_uv(proj_elem, 2u);
	bool r_inside = !need_bounds || proj_uv_inside(r_uv, bounds);
	bool g_inside = !need_bounds || proj_uv_inside(g_uv, bounds);
	bool b_inside = !need_bounds || proj_uv_inside(b_uv, bounds);
	vec4 r = texture(sources[src_idx], r_uv);
	vec4 g = texture(sources[src_idx], g_uv);
	vec4 b = texture(sources[src_idx], b_uv);
	r = r_inside ? r : vec4(0.0);
	g = g_inside ? g : vec4(0.0);
	b = b_inside ? b : vec4(0.0);
	ChromaSample s = chroma_sample_from_rgb(r, g, b);
	// Inverted alpha on in-bounds channels only.
	if (slot_inverted_alpha(slot)) {
		s.alpha = mix(s.alpha, 1.0 - s.alpha, bvec3(r_inside, g_inside, b_inside));
	}
	return s;
}

vec4 sample_projection_mono(uint idx, uint src_idx, int proj_elem, uint slot)
{
	vec2 uv = warp_proj_uv(proj_elem, 0u);
	bool inside = !slot_needs_projection_bounds_test(slot) || proj_uv_inside(uv, ubo.post_transform[idx]);
	vec4 c = texture(sources[src_idx], uv);
	c.a = invert_alpha(c.a, slot);  // before the select: in-bounds only
	return inside ? c : vec4(0.0);
}

ChromaAccum composite_layers_chroma(uint iz, ChromaInputs inputs)
{
	vec3 background = vec3(0.0);
#if XRT_MESH_NLAYER_PASSTHROUGH
	background = sample_passthrough(iz);
#endif
	ChromaAccum accum = ChromaAccum(background);

	// Running first-element index into the warp proj block; advanced per
	// projection slot — including eye-hidden ones, whose entries exist for
	// the other eye's draw of this same pipeline.
	int proj_elem = 0;

	for (int layer = 0; layer < layer_count; ++layer) {
		uint layer_u = uint(layer);
		uint idx = layer_u * uint(VIEWS) + iz;
		uint src_idx = layer_u * uint(view_count) + iz;
		uint slot_type = (layer_types >> (layer_u * 2u)) & 3u;
		uint hidden_bits = (eye_hidden_mask >> (layer_u * 2u)) & 3u;
		bool hidden = ((hidden_bits >> iz) & 1u) != 0u;

		ChromaSample layer_sample = chroma_sample_zero();
		if (!hidden) {
			if (slot_type == LAYER_TYPE_QUAD) {
				layer_sample = sample_quad_chroma(idx, src_idx, select_chroma_rays(inputs, layer_u), layer_u);
			} else if (slot_type == LAYER_TYPE_CYLINDER) {
				layer_sample = sample_cylinder_chroma(idx, src_idx, select_chroma_rays(inputs, layer_u), layer_u);
			} else if (slot_type == LAYER_TYPE_EQUIRECT2) {
				layer_sample = sample_equirect2_chroma(idx, src_idx, select_chroma_rays(inputs, layer_u), layer_u);
			} else {
				layer_sample = sample_projection_chroma(idx, src_idx, proj_elem, layer_u);
			}
		}
		if (slot_type == LAYER_TYPE_PROJECTION) {
			proj_elem += warp_channels;
		}

		layer_sample = chroma_sample_premultiply_if_needed(layer_sample, layer_u);
		accum = chroma_over(accum, layer_sample);
	}

	return accum;
}

vec3 composite_layers_mono(uint iz, MonoInputs inputs)
{
	vec3 accum_rgb = vec3(0.0);
#if XRT_MESH_NLAYER_PASSTHROUGH
	accum_rgb = sample_passthrough(iz);
#endif

	// Same running proj-block index as composite_layers_chroma.
	int proj_elem = 0;

	for (int layer = 0; layer < layer_count; ++layer) {
		uint layer_u = uint(layer);
		uint idx = layer_u * uint(VIEWS) + iz;
		uint src_idx = layer_u * uint(view_count) + iz;
		uint slot_type = (layer_types >> (layer_u * 2u)) & 3u;
		uint hidden_bits = (eye_hidden_mask >> (layer_u * 2u)) & 3u;
		bool hidden = ((hidden_bits >> iz) & 1u) != 0u;

		vec4 layer_sample = vec4(0.0);
		if (!hidden) {
			if (slot_type == LAYER_TYPE_QUAD) {
				layer_sample = sample_quad(idx, src_idx, select_mono_ray(inputs, layer_u), layer_u);
			} else if (slot_type == LAYER_TYPE_CYLINDER) {
				layer_sample = sample_cylinder(idx, src_idx, select_mono_ray(inputs, layer_u), layer_u);
			} else if (slot_type == LAYER_TYPE_EQUIRECT2) {
				layer_sample = sample_equirect2(idx, src_idx, select_mono_ray(inputs, layer_u), layer_u);
			} else {
				layer_sample = sample_projection_mono(idx, src_idx, proj_elem, layer_u);
			}
		}
		if (slot_type == LAYER_TYPE_PROJECTION) {
			proj_elem += warp_channels;
		}

		layer_sample = mono_sample_premultiply_if_needed(layer_sample, layer_u);
		accum_rgb = mono_over(accum_rgb, layer_sample);
	}

	return accum_rgb;
}

void main()
{
	uint iz = pc.view_index;

	vec3 linear_rgb;
	// warp_channels == 3 iff (do_distortion && do_cac); branching on it
	// keeps the chroma/mono split in lockstep with the interface sizing.
	if (warp_channels == 3) {
		ChromaInputs inputs = build_chroma_inputs();
		ChromaAccum accum = composite_layers_chroma(iz, inputs);
		linear_rgb = accum.rgb;
	} else {
		MonoInputs inputs = build_mono_inputs();
		linear_rgb = composite_layers_mono(iz, inputs);
	}

	// The render target format handles the sRGB encode; the base draw's
	// alpha convention is 1.0 so the debug UI can inspect the target.
	out_color = vec4(linear_rgb, 1.0);
}
