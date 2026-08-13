// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR gaze-driven foveation map drawing.
 *
 * Fills @ref xrt_foveation_map grids with a two-level pattern: a full rate
 * disc of constant view angle around the gaze, reduced shading everywhere
 * else. The disc is defined through the lens — the per-texel test compares
 * view-space angles from precomputed per-texel tangents of the display LUT
 * — so it is circular as perceived and warps with the lens distortion on
 * the panel.
 *
 * The draw never touches every texel: the previous fovea is erased from its
 * remembered bounds (the compositor preserves buffer contents between fills
 * of the same buffer), the new fovea's outline is found by a coarse-to-fine
 * hunt for the gaze cell followed by an edge-marching scanline walk, and
 * every touched row is a memset. Per frame that is a few thousand table
 * lookups and roughly fovea-sized writes, tens of microseconds against a
 * 400k texel map.
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_foveation.h"
#include "galaxyxr_profile.h"

#include "os/os_time.h"

#include "math/m_api.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"

#include "xrt/xrt_device.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>


#define GXR_FOVEATION_STATS_WINDOW 4096

// Test modes: "always" bypasses the gaze quantization and walks the fovea
// every fill, "full" also rewrites the whole map every fill, the worst-case
// draw.
DEBUG_GET_ONCE_OPTION(galaxyxr_foveation_draw, "GALAXYXR_FOVEATION_DRAW", "incremental")


/*
 *
 * Lens tables.
 *
 */

//! Get (or rebuild on geometry change) the per-texel view-space table.
static bool
view_tables_ensure(struct galaxyxr_foveation *f, uint32_t view, uint32_t width, uint32_t height)
{
	struct galaxyxr_foveation_view *v = &f->views[view];
	if (v->texels != NULL && v->width == width && v->height == height) {
		return true;
	}

	int64_t start_ns = os_monotonic_get_ns();

	free(v->texels);
	free(v->spans);
	const size_t cells = (size_t)width * height;
	float *block = U_TYPED_ARRAY_CALLOC(float, cells * 3);
	uint16_t *spans = U_TYPED_ARRAY_CALLOC(uint16_t, (size_t)height * 2 * (GXR_FOVEATION_MAX_BUFFERS + 1));
	if (block == NULL || spans == NULL) {
		free(block);
		free(spans);
		U_ZERO(v);
		return false;
	}
	v->texels = block;
	v->spans = spans;
	v->walked_x0 = spans;
	v->walked_x1 = spans + height;

	const struct galaxyxr_profile_eye *eye = &f->eyes[view];
	for (uint32_t y = 0; y < height; y++) {
		const float py = ((y + 0.5f) / (float)height - 0.5f) * f->panel_h;
		for (uint32_t x = 0; x < width; x++) {
			const float px = ((x + 0.5f) / (float)width - 0.5f) * f->panel_w;
			float tx = 0.0f;
			float ty = 0.0f;
			galaxyxr_profile_panel_to_tan(eye, 1, px, py, &tx, &ty);

			float *t = v->texels + ((size_t)y * width + x) * 3;
			t[0] = tx;
			t[1] = ty;
			t[2] = 1.0f / sqrtf(1.0f + tx * tx + ty * ty);
		}
	}

	galaxyxr_profile_panel_to_tan(eye, 1, 0.0f, 0.0f, &v->center_tan_x, &v->center_tan_y);

	v->width = width;
	v->height = height;
	v->seed_x = (int32_t)width / 2;
	v->seed_y = (int32_t)height / 2;
	v->walked_y0 = 0;
	v->walked_y1 = 0;
	v->walked_valid = false;
	memset(v->drawn, 0, sizeof(v->drawn));
	for (uint32_t i = 0; i < GXR_FOVEATION_MAX_BUFFERS; i++) {
		v->drawn[i].x0 = spans + (size_t)height * 2 * (i + 1);
		v->drawn[i].x1 = v->drawn[i].x0 + height;
	}

	U_LOG_IFL_I(*f->log_level, "gxr foveation: view %u lens table %ux%u built in %.1f ms", view, width, height,
	            (double)(os_monotonic_get_ns() - start_ns) / (double)U_TIME_1MS_IN_NS);

	return true;
}


/*
 *
 * Fovea geometry.
 *
 */

/*!
 * Per-view fovea test constants: texel i is inside the fovea iff
 * cos(angle(texel dir, gaze dir)) > cos_radius, with the gaze length folded
 * into the threshold so the per-texel test is one fma-chain and a compare.
 */
struct fovea_ctx
{
	float gtx;
	float gty;
	float cos_thresh;
};

//! Monotone in the cosine of the texel-to-gaze angle.
static inline float
fovea_metric(const struct galaxyxr_foveation_view *v, const struct fovea_ctx *ctx, size_t i)
{
	const float *t = v->texels + i * 3;
	return (1.0f + t[0] * ctx->gtx + t[1] * ctx->gty) * t[2];
}

static inline bool
fovea_inside(const struct galaxyxr_foveation_view *v, const struct fovea_ctx *ctx, size_t i)
{
	return fovea_metric(v, ctx, i) > ctx->cos_thresh;
}

/*!
 * Coarse-to-fine hunt for the grid cell closest in angle to the gaze,
 * starting from the previous frame's cell: axis descent at shrinking step
 * sizes on the smooth unimodal angle field. Returns whether that cell is
 * inside the fovea — if the closest cell is not, no cell is.
 */
static bool
hunt_seed(const struct galaxyxr_foveation_view *v, const struct fovea_ctx *ctx, int32_t *io_x, int32_t *io_y)
{
	const int32_t w = (int32_t)v->width;
	const int32_t h = (int32_t)v->height;

	int32_t x = *io_x < 0 ? 0 : (*io_x >= w ? w - 1 : *io_x);
	int32_t y = *io_y < 0 ? 0 : (*io_y >= h ? h - 1 : *io_y);
	float best = fovea_metric(v, ctx, (size_t)y * w + x);

	for (int32_t step = 64; step > 0; step >>= 2) {
		for (;;) {
			int32_t bx = x;
			int32_t by = y;

			const int32_t cand[4][2] = {
			    {x - step < 0 ? 0 : x - step, y},
			    {x + step >= w ? w - 1 : x + step, y},
			    {x, y - step < 0 ? 0 : y - step},
			    {x, y + step >= h ? h - 1 : y + step},
			};
			for (uint32_t c = 0; c < 4; c++) {
				const float m = fovea_metric(v, ctx, (size_t)cand[c][1] * w + cand[c][0]);
				if (m > best) {
					best = m;
					bx = cand[c][0];
					by = cand[c][1];
				}
			}

			if (bx == x && by == y) {
				break;
			}
			x = bx;
			y = by;
		}
	}

	*io_x = x;
	*io_y = y;
	return best > ctx->cos_thresh;
}

/*!
 * Find row @p y's fovea span by marching its edges from the previous row's
 * span [@p a, @p b): edges of the smooth convex outline move only a little
 * row to row, so each march is a handful of table lookups. Returns false
 * once the row is empty (the outline's end).
 */
static bool
march_row(const struct galaxyxr_foveation_view *v,
          const struct fovea_ctx *ctx,
          int32_t y,
          int32_t a,
          int32_t b,
          int32_t *out_x0,
          int32_t *out_x1)
{
	const int32_t w = (int32_t)v->width;
	const size_t row = (size_t)y * v->width;

	int32_t x0 = a;
	if (fovea_inside(v, ctx, row + x0)) {
		while (x0 > 0 && fovea_inside(v, ctx, row + x0 - 1)) {
			x0--;
		}
	} else {
		while (x0 < b && !fovea_inside(v, ctx, row + x0)) {
			x0++;
		}
		if (x0 >= b) {
			return false;
		}
	}

	int32_t x1 = b > x0 ? b : x0 + 1;
	if (x1 < w && fovea_inside(v, ctx, row + x1)) {
		x1++;
		while (x1 < w && fovea_inside(v, ctx, row + x1)) {
			x1++;
		}
	} else {
		while (x1 > x0 + 1 && !fovea_inside(v, ctx, row + x1 - 1)) {
			x1--;
		}
	}

	*out_x0 = x0;
	*out_x1 = x1;
	return true;
}


/*
 *
 * Drawing.
 *
 */

static inline void
fill_row(uint8_t *data, uint32_t stride, uint32_t texel_bytes, int32_t y, int32_t x0, int32_t x1, uint8_t value)
{
	if (x0 < x1) {
		memset(data + (size_t)y * stride + (size_t)x0 * texel_bytes, value, (size_t)(x1 - x0) * texel_bytes);
	}
}

/*!
 * Make the view's walked spans hold the fovea for this gaze: hunt the gaze
 * cell, then march the outline's edges row by row from the seed row's span.
 *
 * A live gaze never rests — drift and tracker noise move it every frame —
 * so the fovea position is quantized: a gaze within ~0.15 degrees of the
 * previous walk reuses it outright (unless @p force), letting fixation
 * frames skip the walk and, downstream, every staging write. The step is
 * invisible against the fovea's size and the fovea-to-background transition
 * granularity.
 */
static void
walk_fovea(struct galaxyxr_foveation_view *v, const struct fovea_ctx *ctx, bool force)
{
	const int32_t w = (int32_t)v->width;
	const int32_t h = (int32_t)v->height;

	const float eps = 0.0026f; // tan(0.15 deg).
	if (!force && v->walked_valid &&              //
	    fabsf(ctx->gtx - v->walked_gtx) < eps &&  //
	    fabsf(ctx->gty - v->walked_gty) < eps) {  //
		return;
	}
	v->walked_gtx = ctx->gtx;
	v->walked_gty = ctx->gty;
	v->walked_valid = true;
	v->walked_y0 = 0;
	v->walked_y1 = 0;

	int32_t sx = v->seed_x;
	int32_t sy = v->seed_y;
	const bool found = hunt_seed(v, ctx, &sx, &sy);
	v->seed_x = sx;
	v->seed_y = sy;
	if (!found) {
		return;
	}

	const size_t seed_row = (size_t)sy * v->width;
	int32_t x0 = sx;
	while (x0 > 0 && fovea_inside(v, ctx, seed_row + x0 - 1)) {
		x0--;
	}
	int32_t x1 = sx + 1;
	while (x1 < w && fovea_inside(v, ctx, seed_row + x1)) {
		x1++;
	}
	v->walked_x0[sy] = (uint16_t)x0;
	v->walked_x1[sy] = (uint16_t)x1;

	int32_t y0 = sy;
	int32_t y1 = sy + 1;
	int32_t a = x0;
	int32_t b = x1;
	for (int32_t y = sy - 1; y >= 0 && march_row(v, ctx, y, a, b, &a, &b); y--) {
		v->walked_x0[y] = (uint16_t)a;
		v->walked_x1[y] = (uint16_t)b;
		y0 = y;
		if (y >= 4) {
			// Edges move little row to row: pipeline the marches'
			// serial cache misses by prefetching them rows ahead.
			const size_t pr = (size_t)(y - 4) * v->width;
			__builtin_prefetch(v->texels + (pr + a) * 3, 0, 0);
			__builtin_prefetch(v->texels + (pr + b) * 3, 0, 0);
		}
	}
	a = x0;
	b = x1;
	for (int32_t y = sy + 1; y < h && march_row(v, ctx, y, a, b, &a, &b); y++) {
		v->walked_x0[y] = (uint16_t)a;
		v->walked_x1[y] = (uint16_t)b;
		y1 = y + 1;
		if (y + 4 < h) {
			const size_t pr = (size_t)(y + 4) * v->width;
			__builtin_prefetch(v->texels + (pr + a) * 3, 0, 0);
			__builtin_prefetch(v->texels + (pr + b) * 3, 0, 0);
		}
	}

	v->walked_y0 = y0;
	v->walked_y1 = y1;
}

static void
draw_view(struct galaxyxr_foveation *f,
          uint32_t view,
          const struct fovea_ctx *ctx,
          uint8_t *data,
          uint32_t stride,
          uint32_t texel_bytes,
          uint8_t fovea_value,
          uint8_t bg_value,
          uint32_t buffer_id,
          bool preserved)
{
	struct galaxyxr_foveation_view *v = &f->views[view];
	const int32_t w = (int32_t)v->width;
	const int32_t h = (int32_t)v->height;

	walk_fovea(v, ctx, f->draw_mode != GXR_FOVEATION_DRAW_INCREMENTAL);
	const int32_t ny0 = v->walked_y0;
	const int32_t ny1 = v->walked_y1;

	struct galaxyxr_foveation_drawn none = {0};
	struct galaxyxr_foveation_drawn *drawn = buffer_id < GXR_FOVEATION_MAX_BUFFERS ? &v->drawn[buffer_id] : &none;

	if (!preserved || !drawn->valid || f->draw_mode == GXR_FOVEATION_DRAW_FULL) {
		// Unknown buffer contents: background plus the whole fovea.
		for (int32_t y = 0; y < ny0; y++) {
			fill_row(data, stride, texel_bytes, y, 0, w, bg_value);
		}
		for (int32_t y = ny0; y < ny1; y++) {
			const int32_t x0 = v->walked_x0[y];
			const int32_t x1 = v->walked_x1[y];
			fill_row(data, stride, texel_bytes, y, 0, x0, bg_value);
			fill_row(data, stride, texel_bytes, y, x0, x1, fovea_value);
			fill_row(data, stride, texel_bytes, y, x1, w, bg_value);
		}
		for (int32_t y = ny1; y < h; y++) {
			fill_row(data, stride, texel_bytes, y, 0, w, bg_value);
		}
	} else {
		/*
		 * The buffer holds the fovea recorded in drawn; rewrite only
		 * rows whose span changed — none at all while the gaze rests.
		 * A changed row with overlapping spans is composed in cached
		 * scratch and pushed as one sequential write: the staging
		 * memory is write-combined and pays per transaction, not per
		 * byte.
		 */
		uint8_t row_buf[1024];
		const int32_t u0 = drawn->y0 < ny0 ? drawn->y0 : ny0;
		const int32_t u1 = drawn->y1 > ny1 ? drawn->y1 : ny1;
		for (int32_t y = u0; y < u1; y++) {
			const int32_t ox0 = y >= drawn->y0 && y < drawn->y1 ? drawn->x0[y] : 0;
			const int32_t ox1 = y >= drawn->y0 && y < drawn->y1 ? drawn->x1[y] : 0;
			const int32_t nx0 = y >= ny0 && y < ny1 ? v->walked_x0[y] : 0;
			const int32_t nx1 = y >= ny0 && y < ny1 ? v->walked_x1[y] : 0;
			if (ox0 == nx0 && ox1 == nx1) {
				continue;
			}

			const bool have_old = ox0 < ox1;
			const bool have_new = nx0 < nx1;
			if (!have_new) {
				if (have_old) {
					fill_row(data, stride, texel_bytes, y, ox0, ox1, bg_value);
				}
				continue;
			}
			if (!have_old) {
				fill_row(data, stride, texel_bytes, y, nx0, nx1, fovea_value);
				continue;
			}
			if (nx0 >= ox1 || nx1 <= ox0) {
				// Disjoint spans, e.g. a saccade: two writes
				// skip the untouched gap between them.
				fill_row(data, stride, texel_bytes, y, ox0, ox1, bg_value);
				fill_row(data, stride, texel_bytes, y, nx0, nx1, fovea_value);
				continue;
			}

			const int32_t c0 = ox0 < nx0 ? ox0 : nx0;
			const int32_t c1 = ox1 > nx1 ? ox1 : nx1;
			if ((size_t)(c1 - c0) * texel_bytes > sizeof(row_buf)) {
				fill_row(data, stride, texel_bytes, y, c0, c1, bg_value);
				fill_row(data, stride, texel_bytes, y, nx0, nx1, fovea_value);
				continue;
			}

			uint8_t *buf = (uint8_t *)row_buf;
			memset(buf, bg_value, (size_t)(c1 - c0) * texel_bytes);
			memset(buf + (size_t)(nx0 - c0) * texel_bytes, fovea_value, (size_t)(nx1 - nx0) * texel_bytes);
			memcpy(data + (size_t)y * stride + (size_t)c0 * texel_bytes, buf,
			       (size_t)(c1 - c0) * texel_bytes);
		}
	}

	drawn->valid = true;
	drawn->y0 = ny0;
	drawn->y1 = ny1;
	if (drawn != &none && ny0 < ny1) {
		memcpy(drawn->x0 + ny0, v->walked_x0 + ny0, (size_t)(ny1 - ny0) * sizeof(uint16_t));
		memcpy(drawn->x1 + ny0, v->walked_x1 + ny0, (size_t)(ny1 - ny0) * sizeof(uint16_t));
	}
}


/*
 *
 * 'Exported' functions.
 *
 */

bool
galaxyxr_foveation_init(struct galaxyxr_foveation *f,
                        const struct galaxyxr_profile_eye *eyes,
                        uint32_t panel_w,
                        uint32_t panel_h,
                        float radius_deg,
                        const struct xrt_pose *view_poses,
                        enum u_logging_level *log_level)
{
	U_ZERO(f);

	if (eyes == NULL || !eyes[0].valid || !eyes[1].valid || radius_deg <= 0.0f) {
		return false;
	}

	f->eyes = eyes;
	f->log_level = log_level;
	f->panel_w = (float)panel_w;
	f->panel_h = (float)panel_h;
	f->cos_radius = cosf(radius_deg * ((float)M_PI / 180.0f));
	const char *draw = debug_get_option_galaxyxr_foveation_draw();
	if (strcmp(draw, "full") == 0) {
		f->draw_mode = GXR_FOVEATION_DRAW_FULL;
		U_LOG_IFL_I(*log_level, "gxr foveation: full worst-case draw every fill");
	} else if (strcmp(draw, "always") == 0) {
		f->draw_mode = GXR_FOVEATION_DRAW_ALWAYS;
		U_LOG_IFL_I(*log_level, "gxr foveation: gaze quantization off, walking the fovea every fill");
	} else {
		f->draw_mode = GXR_FOVEATION_DRAW_INCREMENTAL;
	}
	f->head_to_eye[0] = (struct xrt_quat)XRT_QUAT_IDENTITY;
	f->head_to_eye[1] = (struct xrt_quat)XRT_QUAT_IDENTITY;
	if (view_poses != NULL) {
		math_quat_invert(&view_poses[0].orientation, &f->head_to_eye[0]);
		math_quat_invert(&view_poses[1].orientation, &f->head_to_eye[1]);
	}

	return true;
}

void
galaxyxr_foveation_begin(struct galaxyxr_foveation *f, const struct xrt_foveation_begin_info *info)
{
	const uint32_t view_count = info->view_count < 2 ? info->view_count : 2;
	for (uint32_t view = 0; view < view_count; view++) {
		if (info->views[view].width == 0 || info->views[view].height == 0) {
			continue;
		}
		if (!view_tables_ensure(f, view, info->views[view].width, info->views[view].height)) {
			// Fills retry the build; failing here need not be fatal.
			return;
		}
	}
}

void
galaxyxr_foveation_fini(struct galaxyxr_foveation *f)
{
	for (uint32_t view = 0; view < 2; view++) {
		free(f->views[view].texels);
		free(f->views[view].spans);
	}
	U_ZERO(f);
}

xrt_result_t
galaxyxr_foveation_fill_map(struct galaxyxr_foveation *f,
                            const struct xrt_vec3 *gaze_dir,
                            struct xrt_foveation_map *map)
{
	uint8_t fovea_value;
	uint8_t bg_value;
	uint32_t texel_bytes;

	switch (map->mechanism) {
	case XRT_FOVEATION_MECHANISM_VK_FSR:
		fovea_value = map->fsr_rates[0][0];
		bg_value = map->fsr_rates[2][2];
		texel_bytes = 1;
		break;
	default: return XRT_ERROR_NOT_IMPLEMENTED;
	}

	int64_t start_ns = os_monotonic_get_ns();

	const uint32_t view_count = map->view_count < 2 ? map->view_count : 2;
	for (uint32_t view = 0; view < view_count; view++) {
		if (map->views[view].data == NULL || map->views[view].width == 0 || map->views[view].height == 0) {
			continue;
		}

		if (!view_tables_ensure(f, view, map->views[view].width, map->views[view].height)) {
			return XRT_ERROR_ALLOCATION;
		}
		struct galaxyxr_foveation_view *v = &f->views[view];

		// The fovea centre: the gaze through this eye's frame, or the
		// panel centre without one.
		struct fovea_ctx ctx;
		ctx.gtx = v->center_tan_x;
		ctx.gty = v->center_tan_y;
		if (gaze_dir != NULL) {
			struct xrt_vec3 e;
			math_quat_rotate_vec3(&f->head_to_eye[view], gaze_dir, &e);
			if (e.z < -1e-3f) {
				// OpenXR view axes to lens tangents: x right, y down.
				ctx.gtx = -e.x / e.z;
				ctx.gty = e.y / e.z;
			}
		}
		ctx.cos_thresh = f->cos_radius * sqrtf(1.0f + ctx.gtx * ctx.gtx + ctx.gty * ctx.gty);

		draw_view(f, view,                 //
		          &ctx,                    //
		          map->views[view].data,   //
		          map->views[view].stride, //
		          texel_bytes,             //
		          fovea_value, bg_value,   //
		          map->buffer_id, map->preserved);
	}

	// Draw time stats: u_var mirrors plus a rare summary log line.
	const uint64_t dur_ns = (uint64_t)(os_monotonic_get_ns() - start_ns);
	f->draw_last_ms = (float)((double)dur_ns / (double)U_TIME_1MS_IN_NS);
	f->draw_avg_ms = f->draw_avg_ms == 0.0f ? f->draw_last_ms
	                                        : f->draw_avg_ms + 0.05f * (f->draw_last_ms - f->draw_avg_ms);
	f->draw_max_ms = f->draw_last_ms > f->draw_max_ms ? f->draw_last_ms : f->draw_max_ms;
	f->window_ns_sum += dur_ns;
	f->window_ns_max = dur_ns > f->window_ns_max ? dur_ns : f->window_ns_max;
	f->window_count++;
	f->draw_count++;
	if (f->draw_count % GXR_FOVEATION_STATS_WINDOW == 0 || f->draw_count == 128) {
		U_LOG_IFL_I(*f->log_level, "gxr foveation: draw avg %.3f ms, max %.3f ms over last %u frames",
		            (double)f->window_ns_sum / (double)f->window_count / (double)U_TIME_1MS_IN_NS,
		            (double)f->window_ns_max / (double)U_TIME_1MS_IN_NS, (uint32_t)f->window_count);
		f->window_ns_sum = 0;
		f->window_ns_max = 0;
		f->window_count = 0;
	}

	return XRT_SUCCESS;
}
