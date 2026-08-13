// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR gaze-driven foveation map drawing.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include "util/u_logging.h"

#include "xrt/xrt_defines.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct galaxyxr_profile_eye;
struct xrt_foveation_begin_info;
struct xrt_foveation_map;

//! Compositor map buffers we keep per-buffer drawn state for.
#define GXR_FOVEATION_MAX_BUFFERS 16

/*!
 * The fovea drawn into one compositor buffer, as per-row spans [x0, x1) —
 * exactly what a later frame must diff against to only write texels that
 * changed. Rows outside [y0, y1) are empty; the span arrays point into the
 * view's span block.
 */
struct galaxyxr_foveation_drawn
{
	bool valid;
	int32_t y0, y1;
	uint16_t *x0;
	uint16_t *x1;
};

/*!
 * Per-view drawing state: the precomputed lens mapping of the map grid and
 * what was last drawn into each compositor buffer.
 */
struct galaxyxr_foveation_view
{
	//! Grid size the tables were built for.
	uint32_t width, height;

	/*!
	 * Per-texel {tan_x, tan_y, inv_len} triplets: eye-frame tangents of
	 * the texel centre through the lens (x right, y down) and
	 * 1 / |(tan_x, tan_y, 1)|, so the per-texel fovea test needs no
	 * sqrt. Interleaved so one test touches one cache line and marching
	 * a row is sequential.
	 */
	float *texels;

	//! Panel-centre tangents, the fovea position when there is no gaze.
	float center_tan_x, center_tan_y;

	//! Fovea hunt seed cell, tracks the gaze between frames.
	int32_t seed_x, seed_y;

	//! Span row block: the walked spans plus every buffer's drawn rows.
	uint16_t *spans;

	/*!
	 * The most recently walked fovea spans and the gaze they were walked
	 * for; reused across fills (and so across buffers) until the gaze
	 * moves perceptibly, letting a resting gaze skip both the walk and,
	 * once every buffer shows these spans, every write.
	 */
	uint16_t *walked_x0;
	uint16_t *walked_x1;
	int32_t walked_y0, walked_y1;
	float walked_gtx, walked_gty;
	bool walked_valid;

	struct galaxyxr_foveation_drawn drawn[GXR_FOVEATION_MAX_BUFFERS];
};

/*!
 * How much of the map each fill redraws (GALAXYXR_FOVEATION_DRAW): the
 * incremental default, or a test mode taking one more optimisation away.
 */
enum galaxyxr_foveation_draw
{
	GXR_FOVEATION_DRAW_INCREMENTAL = 0,

	//! Bypass the gaze quantization: walk the fovea and apply the
	//! changes every fill.
	GXR_FOVEATION_DRAW_ALWAYS,

	//! Rewrite the whole map every fill, the worst-case draw.
	GXR_FOVEATION_DRAW_FULL,
};

/*!
 * Draws @ref xrt_foveation_map contents: a full rate fovea disc of constant
 * view angle around the gaze — circular through the lens, warped on the
 * panel — over a reduced-rate background of 4x4 fragments.
 *
 * The per-frame draw is incremental and scales with the fovea size, not the
 * map: the fovea outline is walked scanline by scanline through the
 * precomputed lens tables and only the previous fovea is erased, relying on
 * the compositor's preserved-buffer contract.
 */
struct galaxyxr_foveation
{
	//! Both eyes' display LUTs, borrowed, must outlive this struct.
	const struct galaxyxr_profile_eye *eyes;

	//! The owning device's log level, borrowed.
	enum u_logging_level *log_level;

	//! Panel size in pixels, the space a view grid spans.
	float panel_w, panel_h;

	//! cos() of the fovea disc's angular radius.
	float cos_radius;

	//! Which walks and writes each fill may skip, see the enum.
	enum galaxyxr_foveation_draw draw_mode;

	//! Per-eye head-to-eye orientation (inverse view pose rotation).
	struct xrt_quat head_to_eye[2];

	struct galaxyxr_foveation_view views[2];

	//! Draw time of the last fill, for the debug GUI.
	float draw_last_ms;
	//! Exponential moving average of draw_last_ms.
	float draw_avg_ms;
	//! Largest draw time seen.
	float draw_max_ms;
	uint64_t draw_count;

	//! Sum, max and fill count over the current stats logging window.
	uint64_t window_ns_sum;
	uint64_t window_ns_max;
	uint64_t window_count;
};

/*!
 * Init the drawing state; the lens tables are built later, when
 * @ref galaxyxr_foveation_begin learns the map geometry. @p eyes must stay
 * alive and loaded. @p radius_deg is the fovea disc's angular radius,
 * @p view_poses the calibrated view poses or NULL.
 */
bool
galaxyxr_foveation_init(struct galaxyxr_foveation *f,
                        const struct galaxyxr_profile_eye *eyes,
                        uint32_t panel_w,
                        uint32_t panel_h,
                        float radius_deg,
                        const struct xrt_pose *view_poses,
                        enum u_logging_level *log_level);

void
galaxyxr_foveation_fini(struct galaxyxr_foveation *f);

/*!
 * Foveated compositing is starting with this fixed map geometry: build the
 * lens tables for it now, off the per-frame path.
 */
void
galaxyxr_foveation_begin(struct galaxyxr_foveation *f, const struct xrt_foveation_begin_info *info);

/*!
 * Fill the compositor's foveation map for this frame; the fovea follows
 * @p gaze_dir (head frame, x right, y up, -z forward) or sits at the panel
 * centre when NULL.
 */
xrt_result_t
galaxyxr_foveation_fill_map(struct galaxyxr_foveation *f,
                            const struct xrt_vec3 *gaze_dir,
                            struct xrt_foveation_map *map);

#ifdef __cplusplus
}
#endif
