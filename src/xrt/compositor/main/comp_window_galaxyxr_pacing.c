// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pacer for the Samsung Galaxy XR dual DSI direct mode.
 *
 * The generic pacers assume feedback this platform does not have (display
 * timing extensions) or need to be bent through environment knobs to fit the
 * one-deep commit pipeline of the galaxyxr target, so this pacer owns the
 * whole model:
 *
 * - The desired present time is the vsync at which the commit latches,
 *   predicted on the lattice of hardware flip timestamps extrapolated with
 *   the exact mode period. The backend verified desired == latch to ~20 us.
 * - The display time is the start of panel scanout: the flip timestamp is
 *   the MDP vsync, which leads the panel vsync by VFP-1 lines of
 *   programmable fetch, plus VSW+VBP lines to the first active line. On top
 *   of that sits the panel's write-to-emission lag, unmeasured so far, as a
 *   tunable that defaults to zero.
 * - The wake lead (comp time) adapts so the render fence signals a target
 *   slack before the kernel needs it: the commit worker must write CTL_FLUSH
 *   at the latest one millisecond (the kernel's forbidden window) plus plane
 *   programming time before the latch vsync.
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup comp_main
 */

#include <assert.h>
#include <stdio.h>

#include "util/u_var.h"
#include "util/u_time.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_pacing.h"
#include "util/u_metrics.h"
#include "util/u_logging.h"

#include "comp_window_galaxyxr_pacing.h"


//! Assumed panel write-to-emission lag on top of the scanout start.
DEBUG_GET_ONCE_FLOAT_OPTION(gxr_emission_lag, "XRT_COMPOSITOR_GALAXYXR_EMISSION_LAG_MS", 0.0f)
//! How long before the latch vsync the render fence must signal for the kernel worker.
DEBUG_GET_ONCE_FLOAT_OPTION(gxr_latch_margin, "XRT_COMPOSITOR_GALAXYXR_LATCH_MARGIN_MS", 2.0f)
//! Initial wake lead before the desired present time.
DEBUG_GET_ONCE_FLOAT_OPTION(gxr_comp_time, "XRT_COMPOSITOR_GALAXYXR_COMP_TIME_MS", 8.0f)
DEBUG_GET_ONCE_LOG_OPTION(gxr_pacing_log, "XRT_COMPOSITOR_GALAXYXR_PACING_LOG", U_LOGGING_INFO)

#define GXR_PACER_LOG_D(...) U_LOG_IFL_D(debug_get_log_option_gxr_pacing_log(), __VA_ARGS__)
#define GXR_PACER_LOG_I(...) U_LOG_IFL_I(debug_get_log_option_gxr_pacing_log(), __VA_ARGS__)

#define GXR_PACER_FRAME_COUNT 16

// Target slack between the fence signal and the kernel's fence deadline.
#define GXR_PACER_TARGET_SLACK_MS 1.5
// Extra slack above the target before the wake lead shrinks (dead band).
#define GXR_PACER_DEAD_BAND_MS 1.5
// Percentage of the period to grow the wake lead by on a miss.
#define GXR_PACER_BUMP_PERCENT 2.0
// Percentage of the period to shrink the wake lead by on excess slack.
#define GXR_PACER_NUDGE_PERCENT 1.0
// Wake lead bounds. The maximum deliberately exceeds one period: when the
// GPU pipeline is longer than a frame the honest wake lead is too, and the
// predictions then land on a correspondingly later vsync instead of
// promising display times the pipeline cannot hit.
#define GXR_PACER_MIN_COMP_MS 4.0
#define GXR_PACER_MAX_COMP_PERCENT 250.0

struct gxr_pacer_frame
{
	int64_t frame_id;
	int64_t desired_present_ns;
	int64_t display_ns;
};

struct gxr_pacer
{
	struct u_pacing_compositor base;

	//! Exact frame period from the mode pixel clock.
	int64_t period_ns;
	//! Latch (MDP) vsync to the first active line on the panel.
	int64_t scanout_begin_ns;
	//! Panel write-to-emission lag, live tunable.
	struct u_var_draggable_f32 emission_lag_ms;

	//! Latest hardware flip timestamp, the lattice anchor.
	int64_t vblank_base_ns;
	//! Newest desired present time handed out, keeps predictions monotonic.
	int64_t last_desired_ns;

	//! Wake lead before the desired present time, adaptive.
	int64_t comp_time_ns;

	int64_t frame_id_gen;
	struct gxr_pacer_frame frames[GXR_PACER_FRAME_COUNT];
};


static inline struct gxr_pacer *
gxr_pacer(struct u_pacing_compositor *upc)
{
	return (struct gxr_pacer *)upc;
}

static int64_t
get_percent_of_period(struct gxr_pacer *p, double percent)
{
	return (int64_t)((double)p->period_ns * percent / 100.0);
}

static struct gxr_pacer_frame *
get_frame_or_null(struct gxr_pacer *p, int64_t frame_id)
{
	struct gxr_pacer_frame *f = &p->frames[(uint64_t)frame_id % GXR_PACER_FRAME_COUNT];
	return f->frame_id == frame_id ? f : NULL;
}


/*
 *
 * Member functions.
 *
 */

static void
pc_predict(struct u_pacing_compositor *upc,
           int64_t now_ns,
           int64_t *out_frame_id,
           int64_t *out_wake_up_time_ns,
           int64_t *out_desired_present_time_ns,
           int64_t *out_present_slop_ns,
           int64_t *out_predicted_display_time_ns,
           int64_t *out_predicted_display_period_ns,
           int64_t *out_min_display_period_ns)
{
	struct gxr_pacer *p = gxr_pacer(upc);

	int64_t frame_id = ++p->frame_id_gen;
	int64_t min_ns = now_ns + p->comp_time_ns;
	int64_t desired_ns = 0;

	if (p->vblank_base_ns != 0) {
		// First lattice point after min_ns; the anchor is at most a
		// few periods stale and the period is exact, so the phase
		// error stays microscopic.
		int64_t k = (min_ns - p->vblank_base_ns) / p->period_ns + 1;
		if (k < 1) {
			k = 1;
		}
		desired_ns = p->vblank_base_ns + k * p->period_ns;
	} else {
		// No flip seen yet (or the panels are off), free-run at the
		// period until a commit latches.
		desired_ns = min_ns + p->period_ns;
	}

	// Never hand out the same vsync twice.
	if (desired_ns <= p->last_desired_ns) {
		desired_ns = p->last_desired_ns + p->period_ns;
	}
	p->last_desired_ns = desired_ns;

	int64_t emission_lag_ns = time_ms_f_to_ns(p->emission_lag_ms.val);
	int64_t display_ns = desired_ns + p->scanout_begin_ns + emission_lag_ns;
	int64_t wake_up_time_ns = desired_ns - p->comp_time_ns;

	struct gxr_pacer_frame *f = &p->frames[(uint64_t)frame_id % GXR_PACER_FRAME_COUNT];
	f->frame_id = frame_id;
	f->desired_present_ns = desired_ns;
	f->display_ns = display_ns;

	*out_frame_id = frame_id;
	*out_wake_up_time_ns = wake_up_time_ns;
	*out_desired_present_time_ns = desired_ns;
	*out_present_slop_ns = U_TIME_HALF_MS_IN_NS;
	*out_predicted_display_time_ns = display_ns;
	*out_predicted_display_period_ns = p->period_ns;
	*out_min_display_period_ns = p->period_ns;

	if (!u_metrics_is_active()) {
		return;
	}

	struct u_metrics_system_frame umsf = {
	    .frame_id = frame_id,
	    .predicted_display_time_ns = display_ns,
	    .predicted_display_period_ns = p->period_ns,
	    .desired_present_time_ns = desired_ns,
	    .wake_up_time_ns = wake_up_time_ns,
	    .present_slop_ns = U_TIME_HALF_MS_IN_NS,
	};

	u_metrics_write_system_frame(&umsf);
}

static void
pc_mark_point(struct u_pacing_compositor *upc, enum u_timing_point point, int64_t frame_id, int64_t when_ns)
{
	switch (point) {
	case U_TIMING_POINT_WAKE_UP:
	case U_TIMING_POINT_BEGIN:
	case U_TIMING_POINT_SUBMIT_BEGIN:
	case U_TIMING_POINT_SUBMIT_END: break;
	default: assert(false);
	}
}

static void
pc_info(struct u_pacing_compositor *upc,
        int64_t frame_id,
        int64_t desired_present_time_ns,
        int64_t actual_present_time_ns,
        int64_t earliest_present_time_ns,
        int64_t present_margin_ns,
        int64_t when_ns)
{
	// Latch feedback flows through info_gpu; the wake lead adapting past
	// one period is what keeps predictions truthful under overload.
}

static void
adapt_comp_time(struct gxr_pacer *p, struct gxr_pacer_frame *f, int64_t gpu_end_ns)
{
	int64_t latch_margin_ns = time_ms_f_to_ns(debug_get_float_option_gxr_latch_margin());
	int64_t deadline_ns = f->desired_present_ns - latch_margin_ns;
	int64_t slack_ns = deadline_ns - gpu_end_ns;

	int64_t target_ns = time_ms_f_to_ns(GXR_PACER_TARGET_SLACK_MS);
	int64_t dead_band_ns = time_ms_f_to_ns(GXR_PACER_DEAD_BAND_MS);
	int64_t max_comp_ns = get_percent_of_period(p, GXR_PACER_MAX_COMP_PERCENT);
	int64_t min_comp_ns = time_ms_f_to_ns(GXR_PACER_MIN_COMP_MS);

	int64_t before_ns = p->comp_time_ns;

	if (slack_ns < target_ns) {
		p->comp_time_ns += get_percent_of_period(p, GXR_PACER_BUMP_PERCENT);
	} else if (slack_ns > target_ns + dead_band_ns) {
		p->comp_time_ns -= get_percent_of_period(p, GXR_PACER_NUDGE_PERCENT);
	}

	if (p->comp_time_ns > max_comp_ns) {
		p->comp_time_ns = max_comp_ns;
	}
	if (p->comp_time_ns < min_comp_ns) {
		p->comp_time_ns = min_comp_ns;
	}

	if (p->comp_time_ns != before_ns) {
		GXR_PACER_LOG_D("Adapted comp time %.2f -> %.2f ms (fence slack %.2f ms)", time_ns_to_ms_f(before_ns),
		                time_ns_to_ms_f(p->comp_time_ns), time_ns_to_ms_f(slack_ns));
	}
}

static void
pc_info_gpu(
    struct u_pacing_compositor *upc, int64_t frame_id, int64_t gpu_start_ns, int64_t gpu_end_ns, int64_t when_ns)
{
	struct gxr_pacer *p = gxr_pacer(upc);

	if (u_metrics_is_active()) {
		struct u_metrics_system_gpu_info umgi = {
		    .frame_id = frame_id,
		    .gpu_start_ns = gpu_start_ns,
		    .gpu_end_ns = gpu_end_ns,
		    .when_ns = when_ns,
		};

		u_metrics_write_system_gpu_info(&umgi);
	}

	// Adapt only on CLOCK_MONOTONIC times from the target's fence
	// feedback; Turnip on kgsl has no calibrated timestamps, so query
	// results from the renderer may be on the GPU clock.
	if (gpu_end_ns > when_ns || when_ns - gpu_end_ns > U_TIME_1S_IN_NS / 2) {
		return;
	}

	struct gxr_pacer_frame *f = get_frame_or_null(p, frame_id);
	if (f != NULL) {
		adapt_comp_time(p, f, gpu_end_ns);
	}
}

static void
pc_update_vblank_from_display_control(struct u_pacing_compositor *upc, int64_t last_vblank_ns)
{
	struct gxr_pacer *p = gxr_pacer(upc);

	if (last_vblank_ns > p->vblank_base_ns) {
		p->vblank_base_ns = last_vblank_ns;
	}
}

static void
pc_update_present_offset(struct u_pacing_compositor *upc, int64_t frame_id, int64_t present_to_display_offset_ns)
{
	// The offset is derived from the mode timing at creation; nothing
	// should push one here.
}

static void
pc_destroy(struct u_pacing_compositor *upc)
{
	struct gxr_pacer *p = gxr_pacer(upc);

	u_var_remove_root(p);

	free(p);
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
comp_window_galaxyxr_pacer_create(const drmModeModeInfo *mode, struct u_pacing_compositor **out_upc)
{
	struct gxr_pacer *p = U_TYPED_CALLOC(struct gxr_pacer);
	if (p == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	p->base.predict = pc_predict;
	p->base.mark_point = pc_mark_point;
	p->base.info = pc_info;
	p->base.info_gpu = pc_info_gpu;
	p->base.update_vblank_from_display_control = pc_update_vblank_from_display_control;
	p->base.update_present_offset = pc_update_present_offset;
	p->base.destroy = pc_destroy;

	if (mode->clock != 0 && mode->htotal != 0 && mode->vtotal != 0) {
		p->period_ns = ((int64_t)mode->htotal * mode->vtotal * 1000000) / mode->clock;
	} else {
		p->period_ns = U_TIME_1S_IN_NS / (mode->vrefresh != 0 ? mode->vrefresh : 90);
	}

	// The flip timestamp is the MDP vsync, which leads the panel vsync
	// by VFP-1 lines of programmable fetch, and VSW+VBP more lines pass
	// before the first active line goes out.
	int64_t line_ns = mode->clock != 0 ? ((int64_t)mode->htotal * 1000000) / mode->clock
	                                   : p->period_ns / (mode->vtotal != 0 ? mode->vtotal : 1);
	int32_t lead_lines = (mode->vsync_start - mode->vdisplay - 1) + (mode->vtotal - mode->vsync_start);
	p->scanout_begin_ns = lead_lines > 0 ? lead_lines * line_ns : 0;

	p->emission_lag_ms = (struct u_var_draggable_f32){
	    .val = debug_get_float_option_gxr_emission_lag(),
	    .min = 0.0f,
	    .step = 0.05f,
	    .max = 20.0f,
	};

	int64_t max_comp_ns = get_percent_of_period(p, GXR_PACER_MAX_COMP_PERCENT);
	int64_t min_comp_ns = time_ms_f_to_ns(GXR_PACER_MIN_COMP_MS);
	p->comp_time_ns = time_ms_f_to_ns(debug_get_float_option_gxr_comp_time());
	if (p->comp_time_ns > max_comp_ns) {
		p->comp_time_ns = max_comp_ns;
	}
	if (p->comp_time_ns < min_comp_ns) {
		p->comp_time_ns = min_comp_ns;
	}

	u_var_add_root(p, "Galaxy XR pacing", true);
	u_var_add_draggable_f32(p, &p->emission_lag_ms, "Panel emission lag(ms)");
	u_var_add_ro_i64(p, &p->period_ns, "Frame period(ns)");
	u_var_add_ro_i64(p, &p->scanout_begin_ns, "Latch to scanout begin(ns)");
	u_var_add_i64(p, &p->comp_time_ns, "Wake lead(ns)");
	u_var_add_ro_i64(p, &p->vblank_base_ns, "Last vblank(ns)");

	GXR_PACER_LOG_I(
	    "Created Galaxy XR pacer: period %.4f ms (%.3f Hz), scanout begins %.3f ms (%d lines) after the latch "
	    "vsync, emission lag %.2f ms, wake lead %.2f ms",
	    time_ns_to_ms_f(p->period_ns), 1e9 / (double)p->period_ns, time_ns_to_ms_f(p->scanout_begin_ns), lead_lines,
	    p->emission_lag_ms.val, time_ns_to_ms_f(p->comp_time_ns));

	*out_upc = &p->base;

	return XRT_SUCCESS;
}
