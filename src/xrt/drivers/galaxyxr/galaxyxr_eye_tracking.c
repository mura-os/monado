// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR eye tracking.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_eye_tracking.h"

#include "os/os_time.h"

#include "math/m_api.h"

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>


#define GXR_EYE_CAMERA_FPS 30u
#define GXR_EYE_SAMPLE_STALE_NS 250000000ll

static void
process_sample(struct galaxyxr_eye_tracking *et, const struct galaxyxr_eye_sample *sample)
{
	if (!galaxyxr_eye_tracking_is_needed(et) || sample->timestamp_ns == 0 || sample->timestamp_ns > INT64_MAX ||
	    !isfinite(sample->display_gaze[0]) || !isfinite(sample->display_gaze[1])) {
		return;
	}

	/*
	 * display_gaze is a tangent-plane prediction. Its presentation x sign
	 * is opposite OpenXR, while y already points up. Gaze poses look along
	 * -Z in OpenXR view space.
	 */
	struct xrt_vec3 gaze_direction = {
	    .x = -sample->display_gaze[0],
	    .y = sample->display_gaze[1],
	    .z = -1.0f,
	};
	math_vec3_normalize(&gaze_direction);

	struct xrt_space_relation relation = {
	    .relation_flags = XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	                      XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT,
	    .pose = XRT_POSE_IDENTITY,
	};
	const struct xrt_vec3 forward = {0.0f, 0.0f, -1.0f};
	math_quat_from_vec_a_to_vec_b(&forward, &gaze_direction, &relation.pose.orientation);

	(void)m_relation_history_push(et->relation_history, &relation, (int64_t)sample->timestamp_ns);
	if (!galaxyxr_eye_tracking_is_needed(et)) {
		m_relation_history_clear(et->relation_history);
	}
}

bool
galaxyxr_eye_tracking_init(struct galaxyxr_eye_tracking *et, struct xrt_device *xdev, enum u_logging_level *log_level)
{
	if (!galaxyxr_eye_inference_preprocessor_available(GALAXYXR_EYETRACKING_PREPROCESSOR_HVX_CHAINED)) {
		U_LOG_XDEV_IFL_W(xdev, *log_level,
		                 "Installed Galaxy XR eye-tracking library has no chained HVX support");
		return false;
	}

	if (galaxyxr_eye_inference_prepare_model(et->model_path, sizeof(et->model_path)) != 0) {
		U_LOG_XDEV_IFL_E(xdev, *log_level, "Could not prepare the OEM eye model: %s", strerror(errno));
		return false;
	}

	m_relation_history_create(&et->relation_history);
	if (et->relation_history == NULL) {
		return false;
	}

	xrt_atomic_s32_store(&et->use_count, 0);
	U_LOG_XDEV_IFL_I(xdev, *log_level, "Eye tracking available on demand: chained HVX at %u FPS",
	                 GXR_EYE_CAMERA_FPS);
	return true;
}

void
galaxyxr_eye_tracking_destroy(struct galaxyxr_eye_tracking *et)
{
	xrt_atomic_s32_store(&et->use_count, 0);
	m_relation_history_destroy(&et->relation_history);
}

void
galaxyxr_eye_tracking_retain(struct galaxyxr_eye_tracking *et)
{
	xrt_atomic_s32_inc_return(&et->use_count);
}

void
galaxyxr_eye_tracking_release(struct galaxyxr_eye_tracking *et)
{
	int32_t count = xrt_atomic_s32_dec_return(&et->use_count);
	if (count < 0) {
		U_LOG_E("Unbalanced eye tracking release");
		xrt_atomic_s32_store(&et->use_count, 0);
		count = 0;
	}
	if (count == 0 && et->relation_history != NULL) {
		m_relation_history_clear(et->relation_history);
	}
}

bool
galaxyxr_eye_tracking_is_needed(struct galaxyxr_eye_tracking *et)
{
	return xrt_atomic_s32_load(&et->use_count) > 0;
}

int
galaxyxr_eye_tracking_open(struct galaxyxr_eye_tracking *et, struct xrt_device *xdev, enum u_logging_level *log_level)
{
	struct galaxyxr_eyetracking_config config = GALAXYXR_EYETRACKING_CONFIG_INIT;
	config.qnn_context_path = et->model_path;
	config.camera_fps = GXR_EYE_CAMERA_FPS;
	config.preprocessor = GALAXYXR_EYETRACKING_PREPROCESSOR_HVX_CHAINED;

	char error[GALAXYXR_EYETRACKING_ERROR_SIZE] = {0};
	if (galaxyxr_eyetracking_open(&et->tracker, &config, error, sizeof(error)) != 0) {
		U_LOG_XDEV_IFL_E(xdev, *log_level, "Could not open eye tracking: %s",
		                 error[0] != '\0' ? error : strerror(errno));
		return -1;
	}

	U_LOG_XDEV_IFL_I(xdev, *log_level, "Eye tracking opened with chained HVX at %u FPS", GXR_EYE_CAMERA_FPS);
	return galaxyxr_eyetracking_get_event_fd(&et->tracker);
}

void
galaxyxr_eye_tracking_close(struct galaxyxr_eye_tracking *et)
{
	(void)galaxyxr_eyetracking_close(&et->tracker, NULL, 0);
	m_relation_history_clear(et->relation_history);
}

bool
galaxyxr_eye_tracking_process_events(struct galaxyxr_eye_tracking *et,
                                     struct xrt_device *xdev,
                                     enum u_logging_level *log_level)
{
	struct galaxyxr_eyetracking_event event;
	while (galaxyxr_eyetracking_read_event(&et->tracker, &event) != 0) {
		switch (event.type) {
		case GALAXYXR_EYETRACKING_EVENT_SAMPLE: process_sample(et, &event.sample); break;
		case GALAXYXR_EYETRACKING_EVENT_START:
			U_LOG_XDEV_IFL_I(xdev, *log_level, "Eye camera stream started");
			break;
		case GALAXYXR_EYETRACKING_EVENT_STOP:
			m_relation_history_clear(et->relation_history);
			U_LOG_XDEV_IFL_W(xdev, *log_level, "Eye camera stream stopped; waiting for Titan recovery");
			break;
		case GALAXYXR_EYETRACKING_EVENT_ERROR: {
			char error[GALAXYXR_EYETRACKING_ERROR_SIZE] = {0};
			galaxyxr_eyetracking_copy_error(&et->tracker, error, sizeof(error));
			m_relation_history_clear(et->relation_history);
			U_LOG_XDEV_IFL_E(xdev, *log_level, "Eye tracking failed: %s",
			                 error[0] != '\0' ? error : "unknown error");
			return false;
		}
		default: U_LOG_XDEV_IFL_W(xdev, *log_level, "Unknown eye-tracking event %d", (int)event.type); break;
		}
	}

	return true;
}

void
galaxyxr_eye_tracking_get_relation(struct galaxyxr_eye_tracking *et,
                                   int64_t at_timestamp_ns,
                                   struct xrt_space_relation *out_relation)
{
	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	if (!galaxyxr_eye_tracking_is_needed(et)) {
		return;
	}

	int64_t latest_timestamp_ns = 0;
	struct xrt_space_relation latest = XRT_SPACE_RELATION_ZERO;
	if (!m_relation_history_get_latest(et->relation_history, &latest_timestamp_ns, &latest)) {
		return;
	}

	int64_t now_ns = os_monotonic_get_ns();
	if (latest_timestamp_ns <= 0 || now_ns < latest_timestamp_ns ||
	    now_ns - latest_timestamp_ns > GXR_EYE_SAMPLE_STALE_NS) {
		return;
	}

	int64_t query_timestamp_ns = at_timestamp_ns > 0 ? at_timestamp_ns : now_ns;
	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	if (m_relation_history_get(et->relation_history, query_timestamp_ns, &relation) !=
	        M_RELATION_HISTORY_RESULT_INVALID &&
	    galaxyxr_eye_tracking_is_needed(et)) {
		*out_relation = relation;
	}
}
