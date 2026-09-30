// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Device-provided camera passthrough stream interface.
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_handles.h"
#include "xrt/xrt_limits.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XRT_PASSTHROUGH_MAX_VIEWS XRT_MAX_VIEWS
#define XRT_PASSTHROUGH_MAX_PLANES 4
#define XRT_PASSTHROUGH_MAX_SLOTS 8
#define XRT_PASSTHROUGH_CURVE_LUT_SIZE 1000

enum xrt_passthrough_format
{
	XRT_PASSTHROUGH_FORMAT_NV12 = 1,
};

/*!
 * One camera's complete optical model. All values must come from one
 * calibration family: drivers must not mix intrinsics and extrinsics from
 * unrelated files.
 */
struct xrt_passthrough_camera_calibration
{
	uint32_t width;
	uint32_t height;
	float fx;
	float fy;
	float cx;
	float cy;
	float k[4];
	float max_valid_undistorted_radius;
	float max_valid_distorted_radius;
	float distortion_offset[2];

	//! Coordinate transform from device/IMU axes to camera CV axes.
	float imu_to_camera[3][3];
	//! Positions in the calibration's device/IMU frame, in metres.
	float camera_position[3];
	float eye_position[3];

	/*!
	 * Unit vector in camera coordinates from the spherical window centre
	 * toward the camera. The LUT supplies alpha in
	 * normalize(ray) + alpha * curved_window_axis.
	 */
	float window_axis[3];
	float curved_window_lut[XRT_PASSTHROUGH_CURVE_LUT_SIZE];

	uint64_t rolling_shutter_readout_ns;
	int64_t timestamp_alignment_ns;
	bool rolling_shutter_from_metadata;
};

struct xrt_passthrough_calibration
{
	uint32_t view_count;
	struct xrt_passthrough_camera_calibration views[XRT_PASSTHROUGH_MAX_VIEWS];
	float eye_midpoint[3];
	float calibration_ipd_mm;
	char source_path[512];
};

struct xrt_passthrough_frame_view
{
	uint32_t slot;
	uint64_t frame_id;
	uint64_t sensor_request_id;
	uint64_t exposure_request_id;

	/*!
	 * Device poses in tracking space at the exposure midpoint of the first
	 * and last sensor rows. Providers must always populate both poses,
	 * resolving any provider-specific camera clock before publishing the
	 * frame.
	 */
	struct xrt_pose capture_pose_begin;
	struct xrt_pose capture_pose_end;

	/*!
	 * A reference owned by this frame. The provider closes/releases it from
	 * xrt_passthrough_stream::release_frame.
	 */
	xrt_graphics_buffer_handle_t handle;
};

/*!
 * A coherent multi-view camera frame. acquire_frame transfers one provider
 * hold to the caller; release_frame must be called exactly once.
 */
struct xrt_passthrough_frame
{
	uint64_t generation;
	enum xrt_passthrough_format format;
	uint32_t view_count;
	uint32_t width;
	uint32_t height;
	uint32_t plane_count;
	uint32_t strides[XRT_PASSTHROUGH_MAX_PLANES];
	uint32_t offsets[XRT_PASSTHROUGH_MAX_PLANES];
	uint64_t buffer_size;
	uint64_t drm_format_modifier;
	struct xrt_passthrough_frame_view views[XRT_PASSTHROUGH_MAX_VIEWS];

	//! Provider-private ownership token.
	void *token;
};

/*!
 * Generic device-to-compositor passthrough provider.
 *
 * Callbacks may be invoked from the compositor render thread. acquire_frame
 * must not block: it returns false when no newer complete frame is available.
 *
 * Streams start disabled and produce frames only while enabled.
 */
struct xrt_passthrough_stream
{
	struct xrt_passthrough_calibration calibration;

	/*!
	 * Mandatory. The consumer enables the stream while frames will be used
	 * and disables it once they no longer are. A disabled provider may
	 * close its camera source and revoke frames the consumer still holds;
	 * acquire_frame returns false until enabled again. Must not block.
	 */
	void (*set_enabled)(struct xrt_passthrough_stream *xp, bool enabled);

	bool (*acquire_frame)(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *out_frame);
	/*!
	 * Optional revocation check for a held frame. Providers whose frames
	 * remain valid until release may leave this NULL.
	 */
	bool (*is_frame_valid)(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame);
	/*!
	 * Optional. Reports one render tick that is about to sample frame.
	 * Repeated use of the same frame is reported on every tick. This does not
	 * transfer or release ownership and must not block.
	 */
	void (*mark_frame_used)(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame);
	void (*release_frame)(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *frame);
	void (*destroy)(struct xrt_passthrough_stream *xp);
};

static inline void
xrt_passthrough_stream_set_enabled(struct xrt_passthrough_stream *xp, bool enabled)
{
	if (xp != NULL) {
		xp->set_enabled(xp, enabled);
	}
}

static inline bool
xrt_passthrough_stream_acquire_frame(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *out_frame)
{
	return xp != NULL && xp->acquire_frame != NULL && xp->acquire_frame(xp, out_frame);
}

static inline bool
xrt_passthrough_stream_is_frame_valid(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame)
{
	return xp != NULL && frame != NULL && frame->token != NULL &&
	       (xp->is_frame_valid == NULL || xp->is_frame_valid(xp, frame));
}

XRT_NONNULL_ALL static inline void
xrt_passthrough_stream_mark_frame_used(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame)
{
	if (xp->mark_frame_used != NULL) {
		xp->mark_frame_used(xp, frame);
	}
}

static inline void
xrt_passthrough_stream_release_frame(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *frame)
{
	if (xp != NULL && xp->release_frame != NULL && frame != NULL && frame->token != NULL) {
		xp->release_frame(xp, frame);
	}
}

static inline void
xrt_passthrough_stream_destroy(struct xrt_passthrough_stream **xp_ptr)
{
	struct xrt_passthrough_stream *xp = xp_ptr != NULL ? *xp_ptr : NULL;
	if (xp == NULL) {
		return;
	}
	*xp_ptr = NULL;
	xp->destroy(xp);
}

#ifdef __cplusplus
}
#endif
