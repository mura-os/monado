// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android XR device_profile.textproto display LUTs.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * One eye's display LUT from the Android XR device profile.
 *
 * Two on-disk generations are supported:
 *
 * v1 (display_profile, the build-baked devkit default): a grid over head
 * frame directions, cell (id_x, id_y) is tan = id * cell_step /
 * surface_distance, each cell giving the panel position in pixels (center
 * origin) where green content for that direction lands plus red/blue
 * offsets. The display cant is baked into the directions.
 *
 * v2 (display_profile_v2, the per-unit factory file on the efs partition):
 * a regular grid over panel pixels (panel_step_px apart, outermost ring
 * clamped to the panel edge), each cell giving per color the ray direction
 * in the display module frame, plus lut_frame_t_eye (module -> eye camera
 * transform) and IMU_0 -> module extrinsics. At load the rays are rotated
 * into the eye frame and stored as tangents, and the chain is composed
 * into a per-eye view pose in the IMU (device) frame.
 */
struct galaxyxr_profile_eye
{
	bool valid;
	//! v2: grid over panel px holding tangents; v1: direction grid holding panel px.
	bool forward_map;
	uint32_t rows, cols;
	//! v1: tan() per grid step.
	float tan_step;
	//! v1: the shared calibration surface distance, meters.
	float surface_dist_m;
	//! v2: panel pixels per grid step.
	float step_px;
	//! v2: panel half extent covered by the (clamped) grid, pixels.
	float half_w_px, half_h_px;
	/*!
	 * Per color (R, G, B), rows * cols values.
	 * v1: panel position in pixels, center origin, y down.
	 * v2: eye frame tangents, x right, y down, row 0 = panel top.
	 */
	struct xrt_vec2 *grid[3];
	//! v1: which cells were present in the file.
	bool *present;
	//! v1: green_fov (head frame); v2: verified_fov (eye frame).
	bool have_fov;
	struct xrt_fov fov;
	//! v2: view pose in the device (IMU_0) frame, position not recentered.
	bool have_pose;
	struct xrt_pose pose;
	//! v2: device_ipd_in_mm, 0 if absent.
	float ipd_mm;
};

/*!
 * Factory IMU intrinsics from the device profile imus{} map. Biases and
 * scales apply to the raw SSC stream in its own frame (before any axis
 * remap): true = scale * (raw - bias) per axis. Misalignment, gyro_q_accel
 * and imu_q_6pts are parsed for completeness but their conventions are
 * unverified, so they are not applied (sub-degree effects). The sigmas are
 * the VIO-style continuous-time noise densities.
 */
struct galaxyxr_imu_cal
{
	bool valid;
	//! default_gyro_bias, rad/s.
	struct xrt_vec3 gyro_bias;
	//! default_accel_bias, m/s^2.
	struct xrt_vec3 accel_bias;
	struct xrt_vec3 gyro_scale;
	struct xrt_vec3 accel_scale;
	struct xrt_vec3 gyro_misalignment;
	struct xrt_vec3 accel_misalignment;
	float gyro_noise_sigma, gyro_bias_sigma;
	float accel_noise_sigma, accel_bias_sigma;
};

int
galaxyxr_profile_load(struct galaxyxr_profile_eye out_eyes[2], const char *path);

/*!
 * Load the imus{} entry whose ordinal id matches @p imu_id (the SSC hw_id;
 * the head IMU is 0). Returns 0 on success.
 */
int
galaxyxr_profile_load_imu_cal(struct galaxyxr_imu_cal *out_cal, const char *path, int imu_id);

void
galaxyxr_profile_free(struct galaxyxr_profile_eye *eye);

/*!
 * Map a panel position (pixels, center origin, y down) to the direction
 * tangents (x right, y down) for one color channel.
 */
void
galaxyxr_profile_panel_to_tan(
    const struct galaxyxr_profile_eye *eye, uint32_t color, float px, float py, float *out_tan_x, float *out_tan_y);

#ifdef __cplusplus
}
#endif
