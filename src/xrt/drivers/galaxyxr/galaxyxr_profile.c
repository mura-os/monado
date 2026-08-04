// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android XR device_profile.textproto display LUTs.
 *
 * Two generations of the display calibration exist:
 *
 * v1, "display_profile" (the build-baked devkit default under
 * /product/etc): per eye a lut_rows x lut_cols grid over a virtual surface
 * at surface_distance meters, cell_step meters apart, so cell (id_x, id_y)
 * is the head frame direction with tan = id * cell_step / surface_distance
 * (display cant baked in). Each cell stores the panel position (u, v) in
 * pixels, center origin, where green content for that direction lands, and
 * red/blue pixel offsets on top of it. green_fov gives the calibrated
 * valid fov in the same frame.
 *
 * v2, "display_profile_v2" (the per-unit factory calibration on the efs
 * partition): per eye a regular grid over panel pixels (u right, v UP,
 * center origin, outermost ring clamped to the panel edge), each cell
 * storing per color the unnormalized ray direction in the display module
 * frame (x right, y up, forward -z, the frame of sensor_id "1"/"2").
 * lut_frame_t_eye gives the module -> eye-camera transform and top level
 * sensor_extrinsics give IMU_0 -> module; in both, p is the origin of
 * frame b in frame a and q rotates vector coordinates from a to b.
 * verified_fov (eye frame, per-side magnitudes) is the calibrated render
 * fov and device_ipd_in_mm the IPD the LUT was calibrated at. On load the
 * rays are rotated straight into the IMU (head) frame and stored as
 * tangents (x right, y down), verified_fov is rebounded there, and the
 * view poses get identity orientation plus the composed eye position
 * p_imu_mod + rot(conj(q_imu_mod), p_mod_eye) - see finish_eye_v2 for why
 * the rotation is folded instead of exposed.
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_profile.h"

#include "util/u_logging.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct parse_cell
{
	float u, v, red_off_x, red_off_y, blue_off_x, blue_off_y;
	float ray[3][3];
	int id_x, id_y;
	uint32_t have;
};

struct parse_ext
{
	char id[8];
	bool from_imu;
	float p[3];
	float q[4]; //!< w, x, y, z
	uint32_t have_p, have_q;
};

struct parse_eye_v2
{
	struct xrt_vec3 *raw[3];
	bool *have;
	float step_px;
	float half_w, half_h;
	struct parse_ext t;
	float fov_verified[4], fov_green[4]; //!< left, right, top, bottom
	uint32_t fov_verified_have, fov_green_have;
	float ipd_mm;
};

static bool
key_value(const char *line, const char *key, float *out)
{
	while (*line == ' ' || *line == '\t') {
		line++;
	}
	size_t klen = strlen(key);
	if (strncmp(line, key, klen) != 0 || line[klen] != ':') {
		return false;
	}
	*out = strtof(line + klen + 1, NULL);
	return true;
}

static bool
key_string(const char *line, const char *key, char *out, size_t out_len)
{
	size_t klen = strlen(key);
	if (strncmp(line, key, klen) != 0 || line[klen] != ':') {
		return false;
	}
	const char *q1 = strchr(line + klen + 1, '"');
	if (q1 == NULL) {
		return false;
	}
	const char *q2 = strchr(q1 + 1, '"');
	if (q2 == NULL || (size_t)(q2 - q1) > out_len) {
		return false;
	}
	memcpy(out, q1 + 1, q2 - q1 - 1);
	out[q2 - q1 - 1] = '\0';
	return true;
}

/*
 *
 * Quaternion helpers, file order (w, x, y, z), coordinate-transform math
 * mirrored from the verified reference implementation.
 *
 */

static void
quat_mul(const float a[4], const float b[4], float out[4])
{
	float r[4] = {
	    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
	    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
	    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
	    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
	};
	memcpy(out, r, sizeof(r));
}

static void
quat_conj(const float q[4], float out[4])
{
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

static void
quat_rot_vec(const float q[4], const float v[3], float out[3])
{
	float p[4] = {0.0f, v[0], v[1], v[2]};
	float c[4], t[4];
	quat_conj(q, c);
	quat_mul(q, p, t);
	quat_mul(t, c, p);
	out[0] = p[1];
	out[1] = p[2];
	out[2] = p[3];
}

/*
 *
 * v1 handling.
 *
 */

static void
cell_store_v1(struct galaxyxr_profile_eye *eye, const struct parse_cell *c)
{
	int col = c->id_x + (int)(eye->cols - 1) / 2;
	int row = c->id_y + (int)(eye->rows - 1) / 2;
	if (col < 0 || col >= (int)eye->cols || row < 0 || row >= (int)eye->rows) {
		return;
	}
	uint32_t i = (uint32_t)row * eye->cols + (uint32_t)col;
	eye->grid[0][i] = (struct xrt_vec2){c->u + c->red_off_x, c->v + c->red_off_y};
	eye->grid[1][i] = (struct xrt_vec2){c->u, c->v};
	eye->grid[2][i] = (struct xrt_vec2){c->u + c->blue_off_x, c->v + c->blue_off_y};
	eye->present[i] = true;
}

// Fill cells outside the calibrated circle by linear extrapolation, first
// along rows from the innermost valid pair, then along columns.
static void
extrapolate_missing(struct galaxyxr_profile_eye *eye)
{
	for (uint32_t c = 0; c < 3; c++) {
		struct xrt_vec2 *g = eye->grid[c];

		for (uint32_t row = 0; row < eye->rows; row++) {
			struct xrt_vec2 *r = &g[row * eye->cols];
			bool *p = &eye->present[row * eye->cols];
			int first = -1, last = -1;
			for (uint32_t col = 0; col < eye->cols; col++) {
				if (p[col]) {
					if (first < 0) {
						first = (int)col;
					}
					last = (int)col;
				}
			}
			if (first < 0 || last - first < 1) {
				continue;
			}
			for (int col = first - 1; col >= 0; col--) {
				r[col].x = r[col + 1].x * 2 - r[col + 2].x;
				r[col].y = r[col + 1].y * 2 - r[col + 2].y;
			}
			for (int col = last + 1; col < (int)eye->cols; col++) {
				r[col].x = r[col - 1].x * 2 - r[col - 2].x;
				r[col].y = r[col - 1].y * 2 - r[col - 2].y;
			}
		}

		for (uint32_t col = 0; col < eye->cols; col++) {
			int first = -1, last = -1;
			for (uint32_t row = 0; row < eye->rows; row++) {
				if (eye->present[row * eye->cols + col]) {
					if (first < 0) {
						first = (int)row;
					}
					last = (int)row;
				}
			}
			if (first < 0 || last - first < 1) {
				continue;
			}
			for (int row = first - 1; row >= 0; row--) {
				struct xrt_vec2 a = g[(row + 1) * eye->cols + col];
				struct xrt_vec2 b = g[(row + 2) * eye->cols + col];
				g[row * eye->cols + col] = (struct xrt_vec2){a.x * 2 - b.x, a.y * 2 - b.y};
			}
			for (int row = last + 1; row < (int)eye->rows; row++) {
				struct xrt_vec2 a = g[(row - 1) * eye->cols + col];
				struct xrt_vec2 b = g[(row - 2) * eye->cols + col];
				g[row * eye->cols + col] = (struct xrt_vec2){a.x * 2 - b.x, a.y * 2 - b.y};
			}
		}
	}
}

static void
fov_from_radians(struct galaxyxr_profile_eye *eye, const float f[4])
{
	// Signed (x right, y down) in some files, per-side magnitudes in others.
	eye->fov.angle_left = -fabsf(f[0]);
	eye->fov.angle_right = fabsf(f[1]);
	eye->fov.angle_up = fabsf(f[2]);
	eye->fov.angle_down = -fabsf(f[3]);
	eye->have_fov = true;
	for (uint32_t i = 0; i < 4; i++) {
		if (fabsf(f[i]) < 0.17f || fabsf(f[i]) > 1.5f) {
			eye->have_fov = false;
		}
	}
}

/*
 *
 * v2 handling.
 *
 */

static void
cell_store_v2(struct galaxyxr_profile_eye *eye, struct parse_eye_v2 *v2, const struct parse_cell *c)
{
	int hc = (int)(eye->cols - 1) / 2;
	int hr = (int)(eye->rows - 1) / 2;
	int col = c->id_x + hc;
	int row = hr - c->id_y; // file v axis is up, store row 0 = panel top
	if (col < 0 || col >= (int)eye->cols || row < 0 || row >= (int)eye->rows) {
		return;
	}
	if (v2->step_px == 0.0f && c->id_x != 0 && abs(c->id_x) < hc) {
		v2->step_px = c->u / (float)c->id_x;
	}
	if (fabsf(c->u) > v2->half_w) {
		v2->half_w = fabsf(c->u);
	}
	if (fabsf(c->v) > v2->half_h) {
		v2->half_h = fabsf(c->v);
	}
	uint32_t i = (uint32_t)row * eye->cols + (uint32_t)col;
	for (uint32_t k = 0; k < 3; k++) {
		v2->raw[k][i] = (struct xrt_vec3){c->ray[k][0], c->ray[k][1], c->ray[k][2]};
	}
	v2->have[i] = true;
}

static bool
finish_eye_v2(struct galaxyxr_profile_eye *eye, struct parse_eye_v2 *v2, const struct parse_ext *se, uint32_t se_count)
{
	uint32_t n = eye->rows * eye->cols;
	uint32_t present = 0;
	for (uint32_t i = 0; i < n; i++) {
		present += v2->have[i];
	}
	if (present != n || v2->step_px <= 0.0f || v2->t.have_q != 4) {
		U_LOG_W("galaxyxr: v2 profile incomplete: %u/%u cells, step %.1f, quat %u", present, n, v2->step_px,
		        v2->t.have_q);
		return false;
	}

	eye->forward_map = true;
	eye->step_px = v2->step_px;
	eye->half_w_px = v2->half_w;
	eye->half_h_px = v2->half_h;
	eye->ipd_mm = v2->ipd_mm;

	const struct parse_ext *s = NULL;
	for (uint32_t i = 0; i < se_count; i++) {
		if (se[i].from_imu && strcmp(se[i].id, v2->t.id) == 0 && se[i].have_q == 4 && se[i].have_p == 3) {
			s = &se[i];
		}
	}

	// The view orientation is kept identity and the whole per-eye rotation
	// is folded into the mapping: many clients mishandle rotated view
	// poses (xrgears' vulkan y-flip via world mirroring negates per-eye
	// pitch/roll), while identity views + asymmetric fov is the path every
	// app is tested against. So map the rays straight into the IMU (head)
	// frame when the module extrinsics are available, and rebound the
	// eye-frame verified_fov there; otherwise fall back to the eye frame.
	float cs[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	bool to_head = s != NULL;
	if (to_head) {
		quat_conj(s->q, cs);
	}

	for (uint32_t c = 0; c < 3; c++) {
		for (uint32_t i = 0; i < n; i++) {
			float d[3];
			if (to_head) {
				// Module -> IMU, GL axes (y up, forward -z).
				quat_rot_vec(cs, (const float *)&v2->raw[c][i], d);
				d[2] = -d[2];
				d[1] = -d[1];
			} else {
				// Module -> eye camera (x right, y down, z forward).
				quat_rot_vec(v2->t.q, (const float *)&v2->raw[c][i], d);
			}
			if (d[2] < 0.001f) {
				d[2] = 0.001f;
			}
			eye->grid[c][i] = (struct xrt_vec2){d[0] / d[2], d[1] / d[2]};
		}
	}

	const float *fovv = v2->fov_verified_have == 4 ? v2->fov_verified
	                                               : (v2->fov_green_have == 4 ? v2->fov_green : NULL);
	if (fovv != NULL) {
		fov_from_radians(eye, fovv);
	}
	if (eye->have_fov && to_head) {
		// verified_fov is an eye frame box: bound its rotated corners.
		float ce[4];
		quat_conj(v2->t.q, ce);
		float tl = tanf(eye->fov.angle_left);
		float tr = tanf(eye->fov.angle_right);
		float tt = -tanf(eye->fov.angle_up);
		float tb = -tanf(eye->fov.angle_down);
		float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
		for (uint32_t k = 0; k < 4; k++) {
			float corner[3] = {k & 1 ? tr : tl, k & 2 ? tb : tt, 1.0f};
			float d[3];
			quat_rot_vec(ce, corner, d);
			quat_rot_vec(cs, d, d);
			float x = d[0] / -d[2];
			float y = -d[1] / -d[2];
			if (k == 0 || x < min_x) {
				min_x = x;
			}
			if (k == 0 || x > max_x) {
				max_x = x;
			}
			if (k == 0 || y < min_y) {
				min_y = y;
			}
			if (k == 0 || y > max_y) {
				max_y = y;
			}
		}
		eye->fov.angle_left = atanf(min_x);
		eye->fov.angle_right = atanf(max_x);
		eye->fov.angle_up = atanf(-min_y);
		eye->fov.angle_down = atanf(-max_y);
	}

	if (to_head && v2->t.have_p == 3) {
		float p[3];
		quat_rot_vec(cs, v2->t.p, p);
		eye->pose.orientation = (struct xrt_quat)XRT_QUAT_IDENTITY;
		eye->pose.position = (struct xrt_vec3){s->p[0] + p[0], s->p[1] + p[1], s->p[2] + p[2]};
		eye->have_pose = true;
	} else {
		U_LOG_W("galaxyxr: v2 profile lacks IMU extrinsics for sensor '%s'", v2->t.id);
	}

	return true;
}

/*
 *
 * Loading.
 *
 */

int
galaxyxr_profile_load(struct galaxyxr_profile_eye out_eyes[2], const char *path)
{
	memset(out_eyes, 0, 2 * sizeof(*out_eyes));

	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return -1;
	}

	struct parse_eye_v2 v2[2];
	memset(v2, 0, sizeof(v2));
	struct parse_ext sensor_ext[8];
	memset(sensor_ext, 0, sizeof(sensor_ext));
	uint32_t sensor_ext_count = 0;

	struct galaxyxr_profile_eye *eye = NULL;
	struct parse_eye_v2 *v2eye = NULL;
	int version = 0;
	bool in_cell = false;
	struct parse_cell cell = {0};
	int ext_depth = 0; // sensor_extrinsics / lut_frame_t_eye subtree depth
	struct parse_ext *ext = NULL;
	int ext_vec = 0; // 1 = in p, 2 = in q
	int fov_block = 0; // 1 = green_fov, 2 = verified_fov
	float rows = 0, cols = 0, step = 0, dist = 0;
	float fovv[4] = {0};
	uint32_t fov_have = 0;
	char line[256];

	while (fgets(line, sizeof(line), f) != NULL) {
		const char *t = line;
		while (*t == ' ' || *t == '\t') {
			t++;
		}
		size_t tlen = strcspn(t, "\r\n");

		if (ext != NULL) {
			if (tlen > 0 && t[tlen - 1] == '{') {
				ext_depth++;
				if (strncmp(t, "p {", 3) == 0) {
					ext_vec = 1;
				} else if (strncmp(t, "q {", 3) == 0) {
					ext_vec = 2;
				}
			} else if (t[0] == '}') {
				if (ext_vec != 0 && ext_depth == 3) {
					ext_vec = 0;
				}
				if (--ext_depth == 0) {
					if (ext->from_imu && sensor_ext_count < ARRAY_SIZE(sensor_ext) &&
					    ext == &sensor_ext[sensor_ext_count]) {
						sensor_ext_count++;
					}
					ext = NULL;
					ext_vec = 0;
				}
			} else if (strncmp(t, "frame_id: IMU_0", 15) == 0) {
				ext->from_imu = true;
			} else if (key_string(t, "sensor_id", ext->id, sizeof(ext->id))) {
			} else if (ext_vec == 1 &&
			           (key_value(t, "x", &ext->p[0]) || key_value(t, "y", &ext->p[1]) ||
			            key_value(t, "z", &ext->p[2]))) {
				ext->have_p++;
			} else if (ext_vec == 2 &&
			           (key_value(t, "w", &ext->q[0]) || key_value(t, "x", &ext->q[1]) ||
			            key_value(t, "y", &ext->q[2]) || key_value(t, "z", &ext->q[3]))) {
				ext->have_q++;
			}
			continue;
		}

		if (line[0] == '}') {
			eye = NULL;
			v2eye = NULL;
			version = 0;
			continue;
		}
		if (strncmp(line, "display_profile {", 17) == 0) {
			version = 1;
			continue;
		}
		if (strncmp(line, "display_profile_v2 {", 20) == 0) {
			version = 2;
			continue;
		}
		if (strncmp(line, "sensor_extrinsics {", 19) == 0) {
			if (sensor_ext_count < ARRAY_SIZE(sensor_ext)) {
				ext = &sensor_ext[sensor_ext_count];
				memset(ext, 0, sizeof(*ext));
				ext_depth = 1;
				ext_vec = 0;
			}
			continue;
		}
		if (version == 0) {
			continue;
		}
		if (strncmp(t, "left_display_profile {", 22) == 0) {
			eye = &out_eyes[0];
			v2eye = version == 2 ? &v2[0] : NULL;
			rows = cols = step = dist = 0;
			fov_have = 0;
			fov_block = 0;
			continue;
		}
		if (strncmp(t, "right_display_profile {", 23) == 0) {
			eye = &out_eyes[1];
			v2eye = version == 2 ? &v2[1] : NULL;
			rows = cols = step = dist = 0;
			fov_have = 0;
			fov_block = 0;
			continue;
		}
		if (eye == NULL) {
			continue;
		}

		if (eye->grid[0] == NULL) {
			key_value(t, "lut_rows", &rows);
			key_value(t, "lut_cols", &cols);
			if (version == 1) {
				key_value(t, "cell_step_x_on_surface_in_meter", &step);
				key_value(t, "surface_distance_to_nominal_eye_in_meter", &dist);
			}
			if (rows >= 3 && cols >= 3 && rows < 512 && cols < 512 &&
			    (version == 2 || (step > 0 && dist > 0))) {
				eye->rows = (uint32_t)rows;
				eye->cols = (uint32_t)cols;
				if (version == 1) {
					eye->tan_step = step / dist;
					eye->surface_dist_m = dist;
					eye->present = calloc(eye->rows * eye->cols, sizeof(bool));
				} else {
					for (uint32_t c = 0; c < 3; c++) {
						v2eye->raw[c] =
						    calloc(eye->rows * eye->cols, sizeof(struct xrt_vec3));
					}
					v2eye->have = calloc(eye->rows * eye->cols, sizeof(bool));
				}
				for (uint32_t c = 0; c < 3; c++) {
					eye->grid[c] = calloc(eye->rows * eye->cols, sizeof(struct xrt_vec2));
				}
			}
			continue;
		}

		if (version == 2) {
			if (strncmp(t, "lut_frame_t_eye {", 17) == 0) {
				ext = &v2eye->t;
				memset(ext, 0, sizeof(*ext));
				ext_depth = 1;
				ext_vec = 0;
				continue;
			}
			if (strncmp(t, "green_fov {", 11) == 0) {
				fov_block = 1;
				continue;
			}
			if (strncmp(t, "verified_fov {", 14) == 0) {
				fov_block = 2;
				continue;
			}
			float *fv = fov_block == 2 ? v2eye->fov_verified : v2eye->fov_green;
			uint32_t *fh = fov_block == 2 ? &v2eye->fov_verified_have : &v2eye->fov_green_have;
			if (key_value(t, "left_radians", &fv[0]) || key_value(t, "right_radians", &fv[1]) ||
			    key_value(t, "top_radians", &fv[2]) || key_value(t, "bottom_radians", &fv[3])) {
				(*fh)++;
				continue;
			}
			key_value(t, "device_ipd_in_mm", &v2eye->ipd_mm);
		} else {
			if (key_value(t, "left_radians", &fovv[0]) || key_value(t, "right_radians", &fovv[1]) ||
			    key_value(t, "top_radians", &fovv[2]) || key_value(t, "bottom_radians", &fovv[3])) {
				if (++fov_have == 4) {
					fov_from_radians(eye, fovv);
				}
				continue;
			}
		}

		if (strncmp(t, "cells {", 7) == 0) {
			in_cell = true;
			memset(&cell, 0, sizeof(cell));
			continue;
		}
		if (!in_cell) {
			continue;
		}

		float v;
		if (key_value(t, "u", &cell.u) || key_value(t, "v", &cell.v)) {
			cell.have++;
			continue;
		}
		if (version == 1 &&
		    (key_value(t, "red_off_x", &cell.red_off_x) || key_value(t, "red_off_y", &cell.red_off_y) ||
		     key_value(t, "blue_off_x", &cell.blue_off_x) || key_value(t, "blue_off_y", &cell.blue_off_y))) {
			cell.have++;
			continue;
		}
		if (version == 2 &&
		    (key_value(t, "red_x", &cell.ray[0][0]) || key_value(t, "red_y", &cell.ray[0][1]) ||
		     key_value(t, "red_z", &cell.ray[0][2]) || key_value(t, "green_x", &cell.ray[1][0]) ||
		     key_value(t, "green_y", &cell.ray[1][1]) || key_value(t, "green_z", &cell.ray[1][2]) ||
		     key_value(t, "blue_x", &cell.ray[2][0]) || key_value(t, "blue_y", &cell.ray[2][1]) ||
		     key_value(t, "blue_z", &cell.ray[2][2]))) {
			cell.have++;
			continue;
		}
		if (key_value(t, "id_x", &v)) {
			cell.id_x = (int)v;
			cell.have++;
			continue;
		}
		if (key_value(t, "id_y", &v)) {
			cell.id_y = (int)v;
			cell.have++;
			continue;
		}
		if (t[0] == '}') {
			if (version == 1 && cell.have >= 8) {
				cell_store_v1(eye, &cell);
			} else if (version == 2 && cell.have >= 13) {
				cell_store_v2(eye, v2eye, &cell);
			}
			in_cell = false;
		}
	}
	fclose(f);

	bool ok = out_eyes[0].grid[0] != NULL && out_eyes[1].grid[0] != NULL;
	for (uint32_t e = 0; ok && e < 2; e++) {
		if (v2[e].have != NULL) {
			ok = finish_eye_v2(&out_eyes[e], &v2[e], sensor_ext, sensor_ext_count);
		} else {
			uint32_t present = 0;
			for (uint32_t i = 0; i < out_eyes[e].rows * out_eyes[e].cols; i++) {
				present += out_eyes[e].present[i];
			}
			if (present < out_eyes[e].rows * out_eyes[e].cols / 2) {
				U_LOG_W("galaxyxr: display profile %u only has %u cells", e, present);
				ok = false;
			} else {
				extrapolate_missing(&out_eyes[e]);
			}
		}
		if (ok && !out_eyes[e].have_fov) {
			U_LOG_W("galaxyxr: display profile %u lacks a usable fov", e);
			ok = false;
		}
		out_eyes[e].valid = ok;
	}

	for (uint32_t e = 0; e < 2; e++) {
		for (uint32_t c = 0; c < 3; c++) {
			free(v2[e].raw[c]);
		}
		free(v2[e].have);
	}
	if (!ok) {
		galaxyxr_profile_free(&out_eyes[0]);
		galaxyxr_profile_free(&out_eyes[1]);
		return -1;
	}

	return 0;
}

int
galaxyxr_profile_load_imu_cal(struct galaxyxr_imu_cal *out_cal, const char *path, int imu_id)
{
	memset(out_cal, 0, sizeof(*out_cal));

	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return -1;
	}

	char want_id[8];
	snprintf(want_id, sizeof(want_id), "%d", imu_id);

	struct galaxyxr_imu_cal cur;
	char id[8] = {0};
	int depth = 0;
	struct xrt_vec3 *vec = NULL;
	bool found = false;
	char line[256];

	while (!found && fgets(line, sizeof(line), f) != NULL) {
		const char *t = line;
		while (*t == ' ' || *t == '\t') {
			t++;
		}
		size_t tlen = strcspn(t, "\r\n");

		if (depth == 0) {
			if (strncmp(line, "imus {", 6) == 0) {
				depth = 1;
				vec = NULL;
				id[0] = '\0';
				memset(&cur, 0, sizeof(cur));
				cur.gyro_scale = (struct xrt_vec3){1.0f, 1.0f, 1.0f};
				cur.accel_scale = (struct xrt_vec3){1.0f, 1.0f, 1.0f};
			}
			continue;
		}

		if (tlen > 0 && t[tlen - 1] == '{') {
			depth++;
			if (strncmp(t, "gyro_scale {", 12) == 0) {
				vec = &cur.gyro_scale;
			} else if (strncmp(t, "gyro_misalignment {", 19) == 0) {
				vec = &cur.gyro_misalignment;
			} else if (strncmp(t, "accel_scale {", 13) == 0) {
				vec = &cur.accel_scale;
			} else if (strncmp(t, "accel_misalignment {", 20) == 0) {
				vec = &cur.accel_misalignment;
			} else if (strncmp(t, "default_gyro_bias {", 19) == 0) {
				vec = &cur.gyro_bias;
			} else if (strncmp(t, "default_accel_bias {", 20) == 0) {
				vec = &cur.accel_bias;
			} else {
				vec = NULL;
			}
			continue;
		}
		if (t[0] == '}') {
			vec = NULL;
			if (--depth == 0 && strcmp(id, want_id) == 0) {
				*out_cal = cur;
				out_cal->valid = true;
				found = true;
			}
			continue;
		}
		if (vec != NULL &&
		    (key_value(t, "x", &vec->x) || key_value(t, "y", &vec->y) || key_value(t, "z", &vec->z))) {
			continue;
		}
		if (key_string(t, "id", id, sizeof(id)) ||                          //
		    key_value(t, "gyro_noise_sigma", &cur.gyro_noise_sigma) ||      //
		    key_value(t, "gyro_bias_sigma", &cur.gyro_bias_sigma) ||        //
		    key_value(t, "accel_noise_sigma", &cur.accel_noise_sigma) ||    //
		    key_value(t, "accel_bias_sigma", &cur.accel_bias_sigma)) {
			continue;
		}
	}
	fclose(f);

	return found ? 0 : -1;
}

void
galaxyxr_profile_free(struct galaxyxr_profile_eye *eye)
{
	for (uint32_t c = 0; c < 3; c++) {
		free(eye->grid[c]);
		eye->grid[c] = NULL;
	}
	free(eye->present);
	eye->present = NULL;
	eye->valid = false;
}

/*
 *
 * Panel -> direction mapping.
 *
 */

// Grid position (fractional cell coords) -> stored value, bilinear with
// linear extrapolation outside.
static void
grid_sample(const struct galaxyxr_profile_eye *eye, uint32_t color, float gx, float gy, float *out_x, float *out_y)
{
	int cx = (int)floorf(gx);
	int cy = (int)floorf(gy);
	if (cx < 0) {
		cx = 0;
	}
	if (cx > (int)eye->cols - 2) {
		cx = (int)eye->cols - 2;
	}
	if (cy < 0) {
		cy = 0;
	}
	if (cy > (int)eye->rows - 2) {
		cy = (int)eye->rows - 2;
	}
	float fx = gx - (float)cx;
	float fy = gy - (float)cy;

	const struct xrt_vec2 *g = eye->grid[color];
	const struct xrt_vec2 *p00 = &g[cy * eye->cols + cx];
	const struct xrt_vec2 *p10 = &g[cy * eye->cols + cx + 1];
	const struct xrt_vec2 *p01 = &g[(cy + 1) * eye->cols + cx];
	const struct xrt_vec2 *p11 = &g[(cy + 1) * eye->cols + cx + 1];

	*out_x = (p00->x * (1 - fx) + p10->x * fx) * (1 - fy) + (p01->x * (1 - fx) + p11->x * fx) * fy;
	*out_y = (p00->y * (1 - fx) + p10->y * fx) * (1 - fy) + (p01->y * (1 - fx) + p11->y * fx) * fy;
}

// Panel px (center origin) -> signed cell coordinate; the outermost cell
// ring is clamped to the panel edge and so covers less than step_px.
static float
grid_coord(float p, float step, float half_cells, float extent)
{
	float edge = (half_cells - 1.0f) * step;
	if (extent <= edge) {
		return p / step;
	}
	if (p > edge) {
		return (half_cells - 1.0f) + (p - edge) / (extent - edge);
	}
	if (p < -edge) {
		return -(half_cells - 1.0f) - (-p - edge) / (extent - edge);
	}
	return p / step;
}

void
galaxyxr_profile_panel_to_tan(
    const struct galaxyxr_profile_eye *eye, uint32_t color, float px, float py, float *out_tan_x, float *out_tan_y)
{
	float half_col = (float)(eye->cols - 1) / 2.0f;
	float half_row = (float)(eye->rows - 1) / 2.0f;

	if (eye->forward_map) {
		float gx = grid_coord(px, eye->step_px, half_col, eye->half_w_px) + half_col;
		float gy = grid_coord(py, eye->step_px, half_row, eye->half_h_px) + half_row;
		grid_sample(eye, color, gx, gy, out_tan_x, out_tan_y);
		return;
	}

	// Cells are roughly 45 px apart on the panel, damped fixed point on
	// the grid coordinates converges in a handful of iterations.
	float gx = half_col;
	float gy = half_row;
	for (uint32_t i = 0; i < 40; i++) {
		float fx, fy;
		grid_sample(eye, color, gx, gy, &fx, &fy);
		float ex = px - fx;
		float ey = py - fy;
		gx += ex * (0.75f / 45.0f);
		gy += ey * (0.75f / 45.0f);
		if (fabsf(ex) < 0.05f && fabsf(ey) < 0.05f) {
			break;
		}
	}

	*out_tan_x = (gx - half_col) * eye->tan_step;
	*out_tan_y = (gy - half_row) * eye->tan_step;
}
