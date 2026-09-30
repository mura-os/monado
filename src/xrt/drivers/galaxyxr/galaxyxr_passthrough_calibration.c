// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*
 * Galaxy XR per-unit VST calibration.
 *
 * The EFS device profile is a textproto rather than a protobuf wire stream,
 * so a small path-aware parser is sufficient here. Camera intrinsics,
 * extrinsics, window geometry, and timing belong to one calibration family:
 * this loader deliberately accepts only rgb-*_curved from the same profile.
 */

#include "galaxyxr_passthrough_calibration.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MAX_BLOCK_DEPTH 16
#define BLOCK_NAME_SIZE 64
#define CALIBRATION_VIEWS 2

/*
 * The reference CurvedWindowDistortionLut shipped on the headset generates
 * its optical correction at a one metre axial depth. This is independent of
 * the passthrough compositor's (normally two metre) camera/eye translation
 * plane.
 */
#define CURVED_WINDOW_OPTIMAL_DEPTH_M 1.0

enum camera_field
{
	CAM_WIDTH = 1ull << 0,
	CAM_HEIGHT = 1ull << 1,
	CAM_FX = 1ull << 2,
	CAM_FY = 1ull << 3,
	CAM_CX = 1ull << 4,
	CAM_CY = 1ull << 5,
	CAM_K1 = 1ull << 6,
	CAM_K2 = 1ull << 7,
	CAM_K3 = 1ull << 8,
	CAM_K4 = 1ull << 9,
	CAM_MAX_UNDISTORTED = 1ull << 10,
	CAM_MAX_DISTORTED = 1ull << 11,
	CAM_OFFSET_X = 1ull << 12,
	CAM_OFFSET_Y = 1ull << 13,
	CAM_WINDOW_X = 1ull << 14,
	CAM_WINDOW_Y = 1ull << 15,
	CAM_WINDOW_Z = 1ull << 16,
	CAM_WINDOW_RADIUS = 1ull << 17,
	CAM_WINDOW_THICKNESS = 1ull << 18,
	CAM_WINDOW_INDEX = 1ull << 19,
	CAM_ROLLING_TIME = 1ull << 20,
	CAM_TIMESTAMP_ALIGNMENT = 1ull << 21,
	CAM_EXPOSURE_BEGIN = 1ull << 22,
	CAM_TIMEBASE_BOOTTIME = 1ull << 23,
	CAM_ROLLING_POS_Y = 1ull << 24,
	CAM_ROLLING_SOURCE = 1ull << 25,
};

#define CAM_REQUIRED ((1ull << 26) - 1)
#define HAVE_X (1u << 0)
#define HAVE_Y (1u << 1)
#define HAVE_Z (1u << 2)
#define HAVE_W (1u << 3)
#define HAVE_XYZ (HAVE_X | HAVE_Y | HAVE_Z)
#define HAVE_WXYZ (HAVE_W | HAVE_XYZ)

struct parsed_camera
{
	char id[64];
	uint64_t have;
	double width;
	double height;
	double fx;
	double fy;
	double cx;
	double cy;
	double k[4];
	double max_undistorted;
	double max_distorted;
	double offset[2];
	double window_center[3];
	double window_radius;
	double window_thickness;
	double window_index;
	double rolling_time_ns;
	double timestamp_alignment_ns;
	bool rolling_from_metadata;
};

struct parsed_transform
{
	char id[64];
	bool from_imu;
	double p[3];
	double q[4]; // w, x, y, z
	uint32_t have_p;
	uint32_t have_q;
};

struct parsed_display_eye
{
	char sensor_id[64];
	double p[3];
	double q[4];
	uint32_t have_p;
	uint32_t have_q;
	double ipd_mm;
	bool have_ipd;
};

struct profile_parser
{
	char block[MAX_BLOCK_DEPTH][BLOCK_NAME_SIZE];
	uint32_t depth;

	struct parsed_camera current_camera;
	struct parsed_transform current_camera_ext;
	struct parsed_transform current_sensor_ext;

	struct parsed_camera camera[CALIBRATION_VIEWS];
	struct parsed_transform camera_ext[CALIBRATION_VIEWS];
	struct parsed_transform sensor_ext[CALIBRATION_VIEWS];
	struct parsed_display_eye display[CALIBRATION_VIEWS];
	bool have_camera[CALIBRATION_VIEWS];
	bool have_camera_ext[CALIBRATION_VIEWS];
	bool have_sensor_ext[CALIBRATION_VIEWS];
};

struct point2
{
	double x;
	double y;
};

struct window2
{
	double center_distance;
	double inner_radius;
	double outer_radius;
	double refractive_index;
};

static const char *default_profile_paths[] = {
    "/mnt/vendor/efs/device_profile.textproto",
    "/.oldroot/mnt/vendor/efs/device_profile.textproto",
};

static char *
trim(char *text)
{
	while (*text == ' ' || *text == '\t' || *text == '\r' || *text == '\n') {
		text++;
	}
	size_t length = strlen(text);
	while (length > 0 && (text[length - 1] == ' ' || text[length - 1] == '\t' || text[length - 1] == '\r' ||
	                      text[length - 1] == '\n')) {
		text[--length] = '\0';
	}
	return text;
}

static bool
key_number(const char *line, const char *key, double *out)
{
	size_t length = strlen(key);
	if (strncmp(line, key, length) != 0 || line[length] != ':') {
		return false;
	}
	char *end = NULL;
	errno = 0;
	double value = strtod(line + length + 1, &end);
	if (errno != 0 || end == line + length + 1 || !isfinite(value)) {
		return false;
	}
	*out = value;
	return true;
}

static bool
key_string(const char *line, const char *key, char *out, size_t out_size)
{
	size_t length = strlen(key);
	if (strncmp(line, key, length) != 0 || line[length] != ':') {
		return false;
	}
	const char *first = strchr(line + length + 1, '"');
	if (first == NULL) {
		return false;
	}
	const char *last = strchr(first + 1, '"');
	if (last == NULL || last == first + 1 || (size_t)(last - first) >= out_size) {
		return false;
	}
	size_t value_length = (size_t)(last - first - 1);
	memcpy(out, first + 1, value_length);
	out[value_length] = '\0';
	return true;
}

static bool
key_token(const char *line, const char *key, const char *expected)
{
	size_t length = strlen(key);
	if (strncmp(line, key, length) != 0 || line[length] != ':') {
		return false;
	}
	const char *value = line + length + 1;
	while (*value == ' ' || *value == '\t') {
		value++;
	}
	return strcmp(value, expected) == 0;
}

static bool
block_is(const struct profile_parser *parser, uint32_t index, const char *name)
{
	return index < parser->depth && strcmp(parser->block[index], name) == 0;
}

static int
find_block(const struct profile_parser *parser, const char *name)
{
	for (uint32_t i = 0; i < parser->depth; i++) {
		if (strcmp(parser->block[i], name) == 0) {
			return (int)i;
		}
	}
	return -1;
}

static int
camera_eye(const char *id)
{
	if (strcmp(id, "rgb-left_curved") == 0) {
		return 0;
	}
	if (strcmp(id, "rgb-right_curved") == 0) {
		return 1;
	}
	return -1;
}

static int
sensor_eye(const char *id)
{
	if (strcmp(id, "1") == 0) {
		return 0;
	}
	if (strcmp(id, "2") == 0) {
		return 1;
	}
	return -1;
}

static void
parse_xyz(const char *line, double value[3], uint32_t *have, uint32_t x_bit, uint32_t y_bit, uint32_t z_bit)
{
	if (key_number(line, "x", &value[0])) {
		*have |= x_bit;
	} else if (key_number(line, "y", &value[1])) {
		*have |= y_bit;
	} else if (key_number(line, "z", &value[2])) {
		*have |= z_bit;
	}
}

static void
parse_quaternion(const char *line, double q[4], uint32_t *have)
{
	if (key_number(line, "w", &q[0])) {
		*have |= HAVE_W;
	} else if (key_number(line, "x", &q[1])) {
		*have |= HAVE_X;
	} else if (key_number(line, "y", &q[2])) {
		*have |= HAVE_Y;
	} else if (key_number(line, "z", &q[3])) {
		*have |= HAVE_Z;
	}
}

static void
commit_camera(struct profile_parser *parser)
{
	int eye = camera_eye(parser->current_camera.id);
	if (eye >= 0) {
		parser->camera[eye] = parser->current_camera;
		parser->have_camera[eye] = true;
	}
}

static void
commit_camera_ext(struct profile_parser *parser)
{
	int eye = camera_eye(parser->current_camera_ext.id);
	if (eye >= 0) {
		parser->camera_ext[eye] = parser->current_camera_ext;
		parser->have_camera_ext[eye] = true;
	}
}

static void
commit_sensor_ext(struct profile_parser *parser)
{
	int eye = sensor_eye(parser->current_sensor_ext.id);
	if (eye >= 0) {
		parser->sensor_ext[eye] = parser->current_sensor_ext;
		parser->have_sensor_ext[eye] = true;
	}
}

static void
open_block(struct profile_parser *parser, const char *line)
{
	if (parser->depth >= MAX_BLOCK_DEPTH) {
		return;
	}
	size_t length = strcspn(line, " \t{");
	if (length == 0 || length >= BLOCK_NAME_SIZE) {
		return;
	}
	if (parser->depth == 0) {
		if (length == strlen("cameras") && strncmp(line, "cameras", length) == 0) {
			memset(&parser->current_camera, 0, sizeof(parser->current_camera));
		} else if (length == strlen("camera_extrinsics") && strncmp(line, "camera_extrinsics", length) == 0) {
			memset(&parser->current_camera_ext, 0, sizeof(parser->current_camera_ext));
		} else if (length == strlen("sensor_extrinsics") && strncmp(line, "sensor_extrinsics", length) == 0) {
			memset(&parser->current_sensor_ext, 0, sizeof(parser->current_sensor_ext));
		}
	}
	memcpy(parser->block[parser->depth], line, length);
	parser->block[parser->depth][length] = '\0';
	parser->depth++;
}

static void
close_block(struct profile_parser *parser)
{
	if (parser->depth == 0) {
		return;
	}
	if (parser->depth == 1) {
		if (block_is(parser, 0, "cameras")) {
			commit_camera(parser);
		} else if (block_is(parser, 0, "camera_extrinsics")) {
			commit_camera_ext(parser);
		} else if (block_is(parser, 0, "sensor_extrinsics")) {
			commit_sensor_ext(parser);
		}
	}
	parser->depth--;
	parser->block[parser->depth][0] = '\0';
}

static void
parse_camera_line(struct profile_parser *parser, const char *line)
{
	struct parsed_camera *camera = &parser->current_camera;
	if (parser->depth == 1) {
		if (key_string(line, "camera_id", camera->id, sizeof(camera->id))) {
			return;
		}
		if (key_number(line, "calibrated_width", &camera->width)) {
			camera->have |= CAM_WIDTH;
		} else if (key_number(line, "calibrated_height", &camera->height)) {
			camera->have |= CAM_HEIGHT;
		} else if (key_number(line, "max_valid_undistorted_radius", &camera->max_undistorted)) {
			camera->have |= CAM_MAX_UNDISTORTED;
		} else if (key_number(line, "max_valid_distorted_radius", &camera->max_distorted)) {
			camera->have |= CAM_MAX_DISTORTED;
		} else if (key_number(line, "rolling_shutter_readout_time_nanoseconds", &camera->rolling_time_ns)) {
			camera->have |= CAM_ROLLING_TIME;
		} else if (key_number(line, "camera_timestamp_alignment_nanoseconds",
		                      &camera->timestamp_alignment_ns)) {
			camera->have |= CAM_TIMESTAMP_ALIGNMENT;
		} else if (key_token(line, "exposure_timestamp_meaning", "BEGINNING_OF_EXPOSURE")) {
			camera->have |= CAM_EXPOSURE_BEGIN;
		} else if (key_token(line, "camera_timebase_trust_level", "TRUSTED_AS_BOOTTIME")) {
			camera->have |= CAM_TIMEBASE_BOOTTIME;
		} else if (key_token(line, "rolling_shutter_direction", "POS_Y_READOUT")) {
			camera->have |= CAM_ROLLING_POS_Y;
		} else if (key_token(line, "rolling_shutter_data_source", "IMAGE_METADATA")) {
			camera->rolling_from_metadata = true;
			camera->have |= CAM_ROLLING_SOURCE;
		} else if (key_token(line, "rolling_shutter_data_source", "DEVICE_PROFILE")) {
			camera->rolling_from_metadata = false;
			camera->have |= CAM_ROLLING_SOURCE;
		}
		return;
	}
	if (parser->depth == 2 && block_is(parser, 1, "calibrated_focal_length")) {
		if (key_number(line, "x", &camera->fx)) {
			camera->have |= CAM_FX;
		} else if (key_number(line, "y", &camera->fy)) {
			camera->have |= CAM_FY;
		}
	} else if (parser->depth == 2 && block_is(parser, 1, "calibrated_principal_point")) {
		if (key_number(line, "x", &camera->cx)) {
			camera->have |= CAM_CX;
		} else if (key_number(line, "y", &camera->cy)) {
			camera->have |= CAM_CY;
		}
	} else if (parser->depth == 2 && block_is(parser, 1, "distortion_kannala_brandt_fisheye")) {
		if (key_number(line, "k1", &camera->k[0])) {
			camera->have |= CAM_K1;
		} else if (key_number(line, "k2", &camera->k[1])) {
			camera->have |= CAM_K2;
		} else if (key_number(line, "k3", &camera->k[2])) {
			camera->have |= CAM_K3;
		} else if (key_number(line, "k4", &camera->k[3])) {
			camera->have |= CAM_K4;
		}
	} else if (parser->depth == 2 && block_is(parser, 1, "distortion_offset")) {
		if (key_number(line, "x", &camera->offset[0])) {
			camera->have |= CAM_OFFSET_X;
		} else if (key_number(line, "y", &camera->offset[1])) {
			camera->have |= CAM_OFFSET_Y;
		}
	} else if (parser->depth == 4 && block_is(parser, 1, "layered_distortion") &&
	           block_is(parser, 2, "curved_window") && block_is(parser, 3, "center")) {
		if (key_number(line, "x", &camera->window_center[0])) {
			camera->have |= CAM_WINDOW_X;
		} else if (key_number(line, "y", &camera->window_center[1])) {
			camera->have |= CAM_WINDOW_Y;
		} else if (key_number(line, "z", &camera->window_center[2])) {
			camera->have |= CAM_WINDOW_Z;
		}
	} else if (parser->depth == 3 && block_is(parser, 1, "layered_distortion") &&
	           block_is(parser, 2, "curved_window")) {
		if (key_number(line, "radius", &camera->window_radius)) {
			camera->have |= CAM_WINDOW_RADIUS;
		} else if (key_number(line, "thickness", &camera->window_thickness)) {
			camera->have |= CAM_WINDOW_THICKNESS;
		} else if (key_number(line, "refractive_index", &camera->window_index)) {
			camera->have |= CAM_WINDOW_INDEX;
		}
	}
}

static void
parse_camera_ext_line(struct profile_parser *parser, const char *line)
{
	struct parsed_transform *transform = &parser->current_camera_ext;
	if (parser->depth == 1) {
		if (key_token(line, "frame_id", "IMU_0")) {
			transform->from_imu = true;
		} else {
			key_string(line, "camera_id", transform->id, sizeof(transform->id));
		}
	} else if (parser->depth == 3 && block_is(parser, 1, "frame_t_camera") && block_is(parser, 2, "p")) {
		parse_xyz(line, transform->p, &transform->have_p, HAVE_X, HAVE_Y, HAVE_Z);
	} else if (parser->depth == 3 && block_is(parser, 1, "frame_t_camera") && block_is(parser, 2, "q")) {
		parse_quaternion(line, transform->q, &transform->have_q);
	}
}

static void
parse_sensor_ext_line(struct profile_parser *parser, const char *line)
{
	struct parsed_transform *transform = &parser->current_sensor_ext;
	if (parser->depth == 2 && block_is(parser, 1, "sensor_a_id")) {
		if (key_token(line, "frame_id", "IMU_0")) {
			transform->from_imu = true;
		}
	} else if (parser->depth == 2 && block_is(parser, 1, "sensor_b_id")) {
		key_string(line, "sensor_id", transform->id, sizeof(transform->id));
	} else if (parser->depth == 3 && block_is(parser, 1, "a_t_b") && block_is(parser, 2, "p")) {
		parse_xyz(line, transform->p, &transform->have_p, HAVE_X, HAVE_Y, HAVE_Z);
	} else if (parser->depth == 3 && block_is(parser, 1, "a_t_b") && block_is(parser, 2, "q")) {
		parse_quaternion(line, transform->q, &transform->have_q);
	}
}

static int
display_eye(const struct profile_parser *parser)
{
	if (block_is(parser, 1, "left_display_profile")) {
		return 0;
	}
	if (block_is(parser, 1, "right_display_profile")) {
		return 1;
	}
	return -1;
}

static void
parse_display_line(struct profile_parser *parser, const char *line)
{
	int eye = display_eye(parser);
	if (eye < 0) {
		return;
	}
	struct parsed_display_eye *display = &parser->display[eye];
	if (parser->depth == 2 && key_number(line, "device_ipd_in_mm", &display->ipd_mm)) {
		display->have_ipd = true;
		return;
	}

	int lut = find_block(parser, "lut_frame_t_eye");
	if (lut < 0) {
		return;
	}
	uint32_t index = (uint32_t)lut;
	if (parser->depth == index + 2 && block_is(parser, index + 1, "sensor_a_id")) {
		key_string(line, "sensor_id", display->sensor_id, sizeof(display->sensor_id));
	} else if (parser->depth == index + 3 && block_is(parser, index + 1, "a_t_b") &&
	           block_is(parser, index + 2, "p")) {
		parse_xyz(line, display->p, &display->have_p, HAVE_X, HAVE_Y, HAVE_Z);
	} else if (parser->depth == index + 3 && block_is(parser, index + 1, "a_t_b") &&
	           block_is(parser, index + 2, "q")) {
		parse_quaternion(line, display->q, &display->have_q);
	}
}

static int
parse_profile(FILE *file, struct profile_parser *parser)
{
	char storage[512];
	while (fgets(storage, sizeof(storage), file) != NULL) {
		char *line = trim(storage);
		if (*line == '\0' || *line == '#') {
			continue;
		}
		size_t length = strlen(line);
		if (line[length - 1] == '{') {
			open_block(parser, line);
			continue;
		}
		if (line[0] == '}') {
			close_block(parser);
			continue;
		}
		if (parser->depth == 0) {
			continue;
		}
		if (block_is(parser, 0, "cameras")) {
			parse_camera_line(parser, line);
		} else if (block_is(parser, 0, "camera_extrinsics")) {
			parse_camera_ext_line(parser, line);
		} else if (block_is(parser, 0, "sensor_extrinsics")) {
			parse_sensor_ext_line(parser, line);
		} else if (block_is(parser, 0, "display_profile_v2")) {
			parse_display_line(parser, line);
		}
	}
	return ferror(file) ? -1 : 0;
}

static bool
normalize_quaternion(double q[4])
{
	double length = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if (!isfinite(length) || length < 0.5) {
		return false;
	}
	for (uint32_t i = 0; i < 4; i++) {
		q[i] /= length;
	}
	return true;
}

static void
quat_conjugate(const double q[4], double out[4])
{
	out[0] = q[0];
	out[1] = -q[1];
	out[2] = -q[2];
	out[3] = -q[3];
}

static void
quat_rotate(const double q[4], const double value[3], double out[3])
{
	double u[3] = {q[1], q[2], q[3]};
	double uv[3] = {
	    u[1] * value[2] - u[2] * value[1],
	    u[2] * value[0] - u[0] * value[2],
	    u[0] * value[1] - u[1] * value[0],
	};
	double uuv[3] = {
	    u[1] * uv[2] - u[2] * uv[1],
	    u[2] * uv[0] - u[0] * uv[2],
	    u[0] * uv[1] - u[1] * uv[0],
	};
	for (uint32_t i = 0; i < 3; i++) {
		out[i] = value[i] + 2.0 * (q[0] * uv[i] + uuv[i]);
	}
}

static void
quat_matrix(const double q[4], float out[3][3])
{
	double w = q[0];
	double x = q[1];
	double y = q[2];
	double z = q[3];
	out[0][0] = (float)(1.0 - 2.0 * (y * y + z * z));
	out[0][1] = (float)(2.0 * (x * y - z * w));
	out[0][2] = (float)(2.0 * (x * z + y * w));
	out[1][0] = (float)(2.0 * (x * y + z * w));
	out[1][1] = (float)(1.0 - 2.0 * (x * x + z * z));
	out[1][2] = (float)(2.0 * (y * z - x * w));
	out[2][0] = (float)(2.0 * (x * z - y * w));
	out[2][1] = (float)(2.0 * (y * z + x * w));
	out[2][2] = (float)(1.0 - 2.0 * (x * x + y * y));
}

static struct point2
point_add(struct point2 a, struct point2 b)
{
	return (struct point2){a.x + b.x, a.y + b.y};
}

static struct point2
point_scale(struct point2 value, double scale)
{
	return (struct point2){value.x * scale, value.y * scale};
}

static double
point_dot(struct point2 a, struct point2 b)
{
	return a.x * b.x + a.y * b.y;
}

static bool
sphere_intersection(
    const struct window2 *window, struct point2 origin, struct point2 direction, double radius, struct point2 *out)
{
	// In this axial plane the sphere centre is (-distance, 0).
	struct point2 relative = {
	    origin.x + window->center_distance,
	    origin.y,
	};
	double b = point_dot(relative, direction);
	double c = point_dot(relative, relative) - radius * radius;
	double discriminant = b * b - c;
	if (discriminant < 0.0) {
		return false;
	}
	double root = sqrt(discriminant);
	double near_t = -b - root;
	double far_t = -b + root;
	double t = near_t > 1e-9 ? near_t : far_t;
	if (!(t > 1e-9) || !isfinite(t)) {
		return false;
	}
	*out = point_add(origin, point_scale(direction, t));
	return true;
}

static bool
refract_direction(
    struct point2 incident, struct point2 normal_to_next_medium, double from_index, double to_index, struct point2 *out)
{
	double eta = from_index / to_index;
	double normal_component = point_dot(incident, normal_to_next_medium);
	struct point2 tangent = {
	    incident.x - normal_component * normal_to_next_medium.x,
	    incident.y - normal_component * normal_to_next_medium.y,
	};
	double k = 1.0 - eta * eta * point_dot(tangent, tangent);
	if (k < 0.0) {
		return false;
	}
	double next_normal = sqrt(k);
	out->x = eta * tangent.x + next_normal * normal_to_next_medium.x;
	out->y = eta * tangent.y + next_normal * normal_to_next_medium.y;
	double length = hypot(out->x, out->y);
	if (!(length > 0.0) || !isfinite(length)) {
		return false;
	}
	out->x /= length;
	out->y /= length;
	return true;
}

static bool
trace_window(const struct window2 *window,
             struct point2 camera_ray,
             struct point2 *outer_point,
             struct point2 *outer_ray)
{
	struct point2 inner_point;
	if (!sphere_intersection(window, (struct point2){0.0, 0.0}, camera_ray, window->inner_radius, &inner_point)) {
		return false;
	}
	struct point2 inner_normal = {
	    (inner_point.x + window->center_distance) / window->inner_radius,
	    inner_point.y / window->inner_radius,
	};
	struct point2 glass_ray;
	if (!refract_direction(camera_ray, inner_normal, 1.0, window->refractive_index, &glass_ray) ||
	    !sphere_intersection(window, inner_point, glass_ray, window->outer_radius, outer_point)) {
		return false;
	}
	struct point2 outer_normal = {
	    (outer_point->x + window->center_distance) / window->outer_radius,
	    outer_point->y / window->outer_radius,
	};
	return refract_direction(glass_ray, outer_normal, window->refractive_index, 1.0, outer_ray);
}

static bool
curve_error(const struct window2 *window, double cosine, double alpha, double *out_error)
{
	double sine = sqrt(fmax(0.0, 1.0 - cosine * cosine));
	struct point2 camera_ray = {cosine + alpha, sine};
	double length = hypot(camera_ray.x, camera_ray.y);
	if (!(length > 0.0)) {
		return false;
	}
	camera_ray.x /= length;
	camera_ray.y /= length;

	struct point2 outer_point;
	struct point2 outer_ray;
	if (!trace_window(window, camera_ray, &outer_point, &outer_ray)) {
		return false;
	}
	struct point2 target = {
	    CURVED_WINDOW_OPTIMAL_DEPTH_M,
	    CURVED_WINDOW_OPTIMAL_DEPTH_M * sine / cosine,
	};
	struct point2 to_target = {
	    target.x - outer_point.x,
	    target.y - outer_point.y,
	};
	*out_error = outer_ray.x * to_target.y - outer_ray.y * to_target.x;
	return isfinite(*out_error);
}

static bool
solve_curve_alpha(const struct window2 *window, double cosine, double *out_alpha)
{
	if (cosine >= 1.0 - 1e-12) {
		*out_alpha = 0.0;
		return true;
	}

	/*
	 * The solution is smooth and close to zero, but scan a conservative
	 * interval so unusual per-unit sphere centres remain supported.
	 */
	const uint32_t scan_steps = 400;
	bool have_previous = false;
	double previous_alpha = 0.0;
	double previous_error = 0.0;
	bool have_bracket = false;
	double low = 0.0;
	double high = 0.0;
	double low_error = 0.0;
	double best_distance = INFINITY;

	for (uint32_t i = 0; i <= scan_steps; i++) {
		double alpha = -0.5 + (double)i / (double)scan_steps;
		double error;
		if (!curve_error(window, cosine, alpha, &error)) {
			have_previous = false;
			continue;
		}
		if (error == 0.0) {
			*out_alpha = alpha;
			return true;
		}
		if (have_previous && signbit(error) != signbit(previous_error)) {
			double distance = fabs(0.5 * (alpha + previous_alpha));
			if (distance < best_distance) {
				best_distance = distance;
				low = previous_alpha;
				high = alpha;
				low_error = previous_error;
				have_bracket = true;
			}
		}
		previous_alpha = alpha;
		previous_error = error;
		have_previous = true;
	}
	if (!have_bracket) {
		return false;
	}

	for (uint32_t iteration = 0; iteration < 48; iteration++) {
		double middle = 0.5 * (low + high);
		double error;
		if (!curve_error(window, cosine, middle, &error)) {
			return false;
		}
		if (signbit(error) == signbit(low_error)) {
			low = middle;
			low_error = error;
		} else {
			high = middle;
		}
	}
	*out_alpha = 0.5 * (low + high);
	return true;
}

static bool
generate_curve_lut(const struct parsed_camera *parsed, struct xrt_passthrough_camera_calibration *camera)
{
	double distance = sqrt(parsed->window_center[0] * parsed->window_center[0] +
	                       parsed->window_center[1] * parsed->window_center[1] +
	                       parsed->window_center[2] * parsed->window_center[2]);
	double inner = parsed->window_radius - 0.5 * parsed->window_thickness;
	double outer = parsed->window_radius + 0.5 * parsed->window_thickness;
	if (!(distance > 0.0) || !(inner > distance) || !(outer > inner) || parsed->window_index < 1.0) {
		return false;
	}
	for (uint32_t i = 0; i < 3; i++) {
		camera->window_axis[i] = (float)(-parsed->window_center[i] / distance);
	}

	struct window2 window = {
	    .center_distance = distance,
	    .inner_radius = inner,
	    .outer_radius = outer,
	    .refractive_index = parsed->window_index,
	};
	const double step = 0.99 / (double)(XRT_PASSTHROUGH_CURVE_LUT_SIZE - 1);
	for (uint32_t i = 0; i < XRT_PASSTHROUGH_CURVE_LUT_SIZE; i++) {
		double cosine = 0.01 + step * (double)i;
		double alpha;
		if (!solve_curve_alpha(&window, cosine, &alpha)) {
			return false;
		}
		camera->curved_window_lut[i] = (float)alpha;
	}
	return true;
}

static bool
finish_profile(const struct profile_parser *parser, struct xrt_passthrough_calibration *out)
{
	double eye_position[CALIBRATION_VIEWS][3];
	for (uint32_t eye = 0; eye < CALIBRATION_VIEWS; eye++) {
		const struct parsed_camera *camera = &parser->camera[eye];
		const struct parsed_transform *camera_ext = &parser->camera_ext[eye];
		const struct parsed_transform *sensor_ext = &parser->sensor_ext[eye];
		const struct parsed_display_eye *display = &parser->display[eye];
		if (!parser->have_camera[eye] || camera->have != CAM_REQUIRED || !parser->have_camera_ext[eye] ||
		    !camera_ext->from_imu || camera_ext->have_p != HAVE_XYZ || camera_ext->have_q != HAVE_WXYZ ||
		    !parser->have_sensor_ext[eye] || !sensor_ext->from_imu || sensor_ext->have_p != HAVE_XYZ ||
		    sensor_ext->have_q != HAVE_WXYZ || display->sensor_id[0] == '\0' || display->have_p != HAVE_XYZ ||
		    display->have_q != HAVE_WXYZ || strcmp(display->sensor_id, sensor_ext->id) != 0) {
			return false;
		}
		if (camera->width < 1.0 || camera->height < 1.0 || camera->width > UINT32_MAX ||
		    camera->height > UINT32_MAX || camera->fx <= 0.0 || camera->fy <= 0.0 ||
		    camera->max_undistorted <= 0.0 || camera->max_distorted <= 0.0 || camera->rolling_time_ns < 0.0 ||
		    camera->rolling_time_ns > (double)UINT64_MAX ||
		    camera->timestamp_alignment_ns < (double)INT64_MIN ||
		    camera->timestamp_alignment_ns > (double)INT64_MAX) {
			return false;
		}

		double camera_q[4];
		double sensor_q[4];
		double eye_q[4];
		memcpy(camera_q, camera_ext->q, sizeof(camera_q));
		memcpy(sensor_q, sensor_ext->q, sizeof(sensor_q));
		memcpy(eye_q, display->q, sizeof(eye_q));
		if (!normalize_quaternion(camera_q) || !normalize_quaternion(sensor_q) ||
		    !normalize_quaternion(eye_q)) {
			return false;
		}

		/*
		 * a_t_b.q rotates coordinates from a to b. Position p is the
		 * origin of b expressed in a, so module-local eye translation
		 * is rotated by conjugate(q_imu_module) into IMU coordinates.
		 */
		double module_to_imu[4];
		double eye_offset[3];
		quat_conjugate(sensor_q, module_to_imu);
		quat_rotate(module_to_imu, display->p, eye_offset);
		for (uint32_t i = 0; i < 3; i++) {
			eye_position[eye][i] = sensor_ext->p[i] + eye_offset[i];
		}

		struct xrt_passthrough_camera_calibration *result = &out->views[eye];
		result->width = (uint32_t)camera->width;
		result->height = (uint32_t)camera->height;
		result->fx = (float)camera->fx;
		result->fy = (float)camera->fy;
		result->cx = (float)camera->cx;
		result->cy = (float)camera->cy;
		for (uint32_t i = 0; i < 4; i++) {
			result->k[i] = (float)camera->k[i];
		}
		result->max_valid_undistorted_radius = (float)camera->max_undistorted;
		result->max_valid_distorted_radius = (float)camera->max_distorted;
		result->distortion_offset[0] = (float)camera->offset[0];
		result->distortion_offset[1] = (float)camera->offset[1];
		result->rolling_shutter_readout_ns = (uint64_t)camera->rolling_time_ns;
		result->timestamp_alignment_ns = (int64_t)camera->timestamp_alignment_ns;
		result->rolling_shutter_from_metadata = camera->rolling_from_metadata;
		quat_matrix(camera_q, result->imu_to_camera);
		for (uint32_t i = 0; i < 3; i++) {
			result->camera_position[i] = (float)camera_ext->p[i];
			result->eye_position[i] = (float)eye_position[eye][i];
		}
		if (!generate_curve_lut(camera, result)) {
			return false;
		}
	}

	out->view_count = CALIBRATION_VIEWS;
	for (uint32_t i = 0; i < 3; i++) {
		out->eye_midpoint[i] = (float)(0.5 * (eye_position[0][i] + eye_position[1][i]));
	}
	if (parser->display[0].have_ipd && parser->display[1].have_ipd) {
		out->calibration_ipd_mm = (float)(0.5 * (parser->display[0].ipd_mm + parser->display[1].ipd_mm));
	}
	return true;
}

static bool
load_one(struct xrt_passthrough_calibration *out, const char *path)
{
	FILE *file = fopen(path, "r");
	if (file == NULL) {
		return false;
	}
	struct profile_parser parser = {0};
	int parsed = parse_profile(file, &parser);
	fclose(file);
	if (parsed != 0 || !finish_profile(&parser, out)) {
		fprintf(stderr,
		        "galaxyxr: %s does not contain a complete EFS "
		        "rgb-*_curved calibration\n",
		        path);
		memset(out, 0, sizeof(*out));
		return false;
	}
	snprintf(out->source_path, sizeof(out->source_path), "%s", path);
	return true;
}

int
galaxyxr_passthrough_calibration_load(struct xrt_passthrough_calibration *out, const char *path)
{
	memset(out, 0, sizeof(*out));
	if (path != NULL && load_one(out, path)) {
		return 0;
	}
	for (uint32_t i = 0; i < ARRAY_SIZE(default_profile_paths); i++) {
		if (path != NULL && strcmp(path, default_profile_paths[i]) == 0) {
			continue;
		}
		if (load_one(out, default_profile_paths[i])) {
			return 0;
		}
	}
	fprintf(stderr, "galaxyxr: no usable per-unit EFS passthrough calibration\n");
	return -1;
}
