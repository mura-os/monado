// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR HMD device, 3DoF from the SSC IMU.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_interface.h"
#include "galaxyxr_hmd_input.h"
#include "galaxyxr_profile.h"
#include "galaxyxr_ssc.h"

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
#include "galaxyxr_eye_tracking.h"
#endif

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#ifdef XRT_FEATURE_TITAN_PASSTHROUGH
#include "galaxyxr_passthrough.h"
#include "xrt/xrt_passthrough.h"
#endif

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_clock_tracking.h"
#include "math/m_imu_xio_ahrs.h"
#include "math/m_mathinclude.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_linux.h"
#include "util/u_logging.h"
#include "util/u_var.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#define GXR_EYE_W 3552
#define GXR_EYE_H 3840
#define GXR_PANEL_VTOTAL 3888
#define GXR_CLOCK_TRACKER_WINDOW_SAMPLES 64

enum galaxyxr_sensor_event
{
	GXR_SENSOR_EVENT_CONTROL,
	GXR_SENSOR_EVENT_SSC,
	GXR_SENSOR_EVENT_POWER,
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	GXR_SENSOR_EVENT_EYE_TRACKING,
#endif
};

DEBUG_GET_ONCE_LOG_OPTION(galaxyxr_log, "GALAXYXR_LOG", U_LOGGING_INFO)
// Panel refresh, shared with the compositor backend's mode choice (the
// panels do 90, 72 and 60).
DEBUG_GET_ONCE_NUM_OPTION(galaxyxr_hz, "XRT_COMPOSITOR_GALAXYXR_HZ", 90)
DEBUG_GET_ONCE_NUM_OPTION(galaxyxr_imu_rate, "GALAXYXR_IMU_RATE", 200)
DEBUG_GET_ONCE_OPTION(galaxyxr_imu_axes, "GALAXYXR_IMU_AXES", "x,y,z")
// "profile" = the Android XR device profile LUT with its per-color CAC,
// "profile-mono" = green geometry only (default), "none" = identity mesh.
DEBUG_GET_ONCE_OPTION(galaxyxr_distortion, "GALAXYXR_DISTORTION", "profile-mono")
DEBUG_GET_ONCE_OPTION(galaxyxr_profile_path, "GALAXYXR_PROFILE_PATH", NULL)
// Factory IMU intrinsics from the efs profile plus the SSC online gyro bias;
// the raw SSC streams apply no calibration at all (registry fac_cal is zero).
DEBUG_GET_ONCE_BOOL_OPTION(galaxyxr_imu_cal, "GALAXYXR_IMU_CAL", true)

#define GXR_TRACE(d, ...) U_LOG_XDEV_IFL_T(&d->base, d->log_level, __VA_ARGS__)
#define GXR_DEBUG(d, ...) U_LOG_XDEV_IFL_D(&d->base, d->log_level, __VA_ARGS__)
#define GXR_INFO(d, ...) U_LOG_XDEV_IFL_I(&d->base, d->log_level, __VA_ARGS__)
#define GXR_WARN(d, ...) U_LOG_XDEV_IFL_W(&d->base, d->log_level, __VA_ARGS__)
#define GXR_ERROR(d, ...) U_LOG_XDEV_IFL_E(&d->base, d->log_level, __VA_ARGS__)

struct galaxyxr_hmd
{
	struct xrt_device base;
	enum u_logging_level log_level;

	struct os_thread_helper sensor_thread;
	struct galaxyxr_ssc ssc;
	bool ssc_opened;

	//! Device axis = sign[i] * imu axis src[i].
	int axis_src[3];
	float axis_sign[3];

	float ipd_m;
	float ipd_logged_mm;
	bool cac_enabled;

	struct galaxyxr_profile_eye profile[2];
	bool profile_active;
	//! Calibrated per-eye view poses (v2 profile), mid-eye centered.
	bool have_view_pose;
	struct xrt_pose view_pose[2];

	struct os_mutex mutex;
	struct m_imu_xio_ahrs fusion;
	struct xrt_space_relation relation;
	//! Correlates local CLOCK_MONOTONIC with the remote hardware QTimer.
	struct m_clock_windowed_skew_tracker *qtimer_clock_tracker;
	//! Fused device relations indexed directly by the shared hardware QTimer.
	struct m_relation_history *qtimer_relation_history;

	bool have_accel;
	struct xrt_vec3 accel;
	struct xrt_vec3 gyro;
	struct xrt_vec3 raw_accel;
	struct xrt_vec3 raw_gyro;

	//! Factory IMU intrinsics for the head IMU from the efs device profile.
	struct galaxyxr_imu_cal imu_cal;
	//! The gyro bias being subtracted: zero, then the factory default,
	//! then the live SSC gyro_cal estimates once they arrive.
	struct xrt_vec3 gyro_bias;

	struct galaxyxr_hmd_input input;

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	struct galaxyxr_eye_tracking eye_tracking;
#endif

	xrt_atomic_s32_t sensor_stop_requested;
	//! Wakes the sensor thread after a control-state change.
	int sensor_control_fd;
};

static inline struct galaxyxr_hmd *
galaxyxr_hmd(struct xrt_device *xdev)
{
	return (struct galaxyxr_hmd *)xdev;
}


/*
 *
 * IMU handling.
 *
 */

static bool
parse_axis_map(const char *str, int *out_src, float *out_sign)
{
	int i = 0;
	for (const char *p = str; *p != '\0' && i < 3; p++) {
		float sign = 1.0f;
		if (*p == ',' || *p == ' ') {
			continue;
		}
		if (*p == '+' || *p == '-') {
			sign = *p == '-' ? -1.0f : 1.0f;
			p++;
		}
		if (*p < 'x' || *p > 'z') {
			return false;
		}
		out_src[i] = *p - 'x';
		out_sign[i] = sign;
		i++;
	}
	return i == 3;
}

static struct xrt_vec3
remap_vec(const struct galaxyxr_hmd *hmd, const float v[3])
{
	struct xrt_vec3 out;
	out.x = hmd->axis_sign[0] * v[hmd->axis_src[0]];
	out.y = hmd->axis_sign[1] * v[hmd->axis_src[1]];
	out.z = hmd->axis_sign[2] * v[hmd->axis_src[2]];
	return out;
}

// The calibration lives in the SSC stream frame, before any axis remap.
static void
apply_imu_cal(const float in[3], const struct xrt_vec3 *bias, const struct xrt_vec3 *scale, float out[3])
{
	out[0] = (in[0] - bias->x) * scale->x;
	out[1] = (in[1] - bias->y) * scale->y;
	out[2] = (in[2] - bias->z) * scale->z;
}

static void
sample_cb(void *ud, const struct galaxyxr_ssc_sample *sample)
{
	struct galaxyxr_hmd *hmd = ud;
	int64_t receive_monotonic_ns = os_monotonic_get_ns();

	if (sample->type == GALAXYXR_SSC_IPD) {
		// Per-side lens travel in um on top of the 28.5 mm per-side
		// hardware minimum (same mapping as the Android actuator HAL).
		float mm = 57.0f + (sample->v[0] + sample->v[1]) / 1000.0f;
		if (mm < 45.0f || mm > 85.0f) {
			return;
		}
		os_mutex_lock(&hmd->mutex);
		hmd->ipd_m = mm / 1000.0f;
		if (hmd->have_view_pose) {
			hmd->view_pose[0].position.x = mm * -0.0005f;
			hmd->view_pose[1].position.x = mm * 0.0005f;
		}
		bool log = fabsf(mm - hmd->ipd_logged_mm) > 0.05f;
		if (log) {
			hmd->ipd_logged_mm = mm;
		}
		os_mutex_unlock(&hmd->mutex);
		if (log && hmd->profile[0].ipd_mm > 0.0f && fabsf(mm - hmd->profile[0].ipd_mm) > 1.0f) {
			GXR_INFO(hmd, "IPD sensor: %.2f mm (L %.1f R %.1f um), display calibrated at %.0f mm",
			         mm, sample->v[0], sample->v[1], hmd->profile[0].ipd_mm);
		} else if (log) {
			GXR_INFO(hmd, "IPD sensor: %.2f mm (L %.1f R %.1f um)", mm, sample->v[0], sample->v[1]);
		}
		return;
	}

	if (sample->type == GALAXYXR_SSC_PROX) {
		galaxyxr_hmd_input_handle_proximity(&hmd->input, sample->v[0], sample->v[1]);
		return;
	}

	if (sample->type == GALAXYXR_SSC_GYRO_CAL) {
		// Only ever touched from this thread; the pose queries never read it.
		hmd->gyro_bias = (struct xrt_vec3){sample->v[0], sample->v[1], sample->v[2]};
		GXR_INFO(hmd, "SSC gyro bias estimate: (%+.2f, %+.2f, %+.2f) mrad/s",
		         sample->v[0] * 1000.0f, sample->v[1] * 1000.0f, sample->v[2] * 1000.0f);
		return;
	}

	static const struct xrt_vec3 vec_one = {1.0f, 1.0f, 1.0f};
	float c[3] = {sample->v[0], sample->v[1], sample->v[2]};

	os_mutex_lock(&hmd->mutex);
	if (sample->type == GALAXYXR_SSC_ACCEL) {
		hmd->raw_accel = (struct xrt_vec3){sample->v[0], sample->v[1], sample->v[2]};
		if (debug_get_bool_option_galaxyxr_imu_cal() && hmd->imu_cal.valid) {
			apply_imu_cal(sample->v, &hmd->imu_cal.accel_bias, &hmd->imu_cal.accel_scale, c);
		}
		hmd->accel = remap_vec(hmd, c);
		hmd->have_accel = true;
		os_mutex_unlock(&hmd->mutex);
		return;
	}

	hmd->raw_gyro = (struct xrt_vec3){sample->v[0], sample->v[1], sample->v[2]};
	if (debug_get_bool_option_galaxyxr_imu_cal()) {
		const struct xrt_vec3 *scale = hmd->imu_cal.valid ? &hmd->imu_cal.gyro_scale : &vec_one;
		apply_imu_cal(sample->v, &hmd->gyro_bias, scale, c);
	}
	hmd->gyro = remap_vec(hmd, c);
	m_clock_windowed_skew_tracker_push(hmd->qtimer_clock_tracker, receive_monotonic_ns, sample->timestamp_ns);
	if (hmd->have_accel) {
		// The AHRS wants accel in g and gyro in deg/s; the SSC delivers
		// m/s^2 and rad/s.
		struct xrt_vec3 accel_g = {
		    hmd->accel.x / MATH_GRAVITY_M_S2,
		    hmd->accel.y / MATH_GRAVITY_M_S2,
		    hmd->accel.z / MATH_GRAVITY_M_S2,
		};
		struct xrt_vec3 gyro_dps = {
		    RAD_TO_DEG(hmd->gyro.x),
		    RAD_TO_DEG(hmd->gyro.y),
		    RAD_TO_DEG(hmd->gyro.z),
		};
		m_imu_xio_ahrs_update(&hmd->fusion, (uint64_t)sample->timestamp_ns, &accel_g, &gyro_dps);

		struct xrt_vec3 angular_velocity_ws;
		math_quat_rotate_vec3(&hmd->fusion.rot, &hmd->gyro, &angular_velocity_ws);

		hmd->relation.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                               XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
		hmd->relation.pose.orientation = hmd->fusion.rot;
		hmd->relation.angular_velocity = angular_velocity_ws;
		m_relation_history_push(hmd->qtimer_relation_history, &hmd->relation, sample->timestamp_ns);
	}
	os_mutex_unlock(&hmd->mutex);
}

static int
sensor_epoll_add(int epoll_fd, int fd, enum galaxyxr_sensor_event source);

static void
sensor_control_drain(struct galaxyxr_hmd *hmd)
{
	eventfd_t value;
	int ret;
	do {
		ret = eventfd_read(hmd->sensor_control_fd, &value);
	} while (ret < 0 && errno == EINTR);
	if (ret < 0 && errno != EAGAIN) {
		GXR_WARN(hmd, "Could not read sensor control eventfd: %s", strerror(errno));
	}
}

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
static void
sensor_eye_tracking_close(struct galaxyxr_hmd *hmd, int epoll_fd, int *eye_fd)
{
	if (*eye_fd < 0) {
		return;
	}

	if (epoll_ctl(epoll_fd, EPOLL_CTL_DEL, *eye_fd, NULL) < 0 && errno != ENOENT) {
		GXR_WARN(hmd, "Could not remove eye tracking from sensor epoll: %s", strerror(errno));
	}
	galaxyxr_eye_tracking_close(&hmd->eye_tracking);
	*eye_fd = -1;
}

static void
sensor_eye_tracking_sync(struct galaxyxr_hmd *hmd, int epoll_fd, int *eye_fd)
{
	bool enabled = galaxyxr_eye_tracking_is_enabled(&hmd->eye_tracking);
	if (enabled == (*eye_fd >= 0)) {
		return;
	}

	if (!enabled) {
		sensor_eye_tracking_close(hmd, epoll_fd, eye_fd);
		return;
	}

	*eye_fd = galaxyxr_eye_tracking_open(&hmd->eye_tracking, &hmd->base, &hmd->log_level);
	if (*eye_fd < 0) {
		galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
		return;
	}

	int ret = sensor_epoll_add(epoll_fd, *eye_fd, GXR_SENSOR_EVENT_EYE_TRACKING);
	if (ret < 0) {
		GXR_ERROR(hmd, "Could not add eye tracking to sensor epoll: %s", strerror(-ret));
		sensor_eye_tracking_close(hmd, epoll_fd, eye_fd);
		galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
	}
}
#endif

static void *
sensor_thread_fn(void *ptr)
{
	struct galaxyxr_hmd *hmd = ptr;
	int epoll_fd = -1;
	int power_fd = -1;
	bool power_watched = false;
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	int eye_fd = -1;
#endif

	epoll_fd = epoll_create1(EPOLL_CLOEXEC);
	if (epoll_fd < 0) {
		GXR_ERROR(hmd, "Could not create sensor epoll: %s", strerror(errno));
		goto out;
	}

	int ret = sensor_epoll_add(epoll_fd, hmd->sensor_control_fd, GXR_SENSOR_EVENT_CONTROL);
	if (ret < 0) {
		GXR_ERROR(hmd, "Could not add control eventfd to sensor epoll: %s", strerror(-ret));
		goto out;
	}
	ret = sensor_epoll_add(epoll_fd, hmd->ssc.fd, GXR_SENSOR_EVENT_SSC);
	if (ret < 0) {
		GXR_ERROR(hmd, "Could not add SSC fd to sensor epoll: %s", strerror(-ret));
		goto out;
	}

	power_fd = galaxyxr_hmd_input_open_power_button(&hmd->input);
	if (power_fd >= 0) {
		ret = sensor_epoll_add(epoll_fd, power_fd, GXR_SENSOR_EVENT_POWER);
		if (ret < 0) {
			GXR_WARN(hmd, "Could not add power button to sensor epoll: %s", strerror(-ret));
			galaxyxr_hmd_input_reset_power_button(&hmd->input);
		} else {
			power_watched = true;
		}
	}

	u_linux_try_to_set_realtime_priority_on_thread(hmd->log_level, "Galaxy XR IMU");

	struct epoll_event events[4];
	for (;;) {
		int event_count = epoll_wait(epoll_fd, events, ARRAY_SIZE(events), -1);
		if (event_count < 0) {
			if (errno == EINTR) {
				continue;
			}
			GXR_ERROR(hmd, "Sensor epoll wait failed: %s", strerror(errno));
			goto out;
		}

		// Control changes take priority over hardware events in the batch.
		for (int i = 0; i < event_count; i++) {
			if (events[i].data.u32 != GXR_SENSOR_EVENT_CONTROL) {
				continue;
			}
			sensor_control_drain(hmd);
			if (xrt_atomic_s32_load(&hmd->sensor_stop_requested) != 0) {
				goto out;
			}
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
			sensor_eye_tracking_sync(hmd, epoll_fd, &eye_fd);
#endif
		}

		for (int i = 0; i < event_count; i++) {
			switch (events[i].data.u32) {
			case GXR_SENSOR_EVENT_SSC: {
				if ((events[i].events & (EPOLLERR | EPOLLHUP)) != 0) {
					GXR_ERROR(hmd, "SSC event fd reported an error");
					goto out;
				}
				int ret = galaxyxr_ssc_dispatch_ready(&hmd->ssc, sample_cb, hmd);
				if (ret < 0) {
					GXR_ERROR(hmd, "SSC dispatch failed: %d", ret);
					goto out;
				}
				break;
			}
			case GXR_SENSOR_EVENT_POWER: {
				bool healthy = galaxyxr_hmd_input_poll_power_button(&hmd->input, power_fd);
				if (healthy && (events[i].events & (EPOLLERR | EPOLLHUP)) != 0) {
					GXR_WARN(hmd, "Power button event fd reported an error");
					healthy = false;
				}
				if (!healthy && power_watched) {
					if (epoll_ctl(epoll_fd, EPOLL_CTL_DEL, power_fd, NULL) < 0 && errno != ENOENT) {
						GXR_ERROR(hmd, "Could not remove power button from sensor epoll: %s", strerror(errno));
						goto out;
					}
					power_watched = false;
					galaxyxr_hmd_input_reset_power_button(&hmd->input);
				}
				break;
			}
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
			case GXR_SENSOR_EVENT_EYE_TRACKING:
				if (eye_fd < 0) {
					break;
				}
				if ((events[i].events & (EPOLLERR | EPOLLHUP)) != 0) {
					GXR_ERROR(hmd, "Eye tracking event fd reported an error");
					galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
					sensor_eye_tracking_close(hmd, epoll_fd, &eye_fd);
					break;
				}
				if (!galaxyxr_eye_tracking_process_events(&hmd->eye_tracking, &hmd->base,
				                                          &hmd->log_level)) {
					galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
					sensor_eye_tracking_close(hmd, epoll_fd, &eye_fd);
				}
				break;
#endif
			case GXR_SENSOR_EVENT_CONTROL: break;
			default: GXR_WARN(hmd, "Unknown sensor epoll event: %u", events[i].data.u32); break;
			}
		}
	}

out:
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
	sensor_eye_tracking_close(hmd, epoll_fd, &eye_fd);
#endif
	if (power_fd >= 0) {
		close(power_fd);
	}
	if (epoll_fd >= 0) {
		close(epoll_fd);
	}
	galaxyxr_hmd_input_reset_power_button(&hmd->input);
	return NULL;
}

static int
sensor_epoll_add(int epoll_fd, int fd, enum galaxyxr_sensor_event source)
{
	struct epoll_event event = {
	    .events = EPOLLIN,
	    .data.u32 = source,
	};
	if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event) < 0) {
		return -errno;
	}
	return 0;
}

static int
sensor_control_event_init(struct galaxyxr_hmd *hmd)
{
	hmd->sensor_control_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (hmd->sensor_control_fd < 0) {
		return -errno;
	}
	return 0;
}

static void
sensor_thread_notify(struct galaxyxr_hmd *hmd)
{
	if (hmd->sensor_control_fd < 0) {
		return;
	}

	eventfd_t value = 1;
	int ret;
	do {
		ret = eventfd_write(hmd->sensor_control_fd, value);
	} while (ret < 0 && errno == EINTR);
	if (ret < 0 && errno != EAGAIN) {
		GXR_WARN(hmd, "Could not wake sensor thread: %s", strerror(errno));
	}
}

static void
sensor_thread_request_stop(struct galaxyxr_hmd *hmd)
{
	xrt_atomic_s32_store(&hmd->sensor_stop_requested, 1);
	sensor_thread_notify(hmd);
}


/*
 *
 * Device entry points.
 *
 */

static void
galaxyxr_hmd_destroy(struct xrt_device *xdev)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);

	if (hmd->sensor_thread.initialized) {
		sensor_thread_request_stop(hmd);
		os_thread_helper_destroy(&hmd->sensor_thread);
	}
	if (hmd->sensor_control_fd >= 0) {
		close(hmd->sensor_control_fd);
	}
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	galaxyxr_eye_tracking_destroy(&hmd->eye_tracking);
#endif
	if (hmd->ssc_opened) {
		galaxyxr_ssc_close(&hmd->ssc);
	}

#ifdef XRT_FEATURE_TITAN_PASSTHROUGH
	xrt_passthrough_stream_destroy(&hmd->base.passthrough);
#endif
	m_relation_history_destroy(&hmd->qtimer_relation_history);
	if (hmd->qtimer_clock_tracker != NULL) {
		m_clock_windowed_skew_tracker_destroy(hmd->qtimer_clock_tracker);
	}

	os_mutex_destroy(&hmd->mutex);

	galaxyxr_profile_free(&hmd->profile[0]);
	galaxyxr_profile_free(&hmd->profile[1]);

	u_var_remove_root(hmd);
	u_device_free(&hmd->base);
}

static xrt_result_t
galaxyxr_hmd_update_inputs(struct xrt_device *xdev)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);
	return galaxyxr_hmd_input_update(&hmd->input);
}

static xrt_result_t
galaxyxr_hmd_get_compositor_info(struct xrt_device *xdev,
                                 const struct xrt_device_compositor_mode *mode,
                                 struct xrt_device_compositor_info *out_info)
{
	/*
	 * All three panel modes (90, 72 and 60 Hz) have 3840 active lines and
	 * 3888 total lines. The two DSI panels scan top-to-bottom in lockstep.
	 */
	*out_info = (struct xrt_device_compositor_info){
	    .scanout_direction = XRT_SCANOUT_DIRECTION_TOP_TO_BOTTOM,
	    .scanout_time_ns = (mode->frame_interval_ns * GXR_EYE_H) / GXR_PANEL_VTOTAL,
	};

	(void)xdev;
	return XRT_SUCCESS;
}

static void
galaxyxr_hmd_get_head_relation(struct galaxyxr_hmd *hmd,
                               int64_t at_timestamp_ns,
                               struct xrt_space_relation *out_relation)
{
	struct xrt_space_relation fallback;
	int64_t query_qtimer_ns = 0;
	os_mutex_lock(&hmd->mutex);
	fallback = hmd->relation;
	/*
	 * In clock-tracker terminology local is host CLOCK_MONOTONIC and remote
	 * is the shared hardware QTimer used by SSC and camera CSID.
	 */
	bool converted = m_clock_windowed_skew_tracker_to_remote(hmd->qtimer_clock_tracker, at_timestamp_ns,
	                                                         &query_qtimer_ns);
	os_mutex_unlock(&hmd->mutex);
	if (!converted || query_qtimer_ns <= 0) {
		*out_relation = fallback;
		return;
	}

	int64_t latest_timestamp_ns = 0;
	struct xrt_space_relation latest = {0};
	if (m_relation_history_get_latest(hmd->qtimer_relation_history, &latest_timestamp_ns, &latest) &&
	    query_qtimer_ns > latest_timestamp_ns && query_qtimer_ns - latest_timestamp_ns > 100000000ll) {
		query_qtimer_ns = latest_timestamp_ns + 100000000ll;
	}

	if (m_relation_history_get(hmd->qtimer_relation_history, query_qtimer_ns, out_relation) ==
	    M_RELATION_HISTORY_RESULT_INVALID) {
		*out_relation = fallback;
	}
}

static xrt_result_t
galaxyxr_hmd_get_tracked_pose(struct xrt_device *xdev,
                              enum xrt_input_name name,
                              int64_t at_timestamp_ns,
                              struct xrt_space_relation *out_relation)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);

	if (name == XRT_INPUT_GENERIC_HEAD_POSE) {
		galaxyxr_hmd_get_head_relation(hmd, at_timestamp_ns, out_relation);
		return XRT_SUCCESS;
	}

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	if (name == XRT_INPUT_GENERIC_EYE_GAZE_POSE && hmd->base.supported.eye_gaze) {
		struct xrt_space_relation gaze_relation = XRT_SPACE_RELATION_ZERO;
		galaxyxr_eye_tracking_get_relation(&hmd->eye_tracking, at_timestamp_ns, &gaze_relation);
		if (gaze_relation.relation_flags == 0) {
			*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
			return XRT_SUCCESS;
		}

		struct xrt_space_relation head_relation = XRT_SPACE_RELATION_ZERO;
		galaxyxr_hmd_get_head_relation(hmd, at_timestamp_ns, &head_relation);
		struct xrt_relation_chain chain = {0};
		m_relation_chain_push_relation(&chain, &gaze_relation);
		m_relation_chain_push_relation(&chain, &head_relation);
		m_relation_chain_resolve(&chain, out_relation);
		return XRT_SUCCESS;
	}
#endif

	U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
	return XRT_ERROR_INPUT_UNSUPPORTED;
}

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
static xrt_result_t
galaxyxr_hmd_begin_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);
	if (type != XRT_DEVICE_FEATURE_EYE_TRACKING || !hmd->base.supported.eye_gaze) {
		return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}
	galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, true);
	sensor_thread_notify(hmd);
	return XRT_SUCCESS;
}

static xrt_result_t
galaxyxr_hmd_end_feature(struct xrt_device *xdev, enum xrt_device_feature_type type)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);
	if (type != XRT_DEVICE_FEATURE_EYE_TRACKING || !hmd->base.supported.eye_gaze) {
		return XRT_ERROR_FEATURE_NOT_SUPPORTED;
	}
	galaxyxr_eye_tracking_set_enabled(&hmd->eye_tracking, false);
	sensor_thread_notify(hmd);
	return XRT_SUCCESS;
}
#endif

static xrt_result_t
galaxyxr_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);

	uint32_t i = view < 2 ? view : 0;
	const struct galaxyxr_profile_eye *eye = &hmd->profile[i];
	const struct xrt_fov *fov = &xdev->hmd->distortion.fov[i];

	float px = (u - 0.5f) * (float)GXR_EYE_W;
	float py = (v - 0.5f) * (float)GXR_EYE_H;

	float t_l = tanf(fov->angle_left);
	float t_r = tanf(fov->angle_right);
	float t_top = -tanf(fov->angle_up);
	float t_bot = -tanf(fov->angle_down);

	struct xrt_vec2 *out[3] = {&out_result->r, &out_result->g, &out_result->b};
	for (uint32_t c = 0; c < 3; c++) {
		float tx, ty;
		if (hmd->cac_enabled || c == 1) {
			galaxyxr_profile_panel_to_tan(eye, c, px, py, &tx, &ty);
			out[c]->x = (tx - t_l) / (t_r - t_l);
			out[c]->y = (ty - t_top) / (t_bot - t_top);
		}
	}
	if (!hmd->cac_enabled) {
		out_result->r = out_result->g;
		out_result->b = out_result->g;
	}
	return XRT_SUCCESS;
}

static bool
load_profile(struct galaxyxr_hmd *hmd)
{
	// The efs file is this unit's factory calibration (display_profile_v2);
	// the /product one is a build-baked devkit default (v1), last resort.
	const char *paths[] = {
	    debug_get_option_galaxyxr_profile_path(),
	    "/mnt/vendor/efs/device_profile.textproto",
	    "/.oldroot/mnt/vendor/efs/device_profile.textproto",
	    "/product/etc/device_profile.textproto",
	    "/.oldroot/product/etc/device_profile.textproto",
	};
	for (uint32_t i = 0; i < ARRAY_SIZE(paths); i++) {
		if (paths[i] == NULL) {
			continue;
		}
		if (galaxyxr_profile_load(hmd->profile, paths[i]) == 0) {
			GXR_INFO(hmd, "Loaded Android XR display profile from %s", paths[i]);
			return true;
		}
	}
	GXR_WARN(hmd, "No Android XR device profile found");
	return false;
}

static xrt_result_t
galaxyxr_hmd_get_view_poses(struct xrt_device *xdev,
                            const struct xrt_vec3 *default_eye_relation,
                            int64_t at_timestamp_ns,
                            enum xrt_view_type view_type,
                            uint32_t view_count,
                            struct xrt_space_relation *out_head_relation,
                            struct xrt_fov *out_fovs,
                            struct xrt_pose *out_poses)
{
	struct galaxyxr_hmd *hmd = galaxyxr_hmd(xdev);

	struct xrt_vec3 eye_relation = *default_eye_relation;
	eye_relation.x = hmd->ipd_m;

	xrt_result_t xret = u_device_get_view_poses(xdev, &eye_relation, at_timestamp_ns, view_type, view_count,
	                                            out_head_relation, out_fovs, out_poses);

	if (xret != XRT_SUCCESS) {
		return xret;
	}

	if (hmd->have_view_pose) {
		os_mutex_lock(&hmd->mutex);
		for (uint32_t i = 0; i < view_count && i < 2; i++) {
			out_poses[i] = hmd->view_pose[i];
		}
		os_mutex_unlock(&hmd->mutex);
	}

	return xret;
}

bool
galaxyxr_is_present(void)
{
	char model[256] = {0};
	FILE *f = fopen("/proc/device-tree/model", "r");
	if (f == NULL) {
		return false;
	}
	size_t n = fread(model, 1, sizeof(model) - 1, f);
	fclose(f);
	model[n] = '\0';

	return strstr(model, "SM-I610") != NULL || strstr(model, "Samsung XR") != NULL;
}

struct xrt_device *
galaxyxr_hmd_create(void)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct galaxyxr_hmd *hmd =
	    U_DEVICE_ALLOCATE(struct galaxyxr_hmd, flags, GALAXYXR_HMD_INPUT_COUNT, 0);
	hmd->log_level = debug_get_log_option_galaxyxr_log();
	hmd->sensor_control_fd = -1;

	const char *axes = debug_get_option_galaxyxr_imu_axes();
	if (!parse_axis_map(axes, hmd->axis_src, hmd->axis_sign)) {
		GXR_ERROR(hmd, "Bad GALAXYXR_IMU_AXES '%s', using identity", axes);
		parse_axis_map("x,y,z", hmd->axis_src, hmd->axis_sign);
	}
	GXR_INFO(hmd, "IMU axis map: device = (%+.0f*%c, %+.0f*%c, %+.0f*%c)", //
	         hmd->axis_sign[0], 'x' + hmd->axis_src[0],                    //
	         hmd->axis_sign[1], 'x' + hmd->axis_src[1],                    //
	         hmd->axis_sign[2], 'x' + hmd->axis_src[2]);                   //

	os_mutex_init(&hmd->mutex);

	long rate = debug_get_num_option_galaxyxr_imu_rate();
	struct m_imu_xio_ahrs_settings ahrs_settings = M_IMU_XIO_AHRS_SETTINGS_DEFAULT;
	ahrs_settings.gain = 0.25f;
	ahrs_settings.recovery_trigger_period = 5 * (unsigned int)rate;
	m_imu_xio_ahrs_init(&hmd->fusion, &ahrs_settings);

	hmd->relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	m_relation_history_create(&hmd->qtimer_relation_history);
	hmd->qtimer_clock_tracker =
	    m_clock_windowed_skew_tracker_alloc(GXR_CLOCK_TRACKER_WINDOW_SAMPLES);
	if (hmd->qtimer_clock_tracker == NULL) {
		GXR_ERROR(hmd, "Failed to allocate QTimer clock tracker");
		galaxyxr_hmd_destroy(&hmd->base);
		return NULL;
	}

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Samsung Galaxy XR");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "Samsung Galaxy XR");

	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	hmd->base.update_inputs = galaxyxr_hmd_update_inputs;
	hmd->base.get_tracked_pose = galaxyxr_hmd_get_tracked_pose;
	hmd->base.get_view_poses = galaxyxr_hmd_get_view_poses;
	hmd->base.get_compositor_info = galaxyxr_hmd_get_compositor_info;
	hmd->base.destroy = galaxyxr_hmd_destroy;
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	hmd->base.begin_feature = galaxyxr_hmd_begin_feature;
	hmd->base.end_feature = galaxyxr_hmd_end_feature;
#endif

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	galaxyxr_hmd_input_init(&hmd->input, &hmd->base, &hmd->log_level);
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = false;
	hmd->base.supported.compositor_info = true;
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	hmd->base.supported.eye_gaze = galaxyxr_eye_tracking_init(&hmd->eye_tracking, &hmd->base, &hmd->log_level);
#endif

	hmd->base.hmd->screens[0].nominal_frame_interval_ns =
	    time_s_to_ns(1.0 / (double)debug_get_num_option_galaxyxr_hz());
	hmd->base.hmd->screens[0].w_pixels = GXR_EYE_W * 2;
	hmd->base.hmd->screens[0].h_pixels = GXR_EYE_H;

	const char *distortion_mode = debug_get_option_galaxyxr_distortion();
	hmd->cac_enabled = strcmp(distortion_mode, "profile") == 0;
	hmd->profile_active = strncmp(distortion_mode, "profile", 7) == 0 && load_profile(hmd);
	hmd->base.hmd->distortion.no_chromatic_aberration_correction = !hmd->cac_enabled;

	if (hmd->profile_active) {
		// The mapping frame carries the display cant baked in (v1:
		// head frame directions; v2: rays rotated into the calibrated
		// eye frames at load): the cameras must not be canted on top,
		// the asymmetric frustum does it. v2 additionally provides
		// per-eye view poses (per-unit eye positions in the IMU
		// frame), which is what makes the two eyes' images fuse.
		hmd->have_view_pose = hmd->profile[0].have_pose && hmd->profile[1].have_pose;
		if (hmd->have_view_pose) {
			struct xrt_vec3 mid = hmd->profile[0].pose.position;
			math_vec3_accum(&hmd->profile[1].pose.position, &mid);
			math_vec3_scalar_mul(0.5f, &mid);
			for (uint32_t e = 0; e < 2; e++) {
				hmd->view_pose[e] = hmd->profile[e].pose;
				math_vec3_subtract(&mid, &hmd->view_pose[e].position);
			}
		}
		if (hmd->profile[0].ipd_mm > 0.0f) {
			hmd->ipd_m = hmd->profile[0].ipd_mm / 1000.0f;
		}

		for (uint32_t e = 0; e < 2; e++) {
			hmd->base.hmd->distortion.fov[e] = hmd->profile[e].fov;
		}

		for (uint32_t e = 0; e < 2; e++) {
			const struct xrt_fov *fov = &hmd->base.hmd->distortion.fov[e];
			float tx, ty;
			galaxyxr_profile_panel_to_tan(&hmd->profile[e], 1, 0.0f, 0.0f, &tx, &ty);
			GXR_INFO(hmd, "Profile fov[%u]: %.1f/%.1f h, %.1f/%.1f v deg, panel center %+.2f/%+.2f deg",
			         e,                                                                              //
			         fov->angle_left * (180.0f / (float)M_PI), fov->angle_right * (180.0f / (float)M_PI),
			         fov->angle_up * (180.0f / (float)M_PI), fov->angle_down * (180.0f / (float)M_PI),
			         atanf(tx) * (180.0f / (float)M_PI), atanf(ty) * (180.0f / (float)M_PI));
			if (hmd->have_view_pose) {
				const struct xrt_pose *p = &hmd->view_pose[e];
				GXR_INFO(hmd,
				         "View pose[%u]: pos (%+.2f, %+.2f, %+.2f) mm, quat (%+.4f, %+.4f, %+.4f, %+.4f)",
				         e, p->position.x * 1000.0f, p->position.y * 1000.0f, p->position.z * 1000.0f,
				         p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w);
			}
		}
	} else {
		// No calibration: identity mesh with the rough panel fov.
		const float h_half = 44.75f * ((float)M_PI / 180.0f);
		const float v_half = atanf(tanf(h_half) * ((float)GXR_EYE_H / (float)GXR_EYE_W));
		struct xrt_fov fov = {
		    .angle_left = -h_half,
		    .angle_right = h_half,
		    .angle_up = v_half,
		    .angle_down = -v_half,
		};
		hmd->base.hmd->distortion.fov[0] = fov;
		hmd->base.hmd->distortion.fov[1] = fov;
	}

	if (hmd->ipd_m <= 0.0f) {
		hmd->ipd_m = 0.065f;
	}

	for (uint8_t eye = 0; eye < 2; ++eye) {
		hmd->base.hmd->views[eye].display.w_pixels = GXR_EYE_W;
		hmd->base.hmd->views[eye].display.h_pixels = GXR_EYE_H;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = GXR_EYE_W;
		hmd->base.hmd->views[eye].viewport.h_pixels = GXR_EYE_H;
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
	}
	hmd->base.hmd->views[0].viewport.x_pixels = 0;
	hmd->base.hmd->views[1].viewport.x_pixels = GXR_EYE_W;

	if (hmd->profile_active) {
		hmd->base.compute_distortion = galaxyxr_compute_distortion;
		hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
		hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
		u_distortion_mesh_fill_in_compute(&hmd->base);
		GXR_INFO(hmd, "Distortion from the Android XR display profile, %s CAC",
		         hmd->cac_enabled ? "with" : "without");
	} else {
		u_distortion_mesh_set_none(&hmd->base);
	}

	// Per-unit factory files only: the /product devkit fallback carries
	// another unit's imus{} entries and must not be applied here.
	const char *cal_paths[] = {
	    debug_get_option_galaxyxr_profile_path(),
	    "/mnt/vendor/efs/device_profile.textproto",
	    "/.oldroot/mnt/vendor/efs/device_profile.textproto",
	};
	for (uint32_t i = 0; i < ARRAY_SIZE(cal_paths); i++) {
		if (cal_paths[i] == NULL) {
			continue;
		}
		if (galaxyxr_profile_load_imu_cal(&hmd->imu_cal, cal_paths[i], 0) == 0) {
			break;
		}
	}
	if (hmd->imu_cal.valid) {
		hmd->gyro_bias = hmd->imu_cal.gyro_bias;
		GXR_INFO(hmd, "IMU factory cal%s: gyro bias (%+.2f, %+.2f, %+.2f) mrad/s scale (%.4f, %.4f, %.4f)",
		         debug_get_bool_option_galaxyxr_imu_cal() ? "" : " (DISABLED)",             //
		         hmd->imu_cal.gyro_bias.x * 1000.0f, hmd->imu_cal.gyro_bias.y * 1000.0f,    //
		         hmd->imu_cal.gyro_bias.z * 1000.0f,                                        //
		         hmd->imu_cal.gyro_scale.x, hmd->imu_cal.gyro_scale.y, hmd->imu_cal.gyro_scale.z);
		GXR_INFO(hmd, "IMU factory cal: accel bias (%+.3f, %+.3f, %+.3f) m/s^2 scale (%.4f, %.4f, %.4f)",
		         hmd->imu_cal.accel_bias.x, hmd->imu_cal.accel_bias.y, hmd->imu_cal.accel_bias.z, //
		         hmd->imu_cal.accel_scale.x, hmd->imu_cal.accel_scale.y, hmd->imu_cal.accel_scale.z);
	} else {
		GXR_WARN(hmd, "No per-unit IMU calibration found, samples stay raw");
	}

	u_var_add_root(hmd, "Galaxy XR", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_ro_vec3_f32(hmd, &hmd->raw_accel, "raw_accel");
	u_var_add_ro_vec3_f32(hmd, &hmd->raw_gyro, "raw_gyro");
	u_var_add_ro_vec3_f32(hmd, &hmd->accel, "accel");
	u_var_add_ro_vec3_f32(hmd, &hmd->gyro, "gyro");
	u_var_add_ro_vec3_f32(hmd, &hmd->gyro_bias, "gyro_bias");
	u_var_add_pose(hmd, &hmd->relation.pose, "pose");

#ifdef XRT_FEATURE_TITAN_PASSTHROUGH
	hmd->base.passthrough =
	    galaxyxr_passthrough_create(debug_get_option_galaxyxr_profile_path(), hmd->qtimer_relation_history);
	if (hmd->base.passthrough != NULL) {
		hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_ALPHA_BLEND;
		hmd->base.hmd->blend_mode_count = idx;
	}
#endif

	int ret = galaxyxr_ssc_open(&hmd->ssc, (float)rate);
	if (ret < 0) {
		GXR_ERROR(hmd, "Failed to reach the SSC sensor service: %d", ret);
		goto cleanup;
	}
	hmd->ssc_opened = true;
	ret = sensor_control_event_init(hmd);
	if (ret < 0) {
		GXR_ERROR(hmd, "Failed to create sensor control eventfd: %s", strerror(-ret));
		goto cleanup;
	}

	if (os_thread_helper_init(&hmd->sensor_thread) != 0) {
		GXR_ERROR(hmd, "Failed to init sensor thread");
		goto cleanup;
	}

	if (os_thread_helper_start(&hmd->sensor_thread, sensor_thread_fn, hmd) != 0) {
		GXR_ERROR(hmd, "Failed to start sensor thread");
		goto cleanup;
	}
	os_thread_helper_name(&hmd->sensor_thread, "Galaxy XR IMU");

	GXR_INFO(hmd, "Galaxy XR HMD created, IMU at %ld Hz", rate);

	return &hmd->base;

cleanup:
	galaxyxr_hmd_destroy(&hmd->base);
	return NULL;
}
