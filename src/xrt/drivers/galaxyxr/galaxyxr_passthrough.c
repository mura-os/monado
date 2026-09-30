// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Galaxy XR implementation of the generic passthrough provider.
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_passthrough.h"
#include "galaxyxr_passthrough_calibration.h"

#include "xrt/xrt_passthrough.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_relation_history.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_misc.h"

#include "titan_client.h"

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

DEBUG_GET_ONCE_BOOL_OPTION(galaxyxr_passthrough, "XRT_PASSTHROUGH", true)
DEBUG_GET_ONCE_LOG_OPTION(galaxyxr_passthrough_log, "XRT_PASSTHROUGH_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_OPTION(galaxyxr_passthrough_calibration, "XRT_PASSTHROUGH_CALIBRATION", NULL)
/*
 * Experimentally apply the device-profile camera-to-IMU timestamp alignment
 * before looking up capture poses. The profile contains +4.3 ms for both RGB
 * cameras and that direction looked preferable in an A/B test, but its
 * semantics for Titan's reconstructed exposure timestamp have not been
 * objectively verified. This uncertainty is why the option is "magical".
 * It defaults on and can be disabled for an uncorrected comparison.
 */
DEBUG_GET_ONCE_BOOL_OPTION(galaxyxr_passthrough_magical_timestamp_offset,
                           "XRT_GALAXY_PASSTHROUGH_MAGICAL_TIMESTAMP_OFFSET",
                           true)

#define GXP_TRACE(p, ...) U_LOG_IFL_T((p)->log_level, __VA_ARGS__)
#define GXP_DEBUG(p, ...) U_LOG_IFL_D((p)->log_level, __VA_ARGS__)
#define GXP_INFO(p, ...) U_LOG_IFL_I((p)->log_level, __VA_ARGS__)
#define GXP_WARN(p, ...) U_LOG_IFL_W((p)->log_level, __VA_ARGS__)
#define GXP_ERROR(p, ...) U_LOG_IFL_E((p)->log_level, __VA_ARGS__)

struct galaxyxr_frame_token
{
	struct galaxyxr_frame_token *next;
	struct titan_msg_frame frame;
	uint64_t generation;
};

struct galaxyxr_pending_frame
{
	struct titan_msg_frame titan;
	struct xrt_pose capture_pose_begin[TITAN_PROTO_MAX_VIEWS];
	struct xrt_pose capture_pose_end[TITAN_PROTO_MAX_VIEWS];
};

struct galaxyxr_frame_used_feedback
{
	bool pending;
	uint64_t generation;
	uint64_t use_sequence;
	uint64_t used_boottime_ns;
	struct titan_msg_frame frame;
};

struct galaxyxr_passthrough
{
	struct xrt_passthrough_stream base;
	enum u_logging_level log_level;

	struct os_thread_helper thread;
	// Consumer intent, protected by the thread helper's lock.
	bool enabled;
	// Wakes the client thread when another thread queues work.
	int wake_fd;

	struct os_mutex mutex;
	struct m_relation_history *qtimer_relation_history;

	// Only the client thread calls the Titan transport helpers.
	int titan_fd;
	bool titan_stream_open;

	// Protected by mutex.
	bool stream_ready;
	uint64_t generation;
	struct titan_msg_stream_info info;
	int slot_fds[TITAN_PROTO_MAX_VIEWS][TITAN_PROTO_MAX_SLOTS];
	bool pending_valid;
	struct galaxyxr_pending_frame pending;
	struct galaxyxr_frame_token *held;
	struct galaxyxr_frame_token *release_head;
	struct galaxyxr_frame_token **release_tail;
	struct galaxyxr_frame_used_feedback feedback;
	uint64_t feedback_sequence;

	// Relaxed atomics: telemetry only, not synchronization.
	_Atomic(uint64_t) feedback_sent;
	_Atomic(uint64_t) feedback_dropped;

	uint64_t frames_received;
	uint64_t frames_acquired;
	uint64_t frames_dropped;
	bool capture_pose_fallback_warned;
	bool feedback_clock_warned;
};

static inline struct galaxyxr_passthrough *
galaxyxr_passthrough(struct xrt_passthrough_stream *xp)
{
	return (struct galaxyxr_passthrough *)xp;
}

static void
close_received_fds(int *fds, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		if (fds[i] >= 0) {
			close(fds[i]);
			fds[i] = -1;
		}
	}
}

static void
wake_client_thread(struct galaxyxr_passthrough *p)
{
	uint64_t value = 1;
	ssize_t ret;
	do {
		ret = write(p->wake_fd, &value, sizeof(value));
	} while (ret < 0 && errno == EINTR);

	// EAGAIN means an earlier notification is still pending, which is sufficient.
	if (ret < 0 && errno != EAGAIN) {
		GXP_WARN(p, "galaxyxr passthrough: failed to wake client thread: %s", strerror(errno));
	} else if (ret >= 0 && (size_t)ret != sizeof(value)) {
		GXP_WARN(p, "galaxyxr passthrough: short write while waking client thread");
	}
}

static bool
drain_client_wake(struct galaxyxr_passthrough *p)
{
	uint64_t value = 0;
	ssize_t ret;
	do {
		ret = read(p->wake_fd, &value, sizeof(value));
	} while (ret < 0 && errno == EINTR);

	if (ret == (ssize_t)sizeof(value) || (ret < 0 && errno == EAGAIN)) {
		return true;
	}
	if (ret >= 0) {
		errno = EIO;
	}
	return false;
}

static int
receive_reply(struct galaxyxr_passthrough *p,
              uint32_t request_type,
              uint32_t reply_type,
              union titan_msg_any *reply,
              int *fds,
              size_t fd_capacity,
              size_t *fd_count)
{
	ssize_t received = titan_client_recv(p->titan_fd, reply, sizeof(*reply), fds, fd_capacity, fd_count, 0);
	if (received == 0) {
		errno = ECONNRESET;
		return -1;
	}
	if (received < 0) {
		return -1;
	}
	if (reply->hdr.type == TITAN_MSG_ERROR) {
		if (reply->error.in_reply_to != request_type) {
			errno = EPROTO;
		} else if (reply->error.code < 0 && reply->error.code >= -4095) {
			errno = -reply->error.code;
		} else {
			errno = EPROTO;
		}
		GXP_WARN(p, "galaxyxr passthrough: Titan request %u failed: %.*s", request_type,
		         (int)TITAN_PROTO_NAME_LEN, reply->error.text);
		return -1;
	}
	if (reply->hdr.type != reply_type) {
		if (fd_count != NULL) {
			close_received_fds(fds, *fd_count);
			*fd_count = 0;
		}
		errno = EPROTO;
		return -1;
	}
	return 0;
}

static int
list_cameras(struct galaxyxr_passthrough *p)
{
	union titan_msg_any reply = {0};

	if (titan_client_send_list(p->titan_fd, 0) != 0 ||
	    receive_reply(p, TITAN_MSG_LIST, TITAN_MSG_LIST_REPLY, &reply, NULL, 0, NULL) != 0) {
		return -1;
	}
	return (int)reply.list_reply.count;
}

static int
list_modes(struct galaxyxr_passthrough *p, int32_t cam_slot, struct titan_wire_mode *out, size_t capacity)
{
	union titan_msg_any reply = {0};

	if (titan_client_send_list_modes(p->titan_fd, cam_slot, 0) != 0 ||
	    receive_reply(p, TITAN_MSG_LIST_MODES, TITAN_MSG_LIST_MODES_REPLY, &reply, NULL, 0, NULL) != 0) {
		return -1;
	}

	size_t count = reply.list_modes_reply.count;
	if (count > capacity) {
		count = capacity;
	}
	memcpy(out, reply.list_modes_reply.modes, count * sizeof(*out));
	return (int)count;
}

static uint32_t
pick_mode(struct galaxyxr_passthrough *p)
{
	struct titan_wire_mode modes[TITAN_PROTO_MAX_MODES];
	int count = list_modes(p, 0, modes, TITAN_PROTO_MAX_MODES);
	if (count < 0) {
		GXP_WARN(p, "galaxyxr passthrough: failed to list Titan camera modes: %s", strerror(errno));
		return 0;
	}

	int best = -1;
	float best_fps = 0.0f;
	for (int i = 0; i < count; i++) {
		const struct titan_wire_mode *mode = &modes[i];
		char name[TITAN_PROTO_NAME_LEN + 1];
		memcpy(name, mode->name, TITAN_PROTO_NAME_LEN);
		name[TITAN_PROTO_NAME_LEN] = '\0';
		if (mode->width != 3000 || mode->height != 3000) {
			continue;
		}
		bool named =
		    (strstr(name, "main") != NULL || strstr(name, "single") != NULL) && strstr(name, "sub") == NULL;
		if (named && mode->fps > best_fps) {
			best = i;
			best_fps = mode->fps;
		}
	}
	if (best < 0) {
		GXP_WARN(p, "galaxyxr passthrough: no 3000x3000 main mode, using automatic mode selection");
		return 0;
	}
	GXP_INFO(p, "galaxyxr passthrough: sensor mode %d '%.*s' %ux%u@%.1f", best + 1, (int)TITAN_PROTO_NAME_LEN,
	         modes[best].name, modes[best].width, modes[best].height, modes[best].fps);
	return (uint32_t)(best + 1);
}

static bool
open_stream(struct galaxyxr_passthrough *p)
{
	struct titan_wire_config config = {
	    .cam_slot = TITAN_WIRE_CAM_STEREO,
	    .mode_index = pick_mode(p),
	    .output_format = TITAN_WIRE_FMT_NV12,
	};
	union titan_msg_any reply = {0};
	int received_fds[TITAN_PROTO_MAX_FDS];
	size_t fd_count = 0;
	for (size_t i = 0; i < TITAN_PROTO_MAX_FDS; i++) {
		received_fds[i] = -1;
	}

	if (titan_client_send_open(p->titan_fd, &config, 0) != 0 ||
	    receive_reply(p, TITAN_MSG_OPEN, TITAN_MSG_STREAM_INFO, &reply, received_fds, TITAN_PROTO_MAX_FDS,
	                  &fd_count) != 0) {
		GXP_WARN(p, "galaxyxr passthrough: failed to open Titan stereo stream: %s", strerror(errno));
		close_received_fds(received_fds, fd_count);
		return false;
	}

	const struct titan_msg_stream_info *info = &reply.stream_info;
	if (info->output_format != TITAN_WIRE_FMT_NV12 || info->num_views != 2 || info->chroma_offset == 0 ||
	    info->num_slots == 0 || info->num_slots > TITAN_PROTO_MAX_SLOTS) {
		GXP_ERROR(p, "galaxyxr passthrough: unusable stream (format %u, views %u, chroma %u, slots %u)",
		          info->output_format, info->num_views, info->chroma_offset, info->num_slots);
		(void)titan_client_send_close(p->titan_fd, info->stream_id, 0);
		close_received_fds(received_fds, fd_count);
		errno = EPROTO;
		return false;
	}

	int fds[TITAN_PROTO_MAX_VIEWS][TITAN_PROTO_MAX_SLOTS];
	for (uint32_t v = 0; v < TITAN_PROTO_MAX_VIEWS; v++) {
		for (uint32_t s = 0; s < TITAN_PROTO_MAX_SLOTS; s++) {
			fds[v][s] = -1;
		}
	}
	for (uint32_t v = 0; v < info->num_views; v++) {
		for (uint32_t s = 0; s < info->num_slots; s++) {
			size_t index = 0;
			if (!titan_proto_stream_slot_index(info, v, s, &index) || index >= fd_count) {
				(void)titan_client_send_close(p->titan_fd, info->stream_id, 0);
				close_received_fds(received_fds, fd_count);
				errno = EPROTO;
				return false;
			}
			fds[v][s] = received_fds[index];
		}
	}
	for (size_t i = 0; i < fd_count; i++) {
		received_fds[i] = -1;
	}

	p->titan_stream_open = true;
	os_mutex_lock(&p->mutex);
	p->info = *info;
	memcpy(p->slot_fds, fds, sizeof(fds));
	p->generation++;
	p->stream_ready = true;
	p->pending_valid = false;
	U_ZERO(&p->feedback);
	p->feedback_sequence = 0;
	atomic_store_explicit(&p->feedback_sent, 0, memory_order_relaxed);
	atomic_store_explicit(&p->feedback_dropped, 0, memory_order_relaxed);
	os_mutex_unlock(&p->mutex);

	GXP_INFO(p,
	         "galaxyxr passthrough: Titan v%u stream '%.*s' %ux%u@%.1f, stride %u, chroma %u, slots %u, "
	         "FRAME_USED %s",
	         TITAN_PROTO_VERSION, (int)TITAN_PROTO_NAME_LEN, info->name, info->width, info->height, info->fps,
	         info->stride, info->chroma_offset, info->num_slots, info->controller ? "enabled" : "viewer-disabled");
	return true;
}

static void
free_token_list(struct galaxyxr_frame_token *token)
{
	while (token != NULL) {
		struct galaxyxr_frame_token *next = token->next;
		free(token);
		token = next;
	}
}

static void
teardown_connection(struct galaxyxr_passthrough *p, bool send_close)
{
	int slot_fds[TITAN_PROTO_MAX_FDS];
	size_t slot_fd_count = 0;
	uint64_t feedback_sequence = 0;
	uint64_t feedback_sent = 0;
	uint64_t feedback_dropped = 0;

	os_mutex_lock(&p->mutex);
	p->stream_ready = false;
	p->pending_valid = false;
	if (p->feedback.pending) {
		atomic_fetch_add_explicit(&p->feedback_dropped, 1, memory_order_relaxed);
		p->feedback.pending = false;
	}
	feedback_sequence = p->feedback_sequence;
	feedback_sent = atomic_exchange_explicit(&p->feedback_sent, 0, memory_order_relaxed);
	feedback_dropped = atomic_exchange_explicit(&p->feedback_dropped, 0, memory_order_relaxed);
	p->feedback_sequence = 0;
	for (uint32_t v = 0; v < TITAN_PROTO_MAX_VIEWS; v++) {
		for (uint32_t s = 0; s < TITAN_PROTO_MAX_SLOTS; s++) {
			if (p->slot_fds[v][s] >= 0) {
				slot_fds[slot_fd_count++] = p->slot_fds[v][s];
				p->slot_fds[v][s] = -1;
			}
		}
	}
	struct galaxyxr_frame_token *releases = p->release_head;
	p->release_head = NULL;
	p->release_tail = &p->release_head;
	os_mutex_unlock(&p->mutex);

	// Disconnecting drops all server-side holds, so these need no RELEASE.
	free_token_list(releases);
	close_received_fds(slot_fds, slot_fd_count);
	if (feedback_sequence > 0) {
		GXP_INFO(p, "galaxyxr passthrough: FRAME_USED %llu sent, %llu locally dropped across %llu render ticks",
		         (unsigned long long)feedback_sent, (unsigned long long)feedback_dropped,
		         (unsigned long long)feedback_sequence);
	}
	if (send_close && p->titan_stream_open && p->titan_fd >= 0) {
		(void)titan_client_send_close(p->titan_fd, p->info.stream_id, 0);
	}
	p->titan_stream_open = false;
	if (p->titan_fd >= 0) {
		titan_client_disconnect(p->titan_fd);
		p->titan_fd = -1;
	}
}

static bool
drain_frame_used_feedback(struct galaxyxr_passthrough *p)
{
	struct galaxyxr_frame_used_feedback feedback = {0};
	struct titan_msg_stream_info info = {0};
	bool send = false;

	os_mutex_lock(&p->mutex);
	if (p->feedback.pending) {
		feedback = p->feedback;
		p->feedback.pending = false;
		if (p->stream_ready && feedback.generation == p->generation) {
			info = p->info;
			send = true;
		} else {
			atomic_fetch_add_explicit(&p->feedback_dropped, 1, memory_order_relaxed);
		}
	}
	os_mutex_unlock(&p->mutex);
	if (!send) {
		return true;
	}

	if (titan_client_send_frame_used(p->titan_fd, &info, &feedback.frame, feedback.use_sequence,
	                                 feedback.used_boottime_ns, MSG_DONTWAIT) == 0) {
		atomic_fetch_add_explicit(&p->feedback_sent, 1, memory_order_relaxed);
		return true;
	}
	if (errno == EAGAIN || errno == EWOULDBLOCK) {
		atomic_fetch_add_explicit(&p->feedback_dropped, 1, memory_order_relaxed);
		return true;
	}

	atomic_fetch_add_explicit(&p->feedback_dropped, 1, memory_order_relaxed);
	GXP_WARN(p, "galaxyxr passthrough: Titan FRAME_USED failed: %s", strerror(errno));
	return false;
}

static bool
drain_releases(struct galaxyxr_passthrough *p)
{
	os_mutex_lock(&p->mutex);
	struct galaxyxr_frame_token *list = p->release_head;
	p->release_head = NULL;
	p->release_tail = &p->release_head;
	uint64_t generation = p->generation;
	bool ready = p->stream_ready;
	struct titan_msg_stream_info info = p->info;
	os_mutex_unlock(&p->mutex);

	bool connection_ok = true;
	while (list != NULL) {
		struct galaxyxr_frame_token *next = list->next;
		if (connection_ok && ready && list->generation == generation) {
			if (titan_client_send_release(p->titan_fd, &info, &list->frame, 0) != 0) {
				GXP_WARN(p, "galaxyxr passthrough: Titan frame release failed: %s", strerror(errno));
				connection_ok = false;
			}
		}
		free(list);
		list = next;
	}
	return connection_ok;
}

static void
interruptible_sleep(struct galaxyxr_passthrough *p, uint32_t milliseconds)
{
	for (uint32_t elapsed = 0; elapsed < milliseconds; elapsed += 100) {
		if (!os_thread_helper_is_running(&p->thread)) {
			return;
		}
		os_nanosleep(100 * 1000 * 1000);
	}
}

static bool
is_enabled(struct galaxyxr_passthrough *p)
{
	os_thread_helper_lock(&p->thread);
	bool enabled = p->enabled;
	os_thread_helper_unlock(&p->thread);
	return enabled;
}

static void
wait_until_enabled(struct galaxyxr_passthrough *p)
{
	os_thread_helper_lock(&p->thread);
	while (!p->enabled && os_thread_helper_is_running_locked(&p->thread)) {
		os_thread_helper_wait_locked(&p->thread);
	}
	os_thread_helper_unlock(&p->thread);
}

static bool
connect_to_titan(struct galaxyxr_passthrough *p)
{
	p->titan_fd = titan_client_connect(NULL);
	if (p->titan_fd < 0) {
		return false;
	}

	int camera_count = list_cameras(p);
	if (camera_count < 0) {
		GXP_WARN(p, "galaxyxr passthrough: failed to query Titan cameras: %s", strerror(errno));
		teardown_connection(p, false);
		return false;
	}
	GXP_INFO(p, "galaxyxr passthrough: connected to Titan, %d cameras", camera_count);
	return true;
}

static int
wait_for_message(struct galaxyxr_passthrough *p, union titan_msg_any *message)
{
	struct pollfd poll_fds[2] = {
	    {
	        .fd = p->titan_fd,
	        .events = POLLIN,
	    },
	    {
	        .fd = p->wake_fd,
	        .events = POLLIN,
	    },
	};
	int ret;
	do {
		ret = poll(poll_fds, ARRAY_SIZE(poll_fds), -1);
	} while (ret < 0 && errno == EINTR && os_thread_helper_is_running(&p->thread));
	if (ret <= 0) {
		return ret;
	}

	// Prioritize queued feedback and releases over receiving another frame.
	if ((poll_fds[1].revents & POLLIN) != 0) {
		return drain_client_wake(p) ? 0 : -1;
	}
	if (poll_fds[1].revents != 0) {
		errno = EIO;
		return -1;
	}
	if ((poll_fds[0].revents & POLLIN) == 0) {
		errno = ECONNRESET;
		return -1;
	}

	ssize_t received = titan_client_recv(p->titan_fd, message, sizeof(*message), NULL, 0, NULL, MSG_DONTWAIT);
	if (received == 0) {
		errno = ECONNRESET;
		return -1;
	}
	if (received < 0) {
		return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
	}
	return 1;
}

static bool
resolve_capture_poses(struct galaxyxr_passthrough *p,
                      const struct titan_msg_frame *frame,
                      struct xrt_pose out_begin[TITAN_PROTO_MAX_VIEWS],
                      struct xrt_pose out_end[TITAN_PROTO_MAX_VIEWS]);

static bool
handle_frame(struct galaxyxr_passthrough *p, const struct titan_msg_frame *frame)
{
	if (titan_proto_validate_frame(frame, &p->info) != TITAN_PROTO_VALID) {
		errno = EPROTO;
		return false;
	}

	struct galaxyxr_pending_frame incoming = {
	    .titan = *frame,
	};
	bool capture_poses_resolved =
	    resolve_capture_poses(p, frame, incoming.capture_pose_begin, incoming.capture_pose_end);
	if (!capture_poses_resolved && !p->capture_pose_fallback_warned) {
		GXP_WARN(p,
		         "galaxyxr passthrough: could not resolve camera QTimer in the IMU pose history; "
		         "using the latest available pose");
	}
	p->capture_pose_fallback_warned = !capture_poses_resolved;

	struct titan_msg_frame stale = {0};
	bool have_stale = false;
	os_mutex_lock(&p->mutex);
	p->frames_received++;
	if (p->pending_valid) {
		stale = p->pending.titan;
		have_stale = true;
		p->frames_dropped++;
	}
	p->pending = incoming;
	p->pending_valid = true;
	os_mutex_unlock(&p->mutex);

	if (have_stale && titan_client_send_release(p->titan_fd, &p->info, &stale, 0) != 0) {
		return false;
	}
	return true;
}

static void *
client_thread(void *ptr)
{
	struct galaxyxr_passthrough *p = ptr;
	GXP_INFO(p, "galaxyxr passthrough: client thread started");

	while (os_thread_helper_is_running(&p->thread)) {
		if (!is_enabled(p)) {
			if (p->titan_fd >= 0) {
				GXP_INFO(p, "galaxyxr passthrough: closing camera stream, passthrough disabled");
				teardown_connection(p, true);
			}
			wait_until_enabled(p);
			continue;
		}
		if (p->titan_fd < 0) {
			if (!connect_to_titan(p)) {
				interruptible_sleep(p, 2000);
				continue;
			}
		}
		if (!p->titan_stream_open && !open_stream(p)) {
			teardown_connection(p, false);
			interruptible_sleep(p, 2000);
			continue;
		}

		if (!drain_frame_used_feedback(p) || !drain_releases(p)) {
			teardown_connection(p, false);
			interruptible_sleep(p, 1000);
			continue;
		}

		union titan_msg_any message = {0};
		int ret = wait_for_message(p, &message);
		if (ret < 0) {
			GXP_WARN(p, "galaxyxr passthrough: Titan connection lost: %s", strerror(errno));
			teardown_connection(p, false);
			interruptible_sleep(p, 1000);
			continue;
		}
		if (ret == 0) {
			continue;
		}

		switch (message.hdr.type) {
		case TITAN_MSG_FRAME:
			if (!handle_frame(p, &message.frame)) {
				GXP_WARN(p, "galaxyxr passthrough: invalid Titan frame or failed release: %s",
				         strerror(errno));
				teardown_connection(p, false);
				interruptible_sleep(p, 1000);
			}
			break;
		case TITAN_MSG_CLOSE:
			GXP_WARN(p, "galaxyxr passthrough: Titan closed the stream");
			teardown_connection(p, false);
			interruptible_sleep(p, 1000);
			break;
		case TITAN_MSG_ERROR:
			GXP_WARN(p, "galaxyxr passthrough: unexpected Titan error: %.*s", (int)TITAN_PROTO_NAME_LEN,
			         message.error.text);
			teardown_connection(p, false);
			interruptible_sleep(p, 1000);
			break;
		default:
			GXP_WARN(p, "galaxyxr passthrough: unexpected Titan message type %u", message.hdr.type);
			teardown_connection(p, false);
			interruptible_sleep(p, 1000);
			break;
		}
	}

	(void)drain_frame_used_feedback(p);
	(void)drain_releases(p);
	teardown_connection(p, true);
	GXP_INFO(p, "galaxyxr passthrough: client thread stopped");
	return NULL;
}

static bool
get_capture_range_qtimer(const struct titan_wire_frame_view *source,
                         const struct xrt_passthrough_camera_calibration *camera,
                         int64_t *out_begin_ns,
                         int64_t *out_end_ns)
{
	if (source->first_row_exposure_start_qtimer_ns > (uint64_t)INT64_MAX ||
	    source->exposure_duration_ns / 2 > (uint64_t)INT64_MAX ||
	    camera->rolling_shutter_readout_ns > (uint64_t)INT64_MAX) {
		return false;
	}

	int64_t begin_ns = (int64_t)source->first_row_exposure_start_qtimer_ns;
	if (debug_get_bool_option_galaxyxr_passthrough_magical_timestamp_offset()) {
		begin_ns += camera->timestamp_alignment_ns;
	}

	// Timestamp each row at the midpoint of its integration interval.
	int64_t half_exposure_ns = (int64_t)(source->exposure_duration_ns / 2);
	int64_t readout_ns = (int64_t)camera->rolling_shutter_readout_ns;
	if (begin_ns <= 0 || begin_ns > INT64_MAX - half_exposure_ns ||
	    begin_ns + half_exposure_ns > INT64_MAX - readout_ns) {
		return false;
	}
	begin_ns += half_exposure_ns;

	*out_begin_ns = begin_ns;
	*out_end_ns = begin_ns + readout_ns;
	return true;
}

static bool
resolve_capture_poses(struct galaxyxr_passthrough *p,
                      const struct titan_msg_frame *frame,
                      struct xrt_pose out_begin[TITAN_PROTO_MAX_VIEWS],
                      struct xrt_pose out_end[TITAN_PROTO_MAX_VIEWS])
{
	const enum xrt_space_relation_flags required =
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	struct xrt_space_relation latest = {0};
	int64_t latest_qtimer_ns = 0;
	bool have_latest = p->qtimer_relation_history != NULL &&
	                   m_relation_history_get_latest(p->qtimer_relation_history, &latest_qtimer_ns, &latest);

	struct xrt_pose fallback = XRT_POSE_IDENTITY;
	if (have_latest && (latest.relation_flags & required) == required) {
		fallback = latest.pose;
	}
	for (uint32_t v = 0; v < TITAN_PROTO_MAX_VIEWS; v++) {
		out_begin[v] = fallback;
		out_end[v] = fallback;
	}
	if (!have_latest) {
		return false;
	}

	struct xrt_pose resolved_begin[TITAN_PROTO_MAX_VIEWS] = {0};
	struct xrt_pose resolved_end[TITAN_PROTO_MAX_VIEWS] = {0};
	for (uint32_t v = 0; v < p->info.num_views; v++) {
		const struct titan_wire_frame_view *source = &frame->view[v];
		const struct xrt_passthrough_camera_calibration *camera = &p->base.calibration.views[v];
		int64_t begin_qtimer_ns = 0;
		int64_t end_qtimer_ns = 0;
		if (!get_capture_range_qtimer(source, camera, &begin_qtimer_ns, &end_qtimer_ns)) {
			return false;
		}

		/*
		 * Reject timestamps that clearly do not share the IMU's QTimer
		 * timeline. A freshly delivered camera frame may be just ahead of
		 * the latest IMU sample; relation history predicts that short tail.
		 */
		const int64_t max_imu_lag_ns = 50LL * U_TIME_1MS_IN_NS;
		const int64_t max_camera_age_ns = 250LL * U_TIME_1MS_IN_NS;
		int64_t latest_min_ns = end_qtimer_ns > max_imu_lag_ns ? end_qtimer_ns - max_imu_lag_ns : 0;
		int64_t latest_max_ns =
		    begin_qtimer_ns < INT64_MAX - max_camera_age_ns ? begin_qtimer_ns + max_camera_age_ns : INT64_MAX;
		if (latest_qtimer_ns < latest_min_ns || latest_qtimer_ns > latest_max_ns) {
			return false;
		}

		struct xrt_space_relation begin = {0};
		struct xrt_space_relation end = {0};
		enum m_relation_history_result begin_result =
		    m_relation_history_get(p->qtimer_relation_history, begin_qtimer_ns, &begin);
		enum m_relation_history_result end_result =
		    m_relation_history_get(p->qtimer_relation_history, end_qtimer_ns, &end);
		bool valid = begin_result != M_RELATION_HISTORY_RESULT_INVALID &&
		             begin_result != M_RELATION_HISTORY_RESULT_REVERSE_PREDICTED &&
		             end_result != M_RELATION_HISTORY_RESULT_INVALID &&
		             end_result != M_RELATION_HISTORY_RESULT_REVERSE_PREDICTED &&
		             (begin.relation_flags & required) == required &&
		             (end.relation_flags & required) == required;
		if (!valid) {
			return false;
		}

		resolved_begin[v] = begin.pose;
		resolved_end[v] = end.pose;
	}

	memcpy(out_begin, resolved_begin, sizeof(resolved_begin));
	memcpy(out_end, resolved_end, sizeof(resolved_end));
	return true;
}

static void
set_enabled(struct xrt_passthrough_stream *xp, bool enabled)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);

	os_thread_helper_lock(&p->thread);
	bool changed = p->enabled != enabled;
	p->enabled = enabled;
	if (changed) {
		os_thread_helper_signal_locked(&p->thread);
	}
	os_thread_helper_unlock(&p->thread);

	if (changed) {
		GXP_DEBUG(p, "galaxyxr passthrough: cameras %s", enabled ? "enabled" : "disabled");
		wake_client_thread(p);
	}
}

static bool
acquire_frame(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *out_frame)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);
	// Refuse immediately after disable; the client thread tears the stream
	// down asynchronously, so stream_ready alone would lag the contract.
	if (!is_enabled(p)) {
		return false;
	}
	struct galaxyxr_frame_token *token = calloc(1, sizeof(*token));
	if (token == NULL) {
		return false;
	}
	struct xrt_passthrough_frame frame = {0};
	for (uint32_t i = 0; i < XRT_PASSTHROUGH_MAX_VIEWS; i++) {
		frame.views[i].handle = XRT_GRAPHICS_BUFFER_HANDLE_INVALID;
	}

	os_mutex_lock(&p->mutex);
	if (!p->stream_ready || !p->pending_valid) {
		os_mutex_unlock(&p->mutex);
		free(token);
		return false;
	}

	token->frame = p->pending.titan;
	token->generation = p->generation;
	frame.generation = p->generation;
	frame.format = XRT_PASSTHROUGH_FORMAT_NV12;
	frame.view_count = p->info.num_views;
	frame.width = p->info.width;
	frame.height = p->info.height;
	frame.plane_count = 2;
	frame.strides[0] = p->info.stride;
	frame.strides[1] = p->info.stride;
	frame.offsets[0] = 0;
	frame.offsets[1] = p->info.chroma_offset;
	frame.buffer_size = p->info.buf_size;
	frame.drm_format_modifier = 0; // DRM_FORMAT_MOD_LINEAR

	bool handles_ok = true;
	for (uint32_t v = 0; v < frame.view_count; v++) {
		const struct titan_wire_frame_view *source = &p->pending.titan.view[v];
		struct xrt_passthrough_frame_view *dest = &frame.views[v];
		dest->slot = source->slot;
		dest->frame_id = source->frame_id;
		dest->sensor_request_id = source->sensor_request_id;
		dest->exposure_request_id = source->exposure_request_id;
		dest->capture_pose_begin = p->pending.capture_pose_begin[v];
		dest->capture_pose_end = p->pending.capture_pose_end[v];
		dest->handle = dup(p->slot_fds[v][source->slot]);
		handles_ok = handles_ok && xrt_graphics_buffer_is_valid(dest->handle);
	}
	if (!handles_ok) {
		os_mutex_unlock(&p->mutex);
		for (uint32_t v = 0; v < frame.view_count; v++) {
			if (xrt_graphics_buffer_is_valid(frame.views[v].handle)) {
				close(frame.views[v].handle);
			}
		}
		free(token);
		return false;
	}

	p->pending_valid = false;
	token->next = p->held;
	p->held = token;
	p->frames_acquired++;
	os_mutex_unlock(&p->mutex);

	frame.token = token;
	*out_frame = frame;
	return true;
}

static bool
is_frame_valid(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);
	const struct galaxyxr_frame_token *token = frame->token;

	os_mutex_lock(&p->mutex);
	bool valid = p->stream_ready && token->generation == p->generation;
	os_mutex_unlock(&p->mutex);
	return valid;
}

static void
mark_frame_used(struct xrt_passthrough_stream *xp, const struct xrt_passthrough_frame *frame)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);
	const struct galaxyxr_frame_token *token = frame->token;
	struct timespec used_boottime = {0};
	if (clock_gettime(CLOCK_BOOTTIME, &used_boottime) != 0) {
		if (!p->feedback_clock_warned) {
			GXP_WARN(p, "galaxyxr passthrough: failed to sample CLOCK_BOOTTIME for FRAME_USED: %s",
			         strerror(errno));
			p->feedback_clock_warned = true;
		}
		return;
	}
	uint64_t used_boottime_ns = (uint64_t)os_timespec_to_ns(&used_boottime);

	bool queue = false;
	os_mutex_lock(&p->mutex);
	if (p->stream_ready && p->info.controller && token->generation == p->generation &&
	    p->feedback_sequence != UINT64_MAX) {
		// Advance for every render tick, even if queued feedback is replaced.
		p->feedback_sequence++;
		if (p->feedback.pending) {
			atomic_fetch_add_explicit(&p->feedback_dropped, 1, memory_order_relaxed);
		}
		p->feedback = (struct galaxyxr_frame_used_feedback){
		    .pending = true,
		    .generation = token->generation,
		    .use_sequence = p->feedback_sequence,
		    .used_boottime_ns = used_boottime_ns,
		    .frame = token->frame,
		};
		queue = true;
	}
	os_mutex_unlock(&p->mutex);

	if (queue) {
		wake_client_thread(p);
	}
}

static void
release_frame(struct xrt_passthrough_stream *xp, struct xrt_passthrough_frame *frame)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);
	struct galaxyxr_frame_token *token = frame->token;
	for (uint32_t v = 0; v < frame->view_count; v++) {
		if (xrt_graphics_buffer_is_valid(frame->views[v].handle)) {
			close(frame->views[v].handle);
			frame->views[v].handle = XRT_GRAPHICS_BUFFER_HANDLE_INVALID;
		}
	}

	os_mutex_lock(&p->mutex);
	struct galaxyxr_frame_token **link = &p->held;
	while (*link != NULL && *link != token) {
		link = &(*link)->next;
	}
	bool found = *link == token;
	if (found) {
		*link = token->next;
	}
	bool queue = found && p->stream_ready && token->generation == p->generation;
	if (queue) {
		token->next = NULL;
		*p->release_tail = token;
		p->release_tail = &token->next;
	}
	os_mutex_unlock(&p->mutex);

	if (queue) {
		wake_client_thread(p);
	} else {
		free(token);
	}
	memset(frame, 0, sizeof(*frame));
}

static void
destroy_provider(struct xrt_passthrough_stream *xp)
{
	struct galaxyxr_passthrough *p = galaxyxr_passthrough(xp);
	if (p->thread.initialized) {
		// Ensure the client cannot re-enter poll before destroy marks it stopped.
		os_thread_helper_lock(&p->thread);
		p->enabled = false;
		os_thread_helper_signal_locked(&p->thread);
		os_thread_helper_unlock(&p->thread);
		wake_client_thread(p);
		os_thread_helper_destroy(&p->thread);
	}

	os_mutex_lock(&p->mutex);
	struct galaxyxr_frame_token *held = p->held;
	struct galaxyxr_frame_token *releases = p->release_head;
	p->held = NULL;
	p->release_head = NULL;
	os_mutex_unlock(&p->mutex);
	if (held != NULL) {
		GXP_WARN(p, "galaxyxr passthrough: destroying provider with camera frames still held");
	}
	free_token_list(held);
	free_token_list(releases);
	close(p->wake_fd);
	os_mutex_destroy(&p->mutex);
	free(p);
}

struct xrt_passthrough_stream *
galaxyxr_passthrough_create(const char *calibration_path, struct m_relation_history *qtimer_relation_history)
{
	if (!debug_get_bool_option_galaxyxr_passthrough()) {
		return NULL;
	}
	if (qtimer_relation_history == NULL) {
		return NULL;
	}

	struct galaxyxr_passthrough *p = U_TYPED_CALLOC(struct galaxyxr_passthrough);
	atomic_init(&p->feedback_sent, 0);
	atomic_init(&p->feedback_dropped, 0);
	p->log_level = debug_get_log_option_galaxyxr_passthrough_log();
	p->qtimer_relation_history = qtimer_relation_history;
	p->titan_fd = -1;
	p->wake_fd = -1;
	for (uint32_t v = 0; v < TITAN_PROTO_MAX_VIEWS; v++) {
		for (uint32_t s = 0; s < TITAN_PROTO_MAX_SLOTS; s++) {
			p->slot_fds[v][s] = -1;
		}
	}
	p->release_tail = &p->release_head;
	p->base.set_enabled = set_enabled;
	p->base.acquire_frame = acquire_frame;
	p->base.is_frame_valid = is_frame_valid;
	p->base.mark_frame_used = mark_frame_used;
	p->base.release_frame = release_frame;
	p->base.destroy = destroy_provider;

	const char *override = debug_get_option_galaxyxr_passthrough_calibration();
	const char *path = override != NULL ? override : calibration_path;
	if (galaxyxr_passthrough_calibration_load(&p->base.calibration, path) != 0) {
		free(p);
		return NULL;
	}
	GXP_INFO(p, "galaxyxr passthrough: calibration from %s, IPD %.2f mm", p->base.calibration.source_path,
	         p->base.calibration.calibration_ipd_mm);

	p->wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (p->wake_fd < 0) {
		GXP_ERROR(p, "galaxyxr passthrough: failed to create client wake eventfd: %s", strerror(errno));
		free(p);
		return NULL;
	}
	if (os_mutex_init(&p->mutex) != 0) {
		GXP_ERROR(p, "galaxyxr passthrough: failed to initialize mutex");
		close(p->wake_fd);
		free(p);
		return NULL;
	}
	if (os_thread_helper_init(&p->thread) != 0) {
		GXP_ERROR(p, "galaxyxr passthrough: failed to initialize thread state");
		os_mutex_destroy(&p->mutex);
		close(p->wake_fd);
		free(p);
		return NULL;
	}
	if (os_thread_helper_start(&p->thread, client_thread, p) != 0) {
		GXP_ERROR(p, "galaxyxr passthrough: failed to start client thread");
		os_thread_helper_destroy(&p->thread);
		os_mutex_destroy(&p->mutex);
		close(p->wake_fd);
		free(p);
		return NULL;
	}
	os_thread_helper_name(&p->thread, "Galaxy XR cameras");

	return &p->base;
}
