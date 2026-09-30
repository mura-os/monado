// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR dual DSI panel direct mode.
 *
 * The Galaxy XR eye panels are two separate DRM devices, one DSI connector
 * each, and a single panel can only be fed by ganging four 888 pixel wide
 * SSPP slices side by side. Vulkan display WSI cannot express any of that, so
 * this target leases both DSI connectors from the Wayland compositor
 * (drm-lease-v1, one lease per DRM device), renders into shared stereo
 * dma-bufs imported into Vulkan as linear DRM-format-modifier images, and
 * scans them out with atomic commits on the two lease fds.
 *
 * Buffer layout is one 7104x3840 image: x 0..3551 left eye (DSI-1/primary),
 * x 3552..7103 right eye (DSI-2/secondary).
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup comp_main
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <linux/dma-heap.h>
#include <linux/sync_file.h>

#include <wayland-client.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "drm-lease-v1-client-protocol.h"

#include "xrt/xrt_compiler.h"
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_pacing.h"
#include "main/comp_window.h"
#include "main/comp_window_galaxyxr_pacing.h"


#define GXR_EYE_W 3552
#define GXR_EYE_H 3840
#define GXR_STEREO_W (GXR_EYE_W * 2)
#define GXR_SLICE_W 888
#define GXR_NPLANES 4
#define GXR_NUM_IMAGES 3
#define GXR_MAX_PLANES 64
#define GXR_MAX_DEVS 4
#define GXR_MAX_CONNS 16
#define GXR_COMMIT_Q 8
// The kernel worker needs ~0.3 ms of plane programming plus the 1 ms flush
// window before the vsync; a commit picked up later slips a period.
#define GXR_PICKUP_DEADLINE_NS (3 * U_TIME_HALF_MS_IN_NS)

DEBUG_GET_ONCE_BOOL_OPTION(gxr_autodetect, "XRT_COMPOSITOR_GALAXYXR", true)
DEBUG_GET_ONCE_OPTION(gxr_dma_heap, "XRT_COMPOSITOR_GALAXYXR_DMA_HEAP", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(gxr_sync_fd, "XRT_COMPOSITOR_GALAXYXR_SYNC_FD", true)
DEBUG_GET_ONCE_BOOL_OPTION(gxr_ubwc, "XRT_COMPOSITOR_GALAXYXR_UBWC", true)
// The panels expose 3552x3840 at 90, 72 and 60 Hz.
DEBUG_GET_ONCE_NUM_OPTION(gxr_hz, "XRT_COMPOSITOR_GALAXYXR_HZ", 90)

#define GXR_ERROR(w, ...) COMP_ERROR((w)->base.c, __VA_ARGS__)
#define GXR_WARN(w, ...) COMP_WARN((w)->base.c, __VA_ARGS__)
#define GXR_INFO(w, ...) COMP_INFO((w)->base.c, __VA_ARGS__)
#define GXR_DEBUG(w, ...) COMP_DEBUG((w)->base.c, __VA_ARGS__)


/*
 *
 * Structs.
 *
 */

struct gxr_wl_dev
{
	struct comp_window_galaxyxr *w;
	struct wp_drm_lease_device_v1 *obj;
	bool done;
	//! Sysfs path of the lessor DRM device, stable across boots.
	char sysfs[256];
};

struct gxr_wl_conn
{
	struct comp_window_galaxyxr *w;
	struct wp_drm_lease_connector_v1 *obj;
	struct wp_drm_lease_device_v1 *dev;
	uint32_t id;
	char name[64];
	bool done;
};

struct gxr_prop
{
	uint32_t obj;
	uint32_t id;
	//! String literal, never freed.
	const char *name;
};

struct gxr_eye
{
	struct comp_window_galaxyxr *w;

	struct wp_drm_lease_v1 *lease;
	int fd;
	bool lease_finished;
	char wl_name[64];

	char name[80];
	bool is_left;
	int src_base;
	int crtc_idx;

	uint32_t conn;
	uint32_t crtc;
	uint32_t mode_blob;
	drmModeModeInfo mode;

	uint32_t planes[GXR_NPLANES];
	uint32_t all_planes[GXR_MAX_PLANES];
	uint32_t all_plane_count;

	uint32_t fbs[GXR_NUM_IMAGES];
	uint32_t handles[GXR_NUM_IMAGES];

	bool flip_pending;
	int64_t last_flip_ns;
	//! Like last_flip_ns but never consumed, for latch diagnostics.
	int64_t latest_flip_ns;

	struct gxr_prop props[256];
	uint32_t prop_count;
};

//! A recently committed frame, matched against left-eye flip events.
struct gxr_commit_rec
{
	int64_t frame_id;
	int64_t desired_ns;
	int64_t fence_sig_ns;
};

struct gxr_image
{
	VkImage image;
	VkImageView view;
	VkDeviceMemory memory;
	int dmabuf_fd;
	uint32_t pitch;

	uint32_t fb_plane_count;
	uint32_t fb_pitches[4];
	uint32_t fb_offsets[4];
};

struct comp_window_galaxyxr
{
	struct comp_target base;

	struct wl_display *display;
	struct wl_registry *registry;

	struct gxr_wl_dev devs[GXR_MAX_DEVS];
	uint32_t dev_count;
	struct gxr_wl_conn conns[GXR_MAX_CONNS];

	//! Wayland listener user data, stable addresses.
	struct gxr_eye eyes[2];
	//! Aliases into @ref eyes, valid after init_pre_vulkan.
	struct gxr_eye *left, *right;

	uint32_t acquired_index;

	struct gxr_image image_data[GXR_NUM_IMAGES];
	struct comp_target_image images[GXR_NUM_IMAGES];
	uint32_t fourcc;
	uint64_t modifier;

	VkFence submit_fences[GXR_NUM_IMAGES];

	//! Pass the render-complete fence to the display instead of blocking.
	bool use_sync_fd;

	struct u_pacing_compositor *upc;

	//! Frame id of the prediction the compositor is currently rendering.
	int64_t pacing_frame_id;

	/*!
	 * Turnip on kgsl has no VK_EXT_calibrated_timestamps, so the pacer
	 * would never get GPU feedback and its comp_time would stay at the
	 * minimum, making every frame miss its vblank. Instead the render
	 * fence's sync_file signal timestamp (CLOCK_MONOTONIC, from the DRM
	 * fence) is reported as gpu_end on a later present.
	 */
	struct
	{
		int fd;
		int64_t frame_id;
		int64_t submit_ns;
	} gpu_feedback[4];

	PFN_vkGetMemoryFdPropertiesKHR get_memory_fd_properties;
	PFN_vkGetFenceFdKHR get_fence_fd;

	bool modeset_done;
	bool has_images;

	//! Raw signal time of the last reported render fence (no margin).
	int64_t last_fence_signal_ns;

	/*!
	 * Commits not yet matched to a left-eye flip event. The kernel sends
	 * exactly one event per commit, in order, so the oldest record is an
	 * arriving event's owner, and the commit gate guarantees frames
	 * never latch before their desired time, which makes events older
	 * than the head record identifiable as spurious (duplicates at
	 * session start). Statistics only: the drain decision comes from the
	 * measured commit block.
	 */
	struct gxr_commit_rec commit_q[GXR_COMMIT_Q];
	uint32_t commit_q_head;
	uint32_t commit_q_tail;
	uint32_t commit_q_discards;

	//! Drop the next commit to drain the display queue back to one-deep.
	bool drain_pending;

	struct
	{
		uint32_t presents;
		int64_t last_report_ns;
		int64_t fence_ns;
		int64_t flip_ns;
		int64_t lead_ns;
		//! Latch error (flip - desired present) sum/max over the window.
		int64_t err_sum_ns, err_max_ns;
		uint32_t err_count;
		//! Frames that latched at least half a period late, and commits dropped to drain.
		uint32_t late, dropped;
		//! Fence signal -> own latch vsync, min/sum over matched frames.
		int64_t latch_min_ns, latch_sum_ns;
		uint32_t latch_count;
		//! Right eye flip - left eye flip, sum over the window.
		int64_t rl_sum_ns;
		uint32_t rl_count;
		//! Left eye flip-to-flip deltas, min/max/sum over the window.
		int64_t prev_flip_ns;
		int64_t period_min_ns, period_max_ns, period_sum_ns;
		uint32_t period_count;
	} stats;
};

static inline struct vk_bundle *
get_vk(struct comp_window_galaxyxr *w)
{
	return &w->base.c->base.vk;
}

static bool
gxr_is_galaxy_xr(void)
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


/*
 *
 * DRM property helpers.
 *
 */

static uint32_t
prop_id(struct gxr_eye *eye, uint32_t obj, uint32_t type, const char *name)
{
	for (uint32_t i = 0; i < eye->prop_count; i++) {
		if (eye->props[i].obj == obj && strcmp(eye->props[i].name, name) == 0) {
			return eye->props[i].id;
		}
	}

	drmModeObjectProperties *props = drmModeObjectGetProperties(eye->fd, obj, type);
	if (props == NULL) {
		return 0;
	}

	uint32_t id = 0;
	for (uint32_t i = 0; i < props->count_props && id == 0; i++) {
		drmModePropertyRes *p = drmModeGetProperty(eye->fd, props->props[i]);
		if (p != NULL && strcmp(p->name, name) == 0) {
			id = p->prop_id;
		}
		if (p != NULL) {
			drmModeFreeProperty(p);
		}
	}
	drmModeFreeObjectProperties(props);

	if (eye->prop_count < ARRAY_SIZE(eye->props)) {
		eye->props[eye->prop_count++] = (struct gxr_prop){.obj = obj, .id = id, .name = name};
	}
	return id;
}

static uint32_t
plane_type(int fd, uint32_t plane)
{
	drmModeObjectProperties *props = drmModeObjectGetProperties(fd, plane, DRM_MODE_OBJECT_PLANE);
	uint32_t type = 0xff;

	for (uint32_t i = 0; props != NULL && i < props->count_props; i++) {
		drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
		if (p != NULL && strcmp(p->name, "type") == 0) {
			type = (uint32_t)props->prop_values[i];
		}
		if (p != NULL) {
			drmModeFreeProperty(p);
		}
	}
	if (props != NULL) {
		drmModeFreeObjectProperties(props);
	}
	return type;
}

static bool
plane_has_format(int fd, uint32_t plane, uint32_t fourcc)
{
	drmModePlane *p = drmModeGetPlane(fd, plane);
	if (p == NULL) {
		return false;
	}

	bool has = false;
	for (uint32_t i = 0; i < p->count_formats; i++) {
		if (p->formats[i] == fourcc) {
			has = true;
			break;
		}
	}
	drmModeFreePlane(p);
	return has;
}

// The kernel remaps possible_crtcs masks for lessees, the bits index the CRTC
// list as seen through the lease fd.
static bool
plane_usable(int fd, uint32_t plane, int crtc_idx)
{
	drmModePlane *p = drmModeGetPlane(fd, plane);
	if (p == NULL) {
		return false;
	}
	bool ok = (p->possible_crtcs >> crtc_idx) & 1;
	drmModeFreePlane(p);

	return ok && plane_type(fd, plane) != DRM_PLANE_TYPE_CURSOR && plane_has_format(fd, plane, DRM_FORMAT_XRGB8888);
}

struct gxr_plane_info
{
	uint32_t plane;
	long pipe_idx;
	bool vig;
	bool virt;
	bool dma;
};

static char *
plane_caps_text(int fd, uint32_t plane)
{
	drmModeObjectProperties *props = drmModeObjectGetProperties(fd, plane, DRM_MODE_OBJECT_PLANE);
	char *text = NULL;

	for (uint32_t i = 0; props != NULL && i < props->count_props && text == NULL; i++) {
		drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
		if (p != NULL && strcmp(p->name, "capabilities") == 0 && props->prop_values[i] != 0) {
			drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd, props->prop_values[i]);
			if (blob != NULL) {
				text = strndup(blob->data, blob->length);
				drmModeFreePropertyBlob(blob);
			}
		}
		if (p != NULL) {
			drmModeFreeProperty(p);
		}
	}
	if (props != NULL) {
		drmModeFreeObjectProperties(props);
	}
	return text;
}

static bool
caps_int(const char *caps, const char *key, long *out)
{
	size_t klen = strlen(key);
	const char *l = caps;

	while (l != NULL && *l != '\0') {
		if (strncmp(l, key, klen) == 0 && l[klen] == '=') {
			*out = strtol(l + klen + 1, NULL, 10);
			return true;
		}
		l = strchr(l, '\n');
		if (l != NULL) {
			l++;
		}
	}
	return false;
}

// Reads the SDE "capabilities" blob so the 1:1 scanout path can prefer
// DMA pipes, falling back to VIG pipes or discovery order when needed.
static struct gxr_plane_info
plane_info_get(int fd, uint32_t plane, int fallback_order)
{
	struct gxr_plane_info pi = {.plane = plane, .pipe_idx = 1000 + fallback_order};
	char *caps = plane_caps_text(fd, plane);
	long v;

	if (caps == NULL) {
		return pi;
	}

	if (caps_int(caps, "pipe_idx", &v)) {
		pi.pipe_idx = v;
	}
	pi.vig = caps_int(caps, "max_upscale", &v) && v > 1;
	pi.virt = caps_int(caps, "primary_smart_plane_id", &v);
	pi.dma = !pi.vig;
	free(caps);
	return pi;
}

static int
mode_hz(const drmModeModeInfo *m)
{
	if (m->clock && m->htotal && m->vtotal) {
		return (int)((m->clock * 1000LL + (m->htotal * m->vtotal / 2)) / ((long long)m->htotal * m->vtotal));
	}
	return m->vrefresh;
}

//! Exact frame period from the pixel clock; vrefresh is rounded to whole Hz
//! and the ~0.1 ms/frame error makes the pacer predict a perpetually
//! drifting vblank.
static int64_t
mode_period_ns(const drmModeModeInfo *m)
{
	if (m->clock && m->htotal && m->vtotal) {
		return ((int64_t)m->htotal * m->vtotal * 1000000LL) / m->clock;
	}
	return U_TIME_1S_IN_NS / (m->vrefresh ? m->vrefresh : 90);
}

static drmModeModeInfo
choose_mode(const drmModeConnector *conn)
{
	int want_hz = (int)debug_get_num_option_gxr_hz();
	for (int i = 0; i < conn->count_modes; i++) {
		if (mode_hz(&conn->modes[i]) == want_hz) {
			return conn->modes[i];
		}
	}
	for (int i = 0; i < conn->count_modes; i++) {
		if (mode_hz(&conn->modes[i]) == 90) {
			return conn->modes[i];
		}
	}
	return conn->modes[0];
}


/*
 *
 * Wayland drm-lease-v1 acquisition.
 *
 */

static struct gxr_wl_conn *
conn_by_obj(struct comp_window_galaxyxr *w, struct wp_drm_lease_connector_v1 *obj)
{
	for (uint32_t i = 0; i < GXR_MAX_CONNS; i++) {
		if (w->conns[i].obj == obj) {
			return &w->conns[i];
		}
	}
	return NULL;
}

static void
_conn_name(void *data, struct wp_drm_lease_connector_v1 *obj, const char *name)
{
	struct gxr_wl_conn *c = conn_by_obj(data, obj);
	if (c != NULL) {
		snprintf(c->name, sizeof(c->name), "%s", name);
	}
}

static void
_conn_description(void *data, struct wp_drm_lease_connector_v1 *obj, const char *desc)
{}

static void
_conn_connector_id(void *data, struct wp_drm_lease_connector_v1 *obj, uint32_t id)
{
	struct gxr_wl_conn *c = conn_by_obj(data, obj);
	if (c != NULL) {
		c->id = id;
	}
}

static void
_conn_done(void *data, struct wp_drm_lease_connector_v1 *obj)
{
	struct comp_window_galaxyxr *w = data;
	struct gxr_wl_conn *c = conn_by_obj(w, obj);
	if (c != NULL) {
		c->done = true;
		GXR_DEBUG(w, "Advertised lease connector: %s (id %u)", c->name, c->id);
	}
}

static void
_conn_withdrawn(void *data, struct wp_drm_lease_connector_v1 *obj)
{
	struct comp_window_galaxyxr *w = data;
	struct gxr_wl_conn *c = conn_by_obj(w, obj);
	if (c != NULL) {
		GXR_WARN(w, "Lease connector %s withdrawn", c->name);
		memset(c, 0, sizeof(*c));
	}
	wp_drm_lease_connector_v1_destroy(obj);
}

static const struct wp_drm_lease_connector_v1_listener conn_listener = {
    .name = _conn_name,
    .description = _conn_description,
    .connector_id = _conn_connector_id,
    .done = _conn_done,
    .withdrawn = _conn_withdrawn,
};

static struct gxr_wl_dev *
dev_by_obj(struct comp_window_galaxyxr *w, struct wp_drm_lease_device_v1 *obj)
{
	for (uint32_t i = 0; i < w->dev_count; i++) {
		if (w->devs[i].obj == obj) {
			return &w->devs[i];
		}
	}
	return NULL;
}

static void
_dev_drm_fd(void *data, struct wp_drm_lease_device_v1 *obj, int fd)
{
	struct comp_window_galaxyxr *w = data;
	struct gxr_wl_dev *d = dev_by_obj(w, obj);

	struct stat st;
	if (d != NULL && fstat(fd, &st) == 0) {
		char link[64];
		snprintf(link, sizeof(link), "/sys/dev/char/%u:%u", major(st.st_rdev), minor(st.st_rdev));
		ssize_t n = readlink(link, d->sysfs, sizeof(d->sysfs) - 1);
		if (n > 0) {
			d->sysfs[n] = '\0';
		}
	}
	close(fd);
}

static void
_dev_connector(void *data, struct wp_drm_lease_device_v1 *obj, struct wp_drm_lease_connector_v1 *connector)
{
	struct comp_window_galaxyxr *w = data;
	for (uint32_t i = 0; i < GXR_MAX_CONNS; i++) {
		if (w->conns[i].obj != NULL) {
			continue;
		}
		w->conns[i].obj = connector;
		w->conns[i].dev = obj;
		w->conns[i].w = w;
		wp_drm_lease_connector_v1_add_listener(connector, &conn_listener, w);
		return;
	}
	wp_drm_lease_connector_v1_destroy(connector);
}

static void
_dev_done(void *data, struct wp_drm_lease_device_v1 *obj)
{
	struct comp_window_galaxyxr *w = data;
	for (uint32_t i = 0; i < w->dev_count; i++) {
		if (w->devs[i].obj == obj) {
			w->devs[i].done = true;
		}
	}
}

static void
_dev_released(void *data, struct wp_drm_lease_device_v1 *obj)
{}

static const struct wp_drm_lease_device_v1_listener dev_listener = {
    .drm_fd = _dev_drm_fd,
    .connector = _dev_connector,
    .done = _dev_done,
    .released = _dev_released,
};

static void
_lease_fd(void *data, struct wp_drm_lease_v1 *lease, int fd)
{
	struct gxr_eye *eye = data;
	if (eye->fd >= 0) {
		close(fd);
		return;
	}
	eye->fd = fd;
}

static void
_lease_finished(void *data, struct wp_drm_lease_v1 *lease)
{
	struct gxr_eye *eye = data;
	eye->lease_finished = true;
	COMP_WARN(eye->w->base.c, "Lease %s finished (denied or revoked by the compositor)", eye->wl_name);
}

static const struct wp_drm_lease_v1_listener lease_listener = {
    .lease_fd = _lease_fd,
    .finished = _lease_finished,
};

static void
_registry_global(void *data, struct wl_registry *registry, uint32_t name, const char *interface, uint32_t version)
{
	struct comp_window_galaxyxr *w = data;
	if (strcmp(interface, wp_drm_lease_device_v1_interface.name) != 0 || w->dev_count >= GXR_MAX_DEVS) {
		return;
	}
	struct gxr_wl_dev *d = &w->devs[w->dev_count++];
	d->w = w;
	d->obj = wl_registry_bind(registry, name, &wp_drm_lease_device_v1_interface, 1);
	wp_drm_lease_device_v1_add_listener(d->obj, &dev_listener, w);
}

static void
_registry_global_remove(void *data, struct wl_registry *registry, uint32_t name)
{}

static const struct wl_registry_listener registry_listener = {
    .global = _registry_global,
    .global_remove = _registry_global_remove,
};

static int
wl_pump(struct wl_display *dpy, int timeout_ms)
{
	while (wl_display_prepare_read(dpy) != 0) {
		if (wl_display_dispatch_pending(dpy) < 0) {
			return -1;
		}
	}
	wl_display_flush(dpy);

	struct pollfd pf = {.fd = wl_display_get_fd(dpy), .events = POLLIN};
	int rc = poll(&pf, 1, timeout_ms);
	if (rc > 0 && (pf.revents & POLLIN)) {
		if (wl_display_read_events(dpy) < 0) {
			return -1;
		}
	} else {
		wl_display_cancel_read(dpy);
		if (rc < 0 && errno != EINTR) {
			return -1;
		}
	}
	return wl_display_dispatch_pending(dpy);
}

static uint32_t
dsi_conns_ready(struct comp_window_galaxyxr *w)
{
	uint32_t n = 0;
	for (uint32_t i = 0; i < GXR_MAX_CONNS; i++) {
		if (w->conns[i].obj != NULL && w->conns[i].done && strncmp(w->conns[i].name, "DSI", 3) == 0) {
			n++;
		}
	}
	return n;
}

static bool
all_devs_done(struct comp_window_galaxyxr *w)
{
	for (uint32_t i = 0; i < w->dev_count; i++) {
		if (!w->devs[i].done) {
			return false;
		}
	}
	return w->dev_count > 0;
}

// Which panel is which eye, stable across boots: DSI-N numbering follows
// probe order, but the left panel always hangs off the ae00000 MDSS, which
// is also the one with the extra (DP/Virtual) connectors.
static bool
gxr_dev_is_left(struct comp_window_galaxyxr *w, struct gxr_wl_dev *d, const char **out_why)
{
	if (strstr(d->sysfs, "ae00000") != NULL) {
		*out_why = "device path ae00000";
		return true;
	}
	if (strstr(d->sysfs, "15600000") != NULL) {
		*out_why = "device path 15600000";
		return false;
	}

	*out_why = "non-DSI connector presence";
	for (uint32_t i = 0; i < GXR_MAX_CONNS; i++) {
		struct gxr_wl_conn *c = &w->conns[i];
		if (c->obj != NULL && c->dev == d->obj && c->done && strncmp(c->name, "DSI", 3) != 0) {
			return true;
		}
	}
	return false;
}

// One lease request per DSI bearing device, the protocol forbids mixing
// connectors from different devices in one request. Only request the panel DSI
// connector, even if the same device also advertises DP or Virtual connectors.
static bool
gxr_acquire_leases(struct comp_window_galaxyxr *w)
{
	int64_t deadline = os_monotonic_get_ns() + (int64_t)5 * U_TIME_1S_IN_NS;
	while (os_monotonic_get_ns() < deadline && !(all_devs_done(w) && dsi_conns_ready(w) >= 2)) {
		if (wl_pump(w->display, 100) < 0) {
			GXR_ERROR(w, "Wayland connection error while waiting for connectors");
			return false;
		}
	}

	if (dsi_conns_ready(w) < 2) {
		GXR_ERROR(w, "Need 2 leasable DSI connectors, got %u", dsi_conns_ready(w));
		return false;
	}

	uint32_t nleases = 0;

	for (uint32_t i = 0; i < w->dev_count && nleases < 2; i++) {
		struct gxr_wl_dev *d = &w->devs[i];
		struct gxr_wl_conn *dsi = NULL;
		for (uint32_t j = 0; j < GXR_MAX_CONNS; j++) {
			struct gxr_wl_conn *c = &w->conns[j];
			if (c->obj != NULL && c->dev == d->obj && c->done && strncmp(c->name, "DSI", 3) == 0) {
				dsi = c;
				break;
			}
		}
		if (dsi == NULL) {
			continue;
		}

		struct gxr_eye *eye = &w->eyes[nleases];
		snprintf(eye->wl_name, sizeof(eye->wl_name), "%s", dsi->name);

		const char *why = NULL;
		eye->is_left = gxr_dev_is_left(w, d, &why);
		GXR_INFO(w, "%s is the %s eye (%s), lessor %s", dsi->name, eye->is_left ? "left" : "right", why,
		         d->sysfs[0] != '\0' ? d->sysfs : "<unknown>");

		struct wp_drm_lease_request_v1 *req = wp_drm_lease_device_v1_create_lease_request(d->obj);
		wp_drm_lease_request_v1_request_connector(req, dsi->obj);
		GXR_INFO(w, "Requesting lease of %s (id %u)", dsi->name, dsi->id);
		eye->lease = wp_drm_lease_request_v1_submit(req);
		wp_drm_lease_v1_add_listener(eye->lease, &lease_listener, eye);
		nleases++;
	}

	if (nleases != 2) {
		GXR_ERROR(w, "Expected the two DSI panels on two DRM devices, got %u lease(s)", nleases);
		return false;
	}

	deadline = os_monotonic_get_ns() + (int64_t)5 * U_TIME_1S_IN_NS;
	for (;;) {
		if (w->eyes[0].fd >= 0 && w->eyes[1].fd >= 0) {
			return true;
		}
		if (w->eyes[0].lease_finished || w->eyes[1].lease_finished || os_monotonic_get_ns() >= deadline) {
			GXR_ERROR(w, "Leases not granted (%d and %d)", w->eyes[0].fd, w->eyes[1].fd);
			return false;
		}
		if (wl_pump(w->display, 100) < 0) {
			GXR_ERROR(w, "Wayland connection error while waiting for leases");
			return false;
		}
	}
}


/*
 *
 * Eye (leased DRM device) setup and scanout.
 *
 */

static bool
gxr_eye_setup(struct comp_window_galaxyxr *w, struct gxr_eye *eye)
{
	eye->w = w;
	drmSetClientCap(eye->fd, DRM_CLIENT_CAP_ATOMIC, 1);
	drmSetClientCap(eye->fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

	drmModeRes *res = drmModeGetResources(eye->fd);
	if (res == NULL) {
		GXR_ERROR(w, "[%s] getting resources on lease fd failed: %s", eye->wl_name, strerror(errno));
		return false;
	}

	drmModeConnector *conn = NULL;
	for (int i = 0; i < res->count_connectors; i++) {
		drmModeConnector *c = drmModeGetConnector(eye->fd, res->connectors[i]);
		if (c != NULL && c->connection == DRM_MODE_CONNECTED && c->connector_type == DRM_MODE_CONNECTOR_DSI &&
		    c->count_modes > 0) {
			conn = c;
			break;
		}
		if (c != NULL) {
			drmModeFreeConnector(c);
		}
	}
	if (conn == NULL) {
		GXR_ERROR(w, "[%s] no connected DSI connector in the lease", eye->wl_name);
		drmModeFreeResources(res);
		return false;
	}

	eye->conn = conn->connector_id;
	eye->src_base = eye->is_left ? 0 : GXR_EYE_W;
	snprintf(eye->name, sizeof(eye->name), "%s/%s", eye->wl_name, eye->is_left ? "left" : "right");
	eye->mode = choose_mode(conn);

	if (eye->mode.hdisplay != GXR_EYE_W || eye->mode.vdisplay != GXR_EYE_H) {
		GXR_WARN(w, "[%s] unexpected mode %dx%d", eye->name, eye->mode.hdisplay, eye->mode.vdisplay);
	}

	for (int i = 0; i < conn->count_encoders && eye->crtc == 0; i++) {
		drmModeEncoder *enc = drmModeGetEncoder(eye->fd, conn->encoders[i]);
		if (enc == NULL) {
			continue;
		}
		for (int j = 0; j < res->count_crtcs; j++) {
			if (enc->possible_crtcs & (1u << j)) {
				eye->crtc = res->crtcs[j];
				eye->crtc_idx = j;
				break;
			}
		}
		drmModeFreeEncoder(enc);
	}
	drmModeFreeConnector(conn);
	drmModeFreeResources(res);

	if (eye->crtc == 0) {
		GXR_ERROR(w, "[%s] no usable CRTC in the lease", eye->name);
		return false;
	}

	drmModePlaneRes *pr = drmModeGetPlaneResources(eye->fd);
	if (pr == NULL) {
		GXR_ERROR(w, "[%s] getting planes on lease fd failed: %s", eye->name, strerror(errno));
		return false;
	}

	uint32_t cand[GXR_MAX_PLANES];
	uint32_t ncand = 0;
	for (uint32_t i = 0; i < pr->count_planes; i++) {
		if (plane_type(eye->fd, pr->planes[i]) != DRM_PLANE_TYPE_CURSOR &&
		    eye->all_plane_count < GXR_MAX_PLANES) {
			eye->all_planes[eye->all_plane_count++] = pr->planes[i];
		}
		if (plane_usable(eye->fd, pr->planes[i], eye->crtc_idx) && ncand < GXR_MAX_PLANES) {
			cand[ncand++] = pr->planes[i];
		}
	}
	uint32_t nleased = pr->count_planes;
	drmModeFreePlaneResources(pr);

	if (ncand < GXR_NPLANES) {
		GXR_ERROR(w, "[%s] lease grants %u planes, only %u usable, need %d", eye->name, nleased, ncand,
		          GXR_NPLANES);
		return false;
	}

	struct gxr_plane_info info[GXR_MAX_PLANES];
	uint32_t ndma = 0, nvig = 0, nvirt = 0;
	for (uint32_t i = 0; i < ncand; i++) {
		info[i] = plane_info_get(eye->fd, cand[i], (int)i);
		bool dma_column =
		    info[i].dma && !info[i].virt && info[i].pipe_idx >= 0 && info[i].pipe_idx < (long)GXR_NPLANES;
		ndma += dma_column;
		nvig += info[i].vig;
		nvirt += info[i].vig && info[i].virt;
	}

	bool want_dma = ndma >= GXR_NPLANES;
	bool want_vig = !want_dma && nvig >= GXR_NPLANES;
	bool want_virt = want_vig && nvirt >= GXR_NPLANES;
	struct gxr_plane_info sel[GXR_MAX_PLANES];
	uint32_t nsel = 0;
	for (uint32_t i = 0; i < ncand; i++) {
		if (want_dma) {
			bool dma_column = info[i].dma && !info[i].virt && info[i].pipe_idx >= 0 &&
			                  info[i].pipe_idx < (long)GXR_NPLANES;
			if (!dma_column) {
				continue;
			}
		} else {
			if (want_vig && !info[i].vig) {
				continue;
			}
			if (want_virt && !info[i].virt) {
				continue;
			}
		}
		sel[nsel++] = info[i];
	}

	for (uint32_t i = 1; i < nsel; i++) {
		struct gxr_plane_info key = sel[i];
		int j = (int)i - 1;
		while (j >= 0 && sel[j].pipe_idx > key.pipe_idx) {
			sel[j + 1] = sel[j];
			j--;
		}
		sel[j + 1] = key;
	}

	for (uint32_t s = 0; s < GXR_NPLANES; s++) {
		eye->planes[s] = sel[s].plane;
	}

	if (drmModeCreatePropertyBlob(eye->fd, &eye->mode, sizeof(eye->mode), &eye->mode_blob) != 0) {
		GXR_ERROR(w, "[%s] mode blob creation failed: %s", eye->name, strerror(errno));
		return false;
	}

	GXR_INFO(w,
	         "[%s] %s conn=%u crtc=%u src_base=%d planes=%u,%u,%u,%u pipe_idx=%ld,%ld,%ld,%ld (%s) "
	         "%dx%d@%d",
	         eye->wl_name, eye->name, eye->conn, eye->crtc, eye->src_base, eye->planes[0], eye->planes[1],
	         eye->planes[2], eye->planes[3], sel[0].pipe_idx, sel[1].pipe_idx, sel[2].pipe_idx, sel[3].pipe_idx,
	         want_dma    ? "DMA"
	         : want_virt ? "virtual VIG"
	         : want_vig  ? "VIG"
	                     : "no caps blob, first granted",
	         eye->mode.hdisplay, eye->mode.vdisplay, mode_hz(&eye->mode));

	return true;
}

static bool
plane_selected(const struct gxr_eye *eye, uint32_t plane)
{
	for (uint32_t i = 0; i < GXR_NPLANES; i++) {
		if (eye->planes[i] == plane) {
			return true;
		}
	}
	return false;
}

static void
detach_plane(drmModeAtomicReq *req, struct gxr_eye *eye, uint32_t plane)
{
	uint32_t fb = prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, "FB_ID");
	uint32_t crtc = prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
	if (fb) {
		drmModeAtomicAddProperty(req, plane, fb, 0);
	}
	if (crtc) {
		drmModeAtomicAddProperty(req, plane, crtc, 0);
	}
}

static void
add_plane(drmModeAtomicReq *req, struct gxr_eye *eye, uint32_t slice, uint32_t fb, int in_fence_fd)
{
	uint32_t plane = eye->planes[slice];
	int dst_x = (int)(slice * GXR_SLICE_W);
	int src_x = eye->src_base + dst_x;

#define AP(name, value) drmModeAtomicAddProperty(req, plane, prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, name), (value))
	AP("FB_ID", fb);
	AP("CRTC_ID", eye->crtc);
	AP("SRC_X", (uint64_t)src_x << 16);
	AP("SRC_Y", 0);
	AP("SRC_W", (uint64_t)GXR_SLICE_W << 16);
	AP("SRC_H", (uint64_t)GXR_EYE_H << 16);
	AP("CRTC_X", (uint64_t)dst_x);
	AP("CRTC_Y", 0);
	AP("CRTC_W", GXR_SLICE_W);
	AP("CRTC_H", GXR_EYE_H);
#undef AP

	uint32_t p = prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, "zpos");
	if (p) {
		drmModeAtomicAddProperty(req, plane, p, 0);
	}
	p = prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, "alpha");
	if (p) {
		drmModeAtomicAddProperty(req, plane, p, 255);
	}
	if (in_fence_fd >= 0) {
		p = prop_id(eye, plane, DRM_MODE_OBJECT_PLANE, "IN_FENCE_FD");
		if (p) {
			drmModeAtomicAddProperty(req, plane, p, (uint64_t)(int64_t)in_fence_fd);
		}
	}
}

static int
commit_eye(struct comp_window_galaxyxr *w, struct gxr_eye *eye, uint32_t fb, bool first_modeset, int in_fence_fd)
{
	drmModeAtomicReq *req = drmModeAtomicAlloc();

	drmModeAtomicAddProperty(req, eye->conn, prop_id(eye, eye->conn, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID"),
	                         eye->crtc);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "MODE_ID"),
	                         eye->mode_blob);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "ACTIVE"), 1);

	if (first_modeset) {
		for (uint32_t i = 0; i < eye->all_plane_count; i++) {
			if (!plane_selected(eye, eye->all_planes[i])) {
				detach_plane(req, eye, eye->all_planes[i]);
			}
		}
	}
	for (uint32_t s = 0; s < GXR_NPLANES; s++) {
		add_plane(req, eye, s, fb, in_fence_fd);
	}

	uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	void *user = NULL;
	if (!first_modeset) {
		flags |= DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT;
		user = eye;
	}

	int rc = drmModeAtomicCommit(eye->fd, req, flags, user);
	drmModeAtomicFree(req);

	if (rc == 0 && !first_modeset) {
		eye->flip_pending = true;
	}
	if (rc != 0) {
		GXR_ERROR(w, "[%s] atomic commit failed: %s", eye->name, strerror(errno));
	}
	return rc;
}

static void
gxr_match_left_flip(struct comp_window_galaxyxr *w, int64_t flip_ns)
{
	if (w->commit_q_head == w->commit_q_tail) {
		return;
	}
	struct gxr_commit_rec *rec = &w->commit_q[w->commit_q_head % GXR_COMMIT_Q];
	int64_t period_ns = mode_period_ns(&w->left->mode);
	int64_t err_ns = flip_ns - rec->desired_ns;

	if (err_ns < -period_ns / 4) {
		if (++w->commit_q_discards >= 2) {
			GXR_DEBUG(w, "latch matcher lost sync, resetting");
			w->commit_q_head = w->commit_q_tail;
			w->commit_q_discards = 0;
		}
		return;
	}
	w->commit_q_discards = 0;
	w->commit_q_head++;

	w->stats.err_sum_ns += err_ns;
	if (w->stats.err_count == 0 || err_ns > w->stats.err_max_ns) {
		w->stats.err_max_ns = err_ns;
	}
	w->stats.err_count++;

	if (err_ns > period_ns / 2) {
		w->stats.late++;
	}

	if (rec->fence_sig_ns != 0 && flip_ns > rec->fence_sig_ns) {
		int64_t latch_ns = flip_ns - rec->fence_sig_ns;
		if (w->stats.latch_count == 0 || latch_ns < w->stats.latch_min_ns) {
			w->stats.latch_min_ns = latch_ns;
		}
		w->stats.latch_sum_ns += latch_ns;
		w->stats.latch_count++;
	}
}

static void
page_flip_handler2(
    int fd, unsigned int seq, unsigned int tv_sec, unsigned int tv_usec, unsigned int crtc_id, void *data)
{
	struct gxr_eye *eye = data;
	eye->flip_pending = false;
	int64_t event_ns = (int64_t)tv_sec * U_TIME_1S_IN_NS + (int64_t)tv_usec * U_TIME_1MS_IN_NS / 1000;
	uint64_t crtc_seq = 0;
	uint64_t seq_ns = 0;
	// Keep the DRM vblank timestamp path live. Without this, the
	// downstream kernel can report zero timestamps in page-flip events.
	bool have_seq_time = drmCrtcGetSequence(fd, eye->crtc, &crtc_seq, &seq_ns) == 0 && seq_ns != 0;
	if (event_ns != 0) {
		eye->last_flip_ns = event_ns;
	} else if (have_seq_time) {
		eye->last_flip_ns = (int64_t)seq_ns;
	} else {
		// Last resort: event delivery time, not the actual latch time.
		eye->last_flip_ns = os_monotonic_get_ns();
	}

	eye->latest_flip_ns = eye->last_flip_ns;
	if (eye->is_left) {
		gxr_match_left_flip(eye->w, eye->latest_flip_ns);
	}
	U_LOG_D("trace flip %s %.3f", eye->name, time_ns_to_ms_f(eye->last_flip_ns % 1000000000));
}

static void
page_flip_handler(int fd, unsigned int seq, unsigned int tv_sec, unsigned int tv_usec, void *data)
{
	page_flip_handler2(fd, seq, tv_sec, tv_usec, 0, data);
}

static bool
gxr_wait_flips_mask(struct comp_window_galaxyxr *w, int timeout_ms, bool wait_left, bool wait_right)
{
	drmEventContext ev = {
	    .version = DRM_EVENT_CONTEXT_VERSION,
	    .page_flip_handler = page_flip_handler,
	    .page_flip_handler2 = page_flip_handler2,
	};
	int64_t deadline = os_monotonic_get_ns() + (int64_t)timeout_ms * U_TIME_1MS_IN_NS;

	while ((wait_left && w->left->flip_pending) || (wait_right && w->right->flip_pending)) {
		struct pollfd fds[2];
		struct gxr_eye *eyes[2];
		nfds_t nfds = 0;
		if (wait_left && w->left->flip_pending) {
			eyes[nfds] = w->left;
			fds[nfds++] = (struct pollfd){.fd = w->left->fd, .events = POLLIN};
		}
		if (wait_right && w->right->flip_pending) {
			eyes[nfds] = w->right;
			fds[nfds++] = (struct pollfd){.fd = w->right->fd, .events = POLLIN};
		}

		int timeout = (int)((deadline - os_monotonic_get_ns()) / U_TIME_1MS_IN_NS);
		if (timeout <= 0 || poll(fds, nfds, timeout) <= 0) {
			GXR_WARN(w, "Timed out waiting for page flip");
			w->left->flip_pending = false;
			w->right->flip_pending = false;
			return false;
		}
		for (nfds_t i = 0; i < nfds; i++) {
			if (fds[i].revents & POLLIN) {
				drmHandleEvent(eyes[i]->fd, &ev);
			}
		}
	}
	return true;
}

static bool
gxr_wait_flips(struct comp_window_galaxyxr *w, int timeout_ms)
{
	return gxr_wait_flips_mask(w, timeout_ms, true, true);
}

//! Read any already-delivered page flip events, never blocks.
static void
gxr_pump_flip_events(struct comp_window_galaxyxr *w)
{
	drmEventContext ev = {
	    .version = DRM_EVENT_CONTEXT_VERSION,
	    .page_flip_handler = page_flip_handler,
	    .page_flip_handler2 = page_flip_handler2,
	};
	struct pollfd fds[2] = {
	    {.fd = w->left->fd, .events = POLLIN},
	    {.fd = w->right->fd, .events = POLLIN},
	};
	struct gxr_eye *eyes[2] = {w->left, w->right};

	while (poll(fds, 2, 0) > 0) {
		for (uint32_t i = 0; i < 2; i++) {
			if (fds[i].revents & POLLIN) {
				drmHandleEvent(eyes[i]->fd, &ev);
			}
		}
	}
}

static void
gxr_eye_disable(struct comp_window_galaxyxr *w, struct gxr_eye *eye)
{
	if (eye->fd < 0 || eye->conn == 0) {
		return;
	}
	drmModeAtomicReq *req = drmModeAtomicAlloc();
	drmModeAtomicAddProperty(req, eye->conn, prop_id(eye, eye->conn, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID"), 0);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "ACTIVE"), 0);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "MODE_ID"), 0);
	for (uint32_t i = 0; i < eye->all_plane_count; i++) {
		detach_plane(req, eye, eye->all_planes[i]);
	}
	drmModeAtomicCommit(eye->fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
	drmModeAtomicFree(req);
}


/*
 *
 * Buffer allocation and Vulkan import.
 *
 */

static int
gxr_alloc_dumb(struct comp_window_galaxyxr *w, int *out_fd, uint32_t *out_pitch)
{
	uint32_t handle = 0, pitch = 0;
	uint64_t size = 0;
	if (drmModeCreateDumbBuffer(w->left->fd, GXR_STEREO_W, GXR_EYE_H, 32, 0, &handle, &pitch, &size) != 0) {
		return -errno;
	}

	int fd = -1;
	if (drmPrimeHandleToFD(w->left->fd, handle, DRM_CLOEXEC | DRM_RDWR, &fd) != 0) {
		int err = -errno;
		drmModeDestroyDumbBuffer(w->left->fd, handle);
		return err;
	}

	*out_fd = fd;
	*out_pitch = pitch;
	return 0;
}

static int
gxr_alloc_dma_heap(struct comp_window_galaxyxr *w, const char *path, int *out_fd, uint32_t *out_pitch)
{
	int heap = open(path, O_RDWR | O_CLOEXEC);
	if (heap < 0) {
		return -errno;
	}

	const uint32_t pitch = GXR_STEREO_W * 4;
	struct dma_heap_allocation_data data = {
	    .len = ((uint64_t)pitch * GXR_EYE_H + 4095) & ~UINT64_C(4095),
	    .fd_flags = O_RDWR | O_CLOEXEC,
	};
	int rc = ioctl(heap, DMA_HEAP_IOCTL_ALLOC, &data);
	int err = -errno;
	close(heap);
	if (rc != 0) {
		return err;
	}

	*out_fd = (int)data.fd;
	*out_pitch = pitch;
	return 0;
}

static bool
gxr_alloc_buffer(struct comp_window_galaxyxr *w, int *out_fd, uint32_t *out_pitch)
{
	const char *heap_override = debug_get_option_gxr_dma_heap();
	if (heap_override != NULL) {
		int ret = gxr_alloc_dma_heap(w, heap_override, out_fd, out_pitch);
		if (ret != 0) {
			GXR_ERROR(w, "dma_heap alloc from %s failed: %d", heap_override, ret);
			return false;
		}
		return true;
	}

	int ret = gxr_alloc_dumb(w, out_fd, out_pitch);
	if (ret == 0) {
		return true;
	}
	GXR_WARN(w, "Dumb buffer alloc failed (%d), trying dma_heaps", ret);

	static const char *heaps[] = {"/dev/dma_heap/qcom,display", "/dev/dma_heap/system"};
	for (uint32_t i = 0; i < ARRAY_SIZE(heaps); i++) {
		ret = gxr_alloc_dma_heap(w, heaps[i], out_fd, out_pitch);
		if (ret == 0) {
			return true;
		}
		GXR_WARN(w, "dma_heap alloc from %s failed: %d", heaps[i], ret);
	}

	GXR_ERROR(w, "All stereo buffer allocation paths failed");
	return false;
}

static uint32_t
vk_format_to_fourcc(VkFormat format)
{
	switch (format) {
	case VK_FORMAT_B8G8R8A8_UNORM:
	case VK_FORMAT_B8G8R8A8_SRGB: return DRM_FORMAT_XRGB8888;
	case VK_FORMAT_R8G8B8A8_UNORM:
	case VK_FORMAT_R8G8B8A8_SRGB: return DRM_FORMAT_XBGR8888;
	case VK_FORMAT_A2B10G10R10_UNORM_PACK32: return DRM_FORMAT_XBGR2101010;
	default: return 0;
	}
}

static bool
gxr_format_supported_by_planes(struct comp_window_galaxyxr *w, uint32_t fourcc)
{
	struct gxr_eye *eyes[2] = {w->left, w->right};
	for (uint32_t e = 0; e < 2; e++) {
		for (uint32_t p = 0; p < GXR_NPLANES; p++) {
			if (!plane_has_format(eyes[e]->fd, eyes[e]->planes[p], fourcc)) {
				return false;
			}
		}
	}
	return true;
}

static bool
gxr_format_supported_by_vk(struct comp_window_galaxyxr *w, VkFormat format, uint64_t modifier, VkImageUsageFlags usage)
{
	struct vk_bundle *vk = get_vk(w);

	VkPhysicalDeviceImageDrmFormatModifierInfoEXT mod_info = {
	    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_DRM_FORMAT_MODIFIER_INFO_EXT,
	    .drmFormatModifier = modifier,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	};
	VkPhysicalDeviceExternalImageFormatInfo ext_info = {
	    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_EXTERNAL_IMAGE_FORMAT_INFO,
	    .pNext = &mod_info,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkPhysicalDeviceImageFormatInfo2 info = {
	    .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_IMAGE_FORMAT_INFO_2,
	    .pNext = &ext_info,
	    .format = format,
	    .type = VK_IMAGE_TYPE_2D,
	    .tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT,
	    .usage = usage,
	};
	VkExternalImageFormatProperties ext_props = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_IMAGE_FORMAT_PROPERTIES,
	};
	VkImageFormatProperties2 props = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_FORMAT_PROPERTIES_2,
	    .pNext = &ext_props,
	};

	VkResult ret = vk->vkGetPhysicalDeviceImageFormatProperties2(vk->physical_device, &info, &props);
	if (ret != VK_SUCCESS) {
		return false;
	}

	return (ext_props.externalMemoryProperties.externalMemoryFeatures &
	        VK_EXTERNAL_MEMORY_FEATURE_IMPORTABLE_BIT) != 0;
}

// Prefer the 8888 formats over anything fancier, XRGB8888 scanout through the
// sliced planes is the proven configuration on this panel pipeline.
static bool
gxr_format_is_8888(VkFormat format)
{
	switch (format) {
	case VK_FORMAT_B8G8R8A8_UNORM:
	case VK_FORMAT_B8G8R8A8_SRGB:
	case VK_FORMAT_R8G8B8A8_UNORM:
	case VK_FORMAT_R8G8B8A8_SRGB: return true;
	default: return false;
	}
}

static VkFormat
gxr_pick_format(struct comp_window_galaxyxr *w,
                const struct comp_target_create_images_info *create_info,
                uint64_t modifier,
                uint32_t *out_fourcc)
{
	for (uint32_t pass = 0; pass < 2; pass++) {
		for (uint32_t i = 0; i < create_info->format_count; i++) {
			VkFormat format = create_info->formats[i];
			if (gxr_format_is_8888(format) != (pass == 0)) {
				continue;
			}
			uint32_t fourcc = vk_format_to_fourcc(format);
			if (fourcc == 0) {
				continue;
			}
			// The SDE only scans UBWC in the ABGR component orders.
			if (modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED && fourcc != DRM_FORMAT_XBGR8888 &&
			    fourcc != DRM_FORMAT_XBGR2101010) {
				continue;
			}
			if (!gxr_format_supported_by_vk(w, format, modifier, create_info->image_usage)) {
				GXR_DEBUG(w, "Format %u not usable as dma-buf image", format);
				continue;
			}
			if (!gxr_format_supported_by_planes(w, fourcc)) {
				GXR_DEBUG(w, "Format %.4s not supported by the leased planes", (const char *)&fourcc);
				continue;
			}
			*out_fourcc = fourcc;
			return format;
		}
	}
	return VK_FORMAT_UNDEFINED;
}

// Number of DRM fb memory planes turnip reports for a format + modifier.
static uint32_t
gxr_modifier_plane_count(struct comp_window_galaxyxr *w, VkFormat format, uint64_t modifier)
{
	struct vk_bundle *vk = get_vk(w);

	VkDrmFormatModifierPropertiesEXT mods[16];
	VkDrmFormatModifierPropertiesListEXT list = {
	    .sType = VK_STRUCTURE_TYPE_DRM_FORMAT_MODIFIER_PROPERTIES_LIST_EXT,
	};
	VkFormatProperties2 props = {.sType = VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_2, .pNext = &list};
	vk->vkGetPhysicalDeviceFormatProperties2(vk->physical_device, format, &props);
	if (list.drmFormatModifierCount > ARRAY_SIZE(mods)) {
		list.drmFormatModifierCount = ARRAY_SIZE(mods);
	}
	list.pDrmFormatModifierProperties = mods;
	vk->vkGetPhysicalDeviceFormatProperties2(vk->physical_device, format, &props);

	for (uint32_t i = 0; i < list.drmFormatModifierCount; i++) {
		if (mods[i].drmFormatModifier == modifier) {
			return mods[i].drmFormatModifierPlaneCount;
		}
	}
	return 1;
}

// Allocate a scanout image from Vulkan with a driver-chosen UBWC layout and
// export it as a dma-buf, the reverse of the dumb buffer import path.
static VkResult
gxr_alloc_image_ubwc(struct comp_window_galaxyxr *w, struct gxr_image *img, VkFormat format, VkImageUsageFlags usage)
{
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	uint64_t modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
	VkImageDrmFormatModifierListCreateInfoEXT mod_list = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT,
	    .drmFormatModifierCount = 1,
	    .pDrmFormatModifiers = &modifier,
	};
	VkExternalMemoryImageCreateInfo ext_image = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO,
	    .pNext = &mod_list,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkImageCreateInfo image_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &ext_image,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .extent = {GXR_STEREO_W, GXR_EYE_H, 1},
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT,
	    .usage = usage,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	ret = vk->vkCreateImage(vk->device, &image_info, NULL, &img->image);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkCreateImage(ubwc): %s", vk_result_string(ret));
		return ret;
	}

	VkMemoryRequirements mem_reqs;
	vk->vkGetImageMemoryRequirements(vk->device, img->image, &mem_reqs);
	if (mem_reqs.memoryTypeBits == 0) {
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}

	VkExportMemoryAllocateInfo export_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkMemoryDedicatedAllocateInfo dedicated = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO,
	    .pNext = &export_info,
	    .image = img->image,
	};
	VkMemoryAllocateInfo alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = &dedicated,
	    .allocationSize = mem_reqs.size,
	    .memoryTypeIndex = (uint32_t)__builtin_ctz(mem_reqs.memoryTypeBits),
	};
	ret = vk->vkAllocateMemory(vk->device, &alloc_info, NULL, &img->memory);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkAllocateMemory(ubwc): %s", vk_result_string(ret));
		return ret;
	}

	ret = vk->vkBindImageMemory(vk->device, img->image, img->memory, 0);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkBindImageMemory(ubwc): %s", vk_result_string(ret));
		return ret;
	}

	VkMemoryGetFdInfoKHR fd_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR,
	    .memory = img->memory,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	ret = vk->vkGetMemoryFdKHR(vk->device, &fd_info, &img->dmabuf_fd);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkGetMemoryFdKHR: %s", vk_result_string(ret));
		return ret;
	}

	img->fb_plane_count = gxr_modifier_plane_count(w, format, modifier);
	if (img->fb_plane_count > 4) {
		return VK_ERROR_FORMAT_NOT_SUPPORTED;
	}
	for (uint32_t p = 0; p < img->fb_plane_count; p++) {
		VkImageSubresource sub = {.aspectMask = VK_IMAGE_ASPECT_MEMORY_PLANE_0_BIT_EXT << p};
		VkSubresourceLayout layout;
		vk->vkGetImageSubresourceLayout(vk->device, img->image, &sub, &layout);
		img->fb_pitches[p] = (uint32_t)layout.rowPitch;
		img->fb_offsets[p] = (uint32_t)layout.offset;
	}
	img->pitch = img->fb_pitches[0];

	VkImageViewCreateInfo view_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
	    .image = img->image,
	    .viewType = VK_IMAGE_VIEW_TYPE_2D,
	    .format = format,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .levelCount = 1,
	            .layerCount = 1,
	        },
	};
	ret = vk->vkCreateImageView(vk->device, &view_info, NULL, &img->view);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkCreateImageView: %s", vk_result_string(ret));
		return ret;
	}

	return VK_SUCCESS;
}

static VkResult
gxr_import_image(struct comp_window_galaxyxr *w, struct gxr_image *img, VkFormat format, VkImageUsageFlags usage)
{
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	VkSubresourceLayout plane_layout = {
	    .offset = 0,
	    .rowPitch = img->pitch,
	};
	VkImageDrmFormatModifierExplicitCreateInfoEXT modifier_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT,
	    .drmFormatModifier = DRM_FORMAT_MOD_LINEAR,
	    .drmFormatModifierPlaneCount = 1,
	    .pPlaneLayouts = &plane_layout,
	};
	VkExternalMemoryImageCreateInfo ext_image = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO,
	    .pNext = &modifier_info,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	};
	VkImageCreateInfo image_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &ext_image,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = format,
	    .extent = {GXR_STEREO_W, GXR_EYE_H, 1},
	    .mipLevels = 1,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT,
	    .usage = usage,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};
	ret = vk->vkCreateImage(vk->device, &image_info, NULL, &img->image);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkCreateImage: %s", vk_result_string(ret));
		return ret;
	}

	VkImageMemoryRequirementsInfo2 req_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_REQUIREMENTS_INFO_2,
	    .image = img->image,
	};
	VkMemoryRequirements2 mem_reqs = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2,
	};
	vk->vkGetImageMemoryRequirements2(vk->device, &req_info, &mem_reqs);

	VkMemoryFdPropertiesKHR fd_props = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR,
	};
	int props_fd = dup(img->dmabuf_fd);
	if (props_fd < 0) {
		return VK_ERROR_OUT_OF_HOST_MEMORY;
	}
	ret = w->get_memory_fd_properties(vk->device, VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT, props_fd,
	                                  &fd_props);
	close(props_fd);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkGetMemoryFdPropertiesKHR: %s", vk_result_string(ret));
		return ret;
	}

	uint32_t memory_bits = mem_reqs.memoryRequirements.memoryTypeBits & fd_props.memoryTypeBits;
	if (memory_bits == 0) {
		GXR_ERROR(w, "No compatible memory type for dma-buf import (req 0x%x fd 0x%x)",
		          mem_reqs.memoryRequirements.memoryTypeBits, fd_props.memoryTypeBits);
		return VK_ERROR_INVALID_EXTERNAL_HANDLE;
	}

	int import_fd = dup(img->dmabuf_fd);
	if (import_fd < 0) {
		return VK_ERROR_OUT_OF_HOST_MEMORY;
	}
	VkMemoryDedicatedAllocateInfo dedicated = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO,
	    .image = img->image,
	};
	VkImportMemoryFdInfoKHR import_info = {
	    .sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR,
	    .pNext = &dedicated,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT,
	    .fd = import_fd,
	};
	VkMemoryAllocateInfo alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = &import_info,
	    .allocationSize = mem_reqs.memoryRequirements.size,
	    .memoryTypeIndex = (uint32_t)__builtin_ctz(memory_bits),
	};
	ret = vk->vkAllocateMemory(vk->device, &alloc_info, NULL, &img->memory);
	if (ret != VK_SUCCESS) {
		close(import_fd);
		GXR_ERROR(w, "vkAllocateMemory(import): %s", vk_result_string(ret));
		return ret;
	}

	ret = vk->vkBindImageMemory(vk->device, img->image, img->memory, 0);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkBindImageMemory: %s", vk_result_string(ret));
		return ret;
	}

	VkImageViewCreateInfo view_info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
	    .image = img->image,
	    .viewType = VK_IMAGE_VIEW_TYPE_2D,
	    .format = format,
	    .subresourceRange =
	        {
	            .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	            .levelCount = 1,
	            .layerCount = 1,
	        },
	};
	ret = vk->vkCreateImageView(vk->device, &view_info, NULL, &img->view);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkCreateImageView: %s", vk_result_string(ret));
		return ret;
	}

	return VK_SUCCESS;
}

static void
gxr_free_images(struct comp_window_galaxyxr *w)
{
	struct vk_bundle *vk = get_vk(w);

	for (uint32_t i = 0; i < GXR_NUM_IMAGES; i++) {
		struct gxr_image *img = &w->image_data[i];

		struct gxr_eye *eyes[2] = {w->left, w->right};
		for (uint32_t e = 0; e < 2; e++) {
			if (eyes[e]->fbs[i] != 0 && eyes[e]->fd >= 0) {
				drmModeRmFB(eyes[e]->fd, eyes[e]->fbs[i]);
				eyes[e]->fbs[i] = 0;
			}
			if (eyes[e]->handles[i] != 0 && eyes[e]->fd >= 0) {
				drmCloseBufferHandle(eyes[e]->fd, eyes[e]->handles[i]);
				eyes[e]->handles[i] = 0;
			}
		}

		if (img->view != VK_NULL_HANDLE) {
			vk->vkDestroyImageView(vk->device, img->view, NULL);
			img->view = VK_NULL_HANDLE;
		}
		if (img->image != VK_NULL_HANDLE) {
			vk->vkDestroyImage(vk->device, img->image, NULL);
			img->image = VK_NULL_HANDLE;
		}
		if (img->memory != VK_NULL_HANDLE) {
			vk->vkFreeMemory(vk->device, img->memory, NULL);
			img->memory = VK_NULL_HANDLE;
		}
		if (img->dmabuf_fd >= 0) {
			close(img->dmabuf_fd);
			img->dmabuf_fd = -1;
		}
	}

	if (w->base.semaphores.render_complete != VK_NULL_HANDLE) {
		vk->vkDestroySemaphore(vk->device, w->base.semaphores.render_complete, NULL);
		w->base.semaphores.render_complete = VK_NULL_HANDLE;
	}
	for (uint32_t i = 0; i < GXR_NUM_IMAGES; i++) {
		if (w->submit_fences[i] != VK_NULL_HANDLE) {
			vk->vkDestroyFence(vk->device, w->submit_fences[i], NULL);
			w->submit_fences[i] = VK_NULL_HANDLE;
		}
	}

	w->has_images = false;
	w->base.images = NULL;
	w->base.image_count = 0;
}

static bool
gxr_test_modeset(struct comp_window_galaxyxr *w, struct gxr_eye *eye, uint32_t fb)
{
	drmModeAtomicReq *req = drmModeAtomicAlloc();

	drmModeAtomicAddProperty(req, eye->conn, prop_id(eye, eye->conn, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID"),
	                         eye->crtc);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "MODE_ID"),
	                         eye->mode_blob);
	drmModeAtomicAddProperty(req, eye->crtc, prop_id(eye, eye->crtc, DRM_MODE_OBJECT_CRTC, "ACTIVE"), 1);
	for (uint32_t i = 0; i < eye->all_plane_count; i++) {
		if (!plane_selected(eye, eye->all_planes[i])) {
			detach_plane(req, eye, eye->all_planes[i]);
		}
	}
	for (uint32_t s = 0; s < GXR_NPLANES; s++) {
		add_plane(req, eye, s, fb, -1);
	}

	int rc = drmModeAtomicCommit(eye->fd, req, DRM_MODE_ATOMIC_TEST_ONLY | DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
	drmModeAtomicFree(req);

	if (rc != 0) {
		GXR_WARN(w, "[%s] test-only modeset rejected: %s", eye->name, strerror(errno));
	}
	return rc == 0;
}

static bool
gxr_try_create_images(struct comp_window_galaxyxr *w,
                      const struct comp_target_create_images_info *create_info,
                      uint64_t modifier)
{
	bool ubwc = modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED;
	VkResult ret;

	uint32_t fourcc = 0;
	VkFormat format = gxr_pick_format(w, create_info, modifier, &fourcc);
	if (format == VK_FORMAT_UNDEFINED) {
		return false;
	}

	for (uint32_t i = 0; i < GXR_NUM_IMAGES; i++) {
		struct gxr_image *img = &w->image_data[i];

		if (ubwc) {
			ret = gxr_alloc_image_ubwc(w, img, format, create_info->image_usage);
			if (ret != VK_SUCCESS) {
				return false;
			}
		} else {
			if (!gxr_alloc_buffer(w, &img->dmabuf_fd, &img->pitch)) {
				return false;
			}
			ret = gxr_import_image(w, img, format, create_info->image_usage);
			if (ret != VK_SUCCESS) {
				return false;
			}
			img->fb_plane_count = 1;
			img->fb_pitches[0] = img->pitch;
			img->fb_offsets[0] = 0;
		}

		struct gxr_eye *eyes[2] = {w->left, w->right};
		for (uint32_t e = 0; e < 2; e++) {
			struct gxr_eye *eye = eyes[e];
			uint32_t handle = 0;
			if (drmPrimeFDToHandle(eye->fd, img->dmabuf_fd, &handle) != 0) {
				GXR_ERROR(w, "[%s] dmabuf import failed: %s", eye->name, strerror(errno));
				return false;
			}
			eye->handles[i] = handle;

			uint32_t handles[4] = {0};
			uint32_t pitches[4] = {0};
			uint32_t offsets[4] = {0};
			uint64_t modifiers[4] = {0};
			for (uint32_t p = 0; p < img->fb_plane_count; p++) {
				handles[p] = handle;
				pitches[p] = img->fb_pitches[p];
				offsets[p] = img->fb_offsets[p];
				modifiers[p] = modifier;
			}

			int rc;
			if (ubwc) {
				rc = drmModeAddFB2WithModifiers(eye->fd, GXR_STEREO_W, GXR_EYE_H, fourcc, handles,
				                                pitches, offsets, modifiers, &eye->fbs[i],
				                                DRM_MODE_FB_MODIFIERS);
			} else {
				rc = drmModeAddFB2(eye->fd, GXR_STEREO_W, GXR_EYE_H, fourcc, handles, pitches, offsets,
				                   &eye->fbs[i], 0);
			}
			if (rc != 0) {
				GXR_WARN(w, "[%s] addfb2%s failed: %s", eye->name, ubwc ? " (UBWC)" : "",
				         strerror(errno));
				return false;
			}
		}

		w->images[i].handle = img->image;
		w->images[i].view = img->view;
	}

	// Only gate the experimental path on the test commit, the linear
	// path is known good even if a test-only modeset gets picky.
	if (ubwc &&
	    (!gxr_test_modeset(w, w->left, w->left->fbs[0]) || !gxr_test_modeset(w, w->right, w->right->fbs[0]))) {
		return false;
	}

	w->fourcc = fourcc;
	w->modifier = modifier;
	w->base.format = format;
	return true;
}


/*
 *
 * Target members.
 *
 */

static bool
gxr_target_init_pre_vulkan(struct comp_target *ct)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;

	w->display = wl_display_connect(NULL);
	if (w->display == NULL) {
		GXR_ERROR(w, "Failed to connect to Wayland display (set WAYLAND_DISPLAY, run as the compositor user)");
		return false;
	}

	w->registry = wl_display_get_registry(w->display);
	wl_registry_add_listener(w->registry, &registry_listener, w);
	wl_display_roundtrip(w->display);

	if (w->dev_count == 0) {
		GXR_ERROR(w, "Compositor does not advertise wp_drm_lease_device_v1");
		return false;
	}

	if (!gxr_acquire_leases(w)) {
		return false;
	}

	if (!gxr_eye_setup(w, &w->eyes[0]) || !gxr_eye_setup(w, &w->eyes[1])) {
		return false;
	}

	if (w->eyes[0].is_left == w->eyes[1].is_left) {
		GXR_ERROR(w, "Could not tell the two panels apart, got two %s eyes",
		          w->eyes[0].is_left ? "left" : "right");
		return false;
	}
	w->left = w->eyes[0].is_left ? &w->eyes[0] : &w->eyes[1];
	w->right = w->eyes[0].is_left ? &w->eyes[1] : &w->eyes[0];

	GXR_INFO(w, "Display commit order: %s then %s (hardware master last)", w->right->name, w->left->name);

	return true;
}

static bool
gxr_target_init_post_vulkan(struct comp_target *ct, uint32_t preferred_width, uint32_t preferred_height)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	struct vk_bundle *vk = get_vk(w);

	w->get_memory_fd_properties =
	    (PFN_vkGetMemoryFdPropertiesKHR)vk->vkGetDeviceProcAddr(vk->device, "vkGetMemoryFdPropertiesKHR");
	if (w->get_memory_fd_properties == NULL) {
		GXR_ERROR(w, "vkGetMemoryFdPropertiesKHR unavailable, missing VK_EXT_external_memory_dma_buf?");
		return false;
	}

	w->get_fence_fd = (PFN_vkGetFenceFdKHR)vk->vkGetDeviceProcAddr(vk->device, "vkGetFenceFdKHR");
	w->use_sync_fd = debug_get_bool_option_gxr_sync_fd() && w->get_fence_fd != NULL;
	if (!w->use_sync_fd) {
		GXR_INFO(w, "Explicit sync disabled (%s), the present path will block on the GPU",
		         w->get_fence_fd == NULL ? "no VK_KHR_external_fence_fd" : "by request");
	}

	return true;
}

static bool
gxr_target_check_ready(struct comp_target *ct)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	return w->left->fd >= 0 && w->right->fd >= 0 && !w->left->lease_finished && !w->right->lease_finished;
}

static bool
gxr_target_is_shared_presentable_image(struct comp_target *ct)
{
	return false;
}

static void
gxr_target_create_images(struct comp_target *ct,
                         const struct comp_target_create_images_info *create_info,
                         struct vk_bundle_queue *present_queue)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	assert(present_queue != NULL);

	if (w->has_images) {
		vk->vkDeviceWaitIdle(vk->device);
		gxr_wait_flips(w, 100);
		gxr_free_images(w);
	}

	if (create_info->extent.width != GXR_STEREO_W || create_info->extent.height != GXR_EYE_H) {
		GXR_WARN(w, "Ignoring requested extent %ux%u, the panels are %ux%u", create_info->extent.width,
		         create_info->extent.height, GXR_STEREO_W, GXR_EYE_H);
	}

	bool ok = false;
	if (debug_get_bool_option_gxr_ubwc()) {
		ok = gxr_try_create_images(w, create_info, DRM_FORMAT_MOD_QCOM_COMPRESSED);
		if (!ok) {
			GXR_WARN(w, "UBWC scanout not usable, falling back to linear");
			gxr_free_images(w);
		}
	}
	if (!ok) {
		ok = gxr_try_create_images(w, create_info, DRM_FORMAT_MOD_LINEAR);
	}
	if (!ok) {
		GXR_ERROR(w, "No usable format, tried %u candidates", create_info->format_count);
		gxr_free_images(w);
		return;
	}
	VkFormat format = w->base.format;

	VkSemaphoreCreateInfo sem_info = {.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
	ret = vk->vkCreateSemaphore(vk->device, &sem_info, NULL, &w->base.semaphores.render_complete);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkCreateSemaphore: %s", vk_result_string(ret));
		gxr_free_images(w);
		return;
	}
	w->base.semaphores.render_complete_is_timeline = false;

	VkExportFenceCreateInfo export_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_FENCE_CREATE_INFO,
	    .handleTypes = VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT,
	};
	VkFenceCreateInfo fence_info = {
	    .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
	    .pNext = w->use_sync_fd ? &export_info : NULL,
	    .flags = VK_FENCE_CREATE_SIGNALED_BIT,
	};
	for (uint32_t i = 0; i < GXR_NUM_IMAGES; i++) {
		ret = vk->vkCreateFence(vk->device, &fence_info, NULL, &w->submit_fences[i]);
		if (ret != VK_SUCCESS) {
			GXR_ERROR(w, "vkCreateFence: %s", vk_result_string(ret));
			gxr_free_images(w);
			return;
		}
	}

	if (w->upc == NULL) {
		xrt_result_t xret = comp_window_galaxyxr_pacer_create(&w->left->mode, &w->upc);
		if (xret != XRT_SUCCESS) {
			GXR_ERROR(w, "Failed to create the pacer");
			gxr_free_images(w);
			return;
		}
	}

	w->base.image_count = GXR_NUM_IMAGES;
	w->base.images = w->images;
	w->base.width = GXR_STEREO_W;
	w->base.height = GXR_EYE_H;
	w->base.final_layout = VK_IMAGE_LAYOUT_GENERAL;
	// Measured: clearing images this large costs real frame time, and the renderer fully overwrites the target.
	w->base.present_load_op = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	w->base.surface_transform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
	w->has_images = true;

	GXR_INFO(w, "Created %u %ux%u stereo scanout images, vk format %u drm format %.4s%s", GXR_NUM_IMAGES,
	         GXR_STEREO_W, GXR_EYE_H, format, (const char *)&w->fourcc,
	         w->modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED ? " UBWC" : " linear");
}

static bool
gxr_target_has_images(struct comp_target *ct)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	return w->has_images;
}

static VkResult
gxr_target_acquire(struct comp_target *ct, uint32_t *out_index)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;

	if (!w->has_images) {
		return VK_ERROR_OUT_OF_DATE_KHR;
	}

	// Never block here: the loop is paced by the pacer's lattice-timed
	// wake. present() enqueues the commits while the kernel is idle (the
	// previous commit completed at the last vsync) and returns without
	// blocking; the commit ioctl's block on a pending commit is only a
	// safety net for slipped frames. Only already-delivered flip events
	// are read, for the pacer's vblank reference and the latch matching.
	gxr_pump_flip_events(w);

	int64_t flip_ns = w->left->last_flip_ns;
	if (flip_ns != 0 && w->upc != NULL && w->upc->update_vblank_from_display_control != NULL) {
		u_pc_update_vblank_from_display_control(w->upc, flip_ns);
		w->left->last_flip_ns = 0;
	}

	w->acquired_index = (w->acquired_index + 1) % GXR_NUM_IMAGES;
	*out_index = w->acquired_index;

	return VK_SUCCESS;
}

//! Signal timestamp of a signaled sync_file, 0 if not (yet) available.
static int64_t
sync_file_signal_time_ns(int fd)
{
	struct pollfd pfd = {fd, POLLIN, 0};
	if (poll(&pfd, 1, 0) <= 0) {
		return 0;
	}

	struct sync_file_info info = {.num_fences = 0};
	if (ioctl(fd, SYNC_IOC_FILE_INFO, &info) != 0 || info.num_fences == 0 || info.num_fences > 8) {
		return 0;
	}
	struct sync_fence_info fences[8];
	memset(fences, 0, sizeof(fences));
	info.sync_fence_info = (uint64_t)(uintptr_t)fences;
	if (ioctl(fd, SYNC_IOC_FILE_INFO, &info) != 0) {
		return 0;
	}
	int64_t ts = 0;
	for (uint32_t i = 0; i < info.num_fences; i++) {
		if (fences[i].status == 1 && (int64_t)fences[i].timestamp_ns > ts) {
			ts = (int64_t)fences[i].timestamp_ns;
		}
	}
	return ts;
}

static void
gxr_gpu_feedback_pump(struct comp_window_galaxyxr *w)
{
	int64_t now_ns = os_monotonic_get_ns();
	for (uint32_t i = 0; i < ARRAY_SIZE(w->gpu_feedback); i++) {
		if (w->gpu_feedback[i].fd < 0) {
			continue;
		}
		int64_t ts = sync_file_signal_time_ns(w->gpu_feedback[i].fd);
		if (ts == 0) {
			continue;
		}
		w->last_fence_signal_ns = ts;
		for (uint32_t j = 0; j < GXR_COMMIT_Q; j++) {
			struct gxr_commit_rec *rec = &w->commit_q[j];
			if (rec->frame_id == w->gpu_feedback[i].frame_id) {
				rec->fence_sig_ns = ts;
				break;
			}
		}
		u_pc_info_gpu(w->upc, w->gpu_feedback[i].frame_id, w->gpu_feedback[i].submit_ns, ts, now_ns);
		close(w->gpu_feedback[i].fd);
		w->gpu_feedback[i].fd = -1;
	}
}

static void
gxr_gpu_feedback_push(struct comp_window_galaxyxr *w, int sync_fd, int64_t frame_id, int64_t submit_ns)
{
	for (uint32_t i = 0; i < ARRAY_SIZE(w->gpu_feedback); i++) {
		if (w->gpu_feedback[i].fd >= 0) {
			continue;
		}
		int fd = fcntl(sync_fd, F_DUPFD_CLOEXEC, 0);
		if (fd < 0) {
			return;
		}
		w->gpu_feedback[i].fd = fd;
		w->gpu_feedback[i].frame_id = frame_id;
		w->gpu_feedback[i].submit_ns = submit_ns;
		return;
	}
}

static VkResult
gxr_target_present(struct comp_target *ct,
                   struct vk_bundle_queue *present_queue,
                   uint32_t index,
                   uint64_t timeline_semaphore_value,
                   int64_t desired_present_time_ns,
                   int64_t present_slop_ns)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	assert(present_queue != NULL);

	gxr_gpu_feedback_pump(w);

	VkPipelineStageFlags stage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
	VkSubmitInfo submit = {
	    .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
	    .waitSemaphoreCount = 1,
	    .pWaitSemaphores = &w->base.semaphores.render_complete,
	    .pWaitDstStageMask = &stage,
	};

	int64_t fence_start_ns = os_monotonic_get_ns();
	VkFence fence = w->submit_fences[index];

	if (w->use_sync_fd) {
		// Exporting a sync fd resets the fence and leaves it in a murky
		// state, recreate instead. Any pending use ended two flips ago.
		vk->vkDestroyFence(vk->device, fence, NULL);
		VkExportFenceCreateInfo export_info = {
		    .sType = VK_STRUCTURE_TYPE_EXPORT_FENCE_CREATE_INFO,
		    .handleTypes = VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT,
		};
		VkFenceCreateInfo fence_info = {
		    .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
		    .pNext = &export_info,
		};
		ret = vk->vkCreateFence(vk->device, &fence_info, NULL, &fence);
		if (ret != VK_SUCCESS) {
			GXR_ERROR(w, "vkCreateFence: %s", vk_result_string(ret));
			w->submit_fences[index] = VK_NULL_HANDLE;
			return ret;
		}
		w->submit_fences[index] = fence;
	} else {
		ret = vk->vkWaitForFences(vk->device, 1, &fence, VK_TRUE, U_TIME_1S_IN_NS);
		if (ret != VK_SUCCESS) {
			GXR_ERROR(w, "vkWaitForFences(reuse): %s", vk_result_string(ret));
		}
		vk->vkResetFences(vk->device, 1, &fence);
	}

	vk_queue_lock(present_queue);
	ret = vk->vkQueueSubmit(present_queue->queue, 1, &submit, fence);
	vk_queue_unlock(present_queue);
	if (ret != VK_SUCCESS) {
		GXR_ERROR(w, "vkQueueSubmit: %s", vk_result_string(ret));
		return ret;
	}

	int sync_fd = -1;
	if (w->use_sync_fd) {
		VkFenceGetFdInfoKHR fd_info = {
		    .sType = VK_STRUCTURE_TYPE_FENCE_GET_FD_INFO_KHR,
		    .fence = fence,
		    .handleType = VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT,
		};
		ret = w->get_fence_fd(vk->device, &fd_info, &sync_fd);
		if (ret != VK_SUCCESS) {
			GXR_ERROR(w, "vkGetFenceFdKHR: %s, falling back to blocking present", vk_result_string(ret));
			w->use_sync_fd = false;
			sync_fd = -1;
		}
	}
	if (sync_fd >= 0) {
		gxr_gpu_feedback_push(w, sync_fd, w->pacing_frame_id, fence_start_ns);
	}
	if (!w->use_sync_fd) {
		ret = vk->vkWaitForFences(vk->device, 1, &fence, VK_TRUE, U_TIME_1S_IN_NS);
		if (ret != VK_SUCCESS) {
			GXR_ERROR(w, "vkWaitForFences: %s", vk_result_string(ret));
		}
		int64_t now_ns = os_monotonic_get_ns();
		u_pc_info_gpu(w->upc, w->pacing_frame_id, fence_start_ns, now_ns, now_ns);
	}

	// Harvest delivered flip events before deciding whether to commit: a
	// period-late latch measured here schedules the drain drop below.
	int64_t flip_start_ns = os_monotonic_get_ns();
	gxr_pump_flip_events(w);

	int64_t now_ns = os_monotonic_get_ns();
	w->stats.flip_ns += now_ns - flip_start_ns;
	w->stats.fence_ns += flip_start_ns - fence_start_ns;
	w->stats.lead_ns += desired_present_time_ns - now_ns;
	w->stats.presents++;
	if (w->stats.presents >= 256) {
		double period_ms = time_ns_to_ms_f((now_ns - w->stats.last_report_ns) / w->stats.presents);
		uint32_t ec = w->stats.err_count > 0 ? w->stats.err_count : 1;
		uint32_t lc = w->stats.latch_count > 0 ? w->stats.latch_count : 1;
		uint32_t rc = w->stats.rl_count > 0 ? w->stats.rl_count : 1;
		uint32_t pc = w->stats.period_count > 0 ? w->stats.period_count : 1;
		GXR_INFO(w,
		         "%u frames: %.2fms/frame (%.1f fps), fence %.2fms, commit wait %.2fms, lead %.2fms, "
		         "latch err avg %.3f max %.3f, fence->latch avg %.2f min %.2f, R-L %.2fms, "
		         "flip period avg %.2f min %.2f max %.2f",
		         w->stats.presents, period_ms, 1000.0 / period_ms,
		         time_ns_to_ms_f(w->stats.fence_ns / w->stats.presents),
		         time_ns_to_ms_f(w->stats.flip_ns / w->stats.presents),
		         time_ns_to_ms_f(w->stats.lead_ns / (int64_t)w->stats.presents),
		         time_ns_to_ms_f(w->stats.err_sum_ns / (int64_t)ec), time_ns_to_ms_f(w->stats.err_max_ns),
		         time_ns_to_ms_f(w->stats.latch_sum_ns / lc), time_ns_to_ms_f(w->stats.latch_min_ns),
		         time_ns_to_ms_f(w->stats.rl_sum_ns / (int64_t)rc),
		         time_ns_to_ms_f(w->stats.period_sum_ns / pc), time_ns_to_ms_f(w->stats.period_min_ns),
		         time_ns_to_ms_f(w->stats.period_max_ns));
		if (w->stats.late > 0 || w->stats.dropped > 0) {
			GXR_WARN(w, "%u of %u frames latched a period late, %u commits dropped to drain", w->stats.late,
			         w->stats.presents, w->stats.dropped);
		}
		w->stats.presents = 0;
		w->stats.fence_ns = 0;
		w->stats.flip_ns = 0;
		w->stats.lead_ns = 0;
		w->stats.err_sum_ns = 0;
		w->stats.err_max_ns = 0;
		w->stats.err_count = 0;
		w->stats.late = 0;
		w->stats.dropped = 0;
		w->stats.latch_min_ns = 0;
		w->stats.latch_sum_ns = 0;
		w->stats.latch_count = 0;
		w->stats.rl_sum_ns = 0;
		w->stats.rl_count = 0;
		w->stats.period_min_ns = 0;
		w->stats.period_max_ns = 0;
		w->stats.period_sum_ns = 0;
		w->stats.period_count = 0;
		w->stats.last_report_ns = now_ns;
	}
	if (w->stats.last_report_ns == 0) {
		w->stats.last_report_ns = now_ns;
	}

	if (w->left->lease_finished || w->right->lease_finished) {
		if (sync_fd >= 0) {
			close(sync_fd);
		}
		return VK_ERROR_SURFACE_LOST_KHR;
	}

	GXR_DEBUG(w, "trace present f%ld commit %.3f fence_sig %.3f", w->pacing_frame_id,
	          time_ns_to_ms_f(os_monotonic_get_ns() % 1000000000),
	          time_ns_to_ms_f(w->last_fence_signal_ns % 1000000000));

	bool do_modeset = !w->modeset_done;

	// A commit that blocked in the ioctl was queued behind a still-pending
	// one: the kernel holds a commit from the ioctl to one vsync past its
	// latch, so once a frame slips, every following commit queues up
	// behind the previous one and the pipeline stays a period deep for
	// good, showing every frame one period later than predicted. Dropping
	// one commit drains the queue back to one-deep.
	if (w->drain_pending && !do_modeset) {
		w->drain_pending = false;
		w->stats.dropped++;
		GXR_DEBUG(w, "dropping frame %ld to drain the display queue", w->pacing_frame_id);
		if (sync_fd >= 0) {
			close(sync_fd);
		}
		return VK_SUCCESS;
	}

	// KMS has no notion of a target present time and the render fence is
	// the only thing holding a commit back, so with a wake lead beyond
	// one period a quickly signaled fence (a heavy scene clearing) could
	// latch a frame a full period early. The kernel's commit
	// serialization and its stale-vsync wait after idle were observed to
	// mask this, but neither is contractual: gate the enqueue to the
	// period preceding the target vsync explicitly.
	if (!do_modeset) {
		int64_t gate_ns = desired_present_time_ns - mode_period_ns(&w->left->mode) + U_TIME_1MS_IN_NS;
		int64_t before_gate_ns = os_monotonic_get_ns();
		if (before_gate_ns < gate_ns) {
			os_nanosleep(gate_ns - before_gate_ns);
		}
	}

	// The two SDE devices are ganged in the kernel (ctl_op_sync: the
	// secondary's DSI clocks are slaved to the primary's PLL): a commit on
	// one only completes once the other commits too, so both eyes are
	// committed back to back, slave first (with an already-signaled fence
	// a master-first commit can let the master's crtc_commit worker run
	// before the slave eye is submitted, and the eyes miss different
	// vblanks).
	//
	// In steady state the previous commit completed at the last vsync,
	// long before this enqueue, so the kernel picks these up immediately,
	// waits for the render fence in the commit kthread, programs the
	// planes and flushes; the frame latches at the next vsync. The ioctl
	// only blocks as a safety net when a frame slipped its vsync.
	int64_t commit_start_ns = os_monotonic_get_ns();
	int er = commit_eye(w, w->right, w->right->fbs[index], do_modeset, sync_fd);
	int el = commit_eye(w, w->left, w->left->fbs[index], do_modeset, sync_fd);
	int64_t commit_block_ns = os_monotonic_get_ns() - commit_start_ns;
	w->stats.flip_ns += commit_block_ns;
	if (er != 0 || el != 0) {
		if (sync_fd >= 0) {
			close(sync_fd);
		}
		return VK_ERROR_SURFACE_LOST_KHR;
	}
	if (do_modeset) {
		w->modeset_done = true;
		GXR_INFO(w, "Panels lit: stereo scanout running, four planes per eye");
	} else {
		// A load spike can delay the previous commit's cleanup kthread
		// and block the ioctl for a few ms without costing anything:
		// the frame still latches on time. Only a commit that both
		// queued up and was picked up too late to program and flush
		// before its own vsync means the queue went a period deep.
		int64_t commit_end_ns = commit_start_ns + commit_block_ns;
		if (commit_block_ns > mode_period_ns(&w->left->mode) / 8 &&
		    commit_end_ns > desired_present_time_ns - GXR_PICKUP_DEADLINE_NS) {
			GXR_DEBUG(w, "frame %ld commit blocked %.2fms past its latch deadline, draining",
			          w->pacing_frame_id, time_ns_to_ms_f(commit_block_ns));
			w->drain_pending = true;
		}

		// The modeset commit is blocking and sends no flip event.
		if (w->commit_q_tail - w->commit_q_head >= GXR_COMMIT_Q) {
			GXR_DEBUG(w, "latch matcher overflow, resetting");
			w->commit_q_head = w->commit_q_tail;
		}
		w->commit_q[w->commit_q_tail++ % GXR_COMMIT_Q] = (struct gxr_commit_rec){
		    .frame_id = w->pacing_frame_id,
		    .desired_ns = desired_present_time_ns,
		};
	}

	// Harvest anything already delivered for the matcher, the stats and
	// the next pacer vblank update. Flip events are delivered 2 ms to a
	// period after the vsync they timestamp, so this usually sees an
	// older frame's latch.
	gxr_pump_flip_events(w);

	if (w->left->latest_flip_ns != 0 && w->right->latest_flip_ns != 0) {
		w->stats.rl_sum_ns += w->right->latest_flip_ns - w->left->latest_flip_ns;
		w->stats.rl_count++;
		if (w->stats.prev_flip_ns != 0 && w->left->latest_flip_ns != w->stats.prev_flip_ns) {
			int64_t period_ns = w->left->latest_flip_ns - w->stats.prev_flip_ns;
			if (w->stats.period_count == 0 || period_ns < w->stats.period_min_ns) {
				w->stats.period_min_ns = period_ns;
			}
			if (period_ns > w->stats.period_max_ns) {
				w->stats.period_max_ns = period_ns;
			}
			w->stats.period_sum_ns += period_ns;
			w->stats.period_count++;
		}
		w->stats.prev_flip_ns = w->left->latest_flip_ns;
	}
	if (sync_fd >= 0) {
		close(sync_fd);
	}

	return VK_SUCCESS;
}

static VkResult
gxr_target_wait_for_present(struct comp_target *ct, time_duration_ns timeout_ns)
{
	return VK_ERROR_EXTENSION_NOT_PRESENT;
}

static void
gxr_target_flush(struct comp_target *ct)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	if (w->display != NULL) {
		wl_pump(w->display, 0);
	}
}

static void
gxr_target_calc_frame_pacing(struct comp_target *ct,
                             int64_t *out_frame_id,
                             int64_t *out_wake_up_time_ns,
                             int64_t *out_desired_present_time_ns,
                             int64_t *out_present_slop_ns,
                             int64_t *out_predicted_display_time_ns)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;

	int64_t frame_id = -1;
	int64_t wake_up_time_ns = 0;
	int64_t desired_present_time_ns = 0;
	int64_t present_slop_ns = 0;
	int64_t predicted_display_time_ns = 0;
	int64_t predicted_display_period_ns = 0;
	int64_t min_display_period_ns = 0;

	u_pc_predict(w->upc,                       //
	             os_monotonic_get_ns(),        //
	             &frame_id,                    //
	             &wake_up_time_ns,             //
	             &desired_present_time_ns,     //
	             &present_slop_ns,             //
	             &predicted_display_time_ns,   //
	             &predicted_display_period_ns, //
	             &min_display_period_ns);      //

	w->pacing_frame_id = frame_id;

	GXR_DEBUG(w, "trace predict f%ld now %.3f wake %.3f desired %.3f", frame_id,
	          time_ns_to_ms_f(os_monotonic_get_ns() % 1000000000), time_ns_to_ms_f(wake_up_time_ns % 1000000000),
	          time_ns_to_ms_f(desired_present_time_ns % 1000000000));

	*out_frame_id = frame_id;
	*out_wake_up_time_ns = wake_up_time_ns;
	*out_desired_present_time_ns = desired_present_time_ns;
	*out_present_slop_ns = present_slop_ns;
	*out_predicted_display_time_ns = predicted_display_time_ns;
}

static void
gxr_target_mark_timing_point(struct comp_target *ct,
                             enum comp_target_timing_point point,
                             int64_t frame_id,
                             int64_t when_ns)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;

	switch (point) {
	case COMP_TARGET_TIMING_POINT_WAKE_UP:
		u_pc_mark_point(w->upc, U_TIMING_POINT_WAKE_UP, frame_id, when_ns);
		break;
	case COMP_TARGET_TIMING_POINT_BEGIN: u_pc_mark_point(w->upc, U_TIMING_POINT_BEGIN, frame_id, when_ns); break;
	case COMP_TARGET_TIMING_POINT_SUBMIT_BEGIN:
		u_pc_mark_point(w->upc, U_TIMING_POINT_SUBMIT_BEGIN, frame_id, when_ns);
		break;
	case COMP_TARGET_TIMING_POINT_SUBMIT_END:
		u_pc_mark_point(w->upc, U_TIMING_POINT_SUBMIT_END, frame_id, when_ns);
		break;
	default: assert(false);
	}
}

static VkResult
gxr_target_update_timings(struct comp_target *ct)
{
	return VK_SUCCESS;
}

static void
gxr_target_info_gpu(struct comp_target *ct, int64_t frame_id, int64_t gpu_start_ns, int64_t gpu_end_ns, int64_t when_ns)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	static uint32_t count = 0;
	if (count++ % 256 == 0) {
		GXR_DEBUG(w, "info_gpu: frame %ld gpu %.2fms, ended %.2fms before now", frame_id,
		          time_ns_to_ms_f(gpu_end_ns - gpu_start_ns), time_ns_to_ms_f(when_ns - gpu_end_ns));
	}
	u_pc_info_gpu(w->upc, frame_id, gpu_start_ns, gpu_end_ns, when_ns);
}

static xrt_result_t
gxr_target_get_refresh_rates(struct comp_target *ct, uint32_t *out_count, float *out_rates)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	*out_count = 1;
	out_rates[0] = (float)mode_hz(&w->left->mode);
	return XRT_SUCCESS;
}

static xrt_result_t
gxr_target_get_current_refresh_rate(struct comp_target *ct, float *out_rate)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	*out_rate = (float)mode_hz(&w->left->mode);
	return XRT_SUCCESS;
}

static xrt_result_t
gxr_target_request_refresh_rate(struct comp_target *ct, float rate)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	if ((int)rate == mode_hz(&w->left->mode)) {
		return XRT_SUCCESS;
	}
	return XRT_ERROR_FEATURE_NOT_SUPPORTED;
}

static VkResult
gxr_target_queue_supports_present(struct comp_target *ct, struct vk_bundle_queue *queue, VkBool32 *out_supported)
{
	*out_supported = VK_TRUE;
	return VK_SUCCESS;
}

static void
gxr_target_set_output_enabled(struct comp_target *ct, bool enabled)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;

	if (enabled) {
		// The next present() runs the full modeset again.
		return;
	}
	if (w->left->fd < 0 || w->right->fd < 0) {
		return;
	}

	gxr_wait_flips(w, 100);
	GXR_INFO(w, "Disabling panels%s", w->modeset_done ? "" : " (never lit)");
	gxr_eye_disable(w, w->left);
	gxr_eye_disable(w, w->right);
	w->modeset_done = false;
	w->commit_q_head = w->commit_q_tail;
	w->commit_q_discards = 0;
	w->drain_pending = false;
}

static void
gxr_target_set_title(struct comp_target *ct, const char *title)
{}

static void
gxr_eye_shutdown(struct gxr_eye *eye)
{
	if (eye->mode_blob != 0 && eye->fd >= 0) {
		drmModeDestroyPropertyBlob(eye->fd, eye->mode_blob);
		eye->mode_blob = 0;
	}
	if (eye->lease != NULL) {
		wp_drm_lease_v1_destroy(eye->lease);
		eye->lease = NULL;
	}
	if (eye->fd >= 0) {
		close(eye->fd);
		eye->fd = -1;
	}
}

static void
gxr_target_destroy(struct comp_target *ct)
{
	struct comp_window_galaxyxr *w = (struct comp_window_galaxyxr *)ct;
	struct vk_bundle *vk = get_vk(w);

	if (w->has_images) {
		vk->vkDeviceWaitIdle(vk->device);
	}

	for (uint32_t i = 0; i < ARRAY_SIZE(w->gpu_feedback); i++) {
		if (w->gpu_feedback[i].fd >= 0) {
			close(w->gpu_feedback[i].fd);
			w->gpu_feedback[i].fd = -1;
		}
	}

	gxr_wait_flips(w, 100);

	if (w->modeset_done) {
		GXR_INFO(w, "Disabling panels and returning the leases");
		gxr_eye_disable(w, w->left);
		gxr_eye_disable(w, w->right);
	}

	gxr_free_images(w);

	gxr_eye_shutdown(w->left);
	gxr_eye_shutdown(w->right);

	for (uint32_t i = 0; i < GXR_MAX_CONNS; i++) {
		if (w->conns[i].obj != NULL) {
			wp_drm_lease_connector_v1_destroy(w->conns[i].obj);
		}
	}
	for (uint32_t i = 0; i < w->dev_count; i++) {
		wp_drm_lease_device_v1_release(w->devs[i].obj);
	}
	if (w->registry != NULL) {
		wl_registry_destroy(w->registry);
	}
	if (w->display != NULL) {
		wl_display_flush(w->display);
		wl_display_disconnect(w->display);
	}

	if (w->upc != NULL) {
		u_pc_destroy(&w->upc);
	}

	free(w);
}

struct comp_target *
comp_window_galaxyxr_create(struct comp_compositor *c)
{
	struct comp_window_galaxyxr *w = U_TYPED_CALLOC(struct comp_window_galaxyxr);

	w->base.name = "galaxyxr";
	w->base.init_pre_vulkan = gxr_target_init_pre_vulkan;
	w->base.init_post_vulkan = gxr_target_init_post_vulkan;
	w->base.get_info = comp_target_get_info_default;
	w->base.check_ready = gxr_target_check_ready;
	w->base.is_shared_presentable_image = gxr_target_is_shared_presentable_image;
	w->base.create_images = gxr_target_create_images;
	w->base.has_images = gxr_target_has_images;
	w->base.acquire = gxr_target_acquire;
	w->base.present = gxr_target_present;
	w->base.wait_for_present = gxr_target_wait_for_present;
	w->base.flush = gxr_target_flush;
	w->base.calc_frame_pacing = gxr_target_calc_frame_pacing;
	w->base.mark_timing_point = gxr_target_mark_timing_point;
	w->base.update_timings = gxr_target_update_timings;
	w->base.info_gpu = gxr_target_info_gpu;
	w->base.get_refresh_rates = gxr_target_get_refresh_rates;
	w->base.get_current_refresh_rate = gxr_target_get_current_refresh_rate;
	w->base.request_refresh_rate = gxr_target_request_refresh_rate;
	w->base.queue_supports_present = gxr_target_queue_supports_present;
	w->base.set_output_enabled = gxr_target_set_output_enabled;
	w->base.set_title = gxr_target_set_title;
	w->base.destroy = gxr_target_destroy;
	w->base.c = c;
	w->base.wait_for_present_supported = false;

	w->eyes[0].fd = -1;
	for (uint32_t i = 0; i < ARRAY_SIZE(w->gpu_feedback); i++) {
		w->gpu_feedback[i].fd = -1;
	}
	w->eyes[1].fd = -1;
	w->left = &w->eyes[0];
	w->right = &w->eyes[1];
	w->acquired_index = GXR_NUM_IMAGES - 1;
	for (uint32_t i = 0; i < GXR_NUM_IMAGES; i++) {
		w->image_data[i].dmabuf_fd = -1;
	}

	return &w->base;
}


/*
 *
 * Factory.
 *
 */

static const char *optional_device_extensions[] = {
    VK_EXT_EXTERNAL_MEMORY_DMA_BUF_EXTENSION_NAME,
    VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME,
};

static bool
gxr_factory_detect(const struct comp_target_factory *ctf, struct comp_compositor *c)
{
	return debug_get_bool_option_gxr_autodetect() && gxr_is_galaxy_xr();
}

static bool
gxr_factory_create_target(const struct comp_target_factory *ctf, struct comp_compositor *c, struct comp_target **out_ct)
{
	struct comp_target *ct = comp_window_galaxyxr_create(c);
	if (ct == NULL) {
		return false;
	}

	*out_ct = ct;

	return true;
}

const struct comp_target_factory comp_target_factory_galaxyxr = {
    .name = "Samsung Galaxy XR Direct Mode",
    .identifier = "galaxyxr",
    .requires_vulkan_for_create = false,
    .is_deferred = false,
    .required_instance_version = 0,
    .required_instance_extensions = NULL,
    .required_instance_extension_count = 0,
    .optional_device_extensions = optional_device_extensions,
    .optional_device_extension_count = ARRAY_SIZE(optional_device_extensions),
    .detect = gxr_factory_detect,
    .create_target = gxr_factory_create_target,
};
