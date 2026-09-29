// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Wayland window code.
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include <errno.h>
#include <linux/input.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <wayland-client.h>

#include "xdg-shell-client-protocol.h"
#include "xrt/xrt_compiler.h"
#include "main/comp_window.h"
#include "util/u_misc.h"


/*
 *
 * Private structs.
 *
 */

/*!
 * A Wayland connection and window.
 *
 * @implements comp_target_swapchain
 */
struct comp_window_wayland
{
	struct comp_target_swapchain base;

	struct wl_display *display;
	struct wl_compositor *compositor;
	struct wl_surface *surface;

	struct xdg_wm_base *wm_base;
	struct xdg_surface *xdg_surface;
	struct xdg_toplevel *xdg_toplevel;

	bool fullscreen_requested;

	/*!
	 * The size the last xdg_toplevel.configure asked for, applied when
	 * its xdg_surface.configure arrives; 0x0 means "your choice".
	 */
	VkExtent2D configured;

	/*!
	 * A configure that needs the swapchain re-created is acknowledged
	 * only once images of the new size exist, so the buffer that
	 * follows the ack is the one the configure asked for (xdg-shell:
	 * the ack belongs to the commit that applies the state). Until then
	 * the serial waits here.
	 */
	bool ack_pending;
	uint32_t ack_serial;

	//! The base target's create_images, chained to acknowledge a deferred configure.
	void (*base_create_images)(struct comp_target *ct,
	                           const struct comp_target_create_images_info *create_info,
	                           struct vk_bundle_queue *present_queue);
};


/*
 *
 * Pre declare functions.
 *
 */

static void
comp_window_wayland_destroy(struct comp_target *ct);

static bool
comp_window_wayland_init(struct comp_target *ct);

static void
comp_window_wayland_update_window_title(struct comp_target *ct, const char *title);

static void
comp_window_wayland_registry_global(struct comp_window_wayland *w,
                                    struct wl_registry *registry,
                                    uint32_t name,
                                    const char *interface);

static void
comp_window_wayland_fullscreen(struct comp_window_wayland *w);

static bool
comp_window_wayland_init_swapchain(struct comp_target *ct, uint32_t width, uint32_t height);

static VkResult
comp_window_wayland_create_surface(struct comp_window_wayland *w, VkSurfaceKHR *out_surface);

static void
comp_window_wayland_flush(struct comp_target *ct);

static bool
comp_window_wayland_configure(struct comp_window_wayland *w, uint32_t width, uint32_t height);

static void
comp_window_wayland_create_images(struct comp_target *ct,
                                  const struct comp_target_create_images_info *create_info,
                                  struct vk_bundle_queue *present_queue);


/*
 *
 * Functions.
 *
 */

static inline struct vk_bundle *
get_vk(struct comp_window_wayland *cww)
{
	return &cww->base.base.c->base.vk;
}

struct comp_target *
comp_window_wayland_create(struct comp_compositor *c)
{
	struct comp_window_wayland *w = U_TYPED_CALLOC(struct comp_window_wayland);

	// The display timing code hasn't been tested on Wayland and may be broken.
	comp_target_swapchain_init_and_set_fnptrs(&w->base, COMP_TARGET_FORCE_FAKE_DISPLAY_TIMING);

	w->base.base.name = "wayland";
	w->base.display = VK_NULL_HANDLE;
	w->base.base.destroy = comp_window_wayland_destroy;
	w->base.base.flush = comp_window_wayland_flush;
	w->base.base.init_pre_vulkan = comp_window_wayland_init;
	w->base.base.init_post_vulkan = comp_window_wayland_init_swapchain;
	w->base.base.set_title = comp_window_wayland_update_window_title;
	w->base.base.c = c;

	// Chain create_images so a deferred configure is acknowledged with its images.
	w->base_create_images = w->base.base.create_images;
	w->base.base.create_images = comp_window_wayland_create_images;

	return &w->base.base;
}

static void
comp_window_wayland_destroy(struct comp_target *ct)
{
	struct comp_window_wayland *cww = (struct comp_window_wayland *)ct;

	comp_target_swapchain_cleanup(&cww->base);

	if (cww->xdg_toplevel) {
		xdg_toplevel_destroy(cww->xdg_toplevel);
	}
	if (cww->xdg_surface) {
		xdg_surface_destroy(cww->xdg_surface);
	}
	if (cww->wm_base) {
		xdg_wm_base_destroy(cww->wm_base);
	}
	if (cww->surface) {
		wl_surface_destroy(cww->surface);
		cww->surface = NULL;
	}
	if (cww->compositor) {
		wl_compositor_destroy(cww->compositor);
		cww->compositor = NULL;
	}
	if (cww->display) {
		wl_display_disconnect(cww->display);
		cww->display = NULL;
	}

	free(ct);
}

static void
comp_window_wayland_update_window_title(struct comp_target *ct, const char *title)
{
	struct comp_window_wayland *w_wayland = (struct comp_window_wayland *)ct;
	xdg_toplevel_set_title(w_wayland->xdg_toplevel, title);
}

static void
comp_window_wayland_fullscreen(struct comp_window_wayland *w)
{
	xdg_toplevel_set_fullscreen(w->xdg_toplevel, NULL);
	wl_surface_commit(w->surface);
}

static void
_xdg_surface_configure_cb(void *data, struct xdg_surface *surface, uint32_t serial)
{
	struct comp_window_wayland *w = (struct comp_window_wayland *)data;

	/*
	 * The compositor's size for us is known now (the toplevel configure
	 * precedes this event). Apply it, and if that means new images, hold
	 * the ack until they exist — the renderer may already have acquired
	 * an image of the old size for the next frame, and acknowledging
	 * before that frame is presented would tell the compositor the old
	 * buffer is the new state.
	 */
	bool recreate = comp_window_wayland_configure(w, w->configured.width, w->configured.height);
	if (recreate) {
		w->ack_pending = true;
		w->ack_serial = serial;
	} else {
		// Acknowledging a serial acknowledges every earlier one too.
		xdg_surface_ack_configure(surface, serial);
		w->ack_pending = false;
	}
}

static void
_xdg_toplevel_configure_cb(
    void *data, struct xdg_toplevel *toplevel, int32_t width, int32_t height, struct wl_array *states)
{
	struct comp_window_wayland *w = (struct comp_window_wayland *)data;
	w->configured.width = width > 0 ? (uint32_t)width : 0;
	w->configured.height = height > 0 ? (uint32_t)height : 0;
}

static const struct xdg_surface_listener xdg_surface_listener = {
    _xdg_surface_configure_cb,
};

static void
_xdg_toplevel_close_cb(void *data, struct xdg_toplevel *toplevel)
{}

static const struct xdg_toplevel_listener xdg_toplevel_listener = {
    _xdg_toplevel_configure_cb,
    _xdg_toplevel_close_cb,
#if XDG_TOPLEVEL_CONFIGURE_BOUNDS_SINCE_VERSION >= 4
    NULL,
#endif
#if XDG_TOPLEVEL_WM_CAPABILITIES_SINCE_VERSION >= 5
    NULL,
#endif
};

static void
_xdg_wm_base_ping_cb(void *data, struct xdg_wm_base *wm_base, uint32_t serial)
{
	xdg_wm_base_pong(wm_base, serial);
}

static const struct xdg_wm_base_listener xdg_wm_base_listener = {
    _xdg_wm_base_ping_cb,
};

static bool
comp_window_wayland_init_swapchain(struct comp_target *ct, uint32_t width, uint32_t height)
{
	struct comp_window_wayland *w_wayland = (struct comp_window_wayland *)ct;
	VkResult ret;

	ret = comp_window_wayland_create_surface(w_wayland, &w_wayland->base.surface.handle);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(ct->c, "Failed to create surface!");
		return false;
	}

	// The window is resizable: configure sizes are applied by
	// comp_window_wayland_configure, so no min/max pin here.

	return true;
}

static VkResult
comp_window_wayland_create_surface(struct comp_window_wayland *w, VkSurfaceKHR *out_surface)
{
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	VkWaylandSurfaceCreateInfoKHR surface_info = {
	    .sType = VK_STRUCTURE_TYPE_WAYLAND_SURFACE_CREATE_INFO_KHR,
	    .display = w->display,
	    .surface = w->surface,
	};

	VkSurfaceKHR surface = VK_NULL_HANDLE;
	ret = vk->vkCreateWaylandSurfaceKHR( //
	    vk->instance,                    //
	    &surface_info,                   //
	    NULL,                            //
	    &surface);                       //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "vkCreateWaylandSurfaceKHR: %s", vk_result_string(ret));
		return ret;
	}

	VK_NAME_SURFACE(vk, surface, "comp_window_wayland surface");
	*out_surface = surface;

	return VK_SUCCESS;
}

static void
comp_window_wayland_flush(struct comp_target *ct)
{
	struct comp_window_wayland *w_wayland = (struct comp_window_wayland *)ct;

	while (wl_display_prepare_read(w_wayland->display) != 0)
		wl_display_dispatch_pending(w_wayland->display);
	if (wl_display_flush(w_wayland->display) < 0 && errno != EAGAIN) {
		wl_display_cancel_read(w_wayland->display);
		return;
	}

	struct pollfd fds[] = {
	    {
	        .fd = wl_display_get_fd(w_wayland->display),
	        .events = POLLIN,
	        .revents = 0,
	    },
	};

	if (poll(fds, 1, 0) > 0) {
		wl_display_read_events(w_wayland->display);
		wl_display_dispatch_pending(w_wayland->display);
	} else {
		wl_display_cancel_read(w_wayland->display);
	}
}

static void
_registry_global_remove_cb(void *data, struct wl_registry *registry, uint32_t name)
{}

static void
_registry_global_cb(void *data, struct wl_registry *registry, uint32_t name, const char *interface, uint32_t version)
{
	struct comp_window_wayland *w = (struct comp_window_wayland *)data;
	// vik_log_d("Interface: %s Version %d", interface, version);
	comp_window_wayland_registry_global(w, registry, name, interface);
}

static const struct wl_registry_listener registry_listener = {
    _registry_global_cb,
    _registry_global_remove_cb,
};

static void
comp_window_wayland_registry_global(struct comp_window_wayland *w,
                                    struct wl_registry *registry,
                                    uint32_t name,
                                    const char *interface)
{
	if (strcmp(interface, "wl_compositor") == 0) {
		w->compositor = (struct wl_compositor *)wl_registry_bind(registry, name, &wl_compositor_interface, 4);
	} else if (strcmp(interface, "xdg_wm_base") == 0) {
		w->wm_base = (struct xdg_wm_base *)wl_registry_bind(registry, name, &xdg_wm_base_interface, 1);
		xdg_wm_base_add_listener(w->wm_base, &xdg_wm_base_listener, w);
	}
}

static bool
comp_window_wayland_init(struct comp_target *ct)
{
	struct comp_window_wayland *w_wayland = (struct comp_window_wayland *)ct;

	w_wayland->display = wl_display_connect(NULL);
	if (!w_wayland->display) {
		return false;
	}

	struct wl_registry *registry = wl_display_get_registry(w_wayland->display);
	wl_registry_add_listener(registry, &registry_listener, w_wayland);

	wl_display_roundtrip(w_wayland->display);

	wl_registry_destroy(registry);

	w_wayland->surface = wl_compositor_create_surface(w_wayland->compositor);

	if (!w_wayland->wm_base) {
		COMP_ERROR(ct->c, "Compositor is missing xdg-shell support");
	}

	w_wayland->xdg_surface = xdg_wm_base_get_xdg_surface(w_wayland->wm_base, w_wayland->surface);

	xdg_surface_add_listener(w_wayland->xdg_surface, &xdg_surface_listener, w_wayland);

	w_wayland->xdg_toplevel = xdg_surface_get_toplevel(w_wayland->xdg_surface);

	xdg_toplevel_add_listener(w_wayland->xdg_toplevel, &xdg_toplevel_listener, w_wayland);
	/* basic defaults */
	xdg_toplevel_set_app_id(w_wayland->xdg_toplevel, "openxr");
	xdg_toplevel_set_title(w_wayland->xdg_toplevel, "OpenXR application");

	wl_surface_commit(w_wayland->surface);

	/*
	 * Wait for the initial configure so a size the compositor picks
	 * for us (maximized, fullscreen, tiled) is known before the
	 * swapchain is created, instead of creating it at the preferred
	 * size and re-creating it on the first frame.
	 */
	wl_display_roundtrip(w_wayland->display);

	return true;
}

/*!
 * Apply a configured size. Returns true when a swapchain exists and will be
 * re-created for it, i.e. when the ack should wait for the new images.
 */
static bool
comp_window_wayland_configure(struct comp_window_wayland *w, uint32_t width, uint32_t height)
{
	if (w->base.base.c->settings.fullscreen && !w->fullscreen_requested) {
		COMP_DEBUG(w->base.base.c, "Setting full screen");
		comp_window_wayland_fullscreen(w);
		w->fullscreen_requested = true;
	}

	/*
	 * A size of 0x0 means the compositor lets us choose, keep the
	 * compositor's preferred extents in that case. Otherwise use what
	 * we were given: the Wayland WSI never reports OUT_OF_DATE for a
	 * window-system resize, the surface is whatever size we make it.
	 */
	if (width == 0 || height == 0) {
		return false;
	}

	VkExtent2D extent = {width, height};
	comp_target_swapchain_override_extents(&w->base, extent);

	return w->base.override.recreate_pending;
}

static void
comp_window_wayland_create_images(struct comp_target *ct,
                                  const struct comp_target_create_images_info *create_info,
                                  struct vk_bundle_queue *present_queue)
{
	struct comp_window_wayland *w = (struct comp_window_wayland *)ct;

	w->base_create_images(ct, create_info, present_queue);

	// Images of the configured size exist now, the next present is that state.
	if (w->ack_pending && w->base.swapchain.handle != VK_NULL_HANDLE) {
		xdg_surface_ack_configure(w->xdg_surface, w->ack_serial);
		w->ack_pending = false;
	}
}


/*
 *
 * Factory
 *
 */

static const char *instance_extensions[] = {
    VK_KHR_WAYLAND_SURFACE_EXTENSION_NAME,
};

static bool
detect(const struct comp_target_factory *ctf, struct comp_compositor *c)
{
	return false;
}

static bool
create_target(const struct comp_target_factory *ctf, struct comp_compositor *c, struct comp_target **out_ct)
{
	struct comp_target *ct = comp_window_wayland_create(c);
	if (ct == NULL) {
		return false;
	}

	*out_ct = ct;

	return true;
}

const struct comp_target_factory comp_target_factory_wayland = {
    .name = "Wayland Windowed",
    .identifier = "wayland",
    .requires_vulkan_for_create = false,
    .is_deferred = false,
    .required_instance_version = 0,
    .required_instance_extensions = instance_extensions,
    .required_instance_extension_count = ARRAY_SIZE(instance_extensions),
    .optional_device_extensions = NULL,
    .optional_device_extension_count = 0,
    .detect = detect,
    .create_target = create_target,
};
