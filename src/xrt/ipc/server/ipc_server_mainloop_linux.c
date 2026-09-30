// Copyright 2020-2021, Collabora, Ltd.
// Copyright 2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Server mainloop details on Linux.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup ipc_server
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_os.h"

#include "os/os_time.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_trace_marker.h"
#include "util/u_file.h"
#include "util/u_truncate_printf.h"

#include "shared/ipc_shmem.h"
#include "server/ipc_server.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <assert.h>
#include <limits.h>
#include "util/u_debug.h"

#ifdef XRT_HAVE_SYSTEMD
#include <systemd/sd-daemon.h>
#endif

#include "server/ipc_server_peer.h"
#include "os/os_threading.h"


/*
 *
 * Static functions.
 *
 */

static const char *
listener_filename(enum ipc_listener_kind kind)
{
	switch (kind) {
	case IPC_LISTENER_APP: return XRT_IPC_MSG_SOCK_FILENAME;
	case IPC_LISTENER_CONTROL: return XRT_IPC_MSG_SOCK_CONTROL_FILENAME;
	default: assert(false); return NULL;
	}
}

static enum ipc_client_role
listener_role(enum ipc_listener_kind kind)
{
	switch (kind) {
	case IPC_LISTENER_APP: return IPC_CLIENT_ROLE_APP;
	case IPC_LISTENER_CONTROL: return IPC_CLIENT_ROLE_CONTROLLER;
	default: assert(false); return IPC_CLIENT_ROLE_APP;
	}
}

#ifdef XRT_HAVE_SYSTEMD
static void
free_names(char **names)
{
	if (names == NULL) {
		return;
	}
	for (char **n = names; *n != NULL; n++) {
		free(*n);
	}
	free(names);
}
#endif

/*!
 * Take the listening sockets systemd passed us, if any.
 *
 * A socket unit names each fd (FileDescriptorName=app / control); an unnamed
 * single fd is the ordinary socket, which is what older unit files pass.
 */
static int
get_systemd_sockets(struct ipc_server_mainloop *ml)
{
#ifdef XRT_HAVE_SYSTEMD
	char **names = NULL;
	int num_fds = sd_listen_fds_with_names(0, &names);
	if (num_fds < 0) {
		U_LOG_E("sd_listen_fds_with_names failed: %i", num_fds);
		return num_fds;
	}
	if (num_fds == 0) {
		return 0;
	}

	for (int i = 0; i < num_fds; i++) {
		int fd = SD_LISTEN_FDS_START + i;
		const char *name = names != NULL ? names[i] : NULL;
		enum ipc_listener_kind kind;

		if (name == NULL || strcmp(name, "unknown") == 0 || strcmp(name, "app") == 0 ||
		    strcmp(name, "connection") == 0 || strcmp(name, "monado.socket") == 0) {
			kind = IPC_LISTENER_APP;
		} else if (strcmp(name, "control") == 0) {
			kind = IPC_LISTENER_CONTROL;
		} else {
			U_LOG_E("Unknown socket name '%s' passed by systemd.", name);
			free_names(names);
			return -1;
		}

		if (ml->listeners[kind].fd >= 0) {
			U_LOG_E("systemd passed two sockets for '%s'.", name != NULL ? name : "app");
			free_names(names);
			return -1;
		}

		ml->listeners[kind].fd = fd;
		U_LOG_D("Got existing %s socket from systemd.", listener_filename(kind));
	}

	free_names(names);
	ml->launched_by_socket = true;
#endif
	return 0;
}

static int
create_listen_socket(struct ipc_server_mainloop *ml, enum ipc_listener_kind kind)
{
	// no fd provided
	struct sockaddr_un addr = XRT_STRUCT_INIT;
	int fd;
	int ret;

	fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		U_LOG_E("Message Socket Create Error!");
		return fd;
	}


	char sock_file[PATH_MAX];

	int size = u_file_get_path_in_runtime_dir(listener_filename(kind), sock_file, PATH_MAX);
	if (size == -1) {
		U_LOG_E("Could not get socket file name");
		close(fd);
		return -1;
	}

	// Make sure the path fits.
	const int dst_size = (int)ARRAY_SIZE(addr.sun_path);
	if (size >= dst_size) {
		U_LOG_E("Total IPC path too long (%i > %i)", size, dst_size);
		close(fd);
		return -1;
	}

	// Struct zero init at declaration.
	addr.sun_family = AF_UNIX;
	// Use truncate here to avoid warnings.
	u_truncate_snprintf(addr.sun_path, dst_size, "%s", sock_file);

	ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));

#ifdef XRT_HAVE_LIBBSD
	// no other instance is running, or we would have never arrived here
	if (ret < 0 && errno == EADDRINUSE) {
		U_LOG_W("Removing stale socket file %s", sock_file);

		ret = unlink(sock_file);
		if (ret < 0) {
			U_LOG_E("Failed to remove stale socket file %s: %s", sock_file, strerror(errno));
			close(fd);
			return ret;
		}
		ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
	}
#endif

	if (ret < 0) {
		U_LOG_E("Could not bind socket to path %s: %s. Is the service running already?", sock_file,
		        strerror(errno));
#ifdef XRT_HAVE_SYSTEMD
		U_LOG_E("Or, is the systemd unit monado.socket or monado-dev.socket active?");
#endif
		if (errno == EADDRINUSE) {
			U_LOG_E("If monado-service is not running, delete %s before starting a new instance",
			        sock_file);
		}
		close(fd);
		return ret;
	}
	// Save for later
	ml->listeners[kind].filename = strdup(sock_file);

	ret = listen(fd, IPC_MAX_CLIENTS);
	if (ret < 0) {
		close(fd);
		return ret;
	}
	U_LOG_D("Created listening socket %s.", sock_file);
	ml->listeners[kind].fd = fd;
	return 0;
}

/*!
 * Open every listener: what systemd handed over, then whatever is still
 * missing bound by ourselves. Both sockets always exist, so a client that only
 * knows one of the paths works whether or not we were socket-activated.
 */
static int
init_listen_sockets(struct ipc_server_mainloop *ml)
{
	int ret;

	for (int i = 0; i < IPC_LISTENER_COUNT; i++) {
		ml->listeners[i].fd = -1;
		ml->listeners[i].role = listener_role((enum ipc_listener_kind)i);
		ml->listeners[i].filename = NULL;
	}
	for (int i = 0; i < IPC_MAX_SANDBOX_LISTENERS; i++) {
		ml->sandbox_listeners[i].fd = -1;
		ml->sandbox_listeners[i].owner_index = -1;
	}

	ret = get_systemd_sockets(ml);
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < IPC_LISTENER_COUNT; i++) {
		if (ml->listeners[i].fd >= 0) {
			continue;
		}
		ret = create_listen_socket(ml, (enum ipc_listener_kind)i);
		if (ret < 0) {
			return ret;
		}
	}

	// All ok!
	U_LOG_D("Listening sockets are fd %d (app) and fd %d (control)", ml->listeners[IPC_LISTENER_APP].fd,
	        ml->listeners[IPC_LISTENER_CONTROL].fd);

	return 0;
}

static volatile sig_atomic_t got_shutdown_signal = 0;

static void
shutdown_signal_handler(int sig)
{
	(void)sig;
	got_shutdown_signal = 1;
}

static void
install_signal_handlers(void)
{
	struct sigaction sa = {0};
	sa.sa_handler = shutdown_signal_handler;
	sigemptyset(&sa.sa_mask);
	// SA_RESETHAND: after first signal, reset to default so a second signal kills immediately.
	sa.sa_flags = SA_RESETHAND;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
}

static int
epoll_add_fd(struct ipc_server_mainloop *ml, int fd)
{
	struct epoll_event ev = {0};
	ev.events = EPOLLIN;
	ev.data.fd = fd;
	int ret = epoll_ctl(ml->epoll_fd, EPOLL_CTL_ADD, fd, &ev);
	if (ret < 0) {
		U_LOG_E("epoll_ctl(%d) failed '%i'", fd, ret);
	}
	return ret;
}

static int
init_epoll(struct ipc_server_mainloop *ml, bool no_stdin)
{
	int ret = epoll_create1(EPOLL_CLOEXEC);
	if (ret < 0) {
		return ret;
	}

	ml->epoll_fd = ret;

	if (!ml->launched_by_socket && !no_stdin) {
		// Can't do this when launched by systemd socket activation by
		// default.
		// This polls stdin.
		ret = epoll_add_fd(ml, 0);
		if (ret < 0) {
			return ret;
		}
	}

	for (int i = 0; i < IPC_LISTENER_COUNT; i++) {
		ret = epoll_add_fd(ml, ml->listeners[i].fd);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static void
handle_listen(struct ipc_server *vs,
              struct ipc_server_mainloop *ml,
              int listen_fd,
              enum ipc_client_role role,
              const struct ipc_client_sandbox_info *sandbox)
{
	int ret = accept(listen_fd, NULL, NULL);
	if (ret < 0) {
		if (role == IPC_CLIENT_ROLE_SANDBOXED_APP && sandbox != NULL) {
			// A registered listener may have been removed between the
			// epoll wakeup and this accept; that is not the server's
			// failure.
			U_LOG_W("accept on sandbox listener failed: %s", strerror(errno));
			return;
		}
		U_LOG_E("accept '%i'", ret);
		ipc_server_handle_failure(vs);
		return;
	}

	// An ordinary arrival may still be a sandboxed process; that lookup can
	// only ever lower the role, never raise it.
	struct ipc_client_sandbox_info lowered = XRT_STRUCT_INIT;
	if (role == IPC_CLIENT_ROLE_APP && ipc_server_peer_is_sandboxed(ret, &lowered)) {
		role = IPC_CLIENT_ROLE_SANDBOXED_APP;
		sandbox = &lowered;
	}

	// Call into the generic client connected handling code.
	ipc_server_handle_client_connected(vs, ret, role, sandbox);
}

/*!
 * Dispatch a readable fd to the listener it belongs to, if any.
 *
 * @return true if the fd was one of ours.
 */
static bool
dispatch_listener_fd(struct ipc_server *vs, struct ipc_server_mainloop *ml, int fd)
{
	for (int i = 0; i < IPC_LISTENER_COUNT; i++) {
		if (ml->listeners[i].fd == fd) {
			handle_listen(vs, ml, fd, ml->listeners[i].role, NULL);
			return true;
		}
	}

	// Sandbox listeners are registered and removed by client threads under
	// the global lock; copy what we need out and accept without it (the
	// connected handler takes the lock itself).
	struct ipc_client_sandbox_info info = XRT_STRUCT_INIT;
	bool found = false;
	os_mutex_lock(&vs->global_state.lock);
	for (int i = 0; i < IPC_MAX_SANDBOX_LISTENERS; i++) {
		if (ml->sandbox_listeners[i].fd == fd) {
			info = ml->sandbox_listeners[i].info;
			found = true;
			break;
		}
	}
	os_mutex_unlock(&vs->global_state.lock);

	if (found) {
		handle_listen(vs, ml, fd, IPC_CLIENT_ROLE_SANDBOXED_APP, &info);
		return true;
	}

	return false;
}

#define NUM_POLL_EVENTS 8
#define NO_SLEEP 0

/*
 *
 * Exported functions
 *
 */

void
ipc_server_mainloop_poll(struct ipc_server *vs, struct ipc_server_mainloop *ml)
{
	IPC_TRACE_MARKER();

	int epoll_fd = ml->epoll_fd;

	struct epoll_event events[NUM_POLL_EVENTS] = {0};

	// No sleeping, returns immediately.
	int ret = epoll_wait(epoll_fd, events, NUM_POLL_EVENTS, NO_SLEEP);
	if (ret < 0) {
		U_LOG_E("epoll_wait failed with '%i'.", ret);
		ipc_server_handle_failure(vs);
		return;
	}

	// Check if a signal handler set the shutdown flag.
	if (got_shutdown_signal) {
		U_LOG_I("Got shutdown signal, shutting down.");
		ipc_server_handle_shutdown_signal(vs);
		return;
	}

	for (int i = 0; i < ret; i++) {
		// If we get data on stdin, stop.
		if (events[i].data.fd == 0) {
			ipc_server_handle_shutdown_signal(vs);
			return;
		}

		// Somebody new at the door.
		dispatch_listener_fd(vs, ml, events[i].data.fd);
	}
}

int
ipc_server_mainloop_init(struct ipc_server_mainloop *ml, bool no_stdin)
{
	IPC_TRACE_MARKER();

	int ret = init_listen_sockets(ml);
	if (ret < 0) {
		ipc_server_mainloop_deinit(ml);
		return ret;
	}

	install_signal_handlers();

	ret = init_epoll(ml, no_stdin);
	if (ret < 0) {
		ipc_server_mainloop_deinit(ml);
		return ret;
	}
	return 0;
}

void
ipc_server_mainloop_deinit(struct ipc_server_mainloop *ml)
{
	IPC_TRACE_MARKER();

	if (ml == NULL) {
		return;
	}
	for (int i = 0; i < IPC_LISTENER_COUNT; i++) {
		struct ipc_listener *l = &ml->listeners[i];
		if (l->fd < 0) {
			continue;
		}
		// Close socket on exit
		close(l->fd);
		l->fd = -1;
		if (l->filename != NULL) {
			// Unlink it too, but only if we bound it.
			unlink(l->filename);
			free(l->filename);
			l->filename = NULL;
		}
	}
	for (int i = 0; i < IPC_MAX_SANDBOX_LISTENERS; i++) {
		if (ml->sandbox_listeners[i].fd >= 0) {
			close(ml->sandbox_listeners[i].fd);
			ml->sandbox_listeners[i].fd = -1;
			ml->sandbox_listeners[i].owner_index = -1;
		}
	}
	//! @todo close epoll_fd?
}

int
ipc_server_mainloop_add_sandbox_listener(struct ipc_server_mainloop *ml,
                                         int listen_fd,
                                         int owner_index,
                                         const struct ipc_client_sandbox_info *info)
{
	for (int i = 0; i < IPC_MAX_SANDBOX_LISTENERS; i++) {
		struct ipc_sandbox_listener *sl = &ml->sandbox_listeners[i];
		if (sl->fd >= 0) {
			continue;
		}

		int ret = epoll_add_fd(ml, listen_fd);
		if (ret < 0) {
			return ret;
		}

		sl->fd = listen_fd;
		sl->owner_index = owner_index;
		sl->info = *info;
		U_LOG_I("Sandbox listener registered (engine='%s', app_id='%s')", info->engine, info->app_id);
		return 0;
	}

	U_LOG_E("No free sandbox listener slot (max %d)", IPC_MAX_SANDBOX_LISTENERS);
	return -1;
}

void
ipc_server_mainloop_remove_sandbox_listeners(struct ipc_server_mainloop *ml, int owner_index)
{
	for (int i = 0; i < IPC_MAX_SANDBOX_LISTENERS; i++) {
		struct ipc_sandbox_listener *sl = &ml->sandbox_listeners[i];
		if (sl->fd < 0 || sl->owner_index != owner_index) {
			continue;
		}
		// Closing the fd removes it from the epoll set as well.
		close(sl->fd);
		sl->fd = -1;
		sl->owner_index = -1;
		U_LOG_I("Sandbox listener removed (engine='%s', app_id='%s')", sl->info.engine, sl->info.app_id);
	}
}
