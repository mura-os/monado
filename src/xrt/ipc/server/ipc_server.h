// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common server side code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup ipc_server
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_limits.h"
#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"

#include "os/os_threading.h"

#include "util/u_logging.h"
#include "util/u_hashmap.h"

#include "shared/ipc_protocol.h"
#include "shared/ipc_message_channel.h"

#include "ipc_server_interface.h"
#include "ipc_server_lease.h"

#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

/*
 *
 * Logging
 *
 */

#define IPC_TRACE(d, ...) U_LOG_IFL_T(d->log_level, __VA_ARGS__)
#define IPC_DEBUG(d, ...) U_LOG_IFL_D(d->log_level, __VA_ARGS__)
#define IPC_INFO(d, ...) U_LOG_IFL_I(d->log_level, __VA_ARGS__)
#define IPC_WARN(d, ...) U_LOG_IFL_W(d->log_level, __VA_ARGS__)
#define IPC_ERROR(d, ...) U_LOG_IFL_E(d->log_level, __VA_ARGS__)

#define IPC_CHK_AND_RET(S, ...) U_LOG_CHK_AND_RET((S)->log_level, __VA_ARGS__)
#define IPC_CHK_WITH_GOTO(S, ...) U_LOG_CHK_WITH_GOTO((S)->log_level, __VA_ARGS__)
#define IPC_CHK_WITH_RET(S, ...) U_LOG_CHK_WITH_RET((S)->log_level, __VA_ARGS__)
#define IPC_CHK_ONLY_PRINT(S, ...) U_LOG_CHK_ONLY_PRINT((S)->log_level, __VA_ARGS__)
#define IPC_CHK_ALWAYS_RET(S, ...) U_LOG_CHK_ALWAYS_RET((S)->log_level, __VA_ARGS__)


/*
 *
 * Structs
 *
 */

#define IPC_MAX_CLIENT_BODY_TRACKERS 16
#define IPC_MAX_CLIENT_HAND_TRACKERS 16
#define IPC_MAX_CLIENT_APP_INSTANCES 4
#define IPC_MAX_CLIENT_APP_SYSTEMS (IPC_MAX_CLIENT_APP_INSTANCES * 4)
#define IPC_MAX_CLIENT_SEMAPHORES 8
#define IPC_MAX_CLIENT_SWAPCHAINS (XRT_MAX_LAYERS * 2)
#define IPC_MAX_CLIENT_SPACES 128
#define IPC_MAX_CLIENT_FUTURES 128

struct xrt_instance;
struct xrt_body_tracker;
struct xrt_hand_tracker;
struct xrt_compositor;
struct xrt_compositor_native;

/*!
 * The fixed listening sockets of the service, by the role they admit.
 *
 * @ingroup ipc_server
 */
enum ipc_listener_kind
{
	//! The ordinary service socket (@ref XRT_IPC_MSG_SOCK_FILENAME): apps.
	IPC_LISTENER_APP = 0,

	//! The control socket (@ref XRT_IPC_MSG_SOCK_CONTROL_FILENAME): controllers.
	IPC_LISTENER_CONTROL = 1,

	IPC_LISTENER_COUNT,
};

//! How many sandbox listeners a controller may register at once.
#define IPC_MAX_SANDBOX_LISTENERS 4


/*!
 * Information about a single swapchain.
 *
 * @ingroup ipc_server
 */
struct ipc_swapchain_data
{
	uint32_t width;
	uint32_t height;
	uint64_t format;
	uint32_t image_count;

	bool active;
};


/*!
 * Holds the state for a single client.
 *
 * @ingroup ipc_server
 */
struct ipc_client_state
{
	//! Link back to the main server.
	struct ipc_server *server;

	//! Has the system part of the shm initialized.
	bool has_init_shm_system;

	struct
	{
		/*!
		 * Array of tracking origins.
		 *
		 * We don't control the lifetime of the tracking origins,
		 * and we only access it from the per client thread,
		 * so we don't need to lock it.
		 */
		struct xrt_tracking_origin *xtracks[XRT_SYSTEM_MAX_DEVICES];

		/*!
		 * Array of devices.
		 *
		 * We don't control the lifetime of the devices,
		 * and we only access it from the per client thread,
		 * so we don't need to lock it.
		 */
		struct xrt_device *xdevs[XRT_SYSTEM_MAX_DEVICES];

		/*!
		 * Body trackers owned by this client.
		 */
		struct xrt_body_tracker *xbts[IPC_MAX_CLIENT_BODY_TRACKERS];

		/*!
		 * Hand trackers owned by this client.
		 */
		struct xrt_hand_tracker *xhts[IPC_MAX_CLIENT_HAND_TRACKERS];

		/*!
		 * Array of app instances owned by this client.
		 */
		struct xrt_app_instance *xainsts[IPC_MAX_CLIENT_APP_INSTANCES];

		/*!
		 * Array of app systems owned by this client.
		 */
		struct xrt_app_system *xasys[IPC_MAX_CLIENT_APP_SYSTEMS];
	} objects;

	//! Session for this client.
	struct xrt_session *xs;

	//! Compositor for this client.
	struct xrt_compositor *xc;

	//! Number of swapchains in use by client
	uint32_t swapchain_count;

	//! Ptrs to the swapchains
	struct xrt_swapchain *xscs[IPC_MAX_CLIENT_SWAPCHAINS];

	//! Data for the swapchains.
	struct ipc_swapchain_data swapchain_data[IPC_MAX_CLIENT_SWAPCHAINS];

	//! Number of compositor semaphores in use by client
	uint32_t compositor_semaphore_count;

	//! Ptrs to the semaphores.
	struct xrt_compositor_semaphore *xcsems[IPC_MAX_CLIENT_SEMAPHORES];

	//! Ptrs to the futures.
	struct xrt_future *xfts[IPC_MAX_CLIENT_FUTURES];

	struct
	{
		uint32_t root;
		uint32_t local;
		uint32_t stage;
		uint32_t unbounded;
	} semantic_spaces;

	//! Number of spaces.
	uint32_t space_count;
	//! Index of localspace in ipc client.
	uint32_t local_space_index;
	//! Index of localspace in space overseer.
	uint32_t local_space_overseer_index;
	//! Index of localfloorspace in ipc client.
	uint32_t local_floor_space_index;
	//! Index of localfloorspace in space overseer.
	uint32_t local_floor_space_overseer_index;

	//! Ptrs to the spaces.
	struct xrt_space *xspcs[IPC_MAX_CLIENT_SPACES];

	//! Which of the references spaces is the client using.
	bool ref_space_used[XRT_SPACE_REFERENCE_TYPE_COUNT];

	//! Which of the device features is the client using.
	bool device_feature_used[XRT_DEVICE_FEATURE_MAX_ENUM];

	//! Socket fd used for client comms
	struct ipc_message_channel imc;

	struct ipc_app_state client_state;


	uint64_t plane_detection_size;
	uint64_t plane_detection_count;

	//! Array of plane detection ids with plane_detection_size entries.
	uint64_t *plane_detection_ids;

	//! Array of xrt_devices with plane_detection_size entries.
	struct xrt_device **plane_detection_xdev;

	int server_thread_index;

	xrt_shmem_handle_t ism_handle;
};

enum ipc_thread_state
{
	IPC_THREAD_READY,
	IPC_THREAD_STARTING,
	IPC_THREAD_RUNNING,
	IPC_THREAD_STOPPING,
};

struct ipc_thread
{
	struct os_thread thread;
	volatile enum ipc_thread_state state;
	volatile struct ipc_client_state ics;
};

/*!
 * Platform-specific mainloop object for the IPC server.
 *
 * Contents are essentially implementation details, but are listed in full here so they may be included by value in the
 * main ipc_server struct.
 *
 * @see ipc_design
 *
 * @ingroup ipc_server
 */
struct ipc_server_mainloop
{

#if defined(XRT_OS_ANDROID) || defined(XRT_OS_LINUX) || defined(XRT_DOXYGEN)
	//! For waiting on various events in the main thread.
	int epoll_fd;
#endif

#if defined(XRT_OS_ANDROID) || defined(XRT_DOXYGEN)
	/*!
	 * @name Android Mainloop Members
	 * @{
	 */

	//! File descriptor for the read end of our pipe for submitting new clients
	int pipe_read;

	/*!
	 * File descriptor for the write end of our pipe for submitting new clients
	 *
	 * Must hold client_push_mutex while writing.
	 */
	int pipe_write;

	/*!
	 * Mutex for being able to register oneself as a new client.
	 *
	 * Locked only by threads in `ipc_server_mainloop_add_fd()`.
	 *
	 * This must be locked first, and kept locked the entire time a client is attempting to register and wait for
	 * confirmation. It ensures no acknowledgements of acceptance are lost and moves the overhead of ensuring this
	 * to the client thread.
	 */
	pthread_mutex_t client_push_mutex;


	/*!
	 * The last client fd we accepted, to acknowledge client acceptance.
	 *
	 * Also used as a sentinel during shutdown.
	 *
	 * Must hold accept_mutex while writing.
	 */
	int last_accepted_fd;

	/*!
	 * Condition variable for accepting clients.
	 *
	 * Signalled when @ref last_accepted_fd is updated.
	 *
	 * Associated with @ref accept_mutex
	 */
	pthread_cond_t accept_cond;

	/*!
	 * Mutex for accepting clients.
	 *
	 * Locked by both clients and server: that is, by threads in `ipc_server_mainloop_add_fd()` and in the
	 * server/compositor thread in an implementation function called from `ipc_server_mainloop_poll()`.
	 *
	 * Exists to operate in conjunction with @ref accept_cond - it exists to make sure that the client can be woken
	 * when the server accepts it.
	 */
	pthread_mutex_t accept_mutex;


	/*! @} */
#define XRT_IPC_GOT_IMPL
#endif

#if (defined(XRT_OS_LINUX) && !defined(XRT_OS_ANDROID)) || defined(XRT_DOXYGEN)
	/*!
	 * @name Desktop Linux Mainloop Members
	 * @{
	 */

	/*!
	 * The sockets we accept connections on, one per @ref ipc_listener_kind.
	 * Every accept stamps the new client with the listener's role, which is
	 * where a client's authority comes from (see @ref ipc_client_role).
	 */
	struct ipc_listener
	{
		//! Listening socket fd, -1 if this listener is not open.
		int fd;

		//! Role given to every client accepted here.
		enum ipc_client_role role;

		//! The socket filename we bound to, NULL if systemd passed the fd.
		char *filename;
	} listeners[IPC_LISTENER_COUNT];

	//! Were we launched by socket activation, instead of explicitly?
	bool launched_by_socket;

	/*!
	 * Listening sockets registered at runtime by the controller for
	 * sandboxed clients (@ref ipc_handle_system_add_sandbox_listener):
	 * every accept is @ref IPC_CLIENT_ROLE_SANDBOXED_APP with the stored
	 * sandbox info. Removed when the registering client goes away.
	 */
	struct ipc_sandbox_listener
	{
		//! Listening socket fd, -1 when the slot is free.
		int fd;

		//! Server thread index of the controller that registered it.
		int owner_index;

		struct ipc_client_sandbox_info info;
	} sandbox_listeners[IPC_MAX_SANDBOX_LISTENERS];

	/*! @} */

#define XRT_IPC_GOT_IMPL
#endif

#if defined(XRT_OS_WINDOWS) || defined(XRT_DOXYGEN)
	/*!
	 * @name Desktop Windows Mainloop Members
	 * @{
	 */

	//! Named Pipe that we accept connections on.
	HANDLE pipe_handle;

	//! Name of the Pipe that we accept connections on.
	char *pipe_name;

	/*! @} */

#define XRT_IPC_GOT_IMPL
#endif

#ifndef XRT_IPC_GOT_IMPL
#error "Need port"
#endif
};

/*!
 * De-initialize the mainloop object.
 * @public @memberof ipc_server_mainloop
 */
void
ipc_server_mainloop_deinit(struct ipc_server_mainloop *ml);

/*!
 * Initialize the mainloop object.
 *
 * @return <0 on error.
 * @public @memberof ipc_server_mainloop
 */
int
ipc_server_mainloop_init(struct ipc_server_mainloop *ml, bool no_stdin);

/*!
 * @brief Poll the mainloop.
 *
 * Any errors are signalled by calling ipc_server_handle_failure()
 * @public @memberof ipc_server_mainloop
 */
void
ipc_server_mainloop_poll(struct ipc_server *vs, struct ipc_server_mainloop *ml);

#if defined(XRT_OS_LINUX) && !defined(XRT_OS_ANDROID)
/*!
 * Register a listening socket whose arrivals are admitted as
 * @ref IPC_CLIENT_ROLE_SANDBOXED_APP, carrying @p info.
 *
 * Called by a client thread holding @ref ipc_server::global_state lock; the
 * server takes ownership of @p listen_fd. Removed again by
 * ipc_server_mainloop_remove_sandbox_listeners() when @p owner_index
 * disconnects.
 *
 * @return <0 on error (no free slot, or epoll failure), the fd is then still
 *         the caller's to close.
 * @public @memberof ipc_server_mainloop
 */
int
ipc_server_mainloop_add_sandbox_listener(struct ipc_server_mainloop *ml,
                                         int listen_fd,
                                         int owner_index,
                                         const struct ipc_client_sandbox_info *info);

/*!
 * Close and forget every sandbox listener registered by @p owner_index.
 *
 * Called holding @ref ipc_server::global_state lock.
 * @public @memberof ipc_server_mainloop
 */
void
ipc_server_mainloop_remove_sandbox_listeners(struct ipc_server_mainloop *ml, int owner_index);
#endif

/*!
 * Main IPC object for the server.
 *
 * @ingroup ipc_server
 */
struct ipc_server
{
	struct xrt_instance *xinst;

	//! Handle for the current process, e.g. pidfile on linux
	struct u_process *process;

	struct u_debug_gui *debug_gui;

	//! The @ref xrt_iface level system.
	struct xrt_system *xsys;

	//! System devices.
	struct xrt_system_devices *xsysd;

	//! Space overseer.
	struct xrt_space_overseer *xso;

	//! System compositor.
	struct xrt_system_compositor *xsysc;

	struct ipc_shared_memory *isms[IPC_MAX_CLIENTS];

	struct ipc_server_mainloop ml;

	// Is the mainloop supposed to run.
	volatile bool running;

	// Should we exit when a client disconnects.
	bool exit_on_disconnect;

	// Should we exit when no clients are connected.
	bool exit_when_idle;

	// Timestamp when last client disconnected (for exit_when_idle delay)
	uint64_t last_client_disconnect_ns;

	// How long to wait after all clients disconnect before exiting (in nanoseconds)
	uint64_t exit_when_idle_delay_ns;

	/*!
	 * Global start of time timestamp. XrTime that is returned to apps
	 * start from this timestamps (with an per app offset applied). In order
	 * to ensure that all timestamps for events and other data that is
	 * tracked by the runtime within the valid range of XrTime it is offset
	 * to be 42 minutes in the past from start of the runtime.
	 *
	 * Used to get all the client's xrt_instance::startup_timestamp, to be
	 * close to each other but not exactly the same, which is then used to
	 * be the base of that app's XrTime start of time.
	 */
	int64_t start_of_time_timestamp_ns;

	enum u_logging_level log_level;

	struct ipc_thread threads[IPC_MAX_CLIENTS];

	volatile uint32_t current_slot_index;

	//! Generator for IDs.
	uint32_t id_generator;

	struct
	{
		int active_client_index;
		int last_active_client_index;

		// Counter for total number of connected clients
		uint32_t connected_client_count;

		//! Which controller client (if any) may drive the service.
		struct ipc_controller_lease lease;

		struct os_mutex lock;
	} global_state;

	/*!
	 * Callbacks for server events.
	 */
	const struct ipc_server_callbacks *callbacks;

	/*!
	 * User data passed to callbacks.
	 */
	void *callback_data;

	//! Disable listening on stdin for server stop.
	bool no_stdin;
};

/*!
 * Finish setting up the server by creating the system, compositor and devices.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_init_system_if_available_locked(struct ipc_server *s,
                                           volatile struct ipc_client_state *ics,
                                           bool *out_available);

/*!
 * Get the current state of a client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_get_client_app_state(struct ipc_server *s, uint32_t client_id, struct ipc_app_state *out_ias);

/*!
 * May this client execute a control verb?
 *
 * The lease holder always may. With @p legacy_open, any client may while no
 * controller holds the lease and the `IPC_REQUIRE_CONTROLLER` option is off;
 * verbs introduced after the lease pass false and are the holder's
 * unconditionally.
 *
 * @return XRT_SUCCESS or XRT_ERROR_IPC_NOT_CONTROLLER; the connection stays.
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_check_controller(volatile struct ipc_client_state *ics, bool legacy_open);

/*!
 * Where this client stands with the controller lease.
 * @ingroup ipc_server
 */
enum ipc_controller_state
ipc_server_get_controller_state(volatile struct ipc_client_state *ics);

/*!
 * Set the new active client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_set_active_client(struct ipc_server *s, uint32_t client_id);

/*!
 * Toggle the io for this client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_toggle_io_client(struct ipc_server *s, uint32_t client_id);

/*!
 * Block certain types of IO for this client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_set_client_io_blocks(struct ipc_server *s, uint32_t client_id, const struct ipc_client_io_blocks *blocks);

/*!
 * Get the session running state for this client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_get_client_session_running_state(struct ipc_server *s,
                                            uint32_t client_id,
                                            struct xrt_compositor_session_running_state *out_running_state);

/*!
 * Get the view configuration for this client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_get_client_view_config(struct ipc_server *s,
                                  uint32_t client_id,
                                  enum xrt_view_type view_type,
                                  struct xrt_view_config *out_default_view_config,
                                  struct xrt_recommended_view_config *out_recommended_view_config);

/*!
 * Set the recommended view configuration for this client.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_set_client_recommended_view_config(struct ipc_server *s,
                                              uint32_t client_id,
                                              enum xrt_view_type view_type,
                                              const struct xrt_recommended_view_config *recommended_view_config);

/*!
 * Called by client threads to set a session to active.
 *
 * @ingroup ipc_server
 */
void
ipc_server_activate_session(volatile struct ipc_client_state *ics);

/*!
 * Called by client threads to set a session to deactivate.
 *
 * @ingroup ipc_server
 */
void
ipc_server_deactivate_session(volatile struct ipc_client_state *ics);

/*!
 * Called by client threads to recalculate active client.
 *
 * @ingroup ipc_server
 */
void
ipc_server_update_state(struct ipc_server *s);

/*!
 * Thread function for the client side dispatching.
 *
 * @ingroup ipc_server
 */
void *
ipc_server_client_thread(void *_ics);

/*!
 * This destroys the native compositor for this client and any extra objects
 * created from it, like all of the swapchains.
 */
void
ipc_server_client_destroy_session_and_compositor(volatile struct ipc_client_state *ics);

/*!
 * @defgroup ipc_server_internals Server Internals
 * @brief These are only called by the platform-specific mainloop polling code.
 * @ingroup ipc_server
 * @{
 */
/*!
 * Called when a client has connected, it takes the client's ipc handle.
 * Handles all things needed to be done for a client connecting, like starting
 * it's thread.
 *
 * The role is decided by the caller from where the connection came from (which
 * listening socket, or which registered listener) and is the only thing the
 * client's authority is ever derived from, see @ref ipc_client_role.
 *
 * @param vs         The IPC server.
 * @param ipc_handle Handle to communicate over.
 * @param role       How the connection was admitted.
 * @param sandbox    Sandbox metadata for @ref IPC_CLIENT_ROLE_SANDBOXED_APP, or NULL.
 * @memberof ipc_server
 */
void
ipc_server_handle_client_connected(struct ipc_server *vs,
                                   xrt_ipc_handle_t ipc_handle,
                                   enum ipc_client_role role,
                                   const struct ipc_client_sandbox_info *sandbox);

/*!
 * Perform whatever needs to be done when the mainloop polling encounters a failure.
 * @memberof ipc_server
 */
void
ipc_server_handle_failure(struct ipc_server *vs);

/*!
 * Perform whatever needs to be done when the mainloop polling identifies that the server should be shut down.
 *
 * Does something like setting a flag or otherwise signalling for shutdown: does not itself explicitly exit.
 * @memberof ipc_server
 */
void
ipc_server_handle_shutdown_signal(struct ipc_server *vs);

xrt_result_t
ipc_server_get_system_properties(struct ipc_server *vs, struct xrt_system_properties *out_properties);
//! @}

/*
 *
 * Helpers
 *
 */

/*!
 * Get the data in the shared memory of the given client.
 */
static inline struct ipc_shared_memory *
get_ism(volatile struct ipc_client_state *ics)
{
	return ics->server->isms[ics->server_thread_index];
}

/*!
 * Get the handle for the shared memory of the given client.
 */
static inline xrt_shmem_handle_t
get_ism_handle(volatile struct ipc_client_state *ics)
{
	return ics->ism_handle;
}

#ifdef __cplusplus
}
#endif
