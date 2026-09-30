# IPC Design and Implementation {#ipc-design}

<!--
Copyright 2021-2022, Collabora, Ltd. and the Monado contributors
SPDX-License-Identifier: BSL-1.0
-->

[TOC]

- Last updated: 12-September-2022

When the service starts, an `xrt_instance` is created and selected, a native
compositor is initialized by a system compositor, a shared memory segment for
device data is initialized, and other internal state is set up. (See
`ipc_server_process.c`.)

There are three main communication needs:

- The client shared library needs to be able to **locate** a running service, if
  any, to start communication. (Auto-starting, where available, is handled by
  platform-specific mechanisms: the client currently has no code to explicitly
  start up the service.) This location mechanism must be able to establish or
  share the RPC channel and shared memory access, often by passing a socket,
  handle, or file descriptor.
- The client and service must share a dedicated channel for IPC calls (also
  known as **RPC** - remote procedure call), typically a socket. Importantly,
  the channel must be able to carry both data messages and native graphics
  buffer/sync handles (file descriptors, HANDLEs, AHardwareBuffers)
- The service must share device data updating at various rates, shared by all
  clients. This is typically done with a form of **shared memory**.

Each platform's implementation has a way of meeting each of these needs. The
specific way each need is met is highlighted below.

## Linux Platform Details

In an typical Linux environment, the Monado service can be launched one of two
ways: manually, or by socket activation (e.g. from systemd). In either case,
there is a Unix domain socket with a well-known name (known at compile time, and
built-in to both the service executable and the client shared library) used by
clients to connect to the service: this provides the **locating** function.
This socket is polled in the service mainloop, using epoll, to detect any new
client connections.

Upon a client connection to this "locating" socket, the service will [accept][]
the connection, returning a file descriptor (FD), which is passed to
`start_client_listener_thread()` to start a thread specific to that client. The
FD produced this way is now also used for the IPC calls - the **RPC** function -
since it is specific to that client-server communication channel. One of the
first calls made transports a duplicate of the **shared memory** segment file
descriptor to the client, so it has (read) access to this data.

[accept]: https://man7.org/linux/man-pages/man2/accept.2.html

### Client roles and the controller lease

The service listens on two well-known sockets, not one. Ordinary clients (the
OpenXR client library) connect to `monado_comp_ipc`; a session controller (a
shell, `monado-ctl`, anything using `libmonado` on `MND_SOCKET_CONTROL`)
connects to `monado_comp_ipc_control`. **Which socket a connection arrives on is
its identity to the service.** At `accept()` the service stamps the new client
with an `ipc_client_role`:

- `IPC_CLIENT_ROLE_APP` — arrived on the application socket.
- `IPC_CLIENT_ROLE_CONTROLLER` — arrived on the control socket.
- `IPC_CLIENT_ROLE_SANDBOXED_APP` — arrived on a listener a controller
  registered with `system_add_sandbox_listener`, *or* arrived on the
  application socket but was found to be running inside a Flatpak or Snap
  (`/proc/<pid>/root/.flatpak-info`, or a systemd user unit named
  `app-flatpak-*`/`snap.*`, looked up through `SO_PEERPIDFD` where available).
  This lookup can only ever lower a client; nothing raises one to controller.

The role is never sent by the client and `instance_describe_client` never
touches it. Under systemd the two sockets are separate units
(`monado.socket`, `monado-control.socket`) passed to the service with
`FileDescriptorName=app`/`control`; a single unnamed fd is treated as the
application socket, so older unit files keep working. When started by hand the
service binds both sockets itself, and a deployment that wants to restrict who
may reach the control socket does so with the ordinary Unix means —
directory permissions, the socket unit's `SocketMode=`/`SocketUser=`, or a
sandbox that does not mount the runtime directory.

Same-uid clients cannot be told apart by credentials, so this is not an
authentication scheme; it is the same split PipeWire (manager vs. client
socket), seatd (VT vs. non-VT) and Wayland compositors (`wp_security_context_v1`
listeners) use: **the arrival path is the identity, the runtime enforces, and
policy lives in the controller.**

Among controllers there is one **lease**. The first controller to connect holds
it; later controllers queue in arrival order and are promoted when the holder
disconnects; a holder is never displaced; nothing is persisted, so a restarted
controller finds the service exactly as its verbs left it. The control verbs —
`system_set_primary_client`, `system_set_focused_client`,
`system_toggle_io_client`, `system_set_client_io_blocks`,
`system_set_client_recommended_view_config`, `system_add_sandbox_listener` —
succeed only for the holder; every other connection gets
`XRT_ERROR_IPC_NOT_CONTROLLER` (libmonado: `MND_ERROR_NOT_CONTROLLER`) and
stays connected. Read-only enumeration (`system_get_clients`,
`system_get_client_info`, `system_get_controller_state`) is open to every role.

For compatibility, the four verbs that predate the lease remain callable from
any connection *while no controller holds the lease*, unless the service runs
with `IPC_REQUIRE_CONTROLLER=true`. That keeps an unmodified `monado-ctl` on the
application socket working for existing users; a deployment with a shell that
owns the session sets the option and the verbs are the holder's from first boot.
Verbs added after the lease are the holder's unconditionally. A controller
learns whether it holds or is pending by calling `system_get_controller_state`
(`mnd_root_get_controller_state`); there is no push event.

The lease itself is a small pure state machine (`ipc_server_lease.h`) driven
under `global_state.lock` from the connect and disconnect paths; it has no
knowledge of sockets or threads and is unit-tested on its own.

## Android Platform Details

On Android, to pass platform objects, allow for service activation, and
fit better within the idioms of the platform, Monado provides a Binder/AIDL
service instead of a named socket. (The named sockets we typically use are not
permitted by the platform, and "abstract" named sockets are currently available,
but are not idiomatic for the platform and lack other useful capabilities.)
Specifically, we provide a [foreground and started][foreground] (to be able to
display), [bound][bound_service] [service][android_service] with an interface
defined using [AIDL][]. (See also
[this third-party guide about such AIDL services][AidlServices]) This is not
like the system services which provide hardware data or system framework data
from native code. this has a Java (JVM/Dalvik/ART) component provided by code in
an APK, exposed by properties in the package manifest.

[NdkBinder][] is not used because it is mainly suitable for the system type of
binder services. An APK-based service would still require some JVM code to
expose it, and since the AIDL service is used for so little, mixing languages
did not make sense.

The service we expose provides an implementation of our AIDL-described
interface, `org.freedesktop.monado.ipc.IMonado`. This can be modified freely, as
both the client and server are built at the same time and packaged in the same
APK, even though they get loaded in different processes.

[foreground]: https://developer.android.com/guide/components/foreground-services
[bound_service]: https://developer.android.com/guide/components/bound-services
[android_service]: https://developer.android.com/guide/components/services
[aidl]: https://developer.android.com/guide/components/aidl
[AidlServices]: https://devarea.com/android-services-and-aidl/
[NdkBinder]: https://developer.android.com/ndk/reference/group/ndk-binder

The first main purpose of this service is for automatic startup and the
**locating** function: helping establish communication between the client and
the service. The Android framework takes care of launching the service process
when the client requests to bind our service by name and package. The framework
also provides us with method calls when we're bound. In this way, the "entry point"
of the Monado service on Android is the
`org.freedesktop.monado.ipc.MonadoService` class, which exposes the
implementation of our AIDL interface, `org.freedesktop.monado.ipc.MonadoImpl`.

From there, the native-code mainloop starts when this service received a valid
`Surface`. By default, the JVM code will signal the mainloop to shut down a short
time after the last client disconnects, to work best within the platform.

At startup, as on Linux, the shared memory segment is created. The [ashmem][]
API is used to create/destroy an anonymous **shared memory** segment on Android,
instead of standard POSIX shared memory, but is otherwise treated and used
exactly the same as on standard Linux: file descriptors are duplicated and
passed through IPC calls, etc.

When the client side starts up, it creates an __anonymous socket pair__ to use
for IPC calls (the **RPC** function) later. It then passes one of the two file
descriptors into the AIDL method we defined named "connect". This transports the
FD to the service process, which uses it as the unique communication channel for
that client in its own thread. This replaces the socket pair produced by
connecting/accepting the named socket as used in standard Linux.

[ashmem]: https://developer.android.com/ndk/reference/group/memory

The AIDL interface is also used for transporting some platform objects. At this
time, the only one transported in this way is the [Surface][] injected into the
client activity which is used for displaying rendered output. Surface only comes
from client when  [Display  over other apps][] is disabled.

The owner of surface will impact the service shutdown behavior. When the
surface comes from the injected window, it becomes invalid when client activity
destroys. Therefore the runtime service must be shutdown when client exits,
because all the graphic resources are associated with that surface. On the other
hand,  when the owner of surface is the runtime service, it's capable to support
multiple clients and client transition without shutdown.

[Surface]: https://developer.android.com/reference/android/view/Surface
[Display over other apps]: https://developer.android.com/reference/android/Manifest.permission#SYSTEM_ALERT_WINDOW

### Synchronization

Synchronization of new client connections is a special challenge on the Android
platform, since new clients arrive using calls into JVM code while the mainloop is
C/C++ code. Unlike Linux, we cannot simply use epoll to check if there are new
connections to our locating socket.

We have the following design goals/constraints:

- All we need to communicate is an integer (file descriptor) within a process.
- Make it fast in the server mainloop in the most common case that there are no
  new clients.
  - This suggests that we should be able to check if there may be a waiting
    client in purely native code, without JNI.
- Make it relatively fast in the server mainloop even when there is a client,
  since it's the compositor thread.
  - This might mean we want to do it all without JNI on the main thread.
- The client should know (and be unblocked) when the server has accepted its
  connection.
  - This suggests that the method called in `MonadoImpl` should block until the
    server consumes/accepts the connection.
  - Not 100% sure this is required, but maybe.
- Resources (file descriptors, etc) should not be leaked.
  - Each should have a well-known owner at each point in time.
- It is OK if only one new client is accepted per mainloop.
  - The mainloop is high rate (compositor rate) and new client connections are
    relatively infrequent.

The IPC service creates a pipe as well as some state variables, two mutexes, and a
condition variable.

When the JVM Service code has a new client, it calls
`ipc_server_mainloop_add_fd()` to pass the FD in. It takes two mutexes, in
order: `ipc_server_mainloop::client_push_mutex` and
`ipc_server_mainloop::accept_mutex`. The purpose of
`ipc_server_mainloop::client_push_mutex` is to allow only one client into the
client-acceptance handshake at a time, so that no acknowledgement of client
accept is lost. Once those two mutexes are locked,
`ipc_server_mainloop_add_fd()` writes the FD number to the pipe. Then, it waits
on the condition variable (releasing `accept_mutex`) to see either that FD
number or the special "shutting down" sentinel value in the `last_accepted_fd`
variable. If it sees the FD number, that indicates that the other side of the
communication (the mainloop) has taken ownership of the FD and will handle
closing it. If it sees the sentinel value, or has an error at some point, it
assumes that ownership is retained and it should close the FD itself.

The other side of the communication works as follows: epoll is used to check if
there is new data waiting on the pipe. If so, the
`ipc_server_mainloop::accept_mutex` lock is taken, and an FD number is read from
the pipe. A client thread is launched for that FD, then the `last_accepted_fd`
variable is updated and the `ipc_server_mainloop::accept_cond` condition
variable signalled.

The initial plan required that the server also wait on
`ipc_server_mainloop::accept_cond` for the `last_accepted_fd` to be reset back
to `0` by the acknowledged client, thus preventing losing acknowledgements.
However, it is undesirable for the clients to be able to block the
compositor/server, so this wait was considered not acceptable. Instead, the
`ipc_server_mainloop::client_push_mutex` is used so that at most one
un-acknowledged client may have written to the pipe at any given time.

## A Note on Graphics IPC

The IPC mechanisms described previously are used solely for small data. Graphics
data communication between application/client and server is done through sharing
of buffers and synchronization primitives, without any copying or serialization
of buffers within a frame loop.

We use the system and graphics API provided mechanisms of sharing graphics
buffers and sync primitives, which all result in some cross-API-usable handle
type (generically processed as the types @ref xrt_graphics_buffer_handle_t and
@ref xrt_graphics_sync_handle_t). On all supported platforms, there exist ways
to share these handle types both within and between processes:

- Linux and Android can send these handles, uniformly represented as file
  descriptors, through a domain socket with a [SCM_RIGHTS][] message.
- It is anticipated that Windows will use DuplicateHandle and send handle
  numbers to achieve an equivalent result. ([reference][win32handles]) While
  recent versions of Windows have added `AF_UNIX` domain socket support,
  [`SCM_RIGHTS` is not supported][WinSCM_RIGHTS].

The @ref xrt_compositor_native and @ref xrt_swapchain_native interfaces conceal
the compositor's own graphics API choice, interacting with a client compositor
solely through these generic handles. As such, even in single-process mode,
buffers and sync primitives are generally exported to handles and imported back
into another graphics API. (There is a small exception to this general statement
to allow in-process execution on a software Vulkan implementation for CI
purposes.)

Generally, when possible, we allocate buffers on the server side in Vulkan, and
import into the client compositor and API. On Android, to support application
quotas and limits on allocation, etc, the client side allocates the buffer using
a @ref xrt_image_native_allocator (aka XINA) and shares it to the server. When
using D3D11 or D3D12 on Windows, buffers are allocated by the client compositor
and imported into the native compositor, because Vulkan can import buffers from
D3D, but D3D cannot import buffers allocated by Vulkan. See @ref swapchains-ipc
for details.

[SCM_RIGHTS]: https://man7.org/linux/man-pages/man3/cmsg.3.html
[win32handles]: https://lackingrhoticity.blogspot.com/2015/05/passing-fds-handles-between-processes.html
[WinSCM_RIGHTS]: https://devblogs.microsoft.com/commandline/af_unix-comes-to-windows/#unsupportedunavailable

## 32 bits client support on 64 bits server

On 64 bits system, the server process will typically be a 64 bits executable,
clients may be 32 or 64 bits. As IPC either through socket or shared memory will
share C struct directly, we must pay attention to memory layout.

All data types must be fixed size and alignments must be based on the 64 bits
targets. On Linux this mostly means `alignas(8)` must be used for (u)int64_t.
