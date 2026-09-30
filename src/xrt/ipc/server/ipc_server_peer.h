// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Peer inspection for accepted IPC sockets: is the client sandboxed?
 * @ingroup ipc_server
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "shared/ipc_protocol.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Decide whether the peer on @p fd is running inside an application sandbox.
 *
 * Checks, in order: a Flatpak instance (`/proc/<pid>/root/.flatpak-info`
 * exists), then when built with systemd, a user unit whose name marks a
 * sandbox (`app-flatpak-*` or `snap.*`). The pid is taken from the socket
 * with `SO_PEERPIDFD` where the kernel offers it, else `SO_PEERCRED`; a
 * pidfd keeps the lookup pinned to the process that connected.
 *
 * This lookup only ever *lowers* a client: an accept on the ordinary socket
 * that turns out to be sandboxed becomes @ref IPC_CLIENT_ROLE_SANDBOXED_APP.
 * Nothing found here can raise a client to controller.
 *
 * @param fd        Connected socket, just accepted.
 * @param[out] info Filled with engine/app_id/instance_id when sandboxed.
 * @return true if the peer is sandboxed and @p info is valid.
 */
bool
ipc_server_peer_is_sandboxed(int fd, struct ipc_client_sandbox_info *info);

#ifdef __cplusplus
}
#endif
