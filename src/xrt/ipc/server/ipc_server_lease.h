// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller lease: which client may drive the service.
 * @ingroup ipc_server
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "shared/ipc_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * One lease over the service's control verbs, held by at most one
 * @ref IPC_CLIENT_ROLE_CONTROLLER client at a time.
 *
 * Controllers connecting while the lease is held queue in arrival order and
 * are promoted when the holder disconnects; a holder is never displaced.
 * Nothing persists across the holder's disconnect, so a restarted controller
 * finds the service in the state its verbs left it and re-asserts what it
 * wants. This is the seat model of seatd / logind, not a permission store.
 *
 * Pure state machine: no locking, no I/O. The server drives it holding
 * `ipc_server::global_state.lock`.
 */
struct ipc_controller_lease
{
	//! Server thread index of the holder, -1 when nobody holds it.
	int holder_index;

	//! Controllers waiting, in arrival order.
	int pending[IPC_MAX_CLIENTS];
	uint32_t pending_count;
};

/*!
 * Reset to "no holder, nobody pending".
 * @public @memberof ipc_controller_lease
 */
void
ipc_controller_lease_init(struct ipc_controller_lease *l);

/*!
 * A client with @p role connected at @p index. Controllers take the lease if
 * free, otherwise queue; every other role is ignored.
 *
 * @return true if @p index became the holder.
 * @public @memberof ipc_controller_lease
 */
bool
ipc_controller_lease_on_connect(struct ipc_controller_lease *l, int index, enum ipc_client_role role);

/*!
 * The client at @p index disconnected. If it held the lease, the oldest
 * pending controller (if any) is promoted; if it was pending, it is removed.
 *
 * @param[out] out_promoted_index Index promoted to holder, or -1.
 * @return true if the holder changed.
 * @public @memberof ipc_controller_lease
 */
bool
ipc_controller_lease_on_disconnect(struct ipc_controller_lease *l, int index, int *out_promoted_index);

/*!
 * Does @p index hold the lease?
 * @public @memberof ipc_controller_lease
 */
static inline bool
ipc_controller_lease_is_holder(const struct ipc_controller_lease *l, int index)
{
	return index >= 0 && l->holder_index == index;
}

/*!
 * Is @p index in the pending queue?
 * @public @memberof ipc_controller_lease
 */
bool
ipc_controller_lease_is_pending(const struct ipc_controller_lease *l, int index);

#ifdef __cplusplus
}
#endif
