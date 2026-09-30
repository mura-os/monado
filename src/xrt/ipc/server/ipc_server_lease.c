// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The controller lease: which client may drive the service.
 * @ingroup ipc_server
 */

#include "server/ipc_server_lease.h"

#include <assert.h>
#include <string.h>


static void
remove_pending_at(struct ipc_controller_lease *l, uint32_t pos)
{
	assert(pos < l->pending_count);
	for (uint32_t i = pos + 1; i < l->pending_count; i++) {
		l->pending[i - 1] = l->pending[i];
	}
	l->pending_count--;
	l->pending[l->pending_count] = -1;
}

void
ipc_controller_lease_init(struct ipc_controller_lease *l)
{
	l->holder_index = -1;
	l->pending_count = 0;
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		l->pending[i] = -1;
	}
}

bool
ipc_controller_lease_on_connect(struct ipc_controller_lease *l, int index, enum ipc_client_role role)
{
	if (role != IPC_CLIENT_ROLE_CONTROLLER || index < 0) {
		return false;
	}

	if (l->holder_index < 0) {
		l->holder_index = index;
		return true;
	}

	if (l->holder_index == index || ipc_controller_lease_is_pending(l, index)) {
		// Already accounted for; a slot cannot connect twice without
		// disconnecting in between, so this is a caller bug, but be
		// forgiving.
		return false;
	}

	assert(l->pending_count < IPC_MAX_CLIENTS);
	l->pending[l->pending_count++] = index;
	return false;
}

bool
ipc_controller_lease_on_disconnect(struct ipc_controller_lease *l, int index, int *out_promoted_index)
{
	if (out_promoted_index != NULL) {
		*out_promoted_index = -1;
	}
	if (index < 0) {
		return false;
	}

	if (l->holder_index == index) {
		if (l->pending_count > 0) {
			l->holder_index = l->pending[0];
			remove_pending_at(l, 0);
		} else {
			l->holder_index = -1;
		}
		if (out_promoted_index != NULL) {
			*out_promoted_index = l->holder_index;
		}
		return true;
	}

	for (uint32_t i = 0; i < l->pending_count; i++) {
		if (l->pending[i] == index) {
			remove_pending_at(l, i);
			break;
		}
	}
	return false;
}

bool
ipc_controller_lease_is_pending(const struct ipc_controller_lease *l, int index)
{
	for (uint32_t i = 0; i < l->pending_count; i++) {
		if (l->pending[i] == index) {
			return true;
		}
	}
	return false;
}
