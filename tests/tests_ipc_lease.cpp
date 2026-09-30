// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Controller lease state machine tests.
 */

#include "server/ipc_server_lease.h"

#include "catch_amalgamated.hpp"


TEST_CASE("ipc_controller_lease")
{
	struct ipc_controller_lease l;
	ipc_controller_lease_init(&l);

	SECTION("starts empty")
	{
		CHECK(l.holder_index == -1);
		CHECK(l.pending_count == 0);
		CHECK_FALSE(ipc_controller_lease_is_holder(&l, 0));
		CHECK_FALSE(ipc_controller_lease_is_pending(&l, 0));
	}

	SECTION("non-controllers never touch the lease")
	{
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 0, IPC_CLIENT_ROLE_APP));
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 1, IPC_CLIENT_ROLE_SANDBOXED_APP));
		CHECK(l.holder_index == -1);
		CHECK(l.pending_count == 0);

		int promoted = 99;
		CHECK_FALSE(ipc_controller_lease_on_disconnect(&l, 0, &promoted));
		CHECK(promoted == -1);
	}

	SECTION("first controller becomes holder")
	{
		CHECK(ipc_controller_lease_on_connect(&l, 3, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(ipc_controller_lease_is_holder(&l, 3));
		CHECK_FALSE(ipc_controller_lease_is_pending(&l, 3));
		CHECK(l.pending_count == 0);
	}

	SECTION("second controller is pending, promoted on holder disconnect")
	{
		REQUIRE(ipc_controller_lease_on_connect(&l, 3, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 5, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(ipc_controller_lease_is_holder(&l, 3));
		CHECK(ipc_controller_lease_is_pending(&l, 5));
		CHECK(l.pending_count == 1);

		// An app disconnecting in between changes nothing.
		int promoted = -1;
		CHECK_FALSE(ipc_controller_lease_on_disconnect(&l, 7, &promoted));
		CHECK(ipc_controller_lease_is_holder(&l, 3));

		// A holder is never displaced by a newcomer.
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 9, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(ipc_controller_lease_is_holder(&l, 3));
		CHECK(l.pending_count == 2);

		// Holder leaves: oldest pending is promoted, in arrival order.
		CHECK(ipc_controller_lease_on_disconnect(&l, 3, &promoted));
		CHECK(promoted == 5);
		CHECK(ipc_controller_lease_is_holder(&l, 5));
		CHECK_FALSE(ipc_controller_lease_is_pending(&l, 5));
		CHECK(ipc_controller_lease_is_pending(&l, 9));
		CHECK(l.pending_count == 1);

		CHECK(ipc_controller_lease_on_disconnect(&l, 5, &promoted));
		CHECK(promoted == 9);
		CHECK(ipc_controller_lease_is_holder(&l, 9));
		CHECK(l.pending_count == 0);
	}

	SECTION("pending controller leaving is removed without a promotion")
	{
		REQUIRE(ipc_controller_lease_on_connect(&l, 1, IPC_CLIENT_ROLE_CONTROLLER));
		REQUIRE_FALSE(ipc_controller_lease_on_connect(&l, 2, IPC_CLIENT_ROLE_CONTROLLER));
		REQUIRE_FALSE(ipc_controller_lease_on_connect(&l, 4, IPC_CLIENT_ROLE_CONTROLLER));

		int promoted = 99;
		CHECK_FALSE(ipc_controller_lease_on_disconnect(&l, 2, &promoted));
		CHECK(promoted == -1);
		CHECK(ipc_controller_lease_is_holder(&l, 1));
		CHECK_FALSE(ipc_controller_lease_is_pending(&l, 2));
		CHECK(ipc_controller_lease_is_pending(&l, 4));
		CHECK(l.pending_count == 1);

		// The queue keeps its order after the removal.
		CHECK(ipc_controller_lease_on_disconnect(&l, 1, &promoted));
		CHECK(promoted == 4);
	}

	SECTION("holder leaving with nobody pending frees the lease")
	{
		REQUIRE(ipc_controller_lease_on_connect(&l, 6, IPC_CLIENT_ROLE_CONTROLLER));
		int promoted = 99;
		CHECK(ipc_controller_lease_on_disconnect(&l, 6, &promoted));
		CHECK(promoted == -1);
		CHECK(l.holder_index == -1);

		// The next controller to arrive takes it fresh: nothing persisted.
		CHECK(ipc_controller_lease_on_connect(&l, 8, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(ipc_controller_lease_is_holder(&l, 8));
	}

	SECTION("a reconnecting index is not queued twice")
	{
		REQUIRE(ipc_controller_lease_on_connect(&l, 1, IPC_CLIENT_ROLE_CONTROLLER));
		REQUIRE_FALSE(ipc_controller_lease_on_connect(&l, 2, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 2, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(l.pending_count == 1);
		CHECK_FALSE(ipc_controller_lease_on_connect(&l, 1, IPC_CLIENT_ROLE_CONTROLLER));
		CHECK(l.pending_count == 1);
	}

	SECTION("queue fills to capacity")
	{
		REQUIRE(ipc_controller_lease_on_connect(&l, 0, IPC_CLIENT_ROLE_CONTROLLER));
		for (int i = 1; i < IPC_MAX_CLIENTS; i++) {
			CHECK_FALSE(ipc_controller_lease_on_connect(&l, i, IPC_CLIENT_ROLE_CONTROLLER));
		}
		CHECK(l.pending_count == (uint32_t)(IPC_MAX_CLIENTS - 1));

		int promoted = -1;
		for (int i = 0; i < IPC_MAX_CLIENTS - 1; i++) {
			CHECK(ipc_controller_lease_on_disconnect(&l, i, &promoted));
			CHECK(promoted == i + 1);
		}
		CHECK(ipc_controller_lease_on_disconnect(&l, IPC_MAX_CLIENTS - 1, &promoted));
		CHECK(promoted == -1);
		CHECK(l.holder_index == -1);
		CHECK(l.pending_count == 0);
	}
}
