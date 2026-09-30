// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Peer inspection for accepted IPC sockets: is the client sandboxed?
 * @ingroup ipc_server
 *
 * Mirrors how the desktop stack recognises a sandboxed peer today: PipeWire
 * reads `/proc/<pid>/root/.flatpak-info` for the connecting pid
 * (module-access.c), and systemd names Flatpak and Snap scopes
 * `app-flatpak-<id>-<pid>.scope` / `snap.<name>.<app>-<uuid>.scope`.
 */

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_os.h"

#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_truncate_printf.h"

#include "server/ipc_server_peer.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef XRT_HAVE_SYSTEMD
#include <systemd/sd-login.h>
#endif

#ifndef SO_PEERPIDFD
#define SO_PEERPIDFD 77
#endif


/*
 *
 * Helpers.
 *
 */

static void
copy_str(char *dst, size_t dst_size, const char *src)
{
	u_truncate_snprintf(dst, (int)dst_size, "%s", src);
}

/*!
 * Get the peer pid and, when the kernel supports it, a pidfd for it.
 *
 * @return false if neither is available.
 */
static bool
get_peer_pid(int fd, pid_t *out_pid, int *out_pidfd)
{
	*out_pid = 0;
	*out_pidfd = -1;

	int pidfd = -1;
	socklen_t len = sizeof(pidfd);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERPIDFD, &pidfd, &len) == 0 && pidfd >= 0) {
		*out_pidfd = pidfd;
	}

	struct ucred cred = XRT_STRUCT_INIT;
	len = sizeof(cred);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &len) == 0 && cred.pid > 0) {
		*out_pid = cred.pid;
	}

	return *out_pid > 0 || *out_pidfd >= 0;
}

/*!
 * Read `/proc/<pid>/root/.flatpak-info`, filling @p info from the
 * `[Application] name=` and `[Instance] instance-id=` keys.
 *
 * Like PipeWire's module-access.c we only trust the file's existence for the
 * classification; the keys are informational.
 */
static bool
read_flatpak_info(pid_t pid, struct ipc_client_sandbox_info *info)
{
	char path[64];
	u_truncate_snprintf(path, (int)sizeof(path), "/proc/%d/root/.flatpak-info", (int)pid);

	// Opening rather than stat'ing: the file must be readable by us to
	// count, and a plain stat on a foreign root can be spoofed less easily
	// than it can be denied.
	FILE *f = fopen(path, "re");
	if (f == NULL) {
		return false;
	}

	U_ZERO(info);
	copy_str(info->engine, sizeof(info->engine), "org.flatpak");

	char line[512];
	enum
	{
		SEC_NONE,
		SEC_APPLICATION,
		SEC_INSTANCE
	} section = SEC_NONE;
	while (fgets(line, sizeof(line), f) != NULL) {
		size_t n = strlen(line);
		while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r')) {
			line[--n] = '\0';
		}
		if (n == 0) {
			continue;
		}
		if (line[0] == '[') {
			if (strcmp(line, "[Application]") == 0) {
				section = SEC_APPLICATION;
			} else if (strcmp(line, "[Instance]") == 0) {
				section = SEC_INSTANCE;
			} else {
				section = SEC_NONE;
			}
			continue;
		}
		char *eq = strchr(line, '=');
		if (eq == NULL) {
			continue;
		}
		*eq = '\0';
		const char *key = line;
		const char *value = eq + 1;

		if (section == SEC_APPLICATION && strcmp(key, "name") == 0) {
			copy_str(info->app_id, sizeof(info->app_id), value);
		} else if (section == SEC_INSTANCE && strcmp(key, "instance-id") == 0) {
			copy_str(info->instance_id, sizeof(info->instance_id), value);
		}
	}

	fclose(f);
	return true;
}

#ifdef XRT_HAVE_SYSTEMD
/*!
 * Recognise a sandbox by the user unit systemd placed the process in.
 *
 * `app-flatpak-<app id>-<pid>.scope` is what Flatpak asks systemd for;
 * `snap.<snap>.<app>-<uuid>.scope` is snapd's. Neither is trusted for
 * anything beyond "this is a sandbox".
 */
static bool
unit_marks_sandbox(pid_t pid, int pidfd, struct ipc_client_sandbox_info *info)
{
	char *unit = NULL;
	int ret = -ENOSYS;

	if (pidfd >= 0) {
		ret = sd_pidfd_get_user_unit(pidfd, &unit);
	}
	if (ret < 0 && pid > 0) {
		ret = sd_pid_get_user_unit(pid, &unit);
	}
	if (ret < 0 || unit == NULL) {
		return false;
	}

	bool sandboxed = false;
	if (strncmp(unit, "app-flatpak-", strlen("app-flatpak-")) == 0) {
		U_ZERO(info);
		copy_str(info->engine, sizeof(info->engine), "org.flatpak");
		// app-flatpak-<escaped id>-<pid>.scope: keep the id part.
		const char *id = unit + strlen("app-flatpak-");
		const char *dash = strrchr(id, '-');
		size_t n = dash != NULL ? (size_t)(dash - id) : strlen(id);
		if (n >= sizeof(info->app_id)) {
			n = sizeof(info->app_id) - 1;
		}
		memcpy(info->app_id, id, n);
		info->app_id[n] = '\0';
		sandboxed = true;
	} else if (strncmp(unit, "snap.", strlen("snap.")) == 0) {
		U_ZERO(info);
		copy_str(info->engine, sizeof(info->engine), "io.snapcraft");
		// snap.<snap>.<app>-<uuid>.scope: keep snap.app.
		const char *id = unit + strlen("snap.");
		const char *dash = strchr(id, '-');
		size_t n = dash != NULL ? (size_t)(dash - id) : strlen(id);
		if (n >= sizeof(info->app_id)) {
			n = sizeof(info->app_id) - 1;
		}
		memcpy(info->app_id, id, n);
		info->app_id[n] = '\0';
		sandboxed = true;
	}

	free(unit);
	return sandboxed;
}
#endif


/*
 *
 * 'Exported' functions.
 *
 */

bool
ipc_server_peer_is_sandboxed(int fd, struct ipc_client_sandbox_info *info)
{
	pid_t pid = 0;
	int pidfd = -1;
	bool sandboxed = false;

	if (!get_peer_pid(fd, &pid, &pidfd)) {
		return false;
	}

	if (pid > 0 && read_flatpak_info(pid, info)) {
		sandboxed = true;
	}

#ifdef XRT_HAVE_SYSTEMD
	if (!sandboxed && unit_marks_sandbox(pid, pidfd, info)) {
		sandboxed = true;
	}
#endif

	if (pidfd >= 0) {
		close(pidfd);
	}

	if (sandboxed) {
		U_LOG_D("Peer pid %d is sandboxed (engine='%s', app_id='%s')", (int)pid, info->engine, info->app_id);
	}

	return sandboxed;
}
