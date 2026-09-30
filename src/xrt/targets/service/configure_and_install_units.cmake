# Copyright 2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

set(XRT_INSTALL_ABSOLUTE_SYSTEMD_UNIT_FILES @XRT_INSTALL_ABSOLUTE_SYSTEMD_UNIT_FILES@)

# Set up variables to use when configuring files
set(conflicts @conflicts@)
set(exit_on_disconnect @exit_on_disconnect@)
set(service_path "monado-service")
set(UNIT_NAME @UNIT_NAME@)
set(XRT_IPC_MSG_SOCK_FILENAME @XRT_IPC_MSG_SOCK_FILENAME@)
set(XRT_IPC_MSG_SOCK_CONTROL_FILENAME @XRT_IPC_MSG_SOCK_CONTROL_FILENAME@)
if(XRT_INSTALL_ABSOLUTE_SYSTEMD_UNIT_FILES)
	set(service_path "${CMAKE_INSTALL_PREFIX}/@CMAKE_INSTALL_BINDIR@/${service_path}")
endif()

# Create unit files
configure_file("@SOCKET_INPUT@" "@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@.socket")
configure_file("@CONTROL_SOCKET_INPUT@" "@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@-control.socket" @ONLY)
configure_file("@SERVICE_INPUT@" "@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@.service")

# Install them
file(
	INSTALL
	DESTINATION "@UNIT_DIR@"
	TYPE
	FILE
	FILES
	"@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@.socket"
	"@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@-control.socket"
	"@CMAKE_CURRENT_BINARY_DIR@/@UNIT_NAME@.service"
	)
