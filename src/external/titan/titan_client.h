#ifndef TITAN_CLIENT_H
#define TITAN_CLIENT_H

/*
 * Header-only transport helpers for the titan-server protocol.
 *
 * This layer owns no client state, buffers, queues, mappings, or event loop.
 * titan_client_connect() returns a pollable socket; the caller owns that
 * socket, all successful receive storage, and every received dma-buf fd.
 * Pass MSG_DONTWAIT to send/receive from a nonblocking event loop.
 *
 * titan_client_recv_prepare() + titan_client_recv_finish() expose the same
 * receive path around a caller-owned msghdr, so a regular io_uring RECVMSG
 * operation can use the validation and SCM_RIGHTS handling without calling
 * recvmsg() here.  The iovec, msghdr, control buffer, and packet storage must
 * remain alive until that operation completes.
 *
 * Pointer arguments are required unless their optional form is documented
 * (the connect path and capacity-dependent fd storage are the main
 * exceptions). Required NULL pointers are caller bugs and are asserted.
 */

#include "titan_proto.h"

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/un.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) && !defined(__clang__)
/*
 * GCC 15 on AArch64 can incorrectly specialize these two nested inline
 * helpers when one caller alternates between receive calls with and without
 * fd storage.  Disable the affected IPA transforms for these helpers only.
 */
#define TITAN_CLIENT_NO_IPA __attribute__((__optimize__("no-ipa-cp", "no-ipa-sra"), __noclone__))
#else
#define TITAN_CLIENT_NO_IPA
#endif

/* Correctly aligned ancillary storage for all fds allowed by STREAM_INFO. */
typedef union titan_client_recv_control {
    struct cmsghdr alignment;
    unsigned char bytes[CMSG_SPACE(sizeof(int) * TITAN_PROTO_MAX_FDS)];
} titan_client_recv_control;

static inline int titan_client_detail_set_cloexec(int fd)
{
    int flags = fcntl(fd, F_GETFD);

    if (flags < 0)
        return -1;
    if (flags & FD_CLOEXEC)
        return 0;
    return fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

static inline int titan_client_detail_connect_path(const char *path)
{
    struct sockaddr_un address = { 0 };
    size_t length;
    int fd;

    assert(path);
    length = strlen(path);
    if (length >= sizeof(address.sun_path)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    address.sun_family = AF_UNIX;
    memcpy(address.sun_path, path, length + 1);

    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd < 0)
        return -1;
    if (titan_client_detail_set_cloexec(fd) != 0 ||
        connect(fd, (const struct sockaddr *)&address, sizeof(address)) != 0) {
        int saved_errno = errno;

        close(fd);
        errno = saved_errno;
        return -1;
    }
    return fd;
}

/*
 * Connect only; no protocol request is performed.  path == NULL probes
 * $TITAN_SERVER_SOCKET, then the system socket, then the session socket.
 */
static inline int titan_client_connect(const char *path)
{
    const char *runtime_dir;
    int fd;
    int system_errno;
    char session_path[sizeof(((struct sockaddr_un *)0)->sun_path)];
    static const char session_suffix[] = "/titan-server.sock";
    size_t runtime_length;

    if (path)
        return titan_client_detail_connect_path(path);
    path = getenv(TITAN_PROTO_SOCKET_ENV);
    if (path)
        return titan_client_detail_connect_path(path);

    fd = titan_client_detail_connect_path(TITAN_PROTO_SOCKET);
    if (fd >= 0)
        return fd;
    system_errno = errno;
    runtime_dir = getenv("XDG_RUNTIME_DIR");
    if (!runtime_dir) {
        errno = system_errno;
        return -1;
    }
    runtime_length = strlen(runtime_dir);
    if (runtime_length > sizeof(session_path) - sizeof(session_suffix)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    memcpy(session_path, runtime_dir, runtime_length);
    memcpy(session_path + runtime_length, session_suffix, sizeof(session_suffix));
    return titan_client_detail_connect_path(session_path);
}

/*
 * Closing the socket tells the server to release this client's holds.  The
 * caller must first stop using every imported/mapped buffer and close its
 * received dma-buf fds.
 */
static inline void titan_client_disconnect(int fd)
{
    if (fd >= 0)
        (void)close(fd);
}

/*
 * Send exactly one client-to-server seqpacket.  flags may include
 * MSG_DONTWAIT.  The packet's type, version, direction, and exact size are
 * validated before it reaches the socket.
 */
static inline int titan_client_send(int fd, const void *message, size_t size, int flags)
{
    ssize_t sent;

    assert(message);
    if (fd < 0 || titan_proto_validate_message(message, size, TITAN_PROTO_CLIENT_TO_SERVER) !=
                      TITAN_PROTO_VALID) {
        errno = EINVAL;
        return -1;
    }
    do {
        sent = send(fd, message, size, flags | MSG_NOSIGNAL);
    } while (sent < 0 && errno == EINTR);
    if (sent < 0)
        return -1;
    if ((size_t)sent != size) {
        errno = EIO;
        return -1;
    }
    return 0;
}

/* Typed convenience sends for every client-to-server protocol message. */
static inline int titan_client_send_list(int fd, int flags)
{
    struct titan_msg_hdr message = titan_proto_make_list();

    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_open(int fd, const struct titan_wire_config *config, int flags)
{
    struct titan_msg_open message;

    assert(config);
    message = titan_proto_make_open(*config);
    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_release(int fd,
                                            const struct titan_msg_stream_info *stream,
                                            const struct titan_msg_frame *frame,
                                            int flags)
{
    struct titan_msg_frame_ref message;

    if (titan_proto_make_release(&message, stream, frame) != TITAN_PROTO_VALID) {
        errno = EINVAL;
        return -1;
    }
    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_frame_used(int fd,
                                               const struct titan_msg_stream_info *stream,
                                               const struct titan_msg_frame *frame,
                                               uint64_t use_sequence,
                                               uint64_t used_boottime_ns,
                                               int flags)
{
    struct titan_msg_frame_used message;

    if (titan_proto_frame_used_from_frame(
            &message, stream, frame, use_sequence, used_boottime_ns) != TITAN_PROTO_VALID) {
        errno = EINVAL;
        return -1;
    }
    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_close(int fd, uint32_t stream_id, int flags)
{
    struct titan_msg_close message = titan_proto_make_close(stream_id);

    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_set_exposure(
    int fd, uint32_t stream_id, uint32_t exp_lines, uint32_t gain, int flags)
{
    struct titan_msg_set_exposure message =
        titan_proto_make_set_exposure(stream_id, exp_lines, gain);

    if (titan_proto_validate_set_exposure(&message) != TITAN_PROTO_VALID) {
        errno = EINVAL;
        return -1;
    }
    return titan_client_send(fd, &message, sizeof(message), flags);
}

static inline int titan_client_send_list_modes(int fd, int32_t cam_slot, int flags)
{
    struct titan_msg_list_modes message = titan_proto_make_list_modes(cam_slot);

    return titan_client_send(fd, &message, sizeof(message), flags);
}

/*
 * Initialize caller-owned recvmsg state.  This function performs no I/O.
 * Every object passed here must remain alive until recvmsg/io_uring and
 * titan_client_recv_finish() have both completed.
 */
static inline int titan_client_recv_prepare(void *packet,
                                            size_t packet_capacity,
                                            struct iovec *iov,
                                            struct msghdr *message,
                                            titan_client_recv_control *control)
{
    assert(packet);
    assert(iov);
    assert(message);
    assert(control);
    if (packet_capacity < sizeof(struct titan_msg_hdr)) {
        errno = EINVAL;
        return -1;
    }
    *iov = (struct iovec){ 0 };
    *message = (struct msghdr){ 0 };
    memset(control, 0, sizeof(*control));
    iov->iov_base = packet;
    iov->iov_len = packet_capacity;
    message->msg_iov = iov;
    message->msg_iovlen = 1;
    message->msg_control = control->bytes;
    message->msg_controllen = sizeof(control->bytes);
    return 0;
}

/*
 * Complete a recvmsg result. packet and message are required for a positive
 * result; io_uring users may pass NULL for both with a nonpositive CQE result,
 * which is converted to the usual return/errno convention.
 *
 * Success transfers fd_count descriptors to fds, all with FD_CLOEXEC set.
 * On every validation, truncation, or capacity error, every descriptor
 * visible in the control data is closed and *fd_count is zero.
 */
static inline TITAN_CLIENT_NO_IPA ssize_t titan_client_recv_finish(void *packet,
                                                                   size_t packet_capacity,
                                                                   struct msghdr *message,
                                                                   ssize_t received,
                                                                   int *fds,
                                                                   size_t fd_capacity,
                                                                   size_t *fd_count)
{
    int argument_error;
    int protocol_error = 0;
    int system_error = 0;
    size_t stored_fds = 0;
    struct titan_msg_hdr packet_header = { 0 };
    struct cmsghdr *header;

    if (fd_count)
        *fd_count = 0;
    if (received < 0) {
        errno = received >= -4095 ? (int)-received : EIO;
        return -1;
    }
    if (received == 0)
        return 0;
    assert(packet);
    assert(message);

    argument_error = packet_capacity < sizeof(packet_header) || (!fds && fd_capacity != 0) ||
                     (fd_count == NULL && (fds != NULL || fd_capacity != 0));
    if (message->msg_controllen && !message->msg_control)
        protocol_error = 1;

    for (header = CMSG_FIRSTHDR(message); header; header = CMSG_NXTHDR(message, header)) {
        const unsigned char *control_begin = (const unsigned char *)message->msg_control;
        const unsigned char *control_end;
        const unsigned char *header_begin = (const unsigned char *)header;
        size_t payload_bytes;
        size_t count;
        size_t i;

        if (!control_begin) {
            protocol_error = 1;
            break;
        }
        control_end = control_begin + message->msg_controllen;
        if (header_begin < control_begin || header_begin > control_end ||
            (size_t)(control_end - header_begin) < sizeof(struct cmsghdr) ||
            header->cmsg_len < CMSG_LEN(0) ||
            header->cmsg_len > (size_t)(control_end - header_begin)) {
            protocol_error = 1;
            break;
        }
        if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS) {
            protocol_error = 1;
            continue;
        }
        payload_bytes = header->cmsg_len - CMSG_LEN(0);
        if (payload_bytes % sizeof(int) != 0)
            protocol_error = 1;
        count = payload_bytes / sizeof(int);
        for (i = 0; i < count; i++) {
            int received_fd;

            memcpy(&received_fd, CMSG_DATA(header) + i * sizeof(received_fd), sizeof(received_fd));
            if (titan_client_detail_set_cloexec(received_fd) != 0) {
                if (!system_error)
                    system_error = errno;
                close(received_fd);
                continue;
            }
            if (argument_error || !fds || stored_fds >= fd_capacity) {
                close(received_fd);
                protocol_error = 1;
                continue;
            }
            fds[stored_fds++] = received_fd;
        }
    }

    if (message->msg_flags & (MSG_TRUNC | MSG_CTRUNC))
        protocol_error = 1;
    if ((size_t)received > packet_capacity)
        protocol_error = 1;
    if (!argument_error &&
        titan_proto_validate_message(packet, (size_t)received, TITAN_PROTO_SERVER_TO_CLIENT) !=
            TITAN_PROTO_VALID)
        protocol_error = 1;

    if (!argument_error && (size_t)received >= sizeof(packet_header))
        memcpy(&packet_header, packet, sizeof(packet_header));
    if (!argument_error && !protocol_error) {
        switch (packet_header.type) {
        case TITAN_MSG_STREAM_INFO: {
            struct titan_msg_stream_info info;

            if (packet_capacity < sizeof(info)) {
                protocol_error = 1;
                break;
            }
            memcpy(&info, packet, sizeof(info));
            if (titan_proto_validate_stream_info(&info, stored_fds) != TITAN_PROTO_VALID)
                protocol_error = 1;
            break;
        }
        case TITAN_MSG_LIST_REPLY: {
            struct titan_msg_list_reply reply;

            if (packet_capacity < sizeof(reply)) {
                protocol_error = 1;
                break;
            }
            memcpy(&reply, packet, sizeof(reply));
            if (titan_proto_validate_list_reply(&reply) != TITAN_PROTO_VALID)
                protocol_error = 1;
            break;
        }
        case TITAN_MSG_LIST_MODES_REPLY: {
            struct titan_msg_list_modes_reply reply;

            if (packet_capacity < sizeof(reply)) {
                protocol_error = 1;
                break;
            }
            memcpy(&reply, packet, sizeof(reply));
            if (titan_proto_validate_list_modes_reply(&reply) != TITAN_PROTO_VALID)
                protocol_error = 1;
            break;
        }
        default:
            if (stored_fds != 0)
                protocol_error = 1;
            break;
        }
    }

    if (argument_error || protocol_error || system_error) {
        size_t i;

        for (i = 0; i < stored_fds; i++)
            close(fds[i]);
        if (argument_error)
            errno = EINVAL;
        else if (system_error)
            errno = system_error;
        else
            errno = EPROTO;
        return -1;
    }
    if (fd_count)
        *fd_count = stored_fds;
    return received;
}

/*
 * Receive exactly one server-to-client seqpacket into caller-owned storage.
 * flags may include MSG_DONTWAIT.  Zero means orderly socket shutdown.
 */
static inline TITAN_CLIENT_NO_IPA ssize_t titan_client_recv(int fd,
                                                            void *packet,
                                                            size_t packet_capacity,
                                                            int *fds,
                                                            size_t fd_capacity,
                                                            size_t *fd_count,
                                                            int flags)
{
    titan_client_recv_control control;
    struct iovec iov;
    struct msghdr message;
    ssize_t received;

    assert(packet);
    if (fd_count)
        *fd_count = 0;
    if (fd < 0 || (!fds && fd_capacity != 0) ||
        (fd_count == NULL && (fds != NULL || fd_capacity != 0))) {
        errno = EINVAL;
        return -1;
    }
    if (titan_client_recv_prepare(packet, packet_capacity, &iov, &message, &control) != 0)
        return -1;
    do {
        received = recvmsg(fd, &message, flags | MSG_CMSG_CLOEXEC);
    } while (received < 0 && errno == EINTR);
    if (received <= 0)
        return received;
    return titan_client_recv_finish(
        packet, packet_capacity, &message, received, fds, fd_capacity, fd_count);
}

#undef TITAN_CLIENT_NO_IPA

#ifdef __cplusplus
}
#endif

#endif /* TITAN_CLIENT_H */
