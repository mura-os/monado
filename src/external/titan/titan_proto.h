#ifndef TITAN_PROTO_H
#define TITAN_PROTO_H

/*
 * titan-server wire protocol, version 12.
 *
 * Transport: SOCK_SEQPACKET AF_UNIX socket — the kernel preserves message
 * boundaries, so every recv() yields exactly one whole message and no
 * framing/length parsing exists.  Every message is one of the fixed
 * little-endian structs below; the first two u32 are always {type, version}.
 * The ring dma-buf fds cross the socket exactly once, attached to
 * TITAN_MSG_STREAM_INFO via SCM_RIGHTS (fd i = ring slot i); everything at
 * frame rate is a plain small struct.
 *
 * Lifecycle:
 *   connect
 *   LIST -> LIST_REPLY
 *   OPEN{cfg} -> STREAM_INFO{geometry}+fds     (or ERROR)
 *   loop:  FRAME{view records} ->  client uses mapped/imported slots
 *          FRAME_USED{delivery id, use clock} <- controller timing feedback
 *          RELEASE{references} <-  client done with them
 *   CLOSE{stream} either way ends the stream; server also sends CLOSE on
 *   teardown.  Disconnect implies release of everything the client held.
 *
 * Roles: the first client to open a camera is its controller
 * (SET_EXPOSURE and FRAME_USED allowed, its OPEN cfg programmed the stream);
 * later opens of the same camera attach as viewers of the existing stream
 * and get the same ring. A slot returns to the ISP only when every subscriber
 * released it.
 *
 * Views: every stream delivers num_views buffers per frame event — 1 for a
 * physical camera, 2 for a virtual pair, 4 for a quad, or 6 for the complete
 * tracking rig (negative cam_slot ids below).  A group event is withheld
 * until every view of the frame completed (all-or-nothing).  STREAM_INFO
 * carries num_views * num_slots ring fds,
 * view-major (view v's slot s = fd index v * num_slots + s), and
 * FRAME carries explicit acquisition and ready timestamps for every view;
 * STREAM_INFO.primary_view identifies the real camera timing leader.
 * RELEASE carries only the slot and frame identity needed for ownership; a
 * group is released as a unit.  OPEN with cfg.group_cam_slot set to a
 * negative virtual-group id on one of its physical members instead brings up
 * that exact group while keeping a per-camera single-view subscription.
 * LIST advertises each virtual group whose members exist; groups whose members
 * share a CSI wire with a live stream (the eye multidrop buses) are refused
 * with EBUSY at OPEN.
 *
 * Trust: a dma-buf fd is read-write access to frame memory shared by all
 * subscribers; access control is the socket path's permissions.  Closing a
 * stream or losing the connection implicitly releases every outstanding
 * frame, so a client must stop all CPU/GPU access first and must not pass the
 * fds beyond the connection lifetime.  Single-user device assumption, by
 * design.
 *
 * Helper pointer arguments are required unless explicitly described as
 * optional. A required NULL pointer is a caller bug and is asserted; wire
 * contents and sizes remain untrusted and are validated at runtime.
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TITAN_PROTO_VERSION 12u

/* OPEN cam_slot of a virtual group camera: coupled physical cameras as one
 * multi-view stream.  STEREO is the imx564 passthrough pair (view 0 = left,
 * view 1 = right; the right member leads shared 3A and the software phase
 * lock).  EYES_DOWN/EYES_UP are one-camera-per-eye tracking pairs on the
 * standalone sensor identities.  EYES_QUAD is all four eye-tracking sensors
 * (views in slot order 8/9/10/11 = left lower, right lower, left upper,
 * right upper; both sensors of each eye share one CSI wire on distinct
 * virtual channels).  The eye quad uses the native-pair identities, so it is
 * mutually exclusive with the standalone eye pairs (EBUSY).  HEAD_QUAD is
 * the four OpenPX head-VIO views in A/B/C/D order (slots 2/3/6/7).
 * TRACKING is the complete six-camera hardware timing domain in slot order
 * 2/3/4/5/6/7.  For both groups, slot 7 supplies hardware FSIN and every
 * other member uses its listener mode. */
#define TITAN_WIRE_CAM_STEREO    (-2)
#define TITAN_WIRE_CAM_EYES_DOWN (-3)
#define TITAN_WIRE_CAM_EYES_UP   (-4)
#define TITAN_WIRE_CAM_EYES_QUAD (-5)
#define TITAN_WIRE_CAM_HEAD_QUAD (-6)
#define TITAN_WIRE_CAM_TRACKING  (-7)

/* Views per stream this protocol can carry (the tracking rig uses all 6). */
#define TITAN_PROTO_MAX_VIEWS  6u
#define TITAN_PROTO_SOCKET     "/run/titan-server.sock"
#define TITAN_PROTO_SOCKET_ENV "TITAN_SERVER_SOCKET"
#define TITAN_PROTO_MAX_SLOTS  8u
#define TITAN_PROTO_MAX_CAMS   32u
#define TITAN_PROTO_NAME_LEN   64u
#define TITAN_PROTO_MAX_FDS    (TITAN_PROTO_MAX_VIEWS * TITAN_PROTO_MAX_SLOTS)

/* Kept independent of libtitancam so protocol-only clients do not need to
 * include the hardware library's public header. */
enum titan_wire_format {
    /* OPEN request sentinel. STREAM_INFO always resolves it to RAW8/10/12. */
    TITAN_WIRE_FMT_RAW_NATIVE = 0,
    TITAN_WIRE_FMT_RAW8 = 1,
    TITAN_WIRE_FMT_RAW10 = 2,
    TITAN_WIRE_FMT_RAW12 = 3,
    TITAN_WIRE_FMT_NV12 = 4,
    TITAN_WIRE_FMT_NV21 = 5,
    /* Bandwidth-compressed NV12 (Adreno UBWC): GPU-import only via
     * DRM_FORMAT_MOD_QCOM_COMPRESSED with two memory planes at offsets
     * {0, chroma_offset}, both at `stride`; the CPU mapping holds
     * compressed tiles. */
    TITAN_WIRE_FMT_NV12_UBWC = 6,
};

static inline int titan_wire_format_is_raw(uint32_t format)
{
    return format == TITAN_WIRE_FMT_RAW_NATIVE || format == TITAN_WIRE_FMT_RAW8 ||
           format == TITAN_WIRE_FMT_RAW10 || format == TITAN_WIRE_FMT_RAW12;
}

static inline int titan_wire_format_is_valid(uint32_t format)
{
    return titan_wire_format_is_raw(format) || format == TITAN_WIRE_FMT_NV12 ||
           format == TITAN_WIRE_FMT_NV21 || format == TITAN_WIRE_FMT_NV12_UBWC;
}

static inline int titan_wire_format_is_delivered(uint32_t format)
{
    return titan_wire_format_is_valid(format) && format != TITAN_WIRE_FMT_RAW_NATIVE;
}

static inline uint32_t titan_wire_raw_bpp(uint32_t format)
{
    if (format == TITAN_WIRE_FMT_RAW8)
        return 8;
    if (format == TITAN_WIRE_FMT_RAW10)
        return 10;
    if (format == TITAN_WIRE_FMT_RAW12)
        return 12;
    return 0;
}

enum titan_msg_type {
    TITAN_MSG_LIST = 1,         /* c->s: titan_msg_hdr */
    TITAN_MSG_LIST_REPLY,       /* s->c: titan_msg_list_reply */
    TITAN_MSG_OPEN,             /* c->s: titan_msg_open */
    TITAN_MSG_STREAM_INFO,      /* s->c: titan_msg_stream_info + SCM_RIGHTS fds */
    TITAN_MSG_FRAME,            /* s->c: titan_msg_frame */
    TITAN_MSG_RELEASE,          /* c->s: titan_msg_frame_ref */
    TITAN_MSG_CLOSE,            /* both: titan_msg_close */
    TITAN_MSG_SET_EXPOSURE,     /* c->s: titan_msg_set_exposure (controller) */
    TITAN_MSG_ERROR,            /* s->c: titan_msg_error */
    TITAN_MSG_LIST_MODES,       /* c->s: titan_msg_list_modes */
    TITAN_MSG_LIST_MODES_REPLY, /* s->c: titan_msg_list_modes_reply */
    TITAN_MSG_FRAME_USED,       /* c->s: titan_msg_frame_used (controller) */
};

#define TITAN_PROTO_MAX_MODES 8u

struct titan_msg_hdr {
    uint32_t type;
    uint32_t version;
};

struct titan_wire_camera {
    int32_t cam_slot;       /* cam_req_mgr cell index (or a virtual id
                             * like TITAN_WIRE_CAM_STEREO), use in OPEN */
    uint32_t width, height; /* default (auto-selected) mode, per view */
    uint32_t bpp;
    float fps;
    uint32_t num_views; /* buffers per frame event (1 for physical) */
    char name[TITAN_PROTO_NAME_LEN];
};

struct titan_msg_list_reply {
    struct titan_msg_hdr h;
    uint32_t count;
    uint32_t reserved;
    struct titan_wire_camera cameras[TITAN_PROTO_MAX_CAMS];
};

/* All fields 0 = defaults (auto mode, AE on).  Mirrors the sane subset of
 * titan_stream_config; bring-up overrides (slave/PHY/lane/...) are
 * deliberately direct-mode-only. */
struct titan_wire_config {
    int32_t cam_slot;
    uint32_t mode_index; /* 1-based sensor mode, 0 = auto */
    uint32_t ae;         /* 0 auto, 1 on, 2 off */
    uint32_t ae_target;
    uint32_t exp_lines;
    uint32_t gain; /* analog register code */
    uint32_t led_brightness;
    uint32_t led_pattern;
    uint32_t output_format; /* enum titan_wire_format */
    uint32_t digital_gain;  /* imx564 Q8 code; 0 = leave, 256 = 1x */
    int32_t group_cam_slot; /* 0 = standalone; otherwise the negative
                             * TITAN_WIRE_CAM_* id of the exact group to
                             * bring up while subscribing only this
                             * physical member's single-view stream.
                             * The physical cam_slot must be a member.
                             * Exposure options apply to its controller;
                             * ordinary pairs ignore mode, while native
                             * eye slots 8..11 honor mode 1/2. A live
                             * conflicting topology is EBUSY. Ignored
                             * when attaching to a running stream. */
    uint32_t reserved;
};

struct titan_msg_open {
    struct titan_msg_hdr h;
    struct titan_wire_config cfg;
};

/* Reply to OPEN.  num_views * num_slots dma-buf fds ride SCM_RIGHTS on this
 * message, view-major: view v's ring slot s is fd index v * num_slots + s.
 * All views share one geometry/format/allocation size.  The fds are the
 * client's own references: mmap/import once, close on stream end. */
struct titan_msg_stream_info {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    int32_t cam_slot;
    uint32_t width, height;
    uint32_t stride;                     /* RAW byte stride or YUV luma/chroma stride */
    uint32_t bpp;                        /* delivered RAW packing depth (8/10/12) */
    uint32_t num_slots;                  /* ring slots per view */
    uint32_t controller;                 /* 1 = you hold the controller role */
    uint32_t num_views;                  /* buffers per frame event */
    uint32_t cfa[TITAN_PROTO_MAX_VIEWS]; /* per-view Bayer order: 0 RGGB,
                                          * 1 GRBG, 2 GBRG, 3 BGGR (bit0 = red on odd
                                          * column, bit1 = red on odd row); colour
                                          * sensors only */
    uint32_t output_format;              /* enum titan_wire_format */
    uint32_t chroma_offset;              /* byte offset of UV/VU plane; 0 for RAW */
    uint32_t primary_view;               /* leader view for real camera timing/identity;
                                          * always 0 for a physical stream */
    uint64_t buf_size;                   /* per-slot allocation size */
    double fps;
    char name[TITAN_PROTO_NAME_LEN];
};

struct titan_wire_frame_view {
    uint32_t slot;
    uint32_t reserved;
    uint64_t frame_id;            /* Titan request/frame identity */
    uint64_t sensor_request_id;   /* kernel-reported applied sensor request */
    uint64_t exposure_request_id; /* request in exposure-controller session */
    uint64_t sof_qtimer_ns;       /* hardware CSID SOF, QTimer nanoseconds */
    uint64_t sof_boottime_ns;     /* same SOF in server CLOCK_BOOTTIME */
    /* Start of integration of the first rolling-shutter row in both
     * hardware QTimer and live-correlated CLOCK_BOOTTIME domains. */
    uint64_t first_row_exposure_start_qtimer_ns;
    uint64_t first_row_exposure_start_boottime_ns;
    uint64_t exposure_duration_ns; /* applied coarse integration */
    uint64_t ready_boottime_ns;    /* server-side buffer-ready observation */
};

/* FRAME is sent only when all views completed successfully. */
struct titan_msg_frame {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    uint32_t num_views;
    /*
     * Opaque identity allocated by titan-server for this atomic delivery.
     * It is not any view's hardware frame_id. Clients echo it in FRAME_USED.
     */
    uint64_t delivery_id;
    struct titan_wire_frame_view view[TITAN_PROTO_MAX_VIEWS];
};

struct titan_wire_frame_ref {
    uint32_t slot;
    uint32_t reserved;
    uint64_t frame_id;
};

/* RELEASE is an ownership message, not frame metadata. */
struct titan_msg_frame_ref {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    uint32_t num_views;
    struct titan_wire_frame_ref view[TITAN_PROTO_MAX_VIEWS];
};

/*
 * Timing feedback for one compositor/use-clock tick.  It is deliberately
 * independent of RELEASE: a grouped FRAME is selected atomically, so its
 * one opaque delivery_id is sufficient and no buffer ownership changes here.
 */
struct titan_msg_frame_used {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    uint32_t reserved;
    uint64_t use_sequence;
    uint64_t delivery_id;
    uint64_t used_boottime_ns;
    /* Maximum ready_boottime_ns of every view in the selected FRAME. */
    uint64_t frame_ready_boottime_ns;
};

#ifdef __cplusplus
static_assert(sizeof(titan_msg_stream_info) == 160, "titan STREAM_INFO wire ABI mismatch");
static_assert(sizeof(titan_wire_frame_view) == 80, "titan frame-view wire ABI mismatch");
static_assert(sizeof(titan_msg_frame) == 504, "titan FRAME wire ABI mismatch");
static_assert(sizeof(titan_msg_frame_ref) == 112, "titan frame-reference wire ABI mismatch");
static_assert(sizeof(titan_msg_frame_used) == 48, "titan FRAME_USED wire ABI mismatch");
#else
_Static_assert(sizeof(struct titan_msg_stream_info) == 160, "titan STREAM_INFO wire ABI mismatch");
_Static_assert(sizeof(struct titan_wire_frame_view) == 80, "titan frame-view wire ABI mismatch");
_Static_assert(sizeof(struct titan_msg_frame) == 504, "titan FRAME wire ABI mismatch");
_Static_assert(sizeof(struct titan_msg_frame_ref) == 112,
               "titan frame-reference wire ABI mismatch");
_Static_assert(sizeof(struct titan_msg_frame_used) == 48, "titan FRAME_USED wire ABI mismatch");
#endif

struct titan_msg_close {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    uint32_t reserved;
};

struct titan_msg_set_exposure {
    struct titan_msg_hdr h;
    uint32_t stream_id;
    uint32_t exp_lines; /* complete future SMIA 0x0202 plan; nonzero */
    uint32_t gain;      /* complete future SMIA 0x0204 code; zero may
                         * be a valid unity code on some sensors */
    uint32_t reserved;
};

struct titan_msg_error {
    struct titan_msg_hdr h;
    int32_t code;         /* -errno style */
    uint32_t in_reply_to; /* titan_msg_type that failed */
    char text[TITAN_PROTO_NAME_LEN];
};

struct titan_msg_list_modes {
    struct titan_msg_hdr h;
    int32_t cam_slot;
    uint32_t reserved;
};

struct titan_wire_mode {
    uint32_t width, height;
    uint32_t bpp;
    float fps;
    char name[TITAN_PROTO_NAME_LEN];
};

/* Modes are 1-based in titan_wire_config.mode_index: modes[i] = index i+1. */
struct titan_msg_list_modes_reply {
    struct titan_msg_hdr h;
    uint32_t count;
    uint32_t reserved;
    struct titan_wire_mode modes[TITAN_PROTO_MAX_MODES];
};

/* One buffer large enough for any protocol message. */
union titan_msg_any {
    struct titan_msg_hdr hdr;
    struct titan_msg_list_reply list_reply;
    struct titan_msg_open open;
    struct titan_msg_stream_info stream_info;
    struct titan_msg_frame frame;
    struct titan_msg_frame_ref frame_ref;
    struct titan_msg_frame_used frame_used;
    struct titan_msg_close close_;
    struct titan_msg_set_exposure set_exposure;
    struct titan_msg_error error;
    struct titan_msg_list_modes list_modes;
    struct titan_msg_list_modes_reply list_modes_reply;
};

/*
 * Header-only message constructors.  Each returns a fully zero-initialized
 * wire record, including reserved fields, with the current protocol header.
 * Callers may then fill any message-specific payload not passed here.
 */
static inline struct titan_msg_hdr titan_proto_make_header(uint32_t type)
{
    struct titan_msg_hdr message = { 0 };

    message.type = type;
    message.version = TITAN_PROTO_VERSION;
    return message;
}

static inline struct titan_msg_hdr titan_proto_make_list(void)
{
    return titan_proto_make_header(TITAN_MSG_LIST);
}

static inline struct titan_msg_open titan_proto_make_open(struct titan_wire_config config)
{
    struct titan_msg_open message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_OPEN);
    message.cfg = config;
    message.cfg.reserved = 0;
    return message;
}

static inline struct titan_msg_frame
titan_proto_make_frame(uint32_t stream_id, uint32_t num_views, uint64_t delivery_id)
{
    struct titan_msg_frame message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_FRAME);
    message.stream_id = stream_id;
    message.num_views = num_views;
    message.delivery_id = delivery_id;
    return message;
}

static inline struct titan_msg_frame_used
titan_proto_make_frame_used(uint32_t stream_id,
                            uint64_t use_sequence,
                            uint64_t delivery_id,
                            uint64_t used_boottime_ns,
                            uint64_t frame_ready_boottime_ns)
{
    struct titan_msg_frame_used message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_FRAME_USED);
    message.stream_id = stream_id;
    message.use_sequence = use_sequence;
    message.delivery_id = delivery_id;
    message.used_boottime_ns = used_boottime_ns;
    message.frame_ready_boottime_ns = frame_ready_boottime_ns;
    return message;
}

static inline struct titan_msg_close titan_proto_make_close(uint32_t stream_id)
{
    struct titan_msg_close message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_CLOSE);
    message.stream_id = stream_id;
    return message;
}

static inline struct titan_msg_set_exposure
titan_proto_make_set_exposure(uint32_t stream_id, uint32_t exp_lines, uint32_t gain)
{
    struct titan_msg_set_exposure message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_SET_EXPOSURE);
    message.stream_id = stream_id;
    message.exp_lines = exp_lines;
    message.gain = gain;
    return message;
}

static inline struct titan_msg_list_modes titan_proto_make_list_modes(int32_t cam_slot)
{
    struct titan_msg_list_modes message = { 0 };

    message.h = titan_proto_make_header(TITAN_MSG_LIST_MODES);
    message.cam_slot = cam_slot;
    return message;
}

/*
 * Transport-independent packet-envelope helpers.  They do not receive,
 * allocate, queue, map, or close anything, and may be used with storage
 * owned by any event loop.
 *
 * Validation covers the fixed wire envelope: complete header, protocol
 * version, known message type, permitted direction, and exact record size.
 * Type-specific payload invariants and SCM_RIGHTS descriptor counts require
 * stream/transport context and remain the receiver's responsibility.
 */
enum titan_proto_direction {
    TITAN_PROTO_CLIENT_TO_SERVER,
    TITAN_PROTO_SERVER_TO_CLIENT,
};

enum titan_proto_validation_result {
    TITAN_PROTO_VALID = 0,
    TITAN_PROTO_INVALID_HEADER,
    TITAN_PROTO_INVALID_VERSION,
    TITAN_PROTO_INVALID_TYPE,
    TITAN_PROTO_INVALID_DIRECTION,
    TITAN_PROTO_INVALID_SIZE,
    TITAN_PROTO_INVALID_PAYLOAD,
    TITAN_PROTO_INVALID_FD_COUNT,
};

static inline size_t titan_proto_message_size(uint32_t type)
{
    switch (type) {
    case TITAN_MSG_LIST:
        return sizeof(struct titan_msg_hdr);
    case TITAN_MSG_LIST_REPLY:
        return sizeof(struct titan_msg_list_reply);
    case TITAN_MSG_OPEN:
        return sizeof(struct titan_msg_open);
    case TITAN_MSG_STREAM_INFO:
        return sizeof(struct titan_msg_stream_info);
    case TITAN_MSG_FRAME:
        return sizeof(struct titan_msg_frame);
    case TITAN_MSG_RELEASE:
        return sizeof(struct titan_msg_frame_ref);
    case TITAN_MSG_CLOSE:
        return sizeof(struct titan_msg_close);
    case TITAN_MSG_SET_EXPOSURE:
        return sizeof(struct titan_msg_set_exposure);
    case TITAN_MSG_ERROR:
        return sizeof(struct titan_msg_error);
    case TITAN_MSG_LIST_MODES:
        return sizeof(struct titan_msg_list_modes);
    case TITAN_MSG_LIST_MODES_REPLY:
        return sizeof(struct titan_msg_list_modes_reply);
    case TITAN_MSG_FRAME_USED:
        return sizeof(struct titan_msg_frame_used);
    default:
        return 0;
    }
}

static inline int titan_proto_message_has_direction(uint32_t type,
                                                    enum titan_proto_direction direction)
{
    switch (type) {
    case TITAN_MSG_LIST:
    case TITAN_MSG_OPEN:
    case TITAN_MSG_RELEASE:
    case TITAN_MSG_SET_EXPOSURE:
    case TITAN_MSG_LIST_MODES:
    case TITAN_MSG_FRAME_USED:
        return direction == TITAN_PROTO_CLIENT_TO_SERVER;
    case TITAN_MSG_LIST_REPLY:
    case TITAN_MSG_STREAM_INFO:
    case TITAN_MSG_FRAME:
    case TITAN_MSG_ERROR:
    case TITAN_MSG_LIST_MODES_REPLY:
        return direction == TITAN_PROTO_SERVER_TO_CLIENT;
    case TITAN_MSG_CLOSE:
        return direction == TITAN_PROTO_CLIENT_TO_SERVER ||
               direction == TITAN_PROTO_SERVER_TO_CLIENT;
    default:
        return 0;
    }
}

static inline enum titan_proto_validation_result
titan_proto_validate_message(const void *message, size_t size, enum titan_proto_direction direction)
{
    struct titan_msg_hdr header;

    assert(message);
    if (size < sizeof(header))
        return TITAN_PROTO_INVALID_HEADER;
    memcpy(&header, message, sizeof(header));
    if (header.version != TITAN_PROTO_VERSION)
        return TITAN_PROTO_INVALID_VERSION;
    size_t expected = titan_proto_message_size(header.type);
    if (!expected)
        return TITAN_PROTO_INVALID_TYPE;
    if (!titan_proto_message_has_direction(header.type, direction))
        return TITAN_PROTO_INVALID_DIRECTION;
    if (size != expected)
        return TITAN_PROTO_INVALID_SIZE;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result
titan_proto_validate_set_exposure(const struct titan_msg_set_exposure *request)
{
    assert(request);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(request, sizeof(*request), TITAN_PROTO_CLIENT_TO_SERVER);

    if (result != TITAN_PROTO_VALID)
        return result;
    if (!request->stream_id || !request->exp_lines || request->exp_lines > UINT16_MAX ||
        request->gain > UINT16_MAX || request->reserved)
        return TITAN_PROTO_INVALID_PAYLOAD;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result
titan_proto_validate_frame_used(const struct titan_msg_frame_used *feedback)
{
    assert(feedback);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(feedback, sizeof(*feedback), TITAN_PROTO_CLIENT_TO_SERVER);

    if (result != TITAN_PROTO_VALID)
        return result;
    if (!feedback->stream_id || feedback->reserved || !feedback->use_sequence ||
        !feedback->delivery_id || !feedback->used_boottime_ns ||
        !feedback->frame_ready_boottime_ns ||
        feedback->used_boottime_ns < feedback->frame_ready_boottime_ns)
        return TITAN_PROTO_INVALID_PAYLOAD;
    return TITAN_PROTO_VALID;
}

/*
 * Payload helpers.  Like the envelope helpers above, these operate only on
 * caller-owned storage.  They perform no socket I/O and take no ownership of
 * dma-buf descriptors.
 */
static inline size_t titan_proto_stream_fd_count(const struct titan_msg_stream_info *info)
{
    assert(info);
    if (info->num_views < 1 || info->num_views > TITAN_PROTO_MAX_VIEWS || info->num_slots < 1 ||
        info->num_slots > TITAN_PROTO_MAX_SLOTS)
        return 0;
    return (size_t)info->num_views * info->num_slots;
}

/* Convert the protocol's view-major {view, slot} address to an fd-array
 * index.  Returns 1 on success and 0 for an invalid address or stream shape. */
static inline int titan_proto_stream_slot_index(const struct titan_msg_stream_info *info,
                                                uint32_t view,
                                                uint32_t slot,
                                                size_t *index)
{
    assert(info);
    assert(index);
    if (!titan_proto_stream_fd_count(info) || view >= info->num_views || slot >= info->num_slots)
        return 0;
    *index = (size_t)view * info->num_slots + slot;
    return 1;
}

static inline enum titan_proto_validation_result
titan_proto_validate_stream_info(const struct titan_msg_stream_info *info, size_t received_fd_count)
{
    assert(info);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(info, sizeof(*info), TITAN_PROTO_SERVER_TO_CLIENT);

    if (result != TITAN_PROTO_VALID)
        return result;
    size_t expected_fd_count = titan_proto_stream_fd_count(info);
    uint32_t raw_bpp = titan_wire_raw_bpp(info->output_format);

    if (!info->stream_id || !expected_fd_count || info->controller > 1 ||
        info->primary_view >= info->num_views ||
        !titan_wire_format_is_delivered(info->output_format) || (raw_bpp && info->bpp != raw_bpp))
        return TITAN_PROTO_INVALID_PAYLOAD;
    if (received_fd_count != expected_fd_count)
        return TITAN_PROTO_INVALID_FD_COUNT;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result
titan_proto_validate_list_reply(const struct titan_msg_list_reply *reply)
{
    assert(reply);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(reply, sizeof(*reply), TITAN_PROTO_SERVER_TO_CLIENT);

    if (result != TITAN_PROTO_VALID)
        return result;
    if (reply->count > TITAN_PROTO_MAX_CAMS)
        return TITAN_PROTO_INVALID_PAYLOAD;
    for (uint32_t i = 0; i < reply->count; i++)
        if (reply->cameras[i].num_views < 1 || reply->cameras[i].num_views > TITAN_PROTO_MAX_VIEWS)
            return TITAN_PROTO_INVALID_PAYLOAD;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result
titan_proto_validate_list_modes_reply(const struct titan_msg_list_modes_reply *reply)
{
    assert(reply);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(reply, sizeof(*reply), TITAN_PROTO_SERVER_TO_CLIENT);

    if (result != TITAN_PROTO_VALID)
        return result;
    return reply->count <= TITAN_PROTO_MAX_MODES ? TITAN_PROTO_VALID : TITAN_PROTO_INVALID_PAYLOAD;
}

static inline enum titan_proto_validation_result
titan_proto_validate_frame(const struct titan_msg_frame *frame,
                           const struct titan_msg_stream_info *stream)
{
    assert(frame);
    assert(stream);
    enum titan_proto_validation_result result =
        titan_proto_validate_message(frame, sizeof(*frame), TITAN_PROTO_SERVER_TO_CLIENT);

    if (result != TITAN_PROTO_VALID)
        return result;
    size_t expected_fd_count = titan_proto_stream_fd_count(stream);
    if (!expected_fd_count ||
        titan_proto_validate_stream_info(stream, expected_fd_count) != TITAN_PROTO_VALID ||
        frame->stream_id != stream->stream_id || frame->num_views != stream->num_views ||
        !frame->delivery_id)
        return TITAN_PROTO_INVALID_PAYLOAD;

    for (uint32_t view = 0; view < frame->num_views; view++) {
        const struct titan_wire_frame_view *item = &frame->view[view];

        if (item->slot >= stream->num_slots || item->reserved || !item->frame_id ||
            !item->sensor_request_id || !item->exposure_request_id || !item->sof_qtimer_ns ||
            !item->sof_boottime_ns || !item->first_row_exposure_start_qtimer_ns ||
            !item->first_row_exposure_start_boottime_ns || !item->exposure_duration_ns ||
            !item->ready_boottime_ns ||
            item->first_row_exposure_start_qtimer_ns >= item->sof_qtimer_ns ||
            item->first_row_exposure_start_boottime_ns >= item->sof_boottime_ns ||
            item->sof_qtimer_ns - item->first_row_exposure_start_qtimer_ns !=
                item->sof_boottime_ns - item->first_row_exposure_start_boottime_ns ||
            item->exposure_duration_ns >=
                item->sof_qtimer_ns - item->first_row_exposure_start_qtimer_ns ||
            item->ready_boottime_ns < item->sof_boottime_ns)
            return TITAN_PROTO_INVALID_PAYLOAD;
    }
    return TITAN_PROTO_VALID;
}

/*
 * Build self-contained use feedback from one validated atomic FRAME.  The
 * client supplies only its use-clock sample and sequence; the helper copies
 * the opaque delivery ID and computes the group-ready timestamp.
 */
static inline enum titan_proto_validation_result
titan_proto_frame_used_from_frame(struct titan_msg_frame_used *feedback,
                                  const struct titan_msg_stream_info *stream,
                                  const struct titan_msg_frame *frame,
                                  uint64_t use_sequence,
                                  uint64_t used_boottime_ns)
{
    assert(feedback);
    assert(stream);
    assert(frame);
    if (!use_sequence || !used_boottime_ns)
        return TITAN_PROTO_INVALID_PAYLOAD;
    enum titan_proto_validation_result result = titan_proto_validate_frame(frame, stream);

    if (result != TITAN_PROTO_VALID)
        return result;
    uint64_t ready = 0;
    for (uint32_t view = 0; view < frame->num_views; view++)
        if (frame->view[view].ready_boottime_ns > ready)
            ready = frame->view[view].ready_boottime_ns;
    *feedback = titan_proto_make_frame_used(
        frame->stream_id, use_sequence, frame->delivery_id, used_boottime_ns, ready);
    return titan_proto_validate_frame_used(feedback);
}

static inline enum titan_proto_validation_result titan_proto_frame_ref_init(
    struct titan_msg_frame_ref *release, uint32_t stream_id, uint32_t num_views)
{
    assert(release);
    if (!stream_id || num_views < 1 || num_views > TITAN_PROTO_MAX_VIEWS)
        return TITAN_PROTO_INVALID_PAYLOAD;
    *release = (struct titan_msg_frame_ref){ 0 };
    release->h = titan_proto_make_header(TITAN_MSG_RELEASE);
    release->stream_id = stream_id;
    release->num_views = num_views;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result titan_proto_frame_ref_set_view(
    struct titan_msg_frame_ref *release, uint32_t view, uint32_t slot, uint64_t frame_id)
{
    assert(release);
    if (release->h.type != TITAN_MSG_RELEASE || release->h.version != TITAN_PROTO_VERSION ||
        !release->stream_id || release->num_views < 1 ||
        release->num_views > TITAN_PROTO_MAX_VIEWS || view >= release->num_views ||
        slot >= TITAN_PROTO_MAX_SLOTS || !frame_id)
        return TITAN_PROTO_INVALID_PAYLOAD;
    release->view[view].slot = slot;
    release->view[view].frame_id = frame_id;
    return TITAN_PROTO_VALID;
}

static inline enum titan_proto_validation_result
titan_proto_make_release(struct titan_msg_frame_ref *release,
                         const struct titan_msg_stream_info *stream,
                         const struct titan_msg_frame *frame)
{
    assert(release);
    assert(stream);
    assert(frame);
    enum titan_proto_validation_result result = titan_proto_validate_frame(frame, stream);

    if (result != TITAN_PROTO_VALID)
        return result;
    result = titan_proto_frame_ref_init(release, frame->stream_id, frame->num_views);
    if (result != TITAN_PROTO_VALID)
        return result;
    for (uint32_t view = 0; view < frame->num_views; view++) {
        result = titan_proto_frame_ref_set_view(
            release, view, frame->view[view].slot, frame->view[view].frame_id);
        if (result != TITAN_PROTO_VALID)
            return result;
    }
    return TITAN_PROTO_VALID;
}

#endif /* TITAN_PROTO_H */
