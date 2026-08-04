// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Minimal Qualcomm SSC (sns_client QMI service over QRTR) IMU client.
 *
 * The Galaxy XR IMU hangs off the ADSP sensor core, reachable through the SEE
 * "sns_client" QMI service (id 400) over an AF_QIPCRTR socket. QMI is only an
 * envelope, the payloads are protobufs, both hand-rolled here.
 *
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_ssc.h"

#include "xrt/xrt_compiler.h"
#include "util/u_logging.h"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef AF_QIPCRTR
#define AF_QIPCRTR 42
#endif

struct sockaddr_qrtr
{
	unsigned short sq_family;
	uint32_t sq_node;
	uint32_t sq_port;
};

#define QRTR_PORT_CTRL 0xfffffffeu
#define QRTR_TYPE_NEW_SERVER 4
#define QRTR_TYPE_NEW_LOOKUP 10

struct qrtr_ctrl_pkt
{
	uint32_t cmd, service, instance, node, port;
};

#define QMI_SSC_SERVICE 400
#define QMI_TYPE_REQUEST 0x00
#define QMI_TYPE_INDICATION 0x04
#define SSC_MSG_CONTROL 0x0020
#define SSC_IND_REPORT_SMALL 0x0021
#define SSC_IND_REPORT_LARGE 0x0022
#define SSC_TLV_REQ_DATA 0x01
#define SSC_TLV_REQ_RTYPE 0x10
#define SSC_TLV_IND_DATA 0x02
#define SSC_REPORT_TYPE_LARGE 0x01

#define SNS_SUID_REQ 512
#define SNS_SUID_EVENT 768
#define SNS_STD_SENSOR_CONFIG 513
#define SNS_STD_ON_CHANGE_CONFIG 514
#define SNS_STD_SENSOR_EVENT 1025
//! sns_cal_event from the *_cal algo sensors: bias/scale_factor/comp_matrix.
#define SNS_CAL_EVENT 1022
//! AKM AKL "ipd" algo event: 14 floats [ipd_L, ipd_R, mag0..3 xyz].
#define SNS_AKL_IPD_EVENT 717
#define SNS_STD_ATTR_REQ 1
#define SNS_STD_ATTR_EVENT 128
#define SNS_ATTR_HW_ID 18
#define SNS_ATTR_RIGID_BODY 19
#define SUID_LOOKUP_VAL 0xABABABABABABABABULL

#define SSC_PROCESSOR_APSS 1
#define SSC_SUSPEND_WAKEUP 0

//! qtimer runs at 19.2MHz, 1e9 / 19.2e6 == 625 / 12 ns per tick.
#define SSC_TICKS_TO_NS(t) ((int64_t)((t) * 625 / 12))


/*
 *
 * Protobuf helpers.
 *
 */

static void
put8(uint8_t *b, size_t *o, uint8_t v)
{
	b[(*o)++] = v;
}

static void
put16(uint8_t *b, size_t *o, uint16_t v)
{
	b[(*o)++] = (uint8_t)v;
	b[(*o)++] = (uint8_t)(v >> 8);
}

static void
pb_varint(uint8_t *b, size_t *o, uint64_t v)
{
	do {
		uint8_t x = v & 0x7f;
		v >>= 7;
		if (v) {
			x |= 0x80;
		}
		b[(*o)++] = x;
	} while (v);
}

static void
pb_key(uint8_t *b, size_t *o, uint32_t field, uint32_t wire)
{
	pb_varint(b, o, ((uint64_t)field << 3) | wire);
}

static void
pb_fixed32(uint8_t *b, size_t *o, uint32_t v)
{
	for (int i = 0; i < 4; i++) {
		b[(*o)++] = (uint8_t)(v >> (8 * i));
	}
}

static void
pb_fixed64(uint8_t *b, size_t *o, uint64_t v)
{
	for (int i = 0; i < 8; i++) {
		b[(*o)++] = (uint8_t)(v >> (8 * i));
	}
}

static void
pb_bytes(uint8_t *b, size_t *o, uint32_t field, const void *d, size_t n)
{
	pb_key(b, o, field, 2);
	pb_varint(b, o, n);
	memcpy(b + *o, d, n);
	*o += n;
}

typedef struct
{
	const uint8_t *p, *end;
} pb_t;

static uint64_t
d_varint(pb_t *b)
{
	uint64_t r = 0;
	int s = 0;
	while (b->p < b->end) {
		uint8_t x = *b->p++;
		r |= (uint64_t)(x & 0x7f) << s;
		if (!(x & 0x80)) {
			break;
		}
		s += 7;
	}
	return r;
}

static uint32_t
d_fixed32(pb_t *b)
{
	uint32_t v = 0;
	for (int i = 0; i < 4 && b->p < b->end; i++) {
		v |= (uint32_t)*b->p++ << (8 * i);
	}
	return v;
}

static uint64_t
d_fixed64(pb_t *b)
{
	uint64_t v = 0;
	for (int i = 0; i < 8 && b->p < b->end; i++) {
		v |= (uint64_t)*b->p++ << (8 * i);
	}
	return v;
}

static int
d_key(pb_t *b, uint32_t *field, uint32_t *wire)
{
	if (b->p >= b->end) {
		return 0;
	}
	uint64_t k = d_varint(b);
	*field = (uint32_t)(k >> 3);
	*wire = k & 7;
	return 1;
}

static pb_t
d_sub(pb_t *b)
{
	uint64_t n = d_varint(b);
	pb_t s = {b->p, b->p + n};
	if (s.end > b->end) {
		s.end = b->end;
	}
	b->p = s.end;
	return s;
}

static void
d_skip(pb_t *b, uint32_t wire)
{
	switch (wire) {
	case 0: d_varint(b); break;
	case 1: b->p += 8; break;
	case 5: b->p += 4; break;
	case 2: {
		uint64_t n = d_varint(b);
		b->p += n;
		break;
	}
	default: b->p = b->end; break;
	}
	if (b->p > b->end) {
		b->p = b->end;
	}
}

static float
u2f(uint32_t u)
{
	float f;
	memcpy(&f, &u, 4);
	return f;
}


/*
 *
 * SSC requests.
 *
 */

static int
ssc_send(struct galaxyxr_ssc *ssc, uint64_t uid_high, uint64_t uid_low, uint32_t msg_id, const uint8_t *payload, size_t payload_len)
{
	uint8_t pb[1024];
	size_t po = 0;

	// SscClientRequest{ uid=1, msg_id=2 fixed32, config=3, request=4 }
	{
		uint8_t uidm[32];
		size_t uo = 0;
		pb_key(uidm, &uo, 1, 1);
		pb_fixed64(uidm, &uo, uid_low);
		pb_key(uidm, &uo, 2, 1);
		pb_fixed64(uidm, &uo, uid_high);
		pb_bytes(pb, &po, 1, uidm, uo);
	}
	pb_key(pb, &po, 2, 5);
	pb_fixed32(pb, &po, msg_id);
	{
		uint8_t cfg[16];
		size_t co = 0;
		pb_key(cfg, &co, 1, 0);
		pb_varint(cfg, &co, SSC_PROCESSOR_APSS);
		pb_key(cfg, &co, 2, 0);
		pb_varint(cfg, &co, SSC_SUSPEND_WAKEUP);
		pb_bytes(pb, &po, 3, cfg, co);
	}
	{
		uint8_t body[768];
		size_t bo = 0;
		if (payload && payload_len) {
			pb_bytes(body, &bo, 2, payload, payload_len);
		}
		pb_bytes(pb, &po, 4, body, bo);
	}

	// QMI SDU: [type u8][txn u16][msgid u16][len u16] + TLVs.
	uint8_t msg[1280];
	size_t mo = 0;
	put8(msg, &mo, QMI_TYPE_REQUEST);
	put16(msg, &mo, ssc->txn++);
	put16(msg, &mo, SSC_MSG_CONTROL);
	size_t len_at = mo;
	put16(msg, &mo, 0);
	size_t tlv_start = mo;
	put8(msg, &mo, SSC_TLV_REQ_DATA);
	put16(msg, &mo, (uint16_t)(2 + po));
	put16(msg, &mo, (uint16_t)po);
	memcpy(msg + mo, pb, po);
	mo += po;
	put8(msg, &mo, SSC_TLV_REQ_RTYPE);
	put16(msg, &mo, 1);
	put8(msg, &mo, SSC_REPORT_TYPE_LARGE);
	msg[len_at] = (uint8_t)(mo - tlv_start);
	msg[len_at + 1] = (uint8_t)((mo - tlv_start) >> 8);

	struct sockaddr_qrtr da = {AF_QIPCRTR, ssc->node, ssc->port};
	if (sendto(ssc->fd, msg, mo, 0, (struct sockaddr *)&da, sizeof(da)) < 0) {
		return -errno;
	}
	return 0;
}

static int
ssc_lookup(struct galaxyxr_ssc *ssc, const char *data_type)
{
	uint8_t pl[64];
	size_t o = 0;
	pb_bytes(pl, &o, 1, data_type, strlen(data_type));
	pb_key(pl, &o, 2, 0);
	pb_varint(pl, &o, 0);
	pb_key(pl, &o, 3, 0);
	pb_varint(pl, &o, 0);
	return ssc_send(ssc, SUID_LOOKUP_VAL, SUID_LOOKUP_VAL, SNS_SUID_REQ, pl, o);
}

static int
ssc_enable(struct galaxyxr_ssc *ssc, uint64_t uid_high, uint64_t uid_low, float rate)
{
	uint8_t pl[16];
	size_t o = 0;
	uint32_t r;
	memcpy(&r, &rate, 4);
	pb_key(pl, &o, 1, 5);
	pb_fixed32(pl, &o, r);
	return ssc_send(ssc, uid_high, uid_low, SNS_STD_SENSOR_CONFIG, pl, o);
}

static int
ssc_attr_req(struct galaxyxr_ssc *ssc, uint64_t uid_high, uint64_t uid_low)
{
	uint8_t pl[8];
	size_t o = 0;
	pb_key(pl, &o, 2, 0);
	pb_varint(pl, &o, 0);
	return ssc_send(ssc, uid_high, uid_low, SNS_STD_ATTR_REQ, pl, o);
}

// SscAttrArrayValue{ v=1 repeated SscAttrValue{ str=2, flt=3, int=4 fx64, bool=5 } }
static long
attr_first_int(pb_t va)
{
	uint32_t f, w;
	while (d_key(&va, &f, &w)) {
		if (f == 1 && w == 2) {
			pb_t val = d_sub(&va);
			uint32_t f2, w2;
			while (d_key(&val, &f2, &w2)) {
				if (f2 == 4 && w2 == 1) {
					return (long)d_fixed64(&val);
				}
				if (f2 == 5 && w2 == 0) {
					return (long)d_varint(&val);
				}
				d_skip(&val, w2);
			}
		} else {
			d_skip(&va, w);
		}
	}
	return -1;
}


/*
 *
 * Response handling.
 *
 */

static void
handle_suid_event(struct galaxyxr_ssc *ssc, pb_t body)
{
	// SscSuidResponse{ data_type=1 string, uid=2 repeated SscUid{lo=1,hi=2} }
	const char *dt = NULL;
	size_t dtlen = 0;
	uint64_t ulo[16], uhi[16];
	uint32_t nu = 0;
	uint32_t f, w;

	while (d_key(&body, &f, &w)) {
		if (f == 1 && w == 2) {
			pb_t s = d_sub(&body);
			dt = (const char *)s.p;
			dtlen = (size_t)(s.end - s.p);
		} else if (f == 2 && w == 2) {
			pb_t s = d_sub(&body);
			uint64_t lo = 0, hi = 0;
			uint32_t f2, w2;
			while (d_key(&s, &f2, &w2)) {
				if (f2 == 1 && w2 == 1) {
					lo = d_fixed64(&s);
				} else if (f2 == 2 && w2 == 1) {
					hi = d_fixed64(&s);
				} else {
					d_skip(&s, w2);
				}
			}
			if (nu < 16) {
				ulo[nu] = lo;
				uhi[nu] = hi;
				nu++;
			}
		} else {
			d_skip(&body, w);
		}
	}

	if (dt == NULL || nu == 0) {
		return;
	}

	uint8_t type;
	if (dtlen == 5 && memcmp(dt, "accel", 5) == 0) {
		type = GALAXYXR_SSC_ACCEL;
	} else if (dtlen == 4 && memcmp(dt, "gyro", 4) == 0) {
		type = GALAXYXR_SSC_GYRO;
	} else if (dtlen == 3 && memcmp(dt, "ipd", 3) == 0) {
		type = GALAXYXR_SSC_IPD;
	} else if (dtlen == 9 && memcmp(dt, "proximity", 9) == 0) {
		type = GALAXYXR_SSC_PROX;
	} else if (dtlen == 8 && memcmp(dt, "gyro_cal", 8) == 0) {
		type = GALAXYXR_SSC_GYRO_CAL;
	} else {
		return;
	}

	U_LOG_D("galaxyxr: %u instance(s) of sensor type %u discovered", nu, type);

	for (uint32_t i = 0; i < nu && ssc->pending_count < ARRAY_SIZE(ssc->pending); i++) {
		struct galaxyxr_ssc_pending *p = &ssc->pending[ssc->pending_count++];
		p->lo = ulo[i];
		p->hi = uhi[i];
		p->type = type;
		p->done = false;
		ssc_attr_req(ssc, uhi[i], ulo[i]);
	}
}

static void
handle_attr_event(struct galaxyxr_ssc *ssc, uint64_t uid_lo, uint64_t uid_hi, pb_t body)
{
	// SscAttrResponse{ attr=1 repeated SscAttr{ id=1 varint, value_array=2 } }
	long hw_id = -1;
	long rigid = -1;
	uint32_t f, w;

	while (d_key(&body, &f, &w)) {
		if (f == 1 && w == 2) {
			pb_t a = d_sub(&body);
			long id = -1;
			pb_t va = {NULL, NULL};
			uint32_t f2, w2;
			while (d_key(&a, &f2, &w2)) {
				if (f2 == 1 && w2 == 0) {
					id = (long)d_varint(&a);
				} else if (f2 == 2 && w2 == 2) {
					va = d_sub(&a);
				} else {
					d_skip(&a, w2);
				}
			}
			if (va.p != NULL && id == SNS_ATTR_HW_ID) {
				hw_id = attr_first_int(va);
			} else if (va.p != NULL && id == SNS_ATTR_RIGID_BODY) {
				rigid = attr_first_int(va);
			}
		} else {
			d_skip(&body, w);
		}
	}

	for (uint32_t i = 0; i < ssc->pending_count; i++) {
		struct galaxyxr_ssc_pending *p = &ssc->pending[i];
		if (p->done || p->lo != uid_lo || p->hi != uid_hi) {
			continue;
		}
		p->done = true;

		if (p->type == GALAXYXR_SSC_GYRO_CAL) {
			// One online bias estimator per IMU; these carry no hw_id
			// attr, the rigid_body attr names the IMU they calibrate.
			if (rigid != 0 || ssc->gyro_cal.valid) {
				return;
			}
			ssc->gyro_cal.lo = uid_lo;
			ssc->gyro_cal.hi = uid_hi;
			ssc->gyro_cal.valid = true;
			U_LOG_I("galaxyxr: enabling gyro_cal (rigid_body 0, on-change)");
			ssc_send(ssc, uid_hi, uid_lo, SNS_STD_ON_CHANGE_CONFIG, NULL, 0);
			return;
		}

		if (p->type == GALAXYXR_SSC_IPD || p->type == GALAXYXR_SSC_PROX) {
			// One instance each; on-change, no rate.
			struct galaxyxr_ssc_suid *chg = p->type == GALAXYXR_SSC_IPD ? &ssc->ipd : &ssc->prox;
			if (chg->valid) {
				return;
			}
			chg->lo = uid_lo;
			chg->hi = uid_hi;
			chg->valid = true;
			U_LOG_I("galaxyxr: enabling %s (on-change)",
			        p->type == GALAXYXR_SSC_IPD ? "ipd" : "proximity");
			ssc_send(ssc, uid_hi, uid_lo, SNS_STD_ON_CHANGE_CONFIG, NULL, 0);
			return;
		}

		struct galaxyxr_ssc_suid *slot = p->type == GALAXYXR_SSC_GYRO ? &ssc->gyro : &ssc->accel;
		if (hw_id != 0 || slot->valid) {
			return;
		}

		slot->lo = uid_lo;
		slot->hi = uid_hi;
		slot->valid = true;
		U_LOG_I("galaxyxr: enabling %s (hw_id 0) at %.0f Hz",
		        p->type == GALAXYXR_SSC_GYRO ? "gyro" : "accel", ssc->rate_hz);
		ssc_enable(ssc, uid_hi, uid_lo, ssc->rate_hz);
		return;
	}
}

static int
handle_body(struct galaxyxr_ssc *ssc,
            uint64_t uid_lo,
            uint64_t uid_hi,
            uint32_t msg_id,
            int64_t timestamp_ns,
            pb_t body,
            galaxyxr_ssc_sample_fn fn,
            void *ud)
{
	if (uid_lo == SUID_LOOKUP_VAL && uid_hi == SUID_LOOKUP_VAL && msg_id == SNS_SUID_EVENT) {
		handle_suid_event(ssc, body);
		return 0;
	}

	if (msg_id == SNS_STD_ATTR_EVENT) {
		handle_attr_event(ssc, uid_lo, uid_hi, body);
		return 0;
	}

	if (msg_id == SNS_CAL_EVENT) {
		if (!ssc->gyro_cal.valid || uid_lo != ssc->gyro_cal.lo || uid_hi != ssc->gyro_cal.hi) {
			return 0;
		}
		// SnsCalEvent{ bias=1 repeated float, scale_factor=2, comp_matrix=3, status=4 }
		float v[3] = {0};
		int nv = 0;
		long status = -1;
		uint32_t f, w;
		while (d_key(&body, &f, &w)) {
			if (f == 1 && w == 2) {
				pb_t s = d_sub(&body);
				while (s.p < s.end && nv < 3) {
					v[nv++] = u2f(d_fixed32(&s));
				}
			} else if (f == 1 && w == 5) {
				if (nv < 3) {
					v[nv++] = u2f(d_fixed32(&body));
				}
			} else if (f == 4 && w == 0) {
				status = (long)d_varint(&body);
			} else {
				d_skip(&body, w);
			}
		}
		// Status is the SEE accuracy enum, only trust medium/high.
		if (nv < 3 || status < 2) {
			return 0;
		}
		struct galaxyxr_ssc_sample sample = {
		    .type = GALAXYXR_SSC_GYRO_CAL,
		    .v = {v[0], v[1], v[2]},
		    .timestamp_ns = timestamp_ns,
		};
		fn(ud, &sample);
		return 1;
	}

	if (msg_id != SNS_STD_SENSOR_EVENT && msg_id != SNS_AKL_IPD_EVENT) {
		return 0;
	}

	enum galaxyxr_ssc_sample_type type;
	if (ssc->accel.valid && uid_lo == ssc->accel.lo && uid_hi == ssc->accel.hi) {
		type = GALAXYXR_SSC_ACCEL;
	} else if (ssc->gyro.valid && uid_lo == ssc->gyro.lo && uid_hi == ssc->gyro.hi) {
		type = GALAXYXR_SSC_GYRO;
	} else if (ssc->ipd.valid && uid_lo == ssc->ipd.lo && uid_hi == ssc->ipd.hi) {
		type = GALAXYXR_SSC_IPD;
	} else if (ssc->prox.valid && uid_lo == ssc->prox.lo && uid_hi == ssc->prox.hi) {
		type = GALAXYXR_SSC_PROX;
	} else {
		return 0;
	}

	// SnsStdSensorEvent{ data=1 repeated float, status=2 }
	float v[4] = {0};
	int nv = 0;
	uint32_t f, w;
	while (d_key(&body, &f, &w)) {
		if (f == 1 && w == 5) {
			if (nv < 4) {
				v[nv++] = u2f(d_fixed32(&body));
			}
		} else if (f == 1 && w == 2) {
			pb_t s = d_sub(&body);
			while (s.p < s.end && nv < 4) {
				v[nv++] = u2f(d_fixed32(&s));
			}
		} else {
			d_skip(&body, w);
		}
	}

	int need = 3;
	if (type == GALAXYXR_SSC_IPD) {
		need = 2;
	} else if (type == GALAXYXR_SSC_PROX) {
		need = 1;
	}
	if (nv < need) {
		return 0;
	}

	struct galaxyxr_ssc_sample sample = {
	    .type = type,
	    .v = {v[0], v[1], v[2]},
	    .timestamp_ns = timestamp_ns,
	};
	fn(ud, &sample);

	return 1;
}

static int
handle_response(struct galaxyxr_ssc *ssc, pb_t r, galaxyxr_ssc_sample_fn fn, void *ud)
{
	// SscClientResponse{ uid=1, response=2 repeated
	//     SscClientResponseBody{ msg_id=1 fixed32, timestamp=2 fixed64, msg=3 bytes } }
	uint64_t uid_lo = 0, uid_hi = 0;
	int samples = 0;
	uint32_t f, w;

	while (d_key(&r, &f, &w)) {
		if (f == 1 && w == 2) {
			pb_t s = d_sub(&r);
			uint32_t f2, w2;
			while (d_key(&s, &f2, &w2)) {
				if (f2 == 1 && w2 == 1) {
					uid_lo = d_fixed64(&s);
				} else if (f2 == 2 && w2 == 1) {
					uid_hi = d_fixed64(&s);
				} else {
					d_skip(&s, w2);
				}
			}
		} else if (f == 2 && w == 2) {
			pb_t b = d_sub(&r);
			uint32_t msg_id = 0;
			uint64_t ticks = 0;
			pb_t inner = {NULL, NULL};
			uint32_t f2, w2;
			while (d_key(&b, &f2, &w2)) {
				if (f2 == 1 && w2 == 5) {
					msg_id = d_fixed32(&b);
				} else if (f2 == 2 && w2 == 1) {
					ticks = d_fixed64(&b);
				} else if (f2 == 3 && w2 == 2) {
					inner = d_sub(&b);
				} else {
					d_skip(&b, w2);
				}
			}
			if (inner.p != NULL) {
				samples += handle_body(ssc, uid_lo, uid_hi, msg_id, SSC_TICKS_TO_NS(ticks), inner,
				                       fn, ud);
			}
		} else {
			d_skip(&r, w);
		}
	}

	return samples;
}


/*
 *
 * QRTR discovery and 'exported' functions.
 *
 */

static int
discover_ssc(struct galaxyxr_ssc *ssc)
{
	struct sockaddr_qrtr sq;
	socklen_t sl = sizeof(sq);
	if (getsockname(ssc->fd, (struct sockaddr *)&sq, &sl) < 0) {
		return -errno;
	}

	struct qrtr_ctrl_pkt lk = {QRTR_TYPE_NEW_LOOKUP, QMI_SSC_SERVICE, 0, 0, 0};
	struct sockaddr_qrtr ctrl = {AF_QIPCRTR, sq.sq_node, QRTR_PORT_CTRL};
	if (sendto(ssc->fd, &lk, sizeof(lk), 0, (struct sockaddr *)&ctrl, sizeof(ctrl)) < 0) {
		return -errno;
	}

	for (;;) {
		struct pollfd pfd = {ssc->fd, POLLIN, 0};
		if (poll(&pfd, 1, 2000) <= 0) {
			break;
		}
		struct qrtr_ctrl_pkt pkt;
		ssize_t r = recvfrom(ssc->fd, &pkt, sizeof(pkt), 0, NULL, NULL);
		if (r < (ssize_t)sizeof(pkt)) {
			continue;
		}
		if (pkt.cmd != QRTR_TYPE_NEW_SERVER) {
			continue;
		}
		if (!pkt.service && !pkt.node && !pkt.port) {
			break;
		}
		if (pkt.service == QMI_SSC_SERVICE) {
			ssc->node = pkt.node;
			ssc->port = pkt.port;
			U_LOG_D("galaxyxr: SSC service found at QRTR node %u port %u", pkt.node, pkt.port);
			return 0;
		}
	}

	return -ENODEV;
}

int
galaxyxr_ssc_open(struct galaxyxr_ssc *ssc, float rate_hz)
{
	memset(ssc, 0, sizeof(*ssc));
	ssc->txn = 1;
	ssc->rate_hz = rate_hz;

	ssc->fd = socket(AF_QIPCRTR, SOCK_DGRAM, 0);
	if (ssc->fd < 0) {
		return -errno;
	}

	struct sockaddr_qrtr self;
	socklen_t sl = sizeof(self);
	getsockname(ssc->fd, (struct sockaddr *)&self, &sl);
	struct sockaddr_qrtr bind_addr = {AF_QIPCRTR, self.sq_node, 0};
	bind(ssc->fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr));

	int ret = discover_ssc(ssc);
	if (ret < 0) {
		close(ssc->fd);
		ssc->fd = -1;
		return ret;
	}

	ret = ssc_lookup(ssc, "accel");
	if (ret == 0) {
		ret = ssc_lookup(ssc, "gyro");
	}
	if (ret == 0) {
		ret = ssc_lookup(ssc, "gyro_cal");
	}
	if (ret == 0) {
		ret = ssc_lookup(ssc, "ipd");
	}
	if (ret == 0) {
		ret = ssc_lookup(ssc, "proximity");
	}
	if (ret < 0) {
		close(ssc->fd);
		ssc->fd = -1;
		return ret;
	}

	return 0;
}

static int
ssc_receive(struct galaxyxr_ssc *ssc, int flags, galaxyxr_ssc_sample_fn fn, void *ud)
{
	uint8_t buf[8192];
	ssize_t r = recvfrom(ssc->fd, buf, sizeof(buf), flags, NULL, NULL);
	if (r < 0) {
		return errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -errno;
	}
	if (r < 7 || buf[0] != QMI_TYPE_INDICATION) {
		return 0;
	}

	uint16_t msg_id = (uint16_t)(buf[3] | (buf[4] << 8));
	if (msg_id != SSC_IND_REPORT_SMALL && msg_id != SSC_IND_REPORT_LARGE) {
		return 0;
	}

	int samples = 0;
	size_t off = 7;
	while (off + 3 <= (size_t)r) {
		uint8_t t = buf[off];
		uint16_t l = (uint16_t)(buf[off + 1] | (buf[off + 2] << 8));
		size_t v = off + 3;
		if (v + l > (size_t)r) {
			break;
		}
		if (t == SSC_TLV_IND_DATA && l >= 2) {
			uint16_t plen = (uint16_t)(buf[v] | (buf[v + 1] << 8));
			pb_t resp = {buf + v + 2, buf + v + 2 + plen};
			if (resp.end > buf + r) {
				resp.end = buf + r;
			}
			samples += handle_response(ssc, resp, fn, ud);
		}
		off = v + l;
	}

	return samples;
}

int
galaxyxr_ssc_dispatch(struct galaxyxr_ssc *ssc, int timeout_ms, galaxyxr_ssc_sample_fn fn, void *ud)
{
	struct pollfd pfd = {ssc->fd, POLLIN, 0};
	int pr = poll(&pfd, 1, timeout_ms);
	if (pr < 0) {
		return errno == EINTR ? 0 : -errno;
	}
	if (pr == 0) {
		return 0;
	}

	return ssc_receive(ssc, 0, fn, ud);
}

int
galaxyxr_ssc_dispatch_ready(struct galaxyxr_ssc *ssc, galaxyxr_ssc_sample_fn fn, void *ud)
{
	return ssc_receive(ssc, MSG_DONTWAIT, fn, ud);
}

void
galaxyxr_ssc_close(struct galaxyxr_ssc *ssc)
{
	if (ssc->fd >= 0) {
		close(ssc->fd);
		ssc->fd = -1;
	}
}
