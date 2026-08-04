// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Minimal Qualcomm SSC (sns_client QMI service over QRTR) IMU client.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum galaxyxr_ssc_sample_type
{
	GALAXYXR_SSC_ACCEL,
	GALAXYXR_SSC_GYRO,
	GALAXYXR_SSC_IPD,
	GALAXYXR_SSC_PROX,
	GALAXYXR_SSC_GYRO_CAL,
};

struct galaxyxr_ssc_sample
{
	enum galaxyxr_ssc_sample_type type;
	//! Accel/gyro: xyz. IPD: per-side lens travel in micrometers, x=L y=R.
	//! Proximity: x is the state (1 near, 2 far), y the raw ADC count.
	//! Gyro cal: the SSC's current estimate of the total gyro bias, rad/s.
	float v[3];
	//! Device timestamp, qtimer ticks converted to nanoseconds.
	int64_t timestamp_ns;
};

typedef void (*galaxyxr_ssc_sample_fn)(void *ud, const struct galaxyxr_ssc_sample *sample);

struct galaxyxr_ssc_suid
{
	uint64_t lo, hi;
	bool valid;
};

struct galaxyxr_ssc_pending
{
	uint64_t lo, hi;
	uint8_t type;
	bool done;
};

struct galaxyxr_ssc
{
	int fd;
	uint32_t node, port;
	uint16_t txn;
	float rate_hz;

	//! Streams enabled for hardware id 0, one per sensor type. The online
	//! gyro_cal algo is the instance whose rigid_body is 0 (no hw_id attr).
	struct galaxyxr_ssc_suid accel, gyro, ipd, prox, gyro_cal;

	//! Discovered instances waiting for their attributes.
	struct galaxyxr_ssc_pending pending[32];
	uint32_t pending_count;
};

/*!
 * Connect to the SSC, request accel + gyro streams at @p rate_hz and the
 * on-change ipd (the motorized lens position) and proximity (wear detection)
 * sensors.
 *
 * Returns 0 on success, negative errno style value on failure.
 */
int
galaxyxr_ssc_open(struct galaxyxr_ssc *ssc, float rate_hz);

/*!
 * Wait up to @p timeout_ms and process incoming messages, calling @p fn for
 * each decoded accel/gyro sample. Returns number of samples delivered or
 * negative on socket error.
 */
int
galaxyxr_ssc_dispatch(struct galaxyxr_ssc *ssc, int timeout_ms, galaxyxr_ssc_sample_fn fn, void *ud);

/*!
 * Process one incoming message after @p ssc's fd has been reported readable.
 * This call never waits. Returns number of samples delivered or negative on
 * socket error.
 */
int
galaxyxr_ssc_dispatch_ready(struct galaxyxr_ssc *ssc, galaxyxr_ssc_sample_fn fn, void *ud);

void
galaxyxr_ssc_close(struct galaxyxr_ssc *ssc);

#ifdef __cplusplus
}
#endif
