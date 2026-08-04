// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0

#pragma once

struct xrt_passthrough_stream;
struct m_relation_history;

/*!
 * Create the Galaxy XR EFS-calibrated titan-server provider. The stream starts
 * disabled: it connects to titan-server and runs the cameras only while
 * enabled, reconnecting in the background as needed.
 */
struct xrt_passthrough_stream *
galaxyxr_passthrough_create(const char *calibration_path, struct m_relation_history *qtimer_relation_history);
