// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the Samsung Galaxy XR driver.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_device;
struct xrt_builder;

/*!
 * @defgroup drv_galaxyxr Samsung Galaxy XR driver
 * @ingroup drv
 *
 * @brief 3DoF driver for the Samsung Galaxy XR (SM-I610) running Linux,
 *        reading accel/gyro from the Qualcomm SSC over QRTR.
 */

/*!
 * True when running on a Galaxy XR (checks the device-tree model).
 *
 * @ingroup drv_galaxyxr
 */
bool
galaxyxr_is_present(void);

/*!
 * Create the Galaxy XR HMD device.
 *
 * @ingroup drv_galaxyxr
 */
struct xrt_device *
galaxyxr_hmd_create(void);

/*!
 * Builder for the Galaxy XR system.
 *
 * @ingroup drv_galaxyxr
 */
struct xrt_builder *
galaxyxr_builder_create(void);

#ifdef __cplusplus
}
#endif
