// Copyright 2023, Collabora, Ltd.
// Copyright 2023, Jarett Millard
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to @ref drv_pssense
 * @author Jarett Millard <jarett.millard@gmail.com>
 * @ingroup drv_pssense
 */

#pragma once

#include "xrt/xrt_frame.h"

#include "tracking/t_time_sync.h"

#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @defgroup drv_pssense PlayStation Sense driver
 * @ingroup drv
 *
 * @brief Driver for the PlayStation Sense motion controllers.
 */

#define PSSENSE_VID 0x054C
#define PSSENSE_PID_LEFT 0x0E45
#define PSSENSE_PID_RIGHT 0x0E46

/*!
 * Create a PlayStation Sense controller device.
 *
 * @param xp                   The prober creating the device.
 * @param xpdev                The prober device.
 * @param xfctx                The frame context.
 * @param[out] out_timing_sink Optional timing event sink output pointer.
 *
 * @ingroup drv_pssense
 */
struct xrt_device *
pssense_create(struct xrt_prober *xp,
               struct xrt_prober_device *xpdev,
               struct xrt_frame_context *xfctx,
               struct t_timing_event_sink **out_timing_sink);

/*!
 * @dir drivers/pssense
 *
 * @brief @ref drv_pssense files.
 */

#ifdef __cplusplus
}
#endif
