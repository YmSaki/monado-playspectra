// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared, refcounted, mutex-protected VirtualDeviceState for the PlaySpectra
 *         driver. The control channel WRITES it; the virtual devices (hmd, and later
 *         left/right controllers) READ it. Decoupling via this shared struct avoids
 *         device-lifetime hazards: the control thread never touches xrt_device objects
 *         directly, so Monado's device destroy order cannot race a live control thread.
 * @ingroup drv_playspectra
 */

#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

struct playspectra_control;

/*!
 * Shared device state. Refcounted: created by the builder, referenced by each device
 * and by the control channel; freed when the last reference drops.
 */
struct playspectra_state;

/*!
 * Create the shared state with refcount = 1 (the creator's reference).
 */
struct playspectra_state *
playspectra_state_create(void);

void
playspectra_state_ref(struct playspectra_state *s);

//! Drop one reference; frees the state (and its mutex) when the count reaches 0.
void
playspectra_state_unref(struct playspectra_state *s);

/*!
 * HMD head pose in STAGE space. Thread-safe.
 */
void
playspectra_state_set_head(struct playspectra_state *s, const struct xrt_space_relation *rel);

void
playspectra_state_get_head(struct playspectra_state *s, struct xrt_space_relation *out);

/*!
 * Store the control-channel handle so the first device to be destroyed can stop it.
 */
void
playspectra_state_set_control(struct playspectra_state *s, struct playspectra_control *c);

/*!
 * Return the control-channel handle exactly once (to the first caller), NULL after.
 * The caller is responsible for stopping it. Used so exactly one device tears the
 * control channel down before any device object is freed.
 */
struct playspectra_control *
playspectra_state_take_control(struct playspectra_state *s);

#ifdef __cplusplus
}
#endif
