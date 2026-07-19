// Copyright 2026, PlaySpectra.
// Based on target_builder_simulated.c (Copyright 2022-2023, Collabora, Ltd.)
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlaySpectra virtual device builder. M2 は仮想 HMD のみ(コントローラは後続)。
 *         PLAYSPECTRA_ENABLE=1 で有効化する。
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_results.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "target_builder_helpers.h"
#include "target_builder_interface.h"

#include "playspectra/playspectra_interface.h"

#include <assert.h>


#ifndef XRT_BUILD_DRIVER_PLAYSPECTRA
#error "Must only be built with XRT_BUILD_DRIVER_PLAYSPECTRA set"
#endif

DEBUG_GET_ONCE_BOOL_OPTION(playspectra_enabled, "PLAYSPECTRA_ENABLE", false)


static const char *driver_list[] = {
    "playspectra",
};

static xrt_result_t
playspectra_estimate_system(struct xrt_builder *xb,
                            cJSON *config,
                            struct xrt_prober *xp,
                            struct xrt_builder_estimate *estimate)
{
	if (!debug_get_bool_option_playspectra_enabled()) {
		return XRT_SUCCESS;
	}

	// M2 は HMD のみ。head を確実に供給する。
	estimate->certain.head = true;
	// simulated(-50)より優先。有効化されているときだけ effective。
	estimate->priority = -40;

	return XRT_SUCCESS;
}

static xrt_result_t
playspectra_open_system_impl(struct xrt_builder *xb,
                             cJSON *config,
                             struct xrt_prober *xp,
                             struct xrt_tracking_origin *origin,
                             struct xrt_system_devices *xsysd,
                             struct xrt_frame_context *xfctx,
                             struct t_builder_roles_helper *tbrh)
{
	// STAGE 基準の初期頭部 pose(床 y=0、立位相当の目高さ 1.6m)。
	const struct xrt_pose head_center = {XRT_QUAT_IDENTITY, {0.0f, 1.6f, 0.0f}};

	struct xrt_device *head = playspectra_hmd_create(&head_center);
	if (head == NULL) {
		return XRT_ERROR_ALLOCATION;
	}

	head->supported.orientation_tracking = true;
	head->supported.position_tracking = true;
	head->tracking_origin->type = XRT_TRACKING_TYPE_OTHER;

	xsysd->static_xdevs[xsysd->static_xdev_count++] = head;

	tbrh->head = head;
	tbrh->left = NULL;
	tbrh->right = NULL;

	return XRT_SUCCESS;
}

static void
playspectra_destroy(struct xrt_builder *xb)
{
	free(xb);
}

struct xrt_builder *
t_builder_playspectra_create(void)
{
	struct t_builder *ub = U_TYPED_CALLOC(struct t_builder);

	ub->base.estimate_system = playspectra_estimate_system;
	ub->base.open_system = t_builder_open_system_static_roles;
	ub->base.destroy = playspectra_destroy;
	ub->base.identifier = "playspectra";
	ub->base.name = "PlaySpectra virtual device builder";
	ub->base.driver_identifiers = driver_list;
	ub->base.driver_identifier_count = ARRAY_SIZE(driver_list);
	ub->base.exclude_from_automatic_discovery = !debug_get_bool_option_playspectra_enabled();

	ub->open_system_static_roles = playspectra_open_system_impl;

	return &ub->base;
}
