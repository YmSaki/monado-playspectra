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
#include "playspectra/playspectra_state.h"

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

	// head + 左右 Touch コントローラを確実に供給する。
	estimate->certain.head = true;
	estimate->certain.left = true;
	estimate->certain.right = true;
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
	// STAGE 基準の初期 pose(床 y=0、立位相当)。
	const struct xrt_pose head_center = {XRT_QUAT_IDENTITY, {0.0f, 1.6f, 0.0f}};
	const struct xrt_pose left_center = {XRT_QUAT_IDENTITY, {-0.2f, 1.3f, -0.5f}};
	const struct xrt_pose right_center = {XRT_QUAT_IDENTITY, {0.2f, 1.3f, -0.5f}};

	// 共有 VirtualDeviceState(制御チャネルが書き、各デバイスが読む)。
	struct playspectra_state *state = playspectra_state_create();

	struct xrt_device *head = playspectra_hmd_create(&head_center, state);
	if (head == NULL) {
		playspectra_state_unref(state);
		return XRT_ERROR_ALLOCATION;
	}
	head->supported.orientation_tracking = true;
	head->supported.position_tracking = true;
	head->tracking_origin->type = XRT_TRACKING_TYPE_OTHER;

	struct xrt_device *left = playspectra_controller_create(XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER, &left_center,
	                                                        head->tracking_origin, state);
	struct xrt_device *right = playspectra_controller_create(XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER, &right_center,
	                                                         head->tracking_origin, state);

	// 制御チャネルは共有 state に書く。最初に破棄されるデバイスが1度だけ停止する。
	struct playspectra_control *control = playspectra_control_start(state, 0);
	playspectra_state_set_control(state, control);
	playspectra_state_unref(state); // builder の生成 ref を手放す(以降は各 device[+control] が保持)

	xsysd->static_xdevs[xsysd->static_xdev_count++] = head;
	if (left != NULL) {
		xsysd->static_xdevs[xsysd->static_xdev_count++] = left;
	}
	if (right != NULL) {
		xsysd->static_xdevs[xsysd->static_xdev_count++] = right;
	}

	tbrh->head = head;
	tbrh->left = left;
	tbrh->right = right;

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
