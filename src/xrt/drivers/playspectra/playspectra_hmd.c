// Copyright 2026, PlaySpectra.
// Based largely on sample_hmd.c / simulated_hmd.c (Copyright 2020-2024, Collabora, Ltd.)
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlaySpectra virtual HMD. Reads its head pose from the shared
 *         VirtualDeviceState (playspectra_state), which the control channel writes.
 *         STAGE-based (床 y=0). No built-in movement.
 * @ingroup drv_playspectra
 */

#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"
#include "xrt/xrt_results.h"

#include "playspectra_interface.h"
#include "playspectra_state.h"

#include <stdio.h>


/*!
 * @implements xrt_device
 */
struct playspectra_hmd
{
	struct xrt_device base;

	enum u_logging_level log_level;

	// 共有 VirtualDeviceState(制御チャネルが書き、ここは読むだけ)。ref を1つ保持。
	struct playspectra_state *state;
};

static inline struct playspectra_hmd *
playspectra_hmd(struct xrt_device *xdev)
{
	return (struct playspectra_hmd *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(playspectra_log, "PLAYSPECTRA_LOG", U_LOGGING_WARN)

#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
playspectra_hmd_destroy(struct xrt_device *xdev)
{
	struct playspectra_hmd *hmd = playspectra_hmd(xdev);

	// 最初に破棄されるデバイスが制御チャネルを止める(どのデバイスも解放される前に、
	// 制御スレッドが確実に停止する)。take_control は1回だけ非 NULL を返す。
	struct playspectra_control *ctl = playspectra_state_take_control(hmd->state);
	if (ctl != NULL) {
		playspectra_control_stop(ctl);
	}

	u_var_remove_root(hmd);
	playspectra_state_unref(hmd->state);
	u_device_free(&hmd->base);
}

static xrt_result_t
playspectra_hmd_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

static xrt_result_t
playspectra_hmd_get_tracked_pose(struct xrt_device *xdev,
                                 enum xrt_input_name name,
                                 int64_t at_timestamp_ns,
                                 struct xrt_space_relation *out_relation)
{
	struct playspectra_hmd *hmd = playspectra_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	playspectra_state_get_head(hmd->state, &relation);

	if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		math_quat_normalize(&relation.pose.orientation);
	}

	*out_relation = relation;
	return XRT_SUCCESS;
}

static xrt_result_t
playspectra_hmd_get_view_poses(struct xrt_device *xdev,
                               const struct xrt_vec3 *default_eye_relation,
                               int64_t at_timestamp_ns,
                               enum xrt_view_type view_type,
                               uint32_t view_count,
                               struct xrt_space_relation *out_head_relation,
                               struct xrt_fov *out_fovs,
                               struct xrt_pose *out_poses)
{
	return u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_type, view_count,
	                               out_head_relation, out_fovs, out_poses);
}

static xrt_result_t
playspectra_hmd_get_visibility_mask(struct xrt_device *xdev,
                                    enum xrt_visibility_mask_type type,
                                    uint32_t view_index,
                                    struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

struct xrt_device *
playspectra_hmd_create(const struct xrt_pose *center, struct playspectra_state *state)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct playspectra_hmd *hmd = U_DEVICE_ALLOCATE(struct playspectra_hmd, flags, 1, 0);

	hmd->state = state;
	playspectra_state_ref(state);

	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	hmd->base.update_inputs = playspectra_hmd_update_inputs;
	hmd->base.get_tracked_pose = playspectra_hmd_get_tracked_pose;
	hmd->base.get_view_poses = playspectra_hmd_get_view_poses;
	hmd->base.get_visibility_mask = playspectra_hmd_get_visibility_mask;
	hmd->base.destroy = playspectra_hmd_destroy;

	u_distortion_mesh_set_none(&hmd->base);

	hmd->log_level = debug_get_log_option_playspectra_log();

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "PlaySpectra HMD");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "PlaySpectra HMD S/N");

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;

	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 90.0f);

	const double hFOV = 90 * (M_PI / 180.0);
	const double vFOV = 96.73 * (M_PI / 180.0);
	const double hCOP = 0.529;
	const double vCOP = 0.5;
	if (!math_compute_fovs(1, hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[1]) ||
	    !math_compute_fovs(1, 1.0 - hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[0])) {
		HMD_ERROR(hmd, "Failed to setup basic device info");
		playspectra_hmd_destroy(&hmd->base);
		return NULL;
	}

	const int panel_w = 1080;
	const int panel_h = 1200;
	hmd->base.hmd->screens[0].w_pixels = panel_w * 2;
	hmd->base.hmd->screens[0].h_pixels = panel_h;

	for (uint8_t eye = 0; eye < 2; ++eye) {
		hmd->base.hmd->views[eye].display.w_pixels = panel_w;
		hmd->base.hmd->views[eye].display.h_pixels = panel_h;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = panel_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = panel_h;
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
	}
	hmd->base.hmd->views[0].viewport.x_pixels = 0;
	hmd->base.hmd->views[1].viewport.x_pixels = panel_w;

	u_distortion_mesh_set_none(&hmd->base);

	// 初期 head pose: center(STAGE)。orientation identity 前提で valid+tracked。共有 state に書く。
	struct xrt_space_relation init = XRT_SPACE_RELATION_ZERO;
	init.pose = (center != NULL) ? *center : (struct xrt_pose)XRT_POSE_IDENTITY;
	init.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	playspectra_state_set_head(state, &init);

	u_var_add_root(hmd, "PlaySpectra HMD", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	return &hmd->base;
}
