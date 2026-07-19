// Copyright 2026, PlaySpectra.
// Based largely on sample_hmd.c / simulated_hmd.c (Copyright 2020-2024, Collabora, Ltd.)
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlaySpectra virtual HMD. Pose is fed externally via the control channel
 *         (playspectra_hmd_set_pose); no built-in movement. STAGE-based (床 y=0).
 * @ingroup drv_playspectra
 */

#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
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

#include <stdio.h>


/*!
 * @implements xrt_device
 */
struct playspectra_hmd
{
	struct xrt_device base;

	enum u_logging_level log_level;

	// built-in mutex, thread safe -- set_pose (control thread) と get_tracked_pose (app) が触る
	struct m_relation_history *relation_hist;

	// NDJSON 制御チャネル(このデバイスの寿命に紐づく)。生成失敗時 NULL。
	struct playspectra_control *control;
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
	playspectra_control_stop(hmd->control);
	hmd->control = NULL;
	u_var_remove_root(hmd);
	m_relation_history_destroy(&hmd->relation_hist);
	u_device_free(&hmd->base);
}

static xrt_result_t
playspectra_hmd_update_inputs(struct xrt_device *xdev)
{
	// Pose は control channel が push する。ここは no-op。
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
	(void)m_relation_history_get(hmd->relation_hist, at_timestamp_ns, &relation);

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

void
playspectra_hmd_set_pose(struct xrt_device *xdev,
                         const struct xrt_pose *pose,
                         bool position_valid,
                         bool orientation_valid,
                         bool position_tracked,
                         bool orientation_tracked)
{
	struct playspectra_hmd *hmd = playspectra_hmd(xdev);

	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = *pose;

	int flags = 0;
	if (position_valid) {
		flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
	}
	if (orientation_valid) {
		flags |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	}
	if (position_tracked) {
		flags |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	if (orientation_tracked) {
		flags |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	}
	relation.relation_flags = (enum xrt_space_relation_flags)flags;

	// spec §4: 適用時刻は now。predicted_display_time との相関は M2 で計測(spec §7)。
	m_relation_history_push(hmd->relation_hist, &relation, os_monotonic_get_ns());
}

struct xrt_device *
playspectra_hmd_create(const struct xrt_pose *center)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct playspectra_hmd *hmd = U_DEVICE_ALLOCATE(struct playspectra_hmd, flags, 1, 0);

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

	m_relation_history_create(&hmd->relation_hist);

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;

	// 表示諸元。実値は spec §7 のとおり後で Descriptor 側で確定。sample 相当の既定。
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

	// 初期 pose: center(STAGE)。orientation は identity 前提で valid+tracked。
	struct xrt_pose initial = XRT_POSE_IDENTITY;
	if (center != NULL) {
		initial = *center;
	}
	playspectra_hmd_set_pose(&hmd->base, &initial, true, true, true, true);

	u_var_add_root(hmd, "PlaySpectra HMD", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	// 制御チャネルを開始(M2.3)。失敗しても pose は center 固定でデバイスは動く。
	hmd->control = playspectra_control_start(&hmd->base, 0);

	return &hmd->base;
}
