// Copyright 2026, PlaySpectra.
// Based on simulated_controller.c (Copyright 2020-2023, Collabora, Ltd.)
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlaySpectra virtual Touch controller (left/right). Reads grip/aim pose and
 *         Touch inputs from the shared VirtualDeviceState (playspectra_state), which the
 *         control channel writes. Native oculus/touch profile.
 * @ingroup drv_playspectra
 */

#include "xrt/xrt_device.h"
#include "os/os_time.h"

#include "math/m_api.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "util/u_device.h"
#include "util/u_misc.h"
#include "util/u_var.h"
#include "util/u_debug.h"
#include "util/u_logging.h"
#include "xrt/xrt_results.h"

#include "playspectra_interface.h"
#include "playspectra_state.h"

#include <stdio.h>


struct playspectra_controller
{
	struct xrt_device base;
	enum u_logging_level log_level;
	struct playspectra_state *state; // 共有(ref を1つ保持)
	enum playspectra_hand hand;
};

static inline struct playspectra_controller *
playspectra_controller(struct xrt_device *xdev)
{
	return (struct playspectra_controller *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(playspectra_ctrl_log, "PLAYSPECTRA_LOG", U_LOGGING_WARN)

// Touch 入力名(左: x/y + menu / 右: a/b + system)。
static enum xrt_input_name left_inputs[] = {
    XRT_INPUT_TOUCH_X_CLICK,         XRT_INPUT_TOUCH_X_TOUCH,        XRT_INPUT_TOUCH_Y_CLICK,
    XRT_INPUT_TOUCH_Y_TOUCH,         XRT_INPUT_TOUCH_MENU_CLICK,     XRT_INPUT_TOUCH_SQUEEZE_VALUE,
    XRT_INPUT_TOUCH_TRIGGER_VALUE,   XRT_INPUT_TOUCH_TRIGGER_TOUCH,  XRT_INPUT_TOUCH_THUMBSTICK_CLICK,
    XRT_INPUT_TOUCH_THUMBSTICK_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK,    XRT_INPUT_TOUCH_THUMBREST_TOUCH,
    XRT_INPUT_TOUCH_GRIP_POSE,       XRT_INPUT_TOUCH_AIM_POSE,
};

static enum xrt_input_name right_inputs[] = {
    XRT_INPUT_TOUCH_A_CLICK,         XRT_INPUT_TOUCH_A_TOUCH,        XRT_INPUT_TOUCH_B_CLICK,
    XRT_INPUT_TOUCH_B_TOUCH,         XRT_INPUT_TOUCH_SYSTEM_CLICK,   XRT_INPUT_TOUCH_SQUEEZE_VALUE,
    XRT_INPUT_TOUCH_TRIGGER_VALUE,   XRT_INPUT_TOUCH_TRIGGER_TOUCH,  XRT_INPUT_TOUCH_THUMBSTICK_CLICK,
    XRT_INPUT_TOUCH_THUMBSTICK_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK,    XRT_INPUT_TOUCH_THUMBREST_TOUCH,
    XRT_INPUT_TOUCH_GRIP_POSE,       XRT_INPUT_TOUCH_AIM_POSE,
};

static enum xrt_output_name touch_outputs[] = {
    XRT_OUTPUT_NAME_TOUCH_HAPTIC,
};

static void
playspectra_controller_destroy(struct xrt_device *xdev)
{
	struct playspectra_controller *c = playspectra_controller(xdev);

	// どのデバイスも解放される前に制御チャネルを止める(最初の1回のみ非 NULL)。
	struct playspectra_control *ctl = playspectra_state_take_control(c->state);
	if (ctl != NULL) {
		playspectra_control_stop(ctl);
	}

	u_var_remove_root(c);
	playspectra_state_unref(c->state);
	u_device_free(&c->base);
}

static xrt_result_t
playspectra_controller_update_inputs(struct xrt_device *xdev)
{
	struct playspectra_controller *c = playspectra_controller(xdev);
	struct playspectra_ctrl st;
	playspectra_state_get_ctrl(c->state, c->hand, &st);

	uint64_t now = os_monotonic_get_ns();

	for (uint32_t i = 0; i < xdev->input_count; i++) {
		struct xrt_input *in = &xdev->inputs[i];
		in->timestamp = now;
		in->active = st.connected;
		U_ZERO(&in->value);
		if (!st.connected) {
			continue;
		}
		switch (in->name) {
		case XRT_INPUT_TOUCH_X_CLICK:
		case XRT_INPUT_TOUCH_A_CLICK: in->value.boolean = st.primary_click; break;
		case XRT_INPUT_TOUCH_X_TOUCH:
		case XRT_INPUT_TOUCH_A_TOUCH: in->value.boolean = st.primary_touch; break;
		case XRT_INPUT_TOUCH_Y_CLICK:
		case XRT_INPUT_TOUCH_B_CLICK: in->value.boolean = st.secondary_click; break;
		case XRT_INPUT_TOUCH_Y_TOUCH:
		case XRT_INPUT_TOUCH_B_TOUCH: in->value.boolean = st.secondary_touch; break;
		case XRT_INPUT_TOUCH_MENU_CLICK: in->value.boolean = st.menu_click; break;
		case XRT_INPUT_TOUCH_SYSTEM_CLICK: in->value.boolean = st.system_click; break;
		case XRT_INPUT_TOUCH_SQUEEZE_VALUE: in->value.vec1.x = st.squeeze; break;
		case XRT_INPUT_TOUCH_TRIGGER_VALUE: in->value.vec1.x = st.trigger; break;
		case XRT_INPUT_TOUCH_TRIGGER_TOUCH: in->value.boolean = st.trigger_touch; break;
		case XRT_INPUT_TOUCH_THUMBSTICK: in->value.vec2 = st.thumbstick; break;
		case XRT_INPUT_TOUCH_THUMBSTICK_CLICK: in->value.boolean = st.thumbstick_click; break;
		case XRT_INPUT_TOUCH_THUMBSTICK_TOUCH: in->value.boolean = st.thumbstick_touch; break;
		case XRT_INPUT_TOUCH_THUMBREST_TOUCH: in->value.boolean = st.thumbrest_touch; break;
		default: break; // pose inputs は get_tracked_pose で扱う
		}
	}
	return XRT_SUCCESS;
}

static xrt_result_t
playspectra_controller_get_tracked_pose(struct xrt_device *xdev,
                                        enum xrt_input_name name,
                                        int64_t at_timestamp_ns,
                                        struct xrt_space_relation *out_relation)
{
	struct playspectra_controller *c = playspectra_controller(xdev);

	if (name != XRT_INPUT_TOUCH_GRIP_POSE && name != XRT_INPUT_TOUCH_AIM_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct playspectra_ctrl st;
	playspectra_state_get_ctrl(c->state, c->hand, &st);

	*out_relation = (name == XRT_INPUT_TOUCH_AIM_POSE) ? st.aim : st.grip;
	if ((out_relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		math_quat_normalize(&out_relation->pose.orientation);
	}
	return XRT_SUCCESS;
}

// haptic 出力(アプリの xrApplyHapticFeedback)を共有 state のキューへ積む。
// 制御チャネルが接続中の observer/writer へ {"event":"haptics",...} として転送する。
static xrt_result_t
playspectra_controller_set_output(struct xrt_device *xdev,
                                  enum xrt_output_name name,
                                  const struct xrt_output_value *value)
{
	struct playspectra_controller *c = playspectra_controller(xdev);
	struct playspectra_haptic_event e = {
	    .hand = c->hand,
	    .frequency = value->vibration.frequency,
	    .amplitude = value->vibration.amplitude,
	    .duration_ns = value->vibration.duration_ns,
	};
	playspectra_state_push_haptic(c->state, &e);
	return XRT_SUCCESS;
}

struct xrt_device *
playspectra_controller_create(enum xrt_device_type type,
                              const struct xrt_pose *center,
                              struct xrt_tracking_origin *origin,
                              struct playspectra_state *state)
{
	const bool is_left = (type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER);
	enum xrt_input_name *inputs = is_left ? left_inputs : right_inputs;
	const uint32_t input_count = is_left ? ARRAY_SIZE(left_inputs) : ARRAY_SIZE(right_inputs);

	const enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
	struct playspectra_controller *c =
	    U_DEVICE_ALLOCATE(struct playspectra_controller, flags, input_count, ARRAY_SIZE(touch_outputs));

	c->state = state;
	playspectra_state_ref(state);
	c->hand = is_left ? PLAYSPECTRA_LEFT : PLAYSPECTRA_RIGHT;
	c->log_level = debug_get_log_option_playspectra_ctrl_log();

	u_device_populate_function_pointers(&c->base, playspectra_controller_get_tracked_pose,
	                                    playspectra_controller_destroy);
	c->base.update_inputs = playspectra_controller_update_inputs;
	c->base.set_output = playspectra_controller_set_output;
	c->base.tracking_origin = origin;
	c->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	c->base.device_type = type;
	c->base.supported.orientation_tracking = true;
	c->base.supported.position_tracking = true;

	snprintf(c->base.str, sizeof(c->base.str), "PlaySpectra %s Touch Controller", is_left ? "Left" : "Right");
	snprintf(c->base.serial, sizeof(c->base.serial), "PlaySpectra Touch %s S/N", is_left ? "L" : "R");

	for (uint32_t i = 0; i < input_count; i++) {
		c->base.inputs[i].active = true;
		c->base.inputs[i].name = inputs[i];
	}
	c->base.outputs[0].name = touch_outputs[0];

	// 初期状態を共有 state に書く(connected + center grip/aim, 入力ゼロ)。
	struct playspectra_ctrl st;
	U_ZERO(&st);
	st.connected = true;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose = (center != NULL) ? *center : (struct xrt_pose)XRT_POSE_IDENTITY;
	rel.relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	st.grip = rel;
	st.aim = rel;
	playspectra_state_set_ctrl(state, c->hand, &st);

	u_var_add_root(c, c->base.str, true);
	u_var_add_log_level(c, &c->log_level, "log_level");

	return &c->base;
}
