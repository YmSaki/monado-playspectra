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
 * どちらの手か。共有 state のコントローラ配列の添字。
 */
enum playspectra_hand
{
	PLAYSPECTRA_LEFT = 0,
	PLAYSPECTRA_RIGHT = 1,
};

/*!
 * 片手コントローラの完全状態(Touch 相当)。set_state は full snapshot なので毎回まるごと差し替える。
 * ボタンは左右を統一するため primary(=x/a) / secondary(=y/b) で持つ。
 */
struct playspectra_ctrl
{
	bool connected;
	struct xrt_space_relation grip;
	struct xrt_space_relation aim;

	float trigger;              // /input/trigger/value
	float squeeze;              // /input/squeeze/value
	struct xrt_vec2 thumbstick; // /input/thumbstick (x,y)

	bool trigger_touch;
	bool thumbstick_click;
	bool thumbstick_touch;
	bool primary_click;   // x(左)/a(右)
	bool primary_touch;
	bool secondary_click; // y(左)/b(右)
	bool secondary_touch;
	bool menu_click;      // 左
	bool system_click;    // 右
	bool thumbrest_touch;
};

/*!
 * コントローラ状態の getter/setter。@p hand は enum playspectra_hand。thread-safe。
 */
void
playspectra_state_set_ctrl(struct playspectra_state *s, enum playspectra_hand hand, const struct playspectra_ctrl *c);

void
playspectra_state_get_ctrl(struct playspectra_state *s, enum playspectra_hand hand, struct playspectra_ctrl *out);

/*!
 * haptic 出力イベント(アプリ → デバイス)。制御チャネルが observer/writer へ転送する。
 */
struct playspectra_haptic_event
{
	enum playspectra_hand hand;
	float frequency;
	float amplitude;
	int64_t duration_ns;
};

/*!
 * haptic イベントをキューに積む(デバイスの set_output から)。満杯なら最古を捨てる。thread-safe。
 */
void
playspectra_state_push_haptic(struct playspectra_state *s, const struct playspectra_haptic_event *e);

/*!
 * haptic イベントを1つ取り出す(制御チャネルが送出)。無ければ false。thread-safe。
 */
bool
playspectra_state_pop_haptic(struct playspectra_state *s, struct playspectra_haptic_event *out);

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
