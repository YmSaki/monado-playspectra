// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pure NDJSON protocol parsing for the PlaySpectra control channel
 *         (playspectra-device-core-spec.md §2/§5). cJSON only -- NO Monado deps,
 *         so it is unit-testable standalone (playspectra_proto_test.c).
 * @ingroup drv_playspectra
 */

#pragma once

#include "cjson/cJSON.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PLAYSPECTRA_PROTOCOL_VERSION 1
#define PLAYSPECTRA_MAX_INPUTS 32 // per controller

enum playspectra_parse_status
{
	PLAYSPECTRA_PARSE_OK = 0,
	PLAYSPECTRA_PARSE_VALIDATION_ERROR,
};

/*!
 * clock.mode (spec §4)。既定は realtime(latest-wins)。frame_synchronized は
 * logical_frame ごとに一度適用し、同一 frame 異内容を conflict とする。
 */
enum playspectra_clock_mode
{
	PLAYSPECTRA_CLOCK_REALTIME = 0, // 既定
	PLAYSPECTRA_CLOCK_FRAME_SYNCHRONIZED,
};

/*!
 * spec §2.1 PoseState を平文(double/bool)で表す。Monado 型に依存しない。
 */
struct playspectra_pose
{
	double position[3];
	double orientation[4]; // xyzw
	bool position_valid;
	bool orientation_valid;
	bool position_tracked;
	bool orientation_tracked;
};

/*!
 * HMD 更新(spec §2.2 hmd)。
 */
struct playspectra_head_update
{
	bool present; // hmd present && connected && head 有り
	struct playspectra_pose pose;
};

/*!
 * コントローラ入力の1エントリ(spec §2.4 semantic path)。suffix で型が決まる:
 * .../value|x|y → is_bool=false の num、.../click|touch → is_bool=true の flag。
 */
struct playspectra_input
{
	char path[48]; // 例: "/input/trigger/value", "/button/a/click"
	bool is_bool;
	double num;
	bool flag;
};

/*!
 * 片手コントローラ更新(spec §2.3)。grip/aim は独立。inputs は semantic-path 集合。
 */
struct playspectra_controller_update
{
	bool present; // connected && (grip|aim|inputs のいずれか有り)
	bool has_grip;
	bool has_aim;
	struct playspectra_pose grip;
	struct playspectra_pose aim;
	struct playspectra_input inputs[PLAYSPECTRA_MAX_INPUTS];
	int input_count;
};

/*!
 * set_state の "state" をパースした結果(spec §2.2)。
 */
struct playspectra_set_state
{
	bool has_sequence;
	uint64_t sequence;
	enum playspectra_clock_mode clock_mode; // 既定 realtime(memset 0)
	bool has_logical_frame;
	int64_t logical_frame;
	struct playspectra_head_update head;
	struct playspectra_controller_update left;
	struct playspectra_controller_update right;
	char error[96]; // status != OK のとき詳細
};

/*!
 * set_state の "state" オブジェクトをパースする(spec §2.1/§2.2/§2.3/§2.4)。
 * 完全性: present な device の pose は position(3)/orientation(4) 必須。欠落は VALIDATION_ERROR。
 * relation_flags は既定 true。純粋関数(cJSON のみ)。
 */
enum playspectra_parse_status
playspectra_proto_parse_state(const cJSON *state, struct playspectra_set_state *out);

/*!
 * hello の protocol_version が @p expected と一致するか。
 */
bool
playspectra_proto_hello_version_ok(const cJSON *req, int expected_version);

/*!
 * frame_synchronized の内容署名(spec §4)。envelope の sequence/clock は含めず、device 状態
 * (head + 両手の grip/aim/inputs)だけを FNV-1a でハッシュする。inputs は順序非依存。純粋関数。
 */
uint64_t
playspectra_proto_content_sig(const struct playspectra_set_state *s);

/*!
 * frame_synchronized の verdict(spec §4)。logical_frame と content_sig だけで決まる I/O 非依存の判定。
 */
enum playspectra_frame_verdict
{
	PLAYSPECTRA_FRAME_APPLY = 0,  // 新 frame(または最初) → 適用して記録
	PLAYSPECTRA_FRAME_IDEMPOTENT, // 同一 frame・同一内容 → 再適用しない冪等成功
	PLAYSPECTRA_FRAME_CONFLICT,   // 同一 frame・異内容 → conflict_error
	PLAYSPECTRA_FRAME_STALE,      // 古い frame → 破棄
};

/*!
 * frame_synchronized の verdict を返す(spec §4)。呼び出し側が verdict に応じて apply と応答 JSON を行う。
 */
enum playspectra_frame_verdict
playspectra_proto_frame_decide(bool has_frame, int64_t last_frame, uint64_t last_sig, int64_t lf, uint64_t sig);

#ifdef __cplusplus
}
#endif
