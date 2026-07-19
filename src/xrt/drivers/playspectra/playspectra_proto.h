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

enum playspectra_parse_status
{
	PLAYSPECTRA_PARSE_OK = 0,
	PLAYSPECTRA_PARSE_VALIDATION_ERROR,
};

/*!
 * spec §2.2 hmd.head を平文(double/bool)で表す。Monado 型に依存しない。
 */
struct playspectra_head_update
{
	bool present; // hmd present && connected && head 有り
	double position[3];
	double orientation[4]; // xyzw
	bool position_valid;
	bool orientation_valid;
	bool position_tracked;
	bool orientation_tracked;
};

/*!
 * set_state の "state" をパースした結果。
 */
struct playspectra_set_state
{
	bool has_sequence;
	uint64_t sequence;
	struct playspectra_head_update head;
	char error[96]; // status != OK のとき詳細
};

/*!
 * set_state の "state" オブジェクトをパースする(spec §2.1/§2.2)。
 * 完全性: hmd.head が present なら position(3)/orientation(4) 必須。欠落は VALIDATION_ERROR。
 * relation_flags は既定 true(spec の relation_flags)。純粋関数(cJSON のみ)。
 */
enum playspectra_parse_status
playspectra_proto_parse_state(const cJSON *state, struct playspectra_set_state *out);

/*!
 * hello の protocol_version が @p expected と一致するか。
 */
bool
playspectra_proto_hello_version_ok(const cJSON *req, int expected_version);

#ifdef __cplusplus
}
#endif
