// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pure NDJSON protocol parsing (see playspectra_proto.h). cJSON only.
 * @ingroup drv_playspectra
 */

#include "playspectra_proto.h"

#include <string.h>
#include <stdio.h>

// obj[key] が長さ n の数値配列なら out[0..n) に格納して true。
static bool
get_num_array(const cJSON *obj, const char *key, double *out, int n)
{
	const cJSON *arr = cJSON_GetObjectItemCaseSensitive(obj, key);
	if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) != n) {
		return false;
	}
	for (int i = 0; i < n; i++) {
		const cJSON *e = cJSON_GetArrayItem(arr, i);
		if (!cJSON_IsNumber(e)) {
			return false;
		}
		out[i] = e->valuedouble;
	}
	return true;
}

static bool
get_flag(const cJSON *obj, const char *key, bool def)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
	if (v == NULL) {
		return def;
	}
	return cJSON_IsTrue(v);
}

enum playspectra_parse_status
playspectra_proto_parse_state(const cJSON *state, struct playspectra_set_state *out)
{
	memset(out, 0, sizeof(*out));

	if (!cJSON_IsObject(state)) {
		snprintf(out->error, sizeof(out->error), "missing:state");
		return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
	}

	const cJSON *seq = cJSON_GetObjectItemCaseSensitive(state, "sequence");
	if (cJSON_IsNumber(seq)) {
		out->has_sequence = true;
		out->sequence = (uint64_t)seq->valuedouble;
	}

	const cJSON *hmd = cJSON_GetObjectItemCaseSensitive(state, "hmd");
	if (cJSON_IsObject(hmd) && get_flag(hmd, "connected", true)) {
		const cJSON *head = cJSON_GetObjectItemCaseSensitive(hmd, "head");
		if (!cJSON_IsObject(head) || !get_num_array(head, "position", out->head.position, 3) ||
		    !get_num_array(head, "orientation", out->head.orientation, 4)) {
			snprintf(out->error, sizeof(out->error), "hmd.head.position/orientation");
			return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
		}
		const cJSON *rf = cJSON_GetObjectItemCaseSensitive(head, "relation_flags");
		out->head.present = true;
		out->head.position_valid = get_flag(rf, "position_valid", true);
		out->head.orientation_valid = get_flag(rf, "orientation_valid", true);
		out->head.position_tracked = get_flag(rf, "position_tracked", true);
		out->head.orientation_tracked = get_flag(rf, "orientation_tracked", true);
	}

	return PLAYSPECTRA_PARSE_OK;
}

bool
playspectra_proto_hello_version_ok(const cJSON *req, int expected_version)
{
	const cJSON *pv = cJSON_GetObjectItemCaseSensitive(req, "protocol_version");
	return cJSON_IsNumber(pv) && (int)pv->valuedouble == expected_version;
}
