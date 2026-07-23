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

// PoseState をパース(spec §2.1)。position(3)/orientation(4) 必須。true=成功。
static bool
parse_pose(const cJSON *o, struct playspectra_pose *p)
{
	if (!cJSON_IsObject(o) || !get_num_array(o, "position", p->position, 3) ||
	    !get_num_array(o, "orientation", p->orientation, 4)) {
		return false;
	}
	const cJSON *rf = cJSON_GetObjectItemCaseSensitive(o, "relation_flags");
	p->position_valid = get_flag(rf, "position_valid", true);
	p->orientation_valid = get_flag(rf, "orientation_valid", true);
	p->position_tracked = get_flag(rf, "position_tracked", true);
	p->orientation_tracked = get_flag(rf, "orientation_tracked", true);
	return true;
}

// controller の "inputs" オブジェクトを semantic-path 集合として取り込む(spec §2.4)。
static void
parse_inputs(const cJSON *inputs, struct playspectra_controller_update *c)
{
	if (!cJSON_IsObject(inputs)) {
		return;
	}
	const cJSON *item = NULL;
	cJSON_ArrayForEach(item, inputs)
	{
		if (c->input_count >= PLAYSPECTRA_MAX_INPUTS || item->string == NULL) {
			break;
		}
		struct playspectra_input *e = &c->inputs[c->input_count];
		snprintf(e->path, sizeof(e->path), "%s", item->string);
		if (cJSON_IsBool(item)) {
			e->is_bool = true;
			e->flag = cJSON_IsTrue(item);
			e->num = 0.0;
		} else if (cJSON_IsNumber(item)) {
			e->is_bool = false;
			e->num = item->valuedouble;
			e->flag = false;
		} else {
			continue; // 未知型は無視(未知フィールド許容)
		}
		c->input_count++;
	}
}

// 片手コントローラをパース。connected なら grip/aim(有れば必須妥当)/inputs を取り込む。
static bool
parse_controller(const cJSON *hand, struct playspectra_controller_update *c, char *err, size_t errlen, const char *name)
{
	memset(c, 0, sizeof(*c));
	if (!cJSON_IsObject(hand) || !get_flag(hand, "connected", true)) {
		return true; // 非接続 or 欠落 → present=false
	}
	c->present = true;

	const cJSON *grip = cJSON_GetObjectItemCaseSensitive(hand, "grip");
	if (grip != NULL) {
		if (!parse_pose(grip, &c->grip)) {
			snprintf(err, errlen, "%s.grip", name);
			return false;
		}
		c->has_grip = true;
	}
	const cJSON *aim = cJSON_GetObjectItemCaseSensitive(hand, "aim");
	if (aim != NULL) {
		if (!parse_pose(aim, &c->aim)) {
			snprintf(err, errlen, "%s.aim", name);
			return false;
		}
		c->has_aim = true;
	}
	parse_inputs(cJSON_GetObjectItemCaseSensitive(hand, "inputs"), c);
	return true;
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

	// clock (spec §4)。mode 既定 realtime。frame_synchronized は logical_frame 必須。
	const cJSON *clock = cJSON_GetObjectItemCaseSensitive(state, "clock");
	if (cJSON_IsObject(clock)) {
		const cJSON *mode = cJSON_GetObjectItemCaseSensitive(clock, "mode");
		if (cJSON_IsString(mode) && strcmp(mode->valuestring, "frame_synchronized") == 0) {
			out->clock_mode = PLAYSPECTRA_CLOCK_FRAME_SYNCHRONIZED;
		}
		const cJSON *lf = cJSON_GetObjectItemCaseSensitive(clock, "logical_frame");
		if (cJSON_IsNumber(lf)) {
			out->has_logical_frame = true;
			out->logical_frame = (int64_t)lf->valuedouble;
		}
	}
	if (out->clock_mode == PLAYSPECTRA_CLOCK_FRAME_SYNCHRONIZED && !out->has_logical_frame) {
		snprintf(out->error, sizeof(out->error), "missing:clock.logical_frame");
		return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
	}

	const cJSON *hmd = cJSON_GetObjectItemCaseSensitive(state, "hmd");
	if (cJSON_IsObject(hmd) && get_flag(hmd, "connected", true)) {
		const cJSON *head = cJSON_GetObjectItemCaseSensitive(hmd, "head");
		if (head == NULL || !parse_pose(head, &out->head.pose)) {
			snprintf(out->error, sizeof(out->error), "hmd.head.position/orientation");
			return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
		}
		out->head.present = true;
	}

	if (!parse_controller(cJSON_GetObjectItemCaseSensitive(state, "left"), &out->left, out->error,
	                      sizeof(out->error), "left")) {
		return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
	}
	if (!parse_controller(cJSON_GetObjectItemCaseSensitive(state, "right"), &out->right, out->error,
	                      sizeof(out->error), "right")) {
		return PLAYSPECTRA_PARSE_VALIDATION_ERROR;
	}

	return PLAYSPECTRA_PARSE_OK;
}

bool
playspectra_proto_hello_version_ok(const cJSON *req, int expected_version)
{
	const cJSON *pv = cJSON_GetObjectItemCaseSensitive(req, "protocol_version");
	return cJSON_IsNumber(pv) && (int)pv->valuedouble == expected_version;
}

/*
 * frame_synchronized の重複判定用「内容署名」(spec §4)。envelope の sequence/clock は
 * 含めず、device 状態(head + 両手の grip/aim/inputs)だけを FNV-1a でハッシュする。
 * inputs は順序非依存(各エントリのハッシュを XOR)にして、順序違いの同一内容を conflict にしない。
 */
static void
fnv1a(uint64_t *h, const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	for (size_t i = 0; i < len; i++) {
		*h ^= p[i];
		*h *= 1099511628211ULL;
	}
}

static void
pose_sig(uint64_t *h, const struct playspectra_pose *p)
{
	fnv1a(h, p->position, sizeof(p->position));
	fnv1a(h, p->orientation, sizeof(p->orientation));
	uint8_t flags = (uint8_t)((p->position_valid ? 1 : 0) | (p->orientation_valid ? 2 : 0) |
	                          (p->position_tracked ? 4 : 0) | (p->orientation_tracked ? 8 : 0));
	fnv1a(h, &flags, sizeof(flags));
}

static void
ctrl_sig(uint64_t *h, const struct playspectra_controller_update *c)
{
	uint8_t present = (uint8_t)(c->present ? 1 : 0);
	fnv1a(h, &present, sizeof(present));
	if (!c->present) {
		return;
	}
	uint8_t g = (uint8_t)(c->has_grip ? 1 : 0);
	uint8_t a = (uint8_t)(c->has_aim ? 1 : 0);
	fnv1a(h, &g, sizeof(g));
	if (c->has_grip) {
		pose_sig(h, &c->grip);
	}
	fnv1a(h, &a, sizeof(a));
	if (c->has_aim) {
		pose_sig(h, &c->aim);
	}
	uint64_t inacc = 0; // inputs は順序非依存
	for (int i = 0; i < c->input_count; i++) {
		uint64_t ih = 1469598103934665603ULL;
		fnv1a(&ih, c->inputs[i].path, strlen(c->inputs[i].path));
		uint8_t b = (uint8_t)(c->inputs[i].is_bool ? 1 : 0);
		fnv1a(&ih, &b, sizeof(b));
		fnv1a(&ih, &c->inputs[i].num, sizeof(c->inputs[i].num));
		uint8_t f = (uint8_t)(c->inputs[i].flag ? 1 : 0);
		fnv1a(&ih, &f, sizeof(f));
		inacc ^= ih;
	}
	fnv1a(h, &inacc, sizeof(inacc));
}

uint64_t
playspectra_proto_content_sig(const struct playspectra_set_state *s)
{
	uint64_t h = 1469598103934665603ULL; // FNV-1a offset basis
	uint8_t hp = (uint8_t)(s->head.present ? 1 : 0);
	fnv1a(&h, &hp, sizeof(hp));
	if (s->head.present) {
		pose_sig(&h, &s->head.pose);
	}
	ctrl_sig(&h, &s->left);
	ctrl_sig(&h, &s->right);
	return h;
}

enum playspectra_frame_verdict
playspectra_proto_frame_decide(bool has_frame, int64_t last_frame, uint64_t last_sig, int64_t lf, uint64_t sig)
{
	if (has_frame && lf < last_frame) {
		return PLAYSPECTRA_FRAME_STALE; // 既に次の frame を適用済み → 遅延到着は破棄
	}
	if (has_frame && lf == last_frame) {
		// 同一 frame: 内容が一致すれば冪等、異なれば conflict。
		return (sig == last_sig) ? PLAYSPECTRA_FRAME_IDEMPOTENT : PLAYSPECTRA_FRAME_CONFLICT;
	}
	return PLAYSPECTRA_FRAME_APPLY; // 新しい frame(または最初)
}
