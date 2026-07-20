// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Standalone unit test for playspectra_proto (cJSON only, no Monado deps).
 *
 * ローカル gcc + 同梱 cJSON で単体コンパイル・実行できる(Monado ビルド不要):
 *   gcc -I src/external/cjson -I src/xrt/drivers/playspectra \
 *       src/xrt/drivers/playspectra/playspectra_proto_test.c \
 *       src/xrt/drivers/playspectra/playspectra_proto.c \
 *       src/external/cjson/cjson/cJSON.c -o playspectra_proto_test && ./playspectra_proto_test
 *
 * @ingroup drv_playspectra
 */

#include "playspectra_proto.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int g_fail = 0;
static int g_total = 0;

#define CHECK(cond, msg)                                                                                                \
	do {                                                                                                           \
		g_total++;                                                                                             \
		if (!(cond)) {                                                                                          \
			g_fail++;                                                                                       \
			printf("  FAIL: %s\n", msg);                                                                    \
		}                                                                                                      \
	} while (0)

static bool
approx(double a, double b)
{
	return fabs(a - b) < 1e-9;
}

// state を JSON 文字列からパースするヘルパ(cJSON_Parse → parse_state)。
static enum playspectra_parse_status
parse_str(const char *json, struct playspectra_set_state *out)
{
	cJSON *root = cJSON_Parse(json);
	// root 自体が state オブジェクト(テスト簡略化のため state を直接渡す)。
	enum playspectra_parse_status st = playspectra_proto_parse_state(root, out);
	cJSON_Delete(root);
	return st;
}

int
main(void)
{
	struct playspectra_set_state s;

	// 1. 完全な set_state.state: hmd.head 有り。
	enum playspectra_parse_status st = parse_str(
	    "{\"sequence\":7,\"hmd\":{\"connected\":true,\"head\":{"
	    "\"position\":[1,2,3],\"orientation\":[0,0,0,1],"
	    "\"relation_flags\":{\"position_valid\":true,\"orientation_valid\":true,"
	    "\"position_tracked\":true,\"orientation_tracked\":true}}}}",
	    &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK, "1 valid parses OK");
	CHECK(s.has_sequence && s.sequence == 7, "1 sequence=7");
	CHECK(s.head.present, "1 head present");
	CHECK(approx(s.head.pose.position[0], 1) && approx(s.head.pose.position[1], 2) &&
	          approx(s.head.pose.position[2], 3),
	      "1 position");
	CHECK(approx(s.head.pose.orientation[3], 1), "1 orientation w=1");
	CHECK(s.head.pose.position_valid && s.head.pose.orientation_tracked, "1 flags true");

	// 2. relation_flags を明示 false。
	st = parse_str("{\"hmd\":{\"connected\":true,\"head\":{\"position\":[0,0,0],\"orientation\":[0,0,0,1],"
	               "\"relation_flags\":{\"position_tracked\":false}}}}",
	               &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK, "2 OK");
	CHECK(s.head.present && s.head.pose.position_tracked == false, "2 position_tracked=false");
	CHECK(s.head.pose.orientation_valid == true, "2 missing flag defaults true");

	// 3. hmd 無し → head.present=false, OK。
	st = parse_str("{\"sequence\":3}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK && !s.head.present, "3 no hmd -> present false");
	CHECK(s.has_sequence && s.sequence == 3, "3 sequence kept");

	// 4. hmd.connected=false → head.present=false。
	st = parse_str("{\"hmd\":{\"connected\":false}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK && !s.head.present, "4 disconnected -> present false");

	// 5. hmd present だが head.position 欠落 → VALIDATION_ERROR。
	st = parse_str("{\"hmd\":{\"connected\":true,\"head\":{\"orientation\":[0,0,0,1]}}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_VALIDATION_ERROR, "5 missing position -> validation error");
	CHECK(strlen(s.error) > 0, "5 error message set");

	// 6. position 長さ不正(2要素) → VALIDATION_ERROR。
	st = parse_str("{\"hmd\":{\"connected\":true,\"head\":{\"position\":[1,2],\"orientation\":[0,0,0,1]}}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_VALIDATION_ERROR, "6 bad array length -> validation error");

	// 7. sequence 無し → has_sequence=false。
	st = parse_str("{\"hmd\":{\"connected\":false}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK && !s.has_sequence, "7 no sequence -> has_sequence false");

	// 8. state が object でない(null) → VALIDATION_ERROR。
	st = parse_str("null", &s);
	CHECK(st == PLAYSPECTRA_PARSE_VALIDATION_ERROR, "8 non-object -> validation error");

	// 9. hello version.
	cJSON *hok = cJSON_Parse("{\"protocol_version\":1}");
	cJSON *hbad = cJSON_Parse("{\"protocol_version\":2}");
	cJSON *hmiss = cJSON_Parse("{}");
	CHECK(playspectra_proto_hello_version_ok(hok, 1), "9 version match");
	CHECK(!playspectra_proto_hello_version_ok(hbad, 1), "9 version mismatch");
	CHECK(!playspectra_proto_hello_version_ok(hmiss, 1), "9 missing version");
	cJSON_Delete(hok);
	cJSON_Delete(hbad);
	cJSON_Delete(hmiss);

	// 10. left controller: grip + aim + inputs (semantic path).
	st = parse_str("{\"left\":{\"connected\":true,"
	               "\"grip\":{\"position\":[-0.2,1.1,-0.1],\"orientation\":[0,0,0,1]},"
	               "\"aim\":{\"position\":[-0.2,1.1,-0.1],\"orientation\":[0,0,0,1]},"
	               "\"inputs\":{\"/input/trigger/value\":0.5,\"/button/x/click\":true}}}",
	               &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK, "10 OK");
	CHECK(s.left.present && s.left.has_grip && s.left.has_aim, "10 left present grip+aim");
	CHECK(approx(s.left.grip.position[0], -0.2) && approx(s.left.grip.position[1], 1.1), "10 grip pos");
	CHECK(s.left.input_count == 2, "10 two inputs");
	{
		int found = 0;
		for (int k = 0; k < s.left.input_count; k++) {
			if (strcmp(s.left.inputs[k].path, "/input/trigger/value") == 0) {
				found += (!s.left.inputs[k].is_bool && approx(s.left.inputs[k].num, 0.5));
			}
			if (strcmp(s.left.inputs[k].path, "/button/x/click") == 0) {
				found += (s.left.inputs[k].is_bool && s.left.inputs[k].flag);
			}
		}
		CHECK(found == 2, "10 input path/type/value");
	}
	CHECK(!s.right.present, "10 right absent");

	// 11. right connected=false -> present false.
	st = parse_str("{\"right\":{\"connected\":false}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK && !s.right.present, "11 right disconnected");

	// 12. malformed grip (position length 2) -> validation error.
	st = parse_str("{\"left\":{\"connected\":true,\"grip\":{\"position\":[1,2],\"orientation\":[0,0,0,1]}}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_VALIDATION_ERROR, "12 bad grip -> validation error");

	// 13. connected controller with only inputs (no grip/aim) is allowed.
	st = parse_str("{\"right\":{\"connected\":true,\"inputs\":{\"/input/squeeze/value\":1.0}}}", &s);
	CHECK(st == PLAYSPECTRA_PARSE_OK && s.right.present && !s.right.has_grip && s.right.input_count == 1,
	      "13 inputs-only controller");

	printf("\nplayspectra_proto_test: %d/%d passed\n", g_total - g_fail, g_total);
	return g_fail == 0 ? 0 : 1;
}
