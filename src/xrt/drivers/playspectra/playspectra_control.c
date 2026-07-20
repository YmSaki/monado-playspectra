// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PlaySpectra NDJSON control channel (spec playspectra-device-core-spec.md §5).
 *
 * 1接続を順次 accept して処理する(writer 排他の最小形。observer 複数は後続)。
 * hello / set_state / get_state / status を扱い、set_state の hmd.head と left/right
 * コントローラを共有 VirtualDeviceState(playspectra_state)へ書き込む(各デバイスがそこから読む)。
 * socket のクロスプラットフォームラップは remote ドライバ(r_hub.c)と同型。
 *
 * @ingroup drv_playspectra
 */

#include "xrt/xrt_defines.h"
#include "os/os_threading.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_json.h"

#include "playspectra_interface.h"
#include "playspectra_proto.h"
#include "playspectra_state.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET ps_socket_t;
#define PS_INVALID_SOCKET INVALID_SOCKET
#define ps_close_socket closesocket
#else
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int ps_socket_t;
#define PS_INVALID_SOCKET (-1)
#define ps_close_socket close
#endif

#define PS_PROTOCOL_VERSION PLAYSPECTRA_PROTOCOL_VERSION
#define PS_LINE_MAX (1024 * 1024) // spec §5.1: 最大 1 MiB

struct playspectra_control
{
	struct playspectra_state *state; // 共有 VirtualDeviceState(ref を1つ保持)
	struct os_thread_helper oth;
	ps_socket_t listen_sock;
	uint16_t port;
	uint64_t sequence; // 最後に適用した sequence(latest-wins)
	enum u_logging_level log_level;
};

#define CTL_ERR(c, ...) U_LOG_IFL_E((c)->log_level, __VA_ARGS__)
#define CTL_INFO(c, ...) U_LOG_IFL_I((c)->log_level, __VA_ARGS__)


/*
 * JSON send helpers (パースは playspectra_proto.c に分離)。
 */

// 1行を送る(末尾に \n)。cJSON は呼び出し側で delete する。
static void
send_json_line(ps_socket_t s, cJSON *obj)
{
	char *str = cJSON_PrintUnformatted(obj);
	if (str == NULL) {
		return;
	}
	size_t len = strlen(str);
	(void)send(s, str, (int)len, 0);
	(void)send(s, "\n", 1, 0);
	cJSON_free(str);
}

static void
reply_error(ps_socket_t s, const cJSON *req, const char *error_type, const char *error)
{
	cJSON *r = cJSON_CreateObject();
	const cJSON *rid = cJSON_GetObjectItemCaseSensitive(req, "request_id");
	if (cJSON_IsString(rid)) {
		cJSON_AddStringToObject(r, "request_id", rid->valuestring);
	}
	cJSON_AddBoolToObject(r, "ok", false);
	cJSON_AddStringToObject(r, "error_type", error_type);
	cJSON_AddStringToObject(r, "error", error);
	send_json_line(s, r);
	cJSON_Delete(r);
}

static void
reply_ok(ps_socket_t s, const cJSON *req, cJSON *extra /*owned*/)
{
	cJSON *r = extra != NULL ? extra : cJSON_CreateObject();
	const cJSON *rid = cJSON_GetObjectItemCaseSensitive(req, "request_id");
	if (cJSON_IsString(rid)) {
		cJSON_AddStringToObject(r, "request_id", rid->valuestring);
	}
	cJSON_AddBoolToObject(r, "ok", true);
	send_json_line(s, r);
	cJSON_Delete(r);
}


/*
 * Command handlers.
 */

static void
handle_hello(struct playspectra_control *c, ps_socket_t s, const cJSON *req)
{
	if (!playspectra_proto_hello_version_ok(req, PLAYSPECTRA_PROTOCOL_VERSION)) {
		reply_error(s, req, "protocol_error", "protocol_version");
		return;
	}
	// 最小 descriptor(HMD のみ)。fov/解像度の実値は spec §7 で後日。
	cJSON *r = cJSON_CreateObject();
	cJSON_AddNumberToObject(r, "protocol_version", PS_PROTOCOL_VERSION);
	cJSON_AddStringToObject(r, "role_granted", "writer");
	cJSON *desc = cJSON_AddObjectToObject(r, "descriptor");
	cJSON_AddNumberToObject(desc, "protocol_version", PS_PROTOCOL_VERSION);
	cJSON *hmd = cJSON_AddObjectToObject(desc, "hmd");
	cJSON_AddNumberToObject(hmd, "recommended_eye_width", 1080);
	cJSON_AddNumberToObject(hmd, "recommended_eye_height", 1200);
	cJSON_AddNumberToObject(hmd, "refresh_hz", 90);
	reply_ok(s, req, r);
}

// playspectra_pose(平文) → xrt_space_relation。
static struct xrt_space_relation
pose_to_relation(const struct playspectra_pose *p)
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose.position.x = (float)p->position[0];
	rel.pose.position.y = (float)p->position[1];
	rel.pose.position.z = (float)p->position[2];
	rel.pose.orientation.x = (float)p->orientation[0];
	rel.pose.orientation.y = (float)p->orientation[1];
	rel.pose.orientation.z = (float)p->orientation[2];
	rel.pose.orientation.w = (float)p->orientation[3];
	int f = 0;
	if (p->position_valid) {
		f |= XRT_SPACE_RELATION_POSITION_VALID_BIT;
	}
	if (p->orientation_valid) {
		f |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	}
	if (p->position_tracked) {
		f |= XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
	if (p->orientation_tracked) {
		f |= XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
	}
	rel.relation_flags = (enum xrt_space_relation_flags)f;
	return rel;
}

// parsed controller(semantic path) → 共有 state。present のときだけ適用(full snapshot)。
// ボタンは左右統一のため x/a→primary, y/b→secondary に写像する。
static void
apply_ctrl(struct playspectra_control *c, enum playspectra_hand hand, const struct playspectra_controller_update *u)
{
	if (!u->present) {
		return;
	}
	struct playspectra_ctrl st;
	U_ZERO(&st);
	st.connected = true;
	if (u->has_grip) {
		st.grip = pose_to_relation(&u->grip);
	}
	if (u->has_aim) {
		st.aim = pose_to_relation(&u->aim);
	}
	for (int i = 0; i < u->input_count; i++) {
		const char *p = u->inputs[i].path;
		double n = u->inputs[i].num;
		bool b = u->inputs[i].flag;
		if (strcmp(p, "/input/trigger/value") == 0) {
			st.trigger = (float)n;
		} else if (strcmp(p, "/input/squeeze/value") == 0) {
			st.squeeze = (float)n;
		} else if (strcmp(p, "/input/thumbstick/x") == 0) {
			st.thumbstick.x = (float)n;
		} else if (strcmp(p, "/input/thumbstick/y") == 0) {
			st.thumbstick.y = (float)n;
		} else if (strcmp(p, "/input/trigger/touch") == 0) {
			st.trigger_touch = b;
		} else if (strcmp(p, "/input/thumbstick/click") == 0) {
			st.thumbstick_click = b;
		} else if (strcmp(p, "/input/thumbstick/touch") == 0) {
			st.thumbstick_touch = b;
		} else if (strcmp(p, "/input/thumbrest/touch") == 0) {
			st.thumbrest_touch = b;
		} else if (strcmp(p, "/button/x/click") == 0 || strcmp(p, "/button/a/click") == 0) {
			st.primary_click = b;
		} else if (strcmp(p, "/button/x/touch") == 0 || strcmp(p, "/button/a/touch") == 0) {
			st.primary_touch = b;
		} else if (strcmp(p, "/button/y/click") == 0 || strcmp(p, "/button/b/click") == 0) {
			st.secondary_click = b;
		} else if (strcmp(p, "/button/y/touch") == 0 || strcmp(p, "/button/b/touch") == 0) {
			st.secondary_touch = b;
		} else if (strcmp(p, "/input/menu/click") == 0) {
			st.menu_click = b;
		} else if (strcmp(p, "/input/system/click") == 0) {
			st.system_click = b;
		}
	}
	playspectra_state_set_ctrl(c->state, hand, &st);
}

// set_state の hmd.head + left/right を共有 state へ適用。spec §2.2/§2.3。
static void
handle_set_state(struct playspectra_control *c, ps_socket_t s, const cJSON *req)
{
	const cJSON *state = cJSON_GetObjectItemCaseSensitive(req, "state");

	struct playspectra_set_state parsed;
	if (playspectra_proto_parse_state(state, &parsed) != PLAYSPECTRA_PARSE_OK) {
		reply_error(s, req, "validation_error", parsed.error);
		return;
	}

	// latest-wins(spec §4 realtime)。古い sequence は無視。
	uint64_t sequence = parsed.has_sequence ? parsed.sequence : c->sequence + 1;
	if (parsed.has_sequence && sequence <= c->sequence && c->sequence != 0) {
		cJSON *r = cJSON_CreateObject();
		cJSON_AddBoolToObject(r, "applied", false);
		cJSON_AddStringToObject(r, "reason", "stale_frame");
		reply_ok(s, req, r);
		return;
	}

	if (parsed.head.present) {
		struct xrt_space_relation rel = pose_to_relation(&parsed.head.pose);
		playspectra_state_set_head(c->state, &rel);
	}
	apply_ctrl(c, PLAYSPECTRA_LEFT, &parsed.left);
	apply_ctrl(c, PLAYSPECTRA_RIGHT, &parsed.right);

	c->sequence = sequence;
	cJSON *r = cJSON_CreateObject();
	cJSON_AddBoolToObject(r, "applied", true);
	cJSON_AddNumberToObject(r, "sequence", (double)sequence);
	reply_ok(s, req, r);
}

static void
handle_status(struct playspectra_control *c, ps_socket_t s, const cJSON *req)
{
	cJSON *r = cJSON_CreateObject();
	cJSON_AddBoolToObject(r, "writer_connected", true);
	cJSON_AddNumberToObject(r, "sequence", (double)c->sequence);
	cJSON_AddStringToObject(r, "runtime", "monado");
	reply_ok(s, req, r);
}

static void
dispatch_line(struct playspectra_control *c, ps_socket_t s, char *line)
{
	cJSON *req = cJSON_Parse(line);
	if (req == NULL) {
		cJSON *r = cJSON_CreateObject();
		cJSON_AddBoolToObject(r, "ok", false);
		cJSON_AddStringToObject(r, "error_type", "protocol_error");
		cJSON_AddStringToObject(r, "error", "invalid_json");
		send_json_line(s, r);
		cJSON_Delete(r);
		return;
	}
	const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(req, "cmd");
	if (cJSON_IsString(cmd)) {
		if (strcmp(cmd->valuestring, "hello") == 0) {
			handle_hello(c, s, req);
		} else if (strcmp(cmd->valuestring, "set_state") == 0) {
			handle_set_state(c, s, req);
		} else if (strcmp(cmd->valuestring, "status") == 0) {
			handle_status(c, s, req);
		} else {
			reply_error(s, req, "protocol_error", "unknown_cmd");
		}
	} else {
		reply_error(s, req, "protocol_error", "missing_cmd");
	}
	cJSON_Delete(req);
}

// select で短いタイムアウト付きに readable を待つ。1=readable / 0=stop 要求 / -1=error。
// blocking accept()/recv() は Linux では close() で確実に解除されない(Monado/WSL で実測)。
// stop フラグを 200ms 以内に検知して抜けることで、移植的にクリーン停止できる。
static int
wait_readable(ps_socket_t s, struct os_thread_helper *oth)
{
	while (os_thread_helper_is_running(oth)) {
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(s, &rfds);
		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 200 * 1000; // 200 ms
		int r = select((int)s + 1, &rfds, NULL, NULL, &tv);
		if (r < 0) {
			return -1; // socket closed / error
		}
		if (r > 0) {
			return 1; // readable
		}
		// r == 0: timeout -> is_running を再確認
	}
	return 0; // stop 要求
}

// 1接続を NDJSON で処理(切断または stop まで)。
static void
serve_connection(struct playspectra_control *c, ps_socket_t conn)
{
	char *buf = malloc(PS_LINE_MAX);
	if (buf == NULL) {
		return;
	}
	size_t used = 0;
	while (os_thread_helper_is_running(&c->oth)) {
		if (wait_readable(conn, &c->oth) != 1) {
			break; // stop 要求 or 切断
		}
		char chunk[4096];
		int n = (int)recv(conn, chunk, sizeof(chunk), 0);
		if (n <= 0) {
			break; // 切断
		}
		for (int i = 0; i < n; i++) {
			char ch = chunk[i];
			if (ch == '\n') {
				buf[used] = '\0';
				if (used > 0) {
					dispatch_line(c, conn, buf);
				}
				used = 0;
			} else if (used + 1 < PS_LINE_MAX) {
				buf[used++] = ch;
			} else {
				used = 0; // 1 MiB 超過 → 破棄(spec §5.1)
			}
		}
	}
	free(buf);
}

static void *
control_thread(void *ptr)
{
	struct playspectra_control *c = (struct playspectra_control *)ptr;
	while (os_thread_helper_is_running(&c->oth)) {
		if (wait_readable(c->listen_sock, &c->oth) != 1) {
			break; // stop 要求 or listen socket error
		}
		ps_socket_t conn = accept(c->listen_sock, NULL, NULL);
		if (conn == PS_INVALID_SOCKET) {
			continue;
		}
		CTL_INFO(c, "PlaySpectra control: writer connected");
		serve_connection(c, conn);
		ps_close_socket(conn);
		CTL_INFO(c, "PlaySpectra control: writer disconnected (state held)");
	}
	return NULL;
}


/*
 * 'Exported' functions.
 */

struct playspectra_control *
playspectra_control_start(struct playspectra_state *state, uint16_t port)
{
	if (port == 0) {
		const char *env = getenv("PLAYSPECTRA_MONADO_PORT");
		port = env != NULL ? (uint16_t)atoi(env) : PLAYSPECTRA_DEFAULT_PORT;
	}

#ifdef _WIN32
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		return NULL;
	}
#endif

	ps_socket_t ls = socket(AF_INET, SOCK_STREAM, 0);
	if (ls == PS_INVALID_SOCKET) {
		return NULL;
	}
	int yes = 1;
	(void)setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 のみ
	addr.sin_port = htons(port);
	if (bind(ls, (struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(ls, 1) != 0) {
		ps_close_socket(ls);
		return NULL;
	}

	struct playspectra_control *c = U_TYPED_CALLOC(struct playspectra_control);
	c->state = state;
	playspectra_state_ref(state);
	c->listen_sock = ls;
	c->port = port;
	c->sequence = 0;
	c->log_level = U_LOGGING_WARN;

	os_thread_helper_init(&c->oth);
	if (os_thread_helper_start(&c->oth, control_thread, c) != 0) {
		ps_close_socket(ls);
		os_thread_helper_destroy(&c->oth);
		free(c);
		return NULL;
	}
	U_LOG_I("PlaySpectra control channel listening on 127.0.0.1:%u", (unsigned)port);
	return c;
}

void
playspectra_control_stop(struct playspectra_control *c)
{
	if (c == NULL) {
		return;
	}
	// stop フラグを立ててスレッド join。wait_readable が 200ms 以内に is_running=false を
	// 検知して抜けるので、close() で accept を叩き起こす必要はない(Linux では不確実なため)。
	os_thread_helper_stop_and_wait(&c->oth);
	os_thread_helper_destroy(&c->oth);
	if (c->listen_sock != PS_INVALID_SOCKET) {
		ps_close_socket(c->listen_sock);
	}
#ifdef _WIN32
	WSACleanup();
#endif
	playspectra_state_unref(c->state);
	free(c);
}
