// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared refcounted VirtualDeviceState (see playspectra_state.h).
 * @ingroup drv_playspectra
 */

#include "playspectra_state.h"

#include "os/os_threading.h"
#include "util/u_misc.h"

#include <stdbool.h>

#define PS_HAPTIC_QUEUE 16

struct playspectra_state
{
	struct os_mutex mutex; // guards everything below
	int refcount;

	struct xrt_space_relation head;
	struct playspectra_ctrl ctrl[2]; // [PLAYSPECTRA_LEFT], [PLAYSPECTRA_RIGHT]

	// haptic イベントのリング(アプリ set_output → 制御チャネルが転送)。
	struct playspectra_haptic_event haptics[PS_HAPTIC_QUEUE];
	int haptic_head;
	int haptic_count;

	struct playspectra_control *control; // taken exactly once at teardown
	bool control_taken;
};

struct playspectra_state *
playspectra_state_create(void)
{
	struct playspectra_state *s = U_TYPED_CALLOC(struct playspectra_state);
	os_mutex_init(&s->mutex);
	s->refcount = 1;
	s->head = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;
	return s;
}

void
playspectra_state_ref(struct playspectra_state *s)
{
	os_mutex_lock(&s->mutex);
	s->refcount++;
	os_mutex_unlock(&s->mutex);
}

void
playspectra_state_unref(struct playspectra_state *s)
{
	if (s == NULL) {
		return;
	}
	os_mutex_lock(&s->mutex);
	int rc = --s->refcount;
	os_mutex_unlock(&s->mutex);
	if (rc == 0) {
		os_mutex_destroy(&s->mutex);
		free(s);
	}
}

void
playspectra_state_set_head(struct playspectra_state *s, const struct xrt_space_relation *rel)
{
	os_mutex_lock(&s->mutex);
	s->head = *rel;
	os_mutex_unlock(&s->mutex);
}

void
playspectra_state_get_head(struct playspectra_state *s, struct xrt_space_relation *out)
{
	os_mutex_lock(&s->mutex);
	*out = s->head;
	os_mutex_unlock(&s->mutex);
}

void
playspectra_state_set_ctrl(struct playspectra_state *s, enum playspectra_hand hand, const struct playspectra_ctrl *c)
{
	os_mutex_lock(&s->mutex);
	s->ctrl[hand] = *c;
	os_mutex_unlock(&s->mutex);
}

void
playspectra_state_get_ctrl(struct playspectra_state *s, enum playspectra_hand hand, struct playspectra_ctrl *out)
{
	os_mutex_lock(&s->mutex);
	*out = s->ctrl[hand];
	os_mutex_unlock(&s->mutex);
}

void
playspectra_state_push_haptic(struct playspectra_state *s, const struct playspectra_haptic_event *e)
{
	os_mutex_lock(&s->mutex);
	if (s->haptic_count == PS_HAPTIC_QUEUE) {
		// 満杯: 最古を捨てる。
		s->haptic_head = (s->haptic_head + 1) % PS_HAPTIC_QUEUE;
		s->haptic_count--;
	}
	int idx = (s->haptic_head + s->haptic_count) % PS_HAPTIC_QUEUE;
	s->haptics[idx] = *e;
	s->haptic_count++;
	os_mutex_unlock(&s->mutex);
}

bool
playspectra_state_pop_haptic(struct playspectra_state *s, struct playspectra_haptic_event *out)
{
	os_mutex_lock(&s->mutex);
	bool has = s->haptic_count > 0;
	if (has) {
		*out = s->haptics[s->haptic_head];
		s->haptic_head = (s->haptic_head + 1) % PS_HAPTIC_QUEUE;
		s->haptic_count--;
	}
	os_mutex_unlock(&s->mutex);
	return has;
}

void
playspectra_state_set_control(struct playspectra_state *s, struct playspectra_control *c)
{
	os_mutex_lock(&s->mutex);
	s->control = c;
	os_mutex_unlock(&s->mutex);
}

struct playspectra_control *
playspectra_state_take_control(struct playspectra_state *s)
{
	os_mutex_lock(&s->mutex);
	struct playspectra_control *c = NULL;
	if (!s->control_taken) {
		s->control_taken = true;
		c = s->control;
		s->control = NULL;
	}
	os_mutex_unlock(&s->mutex);
	return c;
}
