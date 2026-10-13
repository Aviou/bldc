/*
	Copyright 2026 Aviou

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mcsa_core.h"

#include <string.h>

#define TWO_PI_F				6.28318530717958647692f
#define ANGLE_UNITS_PER_RAD		(65536.0f / TWO_PI_F)
#define IDLE_WINDOW				30000

static int32_t round_to_i32(float x) {
	if (!(x == x)) {
		return 0;
	}
	if (x > 2.0e9f) {
		return 2000000000;
	}
	if (x < -2.0e9f) {
		return -2000000000;
	}
	return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

// Division rounded half away from zero, d > 0
static int32_t rdiv32(int32_t n, int32_t d) {
	return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

static int64_t rdiv64(int64_t n, int64_t d) {
	return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

static int64_t floor_div8(int64_t v) {
	return v >= 0 ? v / 8 : -((-v + 7) / 8);
}

static int16_t sat16(mcsa_core *c, int64_t v) {
	if (v > 32767) {
		c->saturated = c->saturated + 1;
		return 32767;
	}
	if (v < -32768) {
		c->saturated = c->saturated + 1;
		return -32768;
	}
	return (int16_t)v;
}

// CIC integrators and combs in modular arithmetic. Integrator overflow is
// well defined and cancels in the combs as long as the comb result fits.
static void cic32_push(uint32_t *integ, uint32_t x) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		integ[s] += x;
		x = integ[s];
	}
}

static int32_t cic32_pull(const uint32_t *integ, uint32_t *comb) {
	uint32_t v = integ[MCSA_CIC_ORDER - 1];
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		uint32_t prev = comb[s];
		comb[s] = v;
		v -= prev;
	}
	return (int32_t)v;
}

static void cic64_push(uint64_t *integ, uint64_t x) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		integ[s] += x;
		x = integ[s];
	}
}

static int64_t cic64_pull(const uint64_t *integ, uint64_t *comb) {
	uint64_t v = integ[MCSA_CIC_ORDER - 1];
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		uint64_t prev = comb[s];
		comb[s] = v;
		v -= prev;
	}
	return (int64_t)v;
}

static uint8_t channel_count(uint16_t mask) {
	uint8_t n = 0;
	for (uint8_t ch = 0; ch < MCSA_CH_NUM; ch++) {
		if (mask & (1u << ch)) {
			n++;
		}
	}
	return n;
}

uint16_t mcsa_crc16(const uint8_t *buf, uint32_t len) {
	uint16_t crc = 0;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)buf[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
		}
	}

	return crc;
}

void mcsa_core_init(mcsa_core *c, volatile int16_t *buf, uint32_t buf_bytes) {
	memset((void *)c, 0, sizeof(*c));
	c->buf = buf;
	c->buf_words = buf_bytes / 2;
	c->sp_valid = true;
	c->state = MCSA_STATE_IDLE;
	c->angle_pos = MCSA_ANGLE_NONE;
}

void mcsa_core_update_angle(mcsa_core *c, float phase) {
	// The comparison also rejects NaN
	if (!(phase > -1000.0f && phase < 1000.0f)) {
		return;
	}

	uint16_t frac = (uint16_t)(int32_t)(phase * ANGLE_UNITS_PER_RAD);

	if (c->theta_valid) {
		c->theta += (int16_t)(uint16_t)(frac - c->frac_last);
	} else {
		c->theta = frac;
		c->theta_valid = true;
	}

	c->frac_last = frac;
}

uint32_t mcsa_core_capacity(const mcsa_core *c, uint16_t mask) {
	uint8_t n = channel_count(mask & MCSA_MASK_ALL);
	return n > 0 ? c->buf_words / n : 0;
}

mcsa_status mcsa_core_start(mcsa_core *c, const mcsa_env *env, uint8_t mode,
		uint16_t mask, uint8_t R, uint32_t length, uint16_t token) {
	if (mode != MCSA_MODE_ONESHOT && mode != MCSA_MODE_CONTINUOUS) {
		return MCSA_BAD_ARG;
	}
	if (mask == 0 || (mask & (uint16_t)~MCSA_MASK_ALL)) {
		return MCSA_BAD_ARG;
	}
	if (R < 1 || R > MCSA_R_MAX) {
		return MCSA_BAD_ARG;
	}

	uint32_t cap = mcsa_core_capacity(c, mask);
	if (cap == 0) {
		return MCSA_BAD_ARG;
	}

	if (mode == MCSA_MODE_ONESHOT) {
		if (length == 0) {
			length = cap;
		}
		if (length > cap) {
			return MCSA_BAD_ARG;
		}
	} else {
		length = 0;
	}

	// Repeated START after a lost answer: keep the capture
	if (token != 0 && token == c->last_token &&
			(c->state == MCSA_STATE_RUNNING || c->state == MCSA_STATE_DONE) &&
			mode == c->mode && mask == c->mask && R == c->R && length == c->length) {
		return MCSA_OK;
	}

	if (c->state == MCSA_STATE_RUNNING) {
		return MCSA_BUSY;
	}
	if (!env->foc_active) {
		return MCSA_NOT_FOC;
	}
	if (env->buffer_free && !env->buffer_free()) {
		return MCSA_BUSY;
	}

	// The ISR ignores the core while it is configured
	c->state = MCSA_STATE_IDLE;
	c->sp_valid = false;

	c->mode = mode;
	c->mask = mask;
	c->nch = 0;
	c->angle_pos = MCSA_ANGLE_NONE;
	for (uint8_t ch = 0; ch < MCSA_CH_NUM; ch++) {
		if (mask & (1u << ch)) {
			if (ch == MCSA_CH_ANGLE) {
				c->angle_pos = c->nch;
			}
			c->ch_list[c->nch++] = ch;
		}
	}
	c->R = R;
	c->r3 = (uint32_t)R * R * R;
	c->length = length;
	c->capacity = cap;
	c->f_isr = env->f_isr;
	c->fac_current = env->fac_current;
	c->q3_per_amp = env->fac_current > 0.0f ? 8.0f / env->fac_current : 0.0f;

	memset(c->cic32_int, 0, sizeof(c->cic32_int));
	memset(c->cic32_comb, 0, sizeof(c->cic32_comb));
	memset(c->cic64_int, 0, sizeof(c->cic64_int));
	memset(c->cic64_comb, 0, sizeof(c->cic64_comb));
	memset(c->flags_win, 0, sizeof(c->flags_win));
	c->in_count = 0;
	c->out_count = 0;
	c->last_angle_q13 = 0;
	c->n_written = 0;
	c->saturated = 0;
	c->isr_max_capture = 0.0f;
	c->stop_reason = MCSA_STOP_NONE;
	c->session++;
	c->last_token = token;
	c->need_init = true;

	__sync_synchronize();
	c->state = MCSA_STATE_RUNNING;

	return MCSA_OK;
}

void mcsa_core_stop(mcsa_core *c, mcsa_stop_reason reason) {
	if (c->state == MCSA_STATE_RUNNING) {
		c->stop_reason = (uint8_t)reason;
		c->state = MCSA_STATE_DONE;
	}
}

void mcsa_core_invalidate(mcsa_core *c, mcsa_stop_reason reason) {
	if (c->state != MCSA_STATE_IDLE) {
		c->stop_reason = (uint8_t)reason;
		c->state = MCSA_STATE_INVALID;
	}
}

bool mcsa_core_sample_print_request(mcsa_core *c, mcsa_sp_request req) {
	if (c->state == MCSA_STATE_RUNNING) {
		return false;
	}

	switch (req) {
	case MCSA_SP_OFF:
		return true;

	case MCSA_SP_SEND_LAST:
		return c->sp_valid;

	default:
		break;
	}

	mcsa_core_invalidate(c, MCSA_STOP_SAMPLE_PRINT);
	c->sp_valid = true;
	return true;
}

void mcsa_core_sample(mcsa_core *c, const mcsa_sample_in *in) {
	if (c->state != MCSA_STATE_RUNNING) {
		return;
	}

	c->sampled_this_isr = true;

	if (c->need_init) {
		for (int k = 0; k < 3; k++) {
			int32_t q3 = round_to_i32(in->offset[k] * 8.0f);
			int32_t oi = q3 >= 0 ? q3 / 8 : -((-q3 + 7) / 8);
			c->off_frozen[k] = in->offset[k];
			c->off_int[k] = oi;
			c->off_frac_q3[k] = q3 - 8 * oi;
		}
		c->theta_start = c->theta;
		c->need_init = false;
	}

	for (int k = 0; k < 3; k++) {
		if (in->offset[k] != c->off_frozen[k]) {
			c->stop_reason = MCSA_STOP_OFFSETS_CHANGED;
			c->state = MCSA_STATE_DONE;
			return;
		}
	}

	const uint16_t mask = c->mask;

	// Flags of this cycle, combined over the current decimation window
	uint16_t fl = 0;
	for (int k = 0; k < 3; k++) {
		if (in->arr > MCSA_HIGH_MOD_MARGIN && in->ccr[k] > in->arr - MCSA_HIGH_MOD_MARGIN) {
			fl |= (uint16_t)(1u << k);
		}
	}
	if (c->in_count == 0) {
		c->flags_win[0] = (uint16_t)(fl | MCSA_FLAG_RUNNING);
	} else {
		c->flags_win[0] |= fl;
	}
	if (!in->running) {
		c->flags_win[0] &= (uint16_t)~MCSA_FLAG_RUNNING;
	}

	// Integrators
	for (int k = 0; k < 3; k++) {
		if (mask & (1u << k)) {
			int32_t x = round_to_i32(in->adc[k]) - c->off_int[k];
			cic32_push(c->cic32_int[k], (uint32_t)x);
		}
	}
	if (mask & (1u << MCSA_CH_ID)) {
		cic64_push(c->cic64_int[0], (uint64_t)(int64_t)round_to_i32(in->id * c->q3_per_amp));
	}
	if (mask & (1u << MCSA_CH_IQ)) {
		cic64_push(c->cic64_int[1], (uint64_t)(int64_t)round_to_i32(in->iq * c->q3_per_amp));
	}
	if (mask & (1u << MCSA_CH_VD)) {
		cic64_push(c->cic64_int[2], (uint64_t)(int64_t)round_to_i32(in->vd * 1000.0f));
	}
	if (mask & (1u << MCSA_CH_VQ)) {
		cic64_push(c->cic64_int[3], (uint64_t)(int64_t)round_to_i32(in->vq * 1000.0f));
	}
	if (mask & (1u << MCSA_CH_ANGLE)) {
		cic64_push(c->cic64_int[4], (uint64_t)(c->theta - c->theta_start));
	}

	if (++c->in_count < c->R) {
		return;
	}
	c->in_count = 0;

	// Combs run for every output, also for the dropped ones
	int32_t y32[3] = {0, 0, 0};
	int64_t y64[5] = {0, 0, 0, 0, 0};
	for (int k = 0; k < 3; k++) {
		if (mask & (1u << k)) {
			y32[k] = cic32_pull(c->cic32_int[k], c->cic32_comb[k]);
		}
	}
	for (int j = 0; j < 5; j++) {
		if (mask & (1u << (MCSA_CH_ID + j))) {
			y64[j] = cic64_pull(c->cic64_int[j], c->cic64_comb[j]);
		}
	}

	// The filter spans the last MCSA_CIC_ORDER windows
	uint16_t hm = 0;
	uint16_t run = MCSA_FLAG_RUNNING;
	for (int w = 0; w < MCSA_CIC_ORDER; w++) {
		hm |= c->flags_win[w] & (MCSA_FLAG_HIGH_MOD_A | MCSA_FLAG_HIGH_MOD_B | MCSA_FLAG_HIGH_MOD_C);
		run &= c->flags_win[w];
	}
	for (int w = MCSA_CIC_ORDER - 1; w > 0; w--) {
		c->flags_win[w] = c->flags_win[w - 1];
	}

	if (++c->out_count <= MCSA_DROP_OUTPUTS) {
		return;
	}

	int16_t val[MCSA_CH_NUM];
	memset(val, 0, sizeof(val));
	const int32_t r3 = (int32_t)c->r3;

	for (int k = 0; k < 3; k++) {
		if (mask & (1u << k)) {
			int32_t q = y32[k] / r3;
			int32_t rem = y32[k] % r3;
			val[k] = sat16(c, (int64_t)8 * q + rdiv32(8 * rem, r3) - c->off_frac_q3[k]);
		}
	}
	for (int j = 0; j < 4; j++) {
		if (mask & (1u << (MCSA_CH_ID + j))) {
			val[MCSA_CH_ID + j] = sat16(c, rdiv64(y64[j], r3));
		}
	}
	if (mask & (1u << MCSA_CH_ANGLE)) {
		int64_t rel = rdiv64(y64[4], r3);
		if (rel >= MCSA_ANGLE_LIMIT || rel <= -MCSA_ANGLE_LIMIT) {
			c->stop_reason = MCSA_STOP_ANGLE_LIMIT;
			c->state = MCSA_STATE_DONE;
			return;
		}
		int64_t q13 = floor_div8(c->theta_start + rel);
		val[MCSA_CH_ANGLE] = (int16_t)(uint16_t)((uint64_t)q13 & 0xFFFFu);
		c->last_angle_q13 = q13;
	}
	val[MCSA_CH_FLAGS] = (int16_t)(uint16_t)(hm | run);

	volatile int16_t *rec = c->buf + (c->n_written % c->capacity) * c->nch;
	for (uint8_t i = 0; i < c->nch; i++) {
		rec[i] = val[c->ch_list[i]];
	}
	c->n_written = c->n_written + 1;

	if (c->mode == MCSA_MODE_ONESHOT && c->n_written >= c->length) {
		c->stop_reason = MCSA_STOP_LENGTH;
		c->state = MCSA_STATE_DONE;
	} else if (c->n_written >= MCSA_MAX_RECORDS) {
		c->stop_reason = MCSA_STOP_INDEX_LIMIT;
		c->state = MCSA_STATE_DONE;
	}
}

void mcsa_core_isr_end(mcsa_core *c, float isr_duration) {
	if (c->sampled_this_isr) {
		c->sampled_this_isr = false;
		if (isr_duration > c->isr_max_capture) {
			c->isr_max_capture = isr_duration;
		}
	} else {
		if (isr_duration > c->isr_max_idle_win) {
			c->isr_max_idle_win = isr_duration;
		}
		if (++c->isr_idle_cnt >= IDLE_WINDOW) {
			c->isr_max_idle = c->isr_max_idle_win;
			c->isr_max_idle_win = 0.0f;
			c->isr_idle_cnt = 0;
		}
	}
}
