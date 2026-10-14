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

static inline __attribute__((always_inline)) int32_t round_to_i32(float x) {
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
static inline __attribute__((always_inline)) int32_t rdiv32(int32_t n, int32_t d) {
	return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

// 64 by 32 bit division for d <= 2^18 in 14-bit steps: the partial remainder stays
// below 2^18, so every step fits a 32-bit hardware division.
static inline __attribute__((always_inline)) uint64_t udiv64_small(uint64_t n, uint32_t d) {
	uint32_t r = (uint32_t)(n >> 56);
	uint64_t q = r / d;
	r -= (uint32_t)q * d;

	for (int shift = 42; shift >= 0; shift -= 14) {
		uint32_t cur = (r << 14) | ((uint32_t)(n >> shift) & 0x3FFFu);
		uint32_t qd = cur / d;
		r = cur - qd * d;
		q = (q << 14) | qd;
	}

	return q;
}

int64_t mcsa_rdiv64(int64_t n, uint32_t d) {
	if (d == 1) {
		return n;
	}

	uint64_t m = n >= 0 ? (uint64_t)n : (uint64_t)0 - (uint64_t)n;
	uint64_t q = udiv64_small(m + d / 2, d);
	return n >= 0 ? (int64_t)q : -(int64_t)q;
}

static inline __attribute__((always_inline)) int64_t floor_div8(int64_t v) {
	return v >= 0 ? v / 8 : -((-v + 7) / 8);
}

static inline __attribute__((always_inline)) int16_t sat16(mcsa_core *c, int64_t v) {
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
static inline __attribute__((always_inline)) void cic32_push(uint32_t *integ, uint32_t x) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		integ[s] += x;
		x = integ[s];
	}
}

static inline __attribute__((always_inline)) void cic64_push(uint64_t *integ, uint64_t x) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		integ[s] += x;
		x = integ[s];
	}
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

static void put_u8(uint8_t *b, int32_t *i, uint8_t v) {
	b[(*i)++] = v;
}

static void put_u16(uint8_t *b, int32_t *i, uint16_t v) {
	b[(*i)++] = (uint8_t)(v >> 8);
	b[(*i)++] = (uint8_t)v;
}

static void put_u32(uint8_t *b, int32_t *i, uint32_t v) {
	b[(*i)++] = (uint8_t)(v >> 24);
	b[(*i)++] = (uint8_t)(v >> 16);
	b[(*i)++] = (uint8_t)(v >> 8);
	b[(*i)++] = (uint8_t)v;
}

static void put_i64(uint8_t *b, int32_t *i, int64_t v) {
	uint64_t u = (uint64_t)v;
	for (int s = 56; s >= 0; s -= 8) {
		b[(*i)++] = (uint8_t)(u >> s);
	}
}

static void put_f32(uint8_t *b, int32_t *i, float f) {
	uint32_t u;
	memcpy(&u, &f, sizeof(u));
	put_u32(b, i, u);
}

static uint16_t get_u16(const uint8_t *b) {
	return (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

static uint32_t get_u32(const uint8_t *b) {
	return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

static void lock_isr(const mcsa_env *env) {
	if (env->lock_isr) {
		env->lock_isr();
	}
}

static void unlock_isr(const mcsa_env *env) {
	if (env->unlock_isr) {
		env->unlock_isr();
	}
}

static uint32_t oldest_index(uint32_t n_written, uint32_t capacity) {
	return n_written >= capacity ? n_written - capacity : 0;
}

static int32_t put_hdr(const mcsa_core *c, const mcsa_env *env, uint8_t *r, uint8_t cmd, mcsa_status st) {
	int32_t i = 0;
	put_u8(r, &i, MCSA_MAGIC0);
	put_u8(r, &i, MCSA_MAGIC1);
	put_u8(r, &i, MCSA_PROTO_VERSION);
	put_u8(r, &i, (uint8_t)(cmd | 0x80u));
	put_u8(r, &i, (uint8_t)st);
	put_u16(r, &i, c->session);
	put_u32(r, &i, env->uptime_ms);
	return i;
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

void mcsa_core_init(mcsa_core *c, volatile int16_t *buf, uint32_t buf_bytes,
		volatile uint32_t *fifo, uint32_t fifo_bytes) {
	memset((void *)c, 0, sizeof(*c));
	c->buf = buf;
	c->buf_words = buf_bytes / 2;
	c->fifo = fifo;
	c->fifo_words = fifo_bytes / 4;
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

	// One FIFO entry: one word per current/flags channel, two per 64-bit channel
	uint32_t words = channel_count(mask & 0x0107u) + 2u * channel_count(mask & 0x00F8u);
	if (c->fifo == 0 || c->fifo_words / words < 2) {
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
	c->isr_limit = env->f_isr > 0.0f ? MCSA_ISR_LIMIT / env->f_isr : 0.0f;

	memset(c->cic32_int, 0, sizeof(c->cic32_int));
	memset(c->cic32_comb, 0, sizeof(c->cic32_comb));
	memset(c->cic64_int, 0, sizeof(c->cic64_int));
	memset(c->cic64_comb, 0, sizeof(c->cic64_comb));
	memset(c->flags_hist, 0, sizeof(c->flags_hist));
	c->isr_flags = 0;
	c->entry_words = 0;
	for (uint8_t i = 0; i < c->nch; i++) {
		uint8_t ch = c->ch_list[i];
		c->entry_words += (ch <= MCSA_CH_IC || ch == MCSA_CH_FLAGS) ? 1u : 2u;
	}
	c->fifo_slots = c->entry_words ? c->fifo_words / c->entry_words : 0;
	c->fifo_head = 0;
	c->fifo_tail = 0;
	c->pushed = 0;
	c->push_limit = mode == MCSA_MODE_ONESHOT ? length + MCSA_DROP_OUTPUTS : 0xFFFFFFFFu;
	c->isr_halt = false;
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

	// First recording after a capture: clear the MCSA data, so that sample
	// print never sends it, not even from slots its recording did not reach
	if (!c->sp_valid) {
		memset((void *)c->buf, 0, c->buf_words * sizeof(int16_t));
	}

	mcsa_core_invalidate(c, MCSA_STOP_SAMPLE_PRINT);
	c->sp_valid = true;
	return true;
}

// ---------------- ISR part ----------------
// Kept minimal: integrators, and every R cycles one FIFO entry with the last integrator
// stage of each channel. Combs, normalization and records run in mcsa_core_drain().

void mcsa_core_sample(mcsa_core *c, const mcsa_sample_in *in) {
	if (c->state != MCSA_STATE_RUNNING || c->isr_halt) {
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

	const uint16_t mask = c->mask;

	// ADC counts are integer-valued floats: plain conversion, no checks needed
	for (int k = 0; k < 3; k++) {
		if (mask & (1u << k)) {
			cic32_push(c->cic32_int[k], (uint32_t)((int32_t)in->adc[k] - c->off_int[k]));
		}
	}
	if (mask & ((1u << MCSA_CH_ID) | (1u << MCSA_CH_IQ) | (1u << MCSA_CH_VD) | (1u << MCSA_CH_VQ))) {
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
	}
	if (mask & (1u << MCSA_CH_ANGLE)) {
		cic64_push(c->cic64_int[4], (uint64_t)(c->theta - c->theta_start));
	}

	if (mask & (1u << MCSA_CH_FLAGS)) {
		// Flags of this cycle, combined over the current decimation window
		uint16_t fl = 0;
		for (int k = 0; k < 3; k++) {
			if (in->arr > MCSA_HIGH_MOD_MARGIN && in->ccr[k] > in->arr - MCSA_HIGH_MOD_MARGIN) {
				fl |= (uint16_t)(1u << k);
			}
		}
		if (c->in_count == 0) {
			c->isr_flags = (uint16_t)(fl | MCSA_FLAG_RUNNING);
		} else {
			c->isr_flags |= fl;
		}
		if (!in->running) {
			c->isr_flags &= (uint16_t)~MCSA_FLAG_RUNNING;
		}
	}

	if (++c->in_count < c->R) {
		return;
	}
	c->in_count = 0;

	for (int k = 0; k < 3; k++) {
		if (in->offset[k] != c->off_frozen[k]) {
			c->stop_reason = MCSA_STOP_OFFSETS_CHANGED;
			c->state = MCSA_STATE_DONE;
			return;
		}
	}

	uint32_t head = c->fifo_head;
	uint32_t next = head + 1 == c->fifo_slots ? 0 : head + 1;
	if (next == c->fifo_tail) {
		// The drain did not keep up; a lost entry would break the combs
		c->stop_reason = MCSA_STOP_FIFO_OVERRUN;
		c->state = MCSA_STATE_DONE;
		return;
	}

	volatile uint32_t *e = c->fifo + head * c->entry_words;
	for (int k = 0; k < 3; k++) {
		if (mask & (1u << k)) {
			*e++ = c->cic32_int[k][MCSA_CIC_ORDER - 1];
		}
	}
	for (int j = 0; j < 5; j++) {
		if (mask & (1u << (MCSA_CH_ID + j))) {
			uint64_t v = c->cic64_int[j][MCSA_CIC_ORDER - 1];
			*e++ = (uint32_t)v;
			*e++ = (uint32_t)(v >> 32);
		}
	}
	if (mask & (1u << MCSA_CH_FLAGS)) {
		*e++ = c->isr_flags;
	}
	c->fifo_head = next;

	if (++c->pushed >= c->push_limit) {
		c->isr_halt = true;
	}
}

// ---------------- Thread part ----------------

static inline __attribute__((always_inline)) int32_t comb32(uint32_t v, uint32_t *comb) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		uint32_t prev = comb[s];
		comb[s] = v;
		v -= prev;
	}
	return (int32_t)v;
}

static inline __attribute__((always_inline)) int64_t comb64(uint64_t v, uint64_t *comb) {
	for (int s = 0; s < MCSA_CIC_ORDER; s++) {
		uint64_t prev = comb[s];
		comb[s] = v;
		v -= prev;
	}
	return (int64_t)v;
}

static void stop_from_drain(mcsa_core *c, mcsa_stop_reason reason) {
	if (c->state == MCSA_STATE_RUNNING) {
		c->stop_reason = (uint8_t)reason;
		c->state = MCSA_STATE_DONE;
	}
	c->isr_halt = true;
}

void mcsa_core_drain(mcsa_core *c) {
	const uint16_t mask = c->mask;
	const int32_t r3 = (int32_t)c->r3;

	while (c->fifo_tail != c->fifo_head) {
		uint32_t tail = c->fifo_tail;
		volatile uint32_t *e = c->fifo + tail * c->entry_words;

		// Only the entries of channels in the mask are written and read
		int32_t y32[3];
		int64_t y64[5];
		uint16_t fl = 0;
		for (int k = 0; k < 3; k++) {
			if (mask & (1u << k)) {
				y32[k] = comb32(*e++, c->cic32_comb[k]);
			}
		}
		for (int j = 0; j < 5; j++) {
			if (mask & (1u << (MCSA_CH_ID + j))) {
				uint64_t v = *e++;
				v |= (uint64_t)(*e++) << 32;
				y64[j] = comb64(v, c->cic64_comb[j]);
			}
		}
		if (mask & (1u << MCSA_CH_FLAGS)) {
			fl = (uint16_t)*e++;
		}
		c->fifo_tail = tail + 1 == c->fifo_slots ? 0 : tail + 1;

		// The filter spans the last MCSA_CIC_ORDER windows
		c->flags_hist[0] = fl;
		uint16_t hm = 0;
		uint16_t run = MCSA_FLAG_RUNNING;
		for (int w = 0; w < MCSA_CIC_ORDER; w++) {
			hm |= c->flags_hist[w] & (MCSA_FLAG_HIGH_MOD_A | MCSA_FLAG_HIGH_MOD_B | MCSA_FLAG_HIGH_MOD_C);
			run &= c->flags_hist[w];
		}
		for (int w = MCSA_CIC_ORDER - 1; w > 0; w--) {
			c->flags_hist[w] = c->flags_hist[w - 1];
		}

		if (++c->out_count <= MCSA_DROP_OUTPUTS) {
			continue;
		}

		int16_t val[MCSA_CH_NUM];		// Only the channels in the mask are set and read
		for (int k = 0; k < 3; k++) {
			if (mask & (1u << k)) {
				int32_t q = y32[k] / r3;
				int32_t rem = y32[k] % r3;
				val[k] = sat16(c, (int64_t)8 * q + rdiv32(8 * rem, r3) - c->off_frac_q3[k]);
			}
		}
		for (int j = 0; j < 4; j++) {
			if (mask & (1u << (MCSA_CH_ID + j))) {
				val[MCSA_CH_ID + j] = sat16(c, mcsa_rdiv64(y64[j], c->r3));
			}
		}
		if (mask & (1u << MCSA_CH_ANGLE)) {
			int64_t rel = mcsa_rdiv64(y64[4], c->r3);
			if (rel >= MCSA_ANGLE_LIMIT || rel <= -MCSA_ANGLE_LIMIT) {
				stop_from_drain(c, MCSA_STOP_ANGLE_LIMIT);
				c->fifo_tail = c->fifo_head;
				return;
			}
			int64_t q13 = floor_div8(c->theta_start + rel);
			// The 16-bit angle can only be unwrapped below 4 revolutions per record
			int64_t step = q13 - c->last_angle_q13;
			if (c->n_written > 0 && (step >= 32768 || step <= -32768)) {
				stop_from_drain(c, MCSA_STOP_ANGLE_STEP);
				c->fifo_tail = c->fifo_head;
				return;
			}
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
			stop_from_drain(c, MCSA_STOP_LENGTH);
			c->fifo_tail = c->fifo_head;
			return;
		} else if (c->n_written >= MCSA_MAX_RECORDS) {
			stop_from_drain(c, MCSA_STOP_INDEX_LIMIT);
			c->fifo_tail = c->fifo_head;
			return;
		}
	}
}

void mcsa_core_isr_end(mcsa_core *c, float isr_duration) {
	if (c->sampled_this_isr) {
		c->sampled_this_isr = false;
		if (isr_duration > c->isr_max_capture) {
			c->isr_max_capture = isr_duration;
		}
		// Protects the motor control: one overlong ISR ends the capture
		if (c->isr_limit > 0.0f && isr_duration > c->isr_limit) {
			mcsa_core_stop(c, MCSA_STOP_ISR_OVERLOAD);
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

uint32_t mcsa_core_max_records_per_read(uint32_t resp_max, uint8_t rec_bytes) {
	if (rec_bytes == 0 || resp_max <= MCSA_READ_OVERHEAD) {
		return 0;
	}
	uint32_t n = (resp_max - MCSA_READ_OVERHEAD) / rec_bytes;
	return n > 0xFFFFu ? 0xFFFFu : n;
}

static int cmd_info(mcsa_core *c, const mcsa_env *env, uint8_t *r) {
	int32_t i = put_hdr(c, env, r, MCSA_CMD_INFO, MCSA_OK);
	put_u8(r, &i, MCSA_PROTO_VERSION);
	put_u8(r, &i, MCSA_CIC_ORDER);
	put_u8(r, &i, MCSA_R_MAX);
	put_u8(r, &i, env->pole_pairs);
	put_u32(r, &i, c->buf_words * 2);
	put_f32(r, &i, env->f_isr);
	put_f32(r, &i, env->fac_current);

	bool end = env->git_hash == 0;
	for (int k = 0; k < MCSA_GIT_HASH_LEN; k++) {
		char ch = end ? 0 : env->git_hash[k];
		if (ch == 0) {
			end = true;
		}
		put_u8(r, &i, (uint8_t)ch);
	}
	return i;
}

static int cmd_start(mcsa_core *c, const mcsa_env *env, const uint8_t *a, uint32_t alen, uint8_t *r) {
	if (alen < 10) {
		return put_hdr(c, env, r, MCSA_CMD_START, MCSA_BAD_ARG);
	}

	mcsa_status st = mcsa_core_start(c, env, a[0], get_u16(a + 1), a[3], get_u32(a + 4), get_u16(a + 8));
	int32_t i = put_hdr(c, env, r, MCSA_CMD_START, st);
	if (st == MCSA_OK) {
		put_u32(r, &i, c->capacity);
		put_u8(r, &i, (uint8_t)(2 * c->nch));
		put_u16(r, &i, c->mask);
		put_u8(r, &i, (uint8_t)c->R);
	}
	return i;
}

static int cmd_stop(mcsa_core *c, const mcsa_env *env, const uint8_t *a, uint32_t alen, uint8_t *r) {
	if (alen < 2) {
		return put_hdr(c, env, r, MCSA_CMD_STOP, MCSA_BAD_ARG);
	}

	mcsa_status st = MCSA_OK;
	lock_isr(env);
	if (get_u16(a) != c->session || c->state == MCSA_STATE_IDLE) {
		st = MCSA_NOT_AVAILABLE;
	} else {
		mcsa_core_stop(c, MCSA_STOP_CMD);
	}
	unlock_isr(env);

	// The ISR no longer pushes; store what is still waiting in the FIFO
	mcsa_core_drain(c);
	uint32_t nw = c->n_written;
	uint8_t reason = c->stop_reason;

	int32_t i = put_hdr(c, env, r, MCSA_CMD_STOP, st);
	put_u32(r, &i, nw);
	put_u8(r, &i, reason);
	return i;
}

static int cmd_status(mcsa_core *c, const mcsa_env *env, uint8_t *r) {
	mcsa_core_drain(c);

	lock_isr(env);
	mcsa_state state = c->state;
	uint8_t reason = c->stop_reason;
	uint32_t nw = c->n_written;
	int64_t theta_start = c->theta_start;
	float isr_idle = c->isr_max_idle;
	float isr_capture = c->isr_max_capture;
	uint32_t saturated = c->saturated;
	unlock_isr(env);

	bool idle = state == MCSA_STATE_IDLE;
	int32_t i = put_hdr(c, env, r, MCSA_CMD_STATUS, MCSA_OK);
	put_u8(r, &i, (uint8_t)state);
	put_u8(r, &i, reason);
	put_u8(r, &i, c->mode);
	put_u16(r, &i, c->mask);
	put_u8(r, &i, (uint8_t)c->R);
	put_u8(r, &i, MCSA_CIC_ORDER);
	put_u8(r, &i, env->pole_pairs);
	put_u32(r, &i, c->capacity);
	put_u32(r, &i, c->length);
	put_u32(r, &i, nw);
	put_u32(r, &i, oldest_index(nw, c->capacity));
	put_f32(r, &i, idle ? env->f_isr : c->f_isr);
	put_f32(r, &i, (idle ? env->fac_current : c->fac_current) / 8.0f);
	for (int k = 0; k < 3; k++) {
		put_f32(r, &i, c->off_frozen[k]);
	}
	put_i64(r, &i, theta_start);
	put_f32(r, &i, isr_idle * 1.0e6f);
	put_f32(r, &i, isr_capture * 1.0e6f);
	put_u32(r, &i, saturated);
	return i;
}

static int read_error(mcsa_core *c, const mcsa_env *env, uint8_t *r, mcsa_status st,
		uint32_t nw) {
	int32_t i = put_hdr(c, env, r, MCSA_CMD_READ, st);
	put_u32(r, &i, oldest_index(nw, c->capacity));
	put_u32(r, &i, nw);
	return i;
}

static int cmd_read(mcsa_core *c, const mcsa_env *env, const uint8_t *a, uint32_t alen,
		uint8_t *r, uint32_t resp_max) {
	if (alen < 8) {
		return put_hdr(c, env, r, MCSA_CMD_READ, MCSA_BAD_ARG);
	}

	uint16_t session = get_u16(a);
	uint32_t start = get_u32(a + 2);
	uint16_t count = get_u16(a + 6);

	mcsa_core_drain(c);

	lock_isr(env);
	mcsa_state state = c->state;
	uint32_t nw = c->n_written;
	int64_t newest_angle = c->last_angle_q13;
	unlock_isr(env);

	const uint32_t cap = c->capacity;
	if (state == MCSA_STATE_IDLE || state == MCSA_STATE_INVALID || session != c->session || start > nw) {
		return read_error(c, env, r, MCSA_NOT_AVAILABLE, nw);
	}
	if (nw - start > cap) {
		return read_error(c, env, r, MCSA_OVERWRITTEN, nw);
	}

	const uint8_t rec_bytes = (uint8_t)(2 * c->nch);
	uint32_t n = nw - start;
	uint32_t max_n = mcsa_core_max_records_per_read(resp_max, rec_bytes);
	if (n > count) {
		n = count;
	}
	if (n > max_n) {
		n = max_n;
	}

	int32_t i = put_hdr(c, env, r, MCSA_CMD_READ, MCSA_OK);
	put_u32(r, &i, start);
	put_u16(r, &i, (uint16_t)n);
	put_u8(r, &i, rec_bytes);
	int32_t anchor_pos = i;
	put_i64(r, &i, 0);
	int32_t data_pos = i;

	for (uint32_t k = 0; k < n; k++) {
		volatile int16_t *rec = c->buf + ((start + k) % cap) * c->nch;
		for (uint8_t ch = 0; ch < c->nch; ch++) {
			put_u16(r, &i, (uint16_t)rec[ch]);
		}
	}

	// Angle anchor: unwind from the newest record back to `start`
	int64_t anchor = 0;
	if (n > 0 && c->angle_pos != MCSA_ANGLE_NONE) {
		anchor = newest_angle;
		uint32_t idx = nw - 1;
		uint16_t later = (uint16_t)c->buf[(idx % cap) * c->nch + c->angle_pos];
		while (idx > start) {
			idx--;
			uint16_t earlier = (uint16_t)c->buf[(idx % cap) * c->nch + c->angle_pos];
			anchor -= (int16_t)(uint16_t)(later - earlier);
			later = earlier;
		}
	}

	if (env->read_copy_hook) {
		env->read_copy_hook(env->read_copy_hook_arg);
	}

	lock_isr(env);
	uint32_t nw2 = c->n_written;
	unlock_isr(env);
	if (nw2 - start > cap) {
		return read_error(c, env, r, MCSA_OVERWRITTEN, nw2);
	}

	put_i64(r, &anchor_pos, anchor);
	put_u16(r, &i, mcsa_crc16(r + data_pos, (uint32_t)(i - data_pos)));
	return i;
}

int mcsa_core_process(mcsa_core *c, const mcsa_env *env, const uint8_t *req, uint32_t len,
		uint8_t *resp, uint32_t resp_max) {
	if (len < 2 || req[0] != MCSA_MAGIC0 || req[1] != MCSA_MAGIC1) {
		return 0;
	}
	if (resp_max < MCSA_RESP_MIN) {
		return -1;
	}

	uint8_t cmd = len >= 4 ? req[3] : 0;
	if (len < 4) {
		return put_hdr(c, env, resp, cmd, MCSA_BAD_ARG);
	}
	if (req[2] != MCSA_PROTO_VERSION) {
		return put_hdr(c, env, resp, cmd, MCSA_BAD_VERSION);
	}

	const uint8_t *a = req + 4;
	uint32_t alen = len - 4;

	switch (cmd) {
	case MCSA_CMD_INFO:
		return cmd_info(c, env, resp);
	case MCSA_CMD_START:
		return cmd_start(c, env, a, alen, resp);
	case MCSA_CMD_STOP:
		return cmd_stop(c, env, a, alen, resp);
	case MCSA_CMD_STATUS:
		return cmd_status(c, env, resp);
	case MCSA_CMD_READ:
		return cmd_read(c, env, a, alen, resp, resp_max);
	default:
		return put_hdr(c, env, resp, cmd, MCSA_UNKNOWN_CMD);
	}
}
