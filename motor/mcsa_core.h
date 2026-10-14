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

#ifndef MCSA_CORE_H_
#define MCSA_CORE_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * MCSA capture core: revolution counter, CIC decimation filter, ring buffer
 * and command protocol. No ChibiOS or HAL dependency, so that it can be
 * tested on the host. See docs/mcsa_capture_protocol.md.
 */

// Protocol
#define MCSA_MAGIC0					0x4D	// 'M'
#define MCSA_MAGIC1					0x43	// 'C'
#define MCSA_PROTO_VERSION			1
#define MCSA_RESP_HDR_LEN			11
#define MCSA_READ_OVERHEAD			28
#define MCSA_RESP_MIN				128
#define MCSA_GIT_HASH_LEN			16

// Filter
#define MCSA_CIC_ORDER				3
#define MCSA_R_MAX					64
#define MCSA_DROP_OUTPUTS			(MCSA_CIC_ORDER - 1)

// A FOC ISR with capture that takes longer than this fraction of its cycle ends the capture
#define MCSA_ISR_LIMIT				0.9f

// Channels: bit in the mask, order in a record
#define MCSA_CH_IA					0
#define MCSA_CH_IB					1
#define MCSA_CH_IC					2
#define MCSA_CH_ID					3
#define MCSA_CH_IQ					4
#define MCSA_CH_VD					5
#define MCSA_CH_VQ					6
#define MCSA_CH_ANGLE				7
#define MCSA_CH_FLAGS				8
#define MCSA_CH_NUM					9
#define MCSA_MASK_ALL				((uint16_t)((1u << MCSA_CH_NUM) - 1u))
#define MCSA_MASK_DEFAULT			0x0087u

// Flag channel
#define MCSA_FLAG_HIGH_MOD_A		(1u << 0)
#define MCSA_FLAG_HIGH_MOD_B		(1u << 1)
#define MCSA_FLAG_HIGH_MOD_C		(1u << 2)
#define MCSA_FLAG_RUNNING			(1u << 3)
#define MCSA_HIGH_MOD_MARGIN		500		// Same threshold as the FOC current selection

// Limits
#define MCSA_MAX_RECORDS			0xF0000000u
#define MCSA_ANGLE_LIMIT			(((int64_t)1) << 44)
#define MCSA_ANGLE_NONE				0xFF

typedef enum {
	MCSA_STATE_IDLE = 0,
	MCSA_STATE_RUNNING,
	MCSA_STATE_DONE,
	MCSA_STATE_INVALID
} mcsa_state;

typedef enum {
	MCSA_MODE_ONESHOT = 1,
	MCSA_MODE_CONTINUOUS = 2
} mcsa_mode;

typedef enum {
	MCSA_CMD_INFO = 1,
	MCSA_CMD_START,
	MCSA_CMD_STOP,
	MCSA_CMD_STATUS,
	MCSA_CMD_READ
} mcsa_cmd;

typedef enum {
	MCSA_OK = 0,
	MCSA_BUSY,
	MCSA_BAD_ARG,
	MCSA_NOT_AVAILABLE,
	MCSA_OVERWRITTEN,
	MCSA_UNKNOWN_CMD,
	MCSA_BAD_VERSION,
	MCSA_NOT_FOC
} mcsa_status;

typedef enum {
	MCSA_STOP_NONE = 0,
	MCSA_STOP_LENGTH,
	MCSA_STOP_CMD,
	MCSA_STOP_OFFSETS_CHANGED,
	MCSA_STOP_CONFIG_CHANGED,
	MCSA_STOP_INDEX_LIMIT,
	MCSA_STOP_ANGLE_LIMIT,
	MCSA_STOP_SAMPLE_PRINT,
	MCSA_STOP_ANGLE_STEP,		// 4 or more electrical revolutions between two records
	MCSA_STOP_ISR_OVERLOAD,		// FOC ISR with capture longer than MCSA_ISR_LIMIT of its cycle
	MCSA_STOP_FIFO_OVERRUN		// The drain thread did not keep up with the ISR
} mcsa_stop_reason;

typedef enum {
	MCSA_SP_OFF = 0,		// COMM_SAMPLE_PRINT stop request, writes nothing
	MCSA_SP_RECORD,			// New COMM_SAMPLE_PRINT recording
	MCSA_SP_SEND_LAST		// Resend of the last COMM_SAMPLE_PRINT recording
} mcsa_sp_request;

// Input of one FOC ISR cycle while a capture is running. The ISR fills id..vq only if
// use_dq is set and ccr..running only if use_flags is set (mcsa_core_start sets both).
typedef struct {
	const float *adc;		// [3] Raw ADC counts of the shunts, before offset and reconstruction
	const float *offset;	// [3] Current offsets in ADC counts
	float id;				// A
	float iq;				// A
	float vd;				// V
	float vq;				// V
	uint32_t ccr[3];		// TIM CCR1..3
	uint32_t arr;			// TIM ARR
	bool running;			// Motor state is RUNNING
} mcsa_sample_in;

// Environment provided by the firmware glue or the tests
typedef struct {
	float f_isr;					// Hz, processing rate of the FOC ISR
	float fac_current;				// A per ADC count
	uint8_t pole_pairs;
	bool foc_active;				// Motor is configured for FOC
	const char *git_hash;
	uint32_t uptime_ms;
	bool (*buffer_free)(void);		// False while COMM_SAMPLE_PRINT uses the buffer; may be NULL
	void (*lock_isr)(void);			// Short critical section against the ISR; may be NULL
	void (*unlock_isr)(void);
	void (*read_copy_hook)(void *arg);	// Tests only: called after copying in READ
	void *read_copy_hook_arg;
} mcsa_env;

typedef struct {
	// Revolution counter, updated in every ISR cycle
	int64_t theta;					// 1/65536 electrical revolution
	uint16_t frac_last;
	bool theta_valid;

	// Buffer
	volatile int16_t *buf;
	uint32_t buf_words;
	bool sp_valid;					// COMM_SAMPLE_PRINT data in the buffer is still intact

	// Capture
	volatile mcsa_state state;
	volatile uint8_t stop_reason;
	bool need_init;
	uint8_t mode;
	uint16_t mask;
	bool use_dq;					// The mask contains id, iq, vd or vq
	bool use_flags;					// The mask contains the flag channel
	uint8_t nch;
	uint8_t angle_pos;
	uint8_t ch_list[MCSA_CH_NUM];
	uint32_t R;
	uint32_t r3;
	uint32_t length;
	uint32_t capacity;
	uint16_t session;
	uint16_t last_token;
	volatile uint32_t n_written;
	float f_isr;
	float fac_current;
	float q3_per_amp;

	// Filter
	uint32_t in_count;
	uint32_t out_count;
	uint32_t cic32_int[3][MCSA_CIC_ORDER];
	uint32_t cic32_comb[3][MCSA_CIC_ORDER];
	uint64_t cic64_int[5][MCSA_CIC_ORDER];
	uint64_t cic64_comb[5][MCSA_CIC_ORDER];
	int32_t off_int[3];
	int32_t off_frac_q3[3];
	float off_frozen[3];
	int64_t theta_start;
	int64_t last_angle_q13;			// Filtered angle of the newest record, 1/8192 revolution
	uint16_t isr_flags;				// Flags of the current window (ISR)
	uint16_t flags_hist[MCSA_CIC_ORDER];	// Flags of the last windows (drain)

	// FIFO from the ISR (last integrator stage per output) to the drain thread
	volatile uint32_t *fifo;
	uint32_t fifo_words;
	uint32_t fifo_slots;
	uint32_t entry_words;
	volatile uint32_t fifo_head;		// Written by the ISR
	volatile uint32_t fifo_tail;		// Written by the drain
	uint32_t pushed;
	uint32_t push_limit;
	volatile bool isr_halt;			// All outputs of a one-shot capture are pushed
	volatile uint32_t saturated;

	// ISR timing in s
	bool sampled_this_isr;
	volatile float isr_max_capture;
	float isr_limit;				// s, 0 = no limit
	volatile float isr_max_idle;
	float isr_max_idle_win;
	uint32_t isr_idle_cnt;
} mcsa_core;

void mcsa_core_init(mcsa_core *c, volatile int16_t *buf, uint32_t buf_bytes,
		volatile uint32_t *fifo, uint32_t fifo_bytes);
void mcsa_core_sample(mcsa_core *c, const mcsa_sample_in *in);
// Takes at most max_entries FIFO entries, and only those waiting at the call: the work per
// call stays bounded while the ISR keeps pushing. Returns the number of entries taken.
uint32_t mcsa_core_drain(mcsa_core *c, uint32_t max_entries);
mcsa_status mcsa_core_start(mcsa_core *c, const mcsa_env *env, uint8_t mode,
		uint16_t mask, uint8_t R, uint32_t length, uint16_t token);
void mcsa_core_stop(mcsa_core *c, mcsa_stop_reason reason);
void mcsa_core_invalidate(mcsa_core *c, mcsa_stop_reason reason);
bool mcsa_core_sample_print_request(mcsa_core *c, mcsa_sp_request req);
uint32_t mcsa_core_capacity(const mcsa_core *c, uint16_t mask);
uint32_t mcsa_core_max_records_per_read(uint32_t resp_max, uint8_t rec_bytes);
int mcsa_core_process(mcsa_core *c, const mcsa_env *env, const uint8_t *req, uint32_t len,
		uint8_t *resp, uint32_t resp_max);
uint16_t mcsa_crc16(const uint8_t *buf, uint32_t len);

// n / d rounded half away from zero, 1 <= d <= 2^18. Uses 32-bit hardware divisions
// only, because __aeabi_ldivmod is far too slow for the FOC ISR.
int64_t mcsa_rdiv64(int64_t n, uint32_t d);

static inline bool mcsa_core_running(const mcsa_core *c) {
	return c->state == MCSA_STATE_RUNNING;
}

static inline bool mcsa_core_wants_sample(const mcsa_core *c) {
	return c->state == MCSA_STATE_RUNNING && !c->isr_halt;
}

// Run in every FOC ISR, also without a capture: inline, so no call costs ISR time
#define MCSA_ANGLE_UNITS_PER_RAD	(65536.0f / 6.28318530717958647692f)
#define MCSA_IDLE_WINDOW			30000

static inline void mcsa_core_update_angle(mcsa_core *c, float phase) {
	// The comparison also rejects NaN
	if (!(phase > -1000.0f && phase < 1000.0f)) {
		return;
	}

	uint16_t frac = (uint16_t)(int32_t)(phase * MCSA_ANGLE_UNITS_PER_RAD);

	if (c->theta_valid) {
		c->theta += (int16_t)(uint16_t)(frac - c->frac_last);
	} else {
		c->theta = frac;
		c->theta_valid = true;
	}

	c->frac_last = frac;
}

static inline void mcsa_core_isr_end(mcsa_core *c, float isr_duration) {
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
		if (++c->isr_idle_cnt >= MCSA_IDLE_WINDOW) {
			c->isr_max_idle = c->isr_max_idle_win;
			c->isr_max_idle_win = 0.0f;
			c->isr_idle_cnt = 0;
		}
	}
}

#endif /* MCSA_CORE_H_ */
