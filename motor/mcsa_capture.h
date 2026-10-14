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

#ifndef MCSA_CAPTURE_H_
#define MCSA_CAPTURE_H_

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"
#include "datatypes.h"
#include "mcsa_core.h"

#define MCSA_SP_SAMPLE_LEN			2000
#define MCSA_CAPTURE_BUF_BYTES		40000

// Layout of the COMM_SAMPLE_PRINT arrays, same order as in stock 6.02
typedef struct {
	int16_t curr0[MCSA_SP_SAMPLE_LEN];
	int16_t curr1[MCSA_SP_SAMPLE_LEN];
	int16_t ph1[MCSA_SP_SAMPLE_LEN];
	int16_t ph2[MCSA_SP_SAMPLE_LEN];
	int16_t ph3[MCSA_SP_SAMPLE_LEN];
	int16_t vzero[MCSA_SP_SAMPLE_LEN];
	uint8_t status[MCSA_SP_SAMPLE_LEN];
	int16_t curr_fir[MCSA_SP_SAMPLE_LEN];
	int16_t f_sw[MCSA_SP_SAMPLE_LEN];
	int8_t phase[MCSA_SP_SAMPLE_LEN];
} mcsa_sp_layout;

// COMM_SAMPLE_PRINT and the MCSA capture share this memory
typedef union {
	mcsa_sp_layout sp;
	int16_t mcsa[MCSA_CAPTURE_BUF_BYTES / 2];
} mcsa_shared_buffer;

extern volatile mcsa_shared_buffer mcsa_capture_shared;

void mcsa_capture_init(void);
void mcsa_capture_lock(void);
void mcsa_capture_unlock(void);

// FOC ISR hooks, motor 1 only. The ISR only integrates and pushes FIFO entries;
// a thread drains them every 0.5 ms (combs, normalization, records).
extern mcsa_core mcsa_capture_core;

static inline void mcsa_capture_isr_angle(float phase) {
	mcsa_core_update_angle(&mcsa_capture_core, phase);
}

static inline bool mcsa_capture_isr_wants_sample(void) {
	return mcsa_core_wants_sample(&mcsa_capture_core);
}

static inline void mcsa_capture_isr_sample(const mcsa_sample_in *in) {
	mcsa_core_sample(&mcsa_capture_core, in);
}

// Sum of all FOC ISR durations in CPU cycles (modular), for the drain cost in mcsa_perf
extern volatile uint32_t mcsa_capture_isr_cyc;

static inline void mcsa_capture_isr_end(float isr_duration) {
	mcsa_capture_isr_cyc += (uint32_t)(isr_duration * (float)STM32_SYSCLK);
	mcsa_core_isr_end(&mcsa_capture_core, isr_duration);
}

// Returns true if the COMM_CUSTOM_APP_DATA packet belonged to the MCSA capture
bool mcsa_capture_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len));

// COMM_SAMPLE_PRINT interplay; call with mcsa_capture_lock() held
bool mcsa_capture_sample_print_request(debug_sampling_mode mode);

// Configuration written: a running capture ends
void mcsa_capture_config_changed(void);

// Cycle counts of the ISR hook (DWT), shown by the terminal command mcsa_perf
void mcsa_capture_perf_add(uint32_t cyc_angle, uint32_t cyc_fill, uint32_t cyc_sample, bool output);

#endif /* MCSA_CAPTURE_H_ */
