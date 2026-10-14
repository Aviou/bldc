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

#include "mcsa_capture.h"

#include <stddef.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "firmware_metadata.h"
#include "hw.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "mempools.h"
#include "packet.h"
#include "terminal.h"
#include "commands.h"

_Static_assert(sizeof(mcsa_sp_layout) == 36000, "COMM_SAMPLE_PRINT layout must be 36000 bytes");
_Static_assert(sizeof(mcsa_shared_buffer) == MCSA_CAPTURE_BUF_BYTES, "shared buffer size");
_Static_assert(offsetof(mcsa_sp_layout, curr1) == 4000 && offsetof(mcsa_sp_layout, vzero) == 20000 &&
		offsetof(mcsa_sp_layout, status) == 24000 && offsetof(mcsa_sp_layout, curr_fir) == 26000 &&
		offsetof(mcsa_sp_layout, f_sw) == 30000 && offsetof(mcsa_sp_layout, phase) == 34000,
		"COMM_SAMPLE_PRINT arrays must be packed without padding");
_Static_assert(PACKET_MAX_PL_LEN - 1 >= MCSA_RESP_MIN, "response buffer too small");

__attribute__((section(".ram4"), aligned(8))) volatile mcsa_shared_buffer mcsa_capture_shared;

// FIFO between the FOC ISR and the drain thread; at most 14 words per entry (all channels)
#define MCSA_FIFO_WORDS		448
__attribute__((section(".ram4"), aligned(4))) static volatile uint32_t m_fifo[MCSA_FIFO_WORDS];

static THD_WORKING_AREA(drain_thread_wa, 512);
static THD_FUNCTION(drain_thread, arg);

mcsa_core mcsa_capture_core;
static mutex_t m_mutex;

static void lock_isr(void) {
	chSysLock();
}

static void unlock_isr(void) {
	chSysUnlock();
}

static bool buffer_free(void) {
	return !mc_interface_sample_print_busy();
}

typedef struct {
	uint32_t count;
	uint32_t max_angle, max_fill, max_sample;
	uint64_t sum_sample;
} perf_cat;

static volatile perf_cat m_perf[2];		// [0] without, [1] with an output value

static void terminal_perf(int argc, const char **argv);

void mcsa_capture_init(void) {
	chMtxObjectInit(&m_mutex);
	mcsa_core_init(&mcsa_capture_core, mcsa_capture_shared.mcsa, MCSA_CAPTURE_BUF_BYTES,
			m_fifo, sizeof(m_fifo));
	chThdCreateStatic(drain_thread_wa, sizeof(drain_thread_wa), NORMALPRIO + 1, drain_thread, NULL);

	// Cycle counter for mcsa_perf
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	terminal_register_command_callback("mcsa_perf",
			"Cycles of the MCSA capture hook in the FOC ISR since the last call (and reset).",
			0, terminal_perf);
}

void mcsa_capture_perf_add(uint32_t cyc_angle, uint32_t cyc_fill, uint32_t cyc_sample, bool output) {
	volatile perf_cat *p = &m_perf[output ? 1 : 0];
	p->count++;
	p->sum_sample += cyc_sample;
	if (cyc_angle > p->max_angle) {
		p->max_angle = cyc_angle;
	}
	if (cyc_fill > p->max_fill) {
		p->max_fill = cyc_fill;
	}
	if (cyc_sample > p->max_sample) {
		p->max_sample = cyc_sample;
	}
}

static void terminal_perf(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	perf_cat p[2];
	chSysLock();
	for (int i = 0; i < 2; i++) {
		p[i].count = m_perf[i].count;
		p[i].max_angle = m_perf[i].max_angle;
		p[i].max_fill = m_perf[i].max_fill;
		p[i].max_sample = m_perf[i].max_sample;
		p[i].sum_sample = m_perf[i].sum_sample;
		memset((void *)&m_perf[i], 0, sizeof(perf_cat));
	}
	chSysUnlock();

	const float us = 1.0e6f / (float)STM32_SYSCLK;
	for (int i = 0; i < 2; i++) {
		float mean = p[i].count ? (float)p[i].sum_sample / (float)p[i].count : 0.0f;
		commands_printf("%s: n=%u angle max %u cyc, fill max %u cyc, sample max %u cyc (%.2f us), mean %.0f cyc (%.2f us)",
				i ? "with output   " : "without output", (unsigned)p[i].count,
				(unsigned)p[i].max_angle, (unsigned)p[i].max_fill, (unsigned)p[i].max_sample,
				(double)((float)p[i].max_sample * us), (double)mean, (double)(mean * us));
	}
	commands_printf("ISR max with capture %.1f us, without %.1f us\n",
			(double)(mcsa_capture_core.isr_max_capture * 1.0e6f),
			(double)(mcsa_capture_core.isr_max_idle * 1.0e6f));
}

void mcsa_capture_lock(void) {
	chMtxLock(&m_mutex);
}

void mcsa_capture_unlock(void) {
	chMtxUnlock(&m_mutex);
}

static THD_FUNCTION(drain_thread, arg) {
	(void)arg;
	chRegSetThreadName("MCSA drain");

	for (;;) {
		chThdSleepMicroseconds(500);
		if (mcsa_capture_core.fifo_tail != mcsa_capture_core.fifo_head) {
			mcsa_capture_lock();
			mcsa_core_drain(&mcsa_capture_core);
			mcsa_capture_unlock();
		}
	}
}

bool mcsa_capture_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	if (len < 2 || data[0] != MCSA_MAGIC0 || data[1] != MCSA_MAGIC1) {
		return false;
	}

	uint8_t *buf = mempools_get_packet_buffer();
	buf[0] = COMM_CUSTOM_APP_DATA;

	mcsa_capture_lock();

	// Read the environment under the lock, after any concurrent START or configuration change
	const volatile mc_configuration *conf = mc_interface_get_configuration();

	mcsa_env env;
	memset(&env, 0, sizeof(env));
	env.f_isr = mcpwm_foc_get_sampling_frequency_now() / (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;
	env.fac_current = FAC_CURRENT;
	env.pole_pairs = conf->si_motor_poles / 2;
	env.foc_active = conf->motor_type == MOTOR_TYPE_FOC;
	env.git_hash = GIT_COMMIT_HASH;
	env.uptime_ms = (uint32_t)(((uint64_t)chVTGetSystemTimeX() * 1000u) / CH_CFG_ST_FREQUENCY);
	env.buffer_free = buffer_free;
	env.lock_isr = lock_isr;
	env.unlock_isr = unlock_isr;

	int n = mcsa_core_process(&mcsa_capture_core, &env, data, len, buf + 1, PACKET_MAX_PL_LEN - 1);
	mcsa_capture_unlock();

	// Synchronous answer in the context of the incoming packet, also for CAN forwarding
	if (n > 0 && reply_func) {
		reply_func(buf, (unsigned int)n + 1);
	}

	mempools_free_packet_buffer(buf);
	return true;
}

bool mcsa_capture_sample_print_request(debug_sampling_mode mode) {
	mcsa_sp_request req = MCSA_SP_RECORD;

	if (mode == DEBUG_SAMPLING_OFF) {
		req = MCSA_SP_OFF;
	} else if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
		req = MCSA_SP_SEND_LAST;
	}

	return mcsa_core_sample_print_request(&mcsa_capture_core, req);
}

void mcsa_capture_config_changed(void) {
	chSysLock();
	mcsa_core_stop(&mcsa_capture_core, MCSA_STOP_CONFIG_CHANGED);
	chSysUnlock();
}
