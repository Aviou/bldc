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

volatile mcsa_capture_perf_cat mcsa_capture_perf[2];

// Drain thread: cycles per entry without FOC ISR time, longest time between two wake-ups,
// highest FIFO fill at a wake-up
typedef struct {
	uint32_t wakes, entries;
	uint64_t cyc;
	uint32_t max_gap, max_fill;
	uint32_t max_sleep, max_lock, max_call, min_one;
} drain_perf;

static volatile drain_perf m_drain;
static uint32_t m_perf_c0, m_perf_isr0;		// Cycle counter and ISR cycles at the last mcsa_perf
static systime_t m_perf_t0;
volatile uint32_t mcsa_capture_isr_cyc;

static void terminal_perf(int argc, const char **argv);

void mcsa_capture_init(void) {
	chMtxObjectInit(&m_mutex);
	mcsa_core_init(&mcsa_capture_core, mcsa_capture_shared.mcsa, MCSA_CAPTURE_BUF_BYTES,
			m_fifo, sizeof(m_fifo));
	chThdCreateStatic(drain_thread_wa, sizeof(drain_thread_wa), NORMALPRIO, drain_thread, NULL);

	// Cycle counter for mcsa_perf
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	terminal_register_command_callback("mcsa_perf",
			"Cycles of the MCSA capture hook in the FOC ISR since the last call (and reset).",
			0, terminal_perf);
}

static void terminal_perf(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	mcsa_capture_perf_cat p[2];
	chSysLock();
	for (int i = 0; i < 2; i++) {
		p[i].count = mcsa_capture_perf[i].count;
		p[i].max = mcsa_capture_perf[i].max;
		p[i].sum = mcsa_capture_perf[i].sum;
		memset((void *)&mcsa_capture_perf[i], 0, sizeof(mcsa_capture_perf_cat));
	}
	chSysUnlock();

	const float us = 1.0e6f / (float)STM32_SYSCLK;
	for (int i = 0; i < 2; i++) {
		float mean = p[i].count ? (float)p[i].sum / (float)p[i].count : 0.0f;
		commands_printf("hook %s: n=%u max %u cyc (%.2f us), mean %.0f cyc (%.2f us)",
				i ? "with output   " : "without output", (unsigned)p[i].count,
				(unsigned)p[i].max, (double)((float)p[i].max * us), (double)mean, (double)(mean * us));
	}

	drain_perf d;
	chSysLock();
	d.wakes = m_drain.wakes;
	d.entries = m_drain.entries;
	d.cyc = m_drain.cyc;
	d.max_gap = m_drain.max_gap;
	d.max_fill = m_drain.max_fill;
	d.max_sleep = m_drain.max_sleep;
	d.max_lock = m_drain.max_lock;
	d.max_call = m_drain.max_call;
	d.min_one = m_drain.min_one;
	memset((void *)&m_drain, 0, sizeof(m_drain));
	m_drain.min_one = UINT32_MAX;
	chSysUnlock();
	commands_printf("drain thread: %u wake-ups, %u entries, %.0f cyc per entry without FOC ISR, "
			"max gap %.0f us, max fill %u of %u entries",
			(unsigned)d.wakes, (unsigned)d.entries,
			(double)(d.entries ? (float)d.cyc / (float)d.entries : 0.0f),
			(double)((float)d.max_gap * us), (unsigned)d.max_fill,
			(unsigned)mcsa_capture_core.fifo_slots);
	commands_printf("drain thread: max sleep %.0f us, max lock wait %.0f us, max call %.0f us, "
			"single entry without ISR min %u cyc",
			(double)((float)d.max_sleep * us), (double)((float)d.max_lock * us),
			(double)((float)d.max_call * us), (unsigned)(d.min_one == UINT32_MAX ? 0 : d.min_one));
	// The 32-bit cycle counter wraps after 25.6 s
	uint32_t c_now = DWT->CYCCNT;
	uint32_t isr_now = mcsa_capture_isr_cyc;
	systime_t t_now = chVTGetSystemTimeX();
	if (m_perf_c0 != 0 && chVTTimeElapsedSinceX(m_perf_t0) < S2ST(20)) {
		commands_printf("FOC ISR load since the last call: %.1f %%",
				(double)(100.0f * (float)(isr_now - m_perf_isr0) / (float)(c_now - m_perf_c0)));
	} else {
		commands_printf("FOC ISR load: call mcsa_perf again within 20 s");
	}
	m_perf_c0 = c_now;
	m_perf_isr0 = isr_now;
	m_perf_t0 = t_now;
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

	mcsa_core *c = &mcsa_capture_core;
	uint32_t last = DWT->CYCCNT;

	m_drain.min_one = UINT32_MAX;

	// Batches of MCSA_DRAIN_BATCH keep the work per call bounded, and the thread yields between
	// them, so the threads of the same priority (among them the watchdog thread) keep running
	// even if the drain only just keeps up with the ISR.
	for (;;) {
		uint32_t s0 = DWT->CYCCNT;
		chThdSleepMicroseconds(500);

		uint32_t now = DWT->CYCCNT;
		if (now - s0 > m_drain.max_sleep) {
			m_drain.max_sleep = now - s0;
		}
		uint32_t head = c->fifo_head;
		uint32_t tail = c->fifo_tail;
		uint32_t fill = head >= tail ? head - tail : head + c->fifo_slots - tail;
		m_drain.wakes++;
		if (c->state == MCSA_STATE_RUNNING) {
			if (now - last > m_drain.max_gap) {
				m_drain.max_gap = now - last;
			}
			if (fill > m_drain.max_fill) {
				m_drain.max_fill = fill;
			}
		}
		last = now;

		uint32_t taken;
		do {
			uint32_t l0 = DWT->CYCCNT;
			mcsa_capture_lock();
			uint32_t c0 = DWT->CYCCNT;
			uint32_t i0 = mcsa_capture_isr_cyc;
			taken = mcsa_core_drain(c, MCSA_DRAIN_BATCH);
			uint32_t wall = DWT->CYCCNT - c0;
			uint32_t isr = mcsa_capture_isr_cyc - i0;
			mcsa_capture_unlock();
			if (c0 - l0 > m_drain.max_lock) {
				m_drain.max_lock = c0 - l0;
			}
			if (taken > 0) {
				m_drain.entries += taken;
				m_drain.cyc += wall > isr ? wall - isr : 0;
				if (wall > m_drain.max_call) {
					m_drain.max_call = wall;
				}
				if (taken == 1 && isr == 0 && wall < m_drain.min_one) {
					m_drain.min_one = wall;
				}
			}
			chThdYield();
		} while (taken == MCSA_DRAIN_BATCH);
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
