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

_Static_assert(sizeof(mcsa_sp_layout) == 36000, "COMM_SAMPLE_PRINT layout must be 36000 bytes");
_Static_assert(sizeof(mcsa_shared_buffer) == MCSA_CAPTURE_BUF_BYTES, "shared buffer size");
_Static_assert(offsetof(mcsa_sp_layout, curr1) == 4000 && offsetof(mcsa_sp_layout, vzero) == 20000 &&
		offsetof(mcsa_sp_layout, status) == 24000 && offsetof(mcsa_sp_layout, curr_fir) == 26000 &&
		offsetof(mcsa_sp_layout, f_sw) == 30000 && offsetof(mcsa_sp_layout, phase) == 34000,
		"COMM_SAMPLE_PRINT arrays must be packed without padding");
_Static_assert(PACKET_MAX_PL_LEN - 1 >= MCSA_RESP_MIN, "response buffer too small");

__attribute__((section(".ram4"), aligned(8))) volatile mcsa_shared_buffer mcsa_capture_shared;

static mcsa_core m_core;
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

void mcsa_capture_init(void) {
	chMtxObjectInit(&m_mutex);
	mcsa_core_init(&m_core, mcsa_capture_shared.mcsa, MCSA_CAPTURE_BUF_BYTES);
}

void mcsa_capture_lock(void) {
	chMtxLock(&m_mutex);
}

void mcsa_capture_unlock(void) {
	chMtxUnlock(&m_mutex);
}

void mcsa_capture_isr_angle(float phase) {
	mcsa_core_update_angle(&m_core, phase);
}

bool mcsa_capture_isr_wants_sample(void) {
	return mcsa_core_running(&m_core);
}

void mcsa_capture_isr_sample(const mcsa_sample_in *in) {
	mcsa_core_sample(&m_core, in);
}

void mcsa_capture_isr_end(float isr_duration) {
	mcsa_core_isr_end(&m_core, isr_duration);
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

	int n = mcsa_core_process(&m_core, &env, data, len, buf + 1, PACKET_MAX_PL_LEN - 1);
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

	return mcsa_core_sample_print_request(&m_core, req);
}

void mcsa_capture_config_changed(void) {
	chSysLock();
	mcsa_core_stop(&m_core, MCSA_STOP_CONFIG_CHANGED);
	chSysUnlock();
}
