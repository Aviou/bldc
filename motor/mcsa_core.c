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
