# MCSA-Aufnahmemodus Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Neuer Aufnahmemodus in der VESC-Firmware 6.02 (HW 60), der Phasenströme und Rotorwinkel CIC-gefiltert in einen Ringpuffer schreibt und über `COMM_CUSTOM_APP_DATA` verlustfrei abholbar macht, dazu ein Python-Client.

**Architecture:** Ein hardwareunabhängiger Kern (`motor/mcsa_core.c`) enthält Umdrehungszähler, CIC-Filter, Ringpuffer und Protokoll und wird auf dem PC getestet. Eine dünne Firmware-Schicht (`motor/mcsa_capture.c`) hängt ihn in die FOC-ISR und den Paketpfad und teilt sich den Speicher mit `COMM_SAMPLE_PRINT`. Der Client (`tools/mcsa_capture_client.py`) wird gegen den als Bibliothek übersetzten Kern getestet.

**Tech Stack:** C99 (arm-none-eabi-gcc 7-2018-q2 für die Firmware, Host-gcc für Tests), ChibiOS 3.0.5, Python 3 mit numpy, pyserial, pytest.

**Spec:** `docs/superpowers/specs/2026-10-13-mcsa-capture-design.md`

## Global Constraints

- Basis `release_6_02`, Build-Ziel `60` (`make fw_60`), keine neuen Compiler-Warnungen (Stock-Build hat 0).
- `FW_VERSION_MAJOR/MINOR` 6.02 und `FW_TEST_VERSION_NUMBER` 0 bleiben unverändert.
- Nichts flashen, nichts auf Geräte schreiben.
- Gemeinsamer Puffer 40 000 B in `.ram4`; `.ram4` darf 63 488 B nicht überschreiten.
- `COMM_SAMPLE_PRINT` verhält sich ohne MCSA-Nutzung exakt wie Stock 6.02.
- Kern ohne ChibiOS-/HAL-Abhängigkeit; Host-Tests mit `-Wall -Wextra -Werror -Wdouble-promotion -Wshadow`.
- Alle Gleitkomma-Konstanten im Kern mit `f`-Suffix; kein `lrintf` in der ISR.
- Protokoll: Magic `0x4D 0x43`, Version 1, Antwortkopf 11 B, READ-Overhead 28 B, Antwortpuffer 511 B.
- Copyright-Kopf neuer C-Dateien: GPL-v3-Kopf wie im Repo, Inhaber „Aviou“.

## Review Focus

1. Fremde `COMM_CUSTOM_APP_DATA`-Pakete (anderes Magic, LispBM) müssen unverändert weiterlaufen → Test „not ours“ in Task 3, Code-Prüfung in Task 4.
2. VESC Tool „Sampling“ nach oder während einer MCSA-Aufnahme darf nie MCSA-Bytes als Sample-Print senden und nichts blockieren → Kernlogik `mcsa_core_sample_print_request` mit Test in Task 2.
3. Verlorene Antworten: wiederholtes `START` startet nicht neu, `READ` ist wiederholbar → Python-Tests in Task 6.
4. Reboot des Controllers mitten in einer Daueraufnahme: Client erkennt ihn und bricht mit klarer Meldung ab, statt endlos zu warten → Python-Test in Task 6.
5. Rückwärtslauf und Stillstand (Offset-Prüfung im Stand) → Umdrehungszähler-Test in Task 1, Langlauf rückwärts in Task 2, Offset-Prüfung in Task 6.

---

## File Structure

| Datei | Verantwortung |
|---|---|
| `motor/mcsa_core.h` | Typen, Konstanten, Schnittstelle des Kerns |
| `motor/mcsa_core.c` | Umdrehungszähler, CIC, Ring, Start/Stop, Sample-Print-Regeln, Protokoll |
| `motor/mcsa_capture.h/.c` | Firmware-Schicht: Union-Puffer, Mutex, Umgebung, Hooks |
| `motor/mcpwm_foc.c` | drei Hooks in der FOC-ISR |
| `motor/mc_interface.c/.h` | Puffer-Makros, Sende-Zähler, Busy-Abfrage, Sperre, Konfigurationshook, Init |
| `comm/commands.c` | Abzweig in `COMM_CUSTOM_APP_DATA` |
| `motor/motor.mk` | neue Quellen |
| `tests/mcsa_core/Makefile`, `test_mcsa_core.c`, `mcsa_testlib.c`, `.gitignore` | Host-Tests und Testbibliothek |
| `tools/mcsa_capture_client.py`, `tools/test_mcsa_capture_client.py`, `tools/.gitignore` | Client und dessen Tests |
| `docs/mcsa_capture_protocol.md`, `docs/mcsa_capture_flashing.md` | Protokoll, Flash-Anleitung |

---

### Task 1: Kern-Gerüst, Testumgebung, Umdrehungszähler

**Files:**
- Create: `motor/mcsa_core.h`, `motor/mcsa_core.c`
- Create: `tests/mcsa_core/Makefile`, `tests/mcsa_core/test_mcsa_core.c`, `tests/mcsa_core/.gitignore`

**Interfaces:**
- Produces: vollständiger Header `mcsa_core.h` (alle Typen und Prototypen, die Task 2 und 3 implementieren); `mcsa_core_init`, `mcsa_core_update_angle`, `mcsa_crc16`.

- [ ] **Step 1: Header anlegen** (`motor/mcsa_core.h`)

```c
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
	MCSA_STOP_SAMPLE_PRINT
} mcsa_stop_reason;

typedef enum {
	MCSA_SP_OFF = 0,		// COMM_SAMPLE_PRINT stop request, writes nothing
	MCSA_SP_RECORD,			// New COMM_SAMPLE_PRINT recording
	MCSA_SP_SEND_LAST		// Resend of the last COMM_SAMPLE_PRINT recording
} mcsa_sp_request;

// Input of one FOC ISR cycle while a capture is running
typedef struct {
	float adc[3];			// Raw ADC counts of the shunts, before offset and reconstruction
	float offset[3];		// Current offsets in ADC counts
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
	uint16_t flags_win[MCSA_CIC_ORDER];
	volatile uint32_t saturated;

	// ISR timing in s
	bool sampled_this_isr;
	volatile float isr_max_capture;
	volatile float isr_max_idle;
	float isr_max_idle_win;
	uint32_t isr_idle_cnt;
} mcsa_core;

void mcsa_core_init(mcsa_core *c, volatile int16_t *buf, uint32_t buf_bytes);
void mcsa_core_update_angle(mcsa_core *c, float phase);
void mcsa_core_sample(mcsa_core *c, const mcsa_sample_in *in);
void mcsa_core_isr_end(mcsa_core *c, float isr_duration);
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

static inline bool mcsa_core_running(const mcsa_core *c) {
	return c->state == MCSA_STATE_RUNNING;
}

#endif /* MCSA_CORE_H_ */
```

- [ ] **Step 2: Testumgebung anlegen**

`tests/mcsa_core/Makefile` (Rezeptzeilen mit Tabulator):

```make
# Host tests for motor/mcsa_core.c
#   make test   build and run the C tests
#   make lib    shared library for tools/test_mcsa_capture_client.py

TOP    := ../..
CC     ?= cc
CFLAGS := -std=gnu99 -O2 -g -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -I$(TOP)/motor
LDLIBS := -lm
CORE   := $(TOP)/motor/mcsa_core.c $(TOP)/motor/mcsa_core.h

.PHONY: all test lib clean

all: test lib

test: test_mcsa_core
	./test_mcsa_core

test_mcsa_core: test_mcsa_core.c $(CORE)
	$(CC) $(CFLAGS) -o $@ test_mcsa_core.c $(TOP)/motor/mcsa_core.c $(LDLIBS)

lib: libmcsa_core.so

libmcsa_core.so: mcsa_testlib.c $(CORE)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ mcsa_testlib.c $(TOP)/motor/mcsa_core.c $(LDLIBS)

clean:
	rm -f test_mcsa_core libmcsa_core.so
```

`tests/mcsa_core/.gitignore`:

```
test_mcsa_core
libmcsa_core.so
```

`tests/mcsa_core/test_mcsa_core.c` (Grundgerüst mit Hilfsfunktionen und den Tests für Task 1):

```c
// Host tests for motor/mcsa_core.c. Run: make -C tests/mcsa_core test

#include "mcsa_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failed++; \
	printf("FEHLER %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

#define CHECK_EQ(a, b) do { long long a_ = (long long)(a); long long b_ = (long long)(b); \
	g_checks++; if (a_ != b_) { g_failed++; \
	printf("FEHLER %s:%d: %s = %lld, erwartet %lld\n", __FILE__, __LINE__, #a, a_, b_); } } while (0)

#define CHECK_NEAR(a, b, tol) do { double a_ = (double)(a); double b_ = (double)(b); \
	double t_ = (double)(tol); g_checks++; if (fabs(a_ - b_) > t_) { g_failed++; \
	printf("FEHLER %s:%d: %s = %.6f, erwartet %.6f +- %g\n", __FILE__, __LINE__, #a, a_, b_, t_); } } while (0)

#define PI_D		3.14159265358979323846
#define TWO_PI_D	(2.0 * PI_D)
#define F_ISR		30000.0
#define FAC_CUR		0.0805861f
#define BUF_BYTES	40000
#define RESP_MAX	511

static int16_t g_buf[BUF_BYTES / 2];
static bool g_sp_busy = false;

static float wrap_pi(double x) {
	double y = fmod(x + PI_D, TWO_PI_D);
	if (y < 0.0) {
		y += TWO_PI_D;
	}
	return (float)(y - PI_D);
}

static bool tb_buffer_free(void) {
	return !g_sp_busy;
}

static mcsa_env test_env(void) {
	mcsa_env e;
	memset(&e, 0, sizeof(e));
	e.f_isr = (float)F_ISR;
	e.fac_current = FAC_CUR;
	e.pole_pairs = 7;
	e.foc_active = true;
	e.git_hash = "abc1234-dirty";
	e.uptime_ms = 1234;
	e.buffer_free = tb_buffer_free;
	return e;
}

static void new_core(mcsa_core *c, uint32_t buf_bytes) {
	memset(g_buf, 0, sizeof(g_buf));
	g_sp_busy = false;
	mcsa_core_init(c, g_buf, buf_bytes);
}

// ---------------- Task 1 ----------------

static void test_crc16(void) {
	const char *s = "123456789";
	CHECK_EQ(mcsa_crc16((const uint8_t *)s, 9), 0x31C3);
}

static void test_angle_counter(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);

	mcsa_core_update_angle(&c, 0.0f);
	CHECK_EQ(c.theta, 0);

	// 10.25 revolutions forward
	for (int i = 1; i <= 1000; i++) {
		mcsa_core_update_angle(&c, wrap_pi(TWO_PI_D * 10.25 * i / 1000.0));
	}
	CHECK_NEAR((double)c.theta, 10.25 * 65536.0, 1.0);

	// 20 revolutions backward
	int64_t t0 = c.theta;
	for (int i = 1; i <= 2000; i++) {
		mcsa_core_update_angle(&c, wrap_pi(TWO_PI_D * (10.25 - 20.0 * i / 2000.0)));
	}
	CHECK_NEAR((double)(c.theta - t0), -20.0 * 65536.0, 2.0);

	// Standstill, NaN and out-of-range values change nothing
	t0 = c.theta;
	for (int i = 0; i < 100; i++) {
		mcsa_core_update_angle(&c, wrap_pi(TWO_PI_D * -9.75));
	}
	mcsa_core_update_angle(&c, NAN);
	mcsa_core_update_angle(&c, 1.0e9f);
	CHECK_EQ(c.theta, t0);
}

int main(void) {
	test_crc16();
	test_angle_counter();

	printf("%d Pruefungen, %d Fehler\n", g_checks, g_failed);
	return g_failed == 0 ? 0 : 1;
}
```

- [ ] **Step 3: Test laufen lassen, er muss beim Linken scheitern**

Run: `make -C tests/mcsa_core test`
Expected: FAIL mit „undefined reference to `mcsa_crc16`“ (bzw. fehlende `mcsa_core.c`).

- [ ] **Step 4: Minimale Implementierung** (`motor/mcsa_core.c`)

```c
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
```

- [ ] **Step 5: Test laufen lassen**

Run: `make -C tests/mcsa_core test`
Expected: `… Pruefungen, 0 Fehler`, Exit-Code 0.

- [ ] **Step 6: Commit**

```bash
git add motor/mcsa_core.h motor/mcsa_core.c tests/mcsa_core/Makefile tests/mcsa_core/test_mcsa_core.c tests/mcsa_core/.gitignore
git commit -m "feat(mcsa): Kern-Geruest mit Umdrehungszaehler und Host-Tests"
```

---

### Task 2: CIC-Filter, Ringpuffer, Start/Stop, Sample-Print-Regeln

**Files:**
- Modify: `motor/mcsa_core.c` (Funktionen anhängen)
- Modify: `tests/mcsa_core/test_mcsa_core.c` (Simulator und Tests einfügen vor `main`, Aufrufe in `main`)

**Interfaces:**
- Consumes: Header aus Task 1.
- Produces: `mcsa_core_sample`, `mcsa_core_isr_end`, `mcsa_core_start`, `mcsa_core_stop`, `mcsa_core_invalidate`, `mcsa_core_sample_print_request`, `mcsa_core_capacity`. Testhelfer `sim_t`, `sim_default`, `sim_step`, `sim_run`, `sim_run_until_done`, `rec_val`, `unwrap_from_start`, `fit_tone`, `cic_gain`, `start_ok` (von Task 3 weiterverwendet).

- [ ] **Step 1: Tests schreiben** — in `test_mcsa_core.c` vor `main` einfügen:

```c
// ---------------- Simulator ----------------

typedef struct {
	double t;			// ISR cycle counter
	double f_e;			// Electrical frequency in Hz, negative = reverse
	float offset[3];	// ADC counts
	double dc[3];		// Added to the raw value, ADC counts
	double amp;			// Fundamental, phase locked to the angle, ADC counts
	double phi;			// Phase of the fundamental relative to the angle, rad
	double f_tone;		// Additional tone on all phases, Hz
	double a_tone;		// ADC counts
	bool ramp;			// Raw value of phase a = offset + (t mod 1000)
	uint32_t ccr[3];
	uint32_t arr;
	bool running;
	float id;
	float iq;
	float vd;
	float vq;
} sim_t;

static sim_t sim_default(void) {
	sim_t s;
	memset(&s, 0, sizeof(s));
	for (int k = 0; k < 3; k++) {
		s.offset[k] = 2048.0f;
	}
	s.arr = 2800;
	s.running = true;
	return s;
}

static double sim_theta(const sim_t *s, double t) {
	return 0.4 + TWO_PI_D * s->f_e * t / F_ISR;
}

static void sim_step(mcsa_core *c, sim_t *s) {
	double th = sim_theta(s, s->t);
	mcsa_core_update_angle(c, wrap_pi(th));

	if (mcsa_core_running(c)) {
		mcsa_sample_in in;
		memset(&in, 0, sizeof(in));
		for (int k = 0; k < 3; k++) {
			double v = (double)s->offset[k] + s->dc[k]
					+ s->amp * cos(th + s->phi - k * TWO_PI_D / 3.0)
					+ s->a_tone * sin(TWO_PI_D * s->f_tone * s->t / F_ISR);
			if (k == 0 && s->ramp) {
				v = (double)s->offset[0] + fmod(s->t, 1000.0);
			}
			in.adc[k] = (float)floor(v + 0.5);
			in.offset[k] = s->offset[k];
			in.ccr[k] = s->ccr[k];
		}
		in.arr = s->arr;
		in.running = s->running;
		in.id = s->id;
		in.iq = s->iq;
		in.vd = s->vd;
		in.vq = s->vq;
		mcsa_core_sample(c, &in);
	}

	mcsa_core_isr_end(c, 1.0e-5f);
	s->t += 1.0;
}

static void sim_run(mcsa_core *c, sim_t *s, uint32_t n) {
	for (uint32_t i = 0; i < n; i++) {
		sim_step(c, s);
	}
}

static void sim_run_until_done(mcsa_core *c, sim_t *s) {
	for (uint32_t i = 0; i < 10000000u && mcsa_core_running(c); i++) {
		sim_step(c, s);
	}
}

static int16_t rec_val(const mcsa_core *c, uint32_t n, uint8_t pos) {
	return c->buf[(n % c->capacity) * c->nch + pos];
}

// Unwrap the stored 16-bit angles of records 0..count-1 (1/8192 revolution)
static void unwrap_from_start(const mcsa_core *c, uint8_t pos, uint32_t count, int64_t *out) {
	int64_t base = c->theta_start >= 0 ? c->theta_start / 8 : -((-c->theta_start + 7) / 8);
	uint16_t a0 = (uint16_t)rec_val(c, 0, pos);
	out[0] = base + (int16_t)(uint16_t)(a0 - (uint16_t)(base & 0xFFFF));
	for (uint32_t n = 1; n < count; n++) {
		uint16_t a = (uint16_t)rec_val(c, n, pos);
		uint16_t p = (uint16_t)rec_val(c, n - 1, pos);
		out[n] = out[n - 1] + (int16_t)(uint16_t)(a - p);
	}
}

static double cic_gain(double f, double R) {
	double x = PI_D * f / F_ISR;
	if (fabs(sin(x)) < 1e-12) {
		return 1.0;
	}
	double h = sin(R * x) / (R * sin(x));
	return fabs(h * h * h);
}

// Amplitude and phase of a tone; needs an integer number of periods in the records
static void fit_tone(const mcsa_core *c, uint8_t pos, uint32_t count, double f, double scale,
		double *amp, double *ph) {
	double sum_c = 0.0;
	double sum_s = 0.0;
	double R = (double)c->R;
	for (uint32_t n = 0; n < count; n++) {
		double t = (((double)n + 3.0) * R - 1.0 - 1.5 * (R - 1.0)) / F_ISR;
		double x = (double)rec_val(c, n, pos) * scale;
		sum_c += x * cos(TWO_PI_D * f * t);
		sum_s += x * sin(TWO_PI_D * f * t);
	}
	*amp = 2.0 / count * sqrt(sum_c * sum_c + sum_s * sum_s);
	*ph = atan2(-sum_s, sum_c);
}

static void start_ok(mcsa_core *c, uint8_t mode, uint16_t mask, uint8_t R, uint32_t length) {
	mcsa_env e = test_env();
	CHECK_EQ(mcsa_core_start(c, &e, mode, mask, R, length, 0), MCSA_OK);
}

// ---------------- Task 2 ----------------

static void test_fixed_point_and_offsets(void) {
	const uint8_t rs[5] = {1, 2, 7, 10, 64};
	for (int i = 0; i < 5; i++) {
		mcsa_core c;
		new_core(&c, BUF_BYTES);
		sim_t s = sim_default();
		for (int k = 0; k < 3; k++) {
			s.offset[k] = 2047.375f;
		}
		s.dc[1] = 2.625;
		s.dc[2] = -1.375;
		start_ok(&c, MCSA_MODE_ONESHOT, 0x0007, rs[i], 5);
		sim_run_until_done(&c, &s);
		CHECK_EQ(c.n_written, 5);
		CHECK_EQ(c.stop_reason, MCSA_STOP_LENGTH);
		for (uint32_t n = 0; n < 5; n++) {
			CHECK_EQ(rec_val(&c, n, 0), -3);	// -0.375 counts
			CHECK_EQ(rec_val(&c, n, 1), 21);	// +2.625 counts
			CHECK_EQ(rec_val(&c, n, 2), -11);	// -1.375 counts
		}
	}

	// Offset that is no multiple of 1/8: error at most one unit
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	s.offset[0] = 2047.4f;
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 10, 3);
	sim_run_until_done(&c, &s);
	CHECK_NEAR(rec_val(&c, 0, 0), (2047.0 - 2047.4) * 8.0, 1.0);
}

static void test_warmup(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 10, 3);
	sim_run(&c, &s, 29);
	CHECK_EQ(c.n_written, 0);
	sim_run(&c, &s, 1);
	CHECK_EQ(c.n_written, 1);
}

static void test_frequency_response(void) {
	double a;
	double p;

	// 400 Hz at R = 10 follows |H|
	{
		mcsa_core c;
		new_core(&c, BUF_BYTES);
		sim_t s = sim_default();
		s.f_tone = 400.0;
		s.a_tone = 1000.0;
		start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 10, 3000);
		sim_run_until_done(&c, &s);
		fit_tone(&c, 0, 3000, 400.0, 1.0 / 8.0, &a, &p);
		double exp_a = 1000.0 * cic_gain(400.0, 10.0);
		CHECK_NEAR(a, exp_a, exp_a * 0.005);
	}

	// 11 kHz at R = 10 folds to 1 kHz and is suppressed by at least 55 dB
	{
		mcsa_core c;
		new_core(&c, BUF_BYTES);
		sim_t s = sim_default();
		s.f_tone = 11000.0;
		s.a_tone = 1000.0;
		start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 10, 3000);
		sim_run_until_done(&c, &s);
		fit_tone(&c, 0, 3000, 1000.0, 1.0 / 8.0, &a, &p);
		CHECK(a < 1000.0 * pow(10.0, -55.0 / 20.0));
	}

	// Carrier 200 Hz plus sideband 230 Hz at R = 20, each scaled by |H| at its frequency
	{
		mcsa_core c;
		new_core(&c, BUF_BYTES);
		sim_t s = sim_default();
		s.f_e = 200.0;
		s.amp = 800.0;
		s.f_tone = 230.0;
		s.a_tone = 50.0;
		start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 20, 1500);
		sim_run_until_done(&c, &s);
		fit_tone(&c, 0, 1500, 200.0, 1.0 / 8.0, &a, &p);
		double exp_c = 800.0 * cic_gain(200.0, 20.0);
		CHECK_NEAR(a, exp_c, exp_c * 0.005);
		fit_tone(&c, 0, 1500, 230.0, 1.0 / 8.0, &a, &p);
		double exp_s = 50.0 * cic_gain(230.0, 20.0);
		CHECK_NEAR(a, exp_s, exp_s * 0.02);
	}
}

static void test_angle_alignment(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	s.f_e = 200.0;
	s.amp = 1000.0;
	s.phi = 0.3;
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0081, 20, 1500);
	sim_run_until_done(&c, &s);

	static int64_t ang[1500];
	unwrap_from_start(&c, 1, 1500, ang);

	double sc = 0.0;
	double ss = 0.0;
	for (uint32_t n = 0; n < 1500; n++) {
		double th = (double)ang[n] / 8192.0 * TWO_PI_D;
		double x = (double)rec_val(&c, n, 0) / 8.0;
		sc += x * cos(th);
		ss += x * sin(th);
	}
	double amp = 2.0 / 1500.0 * sqrt(sc * sc + ss * ss);
	double ph = atan2(-ss, sc);
	CHECK_NEAR(ph * 180.0 / PI_D, 0.3 * 180.0 / PI_D, 0.1);
	double exp_a = 1000.0 * cic_gain(200.0, 20.0);
	CHECK_NEAR(amp, exp_a, exp_a * 0.005);

	// Angle steps match the electrical speed
	double step = 200.0 * 20.0 / F_ISR * 8192.0;
	CHECK_NEAR((double)(ang[1499] - ang[0]) / 1499.0, step, 0.01);
}

static void test_long_capture_angle_overflow(void) {
	const double fe[2] = {1000.0, -1000.0};
	for (int d = 0; d < 2; d++) {
		mcsa_core c;
		new_core(&c, 800);
		sim_t s = sim_default();
		s.f_e = fe[d];
		start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0080, 64, 0);
		sim_run(&c, &s, 900000);	// 30 s; the third integrator wraps after about 10 s
		CHECK(mcsa_core_running(&c));
		uint32_t nw = c.n_written;
		double step = fe[d] * 64.0 / F_ISR * 8192.0;
		for (uint32_t n = nw - 50; n < nw; n++) {
			int16_t diff = (int16_t)(uint16_t)((uint16_t)rec_val(&c, n, 0) - (uint16_t)rec_val(&c, n - 1, 0));
			CHECK_NEAR(diff, step, 1.0);
		}
		double t_c = ((double)(nw - 1) + 3.0) * 64.0 - 1.0 - 1.5 * 63.0;
		double expected = (double)c.theta_start / 8.0 + fe[d] * t_c / F_ISR * 8192.0;
		CHECK_NEAR((double)c.last_angle_q13, expected, 2.0);
	}
}

static void test_ring_oneshot_continuous(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0001, 1, 100);
	sim_run(&c, &s, 200);
	CHECK_EQ(c.n_written, 100);
	CHECK_EQ(c.state, MCSA_STATE_DONE);
	CHECK_EQ(c.stop_reason, MCSA_STOP_LENGTH);

	// Continuous with a small ring: 100 records of 4 channels
	new_core(&c, 800);
	s = sim_default();
	s.ramp = true;
	s.offset[0] = 2000.0f;
	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0087, 1, 0);
	CHECK_EQ(c.capacity, 100);
	sim_run(&c, &s, 252);
	CHECK_EQ(c.n_written, 250);
	CHECK(mcsa_core_running(&c));
	for (uint32_t n = 150; n < 250; n++) {
		CHECK_EQ(rec_val(&c, n, 0), 8 * (int)((n + 2) % 1000));
	}
}

static void test_stop_reasons(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0001, 10, 0);
	sim_run(&c, &s, 100);
	uint32_t nw = c.n_written;
	s.offset[1] = 2049.0f;	// e.g. foc_dc_cal
	sim_run(&c, &s, 50);
	CHECK_EQ(c.state, MCSA_STATE_DONE);
	CHECK_EQ(c.stop_reason, MCSA_STOP_OFFSETS_CHANGED);
	CHECK_EQ(c.n_written, nw);

	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0001, 10, 0);
	sim_run(&c, &s, 100);
	mcsa_core_stop(&c, MCSA_STOP_CONFIG_CHANGED);
	CHECK_EQ(c.state, MCSA_STATE_DONE);
	CHECK_EQ(c.stop_reason, MCSA_STOP_CONFIG_CHANGED);
	mcsa_core_stop(&c, MCSA_STOP_CMD);
	CHECK_EQ(c.stop_reason, MCSA_STOP_CONFIG_CHANGED);
}

static void test_flags(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0100, 4, 0);
	sim_run(&c, &s, 40);
	uint32_t n0 = c.n_written;
	s.ccr[1] = s.arr - 100;		// Phase b highly modulated for one cycle
	sim_run(&c, &s, 1);
	s.ccr[1] = 0;
	sim_run(&c, &s, 40);
	int marked = 0;
	for (uint32_t n = 0; n < c.n_written; n++) {
		uint16_t f = (uint16_t)rec_val(&c, n, 0);
		if (f & MCSA_FLAG_HIGH_MOD_B) {
			marked++;
			CHECK(n >= n0);
		}
		CHECK((f & (MCSA_FLAG_HIGH_MOD_A | MCSA_FLAG_HIGH_MOD_C)) == 0);
		CHECK(f & MCSA_FLAG_RUNNING);
	}
	CHECK_EQ(marked, 3);

	// Motor not running for one cycle clears the running bit in three records
	s.running = false;
	sim_run(&c, &s, 1);
	s.running = true;
	uint32_t n1 = c.n_written;
	sim_run(&c, &s, 40);
	int not_running = 0;
	for (uint32_t n = n1; n < c.n_written; n++) {
		if (!((uint16_t)rec_val(&c, n, 0) & MCSA_FLAG_RUNNING)) {
			not_running++;
		}
	}
	CHECK_EQ(not_running, 3);
}

static void test_channels_and_saturation(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	s.id = 1.0f;
	s.iq = -2.5f;
	s.vd = 3.0f;
	s.vq = -40.0f;
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0078, 10, 4);
	sim_run_until_done(&c, &s);
	CHECK_EQ(c.nch, 4);
	CHECK_EQ(rec_val(&c, 0, 0), 99);		// 1 A / 80.59 mA * 8 = 99.27
	CHECK_EQ(rec_val(&c, 0, 1), -248);		// -2.5 A
	CHECK_EQ(rec_val(&c, 0, 2), 3000);		// 3 V in mV
	CHECK_EQ(rec_val(&c, 0, 3), -32768);	// -40 V saturates
	CHECK_EQ(c.saturated, 4);
}

static void test_start_rules(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	mcsa_env e = test_env();
	CHECK_EQ(mcsa_core_start(&c, &e, 0, 0x0087, 10, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 3, 0x0087, 10, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0000, 10, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0200, 10, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 0, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 65, 0, 0), MCSA_BAD_ARG);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 5001, 0), MCSA_BAD_ARG);
	e.foc_active = false;
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 0), MCSA_NOT_FOC);
	e.foc_active = true;
	g_sp_busy = true;
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 0), MCSA_BUSY);
	g_sp_busy = false;
	CHECK_EQ(c.session, 0);

	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 77), MCSA_OK);
	CHECK_EQ(c.session, 1);
	CHECK_EQ(c.capacity, 5000);
	CHECK_EQ(c.length, 5000);
	CHECK_EQ(c.angle_pos, 3);
	// Repeated START with the same token keeps the capture
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 77), MCSA_OK);
	CHECK_EQ(c.session, 1);
	// A different START while running is refused
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 78), MCSA_BUSY);
	mcsa_core_stop(&c, MCSA_STOP_CMD);
	CHECK_EQ(mcsa_core_start(&c, &e, 1, 0x0087, 10, 0, 78), MCSA_OK);
	CHECK_EQ(c.session, 2);
}

static void test_sample_print_interplay(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	// Before any capture COMM_SAMPLE_PRINT works as in the stock firmware
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_SEND_LAST));
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_RECORD));

	start_ok(&c, MCSA_MODE_ONESHOT, 0x0087, 10, 100);
	CHECK(!mcsa_core_sample_print_request(&c, MCSA_SP_RECORD));
	CHECK(!mcsa_core_sample_print_request(&c, MCSA_SP_SEND_LAST));
	CHECK(!mcsa_core_sample_print_request(&c, MCSA_SP_OFF));
	sim_run_until_done(&c, &s);

	// Old sample print data is gone; a new recording takes the buffer over
	CHECK(!mcsa_core_sample_print_request(&c, MCSA_SP_SEND_LAST));
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_OFF));
	CHECK_EQ(c.state, MCSA_STATE_DONE);
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_RECORD));
	CHECK_EQ(c.state, MCSA_STATE_INVALID);
	CHECK_EQ(c.stop_reason, MCSA_STOP_SAMPLE_PRINT);
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_SEND_LAST));
}

static void test_isr_timing(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	for (int i = 0; i < 30000; i++) {
		mcsa_core_isr_end(&c, i == 100 ? 12.0e-6f : 8.0e-6f);
	}
	CHECK_NEAR(c.isr_max_idle, 12.0e-6, 1e-9);
	c.sampled_this_isr = true;
	mcsa_core_isr_end(&c, 15.0e-6f);
	CHECK_NEAR(c.isr_max_capture, 15.0e-6, 1e-9);
	CHECK(!c.sampled_this_isr);
}
```

In `main` nach `test_angle_counter();` einfügen:

```c
	test_fixed_point_and_offsets();
	test_warmup();
	test_frequency_response();
	test_angle_alignment();
	test_long_capture_angle_overflow();
	test_ring_oneshot_continuous();
	test_stop_reasons();
	test_flags();
	test_channels_and_saturation();
	test_start_rules();
	test_sample_print_interplay();
	test_isr_timing();
```

- [ ] **Step 2: Test laufen lassen, muss beim Linken scheitern**

Run: `make -C tests/mcsa_core test`
Expected: FAIL, „undefined reference to `mcsa_core_start`“ u. a.

- [ ] **Step 3: Implementierung** — an `motor/mcsa_core.c` anhängen (die Hilfsfunktionen direkt hinter die `#define`-Zeilen, die übrigen Funktionen ans Dateiende):

Hinter die `#define`-Zeilen:

```c
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
```

Ans Dateiende:

```c
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
```

- [ ] **Step 4: Test laufen lassen**

Run: `make -C tests/mcsa_core test`
Expected: `… Pruefungen, 0 Fehler`, Exit-Code 0.

- [ ] **Step 5: Commit**

```bash
git add motor/mcsa_core.c tests/mcsa_core/test_mcsa_core.c
git commit -m "feat(mcsa): CIC-Filter, Ringpuffer, Start/Stop und Sample-Print-Regeln im Kern"
```

---

### Task 3: Protokoll (INFO, START, STOP, STATUS, READ)

**Files:**
- Modify: `motor/mcsa_core.c` (Serialisierung und `mcsa_core_process` anhängen)
- Modify: `tests/mcsa_core/test_mcsa_core.c`

**Interfaces:**
- Consumes: Task 2.
- Produces: `int mcsa_core_process(...)` (0 = nicht unser Paket, < 0 = unser Paket ohne Antwortmöglichkeit, > 0 = Antwortlänge), `mcsa_core_max_records_per_read`. Antwortlayouts exakt wie in der Spec, Abschnitt 7.

- [ ] **Step 1: Tests schreiben** — vor `main` einfügen:

```c
// ---------------- Task 3 ----------------

static uint32_t req(uint8_t *b, uint8_t cmd, const uint8_t *args, uint32_t alen) {
	b[0] = MCSA_MAGIC0;
	b[1] = MCSA_MAGIC1;
	b[2] = MCSA_PROTO_VERSION;
	b[3] = cmd;
	if (alen > 0) {
		memcpy(b + 4, args, alen);
	}
	return 4 + alen;
}

static uint16_t be16(const uint8_t *p) {
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t be32(const uint8_t *p) {
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static int64_t be64(const uint8_t *p) {
	uint64_t v = 0;
	for (int i = 0; i < 8; i++) {
		v = (v << 8) | p[i];
	}
	return (int64_t)v;
}

static float bef(const uint8_t *p) {
	uint32_t u = be32(p);
	float f;
	memcpy(&f, &u, sizeof(f));
	return f;
}

static int process(mcsa_core *c, mcsa_env *e, const uint8_t *rq, uint32_t len, uint8_t *resp) {
	return mcsa_core_process(c, e, rq, len, resp, RESP_MAX);
}

static void build_read(uint8_t *rq, uint32_t *l, uint16_t session, uint32_t start, uint16_t count) {
	uint8_t a[8] = {
		(uint8_t)(session >> 8), (uint8_t)session,
		(uint8_t)(start >> 24), (uint8_t)(start >> 16), (uint8_t)(start >> 8), (uint8_t)start,
		(uint8_t)(count >> 8), (uint8_t)count
	};
	*l = req(rq, MCSA_CMD_READ, a, 8);
}

typedef struct {
	mcsa_core *c;
	sim_t *s;
	uint32_t n;
} hook_ctx;

static void hook_feed(void *arg) {
	hook_ctx *h = (hook_ctx *)arg;
	sim_run(h->c, h->s, h->n);
}

static void test_protocol_framing(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	mcsa_env e = test_env();
	uint8_t rq[64];
	uint8_t rs[RESP_MAX];

	// Other app data is not ours
	const uint8_t other[4] = {0x12, 0x34, 1, 1};
	CHECK_EQ(process(&c, &e, other, 4, rs), 0);
	rq[0] = MCSA_MAGIC0;
	CHECK_EQ(process(&c, &e, rq, 1, rs), 0);

	// Our magic but too short
	rq[1] = MCSA_MAGIC1;
	rq[2] = MCSA_PROTO_VERSION;
	int n = process(&c, &e, rq, 3, rs);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN);
	CHECK_EQ(rs[3], 0x80);
	CHECK_EQ(rs[4], MCSA_BAD_ARG);

	// Wrong version
	uint32_t l = req(rq, MCSA_CMD_INFO, NULL, 0);
	rq[2] = 2;
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_BAD_VERSION);

	// Unknown command
	l = req(rq, 9, NULL, 0);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[3], 0x89);
	CHECK_EQ(rs[4], MCSA_UNKNOWN_CMD);

	// Header carries session and uptime
	CHECK_EQ(rs[0], MCSA_MAGIC0);
	CHECK_EQ(rs[1], MCSA_MAGIC1);
	CHECK_EQ(rs[2], MCSA_PROTO_VERSION);
	CHECK_EQ(be16(rs + 5), 0);
	CHECK_EQ(be32(rs + 7), 1234);

	// Too small response buffer: consumed, no answer
	l = req(rq, MCSA_CMD_INFO, NULL, 0);
	CHECK(mcsa_core_process(&c, &e, rq, l, rs, 20) < 0);
}

static void test_protocol_info(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	mcsa_env e = test_env();
	uint8_t rq[64];
	uint8_t rs[RESP_MAX];

	uint32_t l = req(rq, MCSA_CMD_INFO, NULL, 0);
	int n = process(&c, &e, rq, l, rs);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 32);
	CHECK_EQ(rs[4], MCSA_OK);
	const uint8_t *b = rs + MCSA_RESP_HDR_LEN;
	CHECK_EQ(b[0], 1);
	CHECK_EQ(b[1], 3);
	CHECK_EQ(b[2], 64);
	CHECK_EQ(b[3], 7);
	CHECK_EQ(be32(b + 4), BUF_BYTES);
	CHECK_NEAR(bef(b + 8), 30000.0, 0.0);
	CHECK_NEAR(bef(b + 12), FAC_CUR, 0.0);
	CHECK(memcmp(b + 16, "abc1234-dirty\0\0\0", 16) == 0);
}

static void test_protocol_start_status_stop(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	mcsa_env e = test_env();
	sim_t s = sim_default();
	s.offset[0] = 2047.5f;
	uint8_t rq[64];
	uint8_t rs[RESP_MAX];

	const uint8_t a[10] = {MCSA_MODE_CONTINUOUS, 0x00, 0x87, 10, 0, 0, 0, 0, 0x12, 0x34};
	uint32_t l = req(rq, MCSA_CMD_START, a, 10);
	int n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OK);
	CHECK_EQ(be16(rs + 5), 1);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 8);
	CHECK_EQ(be32(rs + 11), 5000);
	CHECK_EQ(rs[15], 8);
	CHECK_EQ(be16(rs + 16), 0x0087);
	CHECK_EQ(rs[18], 10);

	l = req(rq, MCSA_CMD_START, a, 9);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_BAD_ARG);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN);

	sim_run(&c, &s, 1000);	// 100 outputs, 98 stored

	l = req(rq, MCSA_CMD_STATUS, NULL, 0);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 64);
	const uint8_t *b = rs + MCSA_RESP_HDR_LEN;
	CHECK_EQ(b[0], MCSA_STATE_RUNNING);
	CHECK_EQ(b[1], MCSA_STOP_NONE);
	CHECK_EQ(b[2], MCSA_MODE_CONTINUOUS);
	CHECK_EQ(be16(b + 3), 0x0087);
	CHECK_EQ(b[5], 10);
	CHECK_EQ(b[6], 3);
	CHECK_EQ(b[7], 7);
	CHECK_EQ(be32(b + 8), 5000);
	CHECK_EQ(be32(b + 12), 0);
	CHECK_EQ(be32(b + 16), 98);
	CHECK_EQ(be32(b + 20), 0);
	CHECK_NEAR(bef(b + 24), 30000.0, 0.0);
	CHECK_NEAR(bef(b + 28), (double)FAC_CUR / 8.0, 1e-9);
	CHECK_NEAR(bef(b + 32), 2047.5, 0.0);
	CHECK_NEAR(bef(b + 36), 2048.0, 0.0);
	CHECK_NEAR(bef(b + 40), 2048.0, 0.0);
	CHECK_EQ(be64(b + 44), c.theta_start);
	CHECK_NEAR(bef(b + 52), 0.0, 1e-9);
	CHECK_NEAR(bef(b + 56), 10.0, 1e-3);
	CHECK_EQ(be32(b + 60), 0);

	// STOP with a foreign session
	uint8_t sa[2] = {0, 5};
	l = req(rq, MCSA_CMD_STOP, sa, 2);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_NOT_AVAILABLE);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 5);
	CHECK(mcsa_core_running(&c));

	sa[1] = 1;
	l = req(rq, MCSA_CMD_STOP, sa, 2);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OK);
	CHECK_EQ(be32(rs + 11), 98);
	CHECK_EQ(rs[15], MCSA_STOP_CMD);
	CHECK_EQ(c.state, MCSA_STATE_DONE);
}

static void test_protocol_read(void) {
	mcsa_core c;
	new_core(&c, 800);
	mcsa_env e = test_env();
	sim_t s = sim_default();
	s.ramp = true;
	s.offset[0] = 2000.0f;
	s.f_e = 300.0;
	uint8_t rq[64];
	uint8_t rs[RESP_MAX];
	uint32_t l;
	int n;

	// Before any capture
	build_read(rq, &l, 0, 0, 10);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_NOT_AVAILABLE);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 8);

	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0087, 1, 0);	// Capacity 100
	CHECK_EQ(mcsa_core_max_records_per_read(RESP_MAX, 8), 60);
	sim_run(&c, &s, 82);								// 80 records

	// Too short
	build_read(rq, &l, 1, 0, 10);
	process(&c, &e, rq, l - 1, rs);
	CHECK_EQ(rs[4], MCSA_BAD_ARG);

	// Foreign session
	build_read(rq, &l, 2, 0, 10);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_NOT_AVAILABLE);
	CHECK_EQ(be32(rs + 11), 0);
	CHECK_EQ(be32(rs + 15), 80);

	// Nothing new at the end
	build_read(rq, &l, 1, 80, 10);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OK);
	CHECK_EQ(be16(rs + 15), 0);
	CHECK_EQ(n, MCSA_READ_OVERHEAD);

	// Beyond the end
	build_read(rq, &l, 1, 81, 10);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_NOT_AVAILABLE);

	// Normal read, limited to 60 records
	build_read(rq, &l, 1, 5, 0xFFFF);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OK);
	CHECK_EQ(be32(rs + 11), 5);
	CHECK_EQ(be16(rs + 15), 60);
	CHECK_EQ(rs[17], 8);
	CHECK_EQ(n, MCSA_READ_OVERHEAD + 60 * 8);
	const uint8_t *d = rs + 26;
	for (int r = 0; r < 60; r++) {
		CHECK_EQ((int16_t)be16(d + r * 8), 8 * (5 + r + 2));
	}
	CHECK_EQ(be16(rs + 26 + 480), mcsa_crc16(d, 480));

	// Anchor is the unwrapped angle of record 5
	static int64_t ang[80];
	unwrap_from_start(&c, 3, 80, ang);
	CHECK_EQ(be64(rs + 18), ang[5]);
	CHECK_EQ(be16(d + 6), (uint16_t)(ang[5] & 0xFFFF));

	// Ring overrun: records before n_written - capacity are gone
	sim_run(&c, &s, 100);			// 180 records, oldest 80
	build_read(rq, &l, 1, 50, 10);
	n = process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OVERWRITTEN);
	CHECK_EQ(n, MCSA_RESP_HDR_LEN + 8);
	CHECK_EQ(be32(rs + 11), 80);
	CHECK_EQ(be32(rs + 15), 180);

	// The anchor after the overrun still matches the continuous angle
	build_read(rq, &l, 1, 150, 10);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OK);
	double t_c = 150.0 + 2.0;
	double expected = (double)c.theta_start / 8.0 + 300.0 * t_c / F_ISR * 8192.0;
	CHECK_NEAR((double)be64(rs + 18), expected, 2.0);

	// Overwritten while copying: the hook writes a full ring of new records
	hook_ctx h = {&c, &s, 120};
	e.read_copy_hook = hook_feed;
	e.read_copy_hook_arg = &h;
	build_read(rq, &l, 1, 100, 10);
	process(&c, &e, rq, l, rs);
	CHECK_EQ(rs[4], MCSA_OVERWRITTEN);
	e.read_copy_hook = NULL;
}
```

In `main` nach `test_isr_timing();` einfügen:

```c
	test_protocol_framing();
	test_protocol_info();
	test_protocol_start_status_stop();
	test_protocol_read();
```

- [ ] **Step 2: Test laufen lassen, muss beim Linken scheitern**

Run: `make -C tests/mcsa_core test`
Expected: FAIL, „undefined reference to `mcsa_core_process`“.

- [ ] **Step 3: Implementierung** — Serialisierungshelfer hinter `channel_count` einfügen, `mcsa_core_max_records_per_read` und `mcsa_core_process` ans Dateiende:

Hinter `channel_count`:

```c
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
```

Ans Dateiende:

```c
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
	uint32_t nw = c->n_written;
	uint8_t reason = c->stop_reason;
	unlock_isr(env);

	int32_t i = put_hdr(c, env, r, MCSA_CMD_STOP, st);
	put_u32(r, &i, nw);
	put_u8(r, &i, reason);
	return i;
}

static int cmd_status(mcsa_core *c, const mcsa_env *env, uint8_t *r) {
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
```

- [ ] **Step 4: Test laufen lassen**

Run: `make -C tests/mcsa_core test`
Expected: `… Pruefungen, 0 Fehler`, Exit-Code 0.

- [ ] **Step 5: Commit**

```bash
git add motor/mcsa_core.c tests/mcsa_core/test_mcsa_core.c
git commit -m "feat(mcsa): Protokoll INFO/START/STOP/STATUS/READ mit Winkelanker und CRC"
```

---

### Task 4: Firmware-Einbindung, Build, Speicherprüfung

**Files:**
- Create: `motor/mcsa_capture.h`, `motor/mcsa_capture.c`
- Modify: `motor/mcpwm_foc.c` (Include, Hooks vor `mc_interface_mc_timer_isr` und nach `m_last_adc_isr_duration`)
- Modify: `motor/mc_interface.c`, `motor/mc_interface.h`
- Modify: `comm/commands.c` (Include, `case COMM_CUSTOM_APP_DATA`)
- Modify: `motor/motor.mk`

**Interfaces:**
- Consumes: kompletter Kern.
- Produces: `mcsa_capture_init`, `mcsa_capture_lock/unlock`, `mcsa_capture_isr_angle`, `mcsa_capture_isr_wants_sample`, `mcsa_capture_isr_sample`, `mcsa_capture_isr_end`, `mcsa_capture_process_cmd`, `mcsa_capture_sample_print_request`, `mcsa_capture_config_changed`; `mc_interface_sample_print_busy`.

- [ ] **Step 1: `motor/mcsa_capture.h`**

```c
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

// FOC ISR hooks, motor 1 only
void mcsa_capture_isr_angle(float phase);
bool mcsa_capture_isr_wants_sample(void);
void mcsa_capture_isr_sample(const mcsa_sample_in *in);
void mcsa_capture_isr_end(float isr_duration);

// Returns true if the COMM_CUSTOM_APP_DATA packet belonged to the MCSA capture
bool mcsa_capture_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len));

// COMM_SAMPLE_PRINT interplay; call with mcsa_capture_lock() held
bool mcsa_capture_sample_print_request(debug_sampling_mode mode);

// Configuration written: a running capture ends
void mcsa_capture_config_changed(void);

#endif /* MCSA_CAPTURE_H_ */
```

- [ ] **Step 2: `motor/mcsa_capture.c`**

```c
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

	uint8_t *buf = mempools_get_packet_buffer();
	buf[0] = COMM_CUSTOM_APP_DATA;

	mcsa_capture_lock();
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
```

- [ ] **Step 3: `motor/motor.mk`** — die beiden Dateien in die `CSRC`-Liste aufnehmen:

```make
CSRC += \
	motor/foc_math.c \
	motor/gpdrive.c \
	motor/mc_interface.c \
	motor/mcpwm.c \
	motor/mcpwm_foc.c \
	motor/mcsa_capture.c \
	motor/mcsa_core.c \
	motor/virtual_motor.c
```

- [ ] **Step 4: `motor/mc_interface.c` / `.h` anpassen**

1. Include: nach `#include "events.h"` die Zeile `#include "mcsa_capture.h"` einfügen.
2. Sample-Print-Arrays ersetzen. Alt:

```c
#define ADC_SAMPLE_MAX_LEN		2000
__attribute__((section(".ram4"))) static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
__attribute__((section(".ram4"))) static volatile int8_t m_phase_samples[ADC_SAMPLE_MAX_LEN];
```

Neu:

```c
#define ADC_SAMPLE_MAX_LEN		MCSA_SP_SAMPLE_LEN
// The sample arrays share their memory with the MCSA capture (mcsa_capture.h)
#define m_curr0_samples			(mcsa_capture_shared.sp.curr0)
#define m_curr1_samples			(mcsa_capture_shared.sp.curr1)
#define m_ph1_samples			(mcsa_capture_shared.sp.ph1)
#define m_ph2_samples			(mcsa_capture_shared.sp.ph2)
#define m_ph3_samples			(mcsa_capture_shared.sp.ph3)
#define m_vzero_samples			(mcsa_capture_shared.sp.vzero)
#define m_status_samples		(mcsa_capture_shared.sp.status)
#define m_curr_fir_samples		(mcsa_capture_shared.sp.curr_fir)
#define m_f_sw_samples			(mcsa_capture_shared.sp.f_sw)
#define m_phase_samples			(mcsa_capture_shared.sp.phase)
```

3. Nach `static volatile bool m_sample_is_second_motor;` einfügen:

```c
// Requests to the sample send thread and the last request it has served
static volatile uint32_t m_sample_send_req;
static volatile uint32_t m_sample_send_done;
```

4. In `mc_interface_init()` als erste Anweisung `mcsa_capture_init();` einfügen und nach `m_sample_is_second_motor = false;`:

```c
	m_sample_send_req = 0;
	m_sample_send_done = 0;
```

5. In `mc_interface_set_configuration()` direkt nach `volatile motor_if_state_t *motor = motor_now();`:

```c
	mcsa_capture_config_changed();
```

6. `mc_interface_sample_print_data()` vollständig ersetzen durch:

```c
void mc_interface_sample_print_data(debug_sampling_mode mode, uint16_t len, uint8_t decimation, bool raw,
		void(*reply_func)(unsigned char *data, unsigned int len)) {

	mcsa_capture_lock();

	// The buffer is shared with the MCSA capture
	if (!mcsa_capture_sample_print_request(mode)) {
		mcsa_capture_unlock();
		return;
	}

	if (len > ADC_SAMPLE_MAX_LEN) {
		len = ADC_SAMPLE_MAX_LEN;
	}

	if (mode == DEBUG_SAMPLING_SEND_LAST_SAMPLES) {
		chSysLock();
		m_sample_send_req++;
		chEvtSignalI(sample_send_tp, (eventmask_t) 1);
		chSchRescheduleS();
		chSysUnlock();
	} else {
		m_sample_trigger = -1;
		m_sample_now = 0;
		m_sample_len = len;
		m_sample_int = decimation;
		m_sample_mode = mode;
		m_sample_raw = raw;
		send_func_sample = reply_func;
#ifdef HW_HAS_DUAL_MOTORS
		m_sample_is_second_motor = motor_now() == &m_motor_2;
#endif
	}

	mcsa_capture_unlock();
}

/**
 * True while COMM_SAMPLE_PRINT records or its send thread still reads the buffer.
 */
bool mc_interface_sample_print_busy(void) {
	chSysLock();
	bool busy = m_sample_mode != DEBUG_SAMPLING_OFF || m_sample_send_req != m_sample_send_done;
	chSysUnlock();
	return busy;
}
```

7. In `mc_interface_mc_timer_isr()` an allen vier Stellen, an denen `chEvtSignalI(sample_send_tp, (eventmask_t) 1);` steht, direkt davor (innerhalb `chSysLockFromISR()`) einfügen:

```c
			m_sample_send_req++;
```

8. `sample_send_thread`: die Schleife

```c
	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		int len = 0;
		...
			send_func_sample(buffer, index);
		}
	}
```

so umbauen, dass der bisherige Rumpf (ab `int len = 0;` bis einschließlich der `for (int i = 0;i < len;i++) { ... }`-Schleife) unverändert in einer inneren Schleife steht:

```c
	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		for (;;) {
			chSysLock();
			uint32_t req = m_sample_send_req;
			chSysUnlock();

			if (req == m_sample_send_done) {
				break;
			}

			int len = 0;
			int offset = 0;

			// ... bisheriger Rumpf unverändert (switch und Sendeschleife) ...

			m_sample_send_done = req;
		}
	}
```

9. `motor/mc_interface.h`: nach dem Prototyp von `mc_interface_sample_print_data` einfügen:

```c
bool mc_interface_sample_print_busy(void);
```

- [ ] **Step 5: `motor/mcpwm_foc.c` anpassen**

1. Include nach `#include "mc_interface.h"`: `#include "mcsa_capture.h"`.
2. In `mcpwm_foc_adc_int_handler()` direkt vor

```c
#ifdef HW_HAS_DUAL_MOTORS
	mc_interface_mc_timer_isr(is_second_motor);
#else
	mc_interface_mc_timer_isr(false);
#endif
```

einfügen:

```c
	// MCSA capture: revolution counter and samples (motor 1 only)
#ifdef HW_HAS_DUAL_MOTORS
	const bool mcsa_motor_1 = !is_second_motor;
#else
	const bool mcsa_motor_1 = true;
#endif
	if (mcsa_motor_1) {
		mcsa_capture_isr_angle(motor_now->m_motor_state.phase);

		if (mcsa_capture_isr_wants_sample()) {
			mcsa_sample_in mcsa_in;
			for (int i = 0; i < 3; i++) {
				mcsa_in.adc[i] = motor_now->m_currents_adc[i];
				mcsa_in.offset[i] = conf_now->foc_offsets_current[i];
			}
			mcsa_in.id = motor_now->m_motor_state.id;
			mcsa_in.iq = motor_now->m_motor_state.iq;
			mcsa_in.vd = motor_now->m_motor_state.vd;
			mcsa_in.vq = motor_now->m_motor_state.vq;
			mcsa_in.ccr[0] = TIM1->CCR1;
			mcsa_in.ccr[1] = TIM1->CCR2;
			mcsa_in.ccr[2] = TIM1->CCR3;
			mcsa_in.arr = TIM1->ARR;
			mcsa_in.running = motor_now->m_state == MC_STATE_RUNNING;
			mcsa_capture_isr_sample(&mcsa_in);
		}
	}
```

3. Direkt nach `m_last_adc_isr_duration = timer_seconds_elapsed_since(t_start);` (letzte Zeile der ISR) einfügen:

```c
	mcsa_capture_isr_end(m_last_adc_isr_duration);
```

- [ ] **Step 6: `comm/commands.c` anpassen**

1. Include nach `#include "mc_interface.h"`: `#include "mcsa_capture.h"`.
2. Alt:

```c
	case COMM_CUSTOM_APP_DATA:
		if (appdata_func) {
```

Neu:

```c
	case COMM_CUSTOM_APP_DATA:
		// MCSA capture commands start with their own magic, everything else passes on
		if (mcsa_capture_process_cmd(data, len, reply_func)) {
			break;
		}

		if (appdata_func) {
```

- [ ] **Step 7: Firmware bauen**

Run: `make fw_60 2>&1 | tee build/mcsa-build.log | tail -5; grep -c "warning:" build/mcsa-build.log`
Expected: Build endet mit `Done`, Warnungszähler `0`. (`build/` ist per `.gitignore` ausgeschlossen; vorher `mkdir -p build`.)

- [ ] **Step 8: Speicher prüfen**

Run:

```bash
T=tools/gcc-arm-none-eabi-7-2018-q2-update/bin
$T/arm-none-eabi-objdump -h build/60/60.elf | grep -E ' \.(ram4|bss|data) '
$T/arm-none-eabi-nm -S -t d build/60/60.elf | grep -E 'mcsa_capture_shared|m_core|m_curr0_samples'
$T/arm-none-eabi-objdump -d build/60/60.elf | awk '/<mcsa_core_sample>:/,/^$/' | wc -l
$T/arm-none-eabi-objdump -d build/60/60.elf | awk '/<mcsa_core_update_angle>:/,/^$/' | wc -l
```

Expected: `.ram4` = 0xF0C8 (61 640 B) ≤ 63 488 B; `mcsa_capture_shared` 40 000 B mit Adresse 0x1000xxxx; keine Symbole `m_curr0_samples`; `.bss` wächst gegenüber Stock (0x1b2d8) nur um die Größe von `m_core` und `m_mutex` (< 600 B). Die Befehlszahlen für die ISR-Abschätzung notieren.

- [ ] **Step 9: Kern-Tests erneut**

Run: `make -C tests/mcsa_core test`
Expected: `0 Fehler`.

- [ ] **Step 10: Commit**

```bash
git add motor/mcsa_capture.h motor/mcsa_capture.c motor/motor.mk motor/mc_interface.c motor/mc_interface.h motor/mcpwm_foc.c comm/commands.c
git commit -m "feat(mcsa): Aufnahmemodus in FOC-ISR, Paketpfad und Sample-Print-Puffer eingebunden"
```

---

### Task 5: Testbibliothek und Client-Grundlagen

**Files:**
- Create: `tests/mcsa_core/mcsa_testlib.c`
- Create: `tools/mcsa_capture_client.py`, `tools/test_mcsa_capture_client.py`, `tools/.gitignore`

**Interfaces:**
- Consumes: Kern-API.
- Produces (C, ctypes): `mcsa_tl_new(uint32_t)`, `mcsa_tl_free`, `mcsa_tl_set_offsets(h, float, float, float)`, `mcsa_tl_set_uptime(h, uint32_t)`, `mcsa_tl_set_sample_print_busy(h, int)`, `mcsa_tl_reboot(h)`, `mcsa_tl_feed(h, n, phase*, a0*, a1*, a2*, int running)`, `mcsa_tl_process(h, req, len, resp, resp_max)`.
- Produces (Python): `crc16`, `frame`, `Deframer`, `app_request`, `wrap_packet`, `is_response`, `parse_header`, `mask_channels`, `decode_records`, `unwrap_angle`, `cic_gain`, `record_times`, `Recording`, `McsaClient` (`info`, `status`, `start`, `stop`, `read`), Konstanten.

- [ ] **Step 1: `tests/mcsa_core/mcsa_testlib.c`**

```c
// Shared library around motor/mcsa_core.c for tools/test_mcsa_capture_client.py

#include "mcsa_core.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
	mcsa_core core;
	mcsa_env env;
	int16_t *buf;
	uint32_t buf_bytes;
	float offsets[3];
	int sp_busy;
} mcsa_tl;

static mcsa_tl *g_active;

static bool tl_buffer_free(void) {
	return g_active ? !g_active->sp_busy : true;
}

void *mcsa_tl_new(uint32_t buf_bytes) {
	mcsa_tl *t = calloc(1, sizeof(mcsa_tl));
	t->buf = calloc(buf_bytes / 2 + 1, sizeof(int16_t));
	t->buf_bytes = buf_bytes;
	mcsa_core_init(&t->core, t->buf, buf_bytes);
	t->env.f_isr = 30000.0f;
	t->env.fac_current = 0.0805861f;
	t->env.pole_pairs = 7;
	t->env.foc_active = true;
	t->env.git_hash = "testlib";
	t->env.uptime_ms = 0;
	t->env.buffer_free = tl_buffer_free;
	for (int k = 0; k < 3; k++) {
		t->offsets[k] = 2048.0f;
	}
	g_active = t;
	return t;
}

void mcsa_tl_free(void *h) {
	mcsa_tl *t = (mcsa_tl *)h;
	if (g_active == t) {
		g_active = NULL;
	}
	free(t->buf);
	free(t);
}

void mcsa_tl_set_offsets(void *h, float o0, float o1, float o2) {
	mcsa_tl *t = (mcsa_tl *)h;
	t->offsets[0] = o0;
	t->offsets[1] = o1;
	t->offsets[2] = o2;
}

void mcsa_tl_set_uptime(void *h, uint32_t ms) {
	((mcsa_tl *)h)->env.uptime_ms = ms;
}

void mcsa_tl_set_sample_print_busy(void *h, int busy) {
	((mcsa_tl *)h)->sp_busy = busy;
}

void mcsa_tl_reboot(void *h) {
	mcsa_tl *t = (mcsa_tl *)h;
	memset(t->buf, 0, t->buf_bytes);
	mcsa_core_init(&t->core, t->buf, t->buf_bytes);
	t->env.uptime_ms = 0;
}

void mcsa_tl_feed(void *h, uint32_t n, const float *phase, const float *a0, const float *a1,
		const float *a2, int running) {
	mcsa_tl *t = (mcsa_tl *)h;
	for (uint32_t i = 0; i < n; i++) {
		mcsa_core_update_angle(&t->core, phase[i]);
		if (mcsa_core_running(&t->core)) {
			mcsa_sample_in in;
			memset(&in, 0, sizeof(in));
			in.adc[0] = a0[i];
			in.adc[1] = a1[i];
			in.adc[2] = a2[i];
			for (int k = 0; k < 3; k++) {
				in.offset[k] = t->offsets[k];
			}
			in.arr = 2800;
			in.running = running != 0;
			mcsa_core_sample(&t->core, &in);
		}
		mcsa_core_isr_end(&t->core, 1.0e-5f);
	}
}

int mcsa_tl_process(void *h, const uint8_t *req, uint32_t len, uint8_t *resp, uint32_t resp_max) {
	mcsa_tl *t = (mcsa_tl *)h;
	g_active = t;
	return mcsa_core_process(&t->core, &t->env, req, len, resp, resp_max);
}
```

Run: `make -C tests/mcsa_core lib`
Expected: `libmcsa_core.so` entsteht ohne Warnungen.

- [ ] **Step 2: Client-Grundlagen** — `tools/mcsa_capture_client.py` (Teil 1; Aufnahme, CLI und Ausgabe folgen in Task 6):

```python
#!/usr/bin/env python3
"""Referenzclient für den MCSA-Aufnahmemodus der VESC-Firmware (Branch feat/mcsa-capture-6.02).

Spricht einen VESC per USB direkt an oder einen Controller dahinter per COMM_FORWARD_CAN.
Protokoll: docs/mcsa_capture_protocol.md. Abhängigkeiten: pyserial, numpy.

Beispiele:
  python3 mcsa_capture_client.py /dev/ttyACM0 info
  python3 mcsa_capture_client.py /dev/ttyACM0 --can 3 capture --decimation 10 --out /tmp/m3
  python3 mcsa_capture_client.py /dev/ttyACM0 stream --decimation 10 --seconds 20 --out /tmp/m1
  python3 mcsa_capture_client.py /dev/ttyACM0 --can 2 offset-check
"""
import argparse
import json
import random
import struct
import sys
import time

import numpy as np

COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36

MAGIC = b'MC'
PROTO_VERSION = 1
CMD_INFO, CMD_START, CMD_STOP, CMD_STATUS, CMD_READ = 1, 2, 3, 4, 5
OK, BUSY, BAD_ARG, NOT_AVAILABLE, OVERWRITTEN, UNKNOWN_CMD, BAD_VERSION, NOT_FOC = range(8)
STATUS_NAMES = ['OK', 'BUSY', 'BAD_ARG', 'NOT_AVAILABLE', 'OVERWRITTEN', 'UNKNOWN_CMD',
                'BAD_VERSION', 'NOT_FOC']
STATE_IDLE, STATE_RUNNING, STATE_DONE, STATE_INVALID = range(4)
STATE_NAMES = ['IDLE', 'RUNNING', 'DONE', 'INVALID']
STOP_NAMES = ['NONE', 'LENGTH', 'STOP_CMD', 'OFFSETS_CHANGED', 'CONFIG_CHANGED',
              'INDEX_LIMIT', 'ANGLE_LIMIT', 'SAMPLE_PRINT']
MODE_ONESHOT, MODE_CONTINUOUS = 1, 2

CHANNELS = ['ia', 'ib', 'ic', 'id', 'iq', 'vd', 'vq', 'angle', 'flags']
MASK_DEFAULT = 0x0087
CIC_ORDER = 3
RESP_MAX = 511              # PACKET_MAX_PL_LEN - 1 (Paket-ID)
READ_OVERHEAD = 28
HDR_FMT = '>2sBBBHI'        # Magic, Version, Befehl|0x80, Status, Sitzung, Laufzeit ms
INFO_FMT = '>BBBBIff16s'
STATUS_FMT = '>BBBHBBBIIIIff3fqffI'
STATUS_KEYS = ['state', 'stop_reason', 'mode', 'mask', 'decimation', 'cic_order', 'pole_pairs',
               'capacity', 'length', 'n_written', 'oldest', 'f_isr', 'amp_per_unit',
               'offset0', 'offset1', 'offset2', 'theta_start', 'isr_us_idle_max',
               'isr_us_capture_max', 'saturated']


class ProtocolError(Exception):
    pass


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(payload):
    n = len(payload)
    head = bytes([2, n]) if n <= 255 else bytes([3, n >> 8, n & 0xFF])
    return head + payload + struct.pack('>H', crc16(payload)) + b'\x03'


class Deframer:
    """Sammelt Bytes und liefert vollständige, CRC-geprüfte VESC-Pakete."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)

    def packets(self):
        out = []
        while True:
            while self.buf and self.buf[0] not in (2, 3):
                self.buf.pop(0)
            if len(self.buf) < 3:
                break
            if self.buf[0] == 2:
                n, hl = self.buf[1], 2
            else:
                n, hl = (self.buf[1] << 8) | self.buf[2], 3
            total = hl + n + 3
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[hl:hl + n])
            crc = struct.unpack('>H', self.buf[hl + n:hl + n + 2])[0]
            if self.buf[total - 1] == 3 and crc == crc16(payload):
                out.append(payload)
                del self.buf[:total]
            else:
                self.buf.pop(0)
        return out


def app_request(cmd, args=b''):
    return MAGIC + bytes([PROTO_VERSION, cmd]) + args


def wrap_packet(app_payload, can_id=None):
    pl = bytes([COMM_CUSTOM_APP_DATA]) + app_payload
    if can_id is not None:
        pl = bytes([COMM_FORWARD_CAN, can_id]) + pl
    return pl


def is_response(packet, cmd):
    return (len(packet) >= 12 and packet[0] == COMM_CUSTOM_APP_DATA and packet[1:3] == MAGIC
            and packet[4] == (cmd | 0x80))


class SerialTransport:
    """USB-Verbindung zum Bridge-VESC; mit can_id wird jede Anfrage weitergeleitet."""

    def __init__(self, port, can_id=None, timeout=1.0):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.01, exclusive=True)
        self.ser.reset_input_buffer()
        self.deframer = Deframer()
        self.can_id = can_id
        self.timeout = timeout

    def request(self, app_payload, cmd):
        self.ser.write(frame(wrap_packet(app_payload, self.can_id)))
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.timeout:
            chunk = self.ser.read(4096)
            if chunk:
                self.deframer.feed(chunk)
                for p in self.deframer.packets():
                    if is_response(p, cmd):
                        return p[1:]
        raise TimeoutError(f'keine Antwort auf Befehl {cmd}')

    def close(self):
        self.ser.close()


def parse_header(resp):
    if len(resp) < 11 or resp[:2] != MAGIC:
        raise ProtocolError('Antwort zu kurz oder fremd')
    _, version, cmd, status, session, uptime = struct.unpack(HDR_FMT, resp[:11])
    return {'version': version, 'cmd': cmd & 0x7F, 'status': status, 'session': session,
            'uptime_ms': uptime}, resp[11:]


def mask_channels(mask):
    return [name for i, name in enumerate(CHANNELS) if mask & (1 << i)]


def decode_records(data, nch):
    return np.frombuffer(data, dtype='>i2').astype(np.int16).reshape(-1, nch)


def unwrap_angle(angle_u16, anchor_q13):
    """Wickelt die 16-Bit-Winkel eines Blocks ab dem absoluten Anker ab (Einheit 1/8192 U.)."""
    a = np.asarray(angle_u16, dtype=np.int64) & 0xFFFF
    if len(a) == 0:
        return np.zeros(0, dtype=np.int64)
    if (anchor_q13 & 0xFFFF) != a[0]:
        raise ProtocolError('Winkelanker passt nicht zum ersten Datensatz')
    d = ((np.diff(a) + 32768) % 65536) - 32768
    return anchor_q13 + np.concatenate(([0], np.cumsum(d)))


def cic_gain(f, decimation, f_isr):
    """|H(f)| des CIC-Filters 3. Ordnung."""
    x = np.pi * np.asarray(f, dtype=float) / f_isr
    s = np.sin(x)
    small = np.abs(s) < 1e-12
    h = np.sin(decimation * x) / (decimation * np.where(small, 1.0, s))
    return np.where(small, 1.0, np.abs(h) ** CIC_ORDER)


def record_times(indices, decimation, f_isr):
    """Zeit jedes Datensatzes (Fenstermitte) ab dem ersten ISR-Durchlauf der Aufnahme, in s."""
    n = np.asarray(indices, dtype=float)
    return ((n + CIC_ORDER) * decimation - 1 - CIC_ORDER * (decimation - 1) / 2.0) / f_isr


class Recording:
    """Gesammelte Blöcke einer Aufnahme mit Lückenbuchhaltung."""

    def __init__(self, mask, decimation, f_isr):
        self.mask = mask
        self.channels = mask_channels(mask)
        self.nch = len(self.channels)
        self.decimation = decimation
        self.f_isr = f_isr
        self.blocks = []    # (start, records, angle_q13 oder None)
        self.gaps = []      # [von, bis) fehlende Indizes

    def add_block(self, start, data, anchor):
        recs = decode_records(data, self.nch)
        ang = None
        if 'angle' in self.channels:
            col = recs[:, self.channels.index('angle')].astype(np.uint16)
            ang = unwrap_angle(col, anchor)
        self.blocks.append((start, recs, ang))

    def arrays(self):
        if not self.blocks:
            return (np.zeros(0, dtype=np.int64), np.zeros((0, self.nch), dtype=np.int16),
                    np.zeros(0, dtype=np.int64) if 'angle' in self.channels else None)
        idx = np.concatenate([np.arange(s, s + len(r), dtype=np.int64) for s, r, _ in self.blocks])
        recs = np.concatenate([r for _, r, _ in self.blocks])
        ang = None
        if 'angle' in self.channels:
            ang = np.concatenate([a for _, _, a in self.blocks])
        return idx, recs, ang


class McsaClient:
    def __init__(self, transport, retries=3):
        self.t = transport
        self.retries = retries
        self.last_uptime = None
        self.reboots = 0

    def _request(self, cmd, args=b''):
        err = None
        for _ in range(self.retries):
            try:
                resp = self.t.request(app_request(cmd, args), cmd)
            except TimeoutError as e:
                err = e
                continue
            hdr, body = parse_header(resp)
            if self.last_uptime is not None and hdr['uptime_ms'] < self.last_uptime:
                self.reboots += 1
            self.last_uptime = hdr['uptime_ms']
            if hdr['status'] == BAD_VERSION:
                raise ProtocolError('Firmware spricht eine andere Protokollversion')
            return hdr, body
        raise err

    def info(self):
        hdr, b = self._request(CMD_INFO)
        if hdr['status'] != OK:
            raise ProtocolError(f"INFO: {STATUS_NAMES[hdr['status']]}")
        proto, order, rmax, pp, buf_bytes, f_isr, fac, git = struct.unpack(
            INFO_FMT, b[:struct.calcsize(INFO_FMT)])
        return {'protocol': proto, 'cic_order': order, 'decimation_max': rmax, 'pole_pairs': pp,
                'buffer_bytes': buf_bytes, 'f_isr': f_isr, 'fac_current': fac,
                'git_hash': git.rstrip(b'\0').decode(errors='replace')}

    def status(self):
        hdr, b = self._request(CMD_STATUS)
        if hdr['status'] != OK:
            raise ProtocolError(f"STATUS: {STATUS_NAMES[hdr['status']]}")
        s = dict(zip(STATUS_KEYS, struct.unpack(STATUS_FMT, b[:struct.calcsize(STATUS_FMT)])))
        s['offsets'] = [s.pop('offset0'), s.pop('offset1'), s.pop('offset2')]
        s['session'] = hdr['session']
        s['uptime_ms'] = hdr['uptime_ms']
        return s

    def start(self, mode, mask, decimation, length=0, token=None):
        token = token if token is not None else random.randint(1, 0xFFFF)
        args = struct.pack('>BHBIH', mode, mask, decimation, length, token)
        hdr, b = self._request(CMD_START, args)
        if hdr['status'] != OK:
            raise ProtocolError(f"START abgelehnt: {STATUS_NAMES[hdr['status']]}")
        capacity, rec_bytes, mask_r, dec_r = struct.unpack('>IBHB', b[:8])
        return {'session': hdr['session'], 'capacity': capacity, 'rec_bytes': rec_bytes,
                'mask': mask_r, 'decimation': dec_r, 'token': token}

    def stop(self, session):
        hdr, b = self._request(CMD_STOP, struct.pack('>H', session))
        if len(b) < 5:
            raise ProtocolError(f"STOP: {STATUS_NAMES[hdr['status']]}")
        n_written, reason = struct.unpack('>IB', b[:5])
        return hdr['status'], n_written, reason

    def read(self, session, start, count=0xFFFF):
        for _ in range(self.retries):
            hdr, b = self._request(CMD_READ, struct.pack('>HIH', session, start, count))
            if hdr['status'] != OK:
                if len(b) < 8:
                    raise ProtocolError(f"READ: {STATUS_NAMES[hdr['status']]}")
                oldest, n_written = struct.unpack('>II', b[:8])
                return hdr['status'], {'oldest': oldest, 'n_written': n_written,
                                       'session': hdr['session']}
            start_r, n, rec_bytes, anchor = struct.unpack('>IHBq', b[:15])
            data = b[15:15 + n * rec_bytes]
            crc = struct.unpack('>H', b[15 + n * rec_bytes:17 + n * rec_bytes])[0]
            if crc16(data) == crc:
                return OK, {'start': start_r, 'count': n, 'rec_bytes': rec_bytes,
                            'anchor': anchor, 'data': data}
        raise ProtocolError('READ: CRC-Fehler trotz Wiederholung')
```

`tools/.gitignore`:

```
__pycache__/
```

- [ ] **Step 3: Tests schreiben** — `tools/test_mcsa_capture_client.py`:

```python
"""Tests für tools/mcsa_capture_client.py; Ende-zu-Ende gegen den Firmware-Kern per ctypes.

Aufruf aus dem Repo-Wurzelverzeichnis: python3 -m pytest -p no:cacheprovider tools -q
"""
import ctypes
import os
import subprocess

import numpy as np
import pytest

import mcsa_capture_client as mc

HERE = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.normpath(os.path.join(HERE, '..', 'tests', 'mcsa_core'))
LIB_PATH = os.path.join(TEST_DIR, 'libmcsa_core.so')
FP = ctypes.POINTER(ctypes.c_float)


@pytest.fixture(scope='session')
def lib():
    subprocess.run(['make', '-s', '-C', TEST_DIR, 'lib'], check=True)
    L = ctypes.CDLL(LIB_PATH)
    L.mcsa_tl_new.restype = ctypes.c_void_p
    L.mcsa_tl_new.argtypes = [ctypes.c_uint32]
    L.mcsa_tl_free.argtypes = [ctypes.c_void_p]
    L.mcsa_tl_set_offsets.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    L.mcsa_tl_set_uptime.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    L.mcsa_tl_set_sample_print_busy.argtypes = [ctypes.c_void_p, ctypes.c_int]
    L.mcsa_tl_reboot.argtypes = [ctypes.c_void_p]
    L.mcsa_tl_feed.argtypes = [ctypes.c_void_p, ctypes.c_uint32, FP, FP, FP, FP, ctypes.c_int]
    L.mcsa_tl_process.restype = ctypes.c_int
    L.mcsa_tl_process.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32,
                                  ctypes.c_char_p, ctypes.c_uint32]
    return L


class Sim:
    """ISR-Eingänge: Rampe auf Shunt a (exakt prüfbar), Kosinus auf b, Drehwinkel mit f_e."""

    def __init__(self, lib, h, f_e=200.0, f_isr=30000.0, levels=None):
        self.lib, self.h, self.f_e, self.f_isr = lib, h, f_e, f_isr
        self.levels = levels
        self.t = 0

    def feed(self, n, running=1):
        i = np.arange(self.t, self.t + n)
        th = 2 * np.pi * self.f_e * i / self.f_isr
        phase = (np.mod(th + np.pi, 2 * np.pi) - np.pi).astype(np.float32)
        if self.levels is None:
            a0 = (2048 + (i % 100)).astype(np.float32)
            a1 = np.round(2048 + 500 * np.cos(th)).astype(np.float32)
            a2 = np.full(n, 2048, dtype=np.float32)
        else:
            a0, a1, a2 = (np.full(n, v, dtype=np.float32) for v in self.levels)
        p = lambda a: a.ctypes.data_as(FP)
        self.lib.mcsa_tl_feed(self.h, n, p(phase), p(a0), p(a1), p(a2), running)
        self.t += n


class LibTransport:
    """Simulierter Bridge-Weg: Framing, optionale CAN-Weiterleitung, Kern statt Firmware."""

    def __init__(self, lib, h, sim, can_id=None, feed_per_request=0):
        self.lib, self.h, self.sim = lib, h, sim
        self.can_id = can_id
        self.feed_per_request = feed_per_request
        self.uptime = 1000
        self.lose_response_of = set()
        self.requests = 0

    def request(self, app_payload, cmd):
        self.requests += 1
        d = mc.Deframer()
        d.feed(mc.frame(mc.wrap_packet(app_payload, self.can_id)))
        (pkt,) = d.packets()
        if pkt[0] == mc.COMM_FORWARD_CAN:
            assert pkt[1] == self.can_id
            pkt = pkt[2:]
        assert pkt[0] == mc.COMM_CUSTOM_APP_DATA
        if self.feed_per_request:
            self.sim.feed(self.feed_per_request)
        self.uptime += 1
        self.lib.mcsa_tl_set_uptime(self.h, self.uptime)
        req = bytes(pkt[1:])
        resp = ctypes.create_string_buffer(mc.RESP_MAX)
        n = self.lib.mcsa_tl_process(self.h, req, len(req), resp, mc.RESP_MAX)
        assert n > 0
        d.feed(mc.frame(bytes([mc.COMM_CUSTOM_APP_DATA]) + resp.raw[:n]))
        (back,) = d.packets()
        if cmd in self.lose_response_of:
            self.lose_response_of.discard(cmd)
            raise TimeoutError('simulierter Antwortverlust')
        assert mc.is_response(back, cmd)
        return back[1:]


@pytest.fixture
def rig(lib):
    handles = []

    def make(buf_bytes=40000, can_id=None, feed_per_request=0, f_e=200.0, levels=None):
        h = lib.mcsa_tl_new(buf_bytes)
        handles.append(h)
        sim = Sim(lib, h, f_e=f_e, levels=levels)
        t = LibTransport(lib, h, sim, can_id, feed_per_request)
        return mc.McsaClient(t), t, sim

    yield make
    for h in handles:
        lib.mcsa_tl_free(h)


def test_crc16_vector():
    assert mc.crc16(b'123456789') == 0x31C3


def test_frame_roundtrip_short_and_long():
    d = mc.Deframer()
    short = bytes(range(10))
    long = bytes(i & 0xFF for i in range(300))
    d.feed(b'\x00\x07' + mc.frame(short) + mc.frame(long))
    assert d.packets() == [short, long]


def test_unwrap_angle_forward_backward_and_anchor_check():
    anchor = 3 * 65536 + 100
    vals = [(anchor + k * 20000) & 0xFFFF for k in range(10)]
    assert list(mc.unwrap_angle(vals, anchor)) == [anchor + k * 20000 for k in range(10)]
    anchor = -5 * 8192
    vals = [(anchor - k * 30000) & 0xFFFF for k in range(10)]
    assert list(mc.unwrap_angle(vals, anchor)) == [anchor - k * 30000 for k in range(10)]
    with pytest.raises(mc.ProtocolError):
        mc.unwrap_angle([1, 2, 3], 0)


def test_cic_gain_and_record_times():
    assert mc.cic_gain(0.0, 10, 30000.0) == pytest.approx(1.0)
    assert mc.cic_gain(3000.0, 10, 30000.0) == pytest.approx(0.0, abs=1e-12)
    x = np.pi * 400.0 / 30000.0
    assert mc.cic_gain(400.0, 10, 30000.0) == pytest.approx(abs(np.sin(10 * x) / (10 * np.sin(x))) ** 3)
    assert mc.record_times([0], 1, 30000.0)[0] == pytest.approx(2 / 30000.0)
    assert mc.record_times([0], 10, 30000.0)[0] == pytest.approx((30 - 1 - 13.5) / 30000.0)


def test_info_and_status(rig):
    client, _, _ = rig()
    info = client.info()
    assert info['protocol'] == 1 and info['cic_order'] == 3 and info['pole_pairs'] == 7
    assert info['buffer_bytes'] == 40000 and info['git_hash'] == 'testlib'
    s = client.status()
    assert s['state'] == mc.STATE_IDLE and s['session'] == 0
```

Run: `python3 -m pytest -p no:cacheprovider tools -q`
Expected: 5 Tests grün.

- [ ] **Step 4: Commit**

```bash
git add tests/mcsa_core/mcsa_testlib.c tools/mcsa_capture_client.py tools/test_mcsa_capture_client.py tools/.gitignore
git commit -m "feat(mcsa): Testbibliothek und Client-Grundlagen (Framing, Protokoll, Dekodierung)"
```

---

### Task 6: Client-Aufnahmen, CLI, Ende-zu-Ende-Tests

**Files:**
- Modify: `tools/mcsa_capture_client.py` (Methoden an `McsaClient`, Ausgabe, CLI anhängen)
- Modify: `tools/test_mcsa_capture_client.py`

**Interfaces:**
- Consumes: Task 5.
- Produces: `McsaClient.capture(mask, decimation, length, poll, wait)` → `(info, status, Recording)`, `McsaClient.stream(mask, decimation, seconds, idle, clock, wait)` → `(info, status, Recording)`, `McsaClient.offset_check(decimation, length, threshold, wait)` → dict, `to_table`, `write_outputs`, `main`.

- [ ] **Step 1: Tests schreiben** — an `tools/test_mcsa_capture_client.py` anhängen:

```python
NOWAIT = lambda dt: None


def ramp_ok(rec, idx):
    """Shunt a ist eine Rampe (Zählwert mod 100): Werte passen exakt zum Index."""
    ia = rec[:, 0].astype(int) // 8
    assert np.all(rec[:, 0] % 8 == 0)
    base = (ia[0] - idx[0]) % 100
    assert np.all(ia == (base + idx) % 100)


@pytest.mark.parametrize('can_id', [None, 3])
def test_oneshot_capture_direct_and_via_can(rig, can_id):
    client, t, sim = rig(can_id=can_id, feed_per_request=2000)
    info, s, rec = client.capture(mask=0x0083, decimation=1, length=500, wait=NOWAIT)
    idx, recs, ang = rec.arrays()
    assert s['state'] == mc.STATE_DONE and s['stop_reason'] == 1
    assert list(idx) == list(range(500)) and rec.gaps == []
    ramp_ok(recs, idx)
    # Winkel: 200 Hz bei 30 kHz, R = 1
    step = 200.0 / 30000.0 * 8192.0
    assert np.allclose(np.diff(ang), step, atol=1.0)


def test_stream_with_overruns_keeps_exact_indices_and_angle(rig):
    # Ring 800 B, Maske ia+Winkel -> 200 Datensätze; 150 neue je Anfrage, höchstens 120 je READ
    client, t, sim = rig(buf_bytes=800, feed_per_request=150)
    ticks = iter(range(10 ** 6))
    info, s, rec = client.stream(mask=0x0081, decimation=1, seconds=60,
                                 clock=lambda: next(ticks), wait=NOWAIT)
    idx, recs, ang = rec.arrays()
    assert rec.gaps, 'der Test muss Überläufe erzeugen'
    final = s['n_written']
    covered = set(idx.tolist())
    for a, b in rec.gaps:
        assert a < b
        assert not covered.intersection(range(a, b))
        covered.update(range(a, b))
    assert covered == set(range(final))
    ramp_ok(recs, idx)
    step = 200.0 / 30000.0 * 8192.0
    assert np.allclose(ang - ang[0], (idx - idx[0]) * step, atol=2.0)


def test_lost_start_response_does_not_restart(rig):
    client, t, sim = rig(feed_per_request=2000)
    t.lose_response_of.add(mc.CMD_START)
    info, s, rec = client.capture(mask=0x0001, decimation=1, length=100, wait=NOWAIT)
    assert s['session'] == 1
    assert len(rec.arrays()[0]) == 100


def test_lost_read_response_is_repeated(rig):
    client, t, sim = rig(feed_per_request=2000)
    st = client.start(mc.MODE_ONESHOT, 0x0001, 1, 50)
    client.status()
    t.lose_response_of.add(mc.CMD_READ)
    status, r = client.read(st['session'], 0)
    assert status == mc.OK and r['count'] == 50


def test_reboot_is_detected_and_stream_aborts(rig, lib):
    client, t, sim = rig(feed_per_request=100)
    ticks = iter(range(10 ** 6))

    def clock():
        k = next(ticks)
        if k == 20:
            lib.mcsa_tl_reboot(t.h)
            t.uptime = 0
        return k

    with pytest.raises(mc.ProtocolError, match='Reboot'):
        client.stream(mask=0x0001, decimation=1, seconds=1000, clock=clock, wait=NOWAIT)
    assert client.reboots == 1


def test_start_refused_while_sample_print_busy(rig, lib):
    client, t, sim = rig()
    lib.mcsa_tl_set_sample_print_busy(t.h, 1)
    with pytest.raises(mc.ProtocolError, match='BUSY'):
        client.start(mc.MODE_ONESHOT, 0x0087, 10)


def test_offset_check_at_standstill(rig, lib):
    client, t, sim = rig(feed_per_request=5000, f_e=0.0, levels=(2048, 2049, 2046))
    lib.mcsa_tl_set_offsets(t.h, 2047.5, 2048.0, 2046.0)
    r = client.offset_check(decimation=10, length=200, threshold=1.5, wait=NOWAIT)
    assert r['residual_counts'] == pytest.approx([0.5, 1.0, 0.0], abs=1e-9)
    assert r['warning'] is False
    r = client.offset_check(decimation=10, length=200, threshold=0.75, wait=NOWAIT)
    assert r['warning'] is True


def test_write_outputs(rig, tmp_path):
    client, t, sim = rig(feed_per_request=2000)
    info, s, rec = client.capture(mask=0x0187, decimation=10, length=100, wait=NOWAIT)
    prefix = str(tmp_path / 'm1')
    mc.write_outputs(prefix, info, s, rec, {'mode': 'capture'}, client)
    lines = open(prefix + '.csv').read().splitlines()
    assert lines[0] == 'index,t_s,ia_A,ib_A,ic_A,angle_el_rev,flags'
    assert len(lines) == 101
    import json
    meta = json.load(open(prefix + '.json'))
    assert meta['records'] == 100 and meta['gaps'] == []
    assert meta['cic']['group_delay_s'] == pytest.approx(13.5 / 30000.0)
```

- [ ] **Step 2: Tests laufen lassen, müssen scheitern**

Run: `python3 -m pytest -p no:cacheprovider tools -q`
Expected: neue Tests FAIL mit `AttributeError: 'McsaClient' object has no attribute 'capture'` bzw. `write_outputs`.

- [ ] **Step 3: Implementierung** — an `McsaClient` anhängen:

```python
    def _check_session(self, expected, actual):
        if actual != expected:
            if self.reboots:
                raise ProtocolError('Controller-Reboot erkannt, Aufnahme verloren')
            raise ProtocolError('Sitzung hat gewechselt (anderer Client?)')

    def _fetch(self, rec, session, start, end):
        nxt = start
        while nxt < end:
            status, r = self.read(session, nxt)
            if status == OK:
                if r['count'] == 0:
                    raise ProtocolError('READ lieferte vor dem Ende keine Daten')
                rec.add_block(r['start'], r['data'], r['anchor'])
                nxt += r['count']
            elif status == OVERWRITTEN:
                rec.gaps.append((nxt, r['oldest']))
                nxt = r['oldest']
            else:
                self._check_session(session, r['session'])
                raise ProtocolError(f'READ: {STATUS_NAMES[status]}')
        return rec

    def capture(self, mask=MASK_DEFAULT, decimation=10, length=0, poll=0.05, wait=time.sleep):
        """Einzelaufnahme: starten, auf Ende warten, vollständig abholen."""
        info = self.info()
        st = self.start(MODE_ONESHOT, mask, decimation, length)
        n = length or st['capacity']
        timeout = (n + CIC_ORDER) * decimation / info['f_isr'] + 5.0
        t0 = time.monotonic()
        while True:
            s = self.status()
            self._check_session(st['session'], s['session'])
            if s['state'] != STATE_RUNNING:
                break
            if time.monotonic() - t0 > timeout:
                raise TimeoutError('Einzelaufnahme wird nicht fertig')
            wait(poll)
        rec = Recording(mask, decimation, info['f_isr'])
        self._fetch(rec, st['session'], 0, s['n_written'])
        return info, s, rec

    def stream(self, mask=MASK_DEFAULT, decimation=10, seconds=10.0, idle=0.002,
               clock=time.monotonic, wait=time.sleep):
        """Daueraufnahme: fortlaufend abholen, nach `seconds` stoppen und den Rest holen."""
        info = self.info()
        st = self.start(MODE_CONTINUOUS, mask, decimation)
        session = st['session']
        rec = Recording(mask, decimation, info['f_isr'])
        t_end = clock() + seconds
        nxt, final = 0, None
        while final is None or nxt < final:
            if final is None and clock() >= t_end:
                status, final, _ = self.stop(session)
                if status != OK:
                    self._check_session(session, -1)
                continue
            status, r = self.read(session, nxt)
            if status == OK:
                if r['count'] == 0:
                    wait(idle)
                    continue
                rec.add_block(r['start'], r['data'], r['anchor'])
                nxt += r['count']
            elif status == OVERWRITTEN:
                rec.gaps.append((nxt, r['oldest']))
                nxt = r['oldest']
            else:
                self._check_session(session, r['session'] if self.reboots == 0 else -1)
                raise ProtocolError(f'READ: {STATUS_NAMES[status]}')
        s = self.status()
        return info, s, rec

    def offset_check(self, decimation=10, length=1000, threshold=1.5, wait=time.sleep):
        """Rest-Offset der drei Shunts im Stillstand, in ADC-Stufen."""
        info, s, rec = self.capture(0x0007, decimation, length, wait=wait)
        _, recs, _ = rec.arrays()
        resid = recs.astype(float).mean(axis=0) / 8.0
        return {'offsets': s['offsets'], 'residual_counts': resid.tolist(),
                'residual_amp': (resid * info['fac_current']).tolist(),
                'threshold_counts': threshold,
                'warning': bool(np.any(np.abs(resid) > threshold))}
```

Hinter die Klasse `McsaClient` anhängen:

```python
def to_table(rec, status):
    idx, recs, ang = rec.arrays()
    cols = {'index': idx, 't_s': record_times(idx, rec.decimation, rec.f_isr)}
    for j, ch in enumerate(rec.channels):
        x = recs[:, j]
        if ch in ('ia', 'ib', 'ic', 'id', 'iq'):
            cols[ch + '_A'] = x.astype(float) * status['amp_per_unit']
        elif ch in ('vd', 'vq'):
            cols[ch + '_V'] = x.astype(float) / 1000.0
        elif ch == 'angle':
            cols['angle_el_rev'] = ang.astype(float) / 8192.0
        elif ch == 'flags':
            cols['flags'] = x.astype(np.uint16)
    return cols


def _json_default(o):
    return o.item() if hasattr(o, 'item') else str(o)


def write_outputs(prefix, info, status, rec, params, client):
    cols = to_table(rec, status)
    names = list(cols)
    table = np.column_stack([np.asarray(cols[n], dtype=float) for n in names]) if len(cols['index']) \
        else np.zeros((0, len(names)))
    np.savetxt(prefix + '.csv', table, delimiter=',', header=','.join(names), comments='', fmt='%.9g')
    meta = {
        'info': info,
        'status': status,
        'params': params,
        'channels': rec.channels,
        'records': int(len(cols['index'])),
        'gaps': [list(g) for g in rec.gaps],
        'reboots': client.reboots,
        'cic': {'order': CIC_ORDER, 'decimation': rec.decimation, 'f_isr': rec.f_isr,
                'group_delay_s': CIC_ORDER * (rec.decimation - 1) / 2.0 / rec.f_isr},
    }
    with open(prefix + '.json', 'w') as f:
        json.dump(meta, f, indent=2, default=_json_default)


def _summary(s, rec):
    idx = rec.arrays()[0]
    missing = sum(b - a for a, b in rec.gaps)
    return (f"{len(idx)} Datensätze, {len(rec.gaps)} Lücken ({missing} fehlend), "
            f"Stoppgrund {STOP_NAMES[s['stop_reason']]}, ISR max {s['isr_us_capture_max']:.1f} µs "
            f"(ohne Aufnahme {s['isr_us_idle_max']:.1f} µs), Sättigungen {s['saturated']}")


def main(argv=None):
    p = argparse.ArgumentParser(description='MCSA-Aufnahme vom VESC (COMM_CUSTOM_APP_DATA)')
    p.add_argument('port', help='z. B. /dev/ttyACM0')
    p.add_argument('--can', type=int, default=None, help='CAN-ID des Controllers hinter der Bridge')
    p.add_argument('--timeout', type=float, default=1.0, help='Antwort-Zeitlimit je Anfrage in s')
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('info')
    sub.add_parser('status')
    pc = sub.add_parser('capture', help='Einzelaufnahme')
    pc.add_argument('--mask', type=lambda v: int(v, 0), default=MASK_DEFAULT)
    pc.add_argument('--decimation', type=int, default=10)
    pc.add_argument('--length', type=int, default=0, help='0 = ganzer Puffer')
    pc.add_argument('--out', required=True, help='Präfix für .csv und .json')
    ps = sub.add_parser('stream', help='Daueraufnahme')
    ps.add_argument('--mask', type=lambda v: int(v, 0), default=MASK_DEFAULT)
    ps.add_argument('--decimation', type=int, default=10)
    ps.add_argument('--seconds', type=float, required=True)
    ps.add_argument('--out', required=True)
    po = sub.add_parser('offset-check', help='Rest-Offset im Stillstand')
    po.add_argument('--decimation', type=int, default=10)
    po.add_argument('--length', type=int, default=1000)
    po.add_argument('--threshold', type=float, default=1.5, help='Warnschwelle in ADC-Stufen')
    args = p.parse_args(argv)

    transport = SerialTransport(args.port, args.can, args.timeout)
    client = McsaClient(transport)
    try:
        if args.cmd == 'info':
            print(json.dumps(client.info(), indent=2))
        elif args.cmd == 'status':
            print(json.dumps(client.status(), indent=2, default=_json_default))
        elif args.cmd == 'capture':
            info, s, rec = client.capture(args.mask, args.decimation, args.length)
            write_outputs(args.out, info, s, rec, vars(args), client)
            print(_summary(s, rec), file=sys.stderr)
        elif args.cmd == 'stream':
            info, s, rec = client.stream(args.mask, args.decimation, args.seconds)
            write_outputs(args.out, info, s, rec, vars(args), client)
            print(_summary(s, rec), file=sys.stderr)
        elif args.cmd == 'offset-check':
            r = client.offset_check(args.decimation, args.length, args.threshold)
            print(json.dumps(r, indent=2))
            return 1 if r['warning'] else 0
    finally:
        transport.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 4: Tests laufen lassen**

Run: `python3 -m pytest -p no:cacheprovider tools -q`
Expected: alle Tests grün (5 aus Task 5 + 9 neue = 14, mit Parametrisierung).

- [ ] **Step 5: Commit**

```bash
git add tools/mcsa_capture_client.py tools/test_mcsa_capture_client.py
git commit -m "feat(mcsa): Client mit Einzel-/Daueraufnahme, Offset-Pruefung und CLI"
```

---

### Task 7: Dokumentation, Abschlussprüfung, Review, Push

**Files:**
- Create: `docs/mcsa_capture_protocol.md`, `docs/mcsa_capture_flashing.md`

- [ ] **Step 1: `docs/mcsa_capture_protocol.md`** schreiben. Inhalt (vollständig, Deutsch):
  - Zweck und Kurzüberblick (aus Spec 1, 2).
  - Transport: `COMM_CUSTOM_APP_DATA` (36), `COMM_FORWARD_CAN` (34), VESC-Rahmen mit Beispiel in Hex für `INFO` direkt (`02 05 24 4D 43 01 01 <crc16> 03`) und über CAN-ID 3 (`02 07 22 03 24 4D 43 01 01 <crc16> 03`), CRC-Testvektor.
  - Anfrage- und Antwortkopf als Bytetabelle (Offset, Größe, Feld).
  - Je Befehl eine Bytetabelle für Argumente und Antwortkörper, exakt wie Spec 7, mit Offsets.
  - Status-, Zustands-, Stoppgrund-, Modustabellen.
  - Kanaltabelle mit Einheiten (Spec 6.3), Umrechnung in A (`× FAC_CURRENT/8`) und V (`/1000`).
  - Winkel: Kodierung, Abwicklung ab Anker (Python-Snippet aus `unwrap_angle`), mechanischer Winkel `θ_el / Polpaare`.
  - Zeitachse `t_n`, Gruppenlaufzeit, |H(f)| mit Bandtabelle (Spec 5.3), Hinweise zur Auswertung (Spec 5.4).
  - Ablauf Einzelaufnahme, Daueraufnahme (Pull, eine offene Anfrage, `OVERWRITTEN`), idempotenter `START`, Reboot-Erkennung.
  - Zusammenspiel mit `COMM_SAMPLE_PRINT` (Spec 6.2).
  - Durchsatz: USB, CAN (eine Anfrage je Bridge, R ≥ 20 für Daueraufnahme über CAN bei 4 Kanälen).

- [ ] **Step 2: `docs/mcsa_capture_flashing.md`** schreiben. Inhalt (vollständig, Deutsch):
  - Voraussetzungen: App auf motorpi im Leerlauf (`curl localhost:8000/api/v1/tests/status/`), Port frei (`sudo fuser /dev/ttyACM0`), Netzteil an, Motoren stehen.
  - Bauen: `make fw_60` → `build/60/60.bin`, Git-Hash notieren.
  - Sicherung vorher je Controller: `vesc_tool --offscreen --vescPort /dev/ttyACM0 [--canFwd ID] --getMcConf mcN.xml --getAppConf appN.xml` (Bridge ohne `--canFwd`).
  - Prüfen, dass das `vesc_tool` auf dem Pi `--uploadFirmware` kennt (`vesc_tool --help | grep uploadFirmware`); sonst VESC Tool 6.02 am Laptop per USB.
  - Schritt 1 Bridge: `vesc_tool --offscreen --vescPort /dev/ttyACM0 --uploadFirmware 60.bin`; warten bis neu gestartet; `mcsa_capture_client.py /dev/ttyACM0 info` zeigt den Git-Hash; `COMM_FW_VERSION` meldet weiter 6.02, HW „60“; Sample-Print-Test mit `scripts/mcsa/vesc_sampleprint.py` (motorpi) funktioniert; Konfiguration unverändert (`--getMcConf` vergleichen).
  - Schritt 2 Prüfungen an der Bridge: `offset-check`, Einzelaufnahme im Stillstand, Lauf mit `spin_capture.py` (motorpi) parallel nicht möglich → Ablauf: Motor per App/Skript drehen, im zweiten Schritt Aufnahme; ISR-Dauer aus `status` mit Standardmaske und `--mask 0x1FF`.
  - Schritt 3 CAN-Knoten einzeln: `--canFwd 2 --uploadFirmware 60.bin`, danach `info` mit `--can 2`, dann 3 und 4.
  - Schritt 4 Daueraufnahme: Bridge per USB mit R = 10, CAN-Knoten mit R = 20.
  - Rückweg: Stock 6.02 bauen (`git worktree add ../bldc-stock-602 origin/release_6_02 && make -C ../bldc-stock-602 fw_60`) oder die vorher verwendete Firmware; genauso hochladen; Konfiguration bleibt bei gleicher Version erhalten, sonst aus der Sicherung zurückspielen.
  - Client auf dem Pi: Datei per `scp` kopieren (Pi hat keinen GitHub-Zugang), Ausführung im Wegwerf-Container wie die motorpi-Skripte (`docker run --rm --network none --device /dev/ttyACM0 -v "$PWD:/mcsa:ro" -v /tmp/mcsa_out:/out --entrypoint python motor-web /mcsa/mcsa_capture_client.py /dev/ttyACM0 info`).

- [ ] **Step 3: Vollständige Prüfung**

Run:

```bash
make -C tests/mcsa_core clean all
python3 -m pytest -p no:cacheprovider tools -q
make fw_60 2>&1 | tee build/mcsa-build.log | tail -3; grep -c "warning:" build/mcsa-build.log
git status --short
```

Expected: C-Tests `0 Fehler`, alle Python-Tests grün, Build `Done` mit 0 Warnungen, Arbeitsverzeichnis sauber bis auf Build-Artefakte (ignoriert).

- [ ] **Step 4: Abschluss-Review** — Codex (`codex exec -s read-only`) prüft den gesamten Branch-Diff gegen Spec und Plan; Befunde einarbeiten, Tests erneut laufen lassen.

- [ ] **Step 5: Commit und Push**

```bash
git add docs/mcsa_capture_protocol.md docs/mcsa_capture_flashing.md
git commit -m "docs(mcsa): Protokollbeschreibung und Flash-Anleitung"
git push -u aviou feat/mcsa-capture-6.02
```
