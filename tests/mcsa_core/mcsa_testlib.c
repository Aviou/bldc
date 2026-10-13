// Shared library around motor/mcsa_core.c for mcsa_client/test_mcsa_capture_client.py

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
