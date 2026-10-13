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

static __attribute__((unused)) mcsa_env test_env(void) {
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
