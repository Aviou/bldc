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

int main(void) {
	test_crc16();
	test_angle_counter();
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

	printf("%d Pruefungen, %d Fehler\n", g_checks, g_failed);
	return g_failed == 0 ? 0 : 1;
}
