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

// ---------------- Final review fixes ----------------

static bool buf_all_zero(void) {
	for (uint32_t i = 0; i < BUF_BYTES / 2; i++) {
		if (g_buf[i] != 0) {
			return false;
		}
	}
	return true;
}

static void test_sample_print_takeover_clears_mcsa_data(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	s.ramp = true;
	s.offset[0] = 2000.0f;
	s.f_e = 150.0;
	start_ok(&c, MCSA_MODE_ONESHOT, 0x0087, 1, 0);
	sim_run_until_done(&c, &s);
	CHECK(!buf_all_zero());

	// The first sample print recording after a capture must not leave MCSA bytes behind
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_RECORD));
	CHECK(buf_all_zero());

	// Later requests while sample print owns the buffer do not clear it again
	g_buf[0] = 123;
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_RECORD));
	CHECK(mcsa_core_sample_print_request(&c, MCSA_SP_SEND_LAST));
	CHECK_EQ(g_buf[0], 123);
}

static void test_angle_step_limit(void) {
	// Unwrapping needs less than 4 electrical revolutions between records
	const double fe[3] = {2000.0, -2000.0, 1800.0};
	for (int d = 0; d < 3; d++) {
		mcsa_core c;
		new_core(&c, BUF_BYTES);
		sim_t s = sim_default();
		s.f_e = fe[d];
		start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0080, 64, 0);
		sim_run(&c, &s, 64 * 20);
		if (fabs(fe[d]) * 64.0 / F_ISR >= 4.0) {
			CHECK_EQ(c.state, MCSA_STATE_DONE);
			CHECK_EQ(c.stop_reason, MCSA_STOP_ANGLE_STEP);
			CHECK_EQ(c.n_written, 1);
		} else {
			CHECK(mcsa_core_running(&c));
			CHECK_EQ(c.n_written, 18);
		}
	}
}

// ---------------- ISR cost: division without __aeabi_ldivmod ----------------

static int64_t ref_rdiv64(int64_t n, int64_t d) {
	return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d);
}

static void test_rdiv64_matches_reference(void) {
	const uint32_t ds[6] = {1, 8, 1000, 3375, 250047, 262144};
	const int64_t ns[12] = {0, 1, -1, 7, -7, 131072, -131072, 8589934591LL, -8589934591LL,
			(((int64_t)1) << 62) - 1, -((((int64_t)1) << 62) - 1), 4611686018427387904LL / 3};
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 12; j++) {
			CHECK_EQ(mcsa_rdiv64(ns[j], ds[i]), ref_rdiv64(ns[j], ds[i]));
		}
	}
	// Pseudo-random values across the full range used by the filter
	uint64_t x = 0x9E3779B97F4A7C15ull;
	for (int k = 0; k < 20000; k++) {
		x ^= x << 13;
		x ^= x >> 7;
		x ^= x << 17;
		int64_t n = (int64_t)(x >> 1) - (int64_t)(((uint64_t)1) << 62);
		uint32_t d = (uint32_t)(x % 262144u) + 1u;
		if (mcsa_rdiv64(n, d) != ref_rdiv64(n, d)) {
			CHECK_EQ(mcsa_rdiv64(n, d), ref_rdiv64(n, d));
			break;
		}
	}
	g_checks++;
}

// ---------------- ISR overload guard ----------------

static void test_isr_overload_stops_capture(void) {
	mcsa_core c;
	new_core(&c, BUF_BYTES);
	sim_t s = sim_default();
	start_ok(&c, MCSA_MODE_CONTINUOUS, 0x0087, 10, 0);
	sim_run(&c, &s, 100);
	CHECK(mcsa_core_running(&c));

	// 90 % of the 33.3 us cycle at 30 kHz is the limit; 29 us is fine
	c.sampled_this_isr = true;
	mcsa_core_isr_end(&c, 29.0e-6f);
	CHECK(mcsa_core_running(&c));

	// One ISR above the limit ends the capture before the next one
	c.sampled_this_isr = true;
	mcsa_core_isr_end(&c, 31.0e-6f);
	CHECK_EQ(c.state, MCSA_STATE_DONE);
	CHECK_EQ(c.stop_reason, MCSA_STOP_ISR_OVERLOAD);
	CHECK_NEAR(c.isr_max_capture, 31.0e-6, 1e-9);

	// Long ISRs without a capture never stop anything
	mcsa_core_isr_end(&c, 40.0e-6f);
	CHECK_EQ(c.stop_reason, MCSA_STOP_ISR_OVERLOAD);
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
	test_protocol_framing();
	test_protocol_info();
	test_protocol_start_status_stop();
	test_protocol_read();
	test_sample_print_takeover_clears_mcsa_data();
	test_angle_step_limit();
	test_rdiv64_matches_reference();
	test_isr_overload_stops_capture();

	printf("%d Pruefungen, %d Fehler\n", g_checks, g_failed);
	return g_failed == 0 ? 0 : 1;
}
