/*
 * CRC, wheel speed (M/T), Kalman filter, brake measurement, calibration.
 * SPDX-License-Identifier: MIT
 */
#include "harness.h"

/* OR-61: CRC-16/CCITT-FALSE check value */
void test_crc(void)
{
	const uint8_t v[] = "123456789";

	CHECK_EQ(odo_crc16(v, 9U), 0x29B1);
	CHECK_EQ(odo_crc16(v, 0U), 0xFFFF);
}

static void wheel_mt_speed(void)
{
	odo_wheel_t w;
	odo_wheel_raw_t in = {1000U, 5000U};
	const float d = 0.65f;
	const float edge = ODO_PI * d / 400.0f;

	odo_wheel_init(&w, d, &in, 5000U);
	CHECK_NEAR(w.edge_m, edge, 1e-7);
	CHECK(odo_wheel_at_rest(&w));
	/* first edge after init: bounded by the tick */
	in.count = 1002U;
	in.t_edge_us = 9000U;
	odo_wheel_update(&w, &in, 15000U);
	CHECK_NEAR(w.v, 2.0f * edge / ODO_TS, 1e-4);
	CHECK(w.seen);
	/* M/T: 40 edges in 8 ms */
	in.count = 1042U;
	in.t_edge_us = 17000U;
	odo_wheel_update(&w, &in, 25000U);
	CHECK_NEAR(w.v, 40.0f * edge / 0.008f, 1e-3);
	CHECK(!odo_wheel_at_rest(&w));
	/* two edges at the same microsecond: dt clamped to 1 us */
	in.count = 1043U;
	odo_wheel_update(&w, &in, 35000U);
	CHECK(w.v > 1000.0f);
}

static void wheel_decay_and_rest(void)
{
	odo_wheel_t w;
	odo_wheel_raw_t in = {0U, 0U};

	odo_wheel_init(&w, 0.65f, &in, 0U);
	in.count = 10U;
	in.t_edge_us = 9000U;
	odo_wheel_update(&w, &in, 10000U); /* first */
	in.count = 20U;
	in.t_edge_us = 19000U;
	odo_wheel_update(&w, &in, 20000U);
	const float v0 = w.v;

	/* no new edge: the speed is bounded by one edge over the elapsed time */
	odo_wheel_update(&w, &in, 120000U);
	CHECK(w.v < v0);
	CHECK_NEAR(w.v, w.edge_m / (0.101f + 1e-6f), 1e-4);
	/* a later tick with a larger bound keeps the lower value */
	const float v1 = w.v;

	odo_wheel_update(&w, &in, 130000U);
	CHECK(w.v <= v1);
	/* after 0.3 s without an edge: rest */
	odo_wheel_update(&w, &in, 19000U + ODO_STAND_US + 1U);
	CHECK(odo_wheel_at_rest(&w));
	/* counter wrap is harmless */
	odo_wheel_init(&w, 0.65f, &(odo_wheel_raw_t){0xFFFFFFFEU, 0U}, 0U);
	w.seen = true;
	in.count = 2U;
	in.t_edge_us = 4000U;
	odo_wheel_update(&w, &in, 10000U);
	CHECK_NEAR(w.v, 4.0f * w.edge_m / 0.004f, 1e-3);
}

void test_wheel(void)
{
	wheel_mt_speed();
	wheel_decay_and_rest();
}

void test_kf(void)
{
	odo_kf_t k;

	odo_kf_init(&k, 0.3f);
	CHECK_NEAR(k.x[1], 0.3f, 1e-7);
	/* prediction: v += (f - c) T, c += g (w - b) T */
	odo_kf_predict(&k, 1.3f, 0.001f);
	CHECK_NEAR(k.x[0], 0.01f, 1e-6);
	CHECK_NEAR(k.x[1], 0.3f + ODO_G * 0.001f * ODO_TS, 1e-6);
	CHECK(k.p[0][1] < 0.0f); /* v and c become anti-correlated */
	const float p00 = k.p[0][0];

	odo_kf_update(&k, 0U, 0.0f, 0.01f);
	CHECK(k.p[0][0] < p00);
	CHECK(k.x[0] < 0.01f);
	/* an offset update moves c toward z */
	odo_kf_update(&k, 1U, 0.5f, 0.02f);
	CHECK(k.x[1] > 0.3f);
	odo_kf_set_speed(&k, 7.0f, 0.1f);
	CHECK_NEAR(k.x[0], 7.0f, 1e-6);
	CHECK_NEAR(k.p[0][0], 0.01f, 1e-7);
	CHECK_NEAR(k.p[0][1], 0.0f, 1e-9);
	CHECK_NEAR(k.p[2][0], 0.0f, 1e-9);
	/* covariance stays symmetric */
	for (int i = 0; i < 3000; i++) {
		odo_kf_predict(&k, 0.2f, 0.0f);
		odo_kf_update(&k, 0U, 7.0f, 0.02f);
	}
	CHECK_NEAR(k.p[0][1], k.p[1][0], 1e-6);
	CHECK_NEAR(k.p[1][2], k.p[2][1], 1e-6);
	CHECK_NEAR(k.x[0], 7.0f, 0.01); /* the filter follows the wheel */
}

/* inertial path (a wheel slid): v_wheel unused */
static bool bstep(odo_brake_t *b, float v, float fmc, bool rest)
{
	const odo_brake_in_t in = {v, fmc, rest};

	return odo_brake_step(b, &in);
}

/* OR-21..OR-23: brake integration, standstill and boundary correction */
static void brake_constant_offset_is_removed(void)
{
	odo_brake_t b;
	const float v0 = 15.0f;
	const float a = -2.5f;
	const float off = 0.06f; /* offset error: the inertial speed is too high by off * t */
	float v = v0;
	float t = 0.0f;
	int fin_at = -1;

	odo_brake_start(&b, v0, 1.0f);
	for (int k = 0; k < 2000 && fin_at < 0; k++) {
		const float vt = (v0 + a * t) > 0.0f ? (v0 + a * t) : 0.0f;
		const bool moving = vt > 0.0f;

		t += ODO_TS;
		v = (v0 + a * t > 0.0f) ? (v0 + a * t) : 0.0f;
		const float vi = v + off * (t < 6.0f ? t : 6.0f); /* inertial integral with the offset */

		if (bstep(&b, vi, moving ? a : 0.0f, !moving)) {
			fin_at = k;
		}
	}
	CHECK(fin_at > 0);
	CHECK(b.done);
	CHECK(!b.active);
	CHECK_NEAR(b.result_d, v0 * v0 / (2.0f * 2.5f), 0.03); /* 45 m */
	CHECK_NEAR(b.result_t, 6.0f, 0.02);
	CHECK_NEAR(b.result_r, off * 6.0f, 0.03);
	/* inactive: nothing happens */
	CHECK(!bstep(&b, 0.0f, 0.0f, true));
}

static void brake_branches(void)
{
	odo_brake_t b;

	/* onset fraction clamps */
	odo_brake_start(&b, 1.0f, -0.5f);
	CHECK_NEAR(b.w, 0.0f, 1e-9);
	odo_brake_start(&b, 1.0f, 2.0f);
	CHECK_NEAR(b.w, 1.0f, 1e-9);
	odo_brake_start(&b, 10.0f, 0.4f);
	(void)bstep(&b, 10.0f, -2.0f, false);
	CHECK_NEAR(b.d, 10.0f * 0.004f, 1e-5); /* only the part after the edge */
	CHECK_NEAR(b.t, 0.004f, 1e-7);
	/* quiet and wheels at rest, but the inertial speed is high: wait (speed check) ... */
	for (int k = 0; k < 40; k++) {
		CHECK(!bstep(&b, 0.8f, 0.1f, true));
	}
	CHECK_EQ(b.quiet, 40);
	/* ... negative side of the window too */
	CHECK(!bstep(&b, -0.8f, -0.1f, true));
	/* ... until the fallback after ODO_STAND_MAX quiet ticks */
	int k = 0;

	while (!bstep(&b, 0.8f, 0.1f, true)) {
		k++;
	}
	CHECK_EQ(b.quiet, ODO_STAND_MAX);
	/* quiet but wheels still turning: no standstill */
	odo_brake_start(&b, 1.0f, 1.0f);
	for (int i = 0; i < 50; i++) {
		CHECK(!bstep(&b, 0.0f, 0.0f, false));
	}
	/* deceleration resumes: the quiet run restarts */
	CHECK(!bstep(&b, 0.0f, 0.5f, false));
	CHECK_EQ(b.quiet, 0);
	/* quiet from the first tick: T clamps to one tick; small negative speed accepted */
	odo_brake_start(&b, 0.0f, 1.0f);
	for (int i = 0; i < 29; i++) {
		CHECK(!bstep(&b, -0.1f, 0.0f, true));
	}
	CHECK(bstep(&b, -0.1f, 0.0f, true));
	CHECK_NEAR(b.result_t, ODO_TS, 1e-7);
	/* saturation of the quiet counter */
	b.active = true;
	b.quiet = 0xFFFFU;
	(void)bstep(&b, 0.9f, 0.0f, false);
	CHECK_EQ(b.quiet, 0xFFFF);
}

void test_brake(void)
{
	brake_constant_offset_is_removed();
	brake_branches();
}

/* OR-40, OR-41: calibration over two marks */
void test_cal(void)
{
	odo_cal_t c;
	odo_sample_t s;

	memset(&s, 0, sizeof(s));
	odo_cal_start(&c, 100000U); /* 100 m */
	s.inputs = ODO_IN_MARK;     /* mark already active at the start: not an edge */
	CHECK(!odo_cal_step(&c, &s));
	CHECK_EQ(c.state, ODO_CAL_ARMED);
	s.inputs = 0U;
	CHECK(!odo_cal_step(&c, &s));
	s.inputs = ODO_IN_MARK;
	s.wheel[0].count = 1000U;
	s.wheel[1].count = 2000U;
	CHECK(!odo_cal_step(&c, &s));
	CHECK_EQ(c.state, ODO_CAL_RUNNING);
	CHECK(!odo_cal_step(&c, &s)); /* held high: no edge */
	s.inputs = 0U;
	CHECK(!odo_cal_step(&c, &s));
	/* 100 m with D = 0.640 m: 100 / (pi 0.64) * 400 = 19894 edges; axle 1 gives D = 0.5 m */
	s.wheel[0].count = 1000U + 19894U;
	s.wheel[1].count = 2000U + 25465U;
	s.inputs = ODO_IN_MARK;
	CHECK(odo_cal_step(&c, &s));
	CHECK_NEAR(c.diam[0], 0.640f, 0.0001);
	CHECK_NEAR(c.diam[1], 0.500f, 0.001);
	CHECK_EQ(c.ok_mask, 1);
	CHECK_EQ(c.state, ODO_CAL_IDLE);
	/* idle: further marks do nothing */
	s.inputs = 0U;
	(void)odo_cal_step(&c, &s);
	s.inputs = ODO_IN_MARK;
	CHECK(!odo_cal_step(&c, &s));
	/* no movement between the marks: dn = 0 -> rejected; too large -> rejected */
	odo_cal_start(&c, 100000U);
	s.inputs = 0U;
	(void)odo_cal_step(&c, &s);
	s.inputs = ODO_IN_MARK;
	(void)odo_cal_step(&c, &s);
	s.inputs = 0U;
	(void)odo_cal_step(&c, &s);
	s.inputs = ODO_IN_MARK;
	s.wheel[1].count += 10000U; /* D = 1.27 m */
	CHECK(odo_cal_step(&c, &s));
	CHECK_EQ(c.ok_mask, 0);
	CHECK_NEAR(c.diam[0], 0.0f, 1e-9);
}
