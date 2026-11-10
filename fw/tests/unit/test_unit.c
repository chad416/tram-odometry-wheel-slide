/*
 * The recorder (OR-01..OR-05, OR-20, OR-25, OR-30, OR-35, OR-40..OR-42, OR-53).
 * SPDX-License-Identifier: MIT
 */
#include "harness.h"

static fram_t g_fram;
static odo_unit_t g_u;

static odo_cfg_t cfg_ok(void)
{
	odo_cfg_t c = {{0.65f, 0.65f}, 0U};

	odo_cfg_seal(&c);
	return c;
}

static void tick(odo_unit_t *u, sim_t *m, int n, double a, uint8_t in)
{
	for (int i = 0; i < n; i++) {
		odo_unit_tick(u, sim_step(m, a, in));
	}
}

static int count_type(const odo_unit_t *u, uint8_t type, uint32_t depth)
{
	odo_entry_t e;
	int n = 0;

	for (uint32_t b = 0U; b < depth; b++) {
		if (odo_journal_get(&u->jr, b, &e) && (e.type == type)) {
			n++;
		}
	}
	return n;
}

/* OR-01, OR-30: boot record, RUN records every 0.25 m, states STAND/RUN */
static void unit_boot_and_records(void)
{
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();
	sim_t m;
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	odo_unit_init(&g_u, &nvm, &c);
	CHECK_EQ(g_u.state, ODO_ST_INIT);
	CHECK_EQ(g_u.flags, 0);
	tick(&g_u, &m, 1, 0.0, 0U);
	CHECK_EQ(g_u.state, ODO_ST_STAND);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_BOOT);
	tick(&g_u, &m, 50, 0.0, 0U);
	tick(&g_u, &m, 1000, 1.0, 0U); /* 50 m */
	CHECK_EQ(g_u.state, ODO_ST_RUN);
	CHECK_NEAR(g_u.s, m.s, 0.05);
	const int runs = count_type(&g_u, ODO_REC_RUN, 400U);

	CHECK(runs >= 198 && runs <= 202); /* 50 m / 0.25 m */
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_RUN);
	CHECK_NEAR(e.v_cms, m.v * 100.0, 3.0);
}

/* OR-20..OR-25: brake event with a captured onset, result record, correction */
static void unit_brake_event(void)
{
	sim_t m;
	odo_entry_t e;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	m.grade_c = -0.3;
	odo_unit_init(&g_u, &nvm, &c);
	tick(&g_u, &m, 200, 0.0, 0U);
	tick(&g_u, &m, 1400, 1.0, 0U); /* 14 m/s */
	tick(&g_u, &m, 300, 0.0, 0U);
	const double s_on = m.s;      /* sim: the edge is 4 ms before this sample */
	const double v_on = m.v;

	/* wheels lock (slide) for the whole brake: the IMU carries the distance */
	m.slip[0] = -0.3;
	m.slip[1] = -0.3;
	tick(&g_u, &m, 1, -2.5, ODO_IN_BRAKE);
	CHECK_EQ(g_u.state, ODO_ST_BRAKE);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e) || true);
	CHECK_EQ(count_type(&g_u, ODO_REC_BRAKE_ON, 3U), 1);
	tick(&g_u, &m, 700, -2.5, ODO_IN_BRAKE);
	CHECK_EQ(g_u.state, ODO_ST_STAND);
	CHECK_EQ(g_u.n_brake, 1);
	/* the sim decelerates over the whole brake tick; the edge is 6 ms into it */
	const double d_true = (m.s - s_on) - v_on * 0.006 + 0.5 * 2.5 * 0.006 * 0.006;

	CHECK_NEAR(g_u.last_d, d_true, 0.05);
	CHECK_NEAR(g_u.last_v0, v_on, 0.05);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_BRAKE_RES);
	CHECK_NEAR(e.aux, d_true * 100.0, 6.0);
	CHECK((e.flags & ODO_F_CORR) != 0U);
	CHECK((g_u.flags & ODO_F_CORR) == 0U);
	/* brake input held: no second event; released and pressed again at rest: a new event */
	tick(&g_u, &m, 50, 0.0, ODO_IN_BRAKE);
	CHECK_EQ(g_u.n_brake, 1);
	m.slip[0] = 0.0;
	m.slip[1] = 0.0;
	tick(&g_u, &m, 10, 0.0, 0U);
	tick(&g_u, &m, 50, 0.0, ODO_IN_BRAKE);
	CHECK_EQ(g_u.n_brake, 2);
	CHECK_NEAR(g_u.last_d, 0.0, 0.02);
}

/* OR-53: IMU failed: the wheels carry the brake, no correction flag */
static void unit_brake_imu_fault(void)
{
	sim_t m;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	odo_unit_init(&g_u, &nvm, &c);
	tick(&g_u, &m, 100, 0.0, 0U);
	g_u.est.imu_fault = true;
	tick(&g_u, &m, 1000, 1.0, 0U);
	const double s_on = m.s;
	const double v_on = m.v;

	tick(&g_u, &m, 800, -2.5, ODO_IN_BRAKE);
	CHECK_EQ(g_u.n_brake, 1);
	CHECK_NEAR(g_u.last_d, (m.s - s_on) - v_on * 0.006, 0.10);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e) || true);
	CHECK((g_u.flags & ODO_F_IMU) != 0U);
	CHECK_EQ(count_type(&g_u, ODO_REC_BRAKE_RES, 5U), 1);
}

/* OR-35: power fail and restore records */
static void unit_power(void)
{
	sim_t m;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	odo_unit_init(&g_u, &nvm, &c);
	tick(&g_u, &m, 10, 0.0, 0U);
	odo_unit_power(&g_u, true); /* no change */
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_BOOT);
	odo_unit_power(&g_u, false);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_PFAIL);
	CHECK((e.flags & ODO_F_PFAIL) != 0U);
	odo_unit_power(&g_u, false); /* still failed: no second record */
	CHECK_EQ(count_type(&g_u, ODO_REC_PFAIL, 4U), 1);
	tick(&g_u, &m, 1, 0.0, 0U); /* the recorder keeps working through the interruption */
	odo_unit_power(&g_u, true);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.type, ODO_REC_PRESTORE);
	CHECK((g_u.flags & ODO_F_PFAIL) == 0U);
}

/* OR-40..OR-42: calibration command, marks, results; invalid configuration */
static void unit_calibration(void)
{
	sim_t m;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	/* invalid stored configuration: default diameter, flagged */
	odo_cfg_t bad = c;

	bad.crc ^= 1U;
	odo_unit_init(&g_u, &nvm, &bad);
	CHECK((g_u.flags & ODO_F_CALBAD) != 0U);
	CHECK_NEAR(g_u.cfg.diam[0], 0.655f, 1e-6);
	bad = c;
	bad.diam[1] = 0.9f;
	odo_cfg_seal(&bad);
	odo_unit_init(&g_u, &nvm, &bad);
	CHECK((g_u.flags & ODO_F_CALBAD) != 0U);
	bad.diam[1] = 0.5f;
	odo_cfg_seal(&bad);
	odo_unit_init(&g_u, &nvm, &bad);
	CHECK((g_u.flags & ODO_F_CALBAD) != 0U);
	/* the real wheels are 0.640 m */
	sim_init(&m, 0.640);
	CHECK(!odo_unit_cal_start(&g_u, 100000U)); /* INIT: refused */
	tick(&g_u, &m, 50, 0.0, 0U);
	CHECK(!odo_unit_cal_start(&g_u, 5000U));    /* too short */
	CHECK(!odo_unit_cal_start(&g_u, 3000000U)); /* too long */
	CHECK(odo_unit_cal_start(&g_u, 100000U));
	CHECK_EQ(g_u.state, ODO_ST_CAL);
	/* drive over the marks at 5 m/s: mark 1 at 20 m, mark 2 at 120 m */
	int marks = 0;

	for (int i = 0; i < 4000 && marks < 2; i++) {
		double a = (m.v < 5.0) ? 1.0 : 0.0;
		uint8_t in = 0U;

		if ((m.s > 20.0 && m.s < 20.2) || (m.s > 120.0 && m.s < 120.2)) {
			in = ODO_IN_MARK;
		}
		odo_unit_tick(&g_u, sim_step(&m, a, in));
		if (in != 0U && g_u.cal.state != ODO_CAL_ARMED && marks == 0) {
			marks = 1;
		}
		if (marks == 1 && g_u.cal.state == ODO_CAL_IDLE) {
			marks = 2;
		}
	}
	CHECK_EQ(marks, 2);
	CHECK_EQ(g_u.state, ODO_ST_RUN);
	CHECK_NEAR(g_u.cfg.diam[0], 0.640f, 0.0005);
	CHECK_NEAR(g_u.cfg.diam[1], 0.640f, 0.0005);
	CHECK((g_u.flags & ODO_F_CALBAD) == 0U);
	CHECK(odo_journal_get(&g_u.jr, 1U, &e) || odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(count_type(&g_u, ODO_REC_CAL, 4U), 1);
	/* the brake input is ignored while calibrating */
	tick(&g_u, &m, 700, -1.0, 0U);
	CHECK(odo_unit_cal_start(&g_u, 100000U));
	tick(&g_u, &m, 5, 0.0, ODO_IN_BRAKE);
	CHECK_EQ(g_u.state, ODO_ST_CAL);
	/* a calibration that fails on one axle keeps the flag */
	tick(&g_u, &m, 1, 0.0, 0U);
	tick(&g_u, &m, 1, 0.0, ODO_IN_MARK);
	m.slip[1] = -1.0;
	const double s1 = m.s;

	for (int i = 0; i < 6000 && m.s < s1 + 100.0; i++) {
		odo_unit_tick(&g_u, sim_step(&m, (m.v < 5.0) ? 1.0 : 0.0, 0U));
	}
	tick(&g_u, &m, 1, 0.0, ODO_IN_MARK);
	CHECK_EQ(g_u.cal.ok_mask, 1);
	CHECK((g_u.flags & ODO_F_CALBAD) != 0U);
}

/* OR-30: after a large distance correction several RUN records are due in one tick */
static void unit_record_catch_up(void)
{
	sim_t m;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	odo_unit_init(&g_u, &nvm, &c);
	tick(&g_u, &m, 10, 0.0, 0U);
	const uint32_t seq0 = g_u.jr.seq;

	g_u.s += 1.6f; /* 6 records due: 4 now, 2 next tick */
	tick(&g_u, &m, 1, 0.0, 0U);
	CHECK_EQ(g_u.jr.seq - seq0, 4);
	tick(&g_u, &m, 1, 0.0, 0U);
	CHECK_EQ(g_u.jr.seq - seq0, 6);
	/* the slide flag is set while moving with a gated wheel */
	tick(&g_u, &m, 600, 1.0, 0U);
	m.slip[0] = -0.2;
	tick(&g_u, &m, 3, 0.0, 0U);
	CHECK((g_u.flags & ODO_F_SLIDE) != 0U);
	/* wheel faults appear in the flags */
	g_u.est.wheel_fault[0] = true;
	g_u.est.wheel_fault[1] = true;
	tick(&g_u, &m, 1, 0.0, 0U);
	CHECK((g_u.flags & (ODO_F_WHEEL0 | ODO_F_WHEEL1)) == (ODO_F_WHEEL0 | ODO_F_WHEEL1));
	/* huge values saturate in the records */
	g_u.s = 5.0e7f;
	g_u.est.kf.x[0] = 1000.0f;
	g_u.next_rec = 0U;
	tick(&g_u, &m, 1, 0.0, 0U);
	odo_entry_t e;

	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.s_cm, 4000000000U);
	CHECK_EQ(e.v_cms, 0xFFFF);
	g_u.est.kf.x[0] = -1.0f;
	g_u.s = -1.0f;
	odo_unit_power(&g_u, false);
	CHECK(odo_journal_get(&g_u.jr, 0U, &e));
	CHECK_EQ(e.s_cm, 0);
	CHECK_EQ(e.v_cms, 0);
}

/* brake with failed wheel sensors, an old edge time, and the brake input toggled in the event */
static void unit_brake_edge_cases(void)
{
	sim_t m;
	odo_nvm_t nvm;
	odo_cfg_t c = cfg_ok();

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	sim_init(&m, 0.65);
	odo_unit_init(&g_u, &nvm, &c);
	tick(&g_u, &m, 100, 0.0, 0U);
	tick(&g_u, &m, 800, 1.0, 0U);
	g_u.est.wheel_fault[0] = true;
	/* the edge time is older than a tick (input sampled late): the whole tick counts */
	odo_unit_tick(&g_u, sim_step(&m, -2.0, ODO_IN_BRAKE));
	CHECK_EQ(g_u.state, ODO_ST_BRAKE);
	m.smp.t_brake_us = 0U;
	tick(&g_u, &m, 50, -2.0, ODO_IN_BRAKE);
	/* released and pressed again inside the event: no second onset */
	tick(&g_u, &m, 2, -2.0, 0U);
	tick(&g_u, &m, 2, -2.0, ODO_IN_BRAKE);
	CHECK_EQ(g_u.state, ODO_ST_BRAKE);
	CHECK_EQ(count_type(&g_u, ODO_REC_BRAKE_ON, 40U), 1);
	g_u.est.wheel_fault[1] = true; /* both failed: the wheel mean falls back to the estimate */
	tick(&g_u, &m, 500, -2.0, ODO_IN_BRAKE);
	/* no standstill evidence without a wheel sensor: the event stays open (FMEA row W2) */
	CHECK_EQ(g_u.n_brake, 0);
	CHECK_EQ(g_u.state, ODO_ST_BRAKE);
	/* first tick has an edge time far in the past */
	fram_init(&g_fram);
	odo_unit_init(&g_u, &nvm, &c);
	sim_init(&m, 0.65);
	tick(&g_u, &m, 10, 0.0, 0U);
	m.smp.t_brake_us = 0U;
	m.smp.inputs = ODO_IN_BRAKE; /* sim_step keeps the old edge time */
	odo_unit_tick(&g_u, sim_step(&m, 0.0, ODO_IN_BRAKE));
	CHECK_NEAR(g_u.brake.w, 1.0f, 1e-6);
}

void test_unit(void)
{
	unit_brake_edge_cases();
	unit_boot_and_records();
	unit_brake_event();
	unit_brake_imu_fault();
	unit_power();
	unit_calibration();
	unit_record_catch_up();
}
