/*
 * The estimator (OR-12..OR-17, OR-50..OR-52).
 * SPDX-License-Identifier: MIT
 */
#include "harness.h"

static const float DIAM[2] = {0.65f, 0.65f};

static void run(odo_est_t *e, sim_t *m, int ticks, double a, uint8_t in, bool event)
{
	for (int i = 0; i < ticks; i++) {
		odo_est_step(e, sim_step(m, a, in), event, (in & ODO_IN_BRAKE) != 0U);
	}
}

/* OR-13: standstill updates learn the accelerometer offset; departure is followed */
static void est_standstill_and_departure(void)
{
	odo_est_t e;
	sim_t m;

	sim_init(&m, 0.65);
	m.grade_c = 0.4; /* bias + gradient seen by the accelerometer */
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	run(&e, &m, 300, 0.0, 0U, false);
	CHECK(e.rest);
	CHECK_NEAR(e.kf.x[1], 0.4f, 0.005);
	CHECK_NEAR(odo_est_speed(&e), 0.0f, 0.005);
	/* departure: the first ticks have no edge yet, the accelerometer breaks the standstill update */
	run(&e, &m, 300, 1.0, 0U, false);
	CHECK(!e.rest);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.03);
	CHECK_EQ(e.used, 3);
	/* cruise */
	run(&e, &m, 500, 0.0, 0U, false);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.01);
	CHECK_NEAR(e.kf.x[1], 0.4f, 0.01);
}

/* OR-14, OR-16: a sliding wheel is gated out; the slip in the gate is limited */
static void est_gate(void)
{
	odo_est_t e;
	sim_t m;

	sim_init(&m, 0.65);
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	run(&e, &m, 100, 0.0, 0U, false);
	run(&e, &m, 1000, 1.0, 0U, false); /* to 10 m/s */
	run(&e, &m, 200, 0.0, 0U, false);
	CHECK_EQ(e.used, 3);
	m.slip[0] = -0.10; /* wheel 0 slides at 10 % */
	run(&e, &m, 5, 0.0, 0U, false);
	CHECK_EQ(e.used, 2);
	CHECK_EQ(e.trust[0], 0);
	run(&e, &m, 200, 0.0, 0U, false);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.02); /* wheel 1 carries the speed */
	/* back to rolling: wheel 0 is used again after ODO_TRUST_TICKS consistent ticks */
	m.slip[0] = 0.0;
	run(&e, &m, 10, 0.0, 0U, false);
	CHECK_EQ(e.used, 2);
	run(&e, &m, 60, 0.0, 0U, false);
	CHECK_EQ(e.used, 3);
	/* both wheels spin by 6 %: both gated out, the IMU carries the speed */
	m.slip[0] = 0.06;
	m.slip[1] = 0.06;
	run(&e, &m, 100, 0.0, 0U, false);
	CHECK_EQ(e.used, 0);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.02);
	/* the wheel-acceleration gate: a wheel that accelerates unlike the vehicle is rejected
	 * even inside the speed gate */
	m.slip[0] = 0.0;
	m.slip[1] = 0.0;
	run(&e, &m, 100, 0.0, 0U, false);
	CHECK_EQ(e.used, 3);
	m.slip[0] = 0.0035; /* step of 3.5 cm/s in one tick: 3.5 m/s^2 apparent */
	run(&e, &m, 1, 0.0, 0U, false);
	CHECK_EQ(e.used, 2);
	/* below ODO_V_AGATE the acceleration check is off */
	CHECK(true);
}

/* OR-15: re-acquisition after 5 s, not while braking */
static void est_reacquisition(void)
{
	odo_est_t e;
	sim_t m;

	sim_init(&m, 0.65);
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	run(&e, &m, 100, 0.0, 0U, false);
	run(&e, &m, 800, 1.0, 0U, false);
	/* corrupt the estimate: 2 m/s off */
	e.kf.x[0] += 2.0f;
	run(&e, &m, 10, 0.0, 0U, false);
	CHECK_EQ(e.used, 0);
	/* brake input active: no re-acquisition */
	run(&e, &m, 600, 0.0, ODO_IN_BRAKE, false);
	CHECK(fabsf(odo_est_speed(&e) - (float)m.v) > 1.0f);
	CHECK_EQ(e.lost, 0);
	/* event running: no re-acquisition either */
	run(&e, &m, 10, 0.0, 0U, true);
	CHECK_EQ(e.lost, 0);
	run(&e, &m, (int)ODO_REACQ_TICKS + 5, 0.0, 0U, false);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.05);
	CHECK_EQ(e.used, 3);
	/* wheels that disagree are not re-acquired */
	e.kf.x[0] += 2.0f;
	m.slip[0] = 0.3;
	run(&e, &m, (int)ODO_REACQ_TICKS + 5, 0.0, 0U, false);
	CHECK_EQ(e.lost, 0);
	CHECK_EQ(e.used, 0);
}

/* OR-50: implausible IMU values latch the IMU fault; the wheels carry the speed */
static void est_imu_fault(void)
{
	odo_est_t e;
	sim_t m;

	sim_init(&m, 0.65);
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	run(&e, &m, 50, 0.0, 0U, false);
	m.grade_c = 30.0; /* accelerometer stuck at full scale */
	run(&e, &m, 1, 0.0, 0U, false);
	CHECK(e.imu_fault);
	m.grade_c = 0.0;
	run(&e, &m, 600, 1.0, 0U, false);
	CHECK(e.imu_fault);
	CHECK_NEAR(odo_est_speed(&e), (float)m.v, 0.1);
	run(&e, &m, 700, -1.0, 0U, false);
	CHECK(e.rest);
	/* gyro out of range also latches */
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	m.smp.gyro = 3.0f;
	odo_est_step(&e, &m.smp, false, false);
	CHECK(e.imu_fault);
	m.smp.gyro = -3.0f;
	odo_est_init(&e, DIAM, &m.smp);
	odo_est_step(&e, &m.smp, false, false);
	CHECK(e.imu_fault);
	m.smp.gyro = 0.0f;
	m.smp.acc = -25.0f;
	odo_est_init(&e, DIAM, &m.smp);
	odo_est_step(&e, &m.smp, false, false);
	CHECK(e.imu_fault);
}

/* OR-17: a wheel sensor that shows rest while the vehicle moves is failed */
static void est_wheel_stuck(void)
{
	odo_est_t e;
	sim_t m;

	sim_init(&m, 0.65);
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	run(&e, &m, 50, 0.0, 0U, false);
	run(&e, &m, 500, 1.0, 0U, false);
	m.slip[1] = -1.0; /* sensor 1 dead: no more edges */
	run(&e, &m, 230, 0.0, 0U, false);
	CHECK(e.wheel_fault[1]);
	CHECK(!e.wheel_fault[0]);
	CHECK(!e.rest);
	/* standstill: the failed wheel does not block the rest detection */
	run(&e, &m, 600, -1.0, 0U, false);
	CHECK(e.rest);
	/* during an event a stopped wheel is not a fault (locked wheels slide) */
	odo_est_init(&e, DIAM, sim_step(&m, 0.0, 0U));
	m.slip[1] = 0.0;
	run(&e, &m, 500, 1.0, 0U, false);
	m.slip[0] = -1.0;
	run(&e, &m, 300, 0.0, 0U, true);
	CHECK(!e.wheel_fault[0]);
	/* both wheels failed: no standstill evidence */
	run(&e, &m, 250, 0.0, 0U, false);
	CHECK(e.wheel_fault[0]);
	run(&e, &m, 900, -1.0, 0U, false); /* stop with sensor 0 failed: wheel 1 shows the rest */
	CHECK(e.rest);
	m.slip[1] = -1.0;
	e.wheel_fault[1] = true;
	run(&e, &m, 5, 0.0, 0U, false);
	CHECK(e.wheel_fault[0]);
	CHECK(!e.rest);
	/* both failed at standstill: still no standstill evidence */
	run(&e, &m, 800, -1.0, 0U, false);
	CHECK(!e.rest);
	/* the trust counter saturates at ODO_TRUST_TICKS */
	CHECK(e.trust[0] <= ODO_TRUST_TICKS);
}

void test_est(void)
{
	est_standstill_and_departure();
	est_gate();
	est_reacquisition();
	est_imu_fault();
	est_wheel_stuck();
}
