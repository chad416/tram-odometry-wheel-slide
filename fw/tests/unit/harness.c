/*
 * Test runner, FRAM model and the kinematic tram used by the module tests.
 * SPDX-License-Identifier: MIT
 */
#include "harness.h"

int g_checks;
int g_failures;

static uint32_t fram_write(void *ctx, uint32_t addr, const uint8_t *src, uint32_t len)
{
	fram_t *f = ctx;
	uint32_t n = len;

	if (f->tear_after >= 0) {
		n = (uint32_t)f->tear_after < len ? (uint32_t)f->tear_after : len;
		f->tear_after = -1;
	}
	memcpy(&f->mem[addr], src, n);
	f->writes++;
	return n;
}

static void fram_read(void *ctx, uint32_t addr, uint8_t *dst, uint32_t len)
{
	fram_t *f = ctx;

	memcpy(dst, &f->mem[addr], len);
}

void fram_init(fram_t *f)
{
	memset(f->mem, 0xFF, sizeof(f->mem)); /* erased-looking content: CRC fails everywhere */
	f->tear_after = -1;
	f->writes = 0;
}

odo_nvm_t fram_nvm(fram_t *f)
{
	odo_nvm_t n = {fram_write, fram_read, f};

	return n;
}

void sim_init(sim_t *m, double diam)
{
	memset(m, 0, sizeof(*m));
	for (unsigned k = 0; k < ODO_N_AXLES; k++) {
		m->diam[k] = diam;
	}
	m->grade_c = 0.0;
	m->smp.t_us = 0;
}

const odo_sample_t *sim_step(sim_t *m, double a, uint8_t inputs)
{
	const double dt = 0.01;
	double v1 = m->v + a * dt;

	if (v1 < 0.0) {
		v1 = 0.0;
	}
	m->a = (v1 - m->v) / dt;
	m->s += 0.5 * (m->v + v1) * dt;
	for (unsigned k = 0; k < ODO_N_AXLES; k++) {
		const double th0 = m->theta[k];
		const double th1 = th0 + 0.5 * (m->v + v1) * (1.0 + m->slip[k]) * dt / (m->diam[k] / 2.0);
		const double epr = (double)ODO_EDGES_PER_REV / (2.0 * M_PI);
		const uint32_t n1 = (uint32_t)floor(th1 * epr);

		if (n1 != m->smp.wheel[k].count) {
			const double ang = (double)n1 / epr;
			const double frac = (th1 > th0) ? (ang - th0) / (th1 - th0) : 1.0;

			m->smp.wheel[k].t_edge_us = (uint32_t)llround((m->t + frac * dt) * 1e6);
			m->smp.wheel[k].count = n1;
		}
		m->theta[k] = th1;
	}
	m->v = v1;
	m->t += dt;
	m->smp.t_us = (uint32_t)llround(m->t * 1e6);
	m->smp.acc = (float)(m->a + m->grade_c);
	m->smp.gyro = 0.0f;
	if (((inputs & ODO_IN_BRAKE) != 0U) && ((m->smp.inputs & ODO_IN_BRAKE) == 0U)) {
		m->smp.t_brake_us = m->smp.t_us - 4000U; /* edge 4 ms before the sample */
	}
	m->smp.inputs = inputs;
	return &m->smp;
}

int main(void)
{
	test_crc();
	test_wheel();
	test_kf();
	test_est();
	test_brake();
	test_journal();
	test_cal();
	test_unit();
	test_frame();
	test_scenario();
	printf("%d checks, %d failures\n", g_checks, g_failures);
	return g_failures ? 1 : 0;
}
