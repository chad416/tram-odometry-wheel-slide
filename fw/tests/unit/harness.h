/*
 * Minimal test harness, an FRAM model and sample builders for the odometry tests.
 * SPDX-License-Identifier: MIT
 */
#ifndef ODO_TEST_HARNESS_H
#define ODO_TEST_HARNESS_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "odo/odo.h"

extern int g_checks;
extern int g_failures;

#define CHECK(cond)                                                                    \
	do {                                                                           \
		g_checks++;                                                            \
		if (!(cond)) {                                                         \
			g_failures++;                                                  \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
		}                                                                      \
	} while (0)

#define CHECK_EQ(a, b)                                                                 \
	do {                                                                           \
		long long _a = (long long)(a);                                         \
		long long _b = (long long)(b);                                         \
		g_checks++;                                                            \
		if (_a != _b) {                                                        \
			g_failures++;                                                  \
			printf("FAIL %s:%d: %s == %s (%lld != %lld)\n", __FILE__,      \
			       __LINE__, #a, #b, _a, _b);                              \
		}                                                                      \
	} while (0)

#define CHECK_NEAR(a, b, tol)                                                          \
	do {                                                                           \
		double _a = (double)(a);                                               \
		double _b = (double)(b);                                               \
		g_checks++;                                                            \
		if (fabs(_a - _b) > (double)(tol)) {                                   \
			g_failures++;                                                  \
			printf("FAIL %s:%d: %s ~ %s (%g vs %g, tol %g)\n", __FILE__,    \
			       __LINE__, #a, #b, _a, _b, (double)(tol));               \
		}                                                                      \
	} while (0)

/* ---- FRAM model: 128 KiB, with an optional torn write ------------------------------ */
typedef struct {
	uint8_t mem[ODO_SLOTS * ODO_SLOT_BYTES];
	int tear_after; /* -1: off; n >= 0: the next write stores only n bytes */
	unsigned writes;
} fram_t;

void fram_init(fram_t *f);
odo_nvm_t fram_nvm(fram_t *f);

/* ---- A simple kinematic tram for the module tests --------------------------------- */
typedef struct {
	double t;      /* s */
	double s;      /* m */
	double v;      /* m/s */
	double a;      /* m/s^2 */
	double diam[ODO_N_AXLES];
	double theta[ODO_N_AXLES]; /* wheel angle, rad */
	double slip[ODO_N_AXLES];  /* peripheral speed = v (1 + slip) */
	odo_sample_t smp;
	double grade_c;            /* accelerometer offset seen (bias + gravity) */
} sim_t;

void sim_init(sim_t *m, double diam);
/* advance one tick with acceleration a (m/s^2), return the sample */
const odo_sample_t *sim_step(sim_t *m, double a, uint8_t inputs);

/* ---- suites */
void test_crc(void);
void test_wheel(void);
void test_kf(void);
void test_est(void);
void test_brake(void);
void test_journal(void);
void test_cal(void);
void test_unit(void);
void test_frame(void);
void test_scenario(void);

#endif
