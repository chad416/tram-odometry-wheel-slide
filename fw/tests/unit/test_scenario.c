/*
 * Recorded scenarios (OR-80): sensor data from the vehicle and sensor models
 * (tools/export_scenarios.py) replayed through the recorder. The stopping distance must be
 * within the error budget of the truth and agree with the Python reference estimator.
 * SPDX-License-Identifier: MIT
 */
#include <stdlib.h>

#include "harness.h"

#ifndef ODO_SCENARIO_DIR
#define ODO_SCENARIO_DIR "fw/sim/scenarios"
#endif

static fram_t g_fram;
static odo_unit_t g_u;

typedef struct {
	const char *name;
	double tol_truth; /* m: allowed |error| against the truth (DOSSIER §7.1 budget) */
} scenario_t;

static const scenario_t SCEN[] = {
	{"sc1_depot_dry", 0.05},
	{"sc2_wet_down", 0.30},
	{"sc3_leaf_slide", 0.30},
	{"sc4_vcurve", 0.30},
};

static int header(const char *line, const char *key, double *out)
{
	const size_t n = strlen(key);

	if (strncmp(line, "# ", 2) == 0 && strncmp(line + 2, key, n) == 0 && line[2 + n] == ' ') {
		*out = atof(line + 3 + n);
		return 1;
	}
	return 0;
}

static void replay(const scenario_t *sc)
{
	char path[512];
	char line[256];
	double d_true = -1.0;
	double e3 = -1.0;
	double e1 = -1.0;
	odo_cfg_t cfg = {{0.0f, 0.0f}, 0U};
	unsigned rows = 0;

	snprintf(path, sizeof(path), "%s/%s.csv", ODO_SCENARIO_DIR, sc->name);
	FILE *f = fopen(path, "r");

	CHECK(f != NULL);
	if (f == NULL) {
		printf("cannot open %s\n", path);
		return;
	}
	while (fgets(line, sizeof(line), f) != NULL && line[0] == '#') {
		double v;

		(void)header(line, "d_true_m", &d_true);
		(void)header(line, "e3_d_m", &e3);
		(void)header(line, "e1_d_m", &e1);
		if (strncmp(line, "# d_cfg_m ", 10) == 0) {
			double a, b;

			if (sscanf(line + 10, "%lf %lf", &a, &b) == 2) {
				cfg.diam[0] = (float)a;
				cfg.diam[1] = (float)b;
			}
		}
		(void)v;
	}
	odo_cfg_seal(&cfg);
	fram_init(&g_fram);
	odo_nvm_t nvm = fram_nvm(&g_fram);

	odo_unit_init(&g_u, &nvm, &cfg);
	CHECK_EQ(g_u.flags & ODO_F_CALBAD, 0);
	/* the header row has been read by the loop above; data rows follow */
	while (fgets(line, sizeof(line), f) != NULL) {
		unsigned long t, c0, e0, c1, e1v, tb, in, smm, vmm;
		long acc, gyr;
		odo_sample_t s;

		if (sscanf(line, "%lu,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%lu,%lu,%lu", &t, &c0, &e0, &c1, &e1v, &acc, &gyr, &tb,
			   &in, &smm, &vmm) != 11) {
			continue;
		}
		s.t_us = (uint32_t)t;
		s.wheel[0].count = (uint32_t)c0;
		s.wheel[0].t_edge_us = (uint32_t)e0;
		s.wheel[1].count = (uint32_t)c1;
		s.wheel[1].t_edge_us = (uint32_t)e1v;
		s.acc = (float)acc * 1e-5f;
		s.gyro = (float)gyr * 1e-6f;
		s.t_brake_us = (uint32_t)tb;
		s.inputs = (uint8_t)in;
		odo_unit_tick(&g_u, &s);
		rows++;
	}
	fclose(f);
	CHECK(rows > 1000U);
	CHECK_EQ(g_u.n_brake, 1);
	const double err = g_u.last_d - d_true;

	printf("  %-15s rows %u  d_true %.3f m  firmware %.3f m (err %+.3f, budget %.2f)  python E3 %.3f m  one wheel err %+.3f\n",
	       sc->name, rows, d_true, (double)g_u.last_d, err, sc->tol_truth, e3, e1 - d_true);
	CHECK(fabs(err) <= sc->tol_truth);
	CHECK_NEAR(g_u.last_d, e3, 0.03); /* float32 firmware vs float64 reference */
	CHECK(g_u.jr.written > 1000U);
}

void test_scenario(void)
{
	for (unsigned i = 0; i < sizeof(SCEN) / sizeof(SCEN[0]); i++) {
		replay(&SCEN[i]);
	}
}
