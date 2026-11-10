/*
 * Wheel-diameter calibration over a surveyed distance (OR-40..OR-42).
 *
 * Two track marks a surveyed length L apart are detected by the mark input. The edge counts
 * at the two rising edges give, per axle, D = L * ODO_EDGES_PER_REV / (pi * dn). A result
 * outside ODO_DIAM_MIN..ODO_DIAM_MAX is rejected for that axle (ok_mask bit clear).
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

void odo_cal_start(odo_cal_t *c, uint32_t length_mm)
{
	c->state = ODO_CAL_ARMED;
	c->length_mm = length_mm;
	c->prev_mark = true; /* a mark already active at the start is not a rising edge */
	c->ok_mask = 0U;
	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		c->n0[k] = 0U;
		c->diam[k] = 0.0f;
	}
}

bool odo_cal_step(odo_cal_t *c, const odo_sample_t *s)
{
	const bool mark = (s->inputs & ODO_IN_MARK) != 0U;
	const bool rising = mark && (!c->prev_mark);
	bool fin = false;

	c->prev_mark = mark;
	if (rising && (c->state == ODO_CAL_ARMED)) {
		for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
			c->n0[k] = s->wheel[k].count;
		}
		c->state = ODO_CAL_RUNNING;
	} else if (rising && (c->state == ODO_CAL_RUNNING)) {
		const float len = (float)c->length_mm * 1e-3f;

		for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
			const uint32_t dn = s->wheel[k].count - c->n0[k];
			const float d = (dn > 0U) ? ((len * (float)ODO_EDGES_PER_REV) / (ODO_PI * (float)dn)) : 0.0f;

			c->diam[k] = d;
			if ((d >= ODO_DIAM_MIN) && (d <= ODO_DIAM_MAX)) {
				c->ok_mask |= (uint8_t)(1U << k);
			}
		}
		c->state = ODO_CAL_IDLE;
		fin = true;
	} else {
		/* no mark edge, or not calibrating */
	}
	return fin;
}
