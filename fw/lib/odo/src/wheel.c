/*
 * Wheel speed by the M/T method (OR-10, OR-11): the edge count difference between two
 * captured edges divided by the time between those edges. Without a new edge in a tick the
 * speed is bounded by one edge length over the time since the last edge, and becomes 0 after
 * ODO_STAND_US.
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

void odo_wheel_init(odo_wheel_t *w, float diameter_m, const odo_wheel_raw_t *in, uint32_t t_us)
{
	w->n_last = in->count;
	w->t_last_us = t_us;
	w->edge_m = (ODO_PI * diameter_m) / (float)ODO_EDGES_PER_REV;
	w->v = 0.0f;
	w->v_prev = 0.0f;
	w->seen = false;
}

void odo_wheel_update(odo_wheel_t *w, const odo_wheel_raw_t *in, uint32_t t_us)
{
	const uint32_t dn = in->count - w->n_last; /* modulo 2^32: counter wrap is harmless */

	w->v_prev = w->v;
	if (dn != 0U) {
		const uint32_t dt_us = in->t_edge_us - w->t_last_us;
		const float dt = (dt_us > 0U) ? ((float)dt_us * 1e-6f) : 1e-6f;

		/* the first edge after init has no reference edge: bound it by the tick */
		if (w->seen) {
			w->v = ((float)dn * w->edge_m) / dt;
		} else {
			w->v = ((float)dn * w->edge_m) / ODO_TS;
		}
		w->n_last = in->count;
		w->t_last_us = in->t_edge_us;
		w->seen = true;
	} else {
		const uint32_t since = t_us - w->t_last_us;

		if (since > ODO_STAND_US) {
			w->v = 0.0f;
		} else {
			const float bound = w->edge_m / (((float)since * 1e-6f) + 1e-6f);

			w->v = (bound < w->v) ? bound : w->v;
		}
	}
}

bool odo_wheel_at_rest(const odo_wheel_t *w)
{
	return w->v <= 0.0f;
}
