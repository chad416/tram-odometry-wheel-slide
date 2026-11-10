/*
 * Emergency-brake measurement (OR-20..OR-24).
 *
 * From the onset the distance is the integral of the inertial speed (the filter runs without
 * wheel updates). The onset is the captured brake-input edge: only the part of the first tick
 * after the edge is integrated. The standstill is the first tick of a run of ODO_STAND_TICKS
 * ticks in which the deceleration has ended (|f - c| < ODO_A_STOP), confirmed when every wheel
 * is at rest and the inertial speed is below ODO_V_STOP (or after ODO_STAND_MAX quiet ticks).
 * The speed residual r left at the standstill is the integral of an offset error; assuming
 * that error constant over the brake (time T), the corrected distance is
 *     d = D(T) - r T / 2
 * (two-point boundary correction: v(0) = v0 from the wheels before the brake, v(T) = 0).
 * The wheels are not used in the brake at all, not even when they do not slide: the braking
 * creep alone (0.2 % dry, 2 % wet at the wheel-brake demand) makes them worse than the IMU
 * (decision D3, DOSSIER §8.3 and §15).
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

void odo_brake_start(odo_brake_t *b, float v0, float frac)
{
	b->active = true;
	b->done = false;
	b->v0 = v0;
	b->v = v0;
	b->d = 0.0f;
	b->t = 0.0f;
	b->w = (frac < 0.0f) ? 0.0f : ((frac > 1.0f) ? 1.0f : frac);
	b->quiet = 0U;
	b->q_v = 0.0f;
	b->q_d = 0.0f;
	b->q_t = 0.0f;
}

bool odo_brake_step(odo_brake_t *b, const odo_brake_in_t *in)
{
	bool fin = false;

	if (b->active) {
		const float v_inertial = in->v;
		const float prev = b->v;
		const float dt = ODO_TS * b->w;
		const float inc = 0.5f * (prev + v_inertial) * dt;
		const bool still = (in->f_minus_c < ODO_A_STOP) && (in->f_minus_c > -ODO_A_STOP);

		b->v = v_inertial;
		b->d += inc;
		b->t += dt;
		b->w = 1.0f;
		if (still) {
			if (b->quiet == 0U) {
				b->q_v = prev;
				b->q_d = b->d - inc;
				b->q_t = b->t - dt;
			}
			b->quiet = (b->quiet < 0xFFFFU) ? (uint16_t)(b->quiet + 1U) : b->quiet;
		} else {
			b->quiet = 0U;
		}
		const bool slow = (v_inertial < ODO_V_STOP) && (v_inertial > -ODO_V_STOP);

		fin = (b->quiet >= ODO_STAND_TICKS) && in->rest && (slow || (b->quiet >= ODO_STAND_MAX));
		if (fin) {
			const float t = (b->q_t > ODO_TS) ? b->q_t : ODO_TS;

			b->result_t = t;
			b->result_r = b->q_v;
			b->result_d = b->q_d - (0.5f * b->q_v * t);
			b->active = false;
			b->done = true;
		}
	}
	return fin;
}
