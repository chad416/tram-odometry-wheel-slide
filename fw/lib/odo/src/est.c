/*
 * The estimator (OR-12..OR-17, OR-50..OR-52): wheel speeds, IMU prediction, slide/spin gate,
 * re-acquisition, standstill updates and the sensor plausibility checks.
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

static float fabs_f(float x)
{
	return (x < 0.0f) ? -x : x;
}

void odo_est_init(odo_est_t *e, const float diam[ODO_N_AXLES], const odo_sample_t *s0)
{
	odo_kf_init(&e->kf, s0->acc);
	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		odo_wheel_init(&e->wheel[k], diam[k], &s0->wheel[k], s0->t_us);
		e->trust[k] = 0U;
		e->stuck[k] = 0U;
		e->wheel_fault[k] = false;
	}
	e->lost = 0U;
	e->imu_fault = false;
	e->used = 0U;
	e->rest = true;
}

float odo_est_speed(const odo_est_t *e)
{
	return e->kf.x[0];
}

/* OR-17: a wheel that shows rest for 2 s while the other wheel and the estimate move is failed. */
static void stuck_check(odo_est_t *e, bool event)
{
	const float v = e->kf.x[0];

	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		const odo_wheel_t *other = &e->wheel[(k + 1U) % ODO_N_AXLES];
		const bool moving = (other->v > ODO_WSTUCK_V) && (v > ODO_WSTUCK_V);

		if ((!event) && odo_wheel_at_rest(&e->wheel[k]) && moving) {
			e->stuck[k]++;
		} else {
			e->stuck[k] = 0U;
		}
		if (e->stuck[k] >= ODO_WSTUCK_TICKS) {
			e->wheel_fault[k] = true;
		}
	}
}

void odo_est_step(odo_est_t *e, const odo_sample_t *s, bool event, bool brake_in)
{
	float sum = 0.0f;
	uint32_t n = 0U;
	bool rest = true;

	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		odo_wheel_update(&e->wheel[k], &s->wheel[k], s->t_us);
	}

	/* OR-50: IMU plausibility (latched) */
	if ((fabs_f(s->acc) > ODO_ACC_MAX) || (fabs_f(s->gyro) > ODO_GYRO_MAX)) {
		e->imu_fault = true;
	}
	if (e->imu_fault) {
		odo_kf_predict(&e->kf, e->kf.x[1], e->kf.x[2]); /* constant speed */
	} else {
		odo_kf_predict(&e->kf, s->acc, s->gyro);
	}

	const float v = e->kf.x[0];
	const float a_in = s->acc - e->kf.x[1];
	const float gate = ODO_GATE_ABS + (ODO_GATE_REL * fabs_f(v));

	e->used = 0U;
	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		const odo_wheel_t *w = &e->wheel[k];
		const float aw = (w->v - w->v_prev) / ODO_TS;
		/* no IMU: nothing to check the wheels against, they carry the speed (OR-53) */
		const bool ok_v = e->imu_fault || (fabs_f(w->v - v) < gate);
		const bool ok_a = e->imu_fault || (w->v < ODO_V_AGATE) || (fabs_f(aw - a_in) < ODO_A_GATE);

		/* a wheel at rest gives no speed measurement (the standstill update does) */
		if (ok_v && ok_a && (!e->wheel_fault[k]) && (!odo_wheel_at_rest(w))) {
			e->trust[k] = (e->trust[k] < ODO_TRUST_TICKS) ? (uint16_t)(e->trust[k] + 1U) : e->trust[k];
		} else {
			e->trust[k] = 0U;
		}
		if ((e->trust[k] >= ODO_TRUST_TICKS) && (!event)) {
			sum += w->v;
			n++;
			e->used |= (uint8_t)(1U << k);
		}
		if (!e->wheel_fault[k]) {
			rest = rest && odo_wheel_at_rest(w);
		}
	}
	stuck_check(e, event);

	/* OR-15: re-acquisition after 5 s without an accepted wheel, both wheels agreeing */
	const float w0 = e->wheel[0].v;
	const float w1 = e->wheel[1].v;
	const float wmax = (w0 > w1) ? w0 : w1;
	const bool agree = fabs_f(w0 - w1) < (ODO_GATE_ABS + (ODO_GATE_REL * wmax));
	const bool healthy = (!e->wheel_fault[0]) && (!e->wheel_fault[1]);

	if ((n == 0U) && agree && healthy && (!event) && (!brake_in)) {
		e->lost++;
	} else {
		e->lost = 0U;
	}
	if (e->lost >= ODO_REACQ_TICKS) {
		const float vm = 0.5f * (w0 + w1);

		odo_kf_set_speed(&e->kf, vm, ODO_SIG_W_ABS + (ODO_SIG_W_REL * vm));
		e->kf.p[1][1] += 1e-2f; /* the offset is suspect too */
		e->trust[0] = ODO_TRUST_TICKS;
		e->trust[1] = ODO_TRUST_TICKS;
		e->lost = 0U;
		sum = w0 + w1;
		n = 2U;
		e->used = 3U;
	}

	/* both wheels failed: no standstill evidence */
	e->rest = rest && (!(e->wheel_fault[0] && e->wheel_fault[1]));

	/* OR-13, OR-14: wheel update; standstill updates outside an event */
	const bool quiet = e->imu_fault || (fabs_f(a_in) < ODO_A_ZUPT);
	const bool zupt = e->rest && (!event) && quiet;

	if (zupt) {
		odo_kf_update(&e->kf, 0U, 0.0f, ODO_SIG_ZUPT);
		if (!e->imu_fault) {
			odo_kf_update(&e->kf, 1U, s->acc, ODO_SIG_ZACC);
			odo_kf_update(&e->kf, 2U, s->gyro, ODO_SIG_ZARU);
		}
	} else if (n > 0U) {
		const float z = sum / (float)n;

		odo_kf_update(&e->kf, 0U, z, ODO_SIG_W_ABS + (ODO_SIG_W_REL * fabs_f(z)));
	} else {
		/* no measurement this tick: inertial prediction only */
	}
}
