/*
 * The recorder (OR-01..OR-05, OR-20, OR-25, OR-30, OR-35, OR-40, OR-53): operating states,
 * records every 0.25 m, the emergency-brake measurement, calibration and power fail.
 *
 *   INIT --first sample--> STAND <--> RUN          (STAND: every healthy wheel at rest)
 *   STAND/RUN --brake input rising edge--> BRAKE --standstill confirmed--> STAND
 *   STAND --calibration command--> CAL --second mark--> STAND/RUN
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

#include "odo/odo.h"

#define DIAM_DEFAULT 0.655f /* m, middle of the wear range: used until calibrated */

static uint16_t cfg_crc(const odo_cfg_t *c)
{
	uint8_t b[4U * ODO_N_AXLES];

	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		const uint32_t um = (uint32_t)(c->diam[k] * 1e6f);

		b[4U * k] = (uint8_t)(um >> 24U);
		b[(4U * k) + 1U] = (uint8_t)(um >> 16U);
		b[(4U * k) + 2U] = (uint8_t)(um >> 8U);
		b[(4U * k) + 3U] = (uint8_t)um;
	}
	return odo_crc16(b, 4U * ODO_N_AXLES);
}

static bool cfg_valid(const odo_cfg_t *c)
{
	bool ok = c->crc == cfg_crc(c);

	for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
		ok = ok && (c->diam[k] >= ODO_DIAM_MIN) && (c->diam[k] <= ODO_DIAM_MAX);
	}
	return ok;
}

void odo_cfg_seal(odo_cfg_t *c)
{
	c->crc = cfg_crc(c);
}

static uint16_t sat16(float x)
{
	uint16_t r;

	if (x <= 0.0f) {
		r = 0U;
	} else if (x >= 65535.0f) {
		r = 0xFFFFU;
	} else {
		r = (uint16_t)x;
	}
	return r;
}

static uint32_t sat32(float x)
{
	uint32_t r;

	if (x <= 0.0f) {
		r = 0U;
	} else if (x >= 4.0e9f) {
		r = 4000000000U;
	} else {
		r = (uint32_t)x;
	}
	return r;
}

static void record(odo_unit_t *u, uint8_t type, float s, float v, uint16_t aux)
{
	odo_entry_t e;

	e.type = type;
	e.flags = u->flags;
	e.s_cm = sat32(s * 100.0f);
	e.v_cms = sat16(v * 100.0f);
	e.t_ds = (uint16_t)((u->t_us / 100000U) & 0xFFFFU);
	e.aux = aux;
	(void)odo_journal_append(&u->jr, &e);
}

void odo_unit_init(odo_unit_t *u, const odo_nvm_t *nvm, const odo_cfg_t *cfg)
{
	(void)memset(u, 0, sizeof(*u));
	if (cfg_valid(cfg)) {
		u->cfg = *cfg;
	} else {
		/* OR-42: no valid calibration: default diameter, flagged in every record */
		for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
			u->cfg.diam[k] = DIAM_DEFAULT;
		}
		odo_cfg_seal(&u->cfg);
		u->flags = ODO_F_CALBAD;
	}
	odo_journal_recover(&u->jr, nvm);
	u->next_rec = ODO_REC_STEP_MM;
	u->state = ODO_ST_INIT;
	u->cal.state = ODO_CAL_IDLE;
}

bool odo_unit_cal_start(odo_unit_t *u, uint32_t length_mm)
{
	/* OR-40: only at standstill, surveyed length 10 m .. 2 km */
	const bool ok = (u->state == ODO_ST_STAND) && (length_mm >= 10000U) && (length_mm <= 2000000U);

	if (ok) {
		odo_cal_start(&u->cal, length_mm);
		u->state = ODO_ST_CAL;
	}
	return ok;
}

void odo_unit_power(odo_unit_t *u, bool supply_ok)
{
	/* OR-35: the record of the moment the supply went away is written at once (FRAM) */
	if ((!supply_ok) && (!u->pfail)) {
		u->pfail = true;
		u->flags |= ODO_F_PFAIL;
		record(u, ODO_REC_PFAIL, u->s, odo_est_speed(&u->est), 0U);
	} else if (supply_ok && u->pfail) {
		u->pfail = false;
		u->flags &= (uint8_t)~ODO_F_PFAIL;
		record(u, ODO_REC_PRESTORE, u->s, odo_est_speed(&u->est), 0U);
	} else {
		/* no change */
	}
}

static void update_flags(odo_unit_t *u)
{
	uint8_t f = u->flags & (uint8_t)(ODO_F_PFAIL | ODO_F_CALBAD);
	const odo_est_t *e = &u->est;

	if (e->wheel_fault[0]) {
		f |= ODO_F_WHEEL0;
	}
	if (e->wheel_fault[1]) {
		f |= ODO_F_WHEEL1;
	}
	if (e->imu_fault) {
		f |= ODO_F_IMU;
	}
	if ((!e->rest) && (e->used != 3U)) {
		f |= ODO_F_SLIDE;
	}
	u->flags = f;
}

static void calibration(odo_unit_t *u, const odo_sample_t *s)
{
	if (odo_cal_step(&u->cal, s)) {
		for (uint32_t k = 0U; k < ODO_N_AXLES; k++) {
			if ((u->cal.ok_mask & (uint8_t)(1U << k)) != 0U) {
				u->cfg.diam[k] = u->cal.diam[k];
				u->est.wheel[k].edge_m = (ODO_PI * u->cal.diam[k]) / (float)ODO_EDGES_PER_REV;
			}
		}
		odo_cfg_seal(&u->cfg);
		if (u->cal.ok_mask == 3U) {
			u->flags &= (uint8_t)~ODO_F_CALBAD;
		} else {
			u->flags |= ODO_F_CALBAD;
		}
		record(u, ODO_REC_CAL, u->s, odo_est_speed(&u->est), (uint16_t)u->cal.ok_mask);
		u->state = ODO_ST_RUN;
	}
}

static void brake(odo_unit_t *u, const odo_sample_t *s, bool rise)
{
	odo_est_t *e = &u->est;
	const float v = odo_est_speed(e);

	if (rise) {
		const uint32_t after_us = s->t_us - s->t_brake_us;
		const float frac = (after_us < ODO_TICK_US) ? ((float)after_us / (float)ODO_TICK_US) : 1.0f;

		odo_brake_start(&u->brake, v, frac);
		u->brake.s0 = u->s;
		u->state = ODO_ST_BRAKE;
		record(u, ODO_REC_BRAKE_ON, u->s, v, 0U);
	}
	odo_brake_in_t in;

	in.v = v;
	in.f_minus_c = s->acc - e->kf.x[1];
	in.rest = e->rest;
	/* IMU failed: the wheels carry the brake (mean wheel speed); "quiet" = wheels at rest */
	if (e->imu_fault) {
		in.v = 0.5f * (e->wheel[0].v + e->wheel[1].v);
		in.f_minus_c = e->rest ? 0.0f : 1.0f;
	}
	if (odo_brake_step(&u->brake, &in)) {
		const odo_brake_t *b = &u->brake;

		u->s = b->s0 + b->result_d;
		if (!e->imu_fault) {
			e->kf.x[1] += b->result_r / b->result_t; /* the offset that left the residual */
			u->flags |= ODO_F_CORR;
		}
		odo_kf_set_speed(&e->kf, 0.0f, ODO_SIG_ZUPT);
		u->n_brake++;
		u->last_d = b->result_d;
		u->last_v0 = b->v0;
		record(u, ODO_REC_BRAKE_RES, u->s, b->v0, sat16(b->result_d * 100.0f));
		u->flags &= (uint8_t)~ODO_F_CORR;
		u->state = ODO_ST_STAND;
	} else if (!e->imu_fault) {
		u->s = u->brake.s0 + u->brake.d;
	} else {
		u->s += odo_est_speed(e) * ODO_TS;
	}
}

void odo_unit_tick(odo_unit_t *u, const odo_sample_t *s)
{
	const bool brake_in = (s->inputs & ODO_IN_BRAKE) != 0U;

	u->t_us = s->t_us;
	if (!u->started) {
		odo_est_init(&u->est, u->cfg.diam, s);
		u->started = true;
		u->state = ODO_ST_STAND;
		u->prev_brake = brake_in;
		record(u, ODO_REC_BOOT, 0.0f, 0.0f, 0U);
	} else {
		/* OR-20: a rising edge of the brake input starts an event (none while calibrating) */
		const bool rise = brake_in && (!u->prev_brake) && (u->state != ODO_ST_BRAKE) && (u->state != ODO_ST_CAL);
		u->prev_brake = brake_in;
		const bool event = (rise || (u->state == ODO_ST_BRAKE)) && (!u->est.imu_fault);

		odo_est_step(&u->est, s, event, brake_in);
		if (rise || (u->state == ODO_ST_BRAKE)) {
			brake(u, s, rise);
		} else {
			u->s += odo_est_speed(&u->est) * ODO_TS;
			if (u->state == ODO_ST_CAL) {
				calibration(u, s);
			} else {
				u->state = u->est.rest ? ODO_ST_STAND : ODO_ST_RUN;
			}
		}
		update_flags(u);
		/* OR-30: one RUN record per 0.25 m (at most 4 per tick after a correction) */
		for (uint32_t i = 0U; i < 4U; i++) {
			if ((u->s * 1000.0f) >= (float)u->next_rec) {
				record(u, ODO_REC_RUN, u->s, odo_est_speed(&u->est), (uint16_t)u->state);
				u->next_rec += ODO_REC_STEP_MM;
			}
		}
	}
	u->ticks++;
}
