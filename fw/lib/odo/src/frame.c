/*
 * Serial frames (OR-60..OR-62):  A5 5A len type payload CRC16(len, type, payload), big endian.
 *
 *   0x10 sensor  (33 B): t_us, count0, t_edge0, count1, t_edge1 (u32), acc (i32, 1e-5 m/s^2),
 *                         gyro (i32, 1e-6 rad/s), t_brake_us (u32), inputs (u8)
 *   0x20 command  (5 B): cmd (u8), arg (u32)
 *   0x81 status  (28 B): state, flags, v cm/s (u16), s mm (u32), last brake distance mm (u32),
 *                         brake count (u16), last v0 cm/s (u16), journal seq (u16),
 *                         journal head (u16), diameter 0 and 1 in um (u32)
 * The sensor frame stands for the sensor front end (encoder capture, SPI IMU) that the
 * emulator does not model; on the target the same sample is built from the peripherals.
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

static void be_put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24U);
	p[1] = (uint8_t)(v >> 16U);
	p[2] = (uint8_t)(v >> 8U);
	p[3] = (uint8_t)v;
}

static void be_put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8U);
	p[1] = (uint8_t)v;
}

static uint32_t be_get32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) | ((uint32_t)p[2] << 8U) | (uint32_t)p[3];
}

static float to_float(uint32_t raw, float scale)
{
	int32_t v;

	if (raw > 0x7FFFFFFFU) {
		const uint32_t mag = ~raw;

		v = -(int32_t)mag - 1;
	} else {
		v = (int32_t)raw;
	}
	return (float)v * scale;
}

static uint32_t from_float(float x, float scale)
{
	const float q = x / scale;
	int32_t v;

	if (q >= 2.0e9f) {
		v = 2000000000;
	} else if (q <= -2.0e9f) {
		v = -2000000000;
	} else {
		v = (int32_t)((q < 0.0f) ? (q - 0.5f) : (q + 0.5f));
	}
	return (uint32_t)v;
}

void odo_rx_init(odo_rx_t *r)
{
	r->n = 0U;
	r->need = 0U;
	r->crc_err = 0U;
	r->ok = 0U;
}

static odo_rx_kind_t dispatch(odo_rx_t *r, odo_sample_t *s, odo_cmd_t *c)
{
	const uint8_t len = r->buf[2];
	const uint8_t type = r->buf[3];
	const uint8_t *p = &r->buf[4];
	odo_rx_kind_t kind = ODO_RX_NONE;

	if ((type == ODO_T_SENSOR) && (len == ODO_SENSOR_LEN)) {
		s->t_us = be_get32(&p[0]);
		s->wheel[0].count = be_get32(&p[4]);
		s->wheel[0].t_edge_us = be_get32(&p[8]);
		s->wheel[1].count = be_get32(&p[12]);
		s->wheel[1].t_edge_us = be_get32(&p[16]);
		s->acc = to_float(be_get32(&p[20]), 1e-5f);
		s->gyro = to_float(be_get32(&p[24]), 1e-6f);
		s->t_brake_us = be_get32(&p[28]);
		s->inputs = p[32];
		kind = ODO_RX_SENSOR;
	} else if ((type == ODO_T_CMD) && (len == ODO_CMD_LEN)) {
		c->cmd = p[0];
		c->arg = be_get32(&p[1]);
		kind = ODO_RX_CMD;
	} else {
		r->crc_err++; /* well-formed but unknown: counted with the errors */
	}
	return kind;
}

odo_rx_kind_t odo_rx_byte(odo_rx_t *r, uint8_t b, odo_sample_t *s, odo_cmd_t *c)
{
	odo_rx_kind_t kind = ODO_RX_NONE;

	if (r->n == 0U) {
		if (b == ODO_SYNC0) {
			r->buf[0] = b;
			r->n = 1U;
		}
	} else if (r->n == 1U) {
		if (b == ODO_SYNC1) {
			r->buf[1] = b;
			r->n = 2U;
		} else if (b != ODO_SYNC0) {
			r->n = 0U;
		} else {
			/* A5 A5: stay synchronised on the second A5 */
		}
	} else if (r->n == 2U) {
		if (b > ODO_SENSOR_LEN) {
			r->crc_err++;
			r->n = 0U;
		} else {
			r->buf[2] = b;
			r->need = (uint32_t)b + 6U;
			r->n = 3U;
		}
	} else {
		r->buf[r->n] = b;
		r->n++;
		if (r->n == r->need) {
			const uint16_t crc = (uint16_t)(((uint16_t)r->buf[r->n - 2U] << 8U) | (uint16_t)r->buf[r->n - 1U]);

			if (crc == odo_crc16(&r->buf[2], r->n - 4U)) {
				r->ok++;
				kind = dispatch(r, s, c);
			} else {
				r->crc_err++;
			}
			r->n = 0U;
		}
	}
	return kind;
}

static uint32_t finish(uint8_t *out, uint8_t type, uint32_t len)
{
	out[0] = ODO_SYNC0;
	out[1] = ODO_SYNC1;
	out[2] = (uint8_t)len;
	out[3] = type;
	be_put16(&out[4U + len], odo_crc16(&out[2], len + 2U));
	return len + 6U;
}

uint32_t odo_encode_sensor(const odo_sample_t *s, uint8_t *out, uint32_t cap)
{
	uint32_t n = 0U;

	if (cap >= (ODO_SENSOR_LEN + 6U)) {
		uint8_t *p = &out[4];

		be_put32(&p[0], s->t_us);
		be_put32(&p[4], s->wheel[0].count);
		be_put32(&p[8], s->wheel[0].t_edge_us);
		be_put32(&p[12], s->wheel[1].count);
		be_put32(&p[16], s->wheel[1].t_edge_us);
		be_put32(&p[20], from_float(s->acc, 1e-5f));
		be_put32(&p[24], from_float(s->gyro, 1e-6f));
		be_put32(&p[28], s->t_brake_us);
		p[32] = s->inputs;
		n = finish(out, ODO_T_SENSOR, ODO_SENSOR_LEN);
	}
	return n;
}

uint32_t odo_encode_status(const odo_unit_t *u, uint8_t *out, uint32_t cap)
{
	uint32_t n = 0U;

	if (cap >= ODO_MAX_FRAME) {
		uint8_t *p = &out[4];
		const float v = u->est.kf.x[0] * 100.0f;
		const float s = u->s * 1000.0f;
		const float d = u->last_d * 1000.0f;

		p[0] = (uint8_t)u->state;
		p[1] = u->flags;
		be_put16(&p[2], (v > 0.0f) ? (uint16_t)v : 0U);
		be_put32(&p[4], (s > 0.0f) ? (uint32_t)s : 0U);
		be_put32(&p[8], (d > 0.0f) ? (uint32_t)d : 0U);
		be_put16(&p[12], u->n_brake);
		be_put16(&p[14], (u->last_v0 > 0.0f) ? (uint16_t)(u->last_v0 * 100.0f) : 0U);
		be_put16(&p[16], u->jr.seq);
		be_put16(&p[18], (uint16_t)u->jr.head);
		be_put32(&p[20], (uint32_t)(u->cfg.diam[0] * 1e6f));
		be_put32(&p[24], (uint32_t)(u->cfg.diam[1] * 1e6f));
		n = finish(out, ODO_T_STATUS, ODO_STATUS_LEN);
	}
	return n;
}
