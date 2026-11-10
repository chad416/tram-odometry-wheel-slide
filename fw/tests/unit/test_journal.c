/*
 * Record ring (OR-30..OR-34) and serial frames (OR-60..OR-62).
 * SPDX-License-Identifier: MIT
 */
#include "harness.h"

static fram_t g_fram;

static odo_entry_t entry(uint8_t type, uint32_t s_cm)
{
	odo_entry_t e;

	memset(&e, 0, sizeof(e));
	e.type = type;
	e.flags = 0x10U;
	e.s_cm = s_cm;
	e.v_cms = 1234U;
	e.t_ds = 77U;
	e.aux = 0xBEEFU;
	return e;
}

static void journal_empty_and_append(void)
{
	odo_journal_t j;
	odo_nvm_t nvm;
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	odo_journal_recover(&j, &nvm);
	CHECK_EQ(j.valid, 0);
	CHECK_EQ(j.head, 0);
	CHECK_EQ(j.seq, 0);
	CHECK(!odo_journal_get(&j, 0U, &e));
	for (uint32_t i = 0U; i < 10U; i++) {
		e = entry(ODO_REC_RUN, 70000U + i * 25U);
		CHECK(odo_journal_append(&j, &e));
	}
	CHECK(odo_journal_get(&j, 0U, &e));
	CHECK_EQ(e.s_cm, 70000U + 9U * 25U);
	CHECK_EQ(e.seq, 9);
	CHECK_EQ(e.aux, 0xBEEF);
	CHECK(odo_journal_get(&j, 9U, &e));
	CHECK_EQ(e.seq, 0);
	/* recovery after a "reboot" finds the same head and sequence */
	odo_journal_t j2;

	odo_journal_recover(&j2, &nvm);
	CHECK_EQ(j2.valid, 10);
	CHECK_EQ(j2.head, 10);
	CHECK_EQ(j2.seq, 10);
}

/* OR-33: a write cut by the supply loss is discarded; the record before it survives */
static void journal_torn_write(void)
{
	odo_journal_t j;
	odo_nvm_t nvm;
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	odo_journal_recover(&j, &nvm);
	for (uint32_t i = 0U; i < 5U; i++) {
		e = entry(ODO_REC_RUN, i);
		(void)odo_journal_append(&j, &e);
	}
	for (int cut = 0; cut < (int)ODO_SLOT_BYTES; cut++) {
		odo_journal_t r;

		g_fram.tear_after = cut;
		e = entry(ODO_REC_BRAKE_RES, 999U);
		CHECK(!odo_journal_append(&j, &e));
		odo_journal_recover(&r, &nvm);
		CHECK(odo_journal_get(&r, 0U, &e));
		CHECK_EQ(e.s_cm, 4U); /* the last complete record */
		CHECK_EQ(r.head, 5);
	}
	CHECK_EQ(j.torn, 16);
	CHECK_EQ(j.head, 5);
}

/* OR-32: the ring wraps; recovery finds the newest by serial-number arithmetic */
static void journal_wrap(void)
{
	odo_journal_t j;
	odo_nvm_t nvm;
	odo_entry_t e;

	fram_init(&g_fram);
	nvm = fram_nvm(&g_fram);
	odo_journal_recover(&j, &nvm);
	j.seq = 65530U; /* start near the 16-bit wrap */
	for (uint32_t i = 0U; i < ODO_SLOTS + 100U; i++) {
		e = entry(ODO_REC_RUN, i);
		(void)odo_journal_append(&j, &e);
	}
	CHECK_EQ(j.head, 100);
	odo_journal_t r;

	odo_journal_recover(&r, &nvm);
	CHECK_EQ(r.valid, ODO_SLOTS);
	CHECK_EQ(r.head, 100);
	CHECK_EQ(r.seq, j.seq);
	CHECK(odo_journal_get(&r, 0U, &e));
	CHECK_EQ(e.s_cm, ODO_SLOTS + 99U);
	CHECK(odo_journal_get(&r, ODO_SLOTS - 1U, &e));
	CHECK_EQ(e.s_cm, 100U);
	/* a corrupted slot in the middle is skipped, the newest is still found */
	g_fram.mem[50 * ODO_SLOT_BYTES + 5] ^= 0x01U;
	odo_journal_recover(&r, &nvm);
	CHECK_EQ(r.valid, ODO_SLOTS - 1U);
	CHECK_EQ(r.head, 100);
	/* a slot with a valid CRC but type NONE is not a record */
	uint8_t raw[ODO_SLOT_BYTES] = {0};
	const uint16_t crc = odo_crc16(raw, 14U);

	raw[14] = (uint8_t)crc;
	raw[15] = (uint8_t)(crc >> 8);
	CHECK(!odo_entry_unpack(raw, &e));
}

void test_journal(void)
{
	journal_empty_and_append();
	journal_torn_write();
	journal_wrap();
}

/* ---- frames ----------------------------------------------------------------------------- */

static odo_rx_kind_t feed(odo_rx_t *r, const uint8_t *b, uint32_t n, odo_sample_t *s, odo_cmd_t *c)
{
	odo_rx_kind_t k = ODO_RX_NONE;

	for (uint32_t i = 0U; i < n; i++) {
		const odo_rx_kind_t x = odo_rx_byte(r, b[i], s, c);

		if (x != ODO_RX_NONE) {
			k = x;
		}
	}
	return k;
}

static void frame_sensor_roundtrip(void)
{
	odo_sample_t a;
	odo_sample_t b;
	odo_cmd_t c;
	odo_rx_t r;
	uint8_t buf[64];

	memset(&a, 0, sizeof(a));
	a.t_us = 123456789U;
	a.wheel[0].count = 0xA0B0C0D0U;
	a.wheel[0].t_edge_us = 123450000U;
	a.wheel[1].count = 77U;
	a.wheel[1].t_edge_us = 99U;
	a.acc = -2.71828f;
	a.gyro = 0.0123f;
	a.t_brake_us = 5U;
	a.inputs = ODO_IN_BRAKE;
	const uint32_t n = odo_encode_sensor(&a, buf, sizeof(buf));

	CHECK_EQ(n, ODO_SENSOR_LEN + 6U);
	CHECK_EQ(odo_encode_sensor(&a, buf, 10U), 0);
	odo_rx_init(&r);
	CHECK_EQ(feed(&r, buf, n, &b, &c), ODO_RX_SENSOR);
	CHECK_EQ(b.t_us, a.t_us);
	CHECK_EQ(b.wheel[0].count, a.wheel[0].count);
	CHECK_EQ(b.wheel[1].t_edge_us, 99);
	CHECK_NEAR(b.acc, a.acc, 1e-5);
	CHECK_NEAR(b.gyro, a.gyro, 1e-6);
	CHECK_EQ(b.t_brake_us, 5);
	CHECK_EQ(b.inputs, ODO_IN_BRAKE);
	/* positive acceleration, saturation of out-of-range values */
	a.acc = 3.5f;
	a.gyro = -1.0e6f;
	(void)odo_encode_sensor(&a, buf, sizeof(buf));
	CHECK_EQ(feed(&r, buf, n, &b, &c), ODO_RX_SENSOR);
	CHECK_NEAR(b.acc, 3.5f, 1e-5);
	CHECK_NEAR(b.gyro, -2000.0f, 1e-2);
	a.gyro = 1.0e6f;
	(void)odo_encode_sensor(&a, buf, sizeof(buf));
	CHECK_EQ(feed(&r, buf, n, &b, &c), ODO_RX_SENSOR);
	CHECK_NEAR(b.gyro, 2000.0f, 1e-2);
	/* corrupted payload: CRC error, no frame */
	buf[10] ^= 0x40U;
	CHECK_EQ(feed(&r, buf, n, &b, &c), ODO_RX_NONE);
	CHECK_EQ(r.crc_err, 1);
	CHECK_EQ(r.ok, 3);
}

static void frame_sync_and_commands(void)
{
	odo_sample_t s;
	odo_cmd_t c;
	odo_rx_t r;
	/* noise, A5 A5 5A (resync on the second A5), a command frame */
	uint8_t f[] = {0x00, 0x13, 0xA5, 0x00, 0xA5, 0xA5, 0x5A, 0x05, 0x20, 0x03, 0x00, 0x01, 0x86, 0xA0, 0, 0};
	const uint16_t crc = odo_crc16(&f[7], 7U);

	f[14] = (uint8_t)(crc >> 8);
	f[15] = (uint8_t)crc;
	odo_rx_init(&r);
	CHECK_EQ(feed(&r, f, sizeof(f), &s, &c), ODO_RX_CMD);
	CHECK_EQ(c.cmd, ODO_CMD_CAL);
	CHECK_EQ(c.arg, 100000);
	/* a length above the largest frame is rejected at once */
	const uint8_t big[] = {0xA5, 0x5A, 0x40};

	CHECK_EQ(feed(&r, big, sizeof(big), &s, &c), ODO_RX_NONE);
	CHECK_EQ(r.crc_err, 1);
	CHECK_EQ(r.n, 0);
	/* a sensor frame with the wrong length */
	uint8_t w[] = {0xA5, 0x5A, 0x01, ODO_T_SENSOR, 0x00, 0, 0};
	const uint16_t kw = odo_crc16(&w[2], 3U);

	w[5] = (uint8_t)(kw >> 8);
	w[6] = (uint8_t)kw;
	CHECK_EQ(feed(&r, w, sizeof(w), &s, &c), ODO_RX_NONE);
	CHECK_EQ(r.crc_err, 2);
	r.crc_err = 1U;
	/* a valid frame of unknown type, and a command with the wrong length */
	uint8_t u[] = {0xA5, 0x5A, 0x01, 0x33, 0x00, 0, 0};
	uint16_t k = odo_crc16(&u[2], 3U);

	u[5] = (uint8_t)(k >> 8);
	u[6] = (uint8_t)k;
	CHECK_EQ(feed(&r, u, sizeof(u), &s, &c), ODO_RX_NONE);
	CHECK_EQ(r.crc_err, 2);
	u[3] = ODO_T_CMD;
	k = odo_crc16(&u[2], 3U);
	u[5] = (uint8_t)(k >> 8);
	u[6] = (uint8_t)k;
	CHECK_EQ(feed(&r, u, sizeof(u), &s, &c), ODO_RX_NONE);
	CHECK_EQ(r.crc_err, 3);
}

static void frame_status(void)
{
	static odo_unit_t u;
	uint8_t buf[64];
	odo_rx_t r;

	memset(&u, 0, sizeof(u));
	u.state = ODO_ST_RUN;
	u.flags = ODO_F_SLIDE;
	u.est.kf.x[0] = 12.34f;
	u.s = 1234.5f;
	u.last_d = 39.876f;
	u.n_brake = 2U;
	u.last_v0 = 13.9f;
	u.jr.seq = 4321U;
	u.jr.head = 321U;
	u.cfg.diam[0] = 0.655f;
	u.cfg.diam[1] = 0.64f;
	CHECK_EQ(odo_encode_status(&u, buf, 8U), 0);
	const uint32_t n = odo_encode_status(&u, buf, sizeof(buf));

	CHECK_EQ(n, ODO_STATUS_LEN + 6U);
	CHECK_EQ(buf[3], ODO_T_STATUS);
	CHECK_EQ(buf[4], ODO_ST_RUN);
	CHECK_EQ((buf[6] << 8) | buf[7], 1234);
	CHECK_EQ(((uint32_t)buf[8] << 24) | ((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | buf[11], 1234500U);
	CHECK_EQ(((uint32_t)buf[12] << 24) | ((uint32_t)buf[13] << 16) | ((uint32_t)buf[14] << 8) | buf[15], 39876U);
	CHECK_EQ(odo_crc16(&buf[2], n - 4U), (buf[n - 2] << 8) | buf[n - 1]);
	/* negative values are sent as 0 */
	u.est.kf.x[0] = -1.0f;
	u.s = -2.0f;
	u.last_d = -3.0f;
	u.last_v0 = -1.0f;
	(void)odo_encode_status(&u, buf, sizeof(buf));
	CHECK_EQ(buf[6] | buf[7] | buf[8] | buf[11] | buf[12] | buf[15] | buf[18] | buf[19], 0);
	odo_rx_init(&r);
	CHECK_EQ(r.n, 0);
}

void test_frame(void)
{
	frame_sensor_roundtrip();
	frame_sync_and_commands();
	frame_status();
}
