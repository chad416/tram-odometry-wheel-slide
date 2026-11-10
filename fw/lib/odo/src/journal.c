/*
 * Power-fail-safe record ring in FRAM (OR-30..OR-34).
 *
 * Every record is one 16-byte slot: sequence number, type, flags, payload and a CRC-16 over
 * the first 14 bytes. Records are written in order, one slot after the other, and wrap. A
 * write cut by a supply loss leaves a slot whose CRC fails; the recovery ignores it. The
 * newest record is the valid one with the largest sequence number in serial-number
 * arithmetic (16-bit sequence, 8192 slots, so all valid records lie within half the range).
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

static void le_put16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFU);
	p[1] = (uint8_t)(v >> 8U);
}

static uint16_t le_get16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8U));
}

static void entry_pack(const odo_entry_t *e, uint8_t out[ODO_SLOT_BYTES])
{
	le_put16(&out[0], e->seq);
	out[2] = e->type;
	out[3] = e->flags;
	le_put16(&out[4], (uint16_t)(e->s_cm & 0xFFFFU));
	le_put16(&out[6], (uint16_t)(e->s_cm >> 16U));
	le_put16(&out[8], e->v_cms);
	le_put16(&out[10], e->t_ds);
	le_put16(&out[12], e->aux);
	le_put16(&out[14], odo_crc16(out, ODO_SLOT_BYTES - 2U));
}

bool odo_entry_unpack(const uint8_t in[ODO_SLOT_BYTES], odo_entry_t *e)
{
	const uint16_t crc = le_get16(&in[14]);
	const bool ok = (crc == odo_crc16(in, ODO_SLOT_BYTES - 2U)) && (in[2] != (uint8_t)ODO_REC_NONE);

	e->seq = le_get16(&in[0]);
	e->type = in[2];
	e->flags = in[3];
	e->s_cm = (uint32_t)le_get16(&in[4]) | ((uint32_t)le_get16(&in[6]) << 16U);
	e->v_cms = le_get16(&in[8]);
	e->t_ds = le_get16(&in[10]);
	e->aux = le_get16(&in[12]);
	e->crc = crc;
	return ok;
}

void odo_journal_recover(odo_journal_t *j, const odo_nvm_t *nvm)
{
	uint8_t buf[ODO_SLOT_BYTES];
	odo_entry_t e;
	bool any = false;
	uint16_t ref = 0U;
	int32_t best = 0;
	uint32_t best_slot = 0U;
	uint16_t best_seq = 0U;

	j->nvm = *nvm;
	j->valid = 0U;
	j->written = 0U;
	j->torn = 0U;
	for (uint32_t i = 0U; i < ODO_SLOTS; i++) {
		nvm->read(nvm->ctx, i * ODO_SLOT_BYTES, buf, ODO_SLOT_BYTES);
		if (odo_entry_unpack(buf, &e)) {
			if (!any) {
				any = true;
				ref = e.seq;
				best = 0;
				best_slot = i;
				best_seq = e.seq;
			}
			/* serial-number distance from the first valid record found */
			const int32_t d = (int32_t)(int16_t)(uint16_t)(e.seq - ref);

			if (d > best) {
				best = d;
				best_slot = i;
				best_seq = e.seq;
			}
			j->valid++;
		}
	}
	if (any) {
		j->head = (best_slot + 1U) % ODO_SLOTS;
		j->seq = (uint16_t)(best_seq + 1U);
	} else {
		j->head = 0U;
		j->seq = 0U;
	}
}

bool odo_journal_append(odo_journal_t *j, odo_entry_t *e)
{
	uint8_t buf[ODO_SLOT_BYTES];
	bool ok;

	e->seq = j->seq;
	entry_pack(e, buf);
	e->crc = (uint16_t)((uint16_t)buf[14] | (uint16_t)((uint16_t)buf[15] << 8U));
	ok = j->nvm.write(j->nvm.ctx, j->head * ODO_SLOT_BYTES, buf, ODO_SLOT_BYTES) == ODO_SLOT_BYTES;
	if (ok) {
		j->head = (j->head + 1U) % ODO_SLOTS;
		j->seq = (uint16_t)(j->seq + 1U);
		j->written++;
	} else {
		j->torn++;
	}
	return ok;
}

bool odo_journal_get(const odo_journal_t *j, uint32_t back, odo_entry_t *e)
{
	uint8_t buf[ODO_SLOT_BYTES];
	const uint32_t slot = ((j->head + ODO_SLOTS) - 1U - (back % ODO_SLOTS)) % ODO_SLOTS;

	j->nvm.read(j->nvm.ctx, slot * ODO_SLOT_BYTES, buf, ODO_SLOT_BYTES);
	return odo_entry_unpack(buf, e);
}
