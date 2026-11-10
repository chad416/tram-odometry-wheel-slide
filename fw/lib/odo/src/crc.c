/*
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR).
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

uint16_t odo_crc16(const uint8_t *data, uint32_t len)
{
	uint16_t crc = 0xFFFFU;

	for (uint32_t i = 0U; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)data[i] << 8U);
		for (uint32_t b = 0U; b < 8U; b++) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
			} else {
				crc = (uint16_t)(crc << 1U);
			}
		}
	}
	return crc;
}
