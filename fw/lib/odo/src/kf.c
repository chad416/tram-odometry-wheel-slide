/*
 * 3-state Kalman filter (OR-12): x = [v, c, b]
 *   v  speed, m/s
 *   c  accelerometer offset: sensor bias + gravity along the tilted car body, m/s^2
 *   b  gyro zero-rate offset, rad/s
 * Prediction with the accelerometer f and the pitch gyro w (tick ODO_TS):
 *   v += (f - c) T,   c += g (w - b) T,   b unchanged
 * Scalar updates of one state at a time (wheel speed, standstill speed, f = c, w = b).
 * SPDX-License-Identifier: MIT
 */
#include "odo/odo.h"

void odo_kf_init(odo_kf_t *k, float c0)
{
	for (uint32_t i = 0U; i < 3U; i++) {
		k->x[i] = 0.0f;
		for (uint32_t j = 0U; j < 3U; j++) {
			k->p[i][j] = 0.0f;
		}
	}
	k->x[1] = c0;
	k->p[0][0] = 1e-4f;
	k->p[1][1] = 1e-2f;
	k->p[2][2] = 3.0e-4f; /* (1 deg/s)^2: the data-sheet zero-rate offset */
}

void odo_kf_predict(odo_kf_t *k, float acc, float gyro)
{
	const float t = ODO_TS;
	const float gt = ODO_G * ODO_TS;
	float a[3][3];
	float n[3][3];

	k->x[0] += (acc - k->x[1]) * t;
	k->x[1] += (gyro - k->x[2]) * gt;

	/* F = [[1, -t, 0], [0, 1, -gt], [0, 0, 1]];  P = F P F' + Q */
	for (uint32_t j = 0U; j < 3U; j++) {
		a[0][j] = k->p[0][j] - (t * k->p[1][j]);
		a[1][j] = k->p[1][j] - (gt * k->p[2][j]);
		a[2][j] = k->p[2][j];
	}
	for (uint32_t i = 0U; i < 3U; i++) {
		n[i][0] = a[i][0] - (t * a[i][1]);
		n[i][1] = a[i][1] - (gt * a[i][2]);
		n[i][2] = a[i][2];
	}
	n[0][0] += ODO_Q_V * ODO_Q_V * t;
	n[1][1] += ODO_Q_C * ODO_Q_C * t;
	n[2][2] += ODO_Q_B * ODO_Q_B * t;
	for (uint32_t i = 0U; i < 3U; i++) {
		for (uint32_t j = 0U; j < 3U; j++) {
			k->p[i][j] = n[i][j];
		}
	}
}

void odo_kf_update(odo_kf_t *k, uint32_t j, float z, float sig)
{
	const float s = k->p[j][j] + (sig * sig);
	const float e = z - k->x[j];
	float g[3];
	float row[3];

	for (uint32_t i = 0U; i < 3U; i++) {
		g[i] = k->p[i][j] / s;
		row[i] = k->p[j][i];
	}
	for (uint32_t i = 0U; i < 3U; i++) {
		k->x[i] += g[i] * e;
		for (uint32_t m = 0U; m < 3U; m++) {
			k->p[i][m] -= g[i] * row[m];
		}
	}
}

void odo_kf_set_speed(odo_kf_t *k, float v, float sig)
{
	k->x[0] = v;
	for (uint32_t i = 0U; i < 3U; i++) {
		k->p[0][i] = 0.0f;
		k->p[i][0] = 0.0f;
	}
	k->p[0][0] = sig * sig;
}
