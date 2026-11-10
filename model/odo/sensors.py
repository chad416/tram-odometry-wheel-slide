"""Sensor models: wheel pulse generators (from the vehicle log), MEMS accelerometer, Doppler radar.

All error terms are drawn per run from the ranges in data.py; the draw is reproducible from the
generator passed in.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import data
from .vehicle import TS, Log


@dataclass
class Sensors:
    count: np.ndarray  # (R, S, 2) quadrature edge counts
    t_edge: np.ndarray  # (R, S, 2) last-edge time, s
    d_cfg: np.ndarray  # (R, 2) configured wheel diameters of the sensor axles, m
    acc: np.ndarray  # (R, S) measured specific force, m/s^2
    gyro: np.ndarray  # (R, S) measured pitch rate of the car body, rad/s
    radar: np.ndarray  # (R, S) measured speed over ground, m/s
    brake: np.ndarray  # (R, S) brake demand input
    t_on: np.ndarray  # (R,) capture time of the brake-input edge, s
    acc_bias: np.ndarray  # (R,) drawn offset (for the error budget)
    cal_err: np.ndarray  # (R, 2) relative diameter error d_cfg/d_true - 1


def make(
    log: Log,
    d_true_sens: np.ndarray,
    rng: np.random.Generator,
    cal_err_max: float = 0.0005,
    acc_bias_max: float = data.ACC_OFFSET_MAX,
    acc_scale_max: float = data.ACC_NONLIN + data.ACC_SENS_TC * 60.0,
    acc_noise: float = data.ACC_NOISE_DENSITY * np.sqrt(data.ACC_BW_HZ),
    radar_rel: float = data.RADAR_REL_ERR,
    temp_drift_c: float = 2.0,
    vib_rms: float = data.VIB_RMS,
    gyro_offset: float = data.GYR_OFFSET,
    gyro_tco: float = data.GYR_TCO,
    gyro_noise: float = data.GYR_NOISE_DENSITY * np.sqrt(data.ACC_BW_HZ),
    gyro_g_sens: float = data.GYR_G_SENS * data.GYR_G_SENS_RESIDUAL,
) -> Sensors:
    """Draw the sensor errors of each run and produce the measurements.

    cal_err_max: configured wheel diameter error bound (uniform), relative. The default 0.05 %
    is the calibrated case of DOSSIER Â§7.2; S4 sweeps it.
    """
    R, S = log.v.shape
    cal = rng.uniform(-cal_err_max, cal_err_max, (R, 2))
    d_cfg = d_true_sens * (1.0 + cal)
    bias = rng.uniform(-acc_bias_max, acc_bias_max, R)
    scale = rng.uniform(-acc_scale_max, acc_scale_max, R)
    drift = rng.uniform(-1.0, 1.0, R) * data.ACC_OFFSET_TC_MAX * temp_drift_c
    ramp = np.linspace(0.0, 1.0, S)[None, :]
    acc = log.f_spec * (1.0 + scale[:, None]) + bias[:, None] + drift[:, None] * ramp
    acc = (
        acc
        + rng.normal(0.0, acc_noise, (R, S))
        + _vibration(rng, R, S, vib_rms) * (np.minimum(1.0, log.v / 10.0) + 0.05)
    )
    # gyro: body pitch angle = gradient angle + suspension pitch (k a)
    rate = log.extra["pitch_rate"]  # body pitch rate: vertical curve + suspension
    g_off = rng.uniform(-gyro_offset, gyro_offset, R)
    g_drift = rng.uniform(-1.0, 1.0, R) * gyro_tco * data.GYR_DT_K
    g_sens = rng.uniform(-gyro_g_sens, gyro_g_sens, R)
    gyro = rate + g_off[:, None] + g_drift[:, None] * ramp + g_sens[:, None] * log.a
    gyro = gyro + rng.normal(0.0, gyro_noise, (R, S))
    rel = rng.uniform(-radar_rel, radar_rel, R)
    radar = np.maximum(log.v * (1.0 + rel[:, None]) + rng.normal(0.0, data.RADAR_NOISE, (R, S)), 0.0)
    return Sensors(log.count, log.t_edge, d_cfg, acc, gyro, radar, log.brake, log.extra["t_on"], bias, cal)


def _vibration(rng, R, S, rms):
    """Car-body longitudinal vibration: white noise through a 1-10 Hz band-pass (2nd-order
    Butterworth), scaled to rms. The body modes have no DC content, so the vibration does not
    integrate into a speed drift (A, DOSSIER §9.1)."""
    if rms <= 0.0:
        return np.zeros((R, S))
    from scipy import signal

    b, a = signal.butter(2, [1.0, 10.0], btype="band", fs=1.0 / TS)
    w = signal.lfilter(b, a, rng.normal(0.0, 1.0, (R, S + 500)), axis=1)[:, 500:]
    return w * (rms / np.std(w))


def edge_length(d_cfg: np.ndarray) -> np.ndarray:
    return np.pi * d_cfg / data.EDGES_PER_REV


def ts() -> float:
    return TS
