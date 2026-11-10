"""The estimators on a synthetic run: constant cruise, emergency brake with the wheels sliding."""

import numpy as np
import pytest
from odo import estimator as E
from odo import sensors, vehicle

TS = vehicle.TS
IDEAL = dict(
    cal_err_max=0.0,
    acc_bias_max=0.0,
    acc_scale_max=0.0,
    acc_noise=0.0,
    temp_drift_c=0.0,
    vib_rms=0.0,
    gyro_offset=0.0,
    gyro_tco=0.0,
    gyro_noise=0.0,
    gyro_g_sens=0.0,
)


def synthetic(slip=-0.2, v0=12.0, decel=2.5, offset=0.3):
    """One run: 5 s standstill, 1 m/s^2 to v0, 20 s cruise, brake at decel to rest, 3 s rest.
    During the brake both sensor wheels turn (1 + slip) times the vehicle speed."""
    t_acc = v0 / 1.0
    t_b = 5.0 + t_acc + 20.0
    t_end = t_b + v0 / decel + 3.0
    t = np.arange(0.0, t_end, TS)
    v = np.where(t < t_b, np.minimum(v0, np.maximum(t - 5.0, 0.0)), np.maximum(v0 - decel * (t - t_b), 0.0))
    a = np.diff(v, prepend=0.0) / TS
    s = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1])) * TS])
    brake = t >= t_b
    wr = v * np.where(brake, 1.0 + slip, 1.0)
    d = 0.65
    theta = np.concatenate([[0.0], np.cumsum(0.5 * (wr[1:] + wr[:-1])) * TS]) / (d / 2.0)
    cnt, te = vehicle._edges(theta, t)
    S = len(t)
    log = vehicle.Log(
        t=t,
        s=s[None],
        v=v[None],
        a=a[None],
        f_spec=(a + offset)[None],
        count=np.stack([cnt, cnt], -1)[None],
        t_edge=np.stack([te, te], -1)[None],
        w_axle=np.stack([wr, wr], -1)[None],
        brake=brake[None],
        grade=np.zeros((1, S)),
        i_brake=np.array([np.argmax(brake)]),
        i_stop=np.array([np.argmax((v <= 0) & brake)]),
        extra={
            "pitch_rate": np.zeros((1, S)),
            "t_on": np.array([t[np.argmax(brake)] - 0.004]),
            "s_on": np.array([np.interp(t[np.argmax(brake)] - 0.004, t, s)]),
        },
    )
    return log, np.array([[d, d]])


@pytest.fixture(scope="module")
def run():
    log, d = synthetic()
    sen = sensors.make(log, d, np.random.default_rng(1), **IDEAL)
    d_true = log.s[0, log.i_stop[0]] - log.extra["s_on"][0]
    return log, sen, d_true


def test_one_wheel_sensor_underreads_by_the_slip(run):
    log, sen, d_true = run
    r = E.e1_single(sen)
    assert r.d_brake[0] == pytest.approx(0.8 * d_true, rel=0.01)


def test_e3_recovers_the_distance(run):
    log, sen, d_true = run
    r = E.fused(sen)
    assert abs(r.d_brake[0] - d_true) < 0.02
    assert abs(r.v0[0] - 12.0) < 0.02
    assert abs(r.i_stop[0] - log.i_stop[0]) <= 1


def test_no_slide_uses_the_wheels():
    log, d = synthetic(slip=0.0)
    sen = sensors.make(log, d, np.random.default_rng(1), **IDEAL)
    d_true = log.s[0, log.i_stop[0]] - log.extra["s_on"][0]
    assert abs(E.fused(sen).d_brake[0] - d_true) < 0.02
    assert abs(E.e1_single(sen).d_brake[0] - d_true) < 0.02


def test_boundary_correction_removes_a_constant_offset_error():
    log, d = synthetic()
    sen = sensors.make(log, d, np.random.default_rng(1), **IDEAL)
    d_true = log.s[0, log.i_stop[0]] - log.extra["s_on"][0]
    # an offset step of 0.05 m/s^2 at the onset, not seen by the filter
    sen.acc[0, log.i_brake[0] :] += 0.05
    r = E.fused(sen, "inertial")
    assert abs(r.d_brake[0] - d_true) < 0.03
