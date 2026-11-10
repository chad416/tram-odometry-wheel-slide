"""Odometry estimators, vectorised over the runs of a batch.

E1  one wheel pulse sensor (today's recorder): distance = edges x edge length
E2  two wheel sensors on different bogies, larger increment per tick
E3  two wheel sensors + MEMS accelerometer: 2-state Kalman filter (speed v, accelerometer offset
    c = bias + gravity on the gradient), slide/spin gate on the wheel speeds, standstill update,
    and the emergency-brake policy chosen in DOSSIER §8: during an emergency brake the wheels are
    not used at all; the distance is the accelerometer integral from the speed at the onset,
    corrected so that the speed is zero at the detected standstill (two-point boundary
    correction). This is the algorithm of fw/lib/odo (est.c, brake.c), in double precision.
E3h as E3 but the brake distance is the wheel distance when no slide is seen during the brake
E3g as E3 but the wheels stay in use during the brake whenever the gate accepts them
E4  Doppler radar only
E5  E3 + radar speed updates (also during the brake)

Every estimator returns the estimated distance series s_hat (R, S) and the brake result:
distance from the brake onset to the detected standstill, the detected stop sample, and the speed
at the onset. For E3 the brake segment of s_hat is replaced by the corrected profile.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sensors import Sensors, edge_length
from .vehicle import TS

# ---- Parameters shared with the firmware (fw/lib/odo/include/odo/odo.h) --------------------
STAND_T = 0.30  # s without an edge -> that wheel is at rest
GATE_ABS = 0.05  # m/s: wheel-speed gate, absolute part
GATE_REL = 0.005  # wheel-speed gate, part proportional to the speed
A_GATE = 0.5  # m/s^2: wheel acceleration must match the inertial one (above V_AGATE)
V_AGATE = 1.0  # m/s
SLIDE_DV_ABS = 0.05  # m/s: bogie disagreement that marks a slide in the brake
SLIDE_DV_REL = 0.01
SLIDE_A = 2.0  # m/s^2: wheel acceleration beyond the vehicle that marks a slide
TRUST_HOLD = 0.50  # s a gated wheel must stay consistent before it is used again
A_STOP = 0.3  # m/s^2: |f - c| below this ends the deceleration (start of the standstill)
V_STOP = 0.5  # m/s: the inertial speed must be below this to accept the standstill
A_ZUPT = 0.15  # m/s^2: standstill updates only while |f - c| is this small (departure)
STAND_MAX_T = 1.0  # s of quiet after which the standstill is accepted without the speed check
SIG_W_ABS = 0.02  # m/s: wheel-speed measurement noise, absolute part
SIG_W_REL = 0.003  # relative part
Q_V = 0.10  # m/s^2/sqrt(s): process noise on v (vibration, accelerometer noise)
Q_C = 0.05  # m/s^3/sqrt(s): process noise on c (gradient changes, drift)
SIG_ZUPT = 0.002  # m/s
REACQ_T = 5.0  # s without an accepted wheel (wheels agreeing) before re-acquisition
SIG_ZACC = 0.02  # m/s^2: standstill update of the offset (f = c)
SIG_ZARU = 0.002  # rad/s: standstill update of the gyro offset
GYRO_ARW = 0.014  # deg/s/sqrt(Hz) [R20]
Q_C_GYRO = 0.005  # m/s^3/sqrt(s): unmodelled c dynamics with the gyro
Q_B = 2e-5  # rad/s/sqrt(s): gyro offset random walk
SIG_RADAR_ABS = 0.05
SIG_RADAR_REL = 0.005


_DEBUG = None


@dataclass
class Result:
    name: str
    s_hat: np.ndarray  # (R, S) estimated distance
    d_brake: np.ndarray  # (R,) brake distance, onset to detected standstill
    i_stop: np.ndarray  # (R,) detected standstill sample
    v0: np.ndarray  # (R,) speed at the onset
    v_hat: np.ndarray | None = None
    c_hat: np.ndarray | None = None
    b_hat: np.ndarray | None = None


def _onset(sen: Sensors) -> np.ndarray:
    return np.argmax(sen.brake, axis=1)


def _onset_frac(sen: Sensors, i_on: np.ndarray) -> np.ndarray:
    """Time from the captured brake edge to the onset sample, s (all estimators use it)."""
    return np.clip(i_on * TS - sen.t_on, 0.0, TS)


def wheel_distance(sen: Sensors, axle: int) -> np.ndarray:
    return sen.count[:, :, axle] * edge_length(sen.d_cfg[:, axle])[:, None]


def _wheel_stop(sen: Sensors, i_on: np.ndarray, axles=(0, 1)) -> np.ndarray:
    """First sample after the onset from which no axle shows another edge."""
    R, S, _ = sen.count.shape
    out = np.zeros(R, dtype=int)
    for i in range(R):
        last = i_on[i]
        for j in axles:
            c = sen.count[i, :, j]
            ch = np.nonzero(np.diff(c[i_on[i] :]) != 0)[0]
            if len(ch):
                last = max(last, i_on[i] + ch[-1] + 1)
        out[i] = last
    return out


def e1_single(sen: Sensors) -> Result:
    s = wheel_distance(sen, 0)
    i_on = _onset(sen)
    i_st = _wheel_stop(sen, i_on, (0,))
    r = np.arange(s.shape[0])
    v0 = _mt_speed_at(sen, 0, i_on)
    d = s[r, i_st] - s[r, i_on] + v0 * _onset_frac(sen, i_on)
    return Result("E1 one wheel sensor", s, d, i_st, v0)


def e2_two_max(sen: Sensors) -> Result:
    s1 = wheel_distance(sen, 0)
    s2 = wheel_distance(sen, 1)
    d = np.maximum(np.diff(s1, axis=1, prepend=0.0), np.diff(s2, axis=1, prepend=0.0))
    s = np.cumsum(d, axis=1)
    i_on = _onset(sen)
    i_st = _wheel_stop(sen, i_on)
    r = np.arange(s.shape[0])
    v0 = np.maximum(_mt_speed_at(sen, 0, i_on), _mt_speed_at(sen, 1, i_on))
    d = s[r, i_st] - s[r, i_on] + v0 * _onset_frac(sen, i_on)
    return Result("E2 two wheel sensors (max)", s, d, i_st, v0)


def e4_radar(sen: Sensors) -> Result:
    v = sen.radar
    s = np.concatenate(
        [np.zeros((v.shape[0], 1)), np.cumsum(0.5 * (v[:, 1:] + v[:, :-1]) * TS, axis=1)], axis=1
    )
    i_on = _onset(sen)
    R, S = v.shape
    i_st = np.zeros(R, dtype=int)
    for i in range(R):
        low = v[i, i_on[i] :] < 0.15
        # first sample of a run of 0.3 s below 0.15 m/s
        k = int(round(STAND_T / TS))
        run = np.convolve(low.astype(int), np.ones(k, int), "valid") == k
        j = int(np.argmax(run)) if np.any(run) else len(low) - 1
        i_st[i] = i_on[i] + j
    r = np.arange(R)
    d = s[r, i_st] - s[r, i_on] + v[r, i_on] * _onset_frac(sen, i_on)
    return Result("E4 Doppler radar", s, d, i_st, v[r, i_on])


def _mt_speed_at(sen: Sensors, axle: int, idx: np.ndarray) -> np.ndarray:
    """M/T speed over the last 0.1 s before sample idx (for the E1/E2 v0 report)."""
    r = np.arange(sen.count.shape[0])
    k = 10
    c1 = sen.count[r, idx, axle]
    c0 = sen.count[r, np.maximum(idx - k, 0), axle]
    t1 = sen.t_edge[r, idx, axle]
    t0 = sen.t_edge[r, np.maximum(idx - k, 0), axle]
    le = edge_length(sen.d_cfg[:, axle])
    return np.where(t1 > t0, (c1 - c0) * le / np.maximum(t1 - t0, 1e-9), 0.0)


def fused(
    sen: Sensors,
    brake_policy: str = "inertial",
    use_radar: bool = False,
    use_gyro: bool = True,
    name: str | None = None,
) -> Result:
    """E3 / E3a / E3g / E5.

    State x = [v, c, b]: speed, accelerometer offset (bias + gravity on the body pitch), gyro
    zero-rate offset. Prediction with the accelerometer f and the pitch gyro w:
        dv/dt = f - c,   dc/dt = g (w - b),   db/dt = 0
    Updates: accepted wheel speeds (H = [1 0 0]); at standstill outside a brake event the speed
    (0), the offset (f = c) and the gyro offset (w = b). brake_policy "inertial" (E3, chosen):
    no wheel update during an emergency brake, two-point boundary correction at the standstill;
    "gate" (E3g): wheels used whenever the gate accepts them. use_gyro=False (E3a): c is a random
    walk and the gyro is not used.
    """
    R, S, _ = sen.count.shape
    g0 = 9.80665
    le = edge_length(sen.d_cfg)  # (R, 2)
    acc = sen.acc
    gyr = sen.gyro if use_gyro else np.zeros_like(acc)
    x = np.zeros((R, 3))
    x[:, 1] = acc[:, 0]
    P = np.zeros((R, 3, 3))
    P[:, 0, 0] = 1e-4
    P[:, 1, 1] = 1e-2
    P[:, 2, 2] = (np.radians(1.0)) ** 2 if use_gyro else 1e-12
    F = np.array([[1.0, -TS, 0.0], [0.0, 1.0, -g0 * TS], [0.0, 0.0, 1.0]])
    if not use_gyro:
        F[1, 2] = 0.0
    qc = (g0 * np.radians(GYRO_ARW)) ** 2 + Q_C_GYRO**2 if use_gyro else Q_C**2
    Q = np.diag([Q_V**2 * TS, qc * TS, (Q_B**2 * TS) if use_gyro else 0.0])
    n_last = sen.count[:, 0, :].astype(float)
    t_last = np.where(sen.t_edge[:, 0, :] >= 0.0, sen.t_edge[:, 0, :], 0.0)
    vw = np.zeros((R, 2))
    vw_prev = np.zeros((R, 2))
    trust_t = np.zeros((R, 2))
    lost_t = np.zeros(R)
    s = np.zeros(R)
    s_hat = np.zeros((R, S))
    v_hat = np.zeros((R, S))
    c_hat = np.zeros((R, S))
    b_hat = np.zeros((R, S))
    in_ev = np.zeros(R, bool)
    done = np.zeros(R, bool)
    ev_v = np.zeros(R)
    ev_d = np.zeros(R)
    ev_s0 = np.zeros(R)
    ev_i0 = np.zeros(R, dtype=int)
    quiet = np.zeros(R, dtype=int)
    q_v = np.zeros(R)
    q_d = np.zeros(R)
    q_t = np.zeros(R)
    ev_t = np.zeros(R)
    ev_w = np.ones(R)
    ev_dw = np.zeros(R)
    ev_slid = np.zeros(R, bool)
    q_dw = np.zeros(R)
    wheel_used = np.zeros(R, bool)
    d_brake = np.zeros(R)
    i_stop = np.zeros(R, dtype=int)
    v0 = np.zeros(R)
    corr_r = np.zeros(R)
    corr_T = np.ones(R)
    k_stand = int(round(STAND_T / TS))
    k_stand_max = int(round(STAND_MAX_T / TS))
    prev_brake = np.zeros(R, bool)
    for k in range(S):
        t = k * TS
        # ---- wheel speeds (M/T) ----
        cnt = sen.count[:, k, :].astype(float)
        te = sen.t_edge[:, k, :]
        new = cnt > n_last
        vmt = np.where(new, (cnt - n_last) * le / np.maximum(te - t_last, 1e-6), 0.0)
        bound = le / np.maximum(t - t_last, 1e-6)
        vw = np.where(new, vmt, np.minimum(vw, bound))
        vw = np.where(~new & (t - t_last > STAND_T), 0.0, vw)
        t_last = np.where(new, te, t_last)
        n_last = np.where(new, cnt, n_last)
        # ---- predict ----
        f = acc[:, k]
        w = gyr[:, k]
        x = x @ F.T
        x[:, 0] += f * TS
        if use_gyro:
            x[:, 1] += g0 * w * TS
        P = F @ P @ F.T + Q
        # ---- brake event start ----
        rise = sen.brake[:, k] & ~prev_brake & ~in_ev & ~done
        prev_brake = sen.brake[:, k]
        ev_v = np.where(rise, x[:, 0], ev_v)
        v0 = np.where(rise, x[:, 0], v0)
        ev_s0 = np.where(rise, s, ev_s0)
        ev_d = np.where(rise, 0.0, ev_d)
        ev_dw = np.where(rise, 0.0, ev_dw)
        ev_slid = np.where(rise, False, ev_slid)
        # part of the onset tick after the captured brake edge (OR-20)
        ev_w = np.where(rise, np.clip((t - sen.t_on) / TS, 0.0, 1.0), ev_w)
        ev_i0 = np.where(rise, k, ev_i0)
        quiet = np.where(rise, 0, quiet)
        in_ev |= rise
        v = x[:, 0]
        # ---- wheel gate ----
        gate = GATE_ABS + GATE_REL * np.abs(v)[:, None]
        aw = (vw - vw_prev) / TS
        acc_ok = (np.abs(aw - (f - x[:, 1])[:, None]) < A_GATE) | (vw < V_AGATE)
        vw_prev = vw.copy()
        # a wheel at rest gives no speed measurement (the standstill update does)
        ok = (np.abs(vw - v[:, None]) < gate) & acc_ok & (vw > 0.0)
        # slide in the brake (E3h): the two bogies disagree, or a wheel decelerates far
        # beyond the vehicle (robust against the drift of the inertial speed and the noise)
        slide_now = (np.abs(vw[:, 0] - vw[:, 1]) > SLIDE_DV_ABS + SLIDE_DV_REL * np.abs(v)) | (
            np.abs(aw - (f - x[:, 1])[:, None]) > SLIDE_A
        ).any(axis=1)
        ev_slid |= in_ev & slide_now & (v > V_AGATE)
        if _DEBUG is not None:
            _DEBUG.append(
                (k, ev_slid.copy(), vw.copy(), v.copy(), (f - x[:, 1]).copy(), ok.copy(), in_ev.copy())
            )
        trust_t = np.where(ok, trust_t + TS, 0.0)
        use = trust_t >= TRUST_HOLD
        if brake_policy in ("inertial", "hybrid"):
            use &= ~in_ev[:, None]
        agree = np.abs(vw[:, 0] - vw[:, 1]) < (GATE_ABS + GATE_REL * np.abs(vw).max(axis=1))
        none_used = ~use.any(axis=1)
        lost_t = np.where(none_used & agree & ~in_ev & ~sen.brake[:, k], lost_t + TS, 0.0)
        reacq = lost_t >= REACQ_T
        if np.any(reacq):
            x[:, 0] = np.where(reacq, vw.mean(axis=1), x[:, 0])
            P[:, 0, :] = np.where(reacq[:, None], 0.0, P[:, 0, :])
            P[:, :, 0] = np.where(reacq[:, None], 0.0, P[:, :, 0])
            P[:, 0, 0] = np.where(reacq, (SIG_W_ABS + SIG_W_REL * np.abs(x[:, 0])) ** 2, P[:, 0, 0])
            P[:, 1, 1] = np.where(reacq, P[:, 1, 1] + 1e-2, P[:, 1, 1])  # the offset is suspect too
            trust_t = np.where(reacq[:, None], TRUST_HOLD, trust_t)
            use = np.where(reacq[:, None], True, use)
            lost_t = np.where(reacq, 0.0, lost_t)
        n_use = use.sum(axis=1)
        z = np.where(n_use > 0, (vw * use).sum(axis=1) / np.maximum(n_use, 1), 0.0)
        rest = (vw[:, 0] == 0.0) & (vw[:, 1] == 0.0)
        zupt = rest & ~in_ev & (np.abs(f - x[:, 1]) < A_ZUPT)
        zz = np.where(zupt, 0.0, z)
        sz = np.where(zupt, SIG_ZUPT, SIG_W_ABS + SIG_W_REL * np.abs(z))
        x, P = _upd(x, P, 0, zz, sz, (n_use > 0) | zupt)
        x, P = _upd(x, P, 1, f, SIG_ZACC, zupt)
        if use_gyro:
            x, P = _upd(x, P, 2, w, SIG_ZARU, zupt)
        if use_radar:
            zr = sen.radar[:, k]
            x, P = _upd(x, P, 0, zr, SIG_RADAR_ABS + SIG_RADAR_REL * zr, np.ones(R, bool))
        # ---- brake event integration (the KF speed is the inertial speed in the event) ----
        ev_prev_v = ev_v.copy()
        ev_v = np.where(in_ev, x[:, 0], ev_v)
        inc = 0.5 * (ev_prev_v + ev_v) * TS * ev_w
        inc_w = vw.mean(axis=1) * TS * ev_w
        ev_dw = np.where(in_ev, ev_dw + inc_w, ev_dw)
        ev_d = np.where(in_ev, ev_d + inc, ev_d)
        ev_t = np.where(in_ev, ev_t + TS * ev_w, ev_t)
        ev_t = np.where(rise, TS * ev_w, ev_t)
        ev_w = np.where(in_ev, 1.0, ev_w)
        still = np.abs(f - x[:, 1]) < A_STOP
        start_q = in_ev & still & (quiet == 0)
        q_v = np.where(start_q, ev_prev_v, q_v)
        q_d = np.where(start_q, ev_d - inc, q_d)
        q_dw = np.where(start_q, ev_dw - inc_w, q_dw)
        q_t = np.where(start_q, ev_t - TS, q_t)
        quiet = np.where(in_ev & still, quiet + 1, 0)
        # the speed check is waived after STAND_MAX quiet ticks (accelerometer-only fallback)
        fin = in_ev & (quiet >= k_stand) & rest & ((np.abs(ev_v) < V_STOP) | (quiet >= k_stand_max))
        if np.any(fin):
            T = np.maximum(q_t, TS)
            dc = q_d - 0.5 * q_v * T
            if brake_policy == "hybrid":
                # E3h: no slide seen in the whole brake: the wheels measure it
                dc = np.where(ev_slid, dc, q_dw)
                wheel_used = np.where(fin, ~ev_slid, wheel_used)
            d_brake = np.where(fin, dc, d_brake)
            i_stop = np.where(fin, k - quiet + 1, i_stop)
            corr_r = np.where(fin, q_v, corr_r)
            corr_T = np.where(fin, T, corr_T)
            if brake_policy in ("inertial", "hybrid"):
                s = np.where(fin, ev_s0 + dc - x[:, 0] * TS, s)
                x[:, 1] = np.where(fin, x[:, 1] + q_v / T, x[:, 1])
                x[:, 0] = np.where(fin, 0.0, x[:, 0])
            done |= fin
            in_ev &= ~fin
        if brake_policy in ("inertial", "hybrid"):
            s = np.where(in_ev, ev_s0 + ev_d, s + x[:, 0] * TS)
        else:
            s = s + x[:, 0] * TS
        s_hat[:, k] = s
        v_hat[:, k] = x[:, 0]
        c_hat[:, k] = x[:, 1]
        b_hat[:, k] = x[:, 2]
    if brake_policy in ("inertial", "hybrid"):
        # replace the live brake segment by the corrected profile (what the evaluator shows)
        for i in range(R):
            i0, i1 = ev_i0[i], i_stop[i]
            if i1 > i0:
                tt = (np.arange(i0, S) - i0 + 1) * TS
                if wheel_used[i]:
                    end_err = s_hat[i, i1] - (ev_s0[i] + d_brake[i])
                    corr = end_err * np.minimum(tt, corr_T[i]) / corr_T[i]
                else:
                    corr = 0.5 * corr_r[i] * np.minimum(tt, corr_T[i]) ** 2 / corr_T[i]
                s_hat[i, i0:] = s_hat[i, i0:] - corr
                s_hat[i, i1:] = s_hat[i, i1]
    else:
        r = np.arange(R)
        i_stop = np.where(i_stop > 0, i_stop, _wheel_stop(sen, ev_i0))
        d_brake = s_hat[r, i_stop] - s_hat[r, ev_i0] + v0 * _onset_frac(sen, ev_i0)
    if name is None:
        if use_radar:
            name = "E5 wheels + IMU + radar"
        elif brake_policy == "gate":
            name = "E3g wheels + IMU, gate only"
        elif brake_policy == "hybrid":
            name = "E3h wheels + IMU, wheel distance when no slide is seen"
        elif not use_gyro:
            name = "E3a wheels + accelerometer (no gyro)"
        else:
            name = "E3 wheels + IMU (accelerometer + pitch gyro)"
    return Result(name, s_hat, d_brake, i_stop, v0, v_hat, c_hat, b_hat)


def _upd(x, P, j, z, sig, mask):
    """Scalar measurement z of state j with noise sig, applied where mask is true."""
    s = P[:, j, j] + sig * sig
    K = P[:, :, j] / s[:, None]
    e = z - x[:, j]
    xn = x + K * e[:, None]
    Pn = P - K[:, :, None] * P[:, j, None, :]
    m = mask[:, None]
    return np.where(m, xn, x), np.where(m[:, :, None], Pn, P)
