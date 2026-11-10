"""Longitudinal tram model with per-axle wheel dynamics, Polach creep and wheel slide protection.

One call simulates a batch of runs (numpy arrays of shape (runs,) and (runs, axles)). Each run:
start from rest, accelerate to the cruise speed, cruise, emergency brake at a given position, stop,
then 2 s at standstill. The last 1500 m before the stop are therefore all inside the run.

Two phases:

- **Approach** (from rest to 3 s before the brake onset). Wheel inertia does not matter while no
  wheel slips, so the wheels roll with the quasi-static Polach creep that carries the traction or
  cruise force (creep velocity from the inverted creep curve).
- **Brake** (from 3 s before the onset to 2 s after standstill). Full dynamics at 2 ms:

      vehicle   m dv/dt = sum_k Fx_k - F_mtb - F_res(v) - m g sin(gamma(s))
      axle k    J d(omega_k)/dt = T_k - r_k Fx_k,   Fx_k = Q_axle f(|omega_k r_k - v|) sign(...)

  The wheel equation is stiff, so omega_{k+1} is solved implicitly (safeguarded Newton). Wheel
  slide protection (concept of EN 15595 [R8]) scales the torque of each axle by u_k in [0, 1]:
  release above WSP_RELEASE_SLIP or WSP_RELEASE_DECEL, re-apply below WSP_REAPPLY_SLIP.

Outputs are sampled at 100 Hz (the recorder's tick) and contain what the sensors see: the
quadrature edge count and the time of the last edge on two instrumented axles, and the specific
force along the car body (acceleration + gravity on the gradient + body pitch).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import adhesion, data

SENSOR_AXLES = (0, 4)  # instrumented axles: first axle of bogie 1 and of bogie 3 (A)
AXLE_POS = np.array([0.0, 1.8, 10.5, 12.3, 21.0, 22.8])  # m behind the front axle (A)
GRID = 1.0  # m, resolution of the rail-condition map
TS = 0.010  # s, sensor sample period (the firmware's tick)
A_TRACTION = 1.0  # m/s^2, A: acceleration from rest
T_PRE = 3.0  # s of full dynamics before the brake onset
T_DWELL = 5.0  # s at standstill at the start of each run (a tram stop)
DT_BRAKE = 0.002  # s
NEWTON_IT = 60  # upper bound; the loop stops at convergence
NEWTON_TOL = 1e-6  # rad/s


@dataclass
class Batch:
    """Inputs of a batch of runs (all arrays have the run count R as first dimension)."""

    v_cruise: np.ndarray  # m/s
    s_brake: np.ndarray  # m, brake onset position of the front axle
    cond: np.ndarray  # (R, n_cells) int: rail condition per GRID cell (adhesion.DRY..)
    grade_a: np.ndarray  # gradient before the vertical curve (rise/run, + = uphill)
    grade_b: np.ndarray  # gradient after it
    s_vc: np.ndarray  # m, start of the vertical curve
    l_vc: np.ndarray  # m, its length
    mass: np.ndarray  # kg
    d_true: np.ndarray  # (R, N_AXLES) true wheel diameters, m

    @property
    def runs(self) -> int:
        return int(self.v_cruise.shape[0])


@dataclass
class Log:
    t: np.ndarray  # (S,)
    s: np.ndarray  # (R, S) true distance of the front axle
    v: np.ndarray  # (R, S) true speed
    a: np.ndarray  # (R, S) true acceleration
    f_spec: np.ndarray  # (R, S) specific force along the car body (accelerometer truth)
    count: np.ndarray  # (R, S, 2) edge counts of the sensor axles
    t_edge: np.ndarray  # (R, S, 2) time of the last edge, s (-1 before the first)
    w_axle: np.ndarray  # (R, S, 2) true peripheral speed omega r of the sensor axles
    brake: np.ndarray  # (R, S) bool, emergency brake demand (recorder binary input)
    grade: np.ndarray  # (R, S)
    i_brake: np.ndarray  # (R,) first sample with the brake demand
    i_stop: np.ndarray  # (R,) first sample at standstill
    extra: dict = field(default_factory=dict)


def grade_at(b: Batch, s, idx=None):
    if idx is None:
        ga, gb, svc, lvc = b.grade_a, b.grade_b, b.s_vc, b.l_vc
    else:
        ga, gb, svc, lvc = b.grade_a[idx], b.grade_b[idx], b.s_vc[idx], b.l_vc[idx]
    x = np.clip((s - svc) / lvc, 0.0, 1.0)
    return ga + (gb - ga) * x


def body_pitch(a):
    """Body pitch on the secondary suspension, rad, and its rate: a 2nd-order response
    (PITCH_FN_HZ, PITCH_ZETA) to the static pitch k a (nose down when braking). (R, S) arrays."""
    wn = 2.0 * np.pi * data.PITCH_FN_HZ
    z = data.PITCH_ZETA
    R, S = a.shape
    th = np.zeros((R, S))
    om = np.zeros((R, S))
    x = data.PITCH_PER_DECEL_RAD * a[:, 0]
    w = np.zeros(R)
    sub = 10
    h = TS / sub
    for k in range(S):
        target = data.PITCH_PER_DECEL_RAD * a[:, k]
        for _ in range(sub):
            w = w + h * (wn * wn * (target - x) - 2.0 * z * wn * w)
            x = x + h * w
        th[:, k] = x
        om[:, k] = w
    return th, om


def grade_rate(b: Batch, s, v):
    """d atan(gamma(s)) / dt along the run (rad/s), exact for the linear vertical curve."""
    inside = (s > b.s_vc[:, None]) & (s < (b.s_vc + b.l_vc)[:, None])
    dgds = np.where(inside, ((b.grade_b - b.grade_a) / b.l_vc)[:, None], 0.0)
    return dgds * v


def specific_force(a, grade, pitch):
    """Accelerometer truth: a + g sin(gamma + pitch)."""
    return a + data.G0 * np.sin(np.arctan(grade) + pitch)


def resistance(v):
    c0, c1, c2 = data.DAVIS
    return np.where(v > 0.0, c0 + c1 * v + c2 * v * v, 0.0)


def _setup(b: Batch):
    q_axle = b.mass * data.G0 / data.N_AXLES
    k_eps = adhesion.eps_per_creep(float(np.mean(q_axle)) / 2.0, data.D_NEW / 2.0)
    return q_axle, k_eps, adhesion.condition_table(k_eps)


def _creep_table(table_row, k_eps, v_ref=10.0):
    """Quasi-static creep fraction as a function of the adhesion coefficient used (one condition)."""
    w = np.linspace(0.0, 1.0, 4001)
    f = adhesion.creep_coeff(w, v_ref, k_eps, *table_row)
    i = int(np.argmax(f))
    return f[: i + 1], w[: i + 1] / v_ref


def _edges(theta, t, cnt0=0, tl0=-1.0):
    """Edge counts and last-edge times at the sample instants from the wheel angle (rad)."""
    epr = data.EDGES_PER_REV
    n = np.floor(theta * epr / (2.0 * np.pi)).astype(np.int64)
    n = np.maximum(n, cnt0)
    te = np.full(theta.shape, tl0)
    moved = np.zeros(theta.shape, bool)
    moved[1:] = n[1:] > n[:-1]
    i = np.nonzero(moved)[0]
    ang = n[i] * 2.0 * np.pi / epr
    frac = (ang - theta[i - 1]) / np.maximum(theta[i] - theta[i - 1], 1e-12)
    te[i] = t[i - 1] + np.clip(frac, 0.0, 1.0) * (t[i] - t[i - 1])
    # carry the last edge time forward
    last = np.where(moved, np.arange(len(theta)), 0)
    last = np.maximum.accumulate(last)
    out = np.where(last > 0, te[last], tl0)
    return n, out


def simulate(b: Batch, t_stand: float = 2.0) -> Log:
    R, N = b.runs, data.N_AXLES
    q_axle, k_eps, table = _setup(b)
    r_all = b.d_true / 2.0
    sens = list(SENSOR_AXLES)
    creep_tabs = [_creep_table(table[c], k_eps) for c in range(4)]

    # ------------------------------------------------------------------ approach (per run)
    A = []
    for i in range(R):
        vc = float(b.v_cruise[i])
        t_acc = vc / A_TRACTION
        s_acc = 0.5 * vc * t_acc
        s_sw = float(b.s_brake[i]) - vc * T_PRE
        t_sw_raw = T_DWELL + t_acc + (s_sw - s_acc) / vc
        n = int(np.ceil(t_sw_raw / TS))
        t = np.arange(n + 1) * TS  # last point = start of the brake phase
        tm = np.maximum(t - T_DWELL, 0.0)  # standing at a stop for T_DWELL first
        v = np.minimum(vc, A_TRACTION * tm)
        a = np.where((t >= T_DWELL) & (tm < t_acc), A_TRACTION, 0.0)
        s = np.where(tm < t_acc, 0.5 * A_TRACTION * tm * tm, s_acc + vc * (tm - t_acc))
        g = grade_at(b, s, i)
        f_need = (b.mass[i] * a + resistance(v) + b.mass[i] * data.G0 * np.sin(np.arctan(g))) / (
            data.N_AXLES * q_axle[i]
        )
        theta = np.zeros((n + 1, N))
        creep = np.zeros((n + 1, N))
        for k in range(N):
            s_ax = np.maximum(s - AXLE_POS[k], 0.0)
            c = b.cond[i, np.clip((s_ax / GRID).astype(int), 0, b.cond.shape[1] - 1)]
            sl = np.zeros(n + 1)
            for ci in range(4):
                m = c == ci
                if np.any(m):
                    ft, st = creep_tabs[ci]
                    sl[m] = np.interp(np.abs(f_need[m]), ft, st) * np.sign(f_need[m])
            creep[:, k] = sl
            om = v * (1.0 + sl) / (b.d_true[i, k] / 2.0)
            theta[1:, k] = np.cumsum(0.5 * (om[1:] + om[:-1]) * TS)
        A.append(dict(t=t, v=v, a=a, s=s, g=g, theta=theta, creep=creep))

    # ------------------------------------------------------------------ brake phase (vectorised)
    v = np.array([x["v"][-1] for x in A])
    s = np.array([x["s"][-1] for x in A])
    t0 = np.array([x["t"][-1] for x in A])
    th = np.array([x["theta"][-1] for x in A])
    om = np.array([x["v"][-1] * (1.0 + x["creep"][-1]) for x in A]) / r_all
    u = np.ones((R, N))
    braking = np.zeros(R, bool)
    t_brake = np.full(R, np.inf)
    s_on = np.zeros(R)
    stopped = np.zeros(R, bool)
    t_stop = np.full(R, np.inf)
    a_prev = np.zeros(R)
    wr_prev = om * r_all
    qa = q_axle[:, None] * np.ones((1, N))
    m = b.mass
    tau = 0.0
    steps_per_sample = int(round(TS / DT_BRAKE))
    step = 0
    B = {k: [] for k in ("v", "a", "s", "g", "theta", "wr", "br")}
    while True:
        if step % steps_per_sample == 0:
            g = grade_at(b, s)
            B["v"].append(v.copy())
            B["a"].append(a_prev.copy())
            B["s"].append(s.copy())
            B["g"].append(g)
            B["theta"].append(th[:, sens].copy())
            B["wr"].append((om * r_all)[:, sens].copy())
            B["br"].append(braking.copy())
            if np.all(stopped & (tau - (t_stop - t0) >= t_stand - 1e-9)):
                break
        if tau > 200.0:
            bad = np.nonzero(~stopped)[0]
            raise RuntimeError(
                "brake phase did not finish: runs %s, v %s, braking %s" % (bad, v[bad], braking[bad])
            )
        dt = DT_BRAKE
        start = (~braking) & (s >= b.s_brake)
        t_brake = np.where(start, t0 + tau, t_brake)
        s_on = np.where(start, s, s_on)
        braking |= start

        g = grade_at(b, s)
        f_grade = m * data.G0 * np.sin(np.arctan(g))
        f_res = resistance(v)
        a_cmd = np.clip(0.8 * (b.v_cruise - v), -0.8, 1.1)
        f_trac = m * a_cmd + f_res + f_grade
        tb = np.clip(t0 + tau - t_brake, 0.0, None)
        f_wb = -m * np.minimum(data.A_WHEEL_BRAKE_DEMAND, data.JERK_BRAKE * tb)
        f_dem = np.where(braking, f_wb, f_trac)
        torque = u * (f_dem / N)[:, None] * r_all
        f_mtb = np.where(
            braking & (v > 0.0),
            data.MTB_N_MAGNETS * data.MTB_FORCE_N * np.minimum(1.0, tb / data.T_MTB_BUILD),
            0.0,
        )

        s_ax = np.maximum(s[:, None] - AXLE_POS[None, :], 0.0)
        idx = np.clip((s_ax / GRID).astype(int), 0, b.cond.shape[1] - 1)
        p = table[np.take_along_axis(b.cond, idx, axis=1)]
        ka, ks, mu0, ar, bc = p[..., 0], p[..., 1], p[..., 2], p[..., 3], p[..., 4]
        vv = v[:, None] * np.ones((1, N))
        # |d omega| per step is bounded by (|T| + r Q f_max) dt / J: a tight bracket for the solver
        dmax = (np.abs(torque) + r_all * qa * 0.6) * dt / data.J_AXLE + 1e-3
        lo = om - dmax
        hi = om + dmax
        w = om.copy()
        for _ in range(NEWTON_IT):
            cv = w * r_all - vv
            f, df = adhesion.creep_coeff_and_slope(np.abs(cv), vv, k_eps, ka, ks, mu0, ar, bc)
            h = data.J_AXLE * (w - om) / dt - torque + r_all * qa * f * np.sign(cv)
            lo = np.where(h < 0.0, w, lo)
            hi = np.where(h > 0.0, w, hi)
            wn = w - h / (data.J_AXLE / dt + r_all * r_all * qa * df)
            wn = np.where((wn > lo) & (wn < hi), wn, 0.5 * (lo + hi))
            conv = np.max(np.abs(wn - w)) < NEWTON_TOL
            w = wn
            if conv:
                break
        w = np.where((torque < 0.0) & (w < 0.0), 0.0, w)
        cv = w * r_all - vv
        fx = qa * adhesion.creep_coeff(np.abs(cv), vv, k_eps, ka, ks, mu0, ar, bc) * np.sign(cv)

        acc = np.where(stopped, 0.0, (fx.sum(axis=1) - f_mtb - f_res - f_grade) / m)
        v_new = v + acc * dt
        # below 1 cm/s the held wheels stick (static adhesion; the creep model has no force at w = 0)
        stop_now = braking & ~stopped & (v_new <= 0.01)
        v_new = np.where(stop_now | stopped, 0.0, np.maximum(v_new, 0.0))
        w = np.where((stop_now | stopped)[:, None], 0.0, w)
        t_stop = np.where(stop_now, t0 + tau + dt, t_stop)
        stopped |= stop_now
        s = s + 0.5 * (v + v_new) * dt

        slip = (w * r_all - v_new[:, None]) / np.maximum(v_new[:, None], 1.0)
        wr = w * r_all
        dwr = (wr - wr_prev) / dt - acc[:, None]
        release = (np.abs(slip) > data.WSP_RELEASE_SLIP) | (
            braking[:, None] & (dwr < -data.WSP_RELEASE_DECEL)
        )
        apply = np.abs(slip) < data.WSP_REAPPLY_SLIP
        u = np.clip(u + data.WSP_TORQUE_RATE * dt * (apply.astype(float) - release.astype(float)), 0.0, 1.0)
        wr_prev = wr
        th = th + 0.5 * (om + w) * dt
        om = w
        v = v_new
        a_prev = acc
        tau += dt
        step += 1

    # ------------------------------------------------------------------ stitch and pad
    nb = len(B["v"])
    Bv = np.array(B["v"]).T
    Ba = np.array(B["a"]).T
    Bs = np.array(B["s"]).T
    Bg = np.array(B["g"]).T
    Bth = np.transpose(np.array(B["theta"]), (1, 0, 2))
    Bwr = np.transpose(np.array(B["wr"]), (1, 0, 2))
    Bbr = np.array(B["br"]).T
    lengths = [len(x["t"]) - 1 + nb for x in A]
    S = max(lengths)
    tt = np.arange(S) * TS
    out = {k: np.zeros((R, S)) for k in ("s", "v", "a", "g")}
    th_all = np.zeros((R, S, 2))
    wr_all = np.zeros((R, S, 2))
    br_all = np.zeros((R, S), bool)
    for i in range(R):
        x = A[i]
        na = len(x["t"]) - 1
        for key, src in (("s", Bs), ("v", Bv), ("a", Ba), ("g", Bg)):
            seq = np.concatenate([x[key][:na], src[i]])
            out[key][i, : len(seq)] = seq
            out[key][i, len(seq) :] = seq[-1]
        seq = np.concatenate([x["theta"][:na][:, sens], Bth[i]])
        th_all[i, : len(seq)] = seq
        th_all[i, len(seq) :] = seq[-1]
        seq = np.concatenate([(x["v"][:na, None] * (1.0 + x["creep"][:na][:, sens])), Bwr[i]])
        wr_all[i, : len(seq)] = seq
        seq = np.concatenate([np.zeros(na, bool), Bbr[i]])
        br_all[i, : len(seq)] = seq
        br_all[i, len(seq) :] = True
    cnt = np.zeros((R, S, 2), dtype=np.int64)
    te = np.zeros((R, S, 2))
    for i in range(R):
        for j in range(2):
            cnt[i, :, j], te[i, :, j] = _edges(th_all[i, :, j], tt)
    # the accelerometer's digital filter averages over the sample interval: use the mean
    # acceleration of the interval that ends at each sample (consistent with the integration)
    a = np.diff(out["v"], axis=1, prepend=out["v"][:, :1]) / TS
    pitch, pitch_rate = body_pitch(a)
    g_rate = grade_rate(b, out["s"], out["v"]) / (1.0 + out["g"] ** 2)
    i_brake = np.argmax(br_all, axis=1)
    i_stop = np.array([int(np.argmax((out["v"][i] <= 0.0) & (np.arange(S) > i_brake[i]))) for i in range(R)])
    return Log(
        t=tt,
        s=out["s"],
        v=out["v"],
        a=a,
        f_spec=specific_force(a, out["g"], pitch),
        count=cnt,
        t_edge=te,
        w_axle=wr_all,
        brake=br_all,
        grade=out["g"],
        i_brake=i_brake,
        i_stop=i_stop,
        extra={"pitch": pitch, "pitch_rate": pitch_rate + g_rate, "t_on": t_brake, "s_on": s_on},
    )


COND_MODES = {"dry": 0, "wet": 1, "dry_leaf": 2, "wet_leaf": 3}


def make_batch(rng: np.random.Generator, runs: int, cond_mode: str = "mixed", **over) -> Batch:
    """Random batch over the study's operating envelope (DOSSIER §9.1)."""
    v_c = rng.uniform(30.0, 70.0, runs) / 3.6
    s_b = rng.uniform(1550.0, 1650.0, runs)
    if "s_brake" in over:
        s_b = np.asarray(over["s_brake"], dtype=float)
    n_cells = int(1800.0 / GRID)
    cond = np.zeros((runs, n_cells), dtype=np.int64)
    if cond_mode == "mixed":
        kinds = rng.choice(4, size=runs, p=[0.4, 0.3, 0.15, 0.15])
    else:
        kinds = np.full(runs, COND_MODES[cond_mode])
    for i in range(runs):
        k = int(kinds[i])
        if k <= adhesion.WET:
            cond[i, :] = k
            continue
        # leaf patches on a wet rail, 5-30 m long, about half of the braking zone (A)
        cond[i, :] = adhesion.WET
        x = s_b[i] - 20.0
        while x < s_b[i] + 150.0:
            ln = rng.uniform(5.0, 30.0)
            if rng.random() < 0.5:
                a0 = int(x / GRID)
                cond[i, a0 : a0 + int(ln / GRID)] = k
            x += ln
    gmax = np.where(kinds >= adhesion.DRY_LEAF, data.GRADE_MAX_LEAF, data.GRADE_MAX)
    ga = rng.uniform(-1.0, 1.0, runs) * gmax
    gb = np.clip(ga + rng.uniform(-0.04, 0.04, runs), -gmax, gmax)
    svc = s_b + rng.uniform(-60.0, 40.0, runs)
    lvc = rng.uniform(40.0, 120.0, runs)
    mass = rng.uniform(data.M_EMPTY, data.M_LOADED, runs)
    # true wheel diameters between the limit and new; the two axles of a bogie are turned
    # together (+-0.2 mm), the bogies differ (A).
    bog = rng.uniform(data.D_MIN + 0.005, data.D_NEW, (runs, 3))
    d = np.repeat(bog, 2, axis=1) + rng.normal(0.0, 0.0002, (runs, data.N_AXLES))
    b = Batch(v_c, s_b, cond, ga, gb, svc, lvc, mass, d)
    for k, val in over.items():
        setattr(b, k, np.asarray(val, dtype=float) if k != "cond" else val)
    return b
