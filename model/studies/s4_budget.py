"""S4: error budget of the chosen design (E3) and what the 5 cm brake test and the 1500 m window
need. One error source at a time on the same runs; a sweep of the wheel-diameter calibration;
the depot brake test (dry, level, empty)."""

from __future__ import annotations

import numpy as np
from common import QUICK, batch, pct, plt, save, write_json
from odo import data, sensors, vehicle
from odo import estimator as E

RUNS = 8 if QUICK else 24
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
SOURCES = [
    ("none (model and algorithm)", {}),
    ("diameter calibration 0.05 %", {"cal_err_max": 0.0005}),
    ("accelerometer offset 75 mg", {"acc_bias_max": data.ACC_OFFSET_MAX}),
    ("accelerometer scale 0.7 %", {"acc_scale_max": data.ACC_NONLIN + data.ACC_SENS_TC * 60.0}),
    ("accelerometer noise", {"acc_noise": data.ACC_NOISE_DENSITY * np.sqrt(data.ACC_BW_HZ)}),
    ("car-body vibration 0.1 m/s^2", {"vib_rms": data.VIB_RMS}),
    ("gyro offset + drift", {"gyro_offset": data.GYR_OFFSET, "gyro_tco": data.GYR_TCO, "temp_drift_c": 2.0}),
    ("gyro noise", {"gyro_noise": data.GYR_NOISE_DENSITY * np.sqrt(data.ACC_BW_HZ)}),
    ("gyro g-sensitivity (10 % left)", {"gyro_g_sens": data.GYR_G_SENS * data.GYR_G_SENS_RESIDUAL}),
    ("gyro g-sensitivity (uncompensated)", {"gyro_g_sens": data.GYR_G_SENS}),
    ("all (data-sheet values)", None),
]


def brake_errors(b, log, kw, seed=5, est=None):
    sen = sensors.make(log, b.d_true[:, list(vehicle.SENSOR_AXLES)], np.random.default_rng(seed), **kw)
    r = np.arange(b.runs)
    d_true = log.s[r, log.i_stop] - log.extra["s_on"]
    res = (est or E.fused)(sen)
    return res.d_brake - d_true


def run():
    out = {"runs": RUNS}
    b, log = batch("s4", 41, RUNS, "mixed", s_brake=np.full(RUNS, 300.0))
    rows = []
    for name, kw in SOURCES:
        args = dict(IDEAL)
        if kw is None:
            args = {}
        else:
            args.update(kw)
        e = brake_errors(b, log, args)
        rows.append({"source": name, "p50_m": pct(e, 50), "p95_m": pct(e, 95), "max_m": pct(e, 100)})
        print("%-36s p95 %.3f m" % (name, rows[-1]["p95_m"]))
    out["one_at_a_time"] = rows

    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    names = [r["source"] for r in rows]
    y = np.arange(len(rows))
    ax.barh(
        y, [r["p95_m"] * 100 for r in rows], color=["#7a7a7a"] + ["#2a6f97"] * (len(rows) - 2) + ["#3d3d3d"]
    )
    ax.barh(y, [r["p50_m"] * 100 for r in rows], color="#d08a3a", height=0.35, label="median")
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=7.5)
    ax.invert_yaxis()
    ax.axvline(5.0, color="k", ls="--", lw=0.8)
    ax.set_xlabel("stopping-distance error, 95th percentile (cm); orange: median")
    ax.set_title("E3 error budget: one source at a time (%d runs, all rail conditions)" % RUNS)
    ax.legend(fontsize=7.5, loc="lower right")
    save(fig, "s4_budget.png")

    # ---- calibration sweep (everything else at data-sheet values)
    cals = [0.0, 0.0001, 0.0005, 0.001, 0.002, 0.005]
    sweep = []
    for c in cals:
        e = brake_errors(b, log, {"cal_err_max": c})
        sweep.append({"cal_rel": c, "brake_p95_m": pct(e, 95), "window_1500_m": 1500.0 * c})
    out["calibration_sweep"] = sweep

    # ---- depot brake test: dry, level, empty, 30-50 km/h
    nd = RUNS
    bd, ld = batch(
        "s4depot",
        42,
        nd,
        "dry",
        s_brake=np.full(nd, 300.0),
        grade_a=np.zeros(nd),
        grade_b=np.zeros(nd),
        mass=np.full(nd, data.M_EMPTY),
        v_cruise=np.linspace(30.0, 50.0, nd) / 3.6,
    )
    dep = {}
    for c in (0.0001, 0.0005, 0.001):
        e3 = brake_errors(bd, ld, {"cal_err_max": c})
        e1 = brake_errors(bd, ld, {"cal_err_max": c}, est=E.e1_single)
        dep["%.2f%%" % (c * 100)] = {
            "E3_p95_m": pct(e3, 95),
            "E3_within_5cm": float(np.mean(np.abs(e3) <= 0.05)),
            "E1_p95_m": pct(e1, 95),
            "E1_within_5cm": float(np.mean(np.abs(e1) <= 0.05)),
        }
    out["depot_brake_test"] = dep
    stop = ld.s[np.arange(nd), ld.i_stop] - ld.extra["s_on"]
    out["depot_stop_m"] = [float(stop.min()), float(stop.max())]

    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    xs = [s["cal_rel"] * 100 for s in sweep]
    ax.plot(
        xs,
        [s["brake_p95_m"] * 100 for s in sweep],
        "o-",
        color="#2a6f97",
        label="E3 stopping distance, p95 (all conditions)",
    )
    ax.plot(
        xs,
        [s["window_1500_m"] * 100 for s in sweep],
        "s--",
        color="#9a3b3b",
        label="error at the far end of the 1500 m window",
    )
    ax.axhline(5.0, color="k", ls=":", lw=0.8)
    ax.axhline(25.0, color="#9a3b3b", ls=":", lw=0.8)
    ax.text(0.3, 26, "0.25 m", fontsize=7.5, color="#9a3b3b")
    ax.text(0.3, 5.5, "5 cm", fontsize=7.5)
    ax.set_xlabel("wheel-diameter calibration error bound (%)")
    ax.set_ylabel("error (cm)")
    ax.set_yscale("log")
    ax.set_title("What the diameter calibration buys")
    ax.legend(fontsize=7.5)
    save(fig, "s4_calibration.png")
    write_json("s4_budget.json", out)
    return out


if __name__ == "__main__":
    run()
