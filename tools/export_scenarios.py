#!/usr/bin/env python3
"""Export recorded scenarios for the firmware tests (host replay and Renode rig).

Each scenario is one run of the vehicle model (model/odo/vehicle.py) with the sensor models
(model/odo/sensors.py), quantised the way the sensor frame carries it, plus the truth and the
result of the Python reference estimator (E3) on the same quantised data:

    fw/sim/scenarios/<name>.csv
        # name, description, d_true_m, v_on_ms, e3_d_m, e1_d_m, ...
        t_us,cnt0,te0_us,cnt1,te1_us,acc_e5,gyro_e6,tbrk_us,inputs,s_mm,v_mms

    python tools/export_scenarios.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "model"))

from odo import adhesion, estimator, sensors, vehicle  # noqa: E402

OUT = os.path.join(ROOT, "fw", "sim", "scenarios")

# name, description, condition, v0 km/h, grade before, grade after, mass, seed
SCENARIOS = [
    (
        "sc1_depot_dry",
        "depot brake test: dry, level, empty, 40 km/h",
        adhesion.DRY,
        40.0,
        0.0,
        0.0,
        38000.0,
        11,
    ),
    (
        "sc2_wet_down",
        "wet rail, 6 % downhill, loaded, 50 km/h",
        adhesion.WET,
        50.0,
        -0.06,
        -0.06,
        52000.0,
        12,
    ),
    (
        "sc3_leaf_slide",
        "wet leaves, 4 % downhill, 50 km/h: long slide",
        adhesion.WET_LEAF,
        50.0,
        -0.04,
        -0.04,
        45000.0,
        13,
    ),
    (
        "sc4_vcurve",
        "dry leaves into a sag curve (-5 % to 0 %), 60 km/h",
        adhesion.DRY_LEAF,
        60.0,
        -0.05,
        0.0,
        45000.0,
        14,
    ),
]
S_BRAKE = 300.0


def make_batch():
    n = len(SCENARIOS)
    rng = np.random.default_rng(7)
    b = vehicle.make_batch(rng, n, "dry")
    b.v_cruise = np.array([s[3] for s in SCENARIOS]) / 3.6
    b.s_brake = np.full(n, S_BRAKE)
    b.grade_a = np.array([s[4] for s in SCENARIOS])
    b.grade_b = np.array([s[5] for s in SCENARIOS])
    b.s_vc = np.full(n, S_BRAKE - 10.0)
    b.l_vc = np.full(n, 60.0)
    b.mass = np.array([s[6] for s in SCENARIOS])
    cond = np.zeros_like(b.cond)
    for i, s in enumerate(SCENARIOS):
        k = s[2]
        if k <= adhesion.WET:
            cond[i, :] = k
        else:
            cond[i, :] = adhesion.WET
            r = np.random.default_rng(s[7])
            x = S_BRAKE - 20.0
            while x < S_BRAKE + 150.0:
                ln = r.uniform(5.0, 30.0)
                if r.random() < 0.6:
                    cond[i, int(x) : int(x + ln)] = k
                x += ln
    b.cond = cond
    return b


def quantise(sen: sensors.Sensors) -> sensors.Sensors:
    te = np.where(sen.t_edge >= 0.0, np.round(sen.t_edge * 1e6) / 1e6, -1.0)
    acc = np.round(sen.acc * 1e5) / 1e5
    gyro = np.round(sen.gyro * 1e6) / 1e6
    t_on = np.round(sen.t_on * 1e6) / 1e6
    return sensors.Sensors(
        sen.count, te, sen.d_cfg, acc, gyro, sen.radar, sen.brake, t_on, sen.acc_bias, sen.cal_err
    )


def main():
    os.makedirs(OUT, exist_ok=True)
    b = make_batch()
    log = vehicle.simulate(b)
    sen = quantise(sensors.make(log, b.d_true[:, list(vehicle.SENSOR_AXLES)], np.random.default_rng(21)))
    e3 = estimator.fused(sen)
    e1 = estimator.e1_single(sen)
    r = np.arange(b.runs)
    d_true = log.s[r, log.i_stop] - log.extra["s_on"]
    for i, sc in enumerate(SCENARIOS):
        name = sc[0]
        S = int(log.i_stop[i]) + 300  # 3 s at standstill after the stop
        path = os.path.join(OUT, name + ".csv")
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write("# name %s\n# description %s\n" % (name, sc[1]))
            f.write("# d_true_m %.4f\n# v_on_ms %.4f\n" % (d_true[i], log.v[i, log.i_brake[i] - 1]))
            f.write("# e3_d_m %.4f\n# e1_d_m %.4f\n" % (e3.d_brake[i], e1.d_brake[i]))
            f.write("# d_cfg_m %.6f %.6f\n" % (sen.d_cfg[i, 0], sen.d_cfg[i, 1]))
            f.write("# t_on_us %d\n" % int(round(sen.t_on[i] * 1e6)))
            f.write("t_us,cnt0,te0_us,cnt1,te1_us,acc_e5,gyro_e6,tbrk_us,inputs,s_mm,v_mms\n")
            tb = int(round(sen.t_on[i] * 1e6))
            for k in range(S):
                te0 = max(int(round(sen.t_edge[i, k, 0] * 1e6)), 0)
                te1 = max(int(round(sen.t_edge[i, k, 1] * 1e6)), 0)
                br = bool(sen.brake[i, k])
                f.write(
                    "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
                    % (
                        int(round(log.t[k] * 1e6)),
                        sen.count[i, k, 0],
                        te0,
                        sen.count[i, k, 1],
                        te1,
                        int(round(sen.acc[i, k] * 1e5)),
                        int(round(sen.gyro[i, k] * 1e6)),
                        tb if br else 0,
                        1 if br else 0,
                        int(round(log.s[i, k] * 1000.0)),
                        int(round(log.v[i, k] * 1000.0)),
                    )
                )
        print(
            "wrote %s: %d rows, d_true %.3f m, E3 %.3f m (err %+.3f), E1 err %+.3f"
            % (
                os.path.relpath(path, ROOT),
                S,
                d_true[i],
                e3.d_brake[i],
                e3.d_brake[i] - d_true[i],
                e1.d_brake[i] - d_true[i],
            )
        )


if __name__ == "__main__":
    main()
