"""S3: Monte Carlo over the operating envelope. Which sensor set and estimator measures the
emergency-brake distance and the last 1500 m, under slide, gradients and wear?

Per rail condition (dry, wet, dry leaf, wet leaf) a batch of runs (DOSSIER §9.1 ranges) is
simulated once; every estimator E1..E5 then sees the same runs with the same sensor draws.
"""

from __future__ import annotations

import numpy as np
from common import COLORS, QUICK, batch, pct, plt, save, write_json
from odo import estimator as E
from odo import sensors, vehicle

CONDS = ["dry", "wet", "dry_leaf", "wet_leaf"]
RUNS = 8 if QUICK else 48
ESTS = [
    ("E1", E.e1_single),
    ("E2", E.e2_two_max),
    ("E4", E.e4_radar),
    ("E3", lambda s: E.fused(s)),
    ("E3h", lambda s: E.fused(s, "hybrid")),
    ("E3a", lambda s: E.fused(s, use_gyro=False)),
    ("E3g", lambda s: E.fused(s, "gate")),
    ("E5", lambda s: E.fused(s, "inertial", use_radar=True)),
]


def profile_error(log, res):
    """Max |error| of the distance-to-stop over the last 1500 m, per run."""
    R = log.s.shape[0]
    out = np.zeros(R)
    for i in range(R):
        s_end = log.s[i, -1]
        sh_end = res.s_hat[i, -1]
        m = (s_end - log.s[i]) <= 1500.0
        err = (sh_end - res.s_hat[i, m]) - (s_end - log.s[i, m])
        out[i] = np.max(np.abs(err))
    return out


def run():
    rows = {k: {"brake": [], "rel": [], "profile": [], "v0": [], "cond": []} for k, _ in ESTS}
    trace = None
    for ci, cond in enumerate(CONDS):
        b, log = batch("s3", 100 + ci, RUNS, cond)
        sen = sensors.make(log, b.d_true[:, list(vehicle.SENSOR_AXLES)], np.random.default_rng(200 + ci))
        r = np.arange(b.runs)
        d_true = log.s[r, log.i_stop] - log.extra["s_on"]
        v_true = log.v[r, log.i_brake - 1]
        for key, fn in ESTS:
            res = fn(sen)
            err = res.d_brake - d_true
            rows[key]["brake"] += list(err)
            rows[key]["rel"] += list(err / d_true)
            rows[key]["profile"] += list(profile_error(log, res))
            rows[key]["v0"] += list(res.v0 - v_true)
            rows[key]["cond"] += [cond] * b.runs
            if cond == "wet_leaf" and key == "E3" and trace is None:
                trace = (b, log, sen, res)
        print("condition", cond, "done")
    summary = {"runs_per_condition": RUNS, "estimators": {}}
    for key, _ in ESTS:
        x = rows[key]
        e = np.array(x["brake"])
        c = np.array(x["cond"])
        per = {}
        for cond in CONDS:
            m = c == cond
            per[cond] = {"p50_m": pct(e[m], 50), "p95_m": pct(e[m], 95), "max_m": pct(e[m], 100)}
        summary["estimators"][key] = {
            "brake_p50_m": pct(e, 50),
            "brake_p95_m": pct(e, 95),
            "brake_max_m": pct(e, 100),
            "brake_within_5cm": float(np.mean(np.abs(e) <= 0.05)),
            "brake_rel_p95": pct(np.array(x["rel"]), 95),
            "profile_p95_m": pct(np.array(x["profile"]), 95),
            "profile_max_m": pct(np.array(x["profile"]), 100),
            "v0_p95_ms": pct(np.array(x["v0"]), 95),
            "by_condition": per,
        }
    write_json("s3_montecarlo.json", summary)
    figures(rows, summary)
    if trace is not None:
        figure_trace(*trace)
    return summary


def figures(rows, summary):
    fig, ax = plt.subplots(figsize=(7.2, 4.0))
    for key, _ in ESTS:
        e = np.sort(np.abs(np.array(rows[key]["brake"])))
        y = np.arange(1, len(e) + 1) / len(e)
        ax.step(
            np.maximum(e, 1e-3),
            y,
            where="post",
            color=COLORS[key],
            lw=1.6 if key == "E3" else 1.1,
            label="%s  (p95 %.2f m)" % (key, summary["estimators"][key]["brake_p95_m"]),
        )
    ax.axvline(0.05, color="k", ls="--", lw=0.8)
    ax.text(0.052, 0.04, "5 cm", fontsize=8)
    ax.set_xscale("log")
    ax.set_xlabel("|stopping-distance error| (m)")
    ax.set_ylabel("fraction of runs")
    ax.set_title(
        "Emergency-brake distance error, %d runs (dry, wet, leaves; gradients; wear)"
        % len(rows["E3"]["brake"])
    )
    ax.legend(fontsize=7.5, loc="lower right")
    save(fig, "s3_brake_error_cdf.png")

    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    keys = [k for k, _ in ESTS]
    w = 0.8 / len(keys)
    for j, key in enumerate(keys):
        vals = [summary["estimators"][key]["by_condition"][c]["p95_m"] for c in CONDS]
        ax.bar(np.arange(len(CONDS)) + (j - len(keys) / 2) * w + w / 2, vals, w, color=COLORS[key], label=key)
    ax.set_xticks(np.arange(len(CONDS)))
    ax.set_xticklabels(["dry", "wet", "dry leaf", "wet leaf"])
    ax.set_yscale("log")
    ax.axhline(0.05, color="k", ls="--", lw=0.8)
    ax.set_ylabel("p95 |error| (m)")
    ax.set_title("Stopping-distance error by rail condition (95th percentile)")
    ax.legend(ncol=7, fontsize=7, loc="upper left")
    save(fig, "s3_by_condition.png")


def figure_trace(b, log, sen, res):
    i = 0
    k0, k1 = log.i_brake[i] - 150, log.i_stop[i] + 80
    t = log.t[k0:k1] - log.t[log.i_brake[i]]
    fig, axs = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True)
    ax = axs[0]
    ax.plot(t, log.v[i, k0:k1] * 3.6, "k", lw=1.6, label="vehicle (truth)")
    ax.plot(t, log.w_axle[i, k0:k1, 0] * 3.6, color="#d08a3a", lw=0.9, label="wheel, bogie 1")
    ax.plot(t, log.w_axle[i, k0:k1, 1] * 3.6, color="#9a3b3b", lw=0.9, label="wheel, bogie 3")
    ax.plot(t, res.v_hat[i, k0:k1] * 3.6, "--", color="#2a6f97", lw=1.2, label="E3 estimate (live)")
    ax.set_ylabel("speed (km/h)")
    ax.set_title("Emergency brake on wet leaves: wheels slide under WSP, the estimate follows the vehicle")
    ax.legend(fontsize=7.5)
    ax = axs[1]
    ib = log.i_brake[i]
    d_true = log.s[i, k0:k1] - log.s[i, ib]
    w_dist = (log.count[i, k0:k1, 0] - log.count[i, ib, 0]) * np.pi * sen.d_cfg[i, 0] / 400.0
    sh = res.s_hat[i, k0:k1] - res.s_hat[i, ib]
    ax.axhline(0.0, color="k", lw=0.8)
    ax.plot(t, w_dist - d_true, color="#d08a3a", label="one wheel sensor (E1)")
    ax.plot(t, sh - d_true, color="#2a6f97", label="E3 (corrected profile)")
    ax.set_ylabel("distance error since onset (m)")
    ax.set_xlabel("time from the brake onset (s)")
    ax.legend(fontsize=7.5)
    save(fig, "s2_slide_trace.png")


if __name__ == "__main__":
    run()
