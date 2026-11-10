"""S6: analysis of alternatives for the sensor set (Wasson ch. 32). The accuracy criteria are
scored from the S3 Monte Carlo; cost, installation, availability and assurance are scored by
assumption (DOSSIER §8.1, each score with its reason). Robustness: 20 000 random weightings
and +-10 % on each single weight."""

from __future__ import annotations

import json
import math
import os

import numpy as np
from common import RES, plt, save, write_json

ALTS = [
    ("A1", "one wheel sensor", "E1"),
    ("A2", "two wheel sensors", "E2"),
    ("A3", "two wheel sensors + IMU", "E3"),
    ("A4", "wheel sensor + Doppler radar", "E4"),
    ("A5", "two wheel sensors + IMU + radar", "E5"),
]
# criterion: weight
CRIT = [
    ("brake distance under slide (p95)", 0.30),
    ("last-1500 m distance (p95)", 0.15),
    ("unit cost", 0.15),
    ("installation and upkeep", 0.15),
    ("availability (degraded modes)", 0.15),
    ("software assurance effort", 0.10),
]
# assumed scores (A), 1..10, with the reason in DOSSIER §8.1
SCORE_A = {
    "A1": [None, None, 10, 10, 3, 9],
    "A2": [None, None, 9, 9, 6, 8],
    "A3": [None, None, 8, 9, 8, 6],
    "A4": [None, None, 4, 4, 6, 7],
    "A5": [None, None, 3, 4, 9, 4],
}


def acc_score(p95_m: float, target_m: float) -> float:
    """10 at or below the target, -3 per decade above it, at least 1."""
    return float(np.clip(10.0 - 3.0 * math.log10(max(p95_m, 1e-6) / target_m), 1.0, 10.0))


def run():
    mc = json.load(open(os.path.join(RES, "s3_montecarlo.json"), encoding="utf-8"))["estimators"]
    S = np.zeros((len(ALTS), len(CRIT)))
    for i, (key, _, est) in enumerate(ALTS):
        row = list(SCORE_A[key])
        row[0] = acc_score(mc[est]["brake_p95_m"], 0.05)
        row[1] = acc_score(mc[est]["profile_p95_m"], 0.25)
        S[i] = row
    w = np.array([c[1] for c in CRIT])
    total = S @ w
    rng = np.random.default_rng(32)
    W = rng.dirichlet(np.ones(len(CRIT)), 20000)
    wins = np.bincount(np.argmax(W @ S.T, axis=1), minlength=len(ALTS)) / len(W)
    flips = []
    best = int(np.argmax(total))
    for j in range(len(CRIT)):
        for f in (0.9, 1.1):
            w2 = w.copy()
            w2[j] *= f
            w2 /= w2.sum()
            b2 = int(np.argmax(S @ w2))
            if b2 != best:
                flips.append({"criterion": CRIT[j][0], "factor": f, "winner": ALTS[b2][0]})
    out = {
        "criteria": [{"name": c[0], "weight": c[1]} for c in CRIT],
        "alternatives": [
            {
                "id": a[0],
                "name": a[1],
                "scores": S[i].round(2).tolist(),
                "total": round(float(total[i]), 3),
                "win_fraction": round(float(wins[i]), 4),
            }
            for i, a in enumerate(ALTS)
        ],
        "winner": ALTS[best][0],
        "single_weight_flips": flips,
    }
    write_json("s6_aoa.json", out)

    fig, axs = plt.subplots(1, 2, figsize=(8.4, 3.4), gridspec_kw={"width_ratios": [1.4, 1]})
    ax = axs[0]
    left = np.zeros(len(ALTS))
    cols = ["#2a6f97", "#56a0a0", "#d08a3a", "#9a3b3b", "#5b8c5a", "#8c6bb1"]
    for j, (cname, wt) in enumerate(CRIT):
        ax.barh(np.arange(len(ALTS)), S[:, j] * wt, left=left, color=cols[j], label=cname)
        left += S[:, j] * wt
    ax.set_yticks(np.arange(len(ALTS)))
    ax.set_yticklabels(["%s %s" % (a[0], a[1]) for a in ALTS], fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel("weighted score (0-10)")
    ax.legend(fontsize=6.5, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=2)
    ax.set_title("Weighted scores")
    ax = axs[1]
    ax.bar([a[0] for a in ALTS], wins * 100, color="#2a6f97")
    ax.set_ylabel("% of 20 000 random weightings won")
    ax.set_title("Robustness")
    save(fig, "s6_aoa.png")
    print("winner", out["winner"], "totals", total.round(2), "wins", wins.round(3), "flips", flips)
    return out


if __name__ == "__main__":
    run()
