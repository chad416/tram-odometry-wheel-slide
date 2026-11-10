"""S1: wheel-rail adhesion and emergency braking. Does the model brake like a tram (BOStrab
Anlage 2 Table 2), and how much do the wheels creep and slide on each rail condition?"""

from __future__ import annotations

import numpy as np
from common import batch, plt, save, write_json
from odo import adhesion, data

SPEEDS = [20, 30, 40, 50, 60, 70]
CONDS = ["dry", "wet", "dry_leaf", "wet_leaf"]


def creep_curves():
    q = data.M_EMPTY * data.G0 / (2 * data.N_AXLES)
    k = adhesion.eps_per_creep(q, data.D_NEW / 2.0)
    tab = adhesion.condition_table(k)
    w = np.linspace(0.0, 2.0, 2001)
    out = {}
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    cols = ["#2a6f97", "#5b8c5a", "#d08a3a", "#9a3b3b"]
    for i, name in enumerate(adhesion.NAMES):
        for v, ls in ((5.0, "-"), (20.0, "--")):
            f = adhesion.creep_coeff(w, v, k, *tab[i])
            ax.plot(
                w,
                f,
                ls,
                color=cols[i],
                lw=1.4 if ls == "-" else 0.9,
                label="%s, %d km/h" % (name, v * 3.6) if ls == "-" else None,
            )
        fpk, wpk = adhesion.peak(dict(zip(["kA", "kS", "mu0", "A", "B"], tab[i])), k, 10.0)
        # creep needed for the wheel-brake share of an emergency brake (f = 0.2)
        f10 = adhesion.creep_coeff(w, 10.0, k, *tab[i])
        reach = f10 >= 0.2
        s_02 = float(w[np.argmax(reach)] / 10.0) if np.any(reach) else None
        out[name] = {"peak_f_at_36kmh": fpk, "creep_vel_at_peak_ms": wpk, "creep_at_f0.2": s_02}
    ax.axhline(0.2, color="k", lw=0.7, ls=":")
    ax.text(1.55, 0.21, "wheel-brake demand", fontsize=7.5)
    ax.set_xlabel("creep (slip) velocity |omega r - v| (m/s)")
    ax.set_ylabel("adhesion coefficient f = Fx / Q")
    ax.set_title("Polach creep-force curves (Table 2 parameters; leaf peaks from field CoT)")
    ax.legend(fontsize=7.5)
    save(fig, "s1_creep.png")
    return out


def bostrab_check():
    """Empty tram on straight, level track (the BOStrab condition), each speed and condition."""
    res = {}
    n = len(SPEEDS)
    for cond in CONDS:
        b, log = batch(
            "s1",
            1,
            n,
            cond,
            v_cruise=np.array(SPEEDS) / 3.6,
            s_brake=np.full(n, 300.0),
            grade_a=np.zeros(n),
            grade_b=np.zeros(n),
            mass=np.full(n, data.M_EMPTY),
        )
        r = np.arange(n)
        d = log.s[r, log.i_stop] - log.extra["s_on"]
        v0 = log.v[r, log.i_brake - 1]
        res[cond] = {"v0_kmh": SPEEDS, "stop_m": d.tolist(), "mean_decel": (v0**2 / (2.0 * d)).tolist()}
    fig, ax = plt.subplots(figsize=(6.8, 3.6))
    lim = [data.BOSTRAB_EMERGENCY[v][0] for v in SPEEDS]
    ax.plot(SPEEDS, lim, "k-", lw=2, label="BOStrab Anlage 2, Table 2: minimum")
    cols = {"dry": "#2a6f97", "wet": "#5b8c5a", "dry_leaf": "#d08a3a", "wet_leaf": "#9a3b3b"}
    for cond in CONDS:
        ax.plot(
            SPEEDS, res[cond]["mean_decel"], "o-", color=cols[cond], label="model, " + cond.replace("_", " ")
        )
    ax.set_xlabel("speed at the brake onset (km/h)")
    ax.set_ylabel("mean deceleration (m/s^2)")
    ax.set_title("Emergency brake, empty tram, level track: model against the BOStrab minimum")
    ax.legend(fontsize=7.5)
    save(fig, "s1_bostrab.png")
    return res


def run():
    out = {"creep": creep_curves(), "bostrab": bostrab_check()}
    dry = out["bostrab"]["dry"]["mean_decel"]
    out["dry_meets_bostrab"] = bool(all(a >= data.BOSTRAB_EMERGENCY[v][0] for a, v in zip(dry, SPEEDS)))
    write_json("s1_adhesion.json", out)
    return out


if __name__ == "__main__":
    run()
