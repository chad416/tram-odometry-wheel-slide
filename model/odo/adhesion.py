"""Wheel-rail creep force after Polach (2005) [R2], with Hertz contact for the slope.

    f = F/Q = 2 mu / pi * ( kA eps / (1 + (kA eps)^2) + atan(kS eps) )      Polach eq. (11)
    mu(w) = mu0 * ((1 - A) exp(-B w) + A)                                    Polach eq. (9)
    eps = 1/4 * G pi a b c11 / (Q mu) * s,   s = w / V                       Polach eq. (4)

w is the creep (slip) velocity |omega r - V| and s the creep. The functions work on numpy
arrays of any shape, so the vehicle model can evaluate all axles of all Monte-Carlo runs at once.
"""

from __future__ import annotations

import math

import numpy as np

from . import data

# Condition index used by the vehicle model and the studies.
DRY, WET, DRY_LEAF, WET_LEAF = 0, 1, 2, 3
NAMES = ("dry", "wet", "dry leaf", "wet leaf")


def hertz_radius(q_wheel: float, r_wheel: float) -> float:
    """Radius a = b of the (assumed circular) Hertz contact, m (D, textbook Hertz)."""
    r_eq = math.sqrt(r_wheel * data.R_RAIL_CROWN)
    e_star = data.E_STEEL / (2.0 * (1.0 - data.NU_STEEL**2))
    return (3.0 * q_wheel * r_eq / (4.0 * e_star)) ** (1.0 / 3.0)


def eps_per_creep(q_wheel: float, r_wheel: float) -> float:
    """eps / (s / mu): 1/4 G pi a b c11 / Q, so that eps = this * s / mu."""
    a = hertz_radius(q_wheel, r_wheel)
    return 0.25 * data.G_STEEL * math.pi * a * a * data.C11 / q_wheel


def mu_of_w(w, mu0, a_ratio, b_coef):
    return mu0 * ((1.0 - a_ratio) * np.exp(-b_coef * w) + a_ratio)


def creep_coeff(w, v, k_eps, ka, ks, mu0, a_ratio, b_coef, v_min=0.5):
    """Adhesion coefficient f = F/Q for creep velocity w >= 0 at vehicle speed v (arrays)."""
    mu = mu_of_w(w, mu0, a_ratio, b_coef)
    s = w / np.maximum(v, v_min)
    eps = k_eps * s / mu
    return 2.0 * mu / np.pi * (ka * eps / (1.0 + (ka * eps) ** 2) + np.arctan(ks * eps))


def creep_coeff_and_slope(w, v, k_eps, ka, ks, mu0, a_ratio, b_coef, v_min=0.5):
    """f and df/dw (analytic), for the implicit wheel solver."""
    ex = np.exp(-b_coef * w)
    mu = mu0 * ((1.0 - a_ratio) * ex + a_ratio)
    dmu = -mu0 * (1.0 - a_ratio) * b_coef * ex
    vv = np.maximum(v, v_min)
    eps = k_eps * w / (vv * mu)
    deps = k_eps / (vv * mu) * (1.0 - w * dmu / mu)
    x = ka * eps
    y = ks * eps
    g = x / (1.0 + x * x) + np.arctan(y)
    dg = ka * (1.0 - x * x) / (1.0 + x * x) ** 2 + ks / (1.0 + y * y)
    f = 2.0 / np.pi * mu * g
    df = 2.0 / np.pi * (dmu * g + mu * dg * deps)
    return f, df


def invert_creep(f_target, v, k_eps, ka, ks, mu0, a_ratio, b_coef, w_peak):
    """Creep velocity w in [0, w_peak] with f(w) = f_target (quasi-static rolling).

    f_target above the peak returns w_peak (the anti-spin/WSP control holds the peak)."""
    w = np.zeros_like(np.asarray(f_target, dtype=float) + 0.0 * v)
    lo = np.zeros_like(w)
    hi = np.zeros_like(w) + w_peak
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        f = creep_coeff(mid, v, k_eps, ka, ks, mu0, a_ratio, b_coef)
        lo = np.where(f < f_target, mid, lo)
        hi = np.where(f < f_target, hi, mid)
    w = 0.5 * (lo + hi)
    return w


def peak(params: dict, k_eps: float, v: float = 10.0) -> tuple[float, float]:
    """Peak adhesion coefficient and the creep velocity where it occurs, at speed v."""
    w = np.linspace(0.0, 3.0, 30001)
    f = creep_coeff(w, v, k_eps, params["kA"], params["kS"], params["mu0"], params["A"], params["B"])
    i = int(np.argmax(f))
    return float(f[i]), float(w[i])


def leaf_params(target_peak: float, k_eps: float, v: float = 10.0) -> dict:
    """Wet Polach shape with mu0 scaled so that the curve peaks at target_peak (D, from [R3])."""
    base = dict(data.POLACH["wet"])
    lo, hi = 0.01, 0.30
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        base["mu0"] = mid
        if peak(base, k_eps, v)[0] < target_peak:
            lo = mid
        else:
            hi = mid
    base["mu0"] = 0.5 * (lo + hi)
    return base


def condition_table(k_eps: float) -> np.ndarray:
    """Rows DRY, WET, DRY_LEAF, WET_LEAF; columns kA, kS, mu0, A, B."""
    rows = [
        data.POLACH["dry"],
        data.POLACH["wet"],
        leaf_params(data.MU_DRY_LEAF, k_eps),
        leaf_params(data.MU_WET_LEAF, k_eps),
    ]
    return np.array([[r["kA"], r["kS"], r["mu0"], r["A"], r["B"]] for r in rows])
