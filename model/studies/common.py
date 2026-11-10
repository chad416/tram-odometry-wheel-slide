"""Shared helpers for the studies: paths, figure style, JSON results, cached batches."""

from __future__ import annotations

import json
import os
import pickle
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.dirname(HERE)
ROOT = os.path.dirname(MODEL)
IMG = os.path.join(ROOT, "docs", "img")
RES = os.path.join(ROOT, "results")
CACHE = os.path.join(ROOT, "work", "cache")
sys.path.insert(0, MODEL)

from odo import vehicle  # noqa: E402

QUICK = os.environ.get("ODO_QUICK", "") == "1"

plt.rcParams.update(
    {
        "figure.dpi": 110,
        "savefig.dpi": 130,
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)
COLORS = {
    "E1": "#9a3b3b",
    "E2": "#d08a3a",
    "E3": "#2a6f97",
    "E3a": "#7aa6c2",
    "E3h": "#56a0a0",
    "E3g": "#8c6bb1",
    "E4": "#5b8c5a",
    "E5": "#3d3d3d",
}


def save(fig, name: str) -> str:
    os.makedirs(IMG, exist_ok=True)
    path = os.path.join(IMG, name)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("wrote", os.path.relpath(path, ROOT))
    return path


def write_json(name: str, obj) -> None:
    os.makedirs(RES, exist_ok=True)
    path = os.path.join(RES, name)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=2, default=_conv)
        f.write("\n")
    print("wrote", os.path.relpath(path, ROOT))


def _conv(o):
    if isinstance(o, (np.floating,)):
        return round(float(o), 6)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def batch(tag: str, seed: int, runs: int, cond_mode: str = "mixed", **over):
    """Simulate (or load from work/cache) a batch of runs."""
    os.makedirs(CACHE, exist_ok=True)
    f = os.path.join(CACHE, "%s_%d_%d_%s.pkl" % (tag, seed, runs, cond_mode))
    if os.path.exists(f):
        with open(f, "rb") as fh:
            return pickle.load(fh)
    b = vehicle.make_batch(np.random.default_rng(seed), runs, cond_mode, **over)
    log = vehicle.simulate(b)
    with open(f, "wb") as fh:
        pickle.dump((b, log), fh)
    return b, log


def pct(x, q):
    return float(np.percentile(np.abs(x), q))
