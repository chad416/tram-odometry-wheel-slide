#!/usr/bin/env python3
"""Run the studies: python model/run_all.py [s1 s3 s4 s5 s6 s7]   (default: all).
ODO_QUICK=1 runs reduced Monte-Carlo sizes (CI). S3 must run before S6."""

from __future__ import annotations

import importlib
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "studies"))

STUDIES = {
    "s1": "s1_adhesion",
    "s3": "s3_montecarlo",
    "s4": "s4_budget",
    "s5": "s5_holdup",
    "s6": "s6_aoa",
    "s7": "s7_rma",
}


def main(argv):
    keys = argv or list(STUDIES)
    for k in keys:
        t0 = time.time()
        print("== %s (%s)" % (k, STUDIES[k]))
        importlib.import_module(STUDIES[k]).run()
        print("== %s done in %.0f s" % (k, time.time() - t0))


if __name__ == "__main__":
    main(sys.argv[1:])
