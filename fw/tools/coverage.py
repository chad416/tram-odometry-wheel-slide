#!/usr/bin/env python3
"""Summarise GCC 14 gcov JSON (gcov --json-format --stdout --conditions).

Reports statement (line), branch and MC/DC (condition outcome) coverage per
source file under --src, merges copies of header inline functions across
translation units, lists every uncovered item, and can fail below thresholds.

    python3 tools/coverage.py build/host/coverage.json --src lib/odo/src \
        [--min-line 100] [--min-branch 100] [--min-cond 100] [--markdown docs/coverage_table.md]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict


def load(path):
    docs = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                docs.append(json.loads(line))
    return docs


def merge(docs, src):
    # file -> line -> merged record
    out = defaultdict(dict)
    for doc in docs:
        for fe in doc.get("files", []):
            name = fe["file"].replace("\\", "/")
            if src not in name:
                continue
            name = name[name.index(src) :]
            for ln in fe.get("lines", []):
                n = ln["line_number"]
                rec = out[name].setdefault(
                    n, {"count": 0, "branches": None, "cond_total": 0, "cond_nt": None, "cond_nf": None}
                )
                rec["count"] += ln.get("count", 0)
                br = [b.get("count", 0) for b in ln.get("branches", [])]
                if br:
                    if rec["branches"] is None:
                        rec["branches"] = br
                    else:
                        rec["branches"] = [a + b for a, b in zip(rec["branches"], br)]
                for c in ln.get("conditions", []):
                    nt = set(c.get("not_covered_true", []))
                    nf = set(c.get("not_covered_false", []))
                    rec["cond_total"] = max(rec["cond_total"], c.get("count", 0))
                    rec["cond_nt"] = nt if rec["cond_nt"] is None else (rec["cond_nt"] & nt)
                    rec["cond_nf"] = nf if rec["cond_nf"] is None else (rec["cond_nf"] & nf)
    return out


def summarise(merged):
    rows = []
    missing = []
    tot = [0, 0, 0, 0, 0, 0]
    for name in sorted(merged):
        lines = merged[name]
        l_tot = len(lines)
        l_cov = sum(1 for r in lines.values() if r["count"] > 0)
        b_tot = b_cov = c_tot = c_cov = 0
        for n, r in sorted(lines.items()):
            if r["count"] == 0:
                missing.append(f"{name}:{n}: line not executed")
            if r["branches"]:
                b_tot += len(r["branches"])
                taken = sum(1 for x in r["branches"] if x > 0)
                b_cov += taken
                if taken < len(r["branches"]):
                    missing.append(f"{name}:{n}: {len(r['branches']) - taken} branch(es) not taken")
            if r["cond_total"]:
                nc = len(r["cond_nt"] or ()) + len(r["cond_nf"] or ())
                c_tot += r["cond_total"]
                c_cov += r["cond_total"] - nc
                if nc:
                    missing.append(
                        f"{name}:{n}: MC/DC outcomes missing: "
                        f"not shown true {sorted(r['cond_nt'])}, not shown false {sorted(r['cond_nf'])}"
                    )
        rows.append((os.path.basename(name), l_cov, l_tot, b_cov, b_tot, c_cov, c_tot))
        for i, x in enumerate((l_cov, l_tot, b_cov, b_tot, c_cov, c_tot)):
            tot[i] += x
    return rows, tot, missing


def pct(a, b):
    return 100.0 if b == 0 else 100.0 * a / b


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("json")
    ap.add_argument("--src", default="lib/odo/src")
    ap.add_argument("--min-line", type=float, default=0.0)
    ap.add_argument("--min-branch", type=float, default=0.0)
    ap.add_argument("--min-cond", type=float, default=0.0)
    ap.add_argument("--markdown")
    a = ap.parse_args()

    rows, tot, missing = summarise(merge(load(a.json), a.src))
    hdr = f"{'file':<14} {'lines':>14} {'branches':>14} {'MC/DC outcomes':>16}"
    print(hdr)
    for r in rows + [("TOTAL",) + tuple(tot)]:
        print(
            f"{r[0]:<14} {r[1]:>5}/{r[2]:<4}{pct(r[1], r[2]):5.1f}%"
            f" {r[3]:>5}/{r[4]:<4}{pct(r[3], r[4]):5.1f}%"
            f" {r[5]:>5}/{r[6]:<4}{pct(r[5], r[6]):6.1f}%"
        )
    for m in missing:
        print("  " + m)

    if a.markdown:
        with open(a.markdown, "w", encoding="utf-8", newline="\n") as f:
            f.write("| File | Lines | Branches | MC/DC condition outcomes |\n|---|---|---|---|\n")
            for r in rows + [("**Total**",) + tuple(tot)]:
                f.write(
                    f"| {r[0]} | {r[1]}/{r[2]} ({pct(r[1], r[2]):.1f} %)"
                    f" | {r[3]}/{r[4]} ({pct(r[3], r[4]):.1f} %)"
                    f" | {r[5]}/{r[6]} ({pct(r[5], r[6]):.1f} %) |\n"
                )

    ok = (
        pct(tot[0], tot[1]) >= a.min_line
        and pct(tot[2], tot[3]) >= a.min_branch
        and pct(tot[4], tot[5]) >= a.min_cond
    )
    if not ok:
        print("coverage below the required minimum", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
