#!/usr/bin/env python3
"""Traceability check for the recorder SRS.

Fails if a requirement ID in fw/SRS.md is missing from fw/docs/traceability.md,
has no test or evidence, names a unit-test function, an integration test or a
file that does not exist, or if a Renode test tag names an unknown requirement.
"""

from __future__ import annotations

import os
import re
import sys

FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(FW)


def srs_ids():
    text = open(os.path.join(FW, "SRS.md"), encoding="utf-8").read()
    return sorted(set(re.findall(r"\*\*(OR-\d\d)\s*\([^)]*\):\*\*", text)))


def table():
    rows = {}
    for line in open(os.path.join(FW, "docs", "traceability.md"), encoding="utf-8"):
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) == 5 and re.fullmatch(r"OR-\d\d", cells[0]):
            rows[cells[0]] = cells
    return rows


def main():
    errors = []
    udir = os.path.join(FW, "tests", "unit")
    unit_src = {
        f: open(os.path.join(udir, f), encoding="utf-8").read() for f in os.listdir(udir) if f.endswith(".c")
    }
    robot = open(os.path.join(FW, "tests", "robot", "odo.robot"), encoding="utf-8").read()
    it_names = set(re.findall(r"^(IT\d\d)\b", robot, re.M))
    tags = set(re.findall(r"\b(OR-\d\d)\b", " ".join(re.findall(r"\[Tags\](.*)", robot))))

    ids = srs_ids()
    rows = table()
    for rid in ids:
        if rid not in rows:
            errors.append("%s: in SRS.md but not in docs/traceability.md" % rid)
            continue
        _, _, unit, integ, other = rows[rid]
        if not (unit or integ or other):
            errors.append("%s: no test or evidence" % rid)
        for item in filter(None, (x.strip() for x in unit.split(","))):
            fname, _, func = item.partition(":")
            src = unit_src.get(fname)
            if src is None or not re.search(r"^(static )?void %s\(void\)" % re.escape(func), src, re.M):
                errors.append("%s: unit test %s not found" % (rid, item))
        for item in filter(None, (x.strip() for x in integ.split(","))):
            if item not in it_names:
                errors.append("%s: integration test %s not found" % (rid, item))
        for path in re.findall(r"(?:model|fw|docs|tools)/[\w./-]+", other):
            if not os.path.exists(os.path.join(ROOT, path)):
                errors.append("%s: %s does not exist" % (rid, path))
    for rid in rows:
        if rid not in ids:
            errors.append("%s: in docs/traceability.md but not in SRS.md" % rid)
    for t in sorted(tags - set(ids)):
        errors.append("%s: Renode test tag names an unknown requirement" % t)

    for e in errors:
        print("TRACE ERROR " + e)
    print(
        "%d requirements, %d traced, %d integration tests, %d errors"
        % (len(ids), len(rows), len(it_names), len(errors))
    )
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
