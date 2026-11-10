#!/usr/bin/env bash
# MISRA C:2012 check of lib/odo with cppcheck's MISRA addon (OR-70).
# Fails on any finding that is not a documented deviation (fw/misra-suppressions.txt).
# cppcheck's exit code does not cover every addon finding, so any output fails.
set -euo pipefail
FW="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$FW"
OUT="$(cppcheck --addon=misra --std=c99 --enable=warning,style,portability --inline-suppr \
	--suppressions-list=misra-suppressions.txt --error-exitcode=1 \
	-I lib/odo/include --quiet --template='{file}:{line}: {id}: {message}' lib/odo/src 2>&1)" || true
if [ -n "$OUT" ]; then
	echo "$OUT"
	echo "MISRA C:2012 check FAILED" >&2
	exit 1
fi
echo "MISRA C:2012 check: no findings outside the documented deviations"
