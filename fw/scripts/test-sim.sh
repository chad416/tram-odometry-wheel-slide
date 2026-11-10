#!/usr/bin/env bash
# Run the Robot Framework integration tests in Renode against the built firmware.
# Usage (inside WSL or Linux): fw/scripts/test-sim.sh [robot file] [extra renode-test args]
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

SUITE="${1:-tests/robot/odo.robot}"
shift || true
OUT="${TRAM_ROBOT_OUT:-$HOME/tram-robot-out}"
mkdir -p "$OUT"
# renode-test writes its reports to the current directory, and mishandles a
# current directory that contains spaces.
cd "$OUT"
renode-test -r "$OUT" --variable "ELF:$BUILD/zephyr/zephyr.elf" --variable "OUT:$OUT" \
	--variable "SCEN:$REPO/sim/scenarios" "$@" "$REPO/$SUITE"
