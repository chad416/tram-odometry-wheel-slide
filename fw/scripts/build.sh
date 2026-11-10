#!/usr/bin/env bash
# Build the recorder firmware for the MIMXRT1064-EVK (runs in Renode).
# Usage (inside WSL or Linux): fw/scripts/build.sh [extra west build args]
#   TRAM_IMAGE=test  (default) prj.conf + test.conf: the image the Renode tests run
#   TRAM_IMAGE=field prj.conf alone
set -euo pipefail
# shellcheck disable=SC1091
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

case "${TRAM_IMAGE:-test}" in
test) EXTRA=(-DEXTRA_CONF_FILE=test.conf) ;;
field) EXTRA=() ;;
*) echo "TRAM_IMAGE must be test or field" >&2; exit 2 ;;
esac

cd "$WS"
west build -p auto -b mimxrt1064_evk -d "$BUILD" "$REPO/app" "$@" -- "${EXTRA[@]}"
