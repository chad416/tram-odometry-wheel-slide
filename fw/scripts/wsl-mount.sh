#!/usr/bin/env bash
# Bind-mount this repository at a path without spaces (Zephyr's build breaks on
# such paths, and a symlink is not enough). _env.sh calls this automatically
# when needed. mount --bind needs root.
# Usage: bash "<repo>/fw/scripts/wsl-mount.sh" [mount point]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${1:-$HOME/tram-src}"

if mountpoint -q "$TARGET"; then
	echo "already mounted: $TARGET"
	exit 0
fi
[ -L "$TARGET" ] && rm "$TARGET"
mkdir -p "$TARGET"
mount --bind "$REPO" "$TARGET"
echo "mounted $REPO -> $TARGET"
