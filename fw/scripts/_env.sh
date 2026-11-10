# Sourced by the other scripts. Sets REPO, WS and BUILD, and activates the
# Python venv. Environment variables (all optional):
#   TRAM_WS     west workspace (zephyr/, modules/); default $HOME/recorder-ws
#   TRAM_VENV   Python venv with west and Robot; default $HOME/zvenv
#   TRAM_BUILD  firmware build directory; default $TRAM_WS/build/tram
# Zephyr's build breaks on paths that contain a space. If the repository path
# has one (for example a Windows folder seen from WSL), it is bind-mounted at
# $HOME/tram-src first (scripts/wsl-mount.sh, needs root).

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$REPO" == *" "* ]]; then
	bash "$REPO/scripts/wsl-mount.sh" >/dev/null
	REPO="$HOME/tram-src/fw"
fi
WS="${TRAM_WS:-$HOME/recorder-ws}"
BUILD="${TRAM_BUILD:-$WS/build/tram}"

# shellcheck disable=SC1091
source "${TRAM_VENV:-$HOME/zvenv}/bin/activate"
