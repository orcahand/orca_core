#!/bin/bash
# Set up a rig Mac for the camera rig, from four checkouts side by side:
#
#   <dir>/orca_core              internal/rig-calibration      (this script's checkout)
#   <dir>/orca_ui                internal/rig-calibration
#   <dir>/orca_firmware          internal/board-calibration-store   (optional: board writes)
#   <dir>/orcahand_description   main
#
#   experiments/visual_calibration/setup_rig.sh [--no-firmware] [--no-weights] [--update]
#
# Other places: ORCA_UI, ORCA_FIRMWARE_REPO, ORCAHAND_DESCRIPTION (orca_core is always the
# checkout holding this script). Safe to run again: every step only does what is missing.
#
# 1. checks each checkout's branch: orca_core or orca_ui on another branch stops it before anything
#    changes; orca_firmware or orcahand_description on another branch is a warning;
# 2. orca_ui's environment on Python 3.12 (uv sync), orca_core's `rig` extra installed into it,
#    and orca_ui pointed at this orca_core (./dev local);
# 3. the BiRefNet weights (pinned revision) downloaded once (~450 MB) and run on a blank photo
#    (--no-weights skips it);
# 4. orca_firmware's ./setup.sh, its toolchain, unless it is there already (--update runs it
#    anyway, --no-firmware never);
# 5. node/npx and uvcc, Rosetta on an arm64 Mac;
# 6. the doctor (python -m visual_calibration.doctor). Its hardware checks fail until the rig's
#    cameras and boards are plugged in.
#
# Offline, a run again only warns where it could not refresh something it already has.
#
# Then start the console with experiments/visual_calibration/rig-console.sh.
set -eu
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
CORE=$(cd "$HERE/../.." && pwd)
ROOT=$(dirname "$CORE")
UI=${ORCA_UI:-$ROOT/orca_ui}
FIRMWARE=${ORCA_FIRMWARE_REPO:-$ROOT/orca_firmware}
DESCRIPTION=${ORCAHAND_DESCRIPTION:-$ROOT/orcahand_description}
PYTHON_VERSION=3.12
UVCC=${ORCA_UVCC:-npx --yes uvcc@8.0.0}

FIRMWARE_SETUP=1
FIRMWARE_UPDATE=0
WEIGHTS=1
for arg in "$@"; do
    case "$arg" in
        --no-firmware) FIRMWARE_SETUP=0 ;;
        --no-weights) WEIGHTS=0 ;;
        --update) FIRMWARE_UPDATE=1 ;;
        -h | --help) sed -n '2,/^set -eu/s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "setup_rig: unknown option $arg (--no-firmware, --no-weights, --update, --help)" >&2; exit 2 ;;
    esac
done

WARNINGS=0
q() { printf '%q' "$1"; }
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
ok() { printf '  ok    %s\n' "$1"; }
warn() { printf '  WARN  %s\n' "$1"; WARNINGS=$((WARNINGS + 1)); }
die() { printf '  ERROR %s\n' "$1" >&2; exit 1; }

step "Tools"
[ "$(uname -s)" = Darwin ] || warn "not macOS: the rig's camera code (ioreg, AVFoundation) is macOS only"
command -v git >/dev/null 2>&1 || die "git not found: xcode-select --install"
command -v uv >/dev/null 2>&1 || die "uv not found: curl -LsSf https://astral.sh/uv/install.sh | sh   (or: brew install uv)"
ok "git $(git --version | awk '{print $3}'), uv $(uv --version | awk '{print $2}')"

step "Checkouts"
branch_of() { git -C "$1" branch --show-current 2>/dev/null || true; }
check_checkout() {   # name dir branch required|optional; a wrong branch: required stops, optional warns
    if [ ! -d "$2/.git" ] && [ ! -f "$2/.git" ]; then
        if [ "$4" = required ]; then
            die "$1: no git checkout at $(q "$2") (clone it there, or set its path: see --help)"
        fi
        warn "$1: no git checkout at $(q "$2")"
        return 1
    fi
    b=$(branch_of "$2")
    if [ "$b" = "$3" ]; then
        ok "$1 at $(q "$2") ($b)"
    elif [ "$4" = required ]; then
        die "$1 at $(q "$2") is on '${b:-a detached HEAD}', not $3; nothing was changed. Switch it, then run this again:   git -C $(q "$2") switch $3"
    else
        warn "$1 at $(q "$2") is on '${b:-a detached HEAD}', not $3:   git -C $(q "$2") switch $3"
    fi
}
check_checkout orca_core "$CORE" internal/rig-calibration required
check_checkout orca_ui "$UI" internal/rig-calibration required
[ -f "$UI/dev" ] || die "orca_ui at $(q "$UI") has no ./dev script: not internal/rig-calibration?"
HAVE_DESCRIPTION=1
check_checkout orcahand_description "$DESCRIPTION" main optional || HAVE_DESCRIPTION=0
[ "$HAVE_DESCRIPTION" = 1 ] || die "orcahand_description is needed for the hand fit: git clone git@github.com:orcahand/orcahand_description.git $(q "$DESCRIPTION")"
HAVE_FIRMWARE=1
check_checkout orca_firmware "$FIRMWARE" internal/board-calibration-store optional || HAVE_FIRMWARE=0

PY=$UI/.venv/bin/python
doctor() { (cd "$HERE" && "$PY" -m visual_calibration.doctor "$@"); }
packages_ok() { doctor opencv segmenter_packages rig_packages >/dev/null 2>&1; }
core_here() {   # orca_ui's Python imports this orca_core (asked from orca_ui's folder, as orca-ui runs)
    [ "$(cd "$UI" && "$PY" -c 'import os, orca_core; print(os.path.dirname(os.path.realpath(orca_core.__file__)))' 2>/dev/null)" \
        = "$(cd "$CORE/orca_core" && pwd -P)" ]
}

step "orca_ui's environment (Python $PYTHON_VERSION) with orca_core's rig extra"
if ! (cd "$UI" && uv sync --inexact --python "$PYTHON_VERSION" --quiet); then
    [ -x "$PY" ] || die "uv sync in $(q "$UI") failed (see above)"
    warn "uv sync in $(q "$UI") failed (offline?): keeping the environment it has"
fi
have=$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
[ "$have" = "$PYTHON_VERSION" ] || die "$(q "$PY") is Python $have, not $PYTHON_VERSION: rm -rf $(q "$UI/.venv") and run this again"
ok "$(q "$PY") (Python $have)"
if uv pip install --quiet --python "$PY" -r "$CORE/pyproject.toml" --extra rig; then
    ok "orca_core's rig extra installed (OpenCV, torch, transformers, ...)"
elif packages_ok; then
    warn "could not refresh orca_core's rig extra (offline?): the installed packages import"
else
    die "installing orca_core's rig extra failed (see above)"
fi
if ! (cd "$UI" && ./dev local "$CORE" >/dev/null); then
    core_here || die "./dev local $(q "$CORE") failed in $(q "$UI") (see above)"
    warn "./dev local failed (offline?): orca_ui already runs on this orca_core"
fi
core_here || die "orca_ui's Python does not import orca_core from $(q "$CORE"): cd $(q "$UI") && ./dev local $(q "$CORE")"
ok "orca_ui runs on this orca_core ($(q "$CORE"))"

step "BiRefNet weights"
if [ "$WEIGHTS" = 1 ]; then
    offline=
    doctor birefnet_weights >/dev/null 2>&1 && offline=1       # cached: load it without the network
    (cd "$HERE" && HF_HUB_OFFLINE=${offline:-${HF_HUB_OFFLINE:-}} "$PY" -c '
import time
import numpy as np
from hand_study.segment import BiRefNet
t = time.time()
s = BiRefNet()
s.prob(np.zeros((64, 64, 3), np.uint8))
print("  ok    BiRefNet loads and runs on %s (%.0f s)" % (s.dev, time.time() - t))
' 2>&1 | grep -v -e "^Loading weights" -e "Redirects are currently not supported") || die "BiRefNet did not load (network? see above)"
else
    warn "skipped (--no-weights): the first hand run downloads them"
fi

step "Firmware toolchain"
if [ "$HAVE_FIRMWARE" = 0 ]; then
    warn "no orca_firmware checkout: \"Write to board\" stays hidden (optional)"
elif [ "$FIRMWARE_SETUP" = 0 ]; then
    warn "skipped (--no-firmware)"
elif [ -x "$FIRMWARE/.arduino/bin/arduino-cli" ] && [ "$FIRMWARE_UPDATE" = 0 ]; then
    ok "$(q "$FIRMWARE/.arduino") is there (--update runs its ./setup.sh again)"
else
    (cd "$FIRMWARE" && ./setup.sh) || die "orca_firmware's ./setup.sh failed (see above)"
    ok "$(q "$FIRMWARE/.arduino")"
fi

step "Node and uvcc (the camera controls)"
cached=$(echo "$UVCC" | sed 's/ --yes / --no-install /')
if ! command -v npx >/dev/null 2>&1; then
    warn "npx not found: brew install node   (or https://nodejs.org)"
elif v=$($cached --version 2>/dev/null | tail -n 1) || v=$($UVCC --version 2>/dev/null | tail -n 1); then
    ok "uvcc $v ($UVCC)"
else
    warn "$UVCC --version failed (network on its first run?)"
fi
if [ "$(uname -m)" = arm64 ]; then
    if arch -x86_64 /usr/bin/true 2>/dev/null; then
        ok "Rosetta"
    else
        warn "Rosetta missing (the firmware toolchain needs it): softwareupdate --install-rosetta --agree-to-license"
    fi
fi

step "Doctor"
export ORCA_VISUAL_CALIBRATION="$HERE" ORCAHAND_DESCRIPTION="$DESCRIPTION" ORCA_UI="$UI"
if [ "$HAVE_FIRMWARE" = 1 ]; then export ORCA_FIRMWARE_REPO="$FIRMWARE"; else unset ORCA_FIRMWARE_REPO; fi
if [ "$WEIGHTS" = 1 ]; then
    doctor software || die "the software checks above failed: fix them and run this again"
else
    doctor software --skip birefnet_weights || die "the software checks above failed: fix them and run this again"
    doctor birefnet_weights || true
fi
echo
echo "The rig's hardware (errors until the cameras and boards are plugged in):"
doctor hardware || true

step "Done"
echo "  $WARNINGS warning(s) above."
echo "  Cameras: cd $(q "$HERE") && $(q "$PY") -m visual_calibration.sweep --assign-cameras"
echo "  Console: $(q "$HERE/rig-console.sh")"
