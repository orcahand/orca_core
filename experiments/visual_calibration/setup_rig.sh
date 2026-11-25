#!/bin/bash
# Set up a rig Mac for the camera rig, from four checkouts side by side:
#
#   <dir>/orca_core              internal/rig-calibration      (this script's checkout)
#   <dir>/orca_ui                internal/rig-calibration
#   <dir>/orca_firmware          internal/board-calibration-store   (optional: board writes)
#   <dir>/orcahand_description   main
#
#   experiments/visual_calibration/setup_rig.sh [--no-firmware] [--no-weights]
#
# Other places: ORCA_UI, ORCA_FIRMWARE_REPO, ORCAHAND_DESCRIPTION (orca_core is always the
# checkout holding this script). Safe to run again: every step only does what is missing.
#
# 1. checks each checkout's branch (a warning when another is checked out);
# 2. orca_ui's environment on Python 3.12 (uv sync), orca_core's `rig` extra installed into it,
#    and orca_ui pointed at this orca_core (./dev local);
# 3. the BiRefNet weights downloaded once (~450 MB) and run on a blank photo (--no-weights skips);
# 4. orca_firmware's ./setup.sh, its toolchain (--no-firmware skips);
# 5. node/npx and uvcc, Rosetta on an arm64 Mac;
# 6. the doctor (python -m visual_calibration.doctor). Its hardware checks fail until the rig's
#    cameras and boards are plugged in.
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
WEIGHTS=1
for arg in "$@"; do
    case "$arg" in
        --no-firmware) FIRMWARE_SETUP=0 ;;
        --no-weights) WEIGHTS=0 ;;
        -h | --help) sed -n '2,23s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "setup_rig: unknown option $arg (--no-firmware, --no-weights, --help)" >&2; exit 2 ;;
    esac
done

WARNINGS=0
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
check_checkout() {   # name dir branch required
    if [ ! -d "$2/.git" ] && [ ! -f "$2/.git" ]; then
        if [ "$4" = required ]; then
            die "$1: no git checkout at $2 (clone it there, or set the path: see --help)"
        fi
        warn "$1: no git checkout at $2"
        return 1
    fi
    b=$(branch_of "$2")
    if [ "$b" = "$3" ]; then
        ok "$1 at $2 ($b)"
    else
        warn "$1 at $2 is on '${b:-a detached HEAD}', not $3:   git -C $2 switch $3"
    fi
}
check_checkout orca_core "$CORE" internal/rig-calibration required
check_checkout orca_ui "$UI" internal/rig-calibration required
check_checkout orcahand_description "$DESCRIPTION" main required
HAVE_FIRMWARE=1
check_checkout orca_firmware "$FIRMWARE" internal/board-calibration-store optional || HAVE_FIRMWARE=0
[ -f "$UI/dev" ] || die "orca_ui at $UI has no ./dev script: not internal/rig-calibration?"

step "orca_ui's environment (Python $PYTHON_VERSION) with orca_core's rig extra"
PY=$UI/.venv/bin/python
(cd "$UI" && uv sync --inexact --python "$PYTHON_VERSION" --quiet) || die "uv sync in $UI failed"
have=$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')
[ "$have" = "$PYTHON_VERSION" ] || die "$PY is Python $have, not $PYTHON_VERSION: rm -rf $UI/.venv and run again"
ok "$PY (Python $have)"
uv pip install --quiet --python "$PY" -r "$CORE/pyproject.toml" --extra rig || die "installing the rig extra failed"
ok "orca_core's rig extra installed (OpenCV, torch, transformers, ...)"
(cd "$UI" && ./dev local "$CORE" >/dev/null) || die "./dev local $CORE failed in $UI"
where=$("$PY" -c 'import os, orca_core; print(os.path.dirname(os.path.realpath(orca_core.__file__)))')
[ "$where" = "$(cd "$CORE/orca_core" && pwd -P)" ] || die "orca_ui imports orca_core from $where, not $CORE"
ok "orca_ui runs on this orca_core ($CORE)"

step "BiRefNet weights"
if [ "$WEIGHTS" = 1 ]; then
    (cd "$HERE" && "$PY" -c '
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
else
    (cd "$FIRMWARE" && ./setup.sh) || die "orca_firmware ./setup.sh failed"
    ok "$FIRMWARE/.arduino"
fi

step "Node and uvcc (the camera controls)"
if ! command -v npx >/dev/null 2>&1; then
    warn "npx not found: brew install node   (or https://nodejs.org)"
elif v=$($UVCC --version 2>/dev/null | tail -n 1); then
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
export ORCA_VISUAL_CALIBRATION="$HERE" ORCAHAND_DESCRIPTION="$DESCRIPTION"
if [ "$HAVE_FIRMWARE" = 1 ]; then export ORCA_FIRMWARE_REPO="$FIRMWARE"; else unset ORCA_FIRMWARE_REPO; fi
cd "$HERE"
"$PY" -m visual_calibration.doctor python opencv segmenter_packages rig_packages birefnet_weights \
    orcahand_description uvcc orca_core firmware rosetta \
    || die "the software checks above failed: fix them and run this again"
echo
echo "The rig's hardware (errors until the cameras and boards are plugged in):"
"$PY" -m visual_calibration.doctor usb_cameras camera_identity turntable_board hand_board || true

step "Done"
echo "  $WARNINGS warning(s) above."
echo "  Cameras: $PY -m visual_calibration.sweep --assign-cameras   (from $HERE)"
echo "  Console: $HERE/rig-console.sh"
