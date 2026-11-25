#!/bin/bash
# Start orca-ui for the camera rig, from orca_ui's environment as setup_rig.sh made it, with
# the rig's paths set: ORCA_VISUAL_CALIBRATION (this folder), ORCAHAND_DESCRIPTION and, when
# the checkout is there, ORCA_FIRMWARE_REPO ("Write to board"). Same layout and overrides as
# setup_rig.sh (ORCA_UI, ORCA_FIRMWARE_REPO, ORCAHAND_DESCRIPTION).
#
#   experiments/visual_calibration/rig-console.sh [orca-ui options, e.g. --research-ui]
#
# Quit with one Ctrl+C: a hand run is stopped and the torque goes off.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(dirname "$(cd "$HERE/../.." && pwd)")
UI=${ORCA_UI:-$ROOT/orca_ui}
FIRMWARE=${ORCA_FIRMWARE_REPO:-$ROOT/orca_firmware}
PY=$UI/.venv/bin/python

if [ ! -x "$PY" ]; then
    echo "rig-console: no environment at $UI/.venv: run $HERE/setup_rig.sh first" >&2
    exit 1
fi
export ORCA_VISUAL_CALIBRATION="$HERE"
export ORCAHAND_DESCRIPTION="${ORCAHAND_DESCRIPTION:-$ROOT/orcahand_description}"
if [ -f "$FIRMWARE/tools/bench/cal_store.py" ]; then
    export ORCA_FIRMWARE_REPO="$FIRMWARE"
else
    unset ORCA_FIRMWARE_REPO
fi
cd "$UI"
exec "$PY" -m orca_ui "$@"
