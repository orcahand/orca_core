"""Turn a visual calibration result into a hand's ``joint_map.yaml``.

    python -m orca_core.tools.joint_map_from_visual visual_calibration.json -o joint_map.yaml

Copy the output next to the hand's ``config.yaml``; see :mod:`orca_core.joint_map`.
"""

from __future__ import annotations

import argparse
import json
import sys

import yaml

from orca_core.joint_map import JOINT_MAP_FILE, from_visual_calibration
from orca_core.utils.utils import write_text_atomic

HEADER = """\
# This hand's own joint map (orca_core.joint_map). orca_core reads it next to
# config.yaml and never writes it, so a recalibration keeps it.
# joint_roms: own hard stops, override config.yaml joint_roms.
# curves: per joint, s = fraction of the motor's travel between its calibrated
# limits (0 at the lower stop) -> deg; piecewise linear, strictly increasing.
"""


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("json", help="visual_calibration.json (hand_study quickfit)")
    parser.add_argument("-o", "--output", default=JOINT_MAP_FILE, help="joint_map.yaml to write")
    args = parser.parse_args(argv)

    with open(args.json, encoding="utf-8") as f:
        doc, skipped = from_visual_calibration(json.load(f))
    write_text_atomic(args.output, HEADER + yaml.safe_dump(
        doc, sort_keys=False, default_flow_style=None, width=200))
    print(f"{args.output}: {len(doc['curves'])} curves, {len(doc['joint_roms'])} own stops "
          f"({doc['hand_serial']}, {doc['model']})")
    for joint, why in skipped.items():
        print(f"  {joint}: left linear ({why})", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
