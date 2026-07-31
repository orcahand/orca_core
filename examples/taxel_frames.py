#!/usr/bin/env python
"""Example: stream per-taxel forces with 3D taxel positions in a chosen frame.

Positions come from the static sensor geometry and are transformed through the
hand's forward kinematics; forces are rotated into the same frame. With
``--mock`` no hardware is needed (synthetic tactile data, fixed joint pose).
On hardware, the joint-dependent frames (palm/base/world) connect the full
hand for live joint angles; sensor/fingertip frames connect sensors only.

Usage:
    uv run python examples/taxel_frames.py --mock --frame base
    uv run python examples/taxel_frames.py path/to/config.yaml --frame palm
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from orca_core import MockOrcaHandTouch, OrcaHandTouch, frames

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "orca_core" / "models" / "v2" / "orcahand-touch-right" / "config.yaml"
)

MOCK_POSE = {"wrist": -10.0, "index_mcp": 30.0, "index_pip": 20.0, "thumb_cmc": -15.0}


def _make_mock_feeder(hand):
    """Return a feeder that pushes synthetic taxel frames into the mock link."""
    from orca_core.constants import FINGER_NAMES
    from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
    from orca_core.hardware.sensing.tactile_mock import feed_taxels_frame

    # Slot-id order matches the wire-frame order the parser expects.
    active = sorted(FINGER_NAMES, key=lambda f: hand.config.finger_to_sensor_id[f])
    link = hand.tactile_mock_link

    def feed(t: float) -> None:
        taxels = {}
        for finger in FINGER_NAMES:
            n = DEFAULT_TAXEL_COUNTS[finger]
            fz = [1.0 + np.sin(t * 2.0 + i * 0.3) for i in range(n)]
            taxels[finger] = [[0.2, -0.1, round(max(f, 0.0), 1)] for f in fz]
        feed_taxels_frame(link, taxels, active)

    return feed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config_path", nargs="?", default=str(DEFAULT_CONFIG))
    parser.add_argument("--frame", default=frames.FINGERTIP, choices=frames.FRAMES)
    parser.add_argument("--mock", action="store_true", help="run without hardware")
    args = parser.parse_args()

    needs_joints = args.frame in (frames.PALM, frames.BASE, frames.WORLD)
    hand_cls = MockOrcaHandTouch if args.mock else OrcaHandTouch
    hand = hand_cls(config_path=args.config_path)

    if not args.mock and needs_joints:
        # Kinematic frames need live joint angles, so open the motor bus too.
        ok, msg = hand.connect()
    else:
        ok, msg = hand.connect_sensors_only()
    print(msg)
    if not ok:
        sys.exit(1)

    feed = _make_mock_feeder(hand) if args.mock else None
    joint_pos = MOCK_POSE if (args.mock and needs_joints) else None

    try:
        hand.start_tactile_stream(resultant=False, taxels=True, min_sensors=1)
        time.sleep(0.2)

        print(f"Streaming taxel data in the '{args.frame}' frame — Ctrl+C to stop.\n")
        data = None
        while True:
            if feed is not None:
                feed(time.monotonic())
            data = hand.get_taxel_data(frame=args.frame, joint_pos=joint_pos)
            if data is None:
                time.sleep(0.01)
                continue
            lines = []
            for finger, taxels in sorted(data.items()):
                pressed = int(np.argmax(np.linalg.norm(taxels.forces, axis=1)))
                pos_mm = 1000.0 * taxels.positions[pressed]
                force = taxels.forces[pressed]
                lines.append(
                    f"{finger:>6}: peak taxel {pressed:3d} at "
                    f"[{pos_mm[0]:7.1f} {pos_mm[1]:7.1f} {pos_mm[2]:7.1f}] mm, "
                    f"F=[{force[0]:5.2f} {force[1]:5.2f} {force[2]:5.2f}] N"
                )
            print("\n".join(lines) + f"\n({args.frame} frame)", end="\033[F" * len(lines) + "\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n" * (len(data) + 1) if data else "")
    finally:
        hand.disconnect()


if __name__ == "__main__":
    main()
