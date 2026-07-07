#!/usr/bin/env python
"""Example: stream per-taxel forces with 3D taxel positions in a chosen frame.

Positions come from the static sensor geometry and are transformed through the
hand's forward kinematics; forces are rotated into the same frame. With
``--mock`` no hardware is needed (synthetic tactile data, fixed joint pose).

Usage:
    uv run python scripts/example_taxel_frames.py --mock --frame base
    uv run python scripts/example_taxel_frames.py path/to/config.yaml --frame palm
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from orca_core import OrcaHandTouch, frames

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "orca_core" / "models" / "v2" / "orcahand-touch" / "config.yaml"
)

MOCK_POSE = {"wrist": -10.0, "index_mcp": 30.0, "index_pip": 20.0, "thumb_cmc": -15.0}


def _connect_mock(hand):
    """Attach a TactileClient on a mock serial link and return a frame feeder."""
    from orca_core.constants import FINGER_NAMES
    from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
    from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
    from orca_core.hardware.tactile_client import TactileClient
    from tests._tactile_helpers import TactileMockState, feed_taxels_frame, install_tactile_mock

    state = TactileMockState()
    link = MockHandSerialLink()
    install_tactile_mock(link, state)
    link.connect()
    client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
    client.connect()
    hand._tactile_client = client

    def feed(t: float) -> None:
        taxels = {}
        for finger in FINGER_NAMES:
            n = DEFAULT_TAXEL_COUNTS[finger]
            fz = [1.0 + np.sin(t * 2.0 + i * 0.3) for i in range(n)]
            taxels[finger] = [[0.2, -0.1, round(max(f, 0.0), 1)] for f in fz]
        feed_taxels_frame(link, taxels, state.active_sensors)

    return feed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config_path", nargs="?", default=str(DEFAULT_CONFIG))
    parser.add_argument("--frame", default=frames.FINGERTIP, choices=frames.FRAMES)
    parser.add_argument("--mock", action="store_true", help="run without hardware")
    args = parser.parse_args()

    hand = OrcaHandTouch(config_path=args.config_path)
    feed = None
    if args.mock:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        feed = _connect_mock(hand)
    else:
        ok, msg = hand.connect_sensors_only()
        print(msg)
        if not ok:
            sys.exit(1)

    needs_joints = args.frame in (frames.PALM, frames.BASE, frames.WORLD)
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
        if not args.mock:
            hand.disconnect()


if __name__ == "__main__":
    main()
