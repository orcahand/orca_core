#!/usr/bin/env python
"""Minimal example: connect to an OrcaHandTouch and stream resultant forces.

Usage:
    uv run python scripts/example_tactile.py
    uv run python scripts/example_tactile.py path/to/config.yaml
"""

import sys
import time
from pathlib import Path

from orca_core import OrcaHandTouch

DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "orca_core" / "models" / "v2" / "orcahand_touch_right" / "config.yaml"
)


def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else str(DEFAULT_CONFIG)
    hand = OrcaHandTouch(config_path=config_path)

    ok, msg = hand.connect_sensors_only()
    print(msg)
    if not ok:
        sys.exit(1)

    try:
        hand.start_tactile_stream(resultant=True, taxels=False, min_sensors=1)
        time.sleep(0.1)

        print("Streaming resultant forces — press Ctrl+C to stop.\n")
        while True:
            forces = hand.get_tactile_forces()
            if forces is None:
                time.sleep(0.01)
                continue
            parts = [f"{f}: [{v[0]:6.2f} {v[1]:6.2f} {v[2]:6.2f}]"
                     for f, v in forces.forces.items()]
            print("  |  ".join(parts), end="\r")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print()
    finally:
        hand.disconnect()


if __name__ == "__main__":
    main()
