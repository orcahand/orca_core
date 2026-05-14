"""Trigger overload on a chosen finger by driving into the hardstop.

Uses position control (no current limit) so stalling against the mechanical
limit triggers overload quickly. The motor recovers automatically and keeps
pushing — no human intervention required.

Usage:
    python scripts/overload_demo.py              # defaults to index finger
    python scripts/overload_demo.py --finger middle
    python scripts/overload_demo.py --finger thumb --duration 20
"""
import argparse
import os
import sys
import time
import logging
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from orca_core import OrcaHand

FINGER_JOINTS = {
    'thumb':  ['thumb_mcp', 'thumb_dip'],
    'index':  ['index_mcp', 'index_pip'],
    'middle': ['middle_mcp', 'middle_pip'],
    'ring':   ['ring_mcp', 'ring_pip'],
    'pinky':  ['pinky_mcp', 'pinky_pip'],
}

PAST_LIMIT_RAD = np.deg2rad(30)

def msg(text):
    os.write(2, (text + '\n').encode())

def main():
    parser = argparse.ArgumentParser(description="Overload recovery demo.")
    parser.add_argument("--finger", "-f", default="index",
                        choices=FINGER_JOINTS.keys(),
                        help="Finger to test (default: index)")
    parser.add_argument("--duration", "-d", type=float, default=15.0,
                        help="Hold duration in seconds (default: 15)")
    parser.add_argument("config_path", type=str, nargs="?", default=None,
                        help="Path to the hand config.yaml file")
    args = parser.parse_args()

    logging.basicConfig(level=logging.WARNING, format='%(asctime)s %(levelname)s %(message)s')

    hand = OrcaHand(config_path=args.config_path)
    start_positions = None

    success, message = hand.connect()
    if not success:
        msg(f"ERROR: Could not connect to hand: {message}")
        return 1

    try:
        if not hand.calibrated:
            msg("ERROR: Hand must be calibrated so motor limits are known.")
            return 1

        joints = FINGER_JOINTS[args.finger]
        motor_ids = [hand.config.joint_to_motor_map[j] for j in joints]

        # Target past the hardstop (lower limit minus extra) to guarantee a stall
        targets = {}
        for mid in motor_ids:
            lo, hi = hand.motor_limits_dict[mid]
            targets[mid] = lo - PAST_LIMIT_RAD

        msg(f"\n  Finger: {args.finger}")
        msg(f"  Joints: {joints}")
        msg(f"  Motors: {motor_ids}")
        msg(f"  Duration: {args.duration}s")
        msg(f"\n  Driving into hardstop to trigger overload...")
        msg(f"  Motors will recover automatically.\n")

        hand.enable_torque()
        hand.set_control_mode('position', motor_ids=motor_ids)
        start_positions = hand.get_motor_pos(as_dict=True)

        hand._set_motor_pos(targets)

        last_print = 0
        start = time.time()
        while time.time() - start < args.duration:
            elapsed = time.time() - start
            hand._set_motor_pos(targets)

            now_sec = int(elapsed)
            if now_sec > last_print and now_sec % 2 == 0:
                last_print = now_sec
                msg(f"  [{now_sec:3d}s]")

            time.sleep(0.01)
        return 0
    except KeyboardInterrupt:
        msg("\nInterrupted. Returning to a safe state...")
        return 0
    finally:
        if start_positions is not None:
            try:
                msg("\nReturning to start...")
                hand._set_motor_pos(start_positions)
                time.sleep(1)
            except Exception as exc:
                msg(f"WARNING: Failed to restore start positions: {exc}")
        try:
            msg("Disabling torque.")
            hand.disable_torque()
        except Exception as exc:
            msg(f"WARNING: Failed to disable torque: {exc}")
        try:
            hand.disconnect()
        except Exception as exc:
            msg(f"WARNING: Failed to disconnect cleanly: {exc}")

if __name__ == "__main__":
    raise SystemExit(main())
