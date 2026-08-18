"""Nudge a single motor back and forth to confirm it answers and drives its tendon.

The motor is swept over a narrow window around the position it held at startup,
under the hand's configured current limit. This is a bring-up aid for a suspect
motor or tendon, not a way to pose the hand.

Usage:
    uv run python scripts/check_motor.py --motor-id 5
    uv run python scripts/check_motor.py CONFIG --wrist --span 0.5
"""

import argparse
import time

import numpy as np

from orca_core.constants import CONTROL_MODES, WRIST
from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    shutdown_hand,
)

STEP_RAD = 0.1
STEP_INTERVAL_S = 0.2

# Half-width of the sweep, in motor radians around the startup position.
# Matches manual_control's motor-space sliders: tight enough for a nudge.
DEFAULT_SPAN_RAD = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    add_hand_arguments(parser, feedback_flag=False)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--motor-id", type=int, help="Motor ID to sweep.")
    target.add_argument(
        "--wrist", action="store_true",
        help="Sweep the wrist motor named by the config's joint_to_motor_map.",
    )
    parser.add_argument(
        "--control-mode", choices=CONTROL_MODES, default=None,
        help="Control mode to apply. Defaults to the config's control_mode.",
    )
    parser.add_argument(
        "--span", type=float, default=DEFAULT_SPAN_RAD,
        help=f"Half-width of the sweep in motor radians (default {DEFAULT_SPAN_RAD}).",
    )
    parser.add_argument(
        "--reverse", action="store_true", help="Sweep down from the startup position first.",
    )
    return parser.parse_args()


def _resolve_motor_id(hand, args: argparse.Namespace) -> int:
    if args.wrist:
        motor_id = hand.config.joint_to_motor_map.get(WRIST)
        if motor_id is None:
            raise SystemExit(f"This hand has no '{WRIST}' joint in its joint_to_motor_map.")
        return motor_id
    if args.motor_id not in hand.config.motor_ids:
        raise SystemExit(
            f"Motor {args.motor_id} is not in this hand's motor_ids "
            f"({hand.config.motor_ids})."
        )
    return args.motor_id


def main() -> int:
    args = parse_args()

    # Motor-only: a running joint-feedback loop would fight the raw position
    # writes. This sweeps a raw motor position, never a calibrated joint one,
    # so calibration state has no bearing on it either.
    hand = create_hand_from_args(args, engage_feedback=False, quiet=True)
    try:
        motor_id = _resolve_motor_id(hand, args)
        connect_hand(hand)

        mode = args.control_mode or hand.config.control_mode
        hand.set_control_mode(mode, motor_ids=[motor_id])
        hand.set_max_current(hand.config.max_current)
        hand.enable_torque([motor_id])
        print(
            f"Motor {motor_id} in {mode} at "
            f"{hand.config.max_current} mA, span +/-{args.span:.2f} rad."
        )

        start = float(hand.get_motor_pos(as_dict=True)[motor_id])
        lower, upper = start - args.span, start + args.span
        target = start
        step = -STEP_RAD if args.reverse else STEP_RAD

        while True:
            target += step
            if not lower <= target <= upper:
                target = min(max(target, lower), upper)
                step = -step
            hand.write_motor_pos([motor_id], np.array([target]))
            measured = float(hand.get_motor_pos(as_dict=True)[motor_id])
            print(f"  measured {measured:+7.3f} rad, target {target:+7.3f} rad", end="\r")
            time.sleep(STEP_INTERVAL_S)
    except KeyboardInterrupt:
        print("\nStopping.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
