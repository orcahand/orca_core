"""Cycle the hand between OPEN and CLOSE poses while monitoring motor temperatures."""

import argparse
import time

from orca_core.utils import enable_ansi_escapes
from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    group_joints_by_finger,
    shutdown_hand,
)

from orca_core.constants import NUM_STEPS, STEP_SIZE
from orca_core.demo_poses import load_demo_poses


TEMP_CHECK_INTERVAL = 2.0

_MAIN_DEMO_POSES = load_demo_poses()["main"].pose_fractions
OPEN_FRACTIONS = _MAIN_DEMO_POSES["open_hand"]
CLOSE_FRACTIONS = _MAIN_DEMO_POSES["power_grasp"]


RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"


def temp_color(pct: float) -> str:
    if pct >= 90:
        return RED
    if pct >= 70:
        return YELLOW
    return GREEN


def print_temp_table(hand, temps: dict, max_temp: float) -> None:
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.config.motor_to_joint_dict

    by_finger = group_joints_by_finger(hand.config.joint_ids)
    finger_of = {joint: finger for finger, joints in by_finger.items() for joint in joints}
    grouped = {finger: [] for finger in by_finger}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        grouped.setdefault(finger_of.get(joint, "motor"), []).append((joint, mid, t))

    print("\033[2J\033[H", end="")  # clear screen, cursor home

    print(f"{BOLD}  ORCA Hand Temperature Monitor{RST}")
    print(f"  {DIM}Max operating temp: {max_temp:.0f}°C{RST}\n")
    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")

    for finger in grouped:
        for joint, mid, t in grouped[finger]:
            pct = t / max_temp * 100
            c = temp_color(pct)
            bar_len = int(min(pct, 100) / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}")

    print(f"  {'─' * 48}")
    if temps:
        max_t = max(temps.values())
        max_pct = max_t / max_temp * 100
        c = temp_color(max_pct)
        print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


def main() -> int:
    enable_ansi_escapes()
    parser = argparse.ArgumentParser(
        description="Open/close cycle stress test with temperature monitoring."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--num-steps", type=int, default=NUM_STEPS,
        help=f"Interpolation steps per move (default {NUM_STEPS})."
    )
    parser.add_argument(
        "--step-size", type=float, default=STEP_SIZE,
        help=f"Sleep between interpolation steps in seconds (default {STEP_SIZE})."
    )
    parser.add_argument(
        "--hold", type=float, default=2.0,
        help="Seconds to hold each pose AFTER motion completes (default 2)."
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)
        hand.init_joints()

        max_temp = hand.motor_client.max_operating_temp_c
        open_pos = hand.pose_from_fractions(OPEN_FRACTIONS)
        close_pos = hand.pose_from_fractions(CLOSE_FRACTIONS)

        last_temp_check = 0.0
        try:
            while True:
                now = time.monotonic()
                if now - last_temp_check >= TEMP_CHECK_INTERVAL:
                    last_temp_check = now
                    temps = hand.get_motor_temp(as_dict=True)
                    print_temp_table(hand, temps, max_temp)
                    if temps and max(temps.values()) >= max_temp:
                        print(f"{RED}Motor temperature reached {max_temp:.0f}°C — aborting.{RST}")
                        break

                hand.set_joint_positions(
                    open_pos, num_steps=args.num_steps, step_size=args.step_size
                )
                if args.hold:
                    time.sleep(args.hold)

                hand.set_joint_positions(
                    close_pos, num_steps=args.num_steps, step_size=args.step_size
                )
                if args.hold:
                    time.sleep(args.hold)
        except KeyboardInterrupt:
            print("\nInterrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
