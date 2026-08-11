"""Cycle the hand between OPEN and CLOSE poses while monitoring motor temperatures."""

import argparse
import time

from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand_from_args, shutdown_hand

from orca_core.constants import NUM_STEPS, STEP_SIZE


TEMP_CHECK_INTERVAL = 2.0

# Pose targets as fractions of each joint's configured ROM.
OPEN_FRACTIONS = {
    "thumb_cmc": 0.70, "thumb_abd": 0.80, "thumb_mcp": 0.85, "thumb_dip": 0.80,
    "index_abd": 0.10, "middle_abd": 0.50, "ring_abd": 0.70, "pinky_abd": 0.85,
    "index_mcp": 0.15, "middle_mcp": 0.15, "ring_mcp": 0.15, "pinky_mcp": 0.15,
    "index_pip": 0.10, "middle_pip": 0.10, "ring_pip": 0.10, "pinky_pip": 0.10,
    "wrist": 0.30,
}
CLOSE_FRACTIONS = {
    "thumb_cmc": 0.35, "thumb_abd": 0.55, "thumb_mcp": 0.20, "thumb_dip": 0.85,
    "index_mcp": 0.85, "middle_mcp": 0.85, "ring_mcp": 0.85, "pinky_mcp": 0.85,
    "index_pip": 0.90, "middle_pip": 0.90, "ring_pip": 0.90, "pinky_pip": 0.90,
    "wrist": 0.55,
}


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


def finger_groups(joint_ids: list[str]) -> dict[str, list[str]]:
    """Group the config's joint names by finger prefix ({finger}_{type}; bare wrist)."""
    groups: dict[str, list[str]] = {}
    for joint in joint_ids:
        groups.setdefault(joint.split("_", 1)[0], []).append(joint)
    return groups


def print_temp_table(hand, temps: dict, max_temp: float) -> None:
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.config.motor_to_joint_dict

    grouped = {finger: [] for finger in finger_groups(hand.config.joint_ids)}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        grouped.setdefault(joint.split("_", 1)[0], []).append((joint, mid, t))

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
        hand.init_joints(force_calibrate=args.mock)

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
