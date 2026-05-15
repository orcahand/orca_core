"""Cycle the hand between OPEN and CLOSE poses while monitoring motor temperatures."""

import argparse
import time

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand

from orca_core.constants import NUM_STEPS, STEP_SIZE


MAX_TEMP = 70  # °C — conservative across Dynamixel XC330/430 and Feetech STS3215
TEMP_CHECK_INTERVAL = 2.0


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


def print_temp_table(hand, temps: dict) -> None:
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.config.motor_to_joint_dict

    fingers = ["thumb", "index", "middle", "ring", "pinky", "wrist"]
    grouped = {f: [] for f in fingers}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        finger = joint.split("_")[0]
        if finger in grouped:
            grouped[finger].append((joint, mid, t))

    print("\033[2J\033[H", end="")  # clear screen, cursor home

    print(f"{BOLD}  ORCA Hand Temperature Monitor{RST}")
    print(f"  {DIM}Max operating temp: {MAX_TEMP}°C{RST}\n")
    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")

    for finger in fingers:
        for joint, mid, t in grouped[finger]:
            pct = t / MAX_TEMP * 100
            c = temp_color(pct)
            bar_len = int(min(pct, 100) / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}")

    print(f"  {'─' * 48}")
    if temps:
        max_t = max(temps.values())
        max_pct = max_t / MAX_TEMP * 100
        c = temp_color(max_pct)
        print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


JOINT_OPEN = {
    "index_abd": -30,
    "middle_abd": 0,
    "ring_abd": 10,
    "pinky_abd": 25,
    "thumb_abd": -20,
    "index_mcp": -40,
    "middle_mcp": -40,
    "ring_mcp": -40,
    "pinky_mcp": -40,
    "thumb_mcp": 45,
    "index_pip": -10,
    "middle_pip": -10,
    "ring_pip": -10,
    "pinky_pip": -10,
    "thumb_dip": 90,
    "thumb_cmc": 40,
    "wrist": -25,
}

JOINT_CLOSE = {
    "index_abd": 15,
    "middle_abd": 15,
    "ring_abd": -15,
    "pinky_abd": -15,
    "thumb_abd": 55,
    "index_mcp": 45,
    "middle_mcp": 45,
    "ring_mcp": 45,
    "pinky_mcp": 45,
    "thumb_mcp": -40,
    "index_pip": 90,
    "middle_pip": 90,
    "ring_pip": 90,
    "pinky_pip": 90,
    "thumb_dip": -30,
    "thumb_cmc": -50,
    "wrist": 10,
}


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

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

        last_temp_check = 0.0
        try:
            while True:
                now = time.monotonic()
                if now - last_temp_check >= TEMP_CHECK_INTERVAL:
                    last_temp_check = now
                    temps = hand.get_motor_temp(as_dict=True)
                    print_temp_table(hand, temps)
                    if temps and max(temps.values()) >= MAX_TEMP:
                        print(f"{RED}Motor temperature reached {MAX_TEMP}°C — aborting.{RST}")
                        break

                hand.set_joint_positions(
                    JOINT_OPEN, num_steps=args.num_steps, step_size=args.step_size
                )
                if args.hold:
                    time.sleep(args.hold)

                hand.set_joint_positions(
                    JOINT_CLOSE, num_steps=args.num_steps, step_size=args.step_size
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
