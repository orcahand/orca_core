

import time
import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand
from orca_core import OrcaJointPositions
from orca_core.constants import NUM_STEPS, STEP_SIZE

# Max operating temp (°C) — conservative across XC330 (70°C) and XC430 (72°C)
MAX_TEMP = 70
TEMP_CHECK_INTERVAL = 2.0


RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"


def temp_color(pct):
    if pct >= 90:
        return RED
    elif pct >= 70:
        return YELLOW
    return GREEN


def print_temp_table(hand, temps):
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.config.motor_to_joint_dict

    # Group motors by finger
    fingers = ['thumb', 'index', 'middle', 'ring', 'pinky', 'wrist']
    grouped = {f: [] for f in fingers}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        finger = joint.split('_')[0]
        if finger in grouped:
            grouped[finger].append((joint, mid, t))

    # Clear screen and move cursor to top
    print("\033[2J\033[H", end="")

    print(f"{BOLD}  ORCA Hand Temperature Monitor{RST}")
    print(f"  {DIM}Max operating temp: {MAX_TEMP}°C{RST}\n")

    # Header
    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")

    for finger in fingers:
        joints = grouped[finger]
        if not joints:
            continue
        for joint, mid, t in joints:
            pct = t / MAX_TEMP * 100
            c = temp_color(pct)
            bar_len = int(pct / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}")

    print(f"  {'─' * 48}")
    max_t = max(temps.values())
    max_pct = max_t / MAX_TEMP * 100
    c = temp_color(max_pct)
    print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


def pose_from_fractions(hand, fractions: dict[str, float]) -> OrcaJointPositions:
    pose = dict(hand.config.neutral_position)
    for joint, fraction in fractions.items():
        if joint not in hand.config.joint_roms_dict:
            continue
        joint_min, joint_max = hand.config.joint_roms_dict[joint]
        pose[joint] = joint_min + fraction * (joint_max - joint_min)
    return OrcaJointPositions.from_dict(pose)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a looping temperature monitor plus open/close hand demo."
    )
    add_hand_arguments(parser)
    parser.add_argument("--cycles", type=int, default=1, help="Number of open/close cycles. Use 0 for infinite.")
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.init_joints(force_calibrate=args.mock)

        open_pose = pose_from_fractions(
            hand,
            {
                "thumb_cmc": 0.70,
                "thumb_abd": 0.80,
                "thumb_mcp": 0.85,
                "thumb_dip": 0.80,
                "index_abd": 0.10,
                "middle_abd": 0.50,
                "ring_abd": 0.70,
                "pinky_abd": 0.85,
                "index_mcp": 0.15,
                "middle_mcp": 0.15,
                "ring_mcp": 0.15,
                "pinky_mcp": 0.15,
                "index_pip": 0.10,
                "middle_pip": 0.10,
                "ring_pip": 0.10,
                "pinky_pip": 0.10,
                "wrist": 0.25,
            },
        )
        close_pose = pose_from_fractions(
            hand,
            {
                "thumb_cmc": 0.35,
                "thumb_abd": 0.55,
                "thumb_mcp": 0.20,
                "thumb_dip": 0.90,
                "index_mcp": 0.85,
                "middle_mcp": 0.85,
                "ring_mcp": 0.85,
                "pinky_mcp": 0.85,
                "index_pip": 0.90,
                "middle_pip": 0.90,
                "ring_pip": 0.90,
                "pinky_pip": 0.90,
                "wrist": 0.55,
            },
        )

        cycles_remaining = args.cycles
        last_temp_check = 0.0
        while cycles_remaining != 0:
            now = time.monotonic()
            if now - last_temp_check >= TEMP_CHECK_INTERVAL:
                last_temp_check = now
                print_temp_table(hand, hand.get_motor_temp(as_dict=True))

            hand.set_joint_positions(open_pose, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            time.sleep(2.0)
            hand.set_joint_positions(close_pose, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            time.sleep(2.0)

            if cycles_remaining > 0:
                cycles_remaining -= 1

        return 0
    except KeyboardInterrupt:
        print("\nDemo interrupted.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
