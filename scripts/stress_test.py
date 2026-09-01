"""Stress the thumb MCP and DIP while the rest of the hand holds fully open."""

import argparse
import time

from orca_core import OrcaJointPositions
from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand_from_args, shutdown_hand

from orca_core.constants import NUM_STEPS, STEP_SIZE


TEMP_CHECK_INTERVAL = 2.0

ABDUCTION_SUFFIX = "_abd"

# The two joints under test. They sweep their full configured ROM every cycle;
# everything else is parked in HOLD_FRACTIONS and never moves.
CYCLE_JOINTS = ("thumb_mcp", "thumb_dip")
CYCLE_LOW = 0.0
CYCLE_HIGH = 1.0

# Static hold pose, as fractions of each joint's configured ROM: every other
# joint is driven fully outwards to its ROM limit. The outward end differs per
# joint - flexion joints extend towards 0.0, the thumb CMC/ABD towards 1.0, and
# the abduction joints fan apart (index towards the thumb, ring and pinky away
# from it), matching the `fan_out` pose in orca_core/data/demo_poses.yaml.
HOLD_FRACTIONS = {
    "thumb_cmc": 1.0, "thumb_abd": 1.0,
    "index_abd": 0.0, "middle_abd": 0.5, "ring_abd": 1.0, "pinky_abd": 1.0,
    "index_mcp": 0.0, "middle_mcp": 0.0, "ring_mcp": 0.0, "pinky_mcp": 0.0,
    "index_pip": 0.0, "middle_pip": 0.0, "ring_pip": 0.0, "pinky_pip": 0.0,
    "wrist": 0.0,
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


def is_abduction(joint: str) -> bool:
    return joint.endswith(ABDUCTION_SUFFIX)


def cycle_poses(hand, freeze_abduction: bool) -> tuple[OrcaJointPositions, OrcaJointPositions]:
    """Build the two endpoint poses of the cycle.

    Both poses park every joint at HOLD_FRACTIONS and differ only in the
    CYCLE_JOINTS, which sit at opposite ends of their ROM. When
    *freeze_abduction* is set the abduction joints are left out of the commanded
    pose entirely, so they stay wherever the hand already holds them.
    """
    def pose(cycle_fraction: float) -> OrcaJointPositions:
        fractions = {**HOLD_FRACTIONS, **{joint: cycle_fraction for joint in CYCLE_JOINTS}}
        if freeze_abduction:
            fractions = {j: f for j, f in fractions.items() if not is_abduction(j)}

        built = hand.pose_from_fractions(fractions)
        if not freeze_abduction:
            return built
        return OrcaJointPositions.from_dict(
            {j: v for j, v in built.as_dict().items() if not is_abduction(j)}
        )

    return pose(CYCLE_LOW), pose(CYCLE_HIGH)


def print_temp_table(hand, temps: dict, max_temp: float, freeze_abduction: bool) -> None:
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.config.motor_to_joint_dict

    grouped = {finger: [] for finger in finger_groups(hand.config.joint_ids)}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        grouped.setdefault(joint.split("_", 1)[0], []).append((joint, mid, t))

    print("\033[2J\033[H", end="")  # clear screen, cursor home

    print(f"{BOLD}  ORCA Hand Temperature Monitor{RST}")
    print(f"  {DIM}Max operating temp: {max_temp:.0f}°C{RST}")
    print(f"  {DIM}Cycling: {', '.join(CYCLE_JOINTS)} - all other joints held fully open{RST}")
    if freeze_abduction:
        print(f"  {DIM}Abduction frozen: *{ABDUCTION_SUFFIX} joints are not commanded{RST}")
    print()
    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")

    for finger in grouped:
        for joint, mid, t in grouped[finger]:
            pct = t / max_temp * 100
            c = temp_color(pct)
            bar_len = int(min(pct, 100) / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            marker = f" {BOLD}<{RST}" if joint in CYCLE_JOINTS else ""
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}{marker}")

    print(f"  {'─' * 48}")
    if temps:
        max_t = max(temps.values())
        max_pct = max_t / max_temp * 100
        c = temp_color(max_pct)
        print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Thumb MCP/DIP cycle stress test with temperature monitoring."
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
    parser.add_argument(
        "--freeze-abduction", action="store_true",
        help="Leave the abduction joints uncommanded instead of fanning them out."
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)
        hand.init_joints()

        max_temp = hand.motor_client.max_operating_temp_c
        extended_pos, flexed_pos = cycle_poses(hand, args.freeze_abduction)

        last_temp_check = 0.0
        try:
            while True:
                now = time.monotonic()
                if now - last_temp_check >= TEMP_CHECK_INTERVAL:
                    last_temp_check = now
                    temps = hand.get_motor_temp(as_dict=True)
                    print_temp_table(hand, temps, max_temp, args.freeze_abduction)
                    if temps and max(temps.values()) >= max_temp:
                        print(f"{RED}Motor temperature reached {max_temp:.0f}°C — aborting.{RST}")
                        break

                hand.set_joint_positions(
                    extended_pos, num_steps=args.num_steps, step_size=args.step_size
                )
                if args.hold:
                    time.sleep(args.hold)

                hand.set_joint_positions(
                    flexed_pos, num_steps=args.num_steps, step_size=args.step_size
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
