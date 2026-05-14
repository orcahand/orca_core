"""ORCA Hand full setup script.

Runs the complete tension -> calibrate -> test -> verify workflow.
Three rounds of tension+calibration with a 1-minute motion test in between.
"""

import argparse
import time

from common import connect_hand, create_hand, shutdown_hand
from orca_core import OrcaJointPositions
from orca_core.constants import NUM_STEPS, STEP_SIZE


DIVIDER = "=" * 60


def pose_from_fractions(hand, fractions: dict[str, float]) -> OrcaJointPositions:
    pose = dict(hand.config.neutral_position)
    for joint, fraction in fractions.items():
        if joint not in hand.config.joint_roms_dict:
            continue
        joint_min, joint_max = hand.config.joint_roms_dict[joint]
        pose[joint] = joint_min + fraction * (joint_max - joint_min)
    return OrcaJointPositions.from_dict(pose)


def wait_for_enter(msg="Press ENTER to continue..."):
    """Wait for user input. Returns True if the user chose to skip."""
    try:
        response = input(f"\n>>> {msg} ('s' to skip) ")
        return response.strip().lower() in ('s', 'skip')
    except KeyboardInterrupt:
        print()
        return True


def print_step(step_num, title):
    print(f"\n{DIVIDER}")
    print(f"  STEP {step_num}: {title}")
    print(DIVIDER)


def run_tension(hand, step_num, label):
    """Run tension in the foreground until the user interrupts with Ctrl+C."""
    print_step(step_num, f"TENSION — {label}")
    print("  Motors will move to set initial tension, then hold.")
    print("  Use the tensioning tool or pliers to turn the top spool clockwise.")
    print("  Do NOT overtension — just enough to remove slack.")
    if wait_for_enter("Press ENTER to begin tensioning, or 's' to skip..."):
        print("  Tension skipped.")
        return
    print("  Press Ctrl+C when tensioning is done.")
    try:
        hand.tension(move_motors=True, blocking=True)
    except KeyboardInterrupt:
        print("\n  Tension complete.")


def run_calibrate(hand, step_num, label, force_wrist=False):
    """Run calibration."""
    print_step(step_num, f"CALIBRATE — {label}")
    print("  Press Ctrl+C to skip.")
    if force_wrist:
        print("  Calibrating all joints including wrist...")
    else:
        print("  Calibrating finger joints (wrist already calibrated, skipping)...")
    try:
        hand.calibrate(force_wrist=force_wrist)
        print("  Calibration complete.")
    except KeyboardInterrupt:
        print("\n  Calibration skipped.")


def run_neutral(hand, step_num):
    """Move to neutral position."""
    print_step(step_num, "NEUTRAL POSITION")
    print("  Moving hand to neutral position...")
    print("  Press Ctrl+C to skip.")
    hand.enable_torque()
    hand.set_control_mode('current_based_position')
    try:
        hand.set_neutral_position()
        print("  Hand is in neutral position.")
    except KeyboardInterrupt:
        print("\n  Neutral position skipped.")


def run_motion_test(hand, step_num, duration=60):
    """Open/close the hand repeatedly for `duration` seconds with countdown."""
    print_step(step_num, f"MOTION TEST — {duration}s")
    print("  Opening and closing the hand to verify calibration.")
    print("  Watch for any issues with finger movement.")
    print("  Press Ctrl+C to skip.\n")

    hand.enable_torque()
    hand.set_control_mode('current_based_position')

    open_pos = pose_from_fractions(
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
            "wrist": 0.30,
        },
    )
    closed_pos = pose_from_fractions(
        hand,
        {
            "thumb_cmc": 0.35,
            "thumb_abd": 0.55,
            "thumb_mcp": 0.20,
            "thumb_dip": 0.85,
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

    try:
        start = time.time()
        cycle = 0
        while True:
            remaining = duration - (time.time() - start)
            if remaining <= 0:
                break

            if cycle % 2 == 0:
                print(f"  [{int(remaining):3d}s left]  OPEN")
                hand.set_joint_positions(open_pos, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            else:
                print(f"  [{int(remaining):3d}s left]  CLOSE")
                hand.set_joint_positions(closed_pos, num_steps=NUM_STEPS, step_size=STEP_SIZE)
            cycle += 1

            hold_end = min(time.time() + 2.0, start + duration)
            while time.time() < hold_end:
                time.sleep(0.1)

        print("  Motion test complete.")
    except KeyboardInterrupt:
        print("\n  Motion test skipped.")

    hand.set_neutral_position()


def main():
    parser = argparse.ArgumentParser(description="Full ORCA Hand setup workflow.")
    parser.add_argument(
        "config_path", type=str, nargs="?", default=None,
        help="Path to the hand config.yaml file"
    )
    args = parser.parse_args()

    print(DIVIDER)
    print("  ORCA HAND SETUP")
    print("  Full calibration and verification workflow")
    print("  Type 's' at any prompt or Ctrl+C to skip a step")
    print(DIVIDER)

    hand = create_hand(args.config_path, use_mock=False)
    connect_hand(hand)
    print("  Connected and ready.")

    try:
        # --- Round 1: Initial tension + calibration (with wrist) ---
        run_tension(hand, 1, "Initial tensioning")

        wait_for_enter("Place the hand in a neutral position, then press ENTER...")

        run_calibrate(hand, 2, "First calibration (with wrist)", force_wrist=True)

        # --- Round 2: Re-tension + calibration (without wrist) ---
        run_tension(hand, 3, "Second tensioning")

        run_neutral(hand, 4)
        run_calibrate(hand, 5, "Second calibration (fingers only)")

        # --- Motion test ---
        run_motion_test(hand, 6, duration=60)

        # --- Round 3: Final tension + calibration ---
        run_tension(hand, 7, "Final tensioning")

        run_neutral(hand, 8)
        run_calibrate(hand, 9, "Final calibration (fingers only)")

        run_neutral(hand, 10)

        print(f"\n{DIVIDER}")
        print("  Done. Have fun playing with ORCA!")
        print(DIVIDER)

    except KeyboardInterrupt:
        print("\n\n  Setup interrupted by user.")
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    main()
