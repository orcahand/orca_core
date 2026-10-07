"""ORCA Hand full setup script.

Runs the complete tension -> calibrate -> test -> verify workflow.
Three rounds of tension+calibration with a 1-minute motion test in between.
"""

import argparse
import time
from pathlib import Path

from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    print_calibration_progress,
    shutdown_hand,
)
from orca_core.constants import NUM_STEPS, STEP_SIZE
from orca_core.demo_poses import load_demo_poses


DIVIDER = "=" * 60

_MAIN_DEMO_POSES = load_demo_poses()["main"].pose_fractions
OPEN_FRACTIONS = _MAIN_DEMO_POSES["open_hand"]
CLOSE_FRACTIONS = _MAIN_DEMO_POSES["power_grasp"]


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
        hand.calibrate(
            force_wrist=force_wrist, progress_callback=print_calibration_progress
        )
    except KeyboardInterrupt:
        print("\n  Calibration skipped.")


def run_neutral(hand, step_num):
    """Move to neutral position."""
    print_step(step_num, "NEUTRAL POSITION")
    print("  Moving hand to neutral position...")
    print("  Press Ctrl+C to skip.")
    hand.enable_torque()
    hand.set_control_mode(hand.config.control_mode)
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
    hand.set_control_mode(hand.config.control_mode)

    open_pos = hand.pose_from_fractions(OPEN_FRACTIONS)
    closed_pos = hand.pose_from_fractions(CLOSE_FRACTIONS)

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
    add_hand_arguments(parser, feedback_flag=False)
    args = parser.parse_args()

    print(DIVIDER)
    print("  ORCA HAND SETUP")
    print("  Full calibration and verification workflow")
    print("  Type 's' at any prompt or Ctrl+C to skip a step")
    print(DIVIDER)

    # The workflow runs tension and calibrate, neither of which can share the
    # motors with a live joint loop.
    hand = create_hand_from_args(args, engage_feedback=False)
    connect_hand(hand)
    print(f"  Model: {Path(hand.config.config_path).parent.name}")
    print(f"  Motor family: {hand.config.motor_type}")
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
