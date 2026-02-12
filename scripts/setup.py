"""ORCA Hand full setup script.

Runs the complete tension → calibrate → test → verify workflow.
Three rounds of tension+calibration with a 1-minute motion test in between.
"""

import argparse
import sys
import time
from orca_core import OrcaHand


DIVIDER = "=" * 60


def wait_for_enter(msg="Press ENTER to continue..."):
    input(f"\n>>> {msg}")


def print_step(step_num, title):
    print(f"\n{DIVIDER}")
    print(f"  STEP {step_num}: {title}")
    print(DIVIDER)


def run_tension(hand, step_num, label):
    """Run tension with move_motors in background, wait for user to press Enter."""
    print_step(step_num, f"TENSION — {label}")
    print("  Motors will move to set initial tension, then hold.")
    print("  Adjust tendons while motors are holding.")
    hand.tension(move_motors=True, blocking=False)
    wait_for_enter("Press ENTER when tensioning is done...")
    hand.stop_task()
    print("  Tension complete.")


def run_calibrate(hand, step_num, label, force_wrist=False):
    """Run calibration."""
    print_step(step_num, f"CALIBRATE — {label}")
    if force_wrist:
        print("  Calibrating all joints including wrist...")
    else:
        print("  Calibrating finger joints (wrist already calibrated, skipping)...")
    hand.calibrate(force_wrist=force_wrist)
    print("  Calibration complete.")


def run_neutral(hand, step_num):
    """Move to neutral position."""
    print_step(step_num, "NEUTRAL POSITION")
    print("  Moving hand to neutral position...")
    hand.enable_torque()
    hand.set_control_mode('current_based_position')
    hand.set_neutral_position()
    print("  Hand is in neutral position.")


def run_motion_test(hand, step_num, duration=60):
    """Open/close the hand repeatedly for `duration` seconds with countdown."""
    print_step(step_num, f"MOTION TEST — {duration}s")
    print("  Opening and closing the hand to verify calibration.")
    print("  Watch for any issues with finger movement.\n")

    hand.enable_torque()
    hand.set_control_mode('current_based_position')

    open_pos = {
        "index_pip": -10, "middle_pip": -10, "ring_pip": -10, "pinky_pip": -10,
        "thumb_dip": -30,
        "thumb_mcp": -40, "index_mcp": -40, "middle_mcp": -40, "ring_mcp": -40, "pinky_mcp": -40,
        "thumb_abd": 0, "index_abd": 0, "middle_abd": 0, "ring_abd": 0, "pinky_abd": 0,
    }
    closed_pos = {
        "thumb_mcp": 45, "index_mcp": 45, "middle_mcp": 45, "ring_mcp": 45, "pinky_mcp": 45,
        "thumb_dip": 90, "index_pip": 90, "middle_pip": 90, "ring_pip": 90, "pinky_pip": 90,
        "thumb_abd": 40,
    }

    start = time.time()
    cycle = 0
    while True:
        remaining = duration - (time.time() - start)
        if remaining <= 0:
            break

        if cycle % 2 == 0:
            print(f"  [{int(remaining):3d}s left]  OPEN")
            hand.set_joint_pos(open_pos, num_steps=25, step_size=0.001)
        else:
            print(f"  [{int(remaining):3d}s left]  CLOSE")
            hand.set_joint_pos(closed_pos, num_steps=25, step_size=0.001)
        cycle += 1

        hold_end = min(time.time() + 2.0, start + duration)
        while time.time() < hold_end:
            time.sleep(0.1)

    hand.set_neutral_position()
    print("  Motion test complete.")


def main():
    parser = argparse.ArgumentParser(description="Full ORCA Hand setup workflow.")
    parser.add_argument(
        "model_path", type=str, nargs="?", default=None,
        help="Path to the hand model directory"
    )
    args = parser.parse_args()

    print(DIVIDER)
    print("  ORCA HAND SETUP")
    print("  Full calibration and verification workflow")
    print(DIVIDER)

    hand = OrcaHand(args.model_path)
    success, message = hand.connect()
    if not success:
        print(f"Failed to connect: {message}")
        sys.exit(1)
    print(f"  Connected: {message}")

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

        run_calibrate(hand, 8, "Final calibration (fingers only)")

        run_neutral(hand, 9)

        print(f"\n{DIVIDER}")
        print("  Done. Have fun playing with ORCA!")
        print(DIVIDER)

    except KeyboardInterrupt:
        print("\n\n  Setup interrupted by user.")
    finally:
        hand.disconnect()


if __name__ == "__main__":
    main()
