import time
import numpy as np
import argparse
from orca_core import OrcaHand

AMPLITUDE = 10.0  # degrees
CYCLE_TIME = 2.0  # seconds per full cycle
STEP_TIME = 0.02


def main():
    parser = argparse.ArgumentParser(description="Lock all joints and wave the wrist on orca-touch.")
    parser.add_argument("model_path", type=str, nargs="?",
                        default="orca_core/models/orcahand-touch")
    parser.add_argument("--amplitude", type=float, default=AMPLITUDE,
                        help=f"Wrist wave amplitude in degrees (default: {AMPLITUDE})")
    parser.add_argument("--cycle_time", type=float, default=CYCLE_TIME,
                        help=f"Seconds per full wave cycle (default: {CYCLE_TIME})")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect.")
        return

    hand.init_joints()
    time.sleep(0.5)
    hand.disable_torque()

    print("Position the hand as desired, then press Enter to lock and start waving.")
    input()

    hand.enable_torque()
    hand.set_control_mode(hand.control_mode)
    hand.set_max_current(hand.max_current)
    time.sleep(0.1)

    # Lock all joints at current position
    current = hand.get_joint_pos(as_list=False)
    wrist_center = current.get("wrist", 0.0)

    print(f"All joints locked. Wrist waving ±{args.amplitude}° around {wrist_center:.1f}°")
    print("Press Ctrl+C to stop.\n")

    try:
        start_time = time.time()
        while True:
            t = time.time() - start_time
            wrist_pos = wrist_center + args.amplitude * np.sin(2 * np.pi * t / args.cycle_time)
            current["wrist"] = wrist_pos
            hand.set_joint_pos(current)
            time.sleep(STEP_TIME)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        hand.disable_torque()
        print("Torque disabled.")


if __name__ == "__main__":
    main()
