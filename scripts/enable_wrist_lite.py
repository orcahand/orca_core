import argparse
from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(description="Enable torque for the lite wrist motor.")
    parser.add_argument("model_path", type=str, nargs="?",
                        default="orca_core/models/orcahand-lite")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect.")
        return

    wrist_motor = hand.joint_to_motor_map["wrist"]
    hand.enable_torque(motor_ids=[wrist_motor])
    hand.set_control_mode('current_based_position')
    hand.set_max_current(hand.max_current)
    print(f"Wrist motor {wrist_motor} torque enabled.")
    input("Press Enter to disable torque and exit.")
    hand.disable_torque([wrist_motor])

if __name__ == "__main__":
    main()
