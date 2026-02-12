from orca_core import OrcaHand
import time  
import argparse  


def main(): 
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()
    hand = OrcaHand(args.model_path)

    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    while True:
        joint_dict = {
            # All PIPs at 90
            "index_pip": -10,
            "middle_pip": -10,
            "ring_pip": -10,
            "pinky_pip": -10,
            "thumb_dip": -30,
            # All MCPs at 45
            "thumb_mcp": -40,
            "index_mcp": -40,
            "middle_mcp": -40,
            "ring_mcp": -40,
            "pinky_mcp": -40,
            # All abductions at 0
            "thumb_abd": 0,
            "index_abd": 0,
            "middle_abd": 0,
            "ring_abd": 0,
            "pinky_abd": 0,
        }

        hand.set_joint_pos(joint_dict, num_steps=25, step_size=0.001)

        time.sleep(2)

        joint_dict = {
            "thumb_mcp": 45,
            "index_mcp": 45,
            "middle_mcp": 45,
            "ring_mcp": 45,
            "pinky_mcp": 45,
            "thumb_dip": 90,
            "index_pip": 90,
            "middle_pip": 90,
            "ring_pip": 90,
            "pinky_pip": 90,
            "thumb_abd": 40
        }
        hand.set_joint_pos(joint_dict, num_steps=25, step_size=0.001)
        time.sleep(2)


    hand.disable_torque()

    hand.disconnect()

if __name__ == "__main__":  # Added main execution block
    main()