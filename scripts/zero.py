import argparse
import sys
import os
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from orca_core import OrcaHand

def main():
    parser = argparse.ArgumentParser(description='Move OrcaHand joints to zero position.')
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')

    
    args = parser.parse_args()

    try:
        hand = OrcaHand(model_path=args.model_path)
            
        success, message = hand.connect()
        if not success:
            print(f"Failed to connect: {message}")
            return 1
            
        print("Connected to hand successfully")
        
        hand.enable_torque()
        print("Torque enabled")
        print("Available motor IDs:", hand.motor_ids)
        print("Moving all joints to 0 position...")
        hand.set_joint_pos({joint: 0 for joint in hand.joint_ids}, num_steps=25, step_size=0.001) # Setting pos with steps to avoid too fast movement
        print("Reached 0 position of all joints")
        time.sleep(3)  # Wait for the hand to stabilize
        hand.disable_torque()
        hand.disconnect()
        print("Disconnected from hand")
        
        return 0
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 