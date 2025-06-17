import argparse
from orca_core import OrcaHand
import time
import numpy as np

def main():
    parser = argparse.ArgumentParser(
        description="Enable torque and hold tension on the ORCA Hand. "
                    "Specify the path to the orcahand model folder."
    )
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    parser.add_argument('--move_motors', action='store_true', help='If set, move motors 1-16 continuously positively for 3 seconds with calibration current.')

    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()
    print("Torque enabled.")
    calibration_current = hand.calib_current

    if args.move_motors:
        motor_ids_to_move = [i for i in range(1, 17) if i in hand.motor_ids]
        if not motor_ids_to_move:
            print("No motors in the range 1-16 found in the hand configuration.")
        else:
            print(f"Moving motors {motor_ids_to_move} continuously for 3 seconds...")
            hand.set_control_mode('current_based_position', motor_ids=motor_ids_to_move)
            
            # Apply calibration current to all motors; only targeted ones will move.
            hand.set_max_current(calibration_current) 
            print(f"Using calibration current: {calibration_current}mA for targeted motors.")

            loop_duration_sec = 5
            increment_per_step = 0.1

            motor_pos_increments_dict = {motor_id: increment_per_step for motor_id in motor_ids_to_move}

            start_time = time.time()
            while (time.time() - start_time) < loop_duration_sec:
                hand._set_motor_pos(motor_pos_increments_dict, rel_to_current=True)
                time_to_sleep = 0.1
                if time_to_sleep > 0:
                    time.sleep(time_to_sleep)
            
    hand.set_max_current(calibration_current) 
    hand.disable_torque()
    print("Disable tension momentarily to release unwatned tension.")
    time.sleep(1)
    hand.enable_torque()
    print("Holding tension. Press Ctrl+C to exit.")
    try:
        while True:
            # Keep torque enabled; motors will hold their last commanded position.
            time.sleep(0.1) 
    except KeyboardInterrupt:
        print("\nExiting. Disabling torque.")
        hand.disable_torque()
    finally:
        if hand.is_connected():
            print("Is hand connected?", hand.is_connected())
            print("Disconnecting from hand.")
            hand.disconnect()

if __name__ == "__main__":
    main()