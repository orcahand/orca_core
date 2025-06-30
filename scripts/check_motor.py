from orca_core.hardware.dynamixel_client import DynamixelClient
import time
import argparse # Added import

def main(): # Added main function
    parser = argparse.ArgumentParser(description="Check a motor connected to the Dynamixel client.")
    parser.add_argument("--port", type=str, default="/dev/tty.usbserial-FT9MISJT", help="The port to connect to the Dynamixel client.")
    parser.add_argument("--baudrate", type=int, default=3000000, help="The baudrate for the Dynamixel client.")
    parser.add_argument("--motor_id", type=int, default=2, help="The ID of the motor to check.")
    parser.add_argument("--wrist", action="store_true", help="Set if checking a wrist motor (uses position control mode 3).")
    parser.add_argument("--reverse", action="store_true", help="If set, subtracts 0.1 from position, otherwise adds 0.1.")
    
    args = parser.parse_args()

    if args.motor_id == 0 or args.motor_id == 17:
        if not args.wrist:
            print(f"Motor ID {args.motor_id} is often used for wrist motors.")
            print("Consider using the --wrist flag if this is a wrist motor to set operating mode to 3 (position control).")
        elif args.wrist and (args.motor_id != 0 and args.motor_id != 17):
             print(f"Warning: --wrist flag is set, but motor_id ({args.motor_id}) is not a typical wrist ID (0 or 17). Ensure this is intended.")


    dxl_client = DynamixelClient([args.motor_id], args.port, args.baudrate)
    dxl_client.connect()

    operating_mode = 5 
    if args.wrist:
        operating_mode = 3 # Position control mode for wrist
        print(f"Operating in position control mode (3) for wrist motor ID {args.motor_id}.")
    else:
        print(f"Operating in current-based position mode (5) for motor ID {args.motor_id}.")

    dxl_client.set_operating_mode([args.motor_id], operating_mode)

    dxl_client.set_torque_enabled([args.motor_id], True)

    while True:
        pos = dxl_client.read_pos_vel_cur()[0]
        increment = -0.1 if args.reverse else 0.1
        new_pos = pos + increment
        dxl_client.write_desired_pos([args.motor_id], new_pos)
        print(f"Current Position: {pos}, Target Position: {new_pos}, Operating Mode: {operating_mode}") # Formatted output
        time.sleep(0.2)

if __name__ == "__main__":
    main()
