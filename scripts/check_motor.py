from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="Check a motor (Dynamixel or Feetech).")
    parser.add_argument("--port", type=str, default="/dev/tty.usbserial-FT9MISJT", help="The port to connect to the motor.")
    parser.add_argument("--baudrate", type=int, default=None, help="The baudrate for communication. Defaults to 3000000 for dynamixel, 1000000 for feetech.")
    parser.add_argument("--motor_id", type=int, default=1, help="The ID of the motor to check.")
    parser.add_argument("--motor_type", type=str, default="dynamixel", choices=["dynamixel", "feetech"], help="Motor type: dynamixel or feetech.")
    parser.add_argument("--wrist", action="store_true", help="Set if checking a wrist motor (uses position control mode 3).")
    parser.add_argument("--reverse", action="store_true", help="If set, subtracts 0.1 from position, otherwise adds 0.1.")

    args = parser.parse_args()

    if args.baudrate is None:
        args.baudrate = 1000000 if args.motor_type == "feetech" else 3000000

    if args.motor_type == "dynamixel":
        if args.motor_id == 0 or args.motor_id == 17:
            if not args.wrist:
                print(f"Motor ID {args.motor_id} is often used for wrist motors.")
                print("Consider using the --wrist flag if this is a wrist motor to set operating mode to 3 (position control).")
            elif args.wrist and (args.motor_id != 0 and args.motor_id != 17):
                print(f"Warning: --wrist flag is set, but motor_id ({args.motor_id}) is not a typical wrist ID (0 or 17). Ensure this is intended.")

    if args.motor_type == "feetech":
        client = FeetechClient([args.motor_id], args.port, args.baudrate)
    else:
        client = DynamixelClient([args.motor_id], args.port, args.baudrate)

    client.connect()

    if args.motor_type == "dynamixel":
        operating_mode = 5
        if args.wrist:
            operating_mode = 3
            print(f"Operating in position control mode (3) for wrist motor ID {args.motor_id}.")
        else:
            print(f"Operating in current-based position mode (5) for motor ID {args.motor_id}.")
        client.set_operating_mode([args.motor_id], operating_mode)
    else:
        print(f"Feetech motor ID {args.motor_id} - using servo mode (position control).")

    client.set_torque_enabled([args.motor_id], True)

    if args.motor_type == "feetech":
        # Feetech range is 0-4095 raw (0 to 2*pi rad). Oscillate around center.
        import numpy as np
        center = np.pi  # middle of range
        amplitude = 0.5  # ~29 degrees each way
        pos = client.read_pos_vel_cur()[0][0]
        print(f"Initial position: {pos:.3f} rad")
        print(f"Oscillating ±{np.degrees(amplitude):.0f}° around center ({center:.2f} rad)")
        step = 0
        while True:
            target = center + amplitude * np.sin(step * 0.1)
            client.write_desired_pos([args.motor_id], [target])
            pos = client.read_pos_vel_cur()[0][0]
            print(f"Position: {pos:.3f} rad, Target: {target:.3f} rad")
            step += 1
            time.sleep(0.2)
    else:
        while True:
            pos = client.read_pos_vel_cur()[0][0]
            increment = -0.1 if args.reverse else 0.1
            new_pos = pos + increment
            client.write_desired_pos([args.motor_id], [new_pos])
            print(f"Current Position: {pos:.3f}, Target Position: {new_pos:.3f}")
            time.sleep(0.2)

if __name__ == "__main__":
    main()
