from orca_core.hardware.feetech_client import FeetechClient
from orca_core.hardware.dynamixel_client import DynamixelClient
import argparse

def main():
    parser = argparse.ArgumentParser(description="Scan for connected motors.")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port.")
    parser.add_argument("--motor_type", type=str, default="feetech", choices=["dynamixel", "feetech"], help="Motor type.")
    parser.add_argument("--id_range", type=int, nargs=2, default=[0, 20], help="ID range to scan (start end).")
    args = parser.parse_args()

    print(f"Scanning for {args.motor_type} motors on {args.port}")
    print(f"ID range: {args.id_range[0]}-{args.id_range[1]}")
    print("-" * 40)

    if args.motor_type == "feetech":
        client = FeetechClient([], port=args.port)
        baud_rates = [1000000, 500000, 250000, 128000, 115200, 57600, 38400]
    else:
        client = DynamixelClient([], port=args.port)
        baud_rates = [3000000, 1000000, 57600]

    motors = client.scan_for_motors(
        port=args.port,
        id_range=tuple(args.id_range),
        baud_rates=baud_rates,
    )

    if not motors:
        print("No motors found.")
    else:
        print(f"Found {len(motors)} motor(s):")
        for m in motors:
            print(f"  ID: {m['id']}, Baud: {m['baud_rate']}, Model: {m['model_name']}")

if __name__ == "__main__":
    main()
