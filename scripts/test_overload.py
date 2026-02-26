"""Test overload detection and auto-recovery.

Connect a single motor, slowly increment position, and physically block
the finger to trigger an overload. Recovery happens automatically via
the bulk read path in DynamixelClient.
"""

import argparse
import logging
import time
from orca_core.hardware.dynamixel_client import DynamixelClient

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def main():
    parser = argparse.ArgumentParser(description="Test overload detection and auto-recovery.")
    parser.add_argument("--port", type=str, default="/dev/tty.usbserial-FT9MISJT")
    parser.add_argument("--baudrate", type=int, default=3000000)
    parser.add_argument("--motor_id", type=int, default=2)
    args = parser.parse_args()

    mid = args.motor_id
    dxl_client = DynamixelClient([mid], args.port, args.baudrate)
    dxl_client.connect()
    dxl_client.set_operating_mode([mid], 5)  # current-based position

    print(f"Motor {mid} ready. Block the finger to trigger overload.")
    try:
        while True:
            hw_err = dxl_client.read_hardware_error(mid)
            pos, vel, cur = dxl_client.read_pos_vel_cur()
            target = pos + 0.3
            dxl_client.write_desired_pos([mid], target)
            err_str = f"  HW_ERR=0x{hw_err:02X}" if hw_err else ""
            print(f"pos={pos[0]:.3f}  target={target[0]:.3f}  cur={cur[0]:.1f}{err_str}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        dxl_client.disconnect()


if __name__ == "__main__":
    main()
