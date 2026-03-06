"""Debug test: oscillate all motors between two positions without calibration bounds.

Bypasses the OrcaHand API and drives motors directly via the motor client.
Positions stay within the safe 0-4095 raw range (0 to 2*pi radians).
"""

import sys
import os
import time
import argparse
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
from orca_core.utils import read_yaml, get_model_path

RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"

# Safe positions within 0-4095 range (in radians, 0-2*pi)
POS_A = 2.0  # ~117 degrees (~1310 raw)
POS_B = 4.0  # ~229 degrees (~2607 raw)


def create_client(motor_type, motor_ids, port, baudrate):
    if motor_type == 'feetech':
        return FeetechClient(motor_ids, port=port, baudrate=baudrate)
    return DynamixelClient(motor_ids, port=port, baudrate=baudrate)


def main():
    parser = argparse.ArgumentParser(description="Debug test: oscillate all motors (no calibration bounds).")
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    parser.add_argument('--port', type=str, default=None, help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--speed', type=float, default=2.0, help='Seconds between position switches (default: 2.0)')
    args = parser.parse_args()

    config_path = os.path.join(get_model_path(args.model_path), "config.yaml")
    config = read_yaml(config_path)

    motor_type = config.get('motor_type', 'dynamixel')
    motor_ids = config.get('motor_ids', [])
    port = args.port or config.get('port', '/dev/ttyUSB0')
    baudrate = 1000000 if motor_type == 'feetech' else config.get('baudrate', 3000000)

    print(f"\n{BOLD}Motor Debug Test{RST}")
    print(f"  Motor type: {motor_type}")
    print(f"  Port: {port} @ {baudrate:,} bps")
    print(f"  Motors: {motor_ids}")
    print(f"  Oscillating between {POS_A:.2f} and {POS_B:.2f} rad")
    print(f"  Press {BOLD}Ctrl+C{RST} to stop.\n")

    client = create_client(motor_type, motor_ids, port, baudrate)
    client.connect()

    try:
        positions = [POS_A, POS_B]
        step = 0
        while True:
            target = positions[step % 2]
            pos_array = np.full(len(motor_ids), target)
            label = "A" if step % 2 == 0 else "B"
            print(f"  Moving all motors to position {label} ({target:.2f} rad / {int(target / (2.0 * np.pi / 4096))} raw)...")
            client.write_desired_pos(motor_ids, pos_array)
            time.sleep(args.speed)

            # Read back positions
            pos, vel, cur = client.read_pos_vel_cur()
            print(f"  {DIM}Readback (rad): {[f'{p:.2f}' for p in pos]}{RST}")

            step += 1
    except KeyboardInterrupt:
        print(f"\n{GREEN}Stopped.{RST}")
    finally:
        client.set_torque_enabled(motor_ids, False)
        client.disconnect()


if __name__ == '__main__':
    main()
