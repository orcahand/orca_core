"""Debug script to test overload detection and reboot step by step."""

import argparse
import logging
import time
from orca_core.hardware.dynamixel_client import DynamixelClient

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="/dev/tty.usbserial-FTAK89H0")
    parser.add_argument("--baudrate", type=int, default=3000000)
    parser.add_argument("--motor_id", type=int, default=2)
    args = parser.parse_args()

    mid = args.motor_id
    dxl_client = DynamixelClient([mid], args.port, args.baudrate)

    # Open port manually — don't call connect() which retries torque enable forever
    dxl_client.port_handler.openPort()
    dxl_client.port_handler.setBaudRate(dxl_client.baudrate)
    print("Port opened.")

    print(f"\n=== Step 1: Read hardware error status ===")
    hw_err = dxl_client.read_hardware_error(mid)
    print(f"Hardware error: 0x{hw_err:02X} (overload={'YES' if hw_err & 0x20 else 'no'})")

    if not (hw_err & 0x20):
        print("No overload detected. Run test_overload.py first to trigger one.")
        dxl_client.port_handler.closePort()
        return

    print(f"\n=== Step 2: Reboot motor {mid} ===")
    dxl_client.reboot_motor(mid)
    print("Reboot command sent.")

    for wait_total in [0.3, 0.6, 1.0, 2.0]:
        time.sleep(0.3 if wait_total == 0.3 else wait_total - 0.3)
        hw_err = dxl_client.read_hardware_error(mid)
        print(f"  After {wait_total:.1f}s: hw_err=0x{hw_err:02X} ({'STILL ERROR' if hw_err else 'CLEARED'})")
        if not hw_err:
            break

    if hw_err:
        print("\nHardware error did NOT clear after reboot.")
        dxl_client.port_handler.closePort()
        return

    print(f"\n=== Step 3: Set operating mode 5 + enable torque ===")
    dxl_client.set_torque_enabled([mid], False, retries=0)
    dxl_client.set_operating_mode([mid], 5)
    print("Operating mode set, torque enabled.")

    print(f"\n=== Step 4: Read position ===")
    pos, vel, cur = dxl_client.read_pos_vel_cur()
    print(f"pos={pos[0]:.3f}  vel={vel[0]:.3f}  cur={cur[0]:.1f}")

    print("\nReboot and recovery successful!")
    dxl_client.port_handler.closePort()


if __name__ == "__main__":
    main()
