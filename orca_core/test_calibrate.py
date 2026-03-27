from __future__ import annotations

from pathlib import Path

from orca_core import OrcaHand, OrcaJointPositions


CONFIG_PATH = "orca_core/config.yaml"


def main() -> None:
    config_path = str(Path(CONFIG_PATH).expanduser().resolve())
    hand = OrcaHand(config_path=config_path)

    try:
        print("1. Connecting")
        success, message = hand.connect()
        print(f"   connect() -> success={success}, message={message}")
        if not success:
            raise RuntimeError(message)

        print("2. Read-only checks")
        print(f"   connected: {hand.is_connected()}")
        print(f"   motor_pos keys: {list(hand.get_motor_pos(as_dict=True).keys())[:5]} ...")
        print(f"   motor_current keys: {list(hand.get_motor_current(as_dict=True).keys())[:5]} ...")
        print(f"   motor_temp keys: {list(hand.get_motor_temp(as_dict=True).keys())[:5]} ...")

        # print("3. Calibrate and prepare (torque, mode, limits/ratios, wrap, neutral)")
        # hand.calibrate()
        hand.set_neutral_position()

    finally:
        hand.stop_task()
        success, message = hand.disconnect()
        print(f"disconnect() -> success={success}, message={message}")

if __name__ == "__main__":
    main()
