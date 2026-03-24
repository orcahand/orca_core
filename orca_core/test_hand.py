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

        print("3. Calibrate and prepare (torque, mode, limits/ratios, wrap, neutral)")
        hand.init_joints()

        n = hand.config.neutral_position

        print("4. Small pose A")
        pose_a = OrcaJointPositions.from_dict(
            {
                "thumb_mcp": n["thumb_mcp"] + 5.0,
                "index_mcp": n["index_mcp"] + 5.0,
                "middle_mcp": n["middle_mcp"] + 5.0,
                "wrist": n["wrist"] + 3.0,
            }
        )
        hand.set_joint_positions(pose_a, num_steps=8, step_size=0.02)

        print("5. Small pose B")
        pose_b = OrcaJointPositions.from_dict(
            {
                "thumb_mcp": n["thumb_mcp"] - 5.0,
                "index_mcp": n["index_mcp"] + 3.0,
                "middle_mcp": n["middle_mcp"] + 6.0,
                "wrist": n["wrist"] - 3.0,
            }
        )
        hand.set_joint_positions(pose_b, num_steps=8, step_size=0.02)

        print("6. Named pose replay")
        hand.register_position("regression_pose_b", pose_b)
        hand.set_neutral_position(num_steps=8, step_size=0.02)
        hand.set_named_position("regression_pose_b", num_steps=8, step_size=0.02)

        print("7. Tiny jitter")
        motor_pos_before = hand.get_motor_pos()
        hand.jitter(amplitude=1.0, frequency=3.0, duration=0.2, blocking=True)
        motor_pos_after = hand.get_motor_pos()
        print(
            f"   |after - before| sum (expect ~0; pose is restored after jitter): "
            f"{float(abs(motor_pos_after - motor_pos_before).sum())}"
        )

        print("8. Tensioning")
        hand.tension()

        print("9. Return to neutral")
        hand.set_neutral_position()

        print("\nDone.")

    finally:
        hand.stop_task()
        success, message = hand.disconnect()
        print(f"disconnect() -> success={success}, message={message}")

if __name__ == "__main__":
    main()
