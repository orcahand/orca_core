#!/usr/bin/env python
"""Verify each joint's encoder polarity against the motor-derived angle.

Prints the encoder-decoded angle beside the motor-derived one for every
encoder-backed joint, refreshing continuously. Move each joint by hand and
watch the two columns: they must move in the SAME direction. A joint whose
encoder angle moves opposite the motor angle has the wrong sign in
``JOINT_ENCODER_POLARITY`` for this hand's side, and closing the loop on it
would drive it away from its target instead of toward it.

Torque is off and the control loop never starts, so a wrong sign is caught
here without the hand moving on its own.

Usage:
    uv run python scripts/check_encoder_signs.py
    uv run python scripts/check_encoder_signs.py --hold   # keep torque on
"""
import argparse
import sys
import time

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import (
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports
from orca_core.utils.cli import add_hand_arguments, create_hand_from_args


REFRESH_S = 0.2


def _open_encoder_client(port_override, baudrate):
    """Resolve the encoder port, open the link and start the stream."""
    ports = resolve_sensing_ports(
        tactile_override="disabled", encoder_override=port_override
    )
    if ports.encoder is None:
        raise RuntimeError(
            "no encoder port found "
            f"(encoder_serial_port={port_override!r}). Pass --encoder-port."
        )
    link = HandSerialLink(ports.encoder, baudrate=baudrate)
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    try:
        client.start_stream(timeout=2.0)
    except EncodersNotAvailableError:
        client.disconnect()
        link.disconnect()
        raise
    print(f"Encoder stream active on {ports.encoder}")
    return link, client


def _render(joints, polarity, encoder_angles, motor_angles, raw):
    lines = [
        f"{'joint':<12} {'slot':>4} {'sign':>4} {'encoder':>9} "
        f"{'motor':>9} {'diff':>8} {'raw':>7}"
    ]
    for joint in joints:
        enc = encoder_angles.get(joint)
        mot = motor_angles.get(joint)
        if enc is None or mot is None:
            lines.append(f"{joint:<12} {'':>4} {'':>4} {'no data':>9}")
            continue
        lines.append(
            f"{joint:<12} {JOINT_TO_ENCODER_SLOT[joint]:>4} "
            f"{polarity[joint]:>+4d} {enc:>9.2f} {mot:>9.2f} "
            f"{enc - mot:>8.2f} {int(raw[JOINT_TO_ENCODER_SLOT[joint]]):>7d}"
        )
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument("--encoder-port", default=None,
                        help="Override config encoder_serial_port.")
    parser.add_argument("--hold", action="store_true",
                        help="Keep torque enabled instead of releasing it.")
    args = parser.parse_args()

    # On hardware a motor-only hand, whatever the model: the loop must not run
    # while its inputs are unverified, and a tactile connect would hold the
    # sensing CDC this script needs for its own encoder link. The mock has no
    # port to open, so there it reads the hand's own encoder client instead.
    hand = create_hand_from_args(args, engage_feedback=args.mock, engage_sensors=False)
    joints = hand.encoder_backed_joints
    if not joints:
        print("This hand declares no encoder-backed joints.")
        sys.exit(1)

    try:
        polarity = joint_encoder_polarity_for_side(hand.config.type)
    except ValueError as e:
        print(f"FAIL: {e}")
        sys.exit(1)

    anchored = hand.calibration.joint_encoder_calibration_dict
    missing = [j for j in joints if j not in anchored]
    if missing:
        print(f"No encoder anchor for {', '.join(missing)} — run scripts/calibrate.py.")
        joints = [j for j in joints if j in anchored]
        if not joints:
            sys.exit(1)

    ok, msg = hand.connect()
    if not ok:
        print(f"Failed to connect to the hand: {msg}")
        sys.exit(1)

    link = client = None
    try:
        if args.mock:
            client = hand._encoder_client
        else:
            port = args.encoder_port or hand.config.encoder_serial_port
            link, client = _open_encoder_client(port, hand.config.encoder_baudrate)

        if not args.hold:
            hand.disable_torque()
            print("Torque released — move each joint by hand.")

        print(f"Hand side: {hand.config.type}. Encoder and motor columns must "
              "move the SAME direction. Ctrl-C to stop.\n")
        while True:
            reading = client.get_latest()
            if reading is None:
                time.sleep(REFRESH_S)
                continue
            encoder_angles = hand._raw_to_joint_angle(reading.raw_counts)
            motor_angles = hand.get_joint_position().as_dict()
            print(_render(joints, polarity, encoder_angles,
                          motor_angles, reading.raw_counts))
            print()
            time.sleep(REFRESH_S)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if client is not None and not args.mock:
            try:
                client.stop_stream()
            except Exception:
                pass
            client.disconnect()
        if link is not None:
            link.disconnect()
        hand.disconnect()


if __name__ == "__main__":
    main()
