"""Print joint angles from the encoder stream with motor torque disabled.

Loads a calibrated hand, opens the joint-encoder stream, disables torque,
and prints per-joint angles in degrees so the operator can move the hand
by hand and visually verify polarity/anchor values.

Usage:
    uv run python scripts/verify_joint_angle.py /path/to/orcahand_v1/config.yaml
    uv run python scripts/verify_joint_angle.py /path/to/orcahand_v1/config.yaml \
        --encoder-port /dev/cu.usbmodem103
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
import time

import numpy as np

from orca_core import OrcaHand
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import LINK_DEFAULT_BAUDRATE
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Print joint angles from the encoder stream with motor torque disabled."
    )
    p.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to the hand config.yaml file (defaults to the bundled model).",
    )
    p.add_argument(
        "--encoder-port",
        default="auto",
        help='Encoder serial port. "auto" (default) runs discovery; pass an explicit '
             'path (e.g. /dev/cu.usbmodem103) to bypass.',
    )
    p.add_argument("--baudrate", type=int, default=LINK_DEFAULT_BAUDRATE)
    p.add_argument(
        "--duration", type=float, default=0.0,
        help="Seconds to stream before exiting; 0 means run until Ctrl-C.",
    )
    p.add_argument(
        "--rate-hz", type=float, default=2.0,
        help="Print rate in Hz (default 2).",
    )
    return p.parse_args()


def resolve_encoder_port(override: str) -> str | None:
    ports = resolve_sensing_ports(tactile_override="disabled", encoder_override=override)
    return ports.encoder


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    hand = OrcaHand(config_path=args.config_path)
    if not hand.is_calibrated(use_joint_feedback=True, verbose=True):
        print(
            "FAIL: hand is not calibrated for joint feedback. "
            "Set use_joint_feedback: true in config.yaml and re-run calibration."
        )
        return 1

    encoder_port = resolve_encoder_port(args.encoder_port)
    if encoder_port is None:
        print("FAIL: no encoder port detected. Pass --encoder-port explicitly.")
        return 1
    print(f"Encoder port: {encoder_port}")

    motor_ok, motor_msg = hand.connect()
    print(motor_msg)
    if not motor_ok:
        return 1
    hand.disable_torque()

    link = HandSerialLink(encoder_port, baudrate=args.baudrate)
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    try:
        client.start_encoder_stream(timeout=2.0)
    except EncodersNotAvailableError as e:
        print(f"FAIL: no encoder frames within timeout ({e})")
        client.disconnect()
        link.disconnect()
        hand.disconnect()
        return 1

    print("Encoder stream active. Move the hand by hand; angles below are in degrees.")
    print("Press Ctrl-C to stop.")
    print()

    period = 1.0 / max(args.rate_hz, 0.1)
    started = time.monotonic()
    deadline = started + args.duration if args.duration > 0 else math.inf
    try:
        while time.monotonic() < deadline:
            reading = client.get_latest_encoder_reading()
            if reading is None:
                print("(no frame yet)")
            else:
                angles = hand._raw_to_joint_angle(reading.raw_counts)
                row = "  ".join(
                    f"{joint}={math.degrees(angle):+7.1f}°"
                    for joint, angle in sorted(angles.items())
                )
                t = time.monotonic() - started
                print(f"[t={t:5.1f}s] {row}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        client.disconnect()
        link.disconnect()
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
