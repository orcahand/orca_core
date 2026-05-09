"""Print encoder-derived and motor-derived joint angles side by side.

Loads a calibrated hand, opens the joint-encoder stream, disables torque,
and prints two estimates of each encoder-backed joint angle in degrees:

  * ``enc``   — direct from the joint encoder (calibrated via the
    dual-capture sweep).
  * ``motor`` — open-loop estimate from motor position via
    ``_motor_to_joint_pos`` (motor limits + joint-to-motor ratios).

Move the finger by hand; both columns should track each other, and the
delta column shows how much tendon backlash + cable stretch costs you.

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

    encoder_joints = hand._encoder_backed_joints()
    if not encoder_joints:
        print("WARN: no encoder-backed joints in config (joint_encoder_joints is empty).")

    print(
        "Encoder stream active. Move the hand by hand; angles in degrees."
    )
    print("Press Ctrl-C to stop.")
    print()

    period = 1.0 / max(args.rate_hz, 0.1)
    started = time.monotonic()
    deadline = started + args.duration if args.duration > 0 else math.inf
    try:
        while time.monotonic() < deadline:
            reading = client.get_latest_encoder_reading()
            if reading is None:
                print("(no encoder frame yet)")
                time.sleep(period)
                continue
            enc_angles = hand._raw_to_joint_angle(reading.raw_counts)
            motor_angles = hand._motor_to_joint_pos(hand.get_motor_pos())
            t = time.monotonic() - started
            cells = []
            for joint in encoder_joints:
                enc = enc_angles.get(joint)
                motor = motor_angles.get(joint)
                if enc is None or motor is None:
                    cells.append(f"{joint}: (uncalibrated)")
                    continue
                # _raw_to_joint_angle returns radians; _motor_to_joint_pos
                # returns degrees (it's built around joint_roms in degrees).
                enc_deg = math.degrees(enc)
                cells.append(
                    f"{joint}: enc={enc_deg:+7.1f}°  "
                    f"motor={motor:+7.1f}°  "
                    f"Δ={enc_deg - motor:+6.1f}°"
                )
            print(f"[t={t:5.1f}s] " + "   ".join(cells))
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
