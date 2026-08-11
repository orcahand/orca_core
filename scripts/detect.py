#!/usr/bin/env python
"""Report what orca_core thinks is plugged in.

Probes the connected hardware exactly as :func:`orca_core.load_hand` does and
prints the result: the bundled model that matches, the ports found, the motor
family answering on the bus, and the hand class that would be built. Read-only —
no torque, no motion. Run this first on a hand that misbehaves.

Usage:
    uv run python scripts/detect.py
"""

import argparse
import dataclasses

from orca_core import detect_hand, load_hand


def _hand_class_name(detection) -> str:
    """The class load_hand() would build for the detected model."""
    return type(load_hand(model_name=detection.model_name)).__name__


def main() -> int:
    argparse.ArgumentParser(description=__doc__.split("\n", 1)[0]).parse_args()

    print("Probing the connected hand...")
    detection = detect_hand()

    identity = detection.identity
    rows = [
        ("model_name", detection.model_name),
        ("hand_class", _hand_class_name(detection)),
        ("side", detection.side),
        ("has_tactile", detection.has_tactile),
        ("has_encoders", detection.has_encoders),
        ("motor_port", detection.motor_port),
        ("motor_type", detection.motor_type),
        ("motor_baudrate", detection.motor_baudrate),
        ("sensing_port", detection.sensing_port),
        ("tactile_port", detection.tactile_port),
        ("identity", dataclasses.asdict(identity) if identity is not None else None),
    ]
    width = max(len(name) for name, _ in rows)
    for name, value in rows:
        print(f"  {name:<{width}}  {value}")

    if detection.motor_port is None:
        print("\nNo motor bus found; check the USB cable and the hand's power.")
    elif detection.motor_type is None:
        print("\nMotor bus found but no motor answered; check power and the chain wiring.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
