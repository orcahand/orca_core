from __future__ import annotations

import argparse
import sys

from . import FileStateStore, OrcaHand, list_builtin_profiles, load_profile, load_profile_from_path
from .utils import auto_detect_port, get_and_choose_port


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="orca-hand",
        description="CLI for installed ORCA Hand workflows.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor = subparsers.add_parser("doctor", help="Inspect profiles and available serial devices.")
    _add_common_profile_args(doctor)
    doctor.set_defaults(func=_run_doctor)

    calibrate = subparsers.add_parser("calibrate", help="Calibrate a hand profile.")
    _add_common_hand_args(calibrate)
    calibrate.add_argument("--force-wrist", action="store_true", help="Re-run wrist calibration even if state exists.")
    calibrate.set_defaults(func=_run_calibrate)

    tension = subparsers.add_parser("tension", help="Hold tension on the hand.")
    _add_common_hand_args(tension)
    tension.add_argument("--move-motors", action="store_true", help="Move motors before holding tension.")
    tension.set_defaults(func=_run_tension)

    neutral = subparsers.add_parser("neutral", help="Move the hand to its neutral position.")
    _add_common_hand_args(neutral)
    neutral.set_defaults(func=_run_neutral)

    zero = subparsers.add_parser("zero", help="Move the hand to the zero position.")
    _add_common_hand_args(zero)
    zero.set_defaults(func=_run_zero)

    args = parser.parse_args(argv)
    return args.func(args)


def _add_common_profile_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--profile", default=None, help="Built-in profile name.")
    parser.add_argument("--profile-path", default=None, help="Path to a custom profile directory or config.yaml.")
    parser.add_argument("--state-dir", default=None, help="Override the runtime state directory.")


def _add_common_hand_args(parser: argparse.ArgumentParser) -> None:
    _add_common_profile_args(parser)
    parser.add_argument("--port", default=None, help="Explicit serial port to use.")
    parser.add_argument("--auto-port", action="store_true", help="Auto-detect a supported USB serial adapter.")
    parser.add_argument("--choose-port", action="store_true", help="Interactively choose a serial device.")


def _run_doctor(args: argparse.Namespace) -> int:
    profile = _load_profile(args)
    print(f"Profile: {profile.profile_id}")
    print(f"Built-in profiles: {', '.join(list_builtin_profiles())}")

    detected = auto_detect_port()
    if detected:
        print(f"Auto-detected port: {detected}")
    else:
        print("Auto-detected port: none")

    try:
        import serial.tools.list_ports

        ports = list(serial.tools.list_ports.comports())
    except Exception as exc:
        print(f"Failed to inspect serial ports: {exc}")
        return 0

    if not ports:
        print("Available serial devices: none")
        return 0

    print("Available serial devices:")
    for port in ports:
        print(f"- {port.device}: {port.description or 'unknown device'}")
    return 0


def _run_calibrate(args: argparse.Namespace) -> int:
    hand = _build_hand(args)
    try:
        _connect_or_exit(hand, args)
        hand.calibrate(force_wrist=args.force_wrist)
        return 0
    finally:
        hand.disconnect()


def _run_tension(args: argparse.Namespace) -> int:
    hand = _build_hand(args)
    try:
        _connect_or_exit(hand, args)
        hand.enable_torque()
        hand.tension(args.move_motors)
        return 0
    finally:
        hand.disconnect()


def _run_neutral(args: argparse.Namespace) -> int:
    hand = _build_hand(args)
    try:
        _connect_or_exit(hand, args)
        hand.enable_torque()
        hand.set_control_mode("current_based_position")
        hand.set_neutral_position()
        return 0
    finally:
        hand.disconnect()


def _run_zero(args: argparse.Namespace) -> int:
    hand = _build_hand(args)
    try:
        _connect_or_exit(hand, args)
        hand.enable_torque()
        hand.set_control_mode("current_based_position")
        hand.set_zero_position()
        return 0
    finally:
        hand.disconnect()


def _build_hand(args: argparse.Namespace) -> OrcaHand:
    return OrcaHand(
        profile=_load_profile(args),
        state_store=FileStateStore(args.state_dir) if args.state_dir else FileStateStore(),
    )


def _load_profile(args: argparse.Namespace):
    if args.profile_path:
        return load_profile_from_path(args.profile_path)
    return load_profile(args.profile) if args.profile else load_profile()


def _connect_or_exit(hand: OrcaHand, args: argparse.Namespace) -> None:
    port = _resolve_port(args)
    success, message = hand.connect(port=port)
    print(message)
    if not success:
        raise SystemExit(1)


def _resolve_port(args: argparse.Namespace) -> str | None:
    if args.port:
        return args.port
    if args.auto_port:
        return auto_detect_port()
    if args.choose_port:
        return get_and_choose_port()
    return None


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
