import argparse
import dataclasses
import sys
from contextlib import ExitStack

from orca_core.utils.cli import (
    add_hand_arguments,
    create_hand_from_args,
    open_encoder_stream,
    print_calibration_progress,
    select_joints,
    shutdown_hand,
)


ENCODER_DISABLED = "disabled"


def _resolve_joints(parser, args, joint_ids: list[str]) -> list[str] | None:
    """Expand --fingers / validate --joints against the loaded config."""
    try:
        joints = select_joints(joint_ids, args.fingers, args.joints)
    except ValueError as exc:
        parser.error(str(exc))
    if args.fingers:
        print(f"Calibrating fingers: {args.fingers}")
        print(f"Resolved joints: {joints}")
    elif args.joints:
        print(f"Calibrating joints: {args.joints}")
    return joints


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand (autodetects the connected hand by default)."
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Force wrist calibration even if already calibrated",
    )
    parser.add_argument(
        "--fingers",
        type=str,
        nargs="+",
        help="Fingers to calibrate (e.g., --fingers thumb index pinky)",
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        help="Individual joints to calibrate (e.g., --joints thumb_cmc index_mcp)",
    )
    parser.add_argument(
        "--encoder-port",
        default=None,
        help='Override config encoder_serial_port for the joint-encoder pass. '
             '"auto" runs discovery; an explicit path bypasses; "disabled" '
             'forces the open-loop motor-limits pass only.',
    )
    args = parser.parse_args()

    if args.fingers and args.joints:
        parser.error("Cannot specify both --fingers and --joints. Use one or the other.")

    # Leave the feedback loop disengaged regardless of config: calibration drives
    # the motors open-loop and opens its own reader on the encoder stream.
    hand = create_hand_from_args(args, engage_feedback=False, engage_sensors=False)
    if args.encoder_port is not None:
        hand.config = dataclasses.replace(
            hand.config, encoder_serial_port=args.encoder_port,
        )

    joints = _resolve_joints(parser, args, hand.config.joint_ids)

    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        sys.exit(1)
    print(f"Motor family: {hand.config.motor_type} @ {hand.config.baudrate} bps")

    client = None
    streams = ExitStack()
    encoder_pass = (
        hand.config.joint_feedback_enabled
        and not args.mock
        and hand.config.encoder_serial_port != ENCODER_DISABLED
    )
    if hand.config.joint_feedback_enabled and not encoder_pass and not args.mock:
        print("Encoder pass disabled; running the open-loop motor-limits pass only.")

    try:
        if encoder_pass:
            try:
                client = streams.enter_context(open_encoder_stream(
                    hand.config.encoder_serial_port, hand.config.encoder_baudrate
                ))
            except Exception as exc:
                print(f"FAIL: could not open encoder stream ({exc})")
                sys.exit(1)

        hand.calibrate(
            force_wrist=args.force_wrist,
            joints=joints,
            joint_encoder_client=client,
            progress_callback=print_calibration_progress,
        )
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        streams.close()
        shutdown_hand(hand)


if __name__ == "__main__":
    main()
