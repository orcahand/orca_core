import argparse
import dataclasses
import sys

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports
from orca_core.utils.cli import (
    add_hand_arguments,
    create_hand_from_args,
    print_calibration_progress,
    shutdown_hand,
)


ENCODER_DISABLED = "disabled"


def _finger_joint_map(joint_ids: list[str]) -> dict[str, list[str]]:
    """Group the config's joint names by finger prefix ({finger}_{type}; bare wrist)."""
    mapping: dict[str, list[str]] = {}
    for joint in joint_ids:
        finger = joint.split("_", 1)[0]
        mapping.setdefault(finger, []).append(joint)
    return mapping


def _resolve_joints(parser, args, joint_ids: list[str]) -> list[str] | None:
    """Expand --fingers / validate --joints against the loaded config."""
    if args.fingers:
        finger_map = _finger_joint_map(joint_ids)
        unknown = [f for f in args.fingers if f not in finger_map]
        if unknown:
            parser.error(
                f"Unknown finger(s) {unknown}; this hand has {sorted(finger_map)}."
            )
        joints = [j for finger in args.fingers for j in finger_map[finger]]
        print(f"Calibrating fingers: {args.fingers}")
        print(f"Resolved joints: {joints}")
        return joints
    if args.joints:
        unknown = [j for j in args.joints if j not in joint_ids]
        if unknown:
            parser.error(
                f"Unknown joint(s) {unknown}; this hand has {joint_ids}."
            )
        print(f"Calibrating joints: {args.joints}")
        return list(args.joints)
    return None


def _open_encoder_client(encoder_port_override: str, baudrate: int):
    """Resolve the encoder port, open the link, start the AA A9 stream.

    Returns ``(link, client)`` when the stream is up; raises on missing
    port or first-frame timeout.
    """
    ports = resolve_sensing_ports(
        tactile_override="disabled", encoder_override=encoder_port_override,
    )
    if ports.encoder is None:
        raise RuntimeError(
            "use_joint_feedback is enabled but no encoder port was found "
            f"(encoder_serial_port={encoder_port_override!r}). "
            "Pass --encoder-port to override."
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
    hand = create_hand_from_args(args, engage_feedback=False)
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

    link = None
    client = None
    encoder_pass = (
        hand.config.joint_feedback_enabled
        and not args.mock
        and hand.config.encoder_serial_port != ENCODER_DISABLED
    )
    if hand.config.joint_feedback_enabled and not encoder_pass and not args.mock:
        print("Encoder pass disabled; running the open-loop motor-limits pass only.")
    if encoder_pass:
        try:
            link, client = _open_encoder_client(
                hand.config.encoder_serial_port, hand.config.encoder_baudrate
            )
        except Exception as exc:
            print(f"FAIL: could not open encoder stream ({exc})")
            shutdown_hand(hand)
            sys.exit(1)

    try:
        hand.calibrate(
            force_wrist=args.force_wrist,
            joints=joints,
            joint_encoder_client=client,
            progress_callback=print_calibration_progress,
        )
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        if client is not None:
            try:
                client.stop_stream()
            except Exception:
                pass
            client.disconnect()
        if link is not None:
            link.disconnect()
        shutdown_hand(hand)


if __name__ == "__main__":
    main()
