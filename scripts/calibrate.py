import argparse
import dataclasses
import sys

from orca_core import OrcaHand
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import LINK_DEFAULT_BAUDRATE
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports


FINGER_TO_JOINTS = {
    "thumb": ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"],
    "index": ["index_abd", "index_mcp", "index_pip"],
    "middle": ["middle_abd", "middle_mcp", "middle_pip"],
    "ring": ["ring_abd", "ring_mcp", "ring_pip"],
    "pinky": ["pinky_abd", "pinky_mcp", "pinky_pip"],
    "wrist": ["wrist"],
}

ALL_JOINTS = [j for joints in FINGER_TO_JOINTS.values() for j in joints]


def _open_encoder_client(encoder_port_override: str):
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
    link = HandSerialLink(ports.encoder, baudrate=LINK_DEFAULT_BAUDRATE)
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    try:
        client.start_encoder_stream(timeout=2.0)
    except EncodersNotAvailableError:
        client.disconnect()
        link.disconnect()
        raise
    print(f"Encoder stream active on {ports.encoder}")
    return link, client


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate the ORCA Hand. Specify the path to the hand config.yaml file."
    )
    parser.add_argument(
        "config_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the hand config.yaml file (e.g., /path/to/orcahand_v1/config.yaml)",
    )
    parser.add_argument(
        "--force-wrist",
        action="store_true",
        help="Force wrist calibration even if already calibrated",
    )
    parser.add_argument(
        "--fingers",
        type=str,
        nargs="+",
        choices=list(FINGER_TO_JOINTS.keys()),
        help="Fingers to calibrate (e.g., --fingers thumb index pinky)",
    )
    parser.add_argument(
        "--joints",
        type=str,
        nargs="+",
        choices=ALL_JOINTS,
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

    joints = None
    if args.fingers:
        joints = []
        for finger in args.fingers:
            joints.extend(FINGER_TO_JOINTS[finger])
        print(f"Calibrating fingers: {args.fingers}")
        print(f"Resolved joints: {joints}")
    elif args.joints:
        joints = args.joints
        print(f"Calibrating joints: {joints}")

    hand = OrcaHand(config_path=args.config_path)
    if args.encoder_port is not None:
        hand.config = dataclasses.replace(
            hand.config, encoder_serial_port=args.encoder_port,
        )

    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        sys.exit(1)

    link = None
    client = None
    if hand.config.use_joint_feedback:
        try:
            link, client = _open_encoder_client(hand.config.encoder_serial_port)
        except Exception as exc:
            print(f"FAIL: could not open encoder stream ({exc})")
            hand.disconnect()
            sys.exit(1)

    try:
        hand.calibrate(
            force_wrist=args.force_wrist,
            joints=joints,
            joint_encoder_client=client,
        )
    finally:
        if client is not None:
            try:
                client.stop_encoder_stream()
            except Exception:
                pass
            client.disconnect()
        if link is not None:
            link.disconnect()


if __name__ == "__main__":
    main()
