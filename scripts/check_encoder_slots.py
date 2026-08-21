"""Preamble check for the joint encoders: slot health + slot identity.

Run before any procedure that trusts the encoders (hardstop encoder pass,
vision calibration). Health verifies every slot streams unflagged (a flagged
slot wants a power cycle); identity wiggles each joint at limited current and
verifies its mapped slot is the one that responds.
"""
import argparse
import dataclasses
import sys

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports
from orca_core.maintenance.encoder_slot_check import run_slot_check
from orca_core.utils.cli import add_hand_arguments, create_hand_from_args


def _open_encoder_client(encoder_port_override, baudrate):
    """Resolve the encoder port, open the link, start the AA A9 stream."""
    ports = resolve_sensing_ports(
        tactile_override="disabled", encoder_override=encoder_port_override,
    )
    if ports.encoder is None:
        raise RuntimeError(
            "no encoder port found "
            f"(encoder_serial_port={encoder_port_override!r}); pass --encoder-port"
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


def _print_progress(event: dict) -> None:
    name = event.get("event")
    if name == "slot_check_started":
        print(f"Checking {len(event['joints'])} encoder-backed joint(s)...")
    elif name == "slot_health" and event["verdict"] != "ok":
        print(
            f"  {event['joint']} (slot {event['slot']}): {event['verdict'].upper()} "
            f"(angle-error {event['angle_error_pct']}%, "
            f"parity-fail {event['parity_fail_pct']}%)"
        )
    elif name == "identity_started":
        print("Health scan done; wiggling each joint at limited current...")
    elif name == "slot_identity":
        verdict = event["verdict"]
        if verdict == "ok":
            print(f"  {event['joint']}: ok (dominance {event['dominance']}x)")
        elif verdict == "skipped-flagged":
            print(f"  {event['joint']}: skipped (slot flagged in health scan)")
        else:
            print(
                f"  {event['joint']}: {verdict.upper()} — expected slot "
                f"{event['expected_slot']}, strongest responder "
                f"{event['argmax_slot']} (responses {event['responses_deg']})"
            )
    elif name == "cleanup_failed":
        print(f"WARNING: cleanup failed: {event['error']}")


def _print_summary(result) -> None:
    print()
    bad_health = [h for h in result.health if h.verdict != "ok"]
    bad_identity = [i for i in result.identity if i.verdict != "ok"]
    if result.passed:
        print(f"PASS: {len(result.health)} slot(s) healthy"
              + (f", {len(result.identity)} identity check(s) ok"
                 if result.identity else " (identity not run)"))
        return
    print("FAIL:")
    for h in bad_health:
        remedy = ("power-cycle the hand and re-run; persistent flags are a "
                  "firmware-side issue" if h.verdict == "flagged"
                  else "check encoder wiring/baud")
        print(f"  {h.joint} (slot {h.slot}): {h.verdict} — {remedy}")
    for i in bad_identity:
        if i.verdict == "skipped-flagged":
            continue
        print(f"  {i.joint}: {i.verdict} — expected slot {i.expected_slot}, "
              f"strongest responder {i.argmax_slot}")
    if any(i.verdict == "mismapped" for i in bad_identity):
        print("  A mismapped slot means a crossed connector or a wrong "
              "JOINT_TO_ENCODER_SLOT entry; no calibration is valid until fixed.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        epilog="--mock runs the health scan only: the mock encoder stream is "
               "static by design, so identity has nothing to wiggle.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument("--joints", nargs="+", help="Restrict to these joints.")
    parser.add_argument("--health-only", action="store_true",
                        help="Skip the identity wiggle (no motor motion).")
    parser.add_argument("--encoder-port", default=None,
                        help="Override config encoder_serial_port.")
    args = parser.parse_args()

    # On hardware a motor-only hand: the routine opens its own encoder stream,
    # and a second reader on the port would corrupt it. The mock has no port to
    # open, so there it reads the hand's own encoder client instead.
    hand = create_hand_from_args(args, engage_feedback=args.mock, engage_sensors=False)

    if not hand.config.joint_feedback_enabled:
        print("This hand has no joint encoders (use_joint_feedback is off).",
              file=sys.stderr)
        return 2

    if args.joints:
        unknown = [j for j in args.joints if j not in hand.config.joint_ids]
        if unknown:
            parser.error(f"unknown joint(s) {unknown}; hand has {hand.config.joint_ids}")

    if args.encoder_port is not None:
        hand.config = dataclasses.replace(
            hand.config, encoder_serial_port=args.encoder_port
        )

    ok, message = hand.connect()
    print(f"connect() -> {ok}: {message}")
    if not ok:
        return 2

    link = client = None
    try:
        if args.mock:
            client = hand._encoder_client
        else:
            link, client = _open_encoder_client(
                hand.config.encoder_serial_port, hand.config.encoder_baudrate
            )
        result = run_slot_check(
            hand,
            client,
            joints=args.joints,
            identity=not (args.health_only or args.mock),
            progress_callback=_print_progress,
        )
    except KeyboardInterrupt:
        print("\nInterrupted; torque released.", file=sys.stderr)
        return 130
    finally:
        if client is not None and not args.mock:
            try:
                client.disconnect()
            except Exception:
                pass
        if link is not None:
            link.disconnect()
        try:
            hand.disconnect()
        except Exception:
            pass

    if result is None:
        print("Aborted before completion.")
        return 1
    _print_summary(result)
    return 0 if result.passed else 1


if __name__ == "__main__":
    sys.exit(main())
