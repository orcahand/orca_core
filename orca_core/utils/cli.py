# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Shared CLI helpers for the operator scripts and examples."""

import sys
import time
from argparse import ArgumentParser, Namespace
from contextlib import contextmanager
from pathlib import Path

import yaml

from orca_core import BaseHand, load_hand
from orca_core.constants import normalize_hardware_version
from orca_core.hand_config import format_hardware_version
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import JointEncoderClient
from orca_core.hardware.sensing.serial_discovery import board_info_for_port, resolve_sensing_ports


def add_hand_arguments(
    parser: ArgumentParser, *, mock_default: bool = False, feedback_flag: bool = True
) -> None:
    """Add the shared hand-selection arguments.

    ``feedback_flag=False`` omits ``--no-engage-feedback`` for front-ends that
    always drive the motors open-loop, so the flag is never advertised where it
    could not be honoured.
    """
    parser.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to config.yaml; omit to autodetect the connected hand.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        default=mock_default,
        help="Use the in-memory mock hand instead of a physical one.",
    )
    parser.add_argument(
        "--model-name",
        default=None,
        help="Bundled model to load (e.g. orcahand-full-left) instead of autodetecting.",
    )
    parser.add_argument(
        "--motor-port",
        dest="motor_port",
        default=None,
        help="Motor bus device to use (e.g. /dev/cu.usbmodemXXXX) instead of detecting "
             "it. Only that port is tried: if no motor answers there the run stops.",
    )
    parser.add_argument(
        "--hardware-version",
        dest="hardware_version",
        type=int,
        default=None,
        metavar="N",
        help="The hand's hardware version (200 = v2.0.0, 210 = v2.1.0, 211 = v2.1.1) for this "
             "run only, over what its board reports; never saved. Without it a hand "
             "with no board to ask is asked about interactively.",
    )
    if feedback_flag:
        parser.add_argument(
            "--no-engage-feedback",
            dest="engage_feedback",
            action="store_false",
            default=True,
            help="Load the motor-only hand even when the config enables joint feedback.",
        )


def create_hand(
    config_path: str | None,
    *,
    use_mock: bool,
    model_name: str | None = None,
    engage_feedback: bool = True,
    engage_sensors: bool = True,
    motor_port: str | None = None,
) -> BaseHand:
    """Build the hand class matching the selected — or detected — model.

    With no ``config_path`` or ``model_name`` on a physical hand this probes
    the hardware, so the model matches what is actually plugged in rather than
    the packaged default. ``motor_port`` is the only motor bus a connect tries.
    """
    hand = load_hand(
        config_path=config_path,
        mock=use_mock,
        model_name=model_name,
        engage_feedback=engage_feedback,
        engage_sensors=engage_sensors,
        motor_port=motor_port,
    )
    print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return hand


def create_hand_from_args(args: Namespace, **overrides) -> BaseHand:
    """Build the hand every argument in :func:`add_hand_arguments` selects.

    Front-ends call this instead of :func:`create_hand` so a flag can never be
    advertised and then dropped. ``overrides`` pin what the front-end decides
    itself, e.g. ``engage_feedback=False`` for a routine that must drive the
    motors open-loop, or ``engage_sensors=False`` for one that opens its own
    reader on the sensing port.
    """
    options = {
        "use_mock": args.mock,
        "model_name": args.model_name,
        "engage_feedback": getattr(args, "engage_feedback", True),
        "motor_port": getattr(args, "motor_port", None) or None,
    }
    options.update(overrides)
    hand = create_hand(args.config_path, **options)
    version = getattr(args, "hardware_version", None)
    if version is not None:
        pin_hardware_version(hand, version)
    return hand


def pin_hardware_version(hand, version: int) -> int | None:
    """Use ``version`` for this hand for this run only; nothing is saved.
    Returns the version in force."""
    in_force = hand.set_hardware_version(int(version))
    print(f"Hardware version {format_hardware_version(in_force)} for this run "
          "(not saved; the board's version is used when it reports one).")
    return in_force


def prompt_hardware_version(hand, ask=input) -> "int | None":
    """Ask which revision a hand of unknown version is, for this run.

    Reached only when neither a controller board nor config.yaml said: a plain
    adapter. Enter keeps the version unknown for this run (v2.0 behaviour:
    base map, no travel features). The answer is never saved.
    """
    print(
        "Hardware version unknown: no controller board answered and none is pinned.\n"
        "A v2.1 hand needs it for its motor map and travel baseline."
    )
    while True:
        try:
            answer = ask("Enter 200 (v2.0.0), 210 (v2.1.0), 211 (v2.1.1), or press Enter to continue as v2.0: ").strip()
        except (EOFError, OSError):
            answer = ""
        if answer == "":
            print("Continuing with the hardware version unknown (v2.0 behaviour).")
            return None
        if answer.isdigit():
            return pin_hardware_version(hand, int(answer))
        print("Please enter a whole number (200, 210 or 211) or press Enter.")


def settle_hardware_version(hand, *, interactive: bool = True, ask=input) -> None:
    """Decide the connected hand's hardware version the way the console does.

    ``--hardware-version`` or the board's answer at connect settle it. A board
    whose motor port did not answer is asked on its other port. Then a pin in
    config.yaml applies, and only with neither is the operator asked, for this
    run only, and only when a terminal can answer.
    """
    if hand.hardware_version_override is not None:
        return
    if hand.board_hardware_version is not None:
        print("Controller board reports hardware version "
              f"{format_hardware_version(hand.board_hardware_version)}.")
        return
    board = board_info_for_port(getattr(hand.config, "port", "") or "")
    reported = normalize_hardware_version(board.hw_version) if board is not None else None
    if reported is not None:
        hand._board_hardware_version = reported
        hand._apply_hardware_version()
        print(f"Controller board reports hardware version {format_hardware_version(reported)}.")
        return
    if hand.hardware_version is not None:
        return
    # Only a terminal can answer: a piped or captured stdin keeps the version
    # unknown rather than blocking (or failing) the run.
    can_ask = ask is not input or sys.stdin.isatty()
    if interactive and can_ask:
        prompt_hardware_version(hand, ask)


def load_stored_calibration(hand, hand_id: str | None = None) -> str:
    """Load a hand's stored calibration without connecting to it.

    ``hand_id`` names the hand's folder; without it, the one hand of this model
    with a calibration on this computer is used. The calibration is read the
    way the hand reads it at connect: with that hand's joint ranges and ROM
    frame, under the hardware version its board last reported (or
    ``--hardware-version``). Returns the hand id.

    Raises:
        ValueError: no such hand, or several to choose from (they are listed).
    """
    from orca_core import hand_store

    if hand_id is None:
        model = hand._model_name()
        candidates = [
            h["id"] for h in hand_store.known_hands()
            if h.get("model") == model
            and (hand_store.hand_dir(h["id"]) / hand_store.CALIBRATION_FILE).exists()
        ]
        if len(candidates) != 1:
            known = ", ".join(candidates) or "none"
            raise ValueError(
                f"name the hand with --hand-id; calibrated {model} hands on this "
                f"computer: {known}")
        hand_id = candidates[0]
    path = hand_store.hand_dir(hand_id) / hand_store.CALIBRATION_FILE
    if not path.exists():
        raise ValueError(f"no calibration kept for hand {hand_id} ({path})")
    hand._use_stored_hand(hand_id)
    print(f"Using the calibration of hand {hand_id} in {path}")
    if hand.hardware_version is None and hand.config.joint_to_motor_map_v21:
        print("Its hardware version is not known here; pass --hardware-version if it is v2.1.")
    return hand_id


def board_store_problem(hand) -> "str | None":
    """Why this hand's calibration could not be kept on its board, said before a
    run rather than after it; ``None`` when it can (or there is no board)."""
    from ..board_store import BoardStoreError, check_packable

    if not getattr(hand, "board_hand_id", None):
        return None
    try:
        check_packable(hand.config.joint_ids, hand.config.motor_ids)
    except BoardStoreError as exc:
        print(f"Note: this hand's calibration will be kept on this computer only: {exc}.")
        return str(exc)
    return None


def push_calibration_to_board(hand) -> bool:
    """Keep this hand's fresh calibration on the hand's own board too.

    Call once the hand has released its port, after a successful run on a real
    hand. Needs an orca_firmware checkout (``ORCA_FIRMWARE_REPO``, or beside
    orca_core); without one, or on a hand with no board, nothing is written and
    the run is still complete: the hand's folder holds its calibration.
    """
    from ..board_push import push_for_hand

    try:
        result = push_for_hand(hand)
    except Exception as exc:
        print(f"Calibration kept on this computer only: the board was not written ({exc}).")
        return False
    if result.written:
        print(f"Calibration also written to the hand's own board "
              f"(store sequence {result.sequence}, {result.size} bytes).")
    elif result.reason:
        print(f"Calibration kept on this computer only: {result.reason}.")
    return result.written


def connect_hand(hand, *, interactive: bool = True, ask=input) -> None:
    """Connect, then settle the hardware version (see :func:`settle_hardware_version`)."""
    success, message = hand.connect(interactive=interactive)
    print(f"connect() -> success={success}, message={message}")
    if not success:
        raise RuntimeError(message)
    settle_hardware_version(hand, interactive=interactive, ask=ask)


@contextmanager
def open_encoder_stream(port_override: str, baudrate: int, *, timeout: float = 2.0):
    """Open the encoder link and its stream, yield the client, and close both on exit."""
    ports = resolve_sensing_ports(tactile_override="disabled", encoder_override=port_override)
    if ports.encoder is None:
        raise RuntimeError(
            f"no encoder port found (encoder_serial_port={port_override!r}). "
            "Pass --encoder-port to override."
        )
    link = HandSerialLink(ports.encoder, baudrate=baudrate)
    link.connect()
    client = None
    try:
        client = JointEncoderClient(link)
        client.connect()
        client.start_stream(timeout=timeout)
        print(f"Encoder stream active on {ports.encoder}")
        yield client
    finally:
        if client is not None:
            try:
                client.stop_stream()
            except Exception:
                pass
            client.disconnect()
        link.disconnect()


def group_joints_by_finger(joint_ids: list[str]) -> dict[str, list[str]]:
    """Group joint names by finger prefix ({finger}_{type}; bare wrist)."""
    groups: dict[str, list[str]] = {}
    for joint in joint_ids:
        groups.setdefault(joint.split("_", 1)[0], []).append(joint)
    return groups


def select_joints(
    joint_ids: list[str], fingers: list[str] | None = None, joints: list[str] | None = None,
) -> list[str] | None:
    """Expand ``fingers`` or validate ``joints`` against ``joint_ids``, returning ``None`` when neither is given."""
    if fingers:
        by_finger = group_joints_by_finger(joint_ids)
        unknown = [f for f in fingers if f not in by_finger]
        if unknown:
            raise ValueError(f"Unknown finger(s) {unknown}; this hand has {sorted(by_finger)}.")
        return [joint for finger in fingers for joint in by_finger[finger]]
    if joints:
        unknown = [j for j in joints if j not in joint_ids]
        if unknown:
            raise ValueError(f"Unknown joint(s) {unknown}; this hand has {joint_ids}.")
        return list(joints)
    return None


def _recording_timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def build_recording_path(output_dir: Path, kind: str, prefix: str = "") -> Path:
    """Timestamped YAML path ``[prefix_]kind_<time>.yaml`` inside ``output_dir``."""
    stem = f"{prefix}_{kind}_{_recording_timestamp()}" if prefix else f"{kind}_{_recording_timestamp()}"
    return output_dir / f"{stem}.yaml"


def recording_metadata(recording_type: str, hand, **fields) -> dict:
    """Metadata header for a recording: its type, creation time, and the joint order and side it is valid for."""
    return {
        "type": recording_type,
        "created_at": _recording_timestamp(),
        "joint_ids": hand.config.joint_ids,
        "hand_type": hand.config.type,
        **fields,
    }


def load_recording(path: str) -> tuple[Path, dict] | None:
    """Read a recording by name or path; print and return ``None`` when it is missing."""
    resolved = resolve_input_path(path)
    try:
        return resolved, yaml.safe_load(resolved.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        print(f"Replay file not found: {resolved}")
        return None


def check_recording_matches_hand(
    metadata: dict, hand, *, force: bool = False, force_flag: str | None = None,
) -> None:
    """Raise ``ValueError`` if the recording's joint order or side differs from ``hand``'s."""
    expected_joint_ids = metadata.get("joint_ids")
    if expected_joint_ids is not None and expected_joint_ids != hand.config.joint_ids:
        raise ValueError("Replay joint order does not match the connected hand configuration.")

    # Left and right hands share a joint order, so only hand_type catches a
    # sequence recorded on the mirrored hand.
    if not force and metadata.get("hand_type") not in (None, hand.config.type):
        message = (
            f"Replay was recorded for hand_type={metadata['hand_type']}, "
            f"but the connected config is {hand.config.type}."
        )
        if force_flag:
            message += f" Pass {force_flag} to replay it anyway."
        raise ValueError(message)


def print_calibration_progress(event: dict) -> None:
    """Render calibration progress events on the terminal."""
    name = event.get("event")
    if name == "calibration_started":
        print(
            f"Calibrating {len(event['joints'])} joint(s) "
            f"over {event['steps']} step(s)..."
        )
    elif name == "calibration_file":
        print(f"Writing this calibration to {event['path']}")
    elif name == "step_started":
        joints = ", ".join(f"{j} ({d})" for j, d in event["joints"].items())
        print(f"[step {event['index'] + 1}/{event['total']}] {joints}")
    elif name == "limit_recorded":
        print(
            f"  motor {event['motor']} ({event['joint']}) "
            f"{event['bound']} limit at {event['limit']:.4f} rad"
        )
    elif name == "joint_calibrated":
        print(f"  {event['joint']} calibrated (ratio {event['ratio']:.4f})")
    elif name == "wrist_skipped":
        print("Wrist already calibrated; skipping wrist steps.")
    elif name == "encoder_anchor_recorded":
        print(
            f"  {event['joint']} encoder anchor: count {event['anchor_count']} "
            f"at {event['anchor_angle_deg']:.1f} deg"
        )
    elif name == "encoder_anchor_failed":
        print(f"  WARNING: encoder anchor failed for {event['joint']}: {event['error']}")
    elif name == "offset_calibration_failed":
        print(
            f"  WARNING: offset calibration failed for motor {event['motor']} "
            f"({event['joint']}); skipped"
        )
    elif name == "torque_release_failed":
        print(
            f"  WARNING: torque release failed for motor {event['motor']} "
            f"({event['joint']}); limit not recorded"
        )
    elif name == "travel_checked":
        low, high = event["bounds_deg"]
        mark = "ok" if event["within_margin"] else "OUT OF MARGIN"
        print(
            f"  {event['joint']} motor travel {event['travel_deg']:.1f} deg "
            f"vs {event['expected_deg']:.1f} deg baseline "
            f"({event['deviation'] * 100:+.0f}%, accept {low:.1f}-{high:.1f}) [{mark}]"
        )
    elif name == "travel_baseline_missing":
        print(
            f"  {event['joint']} motor travel {event['travel_deg']:.1f} deg "
            f"(no joint_motor_travel baseline; not checked)"
        )
    elif name == "travel_excess":
        print(
            f"  WARNING: {event['joint']} travelled {event['travel_deg']:.1f} deg, "
            f"{event['deviation'] * 100:+.0f}% past its "
            f"{event['expected_deg']:.1f} deg baseline; check the tendon for slip."
        )
    elif name == "travel_retry_started":
        print(
            f"  {event['joint']} fell short; re-driving at "
            f"{event['current']:.0f} mA "
            f"(attempt {event['attempt']}/{event['attempts']})"
        )
    elif name == "travel_retry_succeeded":
        print(
            f"  {event['joint']} recovered to {event['travel_deg']:.1f} deg "
            f"at {event['current']:.0f} mA "
            f"({event['deviation'] * 100:+.0f}% vs baseline)"
        )
    elif name == "travel_retry_exhausted":
        print(
            f"  WARNING: {event['joint']} still short at "
            f"{event['travel_deg']:.1f} deg of {event['expected_deg']:.1f} deg "
            f"after {event['attempts']} re-drive(s); calibrated over a "
            f"shortened range."
        )
    elif name == "travel_retry_unavailable":
        print(
            f"  WARNING: {event['joint']} short at {event['travel_deg']:.1f} deg "
            f"of {event['expected_deg']:.1f} deg; its control mode ignores the "
            f"current cap, so no re-drive can help."
        )
    elif name == "travel_retry_disabled":
        print(
            f"  WARNING: {event['joint']} short at {event['travel_deg']:.1f} deg "
            f"of {event['expected_deg']:.1f} deg; re-drive disabled "
            f"(calibration_travel_retries: 0)."
        )
    elif name == "measured_rom_recorded":
        lo, hi = event["rom"]
        print(
            f"  {event['joint']} measured ROM {lo:.1f}..{hi:.1f} deg "
            f"({event['deviation_deg']:+.1f} deg vs config)"
        )
    elif name == "measured_rom_rejected":
        print(
            f"  WARNING: {event['joint']} measured span {event['span_deg']:.1f} deg "
            f"is {event['deviation_deg']:+.1f} deg off the config ROM; keeping the "
            f"config ROM for this joint."
        )
    elif name == "motor_faulted":
        flags = " + ".join(event.get("flags") or []) or "a hardware fault"
        temp = event.get("temperature_c")
        temp_note = f" at {temp:.0f} degC" if temp is not None else ""
        print(
            f"  ERROR: motor {event['motor']} ({event['joint']}) has latched "
            f"{flags}{temp_note}; skipped. Power-cycle the hand once it has cooled."
        )
    elif name == "torque_enable_failed":
        print(
            f"  ERROR: motor {event['motor']} ({event['joint']}) did not "
            f"acknowledge torque enable; skipped this step."
        )
    elif name == "sweep_no_motion":
        print(
            f"  ERROR: motor {event['motor']} ({event['joint']}) did not move "
            f"during its {event.get('direction') or ''} sweep "
            f"({event['moved_deg']:.1f} deg); check the tendon and the joint."
        )
    elif name == "overrun_retry":
        print(
            f"  WARNING: motor {event['motor']} ({event['joint']}) turned "
            f"{event['travel_deg']:.0f} deg past its {event['limit_deg']:.0f} deg "
            f"limit without meeting a stop: the belt is poorly set (slipping). "
            f"Driving again at {100 * event['pwm_fraction']:.0f}% output "
            f"(attempt {event['attempt']})."
        )
    elif name == "hardstop_not_found":
        print(
            f"  ERROR: motor {event['motor']} ({event['joint']}) turned "
            f"{event['travel_deg']:.0f} deg without meeting a hardstop "
            f"(limit {event.get('limit_deg', 720):.0f} deg); giving up on this direction, "
            f"limit not recorded. The tendon is slack or broken: tension it "
            f"and calibrate again."
        )
    elif name == "drive_step_timeout":
        print(
            f"  ERROR: motor {event['motor']} ({event['joint']}) never settled "
            f"on a hardstop; giving up on this direction, limit not recorded."
        )
    elif name == "limits_rejected":
        print(
            f"  ERROR: {event['joint']} swept only {event['travel_deg']:.1f} deg "
            f"of motor travel ({event.get('reason', 'rejected')}); limits not "
            f"recorded, previous calibration kept."
        )
    elif name == "joints_skipped":
        print(
            f"  Skipping {', '.join(event['joints'])}: {event['reason']}."
        )
    elif name == "travel_retry_skipped":
        print(
            f"  ERROR: {event['joint']} travelled {event['travel_deg']:.1f} deg "
            f"of its {event['expected_deg']:.1f} deg baseline, under the "
            f"{event['floor_deg']:.1f} deg floor: it did not move, so no "
            f"re-drive was attempted."
        )
    elif name == "manual_capture_started":
        print(
            f"  Move {event['joint']} to its {event['direction']} hardstop "
            f"(motor {event['motor']})."
        )
    elif name == "manual_capture_skipped":
        print(f"  {event['joint']} {event['direction']} skipped; previous limit kept.")
    elif name == "manual_capture_back":
        joints = ", ".join(event.get("joints") or {}) or "the previous step"
        print(f"  Back to {joints}: its limits are cleared and captured again.")
    elif name == "calibration_done":
        boosted = event.get("boosted_joints") or {}
        if boosted:
            joints = ", ".join(
                f"{j} @ {c:.0f} mA" for j, c in sorted(boosted.items())
            )
            print(f"Needed a higher-current re-drive: {joints}")
        print("Calibration complete.")
        if event.get("calibration_path"):
            print(f"Calibration written to {event['calibration_path']}")
    elif name == "drive_currents_measured":
        print(event["table"])
    elif name == "calibration_aborted":
        print("Calibration aborted.")
    elif name == "cleanup_failed":
        print(f"WARNING: cleanup after abort failed: {event['error']}")


def shutdown_hand(hand) -> None:
    try:
        hand.stop_task()
    except Exception:
        pass
    try:
        success, message = hand.disconnect()
        print(f"disconnect() -> success={success}, message={message}")
    except Exception as exc:
        print(f"disconnect() failed: {exc}")


def prepare_output_dir(path: str | None, *, default_name: str = "replay_sequences") -> Path:
    output_dir = Path(path) if path is not None else Path.cwd() / default_name
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def resolve_input_path(path: str, *, default_dir: str = "replay_sequences") -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate

    if candidate.parent != Path("."):
        return (Path.cwd() / candidate).resolve()

    return (Path.cwd() / default_dir / candidate).resolve()
