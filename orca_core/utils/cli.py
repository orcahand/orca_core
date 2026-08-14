# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Shared CLI helpers for the operator scripts and examples."""

from argparse import ArgumentParser, Namespace
from pathlib import Path

from orca_core import (
    BaseHand,
    HandSelectionError,
    HandSelector,
    detect_hands,
    load_hand,
)


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
    add_hand_selection_arguments(parser)
    if feedback_flag:
        parser.add_argument(
            "--no-engage-feedback",
            dest="engage_feedback",
            action="store_false",
            default=True,
            help="Load the motor-only hand even when the config enables joint feedback.",
        )


def add_hand_selection_arguments(parser: ArgumentParser) -> None:
    """Add ``--hand`` and ``--list-hands``.

    Separate from :func:`add_hand_arguments` so a front-end that cannot honour
    ``--mock`` or ``--model-name`` can still let the operator name a hand.
    """
    parser.add_argument(
        "--hand",
        default=None,
        metavar="HAND_ID",
        help="Which hand to use when several are attached (serial or board ID). "
             "Run --list-hands to see them.",
    )
    parser.add_argument(
        "--list-hands",
        action="store_true",
        help="List the attached hands and exit.",
    )


def describe_attached_hands(detections=None) -> str:
    """One line per attached hand, as the operator would name it."""
    if detections is None:
        detections = detect_hands()
    if not detections:
        return "  (none attached)"
    lines = []
    for detection in detections:
        name = detection.hand_id or "unidentified (board reports no id)"
        lines.append(f"  {name}  {detection.side} {detection.model_name}")
    return "\n".join(lines)


def _selector_for(hand_id: str | None) -> HandSelector | None:
    """A selector matching ``hand_id`` as either a serial or a board ID.

    Both name the same hand; which one a board reports depends on whether it
    has been provisioned, and the operator should not have to know.
    """
    if hand_id is None:
        return None
    matched = [
        d for d in detect_hands()
        if hand_id in (d.hand_id, d.board_id)
    ]
    if len(matched) == 1 and matched[0].hand_id != hand_id:
        return HandSelector(board_id=hand_id)
    return HandSelector(hand_id=hand_id)


def handle_hand_selection(args: Namespace) -> None:
    """Serve ``--list-hands``, which prints and exits."""
    if getattr(args, "list_hands", False):
        print("Attached hands:")
        print(describe_attached_hands())
        raise SystemExit(0)


def resolve_detected_model(hand_id: str | None = None) -> str:
    """The bundled model of the one attached hand, or the one ``hand_id`` names.

    For front-ends that build a hand class directly rather than through
    :func:`create_hand`. Refuses to guess between two attached hands, which
    would silently report on an arm the operator did not ask about.
    """
    from orca_core.hand_factory import select_hand

    try:
        return select_hand(detect_hands(), _selector_for(hand_id)).model_name
    except HandSelectionError as e:
        raise SystemExit(
            f"{e.summary}\n\nAttached hands:\n"
            f"{describe_attached_hands(e.detections or None)}"
            "\n\nName one with --hand HAND_ID, or --list-hands to see them."
        ) from e


def create_hand(
    config_path: str | None,
    *,
    use_mock: bool,
    model_name: str | None = None,
    engage_feedback: bool = True,
    engage_sensors: bool = True,
    hand_id: str | None = None,
) -> BaseHand:
    """Build the hand class matching the selected — or detected — model.

    With no ``config_path`` or ``model_name`` on a physical hand this probes
    the hardware, so the model matches what is actually plugged in rather than
    the packaged default. A selection failure exits with the attached hands
    listed rather than a traceback naming a Python API.
    """
    if hand_id is not None and use_mock:
        raise SystemExit(
            "--hand names a physical hand to detect; --mock has none to name."
        )
    try:
        hand = load_hand(
            config_path=config_path,
            mock=use_mock,
            model_name=model_name,
            engage_feedback=engage_feedback,
            engage_sensors=engage_sensors,
            select=_selector_for(hand_id),
        )
    except HandSelectionError as e:
        raise SystemExit(
            f"{e.summary}\n\nAttached hands:\n"
            f"{describe_attached_hands(e.detections or None)}"
            "\n\nName one with --hand HAND_ID, or --list-hands to see them."
        ) from e
    print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return hand


def create_hand_from_args(args: Namespace, **overrides) -> BaseHand:
    """Build the hand every argument in :func:`add_hand_arguments` selects.

    Front-ends call this instead of :func:`create_hand` so a flag can never be
    advertised and then dropped. ``overrides`` pin what the front-end decides
    itself, e.g. ``engage_feedback=False`` for a routine that must drive the
    motors open-loop.
    """
    handle_hand_selection(args)
    options = {
        "use_mock": args.mock,
        "model_name": args.model_name,
        "engage_feedback": getattr(args, "engage_feedback", True),
        "hand_id": getattr(args, "hand", None),
    }
    options.update(overrides)
    return create_hand(args.config_path, **options)


def connect_hand(hand, *, interactive: bool = True) -> None:
    success, message = hand.connect(interactive=interactive)
    print(f"connect() -> success={success}, message={message}")
    if not success:
        raise RuntimeError(message)


def print_calibration_progress(event: dict) -> None:
    """Render calibration progress events on the terminal."""
    name = event.get("event")
    if name == "calibration_started":
        print(
            f"Calibrating {len(event['joints'])} joint(s) "
            f"over {event['steps']} step(s)..."
        )
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
        print("Wrist already calibrated; skipping wrist steps (--force-wrist overrides).")
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
    elif name == "calibration_done":
        print("Calibration complete.")
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
