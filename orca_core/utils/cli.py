# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Shared CLI helpers for the operator scripts and examples."""

import contextlib
import io
import logging
import sys
from argparse import ArgumentParser, Namespace
from pathlib import Path

from orca_core import (
    AmbiguousHandError,
    BaseHand,
    HandFleet,
    HandSelectionError,
    HandSelector,
    detect_hands,
    load_hand,
    load_hands,
)


def add_hand_arguments(
    parser: ArgumentParser,
    *,
    mock_default: bool = False,
    feedback_flag: bool = True,
    all_flag: bool = False,
) -> None:
    """Add the shared hand-selection arguments.

    ``feedback_flag=False`` omits ``--no-engage-feedback`` for front-ends that
    always drive the motors open-loop, so the flag is never advertised where it
    could not be honoured. ``all_flag=True`` adds ``--all`` for front-ends that
    can run their operation across every attached hand via
    :func:`create_fleet_from_args`; omitted by default so a front-end that
    only ever builds one hand (:func:`create_hand_from_args`) can't advertise
    a flag it would silently ignore.
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
    add_hand_selection_arguments(parser, all_flag=all_flag)
    if feedback_flag:
        parser.add_argument(
            "--no-engage-feedback",
            dest="engage_feedback",
            action="store_false",
            default=True,
            help="Load the motor-only hand even when the config enables joint feedback.",
        )


def add_hand_selection_arguments(
    parser: ArgumentParser, *, all_flag: bool = False
) -> None:
    """Add ``--hand`` and ``--list-hands``, and optionally ``--all``.

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
    if all_flag:
        parser.add_argument(
            "--all",
            action="store_true",
            help="Use every attached hand instead of exactly one.",
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


def _choose_hand_interactively(detections, *, allow_all: bool = False):
    """Prompt for one of several attached hands, the way ``get_and_choose_port``
    prompts for a port when none was named: a numbered list, chosen with a
    number (or ``a`` for every hand, where offered).

    Returns a :class:`HandSelector`, the string ``"all"``, or ``None`` when
    stdin is not a terminal or the operator quits — the caller falls back to
    the flag-based error in that case.
    """
    if not sys.stdin.isatty():
        return None
    print("\nSeveral hands are attached:")
    for i, d in enumerate(detections, 1):
        name = d.hand_id or "unidentified"
        print(f"  {i}. {name}  {d.side} {d.model_name}")
    if allow_all:
        print(f"  a. All {len(detections)} hands")
    print("  q. Quit")
    while True:
        choice = input("Choose a hand: ").strip().lower()
        if choice in ("q", "quit", ""):
            return None
        if allow_all and choice in ("a", "all"):
            return "all"
        if choice.isdigit() and 1 <= int(choice) <= len(detections):
            return HandSelector(hand_id=detections[int(choice) - 1].hand_id)
        valid = f"1-{len(detections)}{', a,' if allow_all else ''} or q"
        print(f"Enter {valid}.")


def _explain_selection_failure(e: HandSelectionError):
    raise SystemExit(
        f"{e.summary}\n\nAttached hands:\n"
        f"{describe_attached_hands(e.detections or None)}"
        "\n\nName one with --hand HAND_ID, or --list-hands to see them."
    ) from e


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
    except AmbiguousHandError as e:
        chosen = None if hand_id is not None else _choose_hand_interactively(e.detections)
        if not isinstance(chosen, HandSelector):
            _explain_selection_failure(e)
        # e.detections already is the ambiguous set: reuse it rather than
        # probing the bus again for what was just shown on screen.
        try:
            return select_hand(e.detections, chosen).model_name
        except HandSelectionError as e2:
            _explain_selection_failure(e2)
    except HandSelectionError as e:
        _explain_selection_failure(e)


def create_hand(
    config_path: str | None,
    *,
    use_mock: bool,
    model_name: str | None = None,
    engage_feedback: bool = True,
    engage_sensors: bool = True,
    hand_id: str | None = None,
    quiet: "bool | str" = False,
) -> BaseHand:
    """Build the hand class matching the selected — or detected — model.

    With no ``config_path`` or ``model_name`` on a physical hand this probes
    the hardware, so the model matches what is actually plugged in rather than
    the packaged default. A selection failure exits with the attached hands
    listed rather than a traceback naming a Python API. ``quiet`` drops
    construction's one-time not-calibrated banner — ``True`` entirely,
    ``"sensors"`` only the parts that don't matter to a sensor-only tool (see
    :func:`_without_uncalibrated_banner`).
    """
    if hand_id is not None and use_mock:
        raise SystemExit(
            "--hand names a physical hand to detect; --mock has none to name."
        )
    build = dict(
        config_path=config_path, mock=use_mock, model_name=model_name,
        engage_feedback=engage_feedback, engage_sensors=engage_sensors,
    )
    hand, _ = _resolve_selection(build, hand_id=hand_id, allow_all=False, quiet=quiet)
    print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return hand


# Construction prints one Warning line per uncalibrated motor/joint — useful
# the first time an operator sees a hand, noise to a routine whose entire job
# is fixing exactly that (calibrate.py, tension.py, setup.py) or that never
# touches motors or joint calibration at all (check_sensors.py, check_motor.py).
_MOTOR_LIMIT_SNIPPET = "has not been fully calibrated"
_ENCODER_ANCHOR_SNIPPET = "is missing a joint-encoder calibration entry"


def _without_uncalibrated_banner(build_hand, *, mode: "bool | str" = True):
    """Run ``build_hand`` (a zero-arg call into ``load_hand``/``load_hands``)
    with its construction-time not-calibrated banner filtered from what it
    prints.

    ``mode=True`` drops every line: for a caller whose whole job is fixing
    that, or one that never touches motors or joint calibration at all,
    neither category means anything to report before it has even started.

    ``mode="sensors"`` drops only the motor-limit lines — meaningless to any
    sensor-only tool, which never touches a motor — and keeps the
    joint-encoder-anchor lines when the built hand actually declares
    feedback: a tool watching that live stream (monitor_sensors.py) benefits
    from knowing whether what it shows is anchored, which a motor-only or
    touch-only hand never has to begin with.

    Buffers and replays rather than filtering live, which is only safe
    because building a hand never calls ``input()`` — the interactive picker
    that might is always a separate call, never wrapped here, so its prompt
    is never at risk of ending up silently in this buffer.
    """
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        result = build_hand()
    keep_encoder_anchors = False
    if mode == "sensors":
        hands = result if isinstance(result, HandFleet) else (result,)
        keep_encoder_anchors = any(
            getattr(h.config, "joint_feedback_enabled", False) for h in hands
        )
    for line in buf.getvalue().splitlines():
        if _MOTOR_LIMIT_SNIPPET in line:
            continue
        if _ENCODER_ANCHOR_SNIPPET in line and not keep_encoder_anchors:
            continue
        print(line)
    return result


def _resolve_selection(
    build: dict, *, hand_id: str | None, allow_all: bool, quiet: "bool | str" = False
):
    """Resolve one hand from ``build`` (:func:`load_hand` kwargs minus
    ``select``), prompting interactively on ambiguity exactly as a bare
    ``--hand``-less invocation with two hands attached would want.

    Returns ``(hand, picked_all)``. ``hand`` is ``None`` iff ``picked_all``
    is ``True`` — the caller build every hand itself in that case. Exits the
    process (naming the attached hands and ``--hand``) when nothing resolves
    and no interactive choice was made, whether because stdin is not a
    terminal or the operator quit.
    """
    def _load(select):
        call = lambda: load_hand(select=select, **build)  # noqa: E731
        return _without_uncalibrated_banner(call, mode=quiet) if quiet else call()

    try:
        return _load(_selector_for(hand_id)), False
    except AmbiguousHandError as e:
        chosen = None if hand_id is not None else _choose_hand_interactively(
            e.detections, allow_all=allow_all
        )
        if chosen == "all":
            return None, True
        if not isinstance(chosen, HandSelector):
            _explain_selection_failure(e)
        try:
            return _load(chosen), False
        except HandSelectionError as e2:
            _explain_selection_failure(e2)
    except HandSelectionError as e:
        _explain_selection_failure(e)


def create_hand_from_args(
    args: Namespace, *, quiet: "bool | str" = False, **overrides
) -> BaseHand:
    """Build the hand every argument in :func:`add_hand_arguments` selects.

    Front-ends call this instead of :func:`create_hand` so a flag can never be
    advertised and then dropped. ``overrides`` pin what the front-end decides
    itself, e.g. ``engage_feedback=False`` for a routine that must drive the
    motors open-loop. ``quiet`` drops construction's one-time not-calibrated
    banner — see :func:`_without_uncalibrated_banner` for what ``True`` vs
    ``"sensors"`` each drop.
    """
    handle_hand_selection(args)
    options = {
        "use_mock": args.mock,
        "model_name": args.model_name,
        "engage_feedback": getattr(args, "engage_feedback", True),
        "hand_id": getattr(args, "hand", None),
    }
    options.update(overrides)
    return create_hand(args.config_path, quiet=quiet, **options)


def _load_every_hand(
    args: Namespace, overrides: dict, *, quiet: "bool | str" = False
) -> HandFleet:
    options = {
        "mock": args.mock,
        "engage_feedback": getattr(args, "engage_feedback", True),
    }
    options.update(overrides)
    call = lambda: load_hands(**options)  # noqa: E731
    try:
        fleet = _without_uncalibrated_banner(call, mode=quiet) if quiet else call()
    except HandSelectionError as e:
        _explain_selection_failure(e)
    if not fleet:
        raise SystemExit(
            "no hand is attached.\n\nAttached hands:\n" + describe_attached_hands()
        )
    for hand in fleet:
        print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return fleet


def create_fleet_from_args(
    args: Namespace, *, quiet: "bool | str" = False, **overrides
) -> HandFleet:
    """Build the fleet ``--all``/``--hand``/``config_path`` selects.

    Every case that does not name ``--all`` explicitly is exactly
    :func:`create_hand_from_args`, wrapped as a one-hand fleet — a front-end
    that never checks ``args.all`` keeps behaving as it always has. With
    ``--all``, every attached hand is built, each with its own detected
    model; ``config_path``/``--model-name`` are refused alongside it, since
    one config can't stand in for every hand's own. ``quiet=True`` drops
    construction's one-time not-calibrated banner for every hand built this
    way — for a front-end whose whole job is fixing that.

    When the parser advertised ``--all`` (``add_hand_arguments(...,
    all_flag=True)``) and nothing else narrowed the choice, an ambiguous
    bench also offers "all hands" in the interactive picker — not just one
    hand at a time.
    """
    handle_hand_selection(args)
    if getattr(args, "all", False):
        if args.config_path is not None or args.model_name is not None:
            raise SystemExit(
                "--all selects every attached hand's own model; it can't be "
                "combined with config_path or --model-name."
            )
        if getattr(args, "hand", None) is not None:
            raise SystemExit("--all and --hand are mutually exclusive; pick one.")
        return _load_every_hand(args, overrides, quiet=quiet)

    offer_all = (
        hasattr(args, "all")
        and not args.mock
        and getattr(args, "hand", None) is None
        and args.config_path is None
        and args.model_name is None
    )
    if not offer_all:
        return HandFleet((create_hand_from_args(args, quiet=quiet, **overrides),))

    build = dict(
        config_path=None, mock=False, model_name=None,
        engage_feedback=getattr(args, "engage_feedback", True), engage_sensors=True,
    )
    build.update(overrides)
    hand, picked_all = _resolve_selection(
        build, hand_id=None, allow_all=True, quiet=quiet
    )
    if picked_all:
        return _load_every_hand(args, overrides, quiet=quiet)
    print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return HandFleet((hand,))


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


@contextlib.contextmanager
def quiet_uncalibrated_warnings():
    """Suppress the "not calibrated" warning that a calibration routine's
    own position reads re-trigger after every step it commits.

    Each committed step re-arms ``OrcaHand``'s warn-once set (a
    recalibration should re-report motors still missing data), so a
    multi-step routine that commits progress after each step sees it fire
    again and again for whatever it has not reached yet. For the routine
    doing that fixing, it is not something to report on every step.
    """
    logger = logging.getLogger("orca_core.hardware_hand")
    previous = logger.level
    logger.setLevel(logging.ERROR)
    try:
        yield
    finally:
        logger.setLevel(previous)


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
