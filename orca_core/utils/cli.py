# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Shared CLI helpers for the operator scripts and examples."""

import select
import sys
import threading
from argparse import ArgumentParser, Namespace
from pathlib import Path

from orca_core import BaseHand, load_hand


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
) -> BaseHand:
    """Build the hand class matching the selected — or detected — model.

    With no ``config_path`` or ``model_name`` on a physical hand this probes
    the hardware, so the model matches what is actually plugged in rather than
    the packaged default.
    """
    hand = load_hand(
        config_path=config_path,
        mock=use_mock,
        model_name=model_name,
        engage_feedback=engage_feedback,
        engage_sensors=engage_sensors,
    )
    print(f"Loaded {type(hand).__name__} from {hand.config.config_path}")
    return hand


def create_hand_from_args(args: Namespace, **overrides) -> BaseHand:
    """Build the hand every argument in :func:`add_hand_arguments` selects.

    Front-ends call this instead of :func:`create_hand` so a flag can never be
    advertised and then dropped. ``overrides`` pin what the front-end decides
    itself, e.g. ``engage_feedback=False`` for a routine that must drive the
    motors open-loop.
    """
    options = {
        "use_mock": args.mock,
        "model_name": args.model_name,
        "engage_feedback": getattr(args, "engage_feedback", True),
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
    elif name == "calibration_done":
        boosted = event.get("boosted_joints") or {}
        if boosted:
            joints = ", ".join(
                f"{j} @ {c:.0f} mA" for j, c in sorted(boosted.items())
            )
            print(f"Needed a higher-current re-drive: {joints}")
        print("Calibration complete.")
    elif name == "calibration_aborted":
        print("Calibration aborted.")
    elif name == "cleanup_failed":
        print(f"WARNING: cleanup after abort failed: {event['error']}")


def pose_without(hand, pose, skip=()) -> dict:
    """A ``{joint: angle}`` pose from an array aligned with ``joint_ids``,
    leaving out ``skip``.

    Joints omitted from the mapping are never commanded and hold whatever pose
    they are in, which is how a recording made on a whole hand is replayed on
    one with a joint out of action.

    Raises:
        ValueError: ``skip`` names a joint the hand does not have, or ``pose``
            is not aligned with ``joint_ids``.
    """
    joint_ids = list(hand.config.joint_ids)
    if len(pose) != len(joint_ids):
        raise ValueError(
            f"pose has {len(pose)} values but the hand has {len(joint_ids)} joints"
        )
    unknown = [j for j in skip if j not in joint_ids]
    if unknown:
        raise ValueError(
            f"unknown joint(s) {', '.join(unknown)}; this hand has "
            f"{', '.join(joint_ids)}"
        )
    skipped = set(skip)
    return {
        joint: float(value)
        for joint, value in zip(joint_ids, pose)
        if joint not in skipped
    }


class KeyListener:
    """Deliver single keypresses to a callback while a script runs.

    Puts the terminal in cbreak so keys arrive without Enter and are not
    echoed, and restores it on :meth:`stop`. Ctrl-C keeps working — cbreak
    leaves signal generation on — so a long run is still interruptible.

    Inert when stdin is not a terminal (piped input, a service) or on a
    platform without ``termios``, so a script driven non-interactively behaves
    exactly as it did before rather than failing.
    """

    def __init__(self, on_key):
        self._on_key = on_key
        self._thread = None
        self._stop = threading.Event()
        self._fd = None
        self._saved = None

    @property
    def listening(self) -> bool:
        return self._thread is not None

    def start(self) -> "KeyListener":
        try:
            import termios
            import tty
        except ImportError:
            return self
        if not sys.stdin.isatty():
            return self
        self._fd = sys.stdin.fileno()
        try:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except Exception:
            self._fd = self._saved = None
            return self
        self._thread = threading.Thread(target=self._pump, name="keys", daemon=True)
        self._thread.start()
        return self

    def _pump(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not ready:
                    continue
                key = sys.stdin.read(1)
            except Exception:
                return
            if key:
                try:
                    self._on_key(key)
                except Exception:
                    pass

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self._fd is not None and self._saved is not None:
            try:
                import termios
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            except Exception:
                pass
            self._fd = self._saved = None


def print_pacing_report(stats, csv_path=None) -> None:
    """Print a command-cadence summary from :class:`PacingMonitor.stats`.

    ``stats`` may be ``None`` when too few commands were issued to measure.
    """
    if stats is None:
        print("\nNo pacing data: too few commands issued to measure a cadence.")
        return
    ms = lambda seconds: f"{seconds * 1000:.1f}"
    print("\nCommand pacing")
    print(f"  commands         {stats.commands} at a {ms(stats.target_period_s)} ms target")
    print(
        f"  interval         median {ms(stats.interval_median_s)}  "
        f"p95 {ms(stats.interval_p95_s)}  p99 {ms(stats.interval_p99_s)}  "
        f"max {ms(stats.interval_max_s)} ms"
    )
    print(
        f"  command call     median {ms(stats.call_median_s)}  "
        f"p95 {ms(stats.call_p95_s)}  max {ms(stats.call_max_s)} ms"
    )
    print(f"  late / stalled   {stats.late} / {stats.stalls}")
    if stats.loop_cycles_overrun is not None:
        print(
            f"  joint loop       {stats.loop_cycles_overrun} overrun cycles, "
            f"{stats.loop_cycles_exception} exceptions during the run"
        )
    print(f"  verdict          {stats.verdict}")
    if csv_path is not None:
        print(f"  written to       {csv_path}")


def print_trace_report(summary, csv_path=None) -> None:
    """Print a joint-loop trace summary from :class:`LoopTracer.summary`."""
    if summary is None:
        print("\nNo trace data captured.")
        return
    print("\nJoint-loop trace")
    print(
        f"  samples          {summary.samples} over {summary.duration_s:.1f} s "
        f"({summary.achieved_hz:.0f} Hz achieved)"
    )
    print(
        f"  loop counters    overrun {summary.cycles_overrun}  "
        f"exception {summary.cycles_exception}  no-reading {summary.cycles_no_reading}  "
        f"held {summary.cycles_held}  e-stops {summary.e_stops}"
    )
    clamp = summary.correction_clamp_deg
    print(f"  correction clamp {'unknown' if clamp is None else f'{clamp:.1f} deg'}")
    print(
        f"  {'joint':<12} {'|corr| max':>10} {'|corr| p99':>10} "
        f"{'at clamp':>9} {'max jump':>9} {'still':>8} {'temp max':>9}"
    )
    for joint in summary.joints:
        temp = "-" if joint.temp_max_c is None else f"{joint.temp_max_c:.0f}C"
        print(
            f"  {joint.joint:<12} {joint.corr_max_abs:10.2f} {joint.corr_p99_abs:10.2f} "
            f"{joint.near_clamp_fraction * 100:8.1f}% {joint.angle_max_jump_deg:9.2f} "
            f"{joint.angle_still_longest_s:7.2f}s {temp:>9}"
        )
    print(
        "  a correction riding near the clamp then snapping back is the "
        "integrator winding up against stiction"
    )
    if summary.marks:
        print(f"  observed         {len(summary.marks)} marker(s):")
        for at, label in summary.marks:
            print(f"                     t={at:8.2f} s  {label}")
    if csv_path is not None:
        print(f"  written to       {csv_path}")


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
