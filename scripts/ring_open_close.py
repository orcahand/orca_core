"""Open / close cycle on ring_mcp + ring_pip via ``OrcaHandJointFeedback``.

Targets are ramped smoothly between an "open" pose and a "close" pose so
the controller tracks in its linear regime instead of saturating on a
step input. Only ring_mcp and ring_pip move; other fingers stay put.

Usage:
    uv run python scripts/ring_open_close.py \\
        orca_core/models/v2/orcahand_right/config.yaml --max-current 600
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
import threading
import time
from typing import Dict

from orca_core.control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from orca_core.hardware_hand_joint_feedback import (
    OrcaHandJointFeedback,
    OrcaHandJointFeedbackError,
)


DEFAULT_OPEN_RING_MCP_DEG = 0.0
DEFAULT_OPEN_RING_PIP_DEG = 10.0
DEFAULT_CLOSE_RING_MCP_DEG = 40.0
DEFAULT_CLOSE_RING_PIP_DEG = 85.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    p.add_argument(
        "model_path", nargs="?", default=None,
        help="Path to the hand config.yaml (defaults to the bundled model).",
    )
    p.add_argument(
        "--encoder-port", default=None,
        help='Override config encoder_serial_port. "auto" runs discovery.',
    )
    p.add_argument(
        "--max-current", type=int, default=None,
        help="Override config max_current (mA). Pushed to motors after connect "
             "so the inner CBP loop picks up the higher torque cap.",
    )
    p.add_argument(
        "--Kp", type=float, default=DEFAULT_KP,
        help="Proportional gain (deg/deg).",
    )
    p.add_argument(
        "--Ki", type=float, default=DEFAULT_KI,
        help="Integral gain (deg/deg/s).",
    )
    p.add_argument(
        "--correction-max-deg", type=float, default=DEFAULT_CORRECTION_MAX_DEG,
        help="Output clamp on the trim correction (degrees).",
    )
    p.add_argument(
        "--ramp-duration", type=float, default=2.0,
        help="Seconds to ramp between open and close poses.",
    )
    p.add_argument(
        "--hold-duration", type=float, default=2.0,
        help="Seconds to hold each end pose.",
    )
    p.add_argument(
        "--open-mcp", type=float, default=DEFAULT_OPEN_RING_MCP_DEG,
        help="Open-pose ring_mcp target (degrees).",
    )
    p.add_argument(
        "--open-pip", type=float, default=DEFAULT_OPEN_RING_PIP_DEG,
        help="Open-pose ring_pip target (degrees). Stay above the relaxed "
             "extension (~+10°) so the spring can reach it.",
    )
    p.add_argument(
        "--close-mcp", type=float, default=DEFAULT_CLOSE_RING_MCP_DEG,
        help="Close-pose ring_mcp target (degrees).",
    )
    p.add_argument(
        "--close-pip", type=float, default=DEFAULT_CLOSE_RING_PIP_DEG,
        help="Close-pose ring_pip target (degrees).",
    )
    return p.parse_args()


def _ramp(
    hand: OrcaHandJointFeedback,
    start: Dict[str, float],
    end: Dict[str, float],
    duration_s: float,
    stop_event: threading.Event,
) -> None:
    """Linearly interpolate the loop target between two poses (degrees)."""
    deadline = time.monotonic() + duration_s
    while not stop_event.is_set():
        now = time.monotonic()
        if now >= deadline:
            break
        alpha = 1.0 - (deadline - now) / duration_s
        target = {
            joint: start[joint] + alpha * (end[joint] - start[joint])
            for joint in start
        }
        hand._loop.set_target(target)
        time.sleep(0.01)
    if not stop_event.is_set():
        hand._loop.set_target(dict(end))


def _hold(
    hand: OrcaHandJointFeedback,
    pose: Dict[str, float],
    duration_s: float,
    stop_event: threading.Event,
) -> None:
    hand._loop.set_target(dict(pose))
    end = time.monotonic() + duration_s
    while not stop_event.is_set() and time.monotonic() < end:
        time.sleep(0.05)


def _print_status(hand: OrcaHandJointFeedback, label: str) -> None:
    measured = hand._loop.get_measured_joints()
    correction = hand._loop.get_correction()
    stats = hand._loop.get_stats()
    motor_pos_array = hand.get_motor_pos()
    motor_id_to_idx = hand.config.motor_id_to_idx_dict

    parts = [f"{label:<8}"]
    for joint in ("ring_mcp", "ring_pip"):
        idx = hand._loop._joint_names.index(joint)
        motor_id = hand._loop._motor_ids[idx]
        actual_motor = motor_pos_array[motor_id_to_idx[motor_id]]
        parts.append(
            f"{joint}: enc={measured.get(joint, float('nan')):+6.1f}° "
            f"trim={correction.get(joint, 0.0):+5.1f}° "
            f"M_act={actual_motor:+6.2f}"
        )
    parts.append(f"cyc={stats['cycles_ok']:>5} fb={stats['fallback_active']}")
    print("  ".join(parts))


def _stats_loop(
    hand: OrcaHandJointFeedback,
    label_box: list,
    stop_event: threading.Event,
) -> None:
    while not stop_event.is_set():
        _print_status(hand, label_box[0])
        if stop_event.wait(timeout=1.0):
            return


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s"
    )

    hand = OrcaHandJointFeedback(config_path=args.model_path)
    overrides = {}
    if args.encoder_port is not None:
        overrides["encoder_serial_port"] = args.encoder_port
    if args.max_current is not None:
        overrides["max_current"] = args.max_current
    if overrides:
        hand.config = dataclasses.replace(hand.config, **overrides)

    try:
        success, msg = hand.connect()
    except OrcaHandJointFeedbackError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(msg)
    if not success:
        return 1

    if args.max_current is not None:
        hand.set_max_current(args.max_current)
        print(f"  → max_current = {args.max_current} mA")

    hand._controller.set_gains(
        Kp=args.Kp, Ki=args.Ki,
        correction_max_deg=args.correction_max_deg,
        i_clamp_deg=args.correction_max_deg,
    )
    print(
        f"  → gains Kp={args.Kp} Ki={args.Ki} "
        f"correction_max={args.correction_max_deg:.1f}°"
    )

    open_pose = {"ring_mcp": args.open_mcp, "ring_pip": args.open_pip}
    close_pose = {"ring_mcp": args.close_mcp, "ring_pip": args.close_pip}
    print(
        f"  open  ring_mcp={open_pose['ring_mcp']:+.0f}° "
        f"ring_pip={open_pose['ring_pip']:+.0f}°"
    )
    print(
        f"  close ring_mcp={close_pose['ring_mcp']:+.0f}° "
        f"ring_pip={close_pose['ring_pip']:+.0f}°"
    )
    print(
        f"  ramp={args.ramp_duration}s  hold={args.hold_duration}s. "
        "Ctrl+C to stop."
    )

    stop_event = threading.Event()
    label_box = ["init"]
    stats_thread = threading.Thread(
        target=_stats_loop, args=(hand, label_box, stop_event),
        name="stats", daemon=True,
    )
    stats_thread.start()

    try:
        # Start each cycle from the current measured pose so the first
        # ramp doesn't yank from an arbitrary start.
        measured = hand._loop.get_measured_joints()
        current = {j: measured.get(j, 0.0) for j in open_pose}
        while not stop_event.is_set():
            label_box[0] = "→close"
            _ramp(hand, current, close_pose, args.ramp_duration, stop_event)
            label_box[0] = "hold C"
            _hold(hand, close_pose, args.hold_duration, stop_event)
            label_box[0] = "→open"
            _ramp(hand, close_pose, open_pose, args.ramp_duration, stop_event)
            label_box[0] = "hold O"
            _hold(hand, open_pose, args.hold_duration, stop_event)
            current = dict(open_pose)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_event.set()
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
