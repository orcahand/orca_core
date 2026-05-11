"""Single-finger bring-up CLI for the host-side joint loop.

Loads a calibrated hand, opens the encoder link, starts the joint
controller, and prints per-second loop stats and per-joint measurements.
Stdin accepts live commands so a session can probe step responses and
retune without restarting:

    set <joint> <angle_deg>           update one joint target
    gains <Kp> <Ki> <corr_max_deg>    retune the PI controller
    quit                              clean shutdown

Usage:
    uv run python scripts/joint_loop_bringup.py path/to/config.yaml \\
        --encoder-port /dev/cu.usbmodem103
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import sys
import threading
import time

from orca_core.control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_KI,
    DEFAULT_KP,
)
from orca_core.hardware_hand_joint_feedback import (
    OrcaHandJointFeedback,
    OrcaHandJointFeedbackError,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Single-finger bring-up CLI for the host-side joint loop."
    )
    p.add_argument(
        "model_path",
        nargs="?",
        default=None,
        help="Path to the hand config.yaml (defaults to the bundled model).",
    )
    p.add_argument(
        "--encoder-port",
        default=None,
        help='Override config encoder_serial_port. "auto" runs discovery; '
             'an explicit path bypasses; "disabled" forces None.',
    )
    p.add_argument(
        "--max-current",
        type=int,
        default=None,
        help="Override config max_current (mA) and push the new cap to "
             "motors so the inner CBP loop picks it up.",
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
    return p.parse_args()


def _print_stats_row(hand: OrcaHandJointFeedback) -> None:
    stats = hand._loop.get_stats()
    measured = hand._loop.get_measured_joints()
    measured_str = "  ".join(
        f"{joint}={angle:+7.1f}°" for joint, angle in sorted(measured.items())
    )
    print(
        f"cycles_ok={stats['cycles_ok']:>6}  "
        f"last_dt={stats['last_dt_s'] * 1000:6.2f}ms  "
        f"e_stops={stats['e_stops']}  "
        f"fallback={stats['fallback_active']}  | {measured_str}"
    )


def _apply_gains(
    hand: OrcaHandJointFeedback, kp: float, ki: float, corr_max_deg: float,
) -> str:
    hand._controller.set_gains(
        Kp=kp, Ki=ki,
        correction_max_deg=corr_max_deg,
        i_clamp_deg=corr_max_deg,
    )
    return f"Kp={kp} Ki={ki} correction_max={corr_max_deg:.1f}°"


def _handle_command(line: str, hand: OrcaHandJointFeedback) -> bool:
    """Dispatch one stdin command. Returns False to request shutdown."""
    parts = line.strip().split()
    if not parts:
        return True
    cmd = parts[0].lower()
    if cmd in ("quit", "q", "exit"):
        return False
    if cmd == "set" and len(parts) == 3:
        joint, angle = parts[1], float(parts[2])
        hand._loop.set_target({joint: angle})
        print(f"  → target[{joint}] = {angle:.2f}°")
        return True
    if cmd == "gains" and len(parts) == 4:
        kp, ki, corr = float(parts[1]), float(parts[2]), float(parts[3])
        print(f"  → gains {_apply_gains(hand, kp, ki, corr)}")
        return True
    print(f"  unknown command: {line.strip()!r}")
    print("  usage: set <joint> <angle_deg> | gains <Kp> <Ki> <corr_max_deg> | quit")
    return True


def _stdin_loop(hand: OrcaHandJointFeedback, stop_event: threading.Event) -> None:
    for line in sys.stdin:
        if not _handle_command(line, hand):
            stop_event.set()
            return
    stop_event.set()


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

    print(f"  → gains {_apply_gains(hand, args.Kp, args.Ki, args.correction_max_deg)}")
    print(
        "Joint loop running. Commands: "
        "`set <joint> <deg>`, `gains <Kp> <Ki> <corr_max_deg>`, `quit`."
    )

    stop_event = threading.Event()
    stdin_thread = threading.Thread(
        target=_stdin_loop, args=(hand, stop_event), name="stdin", daemon=True,
    )
    stdin_thread.start()

    try:
        while not stop_event.is_set():
            _print_stats_row(hand)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
