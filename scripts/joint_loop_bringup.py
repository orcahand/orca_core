"""Single-finger bring-up CLI for the host-side joint loop.

Loads a calibrated hand, opens the encoder link, starts the closed-loop
joint PID, and prints per-second loop stats and per-joint measurements.
Stdin accepts live commands so a single session can probe step responses
and retune gains without restarting:

    set <joint> <angle_rad>     update one joint target
    gains <Kp> <Ki> <Kd>        retune the (vectorised) PID
    quit                        clean shutdown

Usage:
    uv run python scripts/joint_loop_bringup.py path/to/config.yaml \\
        --encoder-port /dev/cu.usbmodem103 --Kp 200
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import threading
import time

from orca_core.control.constants import (
    DEFAULT_KD_MA_S_PER_RAD,
    DEFAULT_KI_MA_PER_RAD_S,
    DEFAULT_KP_MA_PER_RAD,
)
from orca_core.hardware_hand_joint_feedback import (
    OrcaHandJointFeedback,
    OrcaHandJointFeedbackError,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Single-finger bring-up CLI for the closed-loop joint PID."
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
    p.add_argument("--Kp", type=float, default=DEFAULT_KP_MA_PER_RAD)
    p.add_argument("--Ki", type=float, default=DEFAULT_KI_MA_PER_RAD_S)
    p.add_argument("--Kd", type=float, default=DEFAULT_KD_MA_S_PER_RAD)
    return p.parse_args()


def _print_stats_row(hand: OrcaHandJointFeedback) -> None:
    stats = hand._loop.get_stats()
    measured_rad = hand._loop.get_measured_joints()
    measured = "  ".join(
        f"{joint}={math.degrees(angle):+7.1f}°"
        for joint, angle in sorted(measured_rad.items())
    )
    print(
        f"cycles_ok={stats['cycles_ok']:>6}  "
        f"last_dt={stats['last_dt_s'] * 1000:6.2f}ms  "
        f"e_stops={stats['e_stops']}  "
        f"fallback={stats['fallback_active']}  | {measured}"
    )


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
        print(f"  → target[{joint}] = {angle:.4f} rad")
        return True
    if cmd == "gains" and len(parts) == 4:
        kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
        i_max = float(hand.config.max_current)
        hand._pid.set_gains(Kp=kp, Ki=ki, Kd=kd, i_clamp_mA=i_max, i_max_mA=i_max)
        print(f"  → gains Kp={kp} Ki={ki} Kd={kd} i_max={i_max} mA")
        return True
    print(f"  unknown command: {line.strip()!r}")
    print("  usage: set <joint> <angle_rad> | gains <Kp> <Ki> <Kd> | quit")
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
    if args.encoder_port is not None:
        hand.config = dataclasses.replace(
            hand.config, encoder_serial_port=args.encoder_port,
        )

    try:
        success, msg = hand.connect()
    except OrcaHandJointFeedbackError as exc:
        print(f"FAIL: {exc}")
        return 1
    print(msg)
    if not success:
        return 1

    if (args.Kp, args.Ki, args.Kd) != (
        DEFAULT_KP_MA_PER_RAD, DEFAULT_KI_MA_PER_RAD_S, DEFAULT_KD_MA_S_PER_RAD,
    ):
        i_max = float(hand.config.max_current)
        hand._pid.set_gains(
            Kp=args.Kp, Ki=args.Ki, Kd=args.Kd,
            i_clamp_mA=i_max, i_max_mA=i_max,
        )

    print(
        "Joint loop running. Commands: "
        "`set <joint> <rad>`, `gains <Kp> <Ki> <Kd>`, `quit`."
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
