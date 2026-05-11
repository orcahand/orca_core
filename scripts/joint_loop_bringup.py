"""Single-finger bring-up CLI for the host-side joint loop.

Loads a calibrated hand, opens the encoder link, starts the closed-loop
joint controller, and prints per-second loop stats and per-joint
measurements. Stdin accepts live commands so a single session can probe
step responses and retune gains without restarting:

    set <joint> <angle_rad>      update one joint target
    gains <a> <b> <c>            retune the controller (mode-dependent —
                                 see the meaning printed at startup)
    quit                         clean shutdown

The third positional ``gains`` argument is mode-dependent:

  * current_pid: ``Kd`` in mA·s/rad.
  * cascaded:    ``correction_max`` in degrees (also used as the
                 integrator clamp in radians).

Usage:
    uv run python scripts/joint_loop_bringup.py path/to/config.yaml \\
        --encoder-port /dev/cu.usbmodem103 --joint-control-mode cascaded
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import threading
import time

from orca_core.control import CascadedJointController, JointPIDController
from orca_core.control.constants import (
    DEFAULT_CASCADED_CORRECTION_MAX_RAD,
    DEFAULT_CASCADED_KI_RAD_PER_RAD_S,
    DEFAULT_CASCADED_KP_RAD_PER_RAD,
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
        description="Single-finger bring-up CLI for the closed-loop joint loop."
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
        "--joint-control-mode",
        choices=["current_pid", "cascaded"],
        default=None,
        help="Override config joint_control_mode. Default: use config value.",
    )
    p.add_argument(
        "--max-current",
        type=int,
        default=None,
        help="Override config max_current (mA). For cascaded the new value is "
             "also pushed to the motors after connect so the inner CBP loop "
             "picks up the higher torque cap. Default: use config value.",
    )
    p.add_argument(
        "--Kp",
        type=float,
        default=None,
        help="Override Kp. Default: mode-specific constant.",
    )
    p.add_argument(
        "--Ki",
        type=float,
        default=None,
        help="Override Ki. Default: mode-specific constant.",
    )
    p.add_argument(
        "--Kd",
        type=float,
        default=None,
        help="current_pid: Kd (mA·s/rad). cascaded: correction_max in degrees. "
             "Default: mode-specific constant.",
    )
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


def _apply_gains(hand: OrcaHandJointFeedback, a: float, b: float, c: float) -> str:
    """Push three positional gain values onto the active controller.

    For ``JointPIDController`` ``c`` is Kd; for ``CascadedJointController``
    ``c`` is the correction-max in degrees, also reused as the integrator
    clamp.
    """
    if isinstance(hand._pid, CascadedJointController):
        correction_max_rad = math.radians(c)
        hand._pid.set_gains(
            Kp=a, Ki=b,
            correction_max_rad=correction_max_rad,
            i_clamp_rad=correction_max_rad,
        )
        return f"cascaded: Kp={a} Ki={b} correction_max={c}°"
    if isinstance(hand._pid, JointPIDController):
        i_max = float(hand.config.max_current)
        hand._pid.set_gains(
            Kp=a, Ki=b, Kd=c, i_clamp_mA=i_max, i_max_mA=i_max,
        )
        return f"current_pid: Kp={a} Ki={b} Kd={c} i_max={i_max} mA"
    raise RuntimeError(f"unknown controller type: {type(hand._pid).__name__}")


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
        a, b, c = float(parts[1]), float(parts[2]), float(parts[3])
        summary = _apply_gains(hand, a, b, c)
        print(f"  → gains {summary}")
        return True
    print(f"  unknown command: {line.strip()!r}")
    print("  usage: set <joint> <angle_rad> | gains <a> <b> <c> | quit")
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
    if args.joint_control_mode is not None:
        overrides["joint_control_mode"] = args.joint_control_mode
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

    mode = hand.config.joint_control_mode
    if mode == "cascaded":
        a = DEFAULT_CASCADED_KP_RAD_PER_RAD if args.Kp is None else args.Kp
        b = DEFAULT_CASCADED_KI_RAD_PER_RAD_S if args.Ki is None else args.Ki
        c = (
            math.degrees(DEFAULT_CASCADED_CORRECTION_MAX_RAD)
            if args.Kd is None else args.Kd
        )
    else:
        a = DEFAULT_KP_MA_PER_RAD if args.Kp is None else args.Kp
        b = DEFAULT_KI_MA_PER_RAD_S if args.Ki is None else args.Ki
        c = DEFAULT_KD_MA_S_PER_RAD if args.Kd is None else args.Kd
    print(f"  → gains {_apply_gains(hand, a, b, c)}")

    third_arg_label = (
        "correction_max_deg" if mode == "cascaded" else "Kd"
    )
    print(
        f"Joint loop running ({mode}). Commands: "
        f"`set <joint> <rad>`, `gains <Kp> <Ki> <{third_arg_label}>`, `quit`."
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
