"""Cycle ring_pip between extend and curl poses under PID.

Drives ring_pip through a cosine-eased setpoint trajectory so the joint
loop tracks a smooth target rather than a step. ring_mcp is left
nominally on the loop but with non-aggressive gains (P-only, no
integrator) so an unreliable encoder there can't drive the motor into
bang-bang. Other joints hold whatever pose they were in when the hand
connected. Used to eyeball PID smoothness on a sustained motion.

Usage:
    uv run python scripts/curl_extend_ring.py \\
        orca_core/models/v2/orcahand_right/config.yaml
"""
from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import sys
import threading
import time

import numpy as np

from orca_core.control.constants import (
    DEFAULT_KD_MA_S_PER_RAD,
    DEFAULT_KP_MA_PER_RAD,
)
from orca_core.hardware_hand_joint_feedback import (
    OrcaHandJointFeedback,
    OrcaHandJointFeedbackError,
)


PIP_EXTEND_DEG = -10.0
PIP_CURL_DEG = 100.0

# ring_mcp stays on the loop but hold-only — no integrator wind-up
# against encoder noise.
MCP_HOLD_KP = 50.0
MCP_HOLD_KI = 0.0
MCP_HOLD_KD = 5.0

RAMP_UPDATE_INTERVAL_S = 0.01
STATS_INTERVAL_S = 1.0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Cycle the ring finger between extend and curl under PID."
    )
    p.add_argument(
        "model_path", nargs="?", default=None,
        help="Path to the hand config.yaml (defaults to the bundled model).",
    )
    p.add_argument(
        "--encoder-port", default=None,
        help='Override config encoder_serial_port. "auto" runs discovery; '
             'an explicit path bypasses; "disabled" forces None.',
    )
    p.add_argument("--Kp", type=float, default=DEFAULT_KP_MA_PER_RAD,
                   help="Proportional gain for ring_pip.")
    p.add_argument(
        "--Ki", type=float, default=100.0,
        help="Integral gain for ring_pip. Some Ki is needed to integrate out "
             "tendon stiction; too much produces a limit cycle. Tune live "
             "with `gains`.",
    )
    p.add_argument("--Kd", type=float, default=10.0,
                   help="Derivative gain for ring_pip. Damps bang-bang at the "
                        "stiction breakouts.")
    p.add_argument(
        "--ramp-s", type=float, default=2.0,
        help="Seconds spent ramping from extend to curl (cosine-eased).",
    )
    p.add_argument(
        "--hold-s", type=float, default=1.0,
        help="Seconds to hold each end pose before reversing.",
    )
    p.add_argument(
        "--reps", type=int, default=0,
        help="Number of curl/extend cycles. 0 means run forever (until q+Enter).",
    )
    return p.parse_args()


def _smoothstep(t: float) -> float:
    """Cosine ease-in/ease-out on ``t`` ∈ [0, 1]. Velocity is zero at both
    endpoints and varies continuously in between, so the PID sees a smooth
    setpoint trajectory instead of a step."""
    return 0.5 - 0.5 * math.cos(math.pi * t)


def _interp(a: float, b: float, t: float) -> float:
    return a + (b - a) * _smoothstep(t)


def _ramp_target(
    hand: OrcaHandJointFeedback,
    src_rad: dict[str, float],
    dst_rad: dict[str, float],
    duration_s: float,
    stop_event: threading.Event,
) -> None:
    """Smoothly ramp the loop target from ``src_rad`` to ``dst_rad`` over
    ``duration_s``, updating every 10 ms with a cosine ease-in/ease-out
    profile."""
    start = time.monotonic()
    while not stop_event.is_set():
        elapsed = time.monotonic() - start
        t = min(1.0, elapsed / duration_s)
        target = {j: _interp(src_rad[j], dst_rad[j], t) for j in src_rad}
        hand._loop.set_target(target)
        if t >= 1.0:
            return
        time.sleep(RAMP_UPDATE_INTERVAL_S)


def _hold(stop_event: threading.Event, duration_s: float) -> None:
    end = time.monotonic() + duration_s
    while time.monotonic() < end and not stop_event.is_set():
        time.sleep(0.02)


def _apply_gains(
    hand: OrcaHandJointFeedback,
    pip_idx: int,
    mcp_idx: int | None,
    pip_kp: float, pip_ki: float, pip_kd: float,
) -> None:
    """Set per-joint gains: pip from CLI / live tuning, mcp (when present)
    held at the safe hold-only profile so a flaky encoder there can't push
    the motor."""
    n = hand._pid.num_joints
    Kp = np.full(n, pip_kp); Ki = np.full(n, pip_ki); Kd = np.full(n, pip_kd)
    if mcp_idx is not None:
        Kp[mcp_idx] = MCP_HOLD_KP
        Ki[mcp_idx] = MCP_HOLD_KI
        Kd[mcp_idx] = MCP_HOLD_KD
    i_max = float(hand.config.max_current)
    hand._pid.set_gains(
        Kp=Kp, Ki=Ki, Kd=Kd,
        i_clamp_mA=i_max, i_max_mA=i_max,
    )


def _stdin_loop(
    hand: OrcaHandJointFeedback,
    pip_idx: int,
    mcp_idx: int | None,
    stop_event: threading.Event,
) -> None:
    for line in sys.stdin:
        parts = line.strip().split()
        if not parts:
            continue
        cmd = parts[0].lower()
        if cmd in ("q", "quit", "exit"):
            stop_event.set()
            return
        if cmd == "gains" and len(parts) == 4:
            try:
                kp, ki, kd = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                print(f"  could not parse gains: {parts[1:]}")
                continue
            _apply_gains(hand, pip_idx, mcp_idx, kp, ki, kd)
            print(f"  → ring_pip gains: Kp={kp} Ki={ki} Kd={kd}")
            continue
        print(f"  unknown command: {line.strip()!r}")
        print("  usage: gains <Kp> <Ki> <Kd> | quit")
    stop_event.set()


def _stats_loop(
    hand: OrcaHandJointFeedback,
    label_box: list[str],
    stop_event: threading.Event,
) -> None:
    while not stop_event.is_set():
        stats = hand._loop.get_stats()
        measured = hand._loop.get_measured_joints()
        bits = "  ".join(
            f"{j}={math.degrees(angle):+7.1f}°"
            for j, angle in sorted(measured.items())
        )
        print(
            f"[{label_box[0]:<6}] cycles_ok={stats['cycles_ok']:>6}  "
            f"last_dt={stats['last_dt_s'] * 1000:5.2f}ms  "
            f"e_stops={stats['e_stops']}  fallback={stats['fallback_active']}  | {bits}"
        )
        if stats["fallback_active"]:
            stop_event.set()
            return
        time.sleep(STATS_INTERVAL_S)


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

    joint_names = list(hand._loop._joint_names)
    if "ring_pip" not in joint_names:
        print(f"FAIL: ring_pip is not encoder-backed for this hand "
              f"(snapshotted joints: {joint_names}).")
        hand.disconnect()
        return 1
    pip_idx = joint_names.index("ring_pip")
    mcp_idx = joint_names.index("ring_mcp") if "ring_mcp" in joint_names else None

    _apply_gains(hand, pip_idx, mcp_idx, args.Kp, args.Ki, args.Kd)
    print(f"ring_pip gains: Kp={args.Kp} Ki={args.Ki} Kd={args.Kd}")
    if mcp_idx is not None:
        print(f"ring_mcp gains: Kp={MCP_HOLD_KP} Ki={MCP_HOLD_KI} Kd={MCP_HOLD_KD}  "
              "(hold-only — driven on the loop but no integrator wind-up)")
    print(f"Cycle: ramp={args.ramp_s}s, hold={args.hold_s}s, "
          f"reps={'∞' if args.reps == 0 else args.reps}")
    print(f"  extend ring_pip = {PIP_EXTEND_DEG}°")
    print(f"  curl   ring_pip = {PIP_CURL_DEG}°")
    print("Commands: `gains <Kp> <Ki> <Kd>` (ring_pip), `quit`.\n")

    extend_rad = {"ring_pip": math.radians(PIP_EXTEND_DEG)}
    curl_rad = {"ring_pip": math.radians(PIP_CURL_DEG)}

    stop_event = threading.Event()
    label_box = ["INIT"]

    threading.Thread(
        target=_stdin_loop, args=(hand, pip_idx, mcp_idx, stop_event),
        name="stdin", daemon=True,
    ).start()
    threading.Thread(
        target=_stats_loop, args=(hand, label_box, stop_event),
        name="stats", daemon=True,
    ).start()

    try:
        measured = hand._loop.get_measured_joints()
        prev = {"ring_pip": measured.get("ring_pip", 0.0)}

        rep = 0
        while not stop_event.is_set():
            if args.reps and rep >= args.reps:
                break
            rep += 1
            for label, dst in (("EXTEND", extend_rad), ("CURL", curl_rad)):
                if stop_event.is_set():
                    break
                label_box[0] = f"{label} ↦"
                _ramp_target(hand, prev, dst, args.ramp_s, stop_event)
                if stop_event.is_set():
                    break
                label_box[0] = f"{label} ⏸"
                _hold(stop_event, args.hold_s)
                prev = dict(dst)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop_event.set()
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
