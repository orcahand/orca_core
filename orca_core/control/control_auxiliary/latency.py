"""Split the command-to-motion delay into its electrical and mechanical halves.

Time from a command to observable joint motion is the quantity that caps
closed-loop bandwidth, and it is easy to measure but easy to misattribute.
Measured at the joint it bundles two unrelated things:

``command → current``
    The servo begins drawing current the moment it acts on the new goal —
    before the shaft has turned at all. This is the transport figure: host,
    USB-to-UART bridge, serial transmission and servo processing, and the only
    part a host-side change could touch.

``current → shaft``
    Torque overcoming rotor inertia. Electromechanical.

``shaft → joint``
    Tendon slack take-up and series compliance. Immovable by software.

Detecting on *position* alone would fold the second stage into the first: a
position threshold is not crossed until the motor has accelerated across it,
which is quadratic in time and can easily exceed the delay being measured.
Current departs from its resting value as soon as the command lands, so it
isolates transport. All three come from one bus transaction, so the finer
decomposition costs no sampling rate.

The joint loop must not be running: it would interleave its own writes with
the step and poll the same bus. Load the hand with ``engage_feedback=False``.
"""

from __future__ import annotations

import logging
import statistics as st
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

import numpy as np


logger = logging.getLogger(__name__)


DEFAULT_STEP_DEG = 8.0
DEFAULT_TRIALS = 12
DEFAULT_SETTLE_S = 1.0
DEFAULT_WINDOW_S = 0.5

# Motion is called when a signal leaves its own resting noise by this many
# standard deviations. Detecting as early as the noise floor allows keeps the
# measured delay a transport figure rather than an acceleration one.
NOISE_SIGMAS = 6.0

# Floors, so a very quiet baseline cannot make the threshold meaninglessly small.
MIN_MOTOR_THRESHOLD_RAD = 0.0015  # ~1 encoder count of motor shaft
MIN_JOINT_THRESHOLD_DEG = 0.10
MIN_CURRENT_THRESHOLD_MA = 8.0
MIN_VELOCITY_THRESHOLD = 0.02


@dataclass(frozen=True)
class LatencyTrial:
    """One commanded step and when each stage of the chain responded."""

    joint: str
    motor_id: int
    step_deg: float
    command_to_current_s: Optional[float]
    """Transport: the servo drew current, before any motion."""
    command_to_velocity_s: Optional[float]
    command_to_motor_s: Optional[float]
    motor_to_joint_s: Optional[float]
    command_to_joint_s: Optional[float]
    motor_threshold_rad: float
    joint_threshold_deg: float
    poll_hz: float
    """Achieved motor-polling rate — the resolution of the electrical figure."""


@dataclass(frozen=True)
class LatencySummary:
    joint: str
    motor_id: int
    trials: int
    command_to_current_median_s: Optional[float]
    command_to_current_p90_s: Optional[float]
    command_to_velocity_median_s: Optional[float]
    command_to_motor_median_s: Optional[float]
    command_to_motor_p90_s: Optional[float]
    motor_to_joint_median_s: Optional[float]
    command_to_joint_median_s: Optional[float]
    poll_hz_median: float
    verdict: str


def _first_departure(
    times: Sequence[float], values: Sequence[float], baseline: float, threshold: float
) -> Optional[float]:
    """Time of the first sample more than ``threshold`` from ``baseline``."""
    for at, value in zip(times, values):
        if value is not None and abs(value - baseline) > threshold:
            return at
    return None


def measure_latency(
    hand,
    joint: str,
    read_joint_deg: Optional[Callable[[], Optional[float]]] = None,
    *,
    trials: int = DEFAULT_TRIALS,
    step_deg: float = DEFAULT_STEP_DEG,
    settle_s: float = DEFAULT_SETTLE_S,
    window_s: float = DEFAULT_WINDOW_S,
    progress_callback: Optional[Callable[[dict], None]] = None,
) -> List[LatencyTrial]:
    """Step one joint repeatedly and time each stage of its response.

    Each trial polls the motor's ``PRESENT_POSITION`` as fast as the bus
    allows, so the electrical figure is resolved to roughly one poll period.
    The joint encoder is read from the same loop when ``read_joint_deg`` is
    given; without it only the electrical half is measured.

    Steps alternate direction so tendon slack is taken up and released in
    equal measure rather than the joint drifting one way.

    Args:
        hand: A connected hand with **no joint loop running** — its writes
            would interleave with the step and its reads would contend for
            the bus.
        joint: Which joint to step.
        read_joint_deg: Returns the current joint angle, or ``None`` when no
            fresh reading is available.
        trials: Steps to perform, half in each direction.
        step_deg: Size of each step, in joint degrees.
        settle_s: Quiet time before each step, used to measure the resting
            noise that sets the detection thresholds.
        window_s: How long to watch for a response before giving up.

    Returns:
        One :class:`LatencyTrial` per step, in order.
    """
    motor_id = hand.config.joint_to_motor_map[joint]
    lower, upper = hand.config.joint_roms_dict[joint]
    centre = (lower + upper) / 2.0
    results: List[LatencyTrial] = []

    for index in range(trials):
        direction = 1.0 if index % 2 == 0 else -1.0
        origin = centre - direction * step_deg / 2.0
        target = centre + direction * step_deg / 2.0

        hand.set_joint_positions({joint: origin})
        time.sleep(settle_s)

        # Resting noise on both signals sets how small a departure can be
        # trusted as real motion.
        index_of = hand.config.motor_id_to_idx_dict[motor_id]
        base_motor: List[float] = []
        base_vel: List[float] = []
        base_cur: List[float] = []
        base_joint: List[float] = []
        deadline = time.perf_counter() + 0.25
        while time.perf_counter() < deadline:
            state = hand.get_motor_state()
            base_motor.append(float(state.position[index_of]))
            base_vel.append(float(state.velocity[index_of]))
            base_cur.append(float(state.current[index_of]))
            if read_joint_deg is not None:
                value = read_joint_deg()
                if value is not None:
                    base_joint.append(value)
        if len(base_motor) < 5:
            raise RuntimeError(
                "could not sample the motor position; is the bus free and "
                "the joint loop stopped?"
            )
        motor_zero = st.median(base_motor)
        motor_threshold = max(
            NOISE_SIGMAS * (st.pstdev(base_motor) or 0.0), MIN_MOTOR_THRESHOLD_RAD
        )
        vel_zero = st.median(base_vel)
        vel_threshold = max(
            NOISE_SIGMAS * (st.pstdev(base_vel) or 0.0), MIN_VELOCITY_THRESHOLD
        )
        cur_zero = st.median(base_cur)
        cur_threshold = max(
            NOISE_SIGMAS * (st.pstdev(base_cur) or 0.0), MIN_CURRENT_THRESHOLD_MA
        )
        joint_zero = st.median(base_joint) if base_joint else None
        joint_threshold = max(
            NOISE_SIGMAS * (st.pstdev(base_joint) if len(base_joint) > 1 else 0.0),
            MIN_JOINT_THRESHOLD_DEG,
        )

        _emit(progress_callback, "trial_started", index=index, joint=joint,
              origin=origin, target=target)

        motor_t: List[float] = []
        motor_v: List[float] = []
        vel_v: List[float] = []
        cur_v: List[float] = []
        joint_t: List[float] = []
        joint_v: List[float] = []

        # The command instant. Everything after is measured from here.
        started = time.perf_counter()
        hand.set_joint_positions({joint: target})
        while time.perf_counter() - started < window_s:
            at = time.perf_counter() - started
            state = hand.get_motor_state()
            motor_t.append(at)
            motor_v.append(float(state.position[index_of]))
            vel_v.append(float(state.velocity[index_of]))
            cur_v.append(float(state.current[index_of]))
            if read_joint_deg is not None:
                value = read_joint_deg()
                if value is not None:
                    joint_t.append(at)
                    joint_v.append(value)

        to_current = _first_departure(motor_t, cur_v, cur_zero, cur_threshold)
        to_velocity = _first_departure(motor_t, vel_v, vel_zero, vel_threshold)
        to_motor = _first_departure(motor_t, motor_v, motor_zero, motor_threshold)
        to_joint = (
            _first_departure(joint_t, joint_v, joint_zero, joint_threshold)
            if joint_zero is not None else None
        )
        poll_hz = len(motor_t) / max(motor_t[-1], 1e-9) if motor_t else 0.0

        trial = LatencyTrial(
            joint=joint,
            motor_id=motor_id,
            step_deg=direction * step_deg,
            command_to_current_s=to_current,
            command_to_velocity_s=to_velocity,
            command_to_motor_s=to_motor,
            motor_to_joint_s=(
                to_joint - to_motor
                if to_joint is not None and to_motor is not None else None
            ),
            command_to_joint_s=to_joint,
            motor_threshold_rad=motor_threshold,
            joint_threshold_deg=joint_threshold,
            poll_hz=poll_hz,
        )
        results.append(trial)
        _emit(progress_callback, "trial_finished", index=index, trial=trial)

    return results


def summarize_latency(trials: Sequence[LatencyTrial]) -> Optional[LatencySummary]:
    """Reduce trials to medians and say which half of the chain dominates."""
    if not trials:
        return None
    transport = [t.command_to_current_s for t in trials if t.command_to_current_s is not None]
    onset = [t.command_to_velocity_s for t in trials if t.command_to_velocity_s is not None]
    electrical = [t.command_to_motor_s for t in trials if t.command_to_motor_s is not None]
    mechanical = [t.motor_to_joint_s for t in trials if t.motor_to_joint_s is not None]
    total = [t.command_to_joint_s for t in trials if t.command_to_joint_s is not None]

    e_med = st.median(electrical) if electrical else None
    m_med = st.median(mechanical) if mechanical else None
    t_med = st.median(transport) if transport else None
    return LatencySummary(
        joint=trials[0].joint,
        motor_id=trials[0].motor_id,
        trials=len(trials),
        command_to_current_median_s=t_med,
        command_to_current_p90_s=(
            float(np.percentile(transport, 90)) if transport else None
        ),
        command_to_velocity_median_s=st.median(onset) if onset else None,
        command_to_motor_median_s=e_med,
        command_to_motor_p90_s=(
            float(np.percentile(electrical, 90)) if electrical else None
        ),
        motor_to_joint_median_s=m_med,
        command_to_joint_median_s=st.median(total) if total else None,
        poll_hz_median=st.median([t.poll_hz for t in trials]),
        verdict=_verdict(t_med if t_med is not None else e_med, m_med),
    )


def _verdict(electrical: Optional[float], mechanical: Optional[float]) -> str:
    """Which half dominates, and therefore what could move it."""
    if electrical is None:
        return "no motor response detected — check the joint and the step size"
    if mechanical is None:
        return (
            f"command reaches the motor in {electrical * 1000:.1f} ms; no joint "
            "reading, so the mechanical half is unmeasured"
        )
    if electrical > 2.0 * mechanical:
        return (
            f"transport dominates: {electrical * 1000:.1f} ms to the motor against "
            f"{mechanical * 1000:.1f} ms motor-to-joint. The host path and the "
            "USB-to-UART bridge are where the delay lives, and software can move it"
        )
    if mechanical > 2.0 * electrical:
        return (
            f"mechanics dominate: only {electrical * 1000:.1f} ms to the motor, then "
            f"{mechanical * 1000:.1f} ms of tendon take-up. No host-side change "
            "touches this — it is slack and compliance"
        )
    return (
        f"split roughly evenly: {electrical * 1000:.1f} ms transport, "
        f"{mechanical * 1000:.1f} ms mechanical. Halving the software path would "
        "cut total delay by about a quarter"
    )


def _emit(progress_callback, event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("latency progress callback failed")
