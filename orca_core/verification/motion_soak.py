# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Cycle the hand under load and watch for what a static test cannot see.

A motor heating abnormally from friction, a tendon unseating over cycles, a
connector dropping under motion, and error bits that only set under load —
none of these show up on a still hand.

Temperature *spread* is the signal rather than the peak: absolute temperature
follows the room, so the outlier motor is what matters. Pose repeatability
closes the loop on the mechanical side — the same pose commanded at the start
and at the end should put the motors in the same place, and drift means a
tendon settled or a spool slipped.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, Sequence, TYPE_CHECKING

import numpy as np

from ..constants import NUM_STEPS, STEP_SIZE
from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

DURATION_S = 600.0
"""Provisional. Long enough for motor temperatures to separate, short enough
to overlap with whatever the operator does next."""

HOLD_S = 2.0
SAMPLE_INTERVAL_S = 2.0

MAX_TEMPERATURE_C = 70.0
"""Conservative across the motor families in use; the soak aborts here rather
than scoring a hand that is cooking."""

TEMPERATURE_SPREAD_LIMIT_C = 15.0
"""Provisional. Wider than the cold limit because driven motors legitimately
diverge with duty cycle; the outlier is still the signal."""

MAX_FAILED_READ_FRACTION = 0.01
POSE_REPEATABILITY_LIMIT_RAD = 0.05
"""Provisional. Motor-side drift between the same pose commanded at the start
and the end of the soak."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("motion soak progress callback failed")


@dataclass(frozen=True)
class MotorSoak:
    """One motor's state at the end of the soak."""

    motor: int
    joint: str
    temperature_c: float | None
    peak_temperature_c: float | None
    error_byte: int | None
    faults: List[str] = field(default_factory=list)
    pose_drift_rad: float | None = None
    reboots: int = 0
    messages: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.messages


@dataclass(frozen=True)
class MotionSoakResult:
    """What the soak did and what it found.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    cycles: int
    duration_s: float
    motors: List[MotorSoak]
    reads: int
    failed_reads: int
    reboots: List[int]
    temperature_spread_c: float | None
    peak_temperature_c: float | None
    reboot_check_supported: bool
    thresholds: Dict[str, float]
    step_messages: List[str] = field(default_factory=list)

    @property
    def failed(self) -> List[MotorSoak]:
        return [motor for motor in self.motors if not motor.ok]

    @property
    def passed(self) -> bool:
        return not self.failed and not self.step_messages

    @property
    def messages(self) -> List[str]:
        return [m for motor in self.motors for m in motor.messages] + list(
            self.step_messages
        )

    def measurements(self) -> Dict[str, float]:
        return {
            **flat_measurements(
                "soak_temperature_c", {m.joint: m.temperature_c for m in self.motors}
            ),
            **flat_measurements(
                "soak_peak_temperature_c",
                {m.joint: m.peak_temperature_c for m in self.motors},
            ),
            **flat_measurements(
                "soak_error_byte", {m.joint: m.error_byte for m in self.motors}
            ),
            **flat_measurements(
                "pose_drift_rad", {m.joint: m.pose_drift_rad for m in self.motors}
            ),
            "soak_cycles": float(self.cycles),
            "soak_duration_s": float(self.duration_s),
            "soak_reads": float(self.reads),
            "soak_failed_reads": float(self.failed_reads),
            "soak_reboots": float(len(self.reboots)),
        }


def run_motion_soak(
    hand: "OrcaHand",
    *,
    duration_s: float = DURATION_S,
    poses: Sequence[Mapping[str, float]] | None = None,
    hold_s: float = HOLD_S,
    sample_interval_s: float = SAMPLE_INTERVAL_S,
    num_steps: int = NUM_STEPS,
    step_size: float = STEP_SIZE,
    max_temperature_c: float = MAX_TEMPERATURE_C,
    temperature_spread_limit_c: float = TEMPERATURE_SPREAD_LIMIT_C,
    max_failed_read_fraction: float = MAX_FAILED_READ_FRACTION,
    pose_repeatability_limit_rad: float = POSE_REPEATABILITY_LIMIT_RAD,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> MotionSoakResult | None:
    """Cycle between poses for ``duration_s``, sampling the motors throughout.

    The default poses are each joint at both ends of its configured ROM, so
    this works on any model rather than carrying one hand's angles. Torque is
    released on every exit path, including exceptions.

    Returns ``None`` on early exit (``should_stop`` triggered before a full
    cycle). The soak stops early — and still returns a result — if any motor
    reaches ``max_temperature_c``, because continuing would damage the hand.

    Args:
        hand: A connected, calibrated :class:`~orca_core.OrcaHand`.
        duration_s: Seconds to keep cycling.
        poses: Poses to cycle between. Defaults to fully open and fully
            closed, built from the config ROMs.
        hold_s: Seconds to hold each pose after the motion completes.
        sample_interval_s: Seconds between telemetry samples.
        num_steps: Interpolation steps per move.
        step_size: Seconds between interpolation steps.
        max_temperature_c: Any motor at or above this aborts the soak.
        temperature_spread_limit_c: Largest tolerated gap between the hottest
            and coldest motor at the end.
        max_failed_read_fraction: Dropped status packets tolerated, as a
            fraction of the reads taken.
        pose_repeatability_limit_rad: Motor-side drift tolerated between the
            reference pose commanded at the start and again at the end.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``soak_started``, ``cycle_done``, ``sampled``,
            ``overheated``, ``motors_rebooted``, ``soak_aborted``,
            ``soak_done``, ``cleanup_failed``). Must be fast and
            non-blocking; exceptions it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between moves.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    cycle_poses = list(poses) if poses is not None else _default_poses(hand)
    if len(cycle_poses) < 2:
        raise ValueError("a soak needs at least two poses to cycle between")

    motor_to_joint = hand.config.motor_to_joint_dict
    thresholds = {
        "duration_s": float(duration_s),
        "max_temperature_c": float(max_temperature_c),
        "temperature_spread_limit_c": float(temperature_spread_limit_c),
        "max_failed_read_fraction": float(max_failed_read_fraction),
        "pose_repeatability_limit_rad": float(pose_repeatability_limit_rad),
    }

    peak_temps: Dict[int, float] = {}
    reboots: List[int] = []
    reads = failed_reads = cycles = 0
    overheated = False
    reboot_check_supported = True

    def _restore_hand() -> None:
        hand.disable_torque()

    _emit(progress_callback, "soak_started", duration_s=duration_s,
          poses=len(cycle_poses))
    hand.enable_torque()
    started = time.monotonic()
    try:
        # The reference pose is commanded first and again last; the motor
        # positions it settles at are what pose repeatability compares.
        reference = dict(cycle_poses[0])
        hand.set_joint_positions(reference, num_steps=num_steps, step_size=step_size)
        time.sleep(hold_s)
        start_positions = hand.get_motor_pos(as_dict=True)

        last_sample = 0.0
        # At least one full cycle always runs: a soak cut short still measured
        # the hand under load, and a result beats no result.
        while True:
            if should_stop():
                break
            for pose in cycle_poses:
                if should_stop():
                    break
                hand.set_joint_positions(
                    dict(pose), num_steps=num_steps, step_size=step_size
                )
                if hold_s:
                    time.sleep(hold_s)
            cycles += 1
            _emit(progress_callback, "cycle_done", cycles=cycles)

            now = time.monotonic()
            if now - last_sample < sample_interval_s:
                if now - started >= duration_s:
                    break
                continue
            last_sample = now

            temperatures = hand.get_motor_temp(as_dict=True)
            for motor_id, temperature in temperatures.items():
                peak_temps[motor_id] = max(
                    peak_temps.get(motor_id, float("-inf")), float(temperature)
                )
            reads += 1
            with hand._motor_lock:
                hand.motor_client.read_position_velocity_current()
                if not hand.motor_client.last_read_ok:
                    failed_reads += 1
            _emit(progress_callback, "sampled", cycles=cycles,
                  peak_temperature_c=max(peak_temps.values(), default=None))

            if reboot_check_supported:
                try:
                    with hand._motor_lock:
                        rebooted = hand.motor_client.check_overload_and_reboot(
                            hand.config.motor_ids
                        )
                except NotImplementedError:
                    reboot_check_supported = False
                    logger.info(
                        "motor family cannot reboot overloaded motors; reboots "
                        "not counted during the soak"
                    )
                else:
                    if rebooted:
                        reboots.extend(rebooted)
                        _emit(progress_callback, "motors_rebooted", motors=list(rebooted))

            if temperatures and max(temperatures.values()) >= max_temperature_c:
                overheated = True
                _emit(progress_callback, "overheated",
                      peak_temperature_c=float(max(temperatures.values())))
                break

            if time.monotonic() - started >= duration_s:
                break

        if not cycles:
            _restore_hand()
            _emit(progress_callback, "soak_aborted")
            return None

        hand.set_joint_positions(reference, num_steps=num_steps, step_size=step_size)
        time.sleep(hold_s)
        end_positions = hand.get_motor_pos(as_dict=True)
        final_temps = hand.get_motor_temp(as_dict=True)
        final_errors = _read_errors(hand)
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        try:
            _restore_hand()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after an aborted soak failed: %s", e)
        raise

    _restore_hand()
    duration = round(time.monotonic() - started, 2)

    motors = [
        _judge_motor(
            motor_id=motor_id,
            joint=motor_to_joint.get(motor_id, f"motor_{motor_id}"),
            hand=hand,
            temperature_c=_as_float(final_temps.get(motor_id)),
            peak_temperature_c=_as_float(peak_temps.get(motor_id)),
            error_byte=final_errors.get(motor_id),
            drift_rad=_drift(start_positions, end_positions, motor_id),
            reboots=reboots.count(motor_id),
            max_temperature_c=max_temperature_c,
            pose_repeatability_limit_rad=pose_repeatability_limit_rad,
        )
        for motor_id in hand.config.motor_ids
    ]

    result = MotionSoakResult(
        cycles=cycles,
        duration_s=duration,
        motors=motors,
        reads=reads,
        failed_reads=failed_reads,
        reboots=sorted(set(reboots)),
        temperature_spread_c=_spread([m.temperature_c for m in motors]),
        peak_temperature_c=max(peak_temps.values(), default=None),
        reboot_check_supported=reboot_check_supported,
        thresholds=thresholds,
        step_messages=_step_messages(
            motors=motors,
            reads=reads,
            failed_reads=failed_reads,
            max_failed_read_fraction=max_failed_read_fraction,
            temperature_spread_limit_c=temperature_spread_limit_c,
            reboots=sorted(set(reboots)),
            overheated=overheated,
            max_temperature_c=max_temperature_c,
        ),
    )
    _emit(progress_callback, "soak_done", cycles=cycles, duration_s=duration,
          failed=[m.joint for m in result.failed])
    return result


def _default_poses(hand: "OrcaHand") -> List[Dict[str, float]]:
    """Both ends of every joint's configured ROM.

    Built from fractions rather than fixed angles so the soak drives the full
    travel of whichever model is on the bench.
    """
    return [
        hand.pose_from_fractions({j: 0.0 for j in hand.config.joint_ids}).as_dict(),
        hand.pose_from_fractions({j: 1.0 for j in hand.config.joint_ids}).as_dict(),
    ]


def _as_float(value) -> float | None:
    return None if value is None else float(value)


def _spread(values: List[float | None]) -> float | None:
    present = [v for v in values if v is not None]
    if len(present) < 2:
        return None
    return round(max(present) - min(present), 2)


def _drift(start: Dict[int, float], end: Dict[int, float], motor_id: int) -> float | None:
    if motor_id not in start or motor_id not in end:
        return None
    return round(float(np.abs(end[motor_id] - start[motor_id])), 4)


def _read_errors(hand: "OrcaHand") -> Dict[int, int | None]:
    """Error bytes after the soak, empty on a family that cannot report them."""
    try:
        return hand.get_motor_errors()
    except NotImplementedError:
        logger.info("motor family cannot report hardware errors; none recorded")
        return {}


def _judge_motor(
    *,
    motor_id: int,
    joint: str,
    hand: "OrcaHand",
    temperature_c: float | None,
    peak_temperature_c: float | None,
    error_byte: int | None,
    drift_rad: float | None,
    reboots: int,
    max_temperature_c: float,
    pose_repeatability_limit_rad: float,
) -> MotorSoak:
    """Reduce one motor's end-of-soak state to a verdict."""
    faults = hand.motor_client.decode_hardware_error(error_byte) if error_byte else []

    messages: List[str] = []
    if error_byte:
        named = ", ".join(faults) if faults else "no named bits"
        messages.append(
            f"Motor {motor_id} ({joint}) reports hardware error "
            f"0x{error_byte:02X} ({named}) after the soak."
        )
    if peak_temperature_c is not None and peak_temperature_c >= max_temperature_c:
        messages.append(
            f"Motor {motor_id} ({joint}) reached {peak_temperature_c:.1f}°C "
            f"during the soak (limit {max_temperature_c:.1f}°C)."
        )
    if drift_rad is not None and drift_rad > pose_repeatability_limit_rad:
        messages.append(
            f"Motor {motor_id} ({joint}) settled {drift_rad:.3f} rad from where "
            f"the same pose put it before the soak (limit "
            f"{pose_repeatability_limit_rad:.3f} rad)."
        )
    if reboots:
        messages.append(
            f"Motor {motor_id} ({joint}) was rebooted {reboots} time(s) after "
            f"latching an overload during the soak."
        )

    return MotorSoak(
        motor=motor_id,
        joint=joint,
        temperature_c=temperature_c,
        peak_temperature_c=peak_temperature_c,
        error_byte=error_byte,
        faults=faults,
        pose_drift_rad=drift_rad,
        reboots=reboots,
        messages=messages,
    )


def _step_messages(
    *,
    motors: List[MotorSoak],
    reads: int,
    failed_reads: int,
    max_failed_read_fraction: float,
    temperature_spread_limit_c: float,
    reboots: List[int],
    overheated: bool,
    max_temperature_c: float,
) -> List[str]:
    """Text for what belongs to the soak rather than to one motor."""
    messages: List[str] = []
    if overheated:
        messages.append(
            f"The soak stopped early: a motor reached {max_temperature_c:.1f}°C."
        )
    if reads and failed_reads / reads > max_failed_read_fraction:
        messages.append(
            f"{failed_reads} of {reads} bulk reads returned no status packet "
            f"during the soak (limit {max_failed_read_fraction:.0%})."
        )

    spread = _spread([m.temperature_c for m in motors])
    if spread is not None and spread > temperature_spread_limit_c:
        hottest = max(motors, key=lambda m: m.temperature_c or float("-inf"))
        coldest = min(motors, key=lambda m: m.temperature_c or float("inf"))
        messages.append(
            f"Motor temperatures span {spread:.1f}°C after the soak (limit "
            f"{temperature_spread_limit_c:.1f}°C): {hottest.joint} at "
            f"{hottest.temperature_c:.1f}°C against {coldest.joint} at "
            f"{coldest.temperature_c:.1f}°C."
        )
    return messages
