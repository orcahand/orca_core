# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Calibrate a second time and see whether the hand finds the same limits.

The motor-side twin of the anchor repeatability run: two passes total, one
from the calibration step and one here, compared in joint degrees so the
number means the same thing on every joint. A third pass roughly doubles the
most expensive block of the plan for a marginal gain in confidence.

Nothing is persisted — the repeat runs with ``persist=False`` and the hand's
in-memory calibration is restored on every exit path.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from ..maintenance.calibration_routine import run_calibration
from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand
    from .calibration_step import CalibrationStepResult

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

SPREAD_GOOD_DEG = 0.30
SPREAD_MARGINAL_DEG = 1.00
"""Provisional, taken from the encoder anchor's scale. Motor-side numbers
should be worse — they include tendon stretch and backlash — so these want
re-setting from the first production batch rather than being trusted."""

VERDICT_GOOD = "GOOD"
VERDICT_MARGINAL = "MARGINAL"
VERDICT_POOR = "POOR"
VERDICT_NO_DATA = "NO_DATA"


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("calibration consistency progress callback failed")


@dataclass(frozen=True)
class JointLimitDelta:
    """How far one joint's recorded limits moved between the two passes.

    Deltas are in joint degrees, converted through the joint-to-motor ratio,
    so a wrist and a fingertip are comparable.
    """

    joint: str
    motor: int
    first_rad: List[float | None]
    repeat_rad: List[float | None]
    lower_delta_deg: float | None
    upper_delta_deg: float | None
    span_delta_deg: float | None
    worst_delta_deg: float | None
    verdict: str
    messages: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class CalibrationConsistencyResult:
    """Both passes' limits per joint and the spread between them.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    passes: int
    joints: List[JointLimitDelta]
    thresholds: Dict[str, float]

    @property
    def failed(self) -> List[JointLimitDelta]:
        return [
            joint
            for joint in self.joints
            if joint.verdict in (VERDICT_POOR, VERDICT_NO_DATA)
        ]

    @property
    def passed(self) -> bool:
        return not self.failed

    @property
    def messages(self) -> List[str]:
        return [m for joint in self.joints for m in joint.messages]

    def measurements(self) -> Dict[str, float]:
        return {
            **flat_measurements(
                "limit_lower_delta_deg", {j.joint: j.lower_delta_deg for j in self.joints}
            ),
            **flat_measurements(
                "limit_upper_delta_deg", {j.joint: j.upper_delta_deg for j in self.joints}
            ),
            **flat_measurements(
                "motor_span_delta_deg", {j.joint: j.span_delta_deg for j in self.joints}
            ),
            "passes": float(self.passes),
        }


def run_calibration_consistency(
    hand: "OrcaHand",
    *,
    baseline: "CalibrationStepResult",
    joint_encoder_client=None,
    joints: List[str] | None = None,
    spread_good_deg: float = SPREAD_GOOD_DEG,
    spread_marginal_deg: float = SPREAD_MARGINAL_DEG,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> CalibrationConsistencyResult | None:
    """Re-run the calibration and compare its limits against ``baseline``.

    Returns ``None`` on early exit (``should_stop`` triggered). The hand's
    calibration is restored to what it was on entry on every exit path,
    including exceptions, so the repeat never becomes the hand's calibration.

    Args:
        hand: A connected, already-calibrated :class:`~orca_core.OrcaHand`.
        baseline: The calibration step's result to compare against.
        joint_encoder_client: A streaming ``JointEncoderClient``, if the
            encoder pass should run on the repeat too.
        joints: Restrict to calibration steps touching these joints.
        spread_good_deg: Limit movement at or below which a joint is ``GOOD``.
        spread_marginal_deg: At or below which it is ``MARGINAL``; above,
            ``POOR``.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``consistency_started``, ``calibration_event``
            wrapping each inner calibration event, ``consistency_aborted``,
            ``consistency_done``, ``cleanup_failed``). Must be fast and
            non-blocking; exceptions it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` forwarded into the run.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    original = hand.calibration

    def _restore_hand() -> None:
        hand.calibration = original
        hand.disable_torque()

    _emit(progress_callback, "consistency_started")
    try:
        repeat = run_calibration(
            hand,
            joints=joints,
            joint_encoder_client=joint_encoder_client,
            force_wrist=True,
            persist=False,
            progress_callback=lambda event: _emit(
                progress_callback, "calibration_event", payload=dict(event)
            ),
            should_stop=should_stop,
        )
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        try:
            _restore_hand()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after an aborted consistency run failed: %s", e)
        raise

    repeat_limits = {} if repeat is None else dict(repeat.motor_limits_dict)
    _restore_hand()

    if repeat is None:
        _emit(progress_callback, "consistency_aborted")
        return None

    result = CalibrationConsistencyResult(
        passes=2,
        joints=_compare(
            baseline=baseline,
            repeat_limits=repeat_limits,
            spread_good_deg=spread_good_deg,
            spread_marginal_deg=spread_marginal_deg,
        ),
        thresholds={
            "spread_good_deg": float(spread_good_deg),
            "spread_marginal_deg": float(spread_marginal_deg),
        },
    )
    _emit(
        progress_callback,
        "consistency_done",
        failed=[j.joint for j in result.failed],
    )
    return result


def _verdict_for(delta_deg: float, good_deg: float, marginal_deg: float) -> str:
    if delta_deg <= good_deg:
        return VERDICT_GOOD
    if delta_deg <= marginal_deg:
        return VERDICT_MARGINAL
    return VERDICT_POOR


def _compare(
    *,
    baseline: "CalibrationStepResult",
    repeat_limits: Dict[int, List],
    spread_good_deg: float,
    spread_marginal_deg: float,
) -> List[JointLimitDelta]:
    """Difference each joint's two limit pairs, in joint degrees."""
    compared: List[JointLimitDelta] = []

    for joint in baseline.joints:
        repeat = repeat_limits.get(joint.motor) or [None, None]
        first = [joint.lower_rad, joint.upper_rad]
        ratio = joint.ratio

        lower_delta = _to_degrees(first[0], repeat[0], ratio)
        upper_delta = _to_degrees(first[1], repeat[1], ratio)
        span_delta = None
        if None not in (joint.motor_span_rad, repeat[0], repeat[1]) and ratio:
            repeat_span = abs(float(repeat[1]) - float(repeat[0]))
            span_delta = round(abs(repeat_span - joint.motor_span_rad) / abs(ratio), 4)

        deltas = [d for d in (lower_delta, upper_delta) if d is not None]
        worst = max(deltas) if deltas else None
        verdict = (
            VERDICT_NO_DATA
            if worst is None
            else _verdict_for(worst, spread_good_deg, spread_marginal_deg)
        )

        messages: List[str] = []
        if verdict == VERDICT_NO_DATA:
            messages.append(
                f"Joint {joint.joint} has no comparable limit pair across the "
                f"two passes, so its calibration repeatability is unmeasured."
            )
        elif verdict == VERDICT_POOR:
            messages.append(
                f"Joint {joint.joint} recorded limits {worst:.2f}° apart across "
                f"two calibrations (limit {spread_marginal_deg:.2f}°)."
            )

        compared.append(
            JointLimitDelta(
                joint=joint.joint,
                motor=joint.motor,
                first_rad=first,
                repeat_rad=[
                    None if repeat[0] is None else float(repeat[0]),
                    None if repeat[1] is None else float(repeat[1]),
                ],
                lower_delta_deg=lower_delta,
                upper_delta_deg=upper_delta,
                span_delta_deg=span_delta,
                worst_delta_deg=worst,
                verdict=verdict,
                messages=messages,
            )
        )
    return compared


def _to_degrees(first, repeat, ratio: float | None) -> float | None:
    """Absolute motor-rad difference expressed in joint degrees.

    ``ratio`` is motor-rad per joint-degree, so a joint whose ratio was never
    recorded has no degree-space answer rather than a misleading one.
    """
    if first is None or repeat is None or not ratio:
        return None
    return round(abs(float(repeat) - float(first)) / abs(ratio), 4)
