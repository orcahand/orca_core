# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Run the production calibration and keep what it measured on the way.

The routine is driven unmodified — this step scores its outcome and harvests
the telemetry it already emits and would otherwise discard: the traverse each
motor drove to reach its hardstop (which :mod:`.tendon_friction` reduces to a
friction number for free) and the encoder-pass events (which
:mod:`.encoder_anchor` scores).

The motor-side span carries more than it looks. The routine declares a
hardstop when the position buffer goes stable, which means it cannot
distinguish "reached the hardstop" from "stalled early"; the span between the
two limits is the only quantity that separates them.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, Tuple, TYPE_CHECKING

from ..maintenance.calibration_routine import run_calibration
from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("calibration step progress callback failed")


@dataclass(frozen=True)
class Traverse:
    """One motor's drive toward one hardstop, as the routine recorded it.

    ``positions`` (motor rad) and ``currents`` (mA) are sample-aligned.
    ``reached_limit`` is false when the drive ran out before going stable.
    """

    motor: int
    joint: str
    step_index: int
    direction: int
    reached_limit: bool
    positions: List[float]
    currents: List[float]


@dataclass(frozen=True)
class JointCalibration:
    """What the routine recorded for one joint.

    ``motor_span_rad`` is the travel between the two recorded limits — the
    quantity that separates a real hardstop from an early stall.
    """

    joint: str
    motor: int
    lower_rad: float | None
    upper_rad: float | None
    motor_span_rad: float | None
    ratio: float | None
    messages: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return (
            self.lower_rad is not None
            and self.upper_rad is not None
            and not self.messages
        )


@dataclass(frozen=True)
class CalibrationStepResult:
    """The calibration's outcome, its per-joint spans, and its raw traverses.

    ``traverses`` is the telemetry :func:`~orca_core.verification.
    analyse_tendon_friction` consumes; it is large, and a report that does not
    want the raw samples should store that step's summary instead.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    calibrated: bool
    wrist_calibrated: bool
    duration_s: float
    joints: List[JointCalibration]
    traverses: List[Traverse]
    torque_release_failures: List[str]
    offset_calibration_failures: List[str]
    encoder_events: List[dict]
    thresholds: Dict[str, float]
    step_messages: List[str] = field(default_factory=list)

    @property
    def failed(self) -> List[JointCalibration]:
        return [joint for joint in self.joints if not joint.ok]

    @property
    def passed(self) -> bool:
        return (
            self.calibrated
            and self.wrist_calibrated
            and not self.failed
            and not self.step_messages
        )

    @property
    def messages(self) -> List[str]:
        return [m for joint in self.joints for m in joint.messages] + list(
            self.step_messages
        )

    def measurements(self) -> Dict[str, float]:
        """Per joint: both limits, the span between them, and the ratio. Plus
        the step's duration, which is itself a diagnostic — a calibration that
        took three times as long is a joint that was fighting."""
        return {
            **flat_measurements(
                "motor_limit_lower_rad", {j.joint: j.lower_rad for j in self.joints}
            ),
            **flat_measurements(
                "motor_limit_upper_rad", {j.joint: j.upper_rad for j in self.joints}
            ),
            **flat_measurements(
                "motor_span_rad", {j.joint: j.motor_span_rad for j in self.joints}
            ),
            **flat_measurements(
                "joint_to_motor_ratio", {j.joint: j.ratio for j in self.joints}
            ),
            "duration_s": float(self.duration_s),
        }


def run_calibration_step(
    hand: "OrcaHand",
    *,
    joint_encoder_client=None,
    joints: List[str] | None = None,
    force_wrist: bool = True,
    persist: bool = True,
    motor_span_limits_rad: Mapping[str, Tuple[float, float]] | None = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> CalibrationStepResult | None:
    """Calibrate the hand and score what the routine recorded.

    The calibration itself is unchanged — this drives
    :func:`~orca_core.maintenance.calibration_routine.run_calibration` and
    reads its event stream. Returns ``None`` on early exit (``should_stop``
    triggered); the routine handles its own torque release on any abort.

    ``motor_span_limits_rad`` has no default because no fleet distribution
    exists yet: without it the span is measured and recorded but not gated,
    which is what makes those bands buildable in the first place.

    Args:
        hand: A connected, tensioned :class:`~orca_core.OrcaHand`.
        joint_encoder_client: A streaming ``JointEncoderClient``, to also run
            the encoder anchor pass.
        joints: Restrict to calibration steps touching these joints.
        force_wrist: Recalibrate the wrist even if it already is.
        persist: Write ``calibration.yaml``. This step is the one place in the
            bench suite that legitimately does.
        motor_span_limits_rad: ``{joint: (min, max)}`` motor-side span band.
            Joints absent from it are recorded but not gated.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``calibration_step_started``,
            ``calibration_event`` wrapping each inner calibration event,
            ``calibration_step_aborted``, ``calibration_step_done``). Must be
            fast and non-blocking; exceptions it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` forwarded into the
            calibration run.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    traverses: List[Traverse] = []
    encoder_events: List[dict] = []
    torque_release_failures: List[str] = []
    offset_calibration_failures: List[str] = []
    ratios: Dict[str, float] = {}

    def on_progress(event: dict) -> None:
        name = event.get("event")
        if name == "traverse_recorded":
            traverses.append(
                Traverse(
                    motor=int(event["motor"]),
                    joint=str(event["joint"]),
                    step_index=int(event["index"]),
                    direction=int(event["direction"]),
                    reached_limit=bool(event["reached_limit"]),
                    positions=[float(p) for p in event["positions"]],
                    currents=[float(c) for c in event["currents"]],
                )
            )
        elif name in ("encoder_anchor_failed", "measured_rom_rejected",
                      "encoder_anchor_recorded", "measured_rom_recorded"):
            encoder_events.append(dict(event))
        elif name == "torque_release_failed":
            torque_release_failures.append(str(event["joint"]))
        elif name == "offset_calibration_failed":
            offset_calibration_failures.append(str(event["joint"]))
        elif name == "joint_calibrated":
            ratios[str(event["joint"])] = float(event["ratio"])
        _emit(progress_callback, "calibration_event", payload=dict(event))

    _emit(progress_callback, "calibration_step_started")
    started = time.monotonic()
    calibration = run_calibration(
        hand,
        joints=joints,
        joint_encoder_client=joint_encoder_client,
        force_wrist=force_wrist,
        persist=persist,
        progress_callback=on_progress,
        should_stop=should_stop,
    )
    duration_s = time.monotonic() - started

    if calibration is None:
        _emit(progress_callback, "calibration_step_aborted")
        return None

    thresholds: Dict[str, float] = {}
    for joint, (low, high) in (motor_span_limits_rad or {}).items():
        thresholds[f"motor_span_rad.{joint}.min"] = float(low)
        thresholds[f"motor_span_rad.{joint}.max"] = float(high)

    result = CalibrationStepResult(
        calibrated=bool(calibration.calibrated),
        wrist_calibrated=bool(calibration.wrist_calibrated),
        duration_s=round(duration_s, 3),
        joints=_analyse_joints(
            hand,
            calibration=calibration,
            ratios=ratios,
            joints=joints,
            motor_span_limits_rad=motor_span_limits_rad or {},
        ),
        traverses=traverses,
        torque_release_failures=torque_release_failures,
        offset_calibration_failures=offset_calibration_failures,
        encoder_events=encoder_events,
        thresholds=thresholds,
        step_messages=_step_messages(
            calibrated=bool(calibration.calibrated),
            wrist_calibrated=bool(calibration.wrist_calibrated),
            torque_release_failures=torque_release_failures,
            offset_calibration_failures=offset_calibration_failures,
        ),
    )
    _emit(
        progress_callback,
        "calibration_step_done",
        duration_s=result.duration_s,
        failed=[j.joint for j in result.failed],
    )
    return result


def _analyse_joints(
    hand: "OrcaHand",
    *,
    calibration,
    ratios: Dict[str, float],
    joints: List[str] | None,
    motor_span_limits_rad: Mapping[str, Tuple[float, float]],
) -> List[JointCalibration]:
    """Reduce the calibration's limits to a per-joint span and verdict."""
    targets = list(joints) if joints is not None else list(hand.config.joint_to_motor_map)
    analysed: List[JointCalibration] = []

    for joint in targets:
        motor_id = hand.config.joint_to_motor_map[joint]
        limits = calibration.motor_limits_dict.get(motor_id) or [None, None]
        lower, upper = limits[0], limits[1]
        span = None if lower is None or upper is None else abs(float(upper) - float(lower))

        messages: List[str] = []
        if lower is None or upper is None:
            missing = "lower" if lower is None else "upper"
            if lower is None and upper is None:
                missing = "neither"
            messages.append(
                f"Joint {joint} finished calibration with {missing} limit "
                f"recorded, so its joint-to-motor map is incomplete."
            )
        elif joint in motor_span_limits_rad:
            low, high = motor_span_limits_rad[joint]
            if not low <= span <= high:
                messages.append(
                    f"Joint {joint} travelled {span:.3f} motor rad between its "
                    f"limits; the expected range is {low:.3f}–{high:.3f}. The "
                    f"routine records a limit when the motor stops moving, "
                    f"which happens both at a hardstop and when a joint stalls "
                    f"before reaching one."
                )

        analysed.append(
            JointCalibration(
                joint=joint,
                motor=motor_id,
                lower_rad=None if lower is None else float(lower),
                upper_rad=None if upper is None else float(upper),
                motor_span_rad=None if span is None else round(span, 4),
                ratio=ratios.get(joint, calibration.joint_to_motor_ratios_dict.get(motor_id)),
                messages=messages,
            )
        )
    return analysed


def _step_messages(
    *,
    calibrated: bool,
    wrist_calibrated: bool,
    torque_release_failures: List[str],
    offset_calibration_failures: List[str],
) -> List[str]:
    """Text for the outcomes that belong to the run rather than to one joint."""
    messages: List[str] = []
    if not calibrated:
        messages.append("The calibration did not complete for every joint.")
    if not wrist_calibrated:
        messages.append("The wrist was not calibrated.")
    if torque_release_failures:
        messages.append(
            f"Torque release failed on {sorted(set(torque_release_failures))}, so "
            f"no limit was recorded for them: a tensioned tendon biases the read."
        )
    if offset_calibration_failures:
        messages.append(
            f"Offset calibration failed on {sorted(set(offset_calibration_failures))}, "
            f"so their position frame was not shifted and no limit was recorded."
        )
    return messages
