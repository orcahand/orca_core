# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Measure how repeatably a hand finds its encoder anchors.

Runs the production calibration N times without persisting anything and
compares the anchor count each pass landed on. The spread across passes is
what a closed-loop joint's absolute accuracy ultimately rests on: the anchor
is the fixed reference of the encoder frame, so an anchor that moves between
calibrations moves every angle derived from it.

The same passes also measure each joint's hardstop-to-hardstop span, including
the spans the routine rejected for missing its sanity tolerance — those never
reach the joint map, leaving the joint on a config nominal known to be wrong,
so they are the ones worth seeing.
"""

from __future__ import annotations

import dataclasses
import logging
import statistics
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, TYPE_CHECKING

import numpy as np

from ..hardware.joint_encoder_client import (
    circular_mean_count,
    signed_count_delta,
)
from ..hardware.sensing.constants import ENCODER_LSB_DEG, JOINT_TO_ENCODER_SLOT
from ..maintenance.calibration_routine import run_calibration
from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]

REPEATS = 5

SPREAD_GOOD_DEG = 0.30
SPREAD_MARGINAL_DEG = 1.00

VERDICT_GOOD = "GOOD"
VERDICT_MARGINAL = "MARGINAL"
VERDICT_POOR = "POOR"
VERDICT_UNRELIABLE = "UNRELIABLE"
VERDICT_NO_DATA = "NO_DATA"


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("anchor repeatability progress callback failed")


@dataclass(frozen=True)
class JointAnchorStats:
    """One joint's anchor spread and measured span across the passes.

    ``sample_failures`` counts passes where the anchor sample failed outright;
    any non-zero count forces ``verdict`` to ``UNRELIABLE``, because a spread
    computed from only the passes that did read must not present as clean.
    ``spans_rejected`` counts passes whose measured span the calibration
    routine discarded, leaving the joint on its configured nominal.
    """

    joint: str
    slot: int
    anchored_passes: int
    sample_failures: int
    verdict: str
    mean_count: float | None = None
    std_deg: float | None = None
    peak_to_peak_deg: float | None = None
    nominal_span_deg: float | None = None
    measured_span_mean_deg: float | None = None
    measured_span_spread_deg: float | None = None
    span_error_deg: float | None = None
    spans_rejected: int = 0
    messages: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class AnchorRepeatabilityResult:
    """Every joint's anchor statistics plus the thresholds that judged them.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    repeats: int
    completed_passes: int
    calibration_current: float
    joints: List[JointAnchorStats]
    thresholds: Dict[str, float]

    @property
    def failed(self) -> List[JointAnchorStats]:
        return [
            stats
            for stats in self.joints
            if stats.verdict in (VERDICT_POOR, VERDICT_UNRELIABLE, VERDICT_NO_DATA)
        ]

    @property
    def passed(self) -> bool:
        return not self.failed

    @property
    def messages(self) -> List[str]:
        return [message for stats in self.joints for message in stats.messages]

    def measurements(self) -> Dict[str, float]:
        """Per joint: the anchor spread the verdict rests on, how many passes
        it came from, and the span those same passes measured."""
        metrics = {
            "anchor_spread_p2p_deg": lambda s: s.peak_to_peak_deg,
            "anchor_spread_std_deg": lambda s: s.std_deg,
            "anchor_mean_count": lambda s: s.mean_count,
            "anchored_passes": lambda s: s.anchored_passes,
            "sample_failures": lambda s: s.sample_failures,
            "nominal_span_deg": lambda s: s.nominal_span_deg,
            "measured_span_mean_deg": lambda s: s.measured_span_mean_deg,
            "measured_span_spread_deg": lambda s: s.measured_span_spread_deg,
            "span_error_deg": lambda s: s.span_error_deg,
            "spans_rejected": lambda s: s.spans_rejected,
        }
        measurements: Dict[str, float] = {}
        for metric, pick in metrics.items():
            measurements.update(
                flat_measurements(metric, {s.joint: pick(s) for s in self.joints})
            )
        return measurements


def _verdict_for(spread_deg: float, good_deg: float, marginal_deg: float) -> str:
    if spread_deg <= good_deg:
        return VERDICT_GOOD
    if spread_deg <= marginal_deg:
        return VERDICT_MARGINAL
    return VERDICT_POOR


# One pass: {joint: (anchor_count, span_deg or None, span_was_rejected)}.
Pass = Dict[str, Tuple[int, float | None, bool]]


def _run_one_pass(
    hand: "OrcaHand",
    *,
    baseline,
    joints: List[str] | None,
    joint_encoder_client,
    failures: Dict[str, List[str]],
    rejections: Dict[str, List[float]],
    index: int,
    progress_callback: Optional[ProgressCallback],
    should_stop: ShouldStop,
) -> Pass | None:
    """Re-run the production calibration once and harvest what it anchored.

    The encoder block is cleared first, and that is load-bearing: a joint
    whose anchor sample fails keeps its previously persisted anchor, so
    without the reset a dead encoder would hand back an identical count every
    pass and score a perfect — entirely fictional — repeatability. After the
    reset, a joint present in the result was genuinely measured this pass.
    """
    hand.calibration = dataclasses.replace(
        baseline, joint_encoder_calibration_dict={}, joint_roms_measured_dict={}
    )

    # A span the routine rejects falls back to the config nominal, so the
    # measured value survives only in this event.
    rejected_spans: Dict[str, float] = {}

    def on_progress(event: dict) -> None:
        name = event.get("event")
        if name == "measured_rom_rejected":
            rejected_spans[event["joint"]] = float(event["span_deg"])
            rejections.setdefault(event["joint"], []).append(
                float(event["deviation_deg"])
            )
        elif name == "encoder_anchor_failed":
            failures.setdefault(event["joint"], []).append(str(event["error"]))
        _emit(progress_callback, "calibration_event", index=index, payload=dict(event))

    result = run_calibration(
        hand,
        joints=joints,
        joint_encoder_client=joint_encoder_client,
        force_wrist=True,
        persist=False,
        progress_callback=on_progress,
        should_stop=should_stop,
    )
    if result is None:
        return None

    harvested: Pass = {}
    for joint, cal in result.joint_encoder_calibration_dict.items():
        rom = result.joint_roms_measured_dict.get(joint)
        anchor = int(cal.enc_at_anchor_count)
        if rom:
            harvested[joint] = (anchor, float(rom[1] - rom[0]), False)
        elif joint in rejected_spans:
            harvested[joint] = (anchor, rejected_spans[joint], True)
        else:
            harvested[joint] = (anchor, None, False)
    return harvested


def run_anchor_repeatability(
    hand: "OrcaHand",
    *,
    joint_encoder_client,
    repeats: int = REPEATS,
    joints: List[str] | None = None,
    spread_good_deg: float = SPREAD_GOOD_DEG,
    spread_marginal_deg: float = SPREAD_MARGINAL_DEG,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> AnchorRepeatabilityResult | None:
    """Run the calibration ``repeats`` times and report per-joint anchor spread.

    Nothing is persisted: every pass runs with ``persist=False`` and the
    hand's in-memory calibration is restored on every exit path, including
    exceptions. Returns ``None`` on early exit (``should_stop`` triggered)
    before any pass completed; a stop partway through keeps the passes that
    did complete.

    Run this against a motor-only :class:`~orca_core.OrcaHand` with the joint
    control loop stopped — a sensing hand's ``connect()`` holds the serial
    port this routine's encoder client needs.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand` with joint feedback
            configured.
        joint_encoder_client: A streaming ``JointEncoderClient``.
        repeats: Calibration passes to run.
        joints: Restrict to these joints. Defaults to every calibrated joint.
        spread_good_deg: Peak-to-peak at or below which a joint scores ``GOOD``.
        spread_marginal_deg: Peak-to-peak at or below which it scores
            ``MARGINAL``; above it, ``POOR``.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``repeatability_started``, ``pass_started``,
            ``pass_done``, ``calibration_event`` wrapping each inner
            calibration event, ``repeatability_aborted``,
            ``repeatability_done``, ``cleanup_failed``). Must be fast and
            non-blocking; exceptions it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between passes and
            forwarded into each calibration run.

    Raises:
        ValueError: If ``repeats`` is not positive.
    """
    if repeats <= 0:
        raise ValueError("repeats must be positive")
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    baseline = hand.calibration
    passes: List[Pass] = []
    failures: Dict[str, List[str]] = {}
    rejections: Dict[str, List[float]] = {}

    def _restore_hand() -> None:
        hand.calibration = baseline
        hand.disable_torque()

    _emit(progress_callback, "repeatability_started", repeats=repeats)
    try:
        for index in range(repeats):
            if should_stop():
                break
            _emit(progress_callback, "pass_started", index=index, total=repeats)
            harvested = _run_one_pass(
                hand,
                baseline=baseline,
                joints=joints,
                joint_encoder_client=joint_encoder_client,
                failures=failures,
                rejections=rejections,
                index=index,
                progress_callback=progress_callback,
                should_stop=should_stop,
            )
            if harvested is None:
                break
            passes.append(harvested)
            _emit(progress_callback, "pass_done", index=index, total=repeats)
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        try:
            _restore_hand()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted repeatability run failed: %s", e)
        raise

    _restore_hand()

    if not passes:
        _emit(progress_callback, "repeatability_aborted")
        return None

    result = AnchorRepeatabilityResult(
        repeats=repeats,
        completed_passes=len(passes),
        calibration_current=float(hand.config.calibration_current),
        joints=_analyse(
            passes,
            failures=failures,
            rejections=rejections,
            nominal_spans={
                joint: float(rom[1] - rom[0])
                for joint, rom in hand.config.joint_roms_dict.items()
            },
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
        "repeatability_done",
        completed_passes=len(passes),
        failed=[stats.joint for stats in result.failed],
    )
    return result


def _analyse(
    passes: List[Pass],
    *,
    failures: Dict[str, List[str]],
    rejections: Dict[str, List[float]],
    nominal_spans: Dict[str, float],
    spread_good_deg: float,
    spread_marginal_deg: float,
) -> List[JointAnchorStats]:
    """Reduce the raw passes to per-joint anchor spread and span error."""
    joint_names = sorted({j for p in passes for j in p} | set(failures))
    stats: List[JointAnchorStats] = []

    for joint in joint_names:
        counts = [p[joint][0] for p in passes if joint in p]
        reasons = failures.get(joint, [])
        n_failed = len(reasons)
        nominal = nominal_spans.get(joint)

        mean_count = std_deg = peak_to_peak = None
        if len(counts) >= 2:
            mean_count = circular_mean_count(np.array(counts, dtype=np.uint16))
            residuals = [
                signed_count_delta(mean_count, count) * ENCODER_LSB_DEG
                for count in counts
            ]
            std_deg = round(statistics.stdev(residuals), 4)
            peak_to_peak = round(max(residuals) - min(residuals), 4)
            verdict = (
                VERDICT_UNRELIABLE
                if n_failed
                else _verdict_for(peak_to_peak, spread_good_deg, spread_marginal_deg)
            )
        else:
            verdict = VERDICT_UNRELIABLE if n_failed else VERDICT_NO_DATA

        spans = [p[joint][1] for p in passes if joint in p and p[joint][1] is not None]
        span_mean = span_spread = span_error = None
        if spans:
            span_mean = round(statistics.fmean(spans), 3)
            span_spread = round(max(spans) - min(spans), 3)
            if nominal:
                span_error = round(span_mean - nominal, 3)

        spans_rejected = sum(1 for p in passes if joint in p and p[joint][2])
        stats.append(
            JointAnchorStats(
                joint=joint,
                slot=JOINT_TO_ENCODER_SLOT.get(joint, -1),
                anchored_passes=len(counts),
                sample_failures=n_failed,
                verdict=verdict,
                mean_count=None if mean_count is None else round(mean_count, 2),
                std_deg=std_deg,
                peak_to_peak_deg=peak_to_peak,
                nominal_span_deg=None if nominal is None else round(nominal, 3),
                measured_span_mean_deg=span_mean,
                measured_span_spread_deg=span_spread,
                span_error_deg=span_error,
                spans_rejected=spans_rejected,
                messages=_messages_for(
                    joint=joint,
                    verdict=verdict,
                    reasons=reasons,
                    total_passes=len(passes),
                    anchored_passes=len(counts),
                    peak_to_peak=peak_to_peak,
                    spread_marginal_deg=spread_marginal_deg,
                    spans_rejected=spans_rejected,
                    rejection_deviations=rejections.get(joint, []),
                ),
            )
        )
    return stats


def _messages_for(
    *,
    joint: str,
    verdict: str,
    reasons: List[str],
    total_passes: int,
    anchored_passes: int,
    peak_to_peak: float | None,
    spread_marginal_deg: float,
    spans_rejected: int,
    rejection_deviations: List[float],
) -> List[str]:
    """Text for one joint: the anchor verdict, then any rejected span.

    A rejected span does not decide this step's verdict — that is J2's — but
    it leaves the joint on a configured ROM known to be wrong, so it is
    reported here rather than surviving only as a number.
    """
    messages: List[str] = []
    if verdict == VERDICT_UNRELIABLE:
        messages.append(
            f"Joint {joint} failed its anchor sample on {len(reasons)} of "
            f"{total_passes} passes: {'; '.join(dict.fromkeys(reasons))}"
        )
    elif verdict == VERDICT_NO_DATA:
        messages.append(
            f"Joint {joint} produced an anchor count on {anchored_passes} of "
            f"{total_passes} passes; at least two are needed to measure spread."
        )
    elif verdict == VERDICT_POOR:
        messages.append(
            f"Joint {joint} anchor moved {peak_to_peak:.2f}° peak-to-peak "
            f"across {anchored_passes} calibration passes (limit "
            f"{spread_marginal_deg:.2f}°)."
        )

    if spans_rejected and rejection_deviations:
        worst = max(rejection_deviations, key=abs)
        messages.append(
            f"Joint {joint} had its encoder-measured span rejected on "
            f"{spans_rejected} of {total_passes} passes, worst by {worst:+.1f}° "
            f"at the lower hardstop; the joint keeps its configured ROM."
        )
    return messages
