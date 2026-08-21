# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Score tendon friction from the traverse the calibration already drove.

The target is over-tension, not slack. An over-tensioned tendon raises
friction: the motor draws more current through the traverse and covers less
travel per commanded increment. Slack is not reliably detectable from current
and is not attempted here.

Friction is a property of movement, not of a pose, so it can only be measured
during a traverse — and the traverse that already happens is the calibration
drive, so this costs no bench time. The terminal stall is excluded: current
there measures the hardstop, not the travel to it.

Per joint *per direction*, because the asymmetry is large — one hand measured
88 mA one way against 179 mA the other on the same joint.
"""

from __future__ import annotations

import logging
import statistics
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple

from .step_result import flat_measurements

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]

STALL_EXCLUSION_RAD = 0.01
"""Motion below this between samples counts as stalled rather than moving.
Matches the stall threshold the tensioning routine winds against."""

MIN_MOVING_SAMPLES = 3
"""Below this a traverse has too little movement to average a current over."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("tendon friction progress callback failed")


def _direction_name(direction: int) -> str:
    return "flex" if direction > 0 else "extend"


@dataclass(frozen=True)
class TraverseFriction:
    """One joint's traverse in one direction.

    ``steps`` is the number of commanded increments the drive took to reach
    its stability criterion: under a current cap, an over-tensioned joint
    covers less travel per increment, so a high count is the other half of
    the signature.
    """

    joint: str
    motor: int
    direction: int
    direction_name: str
    reached_limit: bool
    steps: int
    moving_samples: int
    travel_rad: float | None
    current_mean_ma: float | None
    current_p90_ma: float | None
    messages: List[str] = field(default_factory=list)

    @property
    def key(self) -> str:
        """Measurement key for this joint/direction pair."""
        return f"{self.joint}.{self.direction_name}"

    @property
    def ok(self) -> bool:
        return not self.messages


@dataclass(frozen=True)
class TendonFrictionResult:
    """Every traverse's friction proxy plus the bands that judged them.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    traverses: List[TraverseFriction]
    thresholds: Dict[str, float]

    @property
    def failed(self) -> List[TraverseFriction]:
        return [traverse for traverse in self.traverses if not traverse.ok]

    @property
    def passed(self) -> bool:
        return not self.failed

    @property
    def messages(self) -> List[str]:
        return [m for traverse in self.traverses for m in traverse.messages]

    def measurements(self) -> Dict[str, float]:
        """Keyed ``<metric>.<joint>.<direction>``, because the two directions
        of one joint are different measurements, not repeats of one."""
        return {
            **flat_measurements(
                "traverse_current_mean_ma",
                {t.key: t.current_mean_ma for t in self.traverses},
            ),
            **flat_measurements(
                "traverse_current_p90_ma",
                {t.key: t.current_p90_ma for t in self.traverses},
            ),
            **flat_measurements(
                "traverse_travel_rad", {t.key: t.travel_rad for t in self.traverses}
            ),
            **flat_measurements(
                "traverse_steps", {t.key: float(t.steps) for t in self.traverses}
            ),
        }


def analyse_tendon_friction(
    traverses: Iterable,
    *,
    current_bands_ma: Mapping[str, Tuple[float, float]] | None = None,
    stall_exclusion_rad: float = STALL_EXCLUSION_RAD,
    min_moving_samples: int = MIN_MOVING_SAMPLES,
    progress_callback: Optional[ProgressCallback] = None,
) -> TendonFrictionResult:
    """Reduce calibration traverses to a per-joint-per-direction friction proxy.

    No motion and no hardware: this consumes
    :attr:`~orca_core.verification.CalibrationStepResult.traverses`, so it can
    be re-run against stored telemetry when the bands change.

    ``current_bands_ma`` has no default. The per-joint bands differ
    substantially — abduction joints especially — and must be set from fleet
    data rather than assumed, so until they exist every traverse is recorded
    and none is gated. The extreme case, a joint that never reaches its
    hardstop at all, is already graded by the calibration step's span check.

    Args:
        traverses: :class:`~orca_core.verification.Traverse` records from a
            calibration step.
        current_bands_ma: ``{"<joint>.<direction>": (min, max)}`` or
            ``{"<joint>": (min, max)}`` for both directions. Traverses absent
            from it are recorded but not gated.
        stall_exclusion_rad: Motion below this between samples counts as
            stalled and is excluded from the moving portion.
        min_moving_samples: Below this a traverse yields no current average.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``friction_analysed``). Must be fast and
            non-blocking; exceptions it raises are swallowed.
    """
    bands = dict(current_bands_ma or {})
    analysed = [
        _analyse_traverse(
            traverse,
            bands=bands,
            stall_exclusion_rad=stall_exclusion_rad,
            min_moving_samples=min_moving_samples,
        )
        for traverse in traverses
    ]

    thresholds: Dict[str, float] = {"stall_exclusion_rad": float(stall_exclusion_rad)}
    for key, band in bands.items():
        thresholds[f"traverse_current_mean_ma.{key}.min"] = float(band[0])
        thresholds[f"traverse_current_mean_ma.{key}.max"] = float(band[1])

    _emit(
        progress_callback,
        "friction_analysed",
        traverses=len(analysed),
        failed=[t.key for t in analysed if not t.ok],
    )
    return TendonFrictionResult(traverses=analysed, thresholds=thresholds)


def _moving_currents(
    positions: List[float], currents: List[float], *, stall_exclusion_rad: float
) -> Tuple[List[float], float | None]:
    """Currents from the moving portion only, and the travel they cover.

    The terminal stall is where the motor is pressing a hardstop, so its
    current describes the hardstop rather than the friction of getting there.

    Motion between two samples is attributed to the current logged at the
    *earlier* one — that is the draw that produced the move. Taking the later
    sample instead would fold in the arrival at the hardstop, where the
    current has already jumped but the position has only just settled.
    """
    moving: List[float] = []
    travelled = 0.0
    for index in range(1, min(len(positions), len(currents))):
        step = abs(positions[index] - positions[index - 1])
        if step < stall_exclusion_rad:
            continue
        travelled += step
        moving.append(currents[index - 1])
    return moving, (travelled if moving else None)


def _percentile(values: List[float], fraction: float) -> float:
    """Nearest-rank percentile, so a short traverse still yields a number."""
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(fraction * len(ordered)) - 1))
    return ordered[index]


def _analyse_traverse(
    traverse,
    *,
    bands: Mapping[str, Tuple[float, float]],
    stall_exclusion_rad: float,
    min_moving_samples: int,
) -> TraverseFriction:
    """Reduce one traverse to its friction proxy and verdict."""
    direction_name = _direction_name(traverse.direction)
    key = f"{traverse.joint}.{direction_name}"
    moving, travel = _moving_currents(
        traverse.positions, traverse.currents, stall_exclusion_rad=stall_exclusion_rad
    )

    mean_ma = p90_ma = None
    if len(moving) >= min_moving_samples:
        mean_ma = round(statistics.fmean(abs(c) for c in moving), 2)
        p90_ma = round(_percentile([abs(c) for c in moving], 0.9), 2)

    messages: List[str] = []
    band = bands.get(key, bands.get(traverse.joint))
    if band is not None and mean_ma is not None:
        low, high = band
        if not low <= mean_ma <= high:
            messages.append(
                f"Joint {traverse.joint} drew {mean_ma:.0f} mA mean through its "
                f"{direction_name} traverse (expected {low:.0f}–{high:.0f}) and "
                f"needed {len(traverse.positions)} increments to reach its "
                f"limit. Elevated traverse current with slow travel is the "
                f"signature of an over-tensioned tendon."
            )

    return TraverseFriction(
        joint=traverse.joint,
        motor=traverse.motor,
        direction=traverse.direction,
        direction_name=direction_name,
        reached_limit=traverse.reached_limit,
        steps=len(traverse.positions),
        moving_samples=len(moving),
        travel_rad=None if travel is None else round(travel, 4),
        current_mean_ma=mean_ma,
        current_p90_ma=p90_ma,
        messages=messages,
    )
