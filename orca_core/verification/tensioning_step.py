# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Hold the hand for manual tendon tensioning and keep what the wind measured.

The tensioning routine already winds each motor to a stall in both
directions and throws the stall positions away. The travel between them is
that motor's tendon wind, so a misrouted tendon or a mis-wound spool is an
outlier against the fleet — a build-quality number for no extra bench time.

The operator is prompted once per round, when the routine reaches its hold.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from ..maintenance.tensioning import run_jitter, run_tension
from .step_result import flat_measurements

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
PromptCallback = Callable[[dict], object]
ShouldStop = Callable[[], bool]

ROUNDS = 1
"""Tension/jitter rounds. The assembly procedure decides the real number, so
the plan passes it rather than this being a constant."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("tensioning step progress callback failed")


@dataclass(frozen=True)
class MotorWind:
    """One motor's stall positions and the travel between them.

    ``wind_rad`` is ``None`` until both directions have stalled; a direction
    that ran out of time before stalling is flagged in ``stalled``, because a
    wind measured from a drive that never stopped is not a wind.
    """

    motor: int
    joint: str
    positions: Dict[int, float] = field(default_factory=dict)
    stalled: Dict[int, bool] = field(default_factory=dict)
    wind_rad: float | None = None


@dataclass(frozen=True)
class TensioningResult:
    """Each motor's tendon wind and how many rounds were performed.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    rounds: int
    motors: List[MotorWind]
    thresholds: Dict[str, float] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.messages

    def measurements(self) -> Dict[str, float]:
        """Per joint: the tendon wind, and each direction's stall position."""
        return {
            **flat_measurements(
                "tendon_wind_rad", {m.joint: m.wind_rad for m in self.motors}
            ),
            **flat_measurements(
                "stall_position_flex_rad",
                {m.joint: m.positions.get(1) for m in self.motors},
            ),
            **flat_measurements(
                "stall_position_extend_rad",
                {m.joint: m.positions.get(-1) for m in self.motors},
            ),
            "rounds": float(self.rounds),
        }


def run_tensioning(
    hand: "OrcaHand",
    *,
    rounds: int = ROUNDS,
    jitter_between_rounds: bool = True,
    prompt_callback: Optional[PromptCallback] = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> TensioningResult | None:
    """Wind the tendons taut, hold for the operator, and record each wind.

    Each round winds both directions, holds under current, and prompts the
    operator; the hold ends when they confirm. The prompt is raised from
    inside the hold's stop poll, so the routine is never asked for a human
    before it is actually holding.

    ``run_tension`` restores the configured control mode and current limit
    and releases torque on every exit path, including exceptions.

    Args:
        hand: A connected :class:`~orca_core.OrcaHand`.
        rounds: Tension rounds to perform.
        jitter_between_rounds: Seat the tendons before each round after the
            first.
        prompt_callback: ``callable(dict)`` invoked with
            ``{"action": "tension_tendons", "round": n, "rounds": n}`` and
            blocking until the operator is done. Without one the hold ends
            immediately, which is only useful on a mock.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``tensioning_started``, ``round_started``,
            ``tension_event`` wrapping each inner tensioning event,
            ``round_done``, ``tensioning_aborted``, ``tensioning_done``).
            Must be fast and non-blocking; exceptions it raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled throughout.

    Raises:
        ValueError: If ``rounds`` is not positive.
    """
    if rounds <= 0:
        raise ValueError("rounds must be positive")
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    motor_to_joint = hand.config.motor_to_joint_dict
    # {motor: {direction: (position, stalled)}} from the last round to stall it.
    winds: Dict[int, Dict[int, tuple]] = {}
    rounds_done = 0

    _emit(progress_callback, "tensioning_started", rounds=rounds)
    for index in range(rounds):
        if should_stop():
            break
        _emit(progress_callback, "round_started", index=index, total=rounds)
        if jitter_between_rounds and index:
            run_jitter(hand, should_stop=should_stop)

        holding = False
        confirmed = False

        def on_progress(event: dict, _index: int = index) -> None:
            nonlocal holding
            if event.get("event") == "phase" and event.get("phase") == "holding":
                holding = True
            elif event.get("event") == "stall_recorded":
                winds.setdefault(int(event["motor"]), {})[int(event["direction"])] = (
                    float(event["position"]),
                    bool(event["stalled"]),
                )
            _emit(progress_callback, "tension_event", index=_index, payload=dict(event))

        def hold_until_confirmed() -> bool:
            nonlocal confirmed
            if should_stop():
                return True
            if holding and not confirmed:
                _prompt(prompt_callback, action="tension_tendons",
                        round=index + 1, rounds=rounds)
                confirmed = True
            return confirmed

        run_tension(
            hand,
            move_motors=True,
            progress_callback=on_progress,
            should_stop=hold_until_confirmed,
        )
        rounds_done += 1
        _emit(progress_callback, "round_done", index=index, total=rounds)

    if not rounds_done:
        _emit(progress_callback, "tensioning_aborted")
        return None

    result = TensioningResult(
        rounds=rounds_done,
        motors=_collect(winds, motor_to_joint=motor_to_joint),
    )
    _emit(progress_callback, "tensioning_done", rounds=rounds_done)
    return result


def _prompt(prompt_callback: Optional[PromptCallback], **payload) -> None:
    """Ask the operator to act. A missing callback means nobody to ask."""
    if prompt_callback is None:
        return
    prompt_callback(dict(payload))


def _collect(
    winds: Dict[int, Dict[int, tuple]], *, motor_to_joint: Dict[int, str]
) -> List[MotorWind]:
    """Turn the per-direction stall records into one wind per motor."""
    motors: List[MotorWind] = []
    for motor_id in sorted(winds):
        by_direction = winds[motor_id]
        positions = {d: p for d, (p, _) in by_direction.items()}
        stalled = {d: s for d, (_, s) in by_direction.items()}
        wind = None
        if len(positions) == 2 and all(stalled.values()):
            wind = round(abs(positions[1] - positions[-1]), 4)
        motors.append(
            MotorWind(
                motor=motor_id,
                joint=motor_to_joint.get(motor_id, f"motor_{motor_id}"),
                positions=positions,
                stalled=stalled,
                wind_rad=wind,
            )
        )
    return motors
