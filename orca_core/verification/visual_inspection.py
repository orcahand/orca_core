# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Pose the hand and let an operator look at it.

Catches gross assembly errors — a finger built onto the wrong slot, a tendon
routed outside its guide, binding audible in a slow sweep — that no counter
sees. The operator's verdict and their note are the whole measurement; there
is nothing here to compute.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional, TYPE_CHECKING

from ..constants import NUM_STEPS, STEP_SIZE

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
PromptCallback = Callable[[dict], object]
ShouldStop = Callable[[], bool]

SWEEP_HOLD_S = 1.5
"""Seconds each end of the ROM sweep is held, so binding is audible."""


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the routine."""
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("visual inspection progress callback failed")


@dataclass(frozen=True)
class VisualInspectionResult:
    """What the operator said.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    confirmed: bool
    note: str
    swept: bool
    thresholds: Dict[str, float] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return self.confirmed

    def measurements(self) -> Dict[str, float]:
        """An operator verdict is not a scalar; the result carries it directly."""
        return {}


def run_visual_inspection(
    hand: "OrcaHand",
    *,
    prompt_callback: Optional[PromptCallback] = None,
    sweep: bool = True,
    sweep_hold_s: float = SWEEP_HOLD_S,
    num_steps: int = NUM_STEPS,
    step_size: float = STEP_SIZE,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> VisualInspectionResult | None:
    """Move to neutral, sweep the full ROM, and ask the operator to confirm.

    Torque is released on every exit path, including exceptions. Returns
    ``None`` on early exit (``should_stop`` triggered before the prompt).

    Args:
        hand: A connected, calibrated :class:`~orca_core.OrcaHand`.
        prompt_callback: ``callable(dict)`` invoked with
            ``{"action": "confirm_visual_inspection"}`` and blocking until the
            operator answers. It returns either a bool, or a mapping with
            ``confirmed`` and an optional ``note``. Without one the step
            cannot reach a verdict and reports as unconfirmed.
        sweep: Drive a slow full-ROM sweep before asking.
        sweep_hold_s: Seconds held at each end of the sweep.
        num_steps: Interpolation steps per move.
        step_size: Seconds between interpolation steps.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``inspection_started``, ``posed_neutral``,
            ``swept``, ``inspection_aborted``, ``inspection_done``,
            ``cleanup_failed``). Must be fast and non-blocking; exceptions it
            raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled between moves.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    _emit(progress_callback, "inspection_started")
    swept = False
    try:
        if should_stop():
            _emit(progress_callback, "inspection_aborted")
            return None

        hand.set_neutral_position(num_steps=num_steps, step_size=step_size)
        _emit(progress_callback, "posed_neutral")

        if sweep and not should_stop():
            swept = _sweep_full_rom(
                hand, hold_s=sweep_hold_s, num_steps=num_steps,
                step_size=step_size, should_stop=should_stop,
            )
            _emit(progress_callback, "swept", completed=swept)
            hand.set_neutral_position(num_steps=num_steps, step_size=step_size)
    except BaseException:
        # Cleanup failures are reported, not raised, so the original error
        # propagates instead of being masked.
        try:
            hand.disable_torque()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("torque release after a failed inspection failed: %s", e)
        raise

    if should_stop():
        hand.disable_torque()
        _emit(progress_callback, "inspection_aborted")
        return None

    confirmed, note = _ask(prompt_callback)
    hand.disable_torque()

    messages: List[str] = []
    if not confirmed:
        detail = f": {note}" if note else "."
        messages.append(f"The operator did not confirm the hand at neutral{detail}")

    result = VisualInspectionResult(
        confirmed=confirmed, note=note, swept=swept, messages=messages
    )
    _emit(progress_callback, "inspection_done", confirmed=confirmed, note=note)
    return result


def _sweep_full_rom(
    hand: "OrcaHand",
    *,
    hold_s: float,
    num_steps: int,
    step_size: float,
    should_stop: ShouldStop,
) -> bool:
    """Drive every joint to both ends of its ROM. False if stopped partway."""
    for fraction in (0.0, 1.0):
        if should_stop():
            return False
        pose = hand.pose_from_fractions(
            {joint: fraction for joint in hand.config.joint_ids}
        )
        hand.set_joint_positions(pose, num_steps=num_steps, step_size=step_size)
        time.sleep(hold_s)
    return True


def _ask(prompt_callback: Optional[PromptCallback]) -> tuple[bool, str]:
    """Put the question to the operator and normalise whatever comes back.

    A missing callback means nobody was asked, which is not a confirmation.
    """
    if prompt_callback is None:
        return False, "no operator prompt was available"

    answer = prompt_callback({"action": "confirm_visual_inspection"})
    if isinstance(answer, Mapping):
        return bool(answer.get("confirmed")), str(answer.get("note", ""))
    return bool(answer), ""
