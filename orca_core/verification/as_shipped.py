# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Leave the hand in its shipping state and record exactly what ships with it.

The hashes are the point. A hand that comes back can be diffed against the
files it left the bench with, so "was it recalibrated in the field?" stops
being a matter of recollection.
"""

from __future__ import annotations

import hashlib
import logging
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from ..constants import NUM_STEPS, STEP_SIZE

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
        logger.exception("as-shipped progress callback failed")


@dataclass(frozen=True)
class AsShippedResult:
    """The pose, torque state and file hashes the hand ships with.

    Implements :class:`~orca_core.verification.StepResult`.
    """

    config_path: str
    calibration_path: str
    config_sha256: str | None
    calibration_sha256: str | None
    torque_off: bool
    at_neutral: bool
    thresholds: Dict[str, float] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.messages

    def measurements(self) -> Dict[str, float]:
        """Nothing here is a scalar worth trending — the hashes and the paths
        are the record, and they travel in the result rather than the fleet
        query."""
        return {}


def run_as_shipped_state(
    hand: "OrcaHand",
    *,
    move_to_neutral: bool = True,
    num_steps: int = NUM_STEPS,
    step_size: float = STEP_SIZE,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> AsShippedResult | None:
    """Pose the hand at neutral, release torque, and hash what it ships with.

    Torque is released whatever else happens, including on an exception:
    leaving the bench with the motors energised is the one outcome this step
    must never produce. Returns ``None`` on early exit (``should_stop``
    triggered before the pose).

    Args:
        hand: A connected, calibrated :class:`~orca_core.OrcaHand`.
        move_to_neutral: Command the configured neutral pose first. Set
            ``False`` for a hand that must not move.
        num_steps: Interpolation steps for the move to neutral.
        step_size: Seconds between interpolation steps.
        progress_callback: Optional ``callable(dict)`` invoked with structured
            progress events (``as_shipped_started``, ``posed_neutral``,
            ``torque_released``, ``as_shipped_aborted``, ``as_shipped_done``,
            ``cleanup_failed``). Must be fast and non-blocking; exceptions it
            raises are swallowed.
        should_stop: Optional ``callable() -> bool`` polled before the pose.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    _emit(progress_callback, "as_shipped_started")

    at_neutral = False
    try:
        if should_stop():
            _emit(progress_callback, "as_shipped_aborted")
            return None
        if move_to_neutral:
            hand.set_neutral_position(num_steps=num_steps, step_size=step_size)
            at_neutral = True
            _emit(progress_callback, "posed_neutral")
    except BaseException:
        try:
            hand.disable_torque()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("torque release after a failed as-shipped step failed: %s", e)
        raise

    failed_ids = hand.disable_torque()
    _emit(progress_callback, "torque_released", failed=list(failed_ids))

    config_path = hand.config.config_path
    calibration_path = hand.config.calibration_path
    result = AsShippedResult(
        config_path=config_path,
        calibration_path=calibration_path,
        config_sha256=_sha256(config_path),
        calibration_sha256=_sha256(calibration_path),
        torque_off=not failed_ids,
        at_neutral=at_neutral,
        messages=_messages(
            failed_ids=list(failed_ids),
            config_path=config_path,
            calibration_path=calibration_path,
        ),
    )
    _emit(
        progress_callback,
        "as_shipped_done",
        config_sha256=result.config_sha256,
        calibration_sha256=result.calibration_sha256,
    )
    return result


def _sha256(path: str) -> str | None:
    """Hash a file's bytes, or ``None`` when it is not there to hash."""
    try:
        with open(path, "rb") as handle:
            return hashlib.sha256(handle.read()).hexdigest()
    except OSError:
        return None


def _messages(
    *, failed_ids: List[int], config_path: str, calibration_path: str
) -> List[str]:
    messages: List[str] = []
    if failed_ids:
        messages.append(
            f"Motors {sorted(failed_ids)} did not acknowledge torque disable, "
            f"so the hand may still be energised."
        )
    for name, path in (
        ("config.yaml", config_path),
        ("calibration.yaml", calibration_path),
    ):
        if not os.path.isfile(path):
            messages.append(
                f"No {name} at {path}, so the hand ships with nothing to diff "
                f"a returned unit against."
            )
    return messages
