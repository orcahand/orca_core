# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Leaving a maintenance routine: the hand's settings back, and torque off whatever happens."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Callable, Optional

from .progress import ProgressCallback, progress_emitter

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)
_emit = progress_emitter(logger, "release progress callback failed")


def release_hand(
    hand: "OrcaHand",
    restore: Callable[[], None],
    *,
    routine: str,
    progress_callback: Optional[ProgressCallback] = None,
) -> None:
    """Run ``restore`` (the hand's mode and current limit back), then switch
    torque off on every motor even when ``restore`` raised; its error still
    propagates. Motors left energized are logged and reported as a
    ``torque_off_failed`` event (``motors``, ``joints``)."""
    still_on = list(hand.config.motor_ids)
    try:
        restore()
    finally:
        try:
            still_on = list(hand.disable_torque() or [])
        finally:
            if still_on:
                joint_of = hand.config.motor_to_joint_dict
                logger.error("%s could not switch torque off on motors %s", routine, still_on)
                _emit(
                    progress_callback,
                    "torque_off_failed",
                    motors=still_on,
                    joints=[joint_of.get(m) for m in still_on],
                )
