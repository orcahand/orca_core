# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""What a hand's controller board says when a hand connects.

Asked over the motor port in the window after the motor driver is resolved
and before the motor client takes the port: the board answers ``ORCA_INFO?``
itself, without putting it on the motor wire. A plain USB adapter has no board
to answer and its motors ignore the unframed query, so silence is the answer.
Nothing here can fail a connect.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

from .constants import ORCA_ID_PROBE_BAUDRATE
from .hardware.sensing.serial_discovery import OrcaBoardInfo, probe_orca_info_over_link

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BoardContact:
    """The board's identity (``None`` for no board) as read at connect."""

    info: Optional[OrcaBoardInfo] = None


NO_BOARD = BoardContact()


def contact_board(port: str) -> BoardContact:
    """Ask the board behind ``port`` who it is; :data:`NO_BOARD` when nothing answers."""
    try:
        import serial

        with serial.Serial(port, baudrate=ORCA_ID_PROBE_BAUDRATE,
                           timeout=0.05, exclusive=True) as link:
            info = probe_orca_info_over_link(link)
    except Exception as exc:
        logger.debug("no board identity on %s: %s", port, exc)
        return NO_BOARD
    return BoardContact(info=info)
