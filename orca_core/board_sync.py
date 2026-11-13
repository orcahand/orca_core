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
and ``ORCA_CAL?`` itself, without putting them on the motor wire. A plain USB
adapter has no board to answer and its motors ignore the unframed queries, so
silence is the answer. The one answer that fails a connect is a controller
board (known by its USB IDs and ORCA's USB product string) that stays silent
through every retry: filing it as a board-less hand would put its files in a
folder other hands share. A stock board with the same USB IDs (an OpenRB-150
running its maker's firmware) answers nothing either, and is a plain adapter.

The calibration store is read whole only when it differs from the copy last
read (``board.bin`` in the hand's folder): the board reports the store's CRC
first, so an unchanged store costs one round trip.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .constants import ORCA_ID_PROBE_BAUDRATE
from .hardware.sensing.serial_discovery import OrcaBoardInfo, probe_orca_info_over_link

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class BoardContact:
    """What the board said at connect: its identity (``None`` for no board) and
    the bytes of its calibration store (``None`` when it holds none, or could
    not be read)."""

    info: Optional[OrcaBoardInfo] = None
    store: Optional[bytes] = None
    # A controller board is behind the port but did not say who it is.
    silent_board: bool = False
    # The USB serial number of a board whose firmware is too old to report an identity.
    usb_serial: Optional[str] = None


NO_BOARD = BoardContact()

# How often a controller board is asked for its identity before it counts as silent.
IDENTITY_ATTEMPTS = 3
IDENTITY_RETRY_S = 0.1


def is_board_port(port: str) -> bool:
    """Whether ``port`` is presented by a hand's controller board (by its USB IDs)."""
    from .hardware.sensing.serial_discovery import canonical_port, oh_board_ports

    try:
        return canonical_port(port) in oh_board_ports()
    except Exception as exc:
        logger.debug("could not list serial ports: %s", exc)
        return False


def _says_orca(port: str) -> bool:
    from .hardware.sensing.serial_discovery import oh_board_says_orca

    try:
        return oh_board_says_orca(port)
    except Exception as exc:
        logger.debug("could not list serial ports: %s", exc)
        return False


def _answers_legacy_id(link) -> bool:
    """Whether a board too old to report an identity answers ``ORCA_ID?``."""
    from .constants import (
        ORCA_ID_PROBE_TIMEOUT_S, ORCA_ID_QUERY, ORCA_ID_RESP_MOTOR, ORCA_ID_RESP_SENSOR)
    from .hardware.sensing.serial_discovery import _query_link

    replies = (ORCA_ID_RESP_MOTOR, ORCA_ID_RESP_SENSOR)
    return _query_link(link, ORCA_ID_PROBE_TIMEOUT_S, ORCA_ID_QUERY,
                       lambda buf: any(r in buf for r in replies)) is not None


def _old_firmware_contact(port: str) -> BoardContact:
    from .hardware.sensing.serial_discovery import oh_board_usb_serial

    usb_serial = oh_board_usb_serial(port)
    logger.warning(
        "the controller board on %s runs firmware too old to report its identity; update "
        "its firmware. Until then this hand is filed under %s.", port,
        f"its USB serial number ({usb_serial})" if usb_serial
        else "its model's shared no-board folder, with every such hand of that model")
    return BoardContact(usb_serial=usb_serial)


def contact_board(
    port: str, cached_store: Optional[Callable[[OrcaBoardInfo], Optional[bytes]]] = None
) -> BoardContact:
    """Ask the board behind ``port`` who it is and what calibration it holds.

    ``cached_store(info)`` returns the store as last read for that board; when
    its CRC matches what the board reports, it is used instead of a full read.
    """
    from . import board_store_link
    from .board_store import crc16

    info = None
    board = is_board_port(port)
    try:
        import serial

        with serial.Serial(port, baudrate=ORCA_ID_PROBE_BAUDRATE,
                           timeout=0.05, exclusive=True) as link:
            for attempt in range(IDENTITY_ATTEMPTS if board else 1):
                if attempt:
                    time.sleep(IDENTITY_RETRY_S)
                info = probe_orca_info_over_link(link)
                if info is not None:
                    break
            if info is None:
                if not board:
                    return NO_BOARD
                if not _answers_legacy_id(link):
                    return BoardContact(silent_board=_says_orca(port))
                # Firmware older than ORCA_INFO? has no identity to give.
                return _old_firmware_contact(port)
            _, metadata = board_store_link.query_metadata(link)
            if metadata is None:
                return BoardContact(info=info)
            cached = cached_store(info) if cached_store is not None else None
            if cached is not None and len(cached) == metadata.length \
                    and crc16(cached) == metadata.crc:
                return BoardContact(info=info, store=cached)
            return BoardContact(info=info,
                                store=board_store_link.read_store_over_link(link, metadata))
    except Exception as exc:
        logger.debug("board on %s: %s", port, exc)
        return BoardContact(info=info, silent_board=board and info is None and _says_orca(port))
