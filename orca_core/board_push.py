# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Put a hand's calibration onto the hand's own board.

orca_core owns the store format (:mod:`orca_core.board_store`) but not the
serial write, which lives with the firmware it talks to: the bench tool in
orca_firmware. That repo ships with neither orca_core nor a hand, so this is
best-effort by design -- on a computer without it nothing is written and the
hand's calibration stays where it always is, in the hand's folder.

Only scripts/calibrate.py pushes, after a successful run on a real hand.

The identity on the board is carried into every write. It lives in the same
store as the calibration, so a blob packed without it would make the board
forget which hand it is.
"""

import logging
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

from . import board_store, calibration_store
from .board_store import BoardIdentity
from .constants import ORCA_ID_PROBE_BAUDRATE

logger = logging.getLogger(__name__)

FIRMWARE_REPO_ENV = "ORCA_FIRMWARE_REPO"
TOOL_RELATIVE_PATH = Path("tools") / "bench" / "cal_store.py"

# Nothing here reflashes the board, so its application is already up; this only
# covers one busy finishing the reply it was in the middle of.
RESTORE_WAIT_S = 5


@dataclass(frozen=True)
class PushResult:
    """What became of a push. ``written`` False is a normal outcome."""

    written: bool
    reason: str = ""
    sequence: Optional[int] = None
    size: Optional[int] = None
    blob: Optional[bytes] = None


def firmware_tool() -> Optional[Path]:
    """The firmware repo's store tool, if this machine has a checkout.

    ``ORCA_FIRMWARE_REPO`` points at one explicitly and is then the only place
    looked; otherwise a checkout beside orca_core is used, which is how the
    sibling repos sit on a dev machine. None when there is none -- the ordinary
    case for an installed wheel.
    """
    tool = firmware_checkout() / TOOL_RELATIVE_PATH
    return tool if tool.is_file() else None


def firmware_checkout() -> Path:
    """Where the orca_firmware checkout is looked for: ``ORCA_FIRMWARE_REPO``, else beside orca_core."""
    pinned = os.environ.get(FIRMWARE_REPO_ENV)
    if pinned:
        return Path(pinned)
    # orca_core/orca_core/board_push.py -> the checkout's parent
    return Path(__file__).resolve().parents[2] / "orca_firmware"


def _missing_tool_reason() -> str:
    """Why there is no tool: no checkout where one was looked for, or one without the store tool."""
    where = firmware_checkout()
    if not where.is_dir():
        return f"no orca_firmware checkout at {where}"
    return f"the orca_firmware checkout at {where} has no {TOOL_RELATIVE_PATH.as_posix()}"


# How often the board is asked who it is before a write is refused.
INFO_ATTEMPTS = 3


def board_state(port: str, timeout: float = 0.2) -> "tuple[BoardIdentity, int]":
    """The identity to carry forward and the sequence to beat.

    Read from the board's own store when it has one, else from what the board
    reports about itself, so a board written for the first time still keeps its
    identity. Sequence 0 means the board said it holds nothing.

    Raises:
        OSError: the board never said what it holds, or who it is: a write
            then could go out under a sequence the board ignores, or make
            it forget which hand it is.
    """
    import serial

    from . import board_store_link
    from .hardware.sensing.serial_discovery import probe_orca_info_over_link

    identity = None
    with serial.Serial(port, baudrate=ORCA_ID_PROBE_BAUDRATE, timeout=timeout,
                       exclusive=True) as link:
        answered, metadata = board_store_link.query_metadata(link)
        if not answered:
            raise OSError("the board did not say what calibration it holds")
        sequence = metadata.sequence if metadata is not None else 0
        if metadata is not None:
            try:
                blob = board_store_link.read_store_over_link(link, metadata)
                identity = board_store.unpack(blob).identity
            except Exception as exc:
                logger.debug("could not read the store on %s: %s", port, exc)
        for _ in range(INFO_ATTEMPTS):
            if identity is not None and identity.provisioned:
                break
            info = probe_orca_info_over_link(link)
            if info is not None:
                identity = BoardIdentity(
                    side=info.side,
                    hw_version=info.hw_version,
                    config=info.config,
                    serial=info.serial,
                )
    if identity is None:
        raise OSError("the board did not say which hand it is")
    return identity, sequence


def board_serves(port: str, blob: bytes, timeout: float = 0.2) -> bool:
    """Whether the board on ``port`` now serves ``blob`` as its store."""
    import serial

    from . import board_store_link
    from .board_store import crc16

    with serial.Serial(port, baudrate=ORCA_ID_PROBE_BAUDRATE, timeout=timeout,
                       exclusive=True) as link:
        _, metadata = board_store_link.query_metadata(link)
    return (metadata is not None and metadata.length == len(blob)
            and metadata.crc == crc16(blob))


def push(
    hand_id: str,
    port: str,
    calibration: Mapping[str, Any],
) -> PushResult:
    """Write ``calibration`` onto the board on ``port``, keyed as ``hand_id``.

    The port must be free: the tool opens it itself, so this runs once the hand
    has let go of it. Nothing here raises -- a hand whose board could not be
    written is still a calibrated hand, and the reason comes back in the
    result for the caller to report.
    """
    tool = firmware_tool()
    if tool is None:
        return PushResult(False, _missing_tool_reason())

    try:
        identity, sequence = board_state(port)
    except Exception as exc:
        # A locked port means the hand itself still has it: the tool opens the
        # port too, so the write waits until the hand has let go.
        if "lock" in str(exc).lower() or "unavailable" in str(exc).lower():
            return PushResult(
                False, "the hand is still using its port; the board can be "
                       "written once the hand has disconnected")
        return PushResult(False, f"the board on {port} did not answer ({exc})")

    try:
        blob = board_store.pack(calibration, identity=identity, sequence=sequence + 1)
    except board_store.BoardStoreError as exc:
        return PushResult(False, f"this calibration cannot be stored on a board ({exc})")

    # The tool reads the blob for this name out of the directory it is given.
    with tempfile.TemporaryDirectory(prefix="orca-board-push-") as directory:
        (Path(directory) / f"{hand_id}.bin").write_bytes(blob)
        command = [
            sys.executable, str(tool), "--dir", directory,
            "restore", "--port", port, "--board-id", hand_id,
            "--wait", str(RESTORE_WAIT_S),
        ]
        try:
            done = subprocess.run(command, capture_output=True, text=True, timeout=120)
        except (OSError, subprocess.SubprocessError) as exc:
            return PushResult(False, f"the firmware tool would not run ({exc})")

    if done.returncode != 0:
        detail = (done.stderr or done.stdout or "").strip().splitlines()
        return PushResult(
            False,
            detail[-1] if detail else f"the firmware tool failed ({done.returncode})")
    try:
        served = board_serves(port, blob)
    except Exception as exc:
        return PushResult(False, f"what the board holds now could not be read back ({exc})")
    if not served:
        return PushResult(False, "the board does not serve the calibration just written")
    return PushResult(True, sequence=sequence + 1, size=len(blob), blob=blob)


def push_for_hand(hand) -> PushResult:
    """Write what this hand's calibration file says onto this hand's board.

    Skipped, not failed, when there is nothing to write to: a hand on a plain
    adapter has no board. Call it once the hand has released its port. The
    board copy carries the measurements, stamp and fingerprints, plus the
    ratios in force for readers that still expect them.
    """
    if not hand.board_hand_id:
        return PushResult(False, "this hand has no board to keep a calibration on")
    port = getattr(hand.config, "port", None)
    if not port or port == "auto":
        return PushResult(False, "the port this hand answered on is not known")
    path = hand.config.calibration_path
    document = calibration_store.read_document(path)
    if not document.get("motor_limits"):
        return PushResult(False, "there is no calibration to write")
    document = dict(document)
    document["joint_to_motor_ratios"] = dict(hand.calibration.joint_to_motor_ratios_dict)
    result = push(hand.hand_id, port, document)
    if result.written:
        hand._remember_board_store(result.blob)
    return result
