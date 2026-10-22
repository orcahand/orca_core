"""Put a hand's calibration onto the hand's own board.

orca_core owns the store format (:mod:`orca_core.board_store`) but not the
serial write, which lives with the firmware it talks to: the bench tool in
orca_firmware. That repo ships with neither orca_core nor a hand, so this is
best-effort by design -- on a machine without it nothing is written and the
hand's calibration stays where it always is, in this machine's keyed file.

The identity on the board is carried into every write. It lives in the same
store as the calibration, so a blob packed without it would make the board
forget which hand it is.
"""

import logging
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

from . import board_store
from .board_store import BoardIdentity

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


def firmware_tool(repo: Optional[Path] = None) -> Optional[Path]:
    """The firmware repo's store tool, if this machine has a checkout.

    ``ORCA_FIRMWARE_REPO`` points at one explicitly and is then the only place
    looked; otherwise a checkout beside orca_core is used, which is how the
    sibling repos sit on a dev machine. None when there is none -- the ordinary
    case for an installed wheel.
    """
    pinned = repo if repo is not None else os.environ.get(FIRMWARE_REPO_ENV)
    if pinned:
        where = Path(pinned)
    else:
        # orca_core/orca_core/board_push.py -> the checkout's parent
        where = Path(__file__).resolve().parents[2] / "orca_firmware"
    tool = where / TOOL_RELATIVE_PATH
    return tool if tool.is_file() else None


def board_state(port: str, timeout: float = 0.2) -> "tuple[Optional[BoardIdentity], int]":
    """The identity to carry forward and the sequence to beat.

    Read from the board's own store when it has one, else from what the board
    reports about itself, so a board written for the first time still keeps its
    identity. Sequence 0 means nothing readable is there.
    """
    import serial

    from .hardware.sensing.serial_discovery import probe_orca_info_over_link

    identity = None
    sequence = 0
    with serial.Serial(port, baudrate=115200, timeout=timeout, exclusive=True) as link:
        from . import board_store_link

        metadata = board_store_link.read_metadata_over_link(link)
        if metadata is not None:
            sequence = metadata.sequence
            try:
                blob = board_store_link.read_store_over_link(link, metadata)
                identity = board_store.unpack(blob).identity
            except Exception as exc:
                logger.debug("could not read the store on %s: %s", port, exc)
        if identity is None or not identity.provisioned:
            info = probe_orca_info_over_link(link)
            if info is not None:
                identity = BoardIdentity(
                    side=info.side,
                    hw_version=info.hw_version,
                    config=info.config,
                    serial=info.serial,
                )
    return identity, sequence


def push(
    hand_id: str,
    port: str,
    calibration: Mapping[str, Any],
    *,
    directory: Optional[Path] = None,
    repo: Optional[Path] = None,
) -> PushResult:
    """Write ``calibration`` onto the board on ``port``, keyed as ``hand_id``.

    The port must be free: the tool opens it itself, so this runs once the hand
    has let go of it. Nothing here raises -- a hand whose board could not be
    written is still a calibrated hand, and the reason comes back in the
    result for the caller to report.
    """
    tool = firmware_tool(repo)
    if tool is None:
        return PushResult(False, "no orca_firmware checkout on this machine")

    directory = Path(directory) if directory else board_store.calibrations_dir()
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
        blob = board_store.pack(
            calibration,
            identity=identity or BoardIdentity(),
            sequence=sequence + 1,
        )
    except board_store.BoardStoreError as exc:
        return PushResult(False, f"this calibration cannot be stored on a board ({exc})")

    # The tool reads the blob for this name out of the directory, so it has to
    # be in place under that name before the call. A write that does not land
    # must not leave a mirror claiming it did.
    path = directory / f"{hand_id}.bin"
    previous = path.read_bytes() if path.exists() else None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(blob)
    except OSError as exc:
        return PushResult(False, f"could not keep the blob to write ({exc})")

    command = [
        sys.executable, str(tool), "--dir", str(directory),
        "restore", "--port", port, "--board-id", hand_id,
        "--wait", str(RESTORE_WAIT_S),
    ]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError) as exc:
        _roll_back(path, previous)
        return PushResult(False, f"the firmware tool would not run ({exc})")

    if done.returncode != 0:
        _roll_back(path, previous)
        detail = (done.stderr or done.stdout or "").strip().splitlines()
        return PushResult(
            False,
            detail[-1] if detail else f"the firmware tool failed ({done.returncode})")
    return PushResult(True, sequence=sequence + 1, size=len(blob))


def _roll_back(path: Path, previous: Optional[bytes]) -> None:
    """Leave the mirror saying what the board actually holds."""
    try:
        if previous is None:
            path.unlink(missing_ok=True)
        else:
            path.write_bytes(previous)
    except OSError as exc:
        logger.debug("could not roll back %s: %s", path, exc)


def push_for_hand(hand, *, directory: Optional[Path] = None) -> PushResult:
    """Write what this hand's file says onto this hand's board.

    Skipped, not failed, when there is nothing to write to: a hand on a plain
    adapter has no board, and a hand nobody has named has no file. Call it once
    the hand has released its port.
    """
    from .utils.utils import read_yaml

    hand_id = getattr(hand, "hand_id", None)
    if not hand_id:
        return PushResult(False, "this hand has no id to file a calibration under")
    if not getattr(hand, "_board_hand_id", None):
        return PushResult(False, "this hand has no board to keep a calibration on")
    port = getattr(hand.config, "port", None)
    if not port or port == "auto":
        return PushResult(False, "the port this hand answered on is not known")

    document = read_yaml(hand.config.calibration_path) or {}
    if not document:
        return PushResult(False, "there is no calibration to write")
    return push(hand_id, port, document, directory=directory)
