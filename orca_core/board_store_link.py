"""Reading a hand's calibration store off its own board, over the motor port.

The board answers ``ORCA_CAL?`` in band — the bridge intercepts it rather than
putting it on the wire — so the store is read through the same port the motors
use, while no session holds it.

The metadata line comes first and carries the whole store's CRC. When that
matches the copy already mirrored on this machine, nothing else is pulled: a
connect costs one round trip, not one per chunk. Only a board whose store has
changed is read in full.

Chunks each carry their own CRC, so a corrupt line is re-requested rather than
failing the read at the end; a chunk that will not come back clean fails the
read, because half a calibration is worse than none.
"""

from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass
from typing import Optional

from .board_store import crc16
from .constants import ORCA_ID_PROBE_BAUDRATE, ORCA_ID_PROBE_TIMEOUT_S

logger = logging.getLogger(__name__)

STORE_QUERY = b"ORCA_CAL?\n"
#: Per-line timeout for the chunks. A chunk is one flash page read out of
#: memory-mapped flash, so the board answers immediately or not at all.
LINE_TIMEOUT_S = 0.5
#: Timeout for the first question, which is the one a hand with no ORCA board
#: pays on every connect: the same budget the ``ORCA_ID?`` probe already uses,
#: so asking costs no more than asking what the board is.
METADATA_TIMEOUT_S = ORCA_ID_PROBE_TIMEOUT_S
#: How many times one line is asked for again before the read is given up.
LINE_RETRIES = 2

_META = re.compile(
    rb"ORCA:CAL;SLOT=(\d+);SEQ=(\d+);LEN=(\d+);CRC=([0-9A-Fa-f]{4});CHUNK=(\d+)"
)
_CHUNK = re.compile(rb"ORCA:CALD;(\d+);([0-9A-Fa-f]{4});([0-9A-Fa-f]*)")
_NONE = re.compile(rb"ORCA:CAL;NONE")


@dataclass(frozen=True)
class StoreMetadata:
    """What the board says it is holding, before any of it is pulled."""

    slot: int
    sequence: int
    length: int
    crc: int
    chunk_bytes: int


def _ask(link, query: bytes, pattern: "re.Pattern[bytes]", timeout: float):
    """Send ``query`` and return the first match in what comes back."""
    try:
        link.reset_input_buffer()
        link.write(query)
        link.flush()
    except Exception as exc:
        logger.debug("calibration-store query %r failed: %s", query, exc)
        return None
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = link.read(512)
        if not chunk:
            continue
        buf.extend(chunk)
        found = pattern.search(buf)
        if found:
            return found
        if _NONE.search(buf):
            return None
    return None


def read_metadata_over_link(link,
                            timeout: float = METADATA_TIMEOUT_S) -> Optional[StoreMetadata]:
    """What the board holds, or ``None`` when it holds nothing or is not one."""
    found = _ask(link, STORE_QUERY, _META, timeout)
    if found is None:
        return None
    return StoreMetadata(
        slot=int(found.group(1)),
        sequence=int(found.group(2)),
        length=int(found.group(3)),
        crc=int(found.group(4), 16),
        chunk_bytes=int(found.group(5)),
    )


def read_store_over_link(link, metadata: Optional[StoreMetadata] = None,
                         timeout: float = LINE_TIMEOUT_S,
                         retries: int = LINE_RETRIES) -> Optional[bytes]:
    """Pull the whole store. ``None`` when the board has none to give.

    Raises ``OSError`` when a chunk will not come back clean: a store read down
    the middle must not be mistaken for the hand's calibration.
    """
    meta = metadata or read_metadata_over_link(link, timeout)
    if meta is None:
        return None
    if meta.length == 0 or meta.chunk_bytes == 0:
        return None

    blob = bytearray()
    for index in range(-(-meta.length // meta.chunk_bytes)):
        for _ in range(retries + 1):
            found = _ask(link, b"ORCA_CAL?%d\n" % index, _CHUNK, timeout)
            if found is None or int(found.group(1)) != index:
                continue
            payload = bytes.fromhex(found.group(3).decode("ascii"))
            if crc16(payload) == int(found.group(2), 16):
                blob.extend(payload)
                break
        else:
            raise OSError(
                f"the board would not read back chunk {index} of its calibration store"
            )
    blob = bytes(blob[:meta.length])
    if crc16(blob) != meta.crc:
        raise OSError(
            "the calibration store read off the board does not match the CRC it reported"
        )
    return blob


def read_store(port: str, baudrate: int = ORCA_ID_PROBE_BAUDRATE,
               timeout: float = LINE_TIMEOUT_S) -> Optional[bytes]:
    """Open ``port`` exclusively and read the store on it."""
    import serial

    with serial.Serial(port, baudrate=baudrate, timeout=0.05, exclusive=True) as link:
        return read_store_over_link(link, timeout=timeout)
