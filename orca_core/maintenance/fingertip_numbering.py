# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Number tactile fingertips through the hand.

A fingertip's module number decides which finger slot its data lands in
(slot = module number, bus address = module + 1), and fresh fingertips all
share one default. The connector board renumbers a fingertip and builds the
sensor command itself; this module only drives the board's numbering
registers through a connected ``TactileClient``.

Fresh fingertips are numbered one at a time, with only that one plugged in.
Fingertips that already have distinct numbers can be renumbered in a full
hand by naming the one to change; ``NUMBERING_SPARE_MODULE`` is free for
swaps. Each change is ``scan_fingertips`` -> ``set_fingertip_module`` ->
power-cycle the hand -> ``scan_fingertips`` again to confirm. The board
pauses its own polling while a command runs.
"""
from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Optional

from ..hardware.sensing.constants import (
    ADDR_EXT_ID,
    ADDR_NUMBERING_COMMAND,
    ADDR_NUMBERING_IDENTITY,
    ADDR_NUMBERING_RAW_REPLY,
    ADDR_NUMBERING_STATUS,
    ADDR_SLOT_IDENTITY_START,
    DEFAULT_FINGER_TO_SENSOR_ID,
    EXT_CAP_FINGERTIP_NUMBERING,
    EXT_CAP_NUMBERING_SET_AT,
    EXT_CAP_SLOT_IDENTITY,
    EXT_ID_LENGTH,
    EXT_ID_MAGIC,
    FINGER_SENSOR_PART,
    NUMBERING_MAX_MODULE,
    NUMBERING_OP_SCAN,
    NUMBERING_OP_SET,
    NUMBERING_OP_SET_AT,
    NUMBERING_SET_KEY,
    NUMBERING_SPARE_MODULE,  # noqa: F401  (callers read it from here)
    NUMBERING_STATUS_LENGTH,
    SLOT_IDENTITY_LENGTH,
    SLOT_IDENTITY_STRIDE,
)
from ..hardware.tactile_client import TactileClient

STATE_BUSY = 1
STATE_DONE = 2
COMMAND_TIMEOUT_S = 2.0
POLL_INTERVAL_S = 0.02
# Highest bus address the board's scan covers.
SCAN_LAST_ADDRESS = 8

RESULTS = {
    0: "none",
    1: "set_ok",
    2: "scan_ok",
    10: "no_sensor",
    11: "multiple",
    12: "bus_error",
    13: "already",
    15: "no_reply",
    16: "bad_reply",
    17: "sensor_rejected",
    18: "taken",
}
RESULT_MESSAGES = {
    "no_sensor": "No fingertip answered. Check that the fingertip is plugged in.",
    "multiple": "Several fingertips are connected. Say which one to renumber.",
    "bus_error": "Two fingertips seem to share a number (or a cable is bad). Unplug one of them.",
    "already": "The fingertip already has that number.",
    "taken": "Another fingertip already has that number. Move it to the spare number first.",
    "no_reply": "The fingertip did not answer the numbering command.",
    "bad_reply": "The fingertip answered the numbering command unexpectedly.",
    "sensor_rejected": "The fingertip rejected the numbering command.",
}

_PART = re.compile(r"PXSR-[A-Z0-9]+")


class FingertipNumberingError(RuntimeError):
    """A numbering command was refused or failed; ``result`` names why."""

    def __init__(self, result: str, message: str):
        super().__init__(message)
        self.result = result


class FingertipNumberingUnsupported(FingertipNumberingError):
    """The connector-board firmware has no numbering registers."""


@dataclass(frozen=True)
class FingertipScan:
    """Which bus addresses answered, and the name string of a lone fingertip.

    ``garbled`` lists addresses whose reply came back broken, usually two
    fingertips sharing that number.
    """

    result: str
    addresses: tuple[int, ...]
    identity: str = ""
    garbled: tuple[int, ...] = ()

    @property
    def modules(self) -> tuple[int, ...]:
        return tuple(a - 1 for a in self.addresses)

    @property
    def garbled_modules(self) -> tuple[int, ...]:
        return tuple(a - 1 for a in self.garbled)

    @property
    def single_module(self) -> Optional[int]:
        """The module number when exactly one fingertip answered."""
        return self.modules[0] if self.result == "scan_ok" and len(self.addresses) == 1 else None

    @property
    def part(self) -> Optional[str]:
        return sensor_part(self.identity)


@dataclass(frozen=True)
class FingertipNumbering:
    """A renumbering sent to the fingertip; it applies after a power cycle.

    ``acknowledged`` is False when the fingertip sent no reply the board could
    read. The write may still have landed, so scan after the power cycle.
    """

    old_module: int
    new_module: int
    identity: str = ""
    acknowledged: bool = True
    raw_reply: bytes = b""


def sensor_part(identity: str) -> Optional[str]:
    """The Paxini part number in a sensor name string, if any."""
    match = _PART.search(identity or "")
    return match.group(0) if match else None


def read_extension_caps(client: TactileClient) -> int:
    """Capability bits of the board's numbering registers; raises if absent."""
    ext = client._read_register(ADDR_EXT_ID, EXT_ID_LENGTH)
    if ext[:2] != EXT_ID_MAGIC or not ext[3] & EXT_CAP_FINGERTIP_NUMBERING:
        raise FingertipNumberingUnsupported(
            "unsupported",
            "The connector-board firmware can't number fingertips; update it first.",
        )
    return ext[3]


def scan_fingertips(client: TactileClient, timeout_s: float = COMMAND_TIMEOUT_S) -> FingertipScan:
    """Ask the board which fingertip addresses answer (read-only)."""
    read_extension_caps(client)
    status = _run(client, bytes([NUMBERING_OP_SCAN, 0, 0, 0]), timeout_s)
    return FingertipScan(
        result=status["result"],
        addresses=status["addresses"],
        identity=status["identity"],
        garbled=status["garbled"],
    )


def set_fingertip_module(
    client: TactileClient,
    new_module: int,
    current: Optional[int] = None,
    timeout_s: float = COMMAND_TIMEOUT_S,
) -> FingertipNumbering:
    """Renumber a fingertip to ``new_module``.

    ``current`` picks the fingertip by its present number, so the others can
    stay connected as long as no two share a number and ``new_module`` is
    free. Without it, exactly one fingertip may be connected. The board
    re-checks all of this right before writing.
    """
    for n in (new_module, current):
        if n is not None and not 0 <= n <= NUMBERING_MAX_MODULE:
            raise ValueError(f"module number must be 0..{NUMBERING_MAX_MODULE}, got {n}")
    caps = read_extension_caps(client)
    scan = scan_fingertips(client, timeout_s)
    if scan.result == "bus_error":
        raise FingertipNumberingError("bus_error", _collision_message(scan))
    targeted = current is not None
    if not targeted:
        if scan.result != "scan_ok":
            raise FingertipNumberingError(scan.result, _message(scan.result))
        current = scan.single_module
    elif current not in scan.modules:
        raise FingertipNumberingError("no_sensor", f"No fingertip answers at module {current}.")
    if current == new_module:
        raise FingertipNumberingError("already", RESULT_MESSAGES["already"])
    if new_module in scan.modules:
        raise FingertipNumberingError("taken", RESULT_MESSAGES["taken"])

    if targeted and caps & EXT_CAP_NUMBERING_SET_AT:
        command = bytes([NUMBERING_OP_SET_AT, *NUMBERING_SET_KEY, new_module, current])
    elif len(scan.modules) == 1:
        command = bytes([NUMBERING_OP_SET, *NUMBERING_SET_KEY, new_module])
    else:
        raise FingertipNumberingError(
            "multiple",
            "This connector-board firmware renumbers only a lone fingertip; "
            "update it or unplug the others.",
        )
    status = _run(client, command, timeout_s)
    if status["result"] in ("no_reply", "bad_reply"):
        return FingertipNumbering(
            old_module=status["found_address"] - 1,
            new_module=new_module,
            identity=status["identity"],
            acknowledged=False,
            raw_reply=status["raw_reply"],
        )
    if status["result"] != "set_ok":
        message = _message(status["result"])
        if status["raw_reply"]:
            message += f" Reply: {status['raw_reply'].hex(' ')}"
        raise FingertipNumberingError(status["result"], message)
    return FingertipNumbering(
        old_module=status["found_address"] - 1,
        new_module=new_module,
        identity=status["identity"],
    )


def read_slot_identities(client: TactileClient) -> dict[str, str]:
    """Name string of the fingertip in each finger's slot ('' when absent)."""
    if not read_extension_caps(client) & EXT_CAP_SLOT_IDENTITY:
        raise FingertipNumberingUnsupported("unsupported", "The firmware doesn't report slot identities.")
    out = {}
    for finger, slot in client._finger_to_sensor_id.items():
        raw = client._read_register(ADDR_SLOT_IDENTITY_START + SLOT_IDENTITY_STRIDE * slot, SLOT_IDENTITY_LENGTH)
        out[finger] = _text(raw)
    return out


def identity_mismatches(identities: dict[str, str]) -> dict[str, tuple[str, str]]:
    """Fingers whose slot holds another fingertip model: {finger: (expected, found)}."""
    out = {}
    for finger, identity in identities.items():
        expected = FINGER_SENSOR_PART.get(finger)
        found = sensor_part(identity)
        if expected and found and found != expected:
            out[finger] = (expected, found)
    return out


def default_module(finger: str) -> int:
    """Module number a fingertip on ``finger`` should carry."""
    return DEFAULT_FINGER_TO_SENSOR_ID[finger]


def _run(client: TactileClient, command: bytes, timeout_s: float) -> dict:
    seq_before = client._read_register(ADDR_NUMBERING_STATUS, NUMBERING_STATUS_LENGTH)[2]
    try:
        client._write_register(ADDR_NUMBERING_COMMAND, command, attempts=1)
    except IOError as e:
        # A lost ACK fails the write too; the board marks a command it took busy before acking it.
        st = client._read_register(ADDR_NUMBERING_STATUS, NUMBERING_STATUS_LENGTH)
        if st[0] != STATE_BUSY and st[2] == seq_before:
            raise FingertipNumberingError("rejected", f"The board didn't take the numbering command: {e}") from e
    deadline = time.monotonic() + timeout_s
    while True:
        st = client._read_register(ADDR_NUMBERING_STATUS, NUMBERING_STATUS_LENGTH)
        if st[0] == STATE_DONE and st[2] != seq_before:
            break
        if time.monotonic() >= deadline:
            raise FingertipNumberingError("timeout", "The board didn't finish the numbering command.")
        time.sleep(POLL_INTERVAL_S)
    identity = _text(client._read_register(ADDR_NUMBERING_IDENTITY, st[8])) if st[8] else ""
    raw = bytes(client._read_register(ADDR_NUMBERING_RAW_REPLY, st[9])) if st[9] else b""
    return {
        "result": RESULTS.get(st[1], f"code_{st[1]}"),
        "found_address": st[4],
        "sensor_status": st[6],
        "addresses": _addresses(st[7]),
        "garbled": _addresses(st[10]),
        "identity": identity,
        "raw_reply": raw,
    }


def _addresses(mask: int) -> tuple[int, ...]:
    return tuple(a for a in range(1, SCAN_LAST_ADDRESS + 1) if mask >> (a - 1) & 1)


def _message(result: str) -> str:
    return RESULT_MESSAGES.get(result, f"Numbering failed ({result}).")


def _collision_message(scan: FingertipScan) -> str:
    if not scan.garbled_modules:
        return RESULT_MESSAGES["bus_error"]
    shared = ", ".join(str(m) for m in scan.garbled_modules)
    return f"Two fingertips seem to share module {shared} (or a cable is bad). Unplug one of them."


def _text(raw: bytes) -> str:
    return bytes(raw).split(b"\x00", 1)[0].decode("ascii", "replace")
