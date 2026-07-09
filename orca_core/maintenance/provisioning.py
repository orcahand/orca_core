# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Provision a hand's identity (side, hardware version) onto its OH board.

One-time assembly step: the board stores the identity in flash and reports
it via ``ORCA_INFO?``, which hand autodetection reads. A full-erase firmware
upload clears the stored identity, so re-run this after reflashing the board.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

from ..constants import (
    ORCA_ID_PROBE_BAUDRATE,
    ORCA_SET_RESP_ERR,
    ORCA_SET_RESP_OK,
)
from ..hardware.sensing.serial_discovery import (
    OrcaBoardInfo,
    oh_board_ports,
    probe_orca_info,
)

ProgressCallback = Callable[[dict], None]

_SET_RESPONSE_TIMEOUT_S = 1.0


class ProvisioningError(RuntimeError):
    """Provisioning could not complete (no board, no reply, or rejected)."""


def _send_set_command(port: str, command: bytes) -> bool:
    import serial

    with serial.Serial(
        port, baudrate=ORCA_ID_PROBE_BAUDRATE, timeout=0.05, exclusive=True
    ) as link:
        link.reset_input_buffer()
        link.write(command)
        link.flush()
        deadline = time.monotonic() + _SET_RESPONSE_TIMEOUT_S
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = link.read(256)
            if chunk:
                buf.extend(chunk)
                if ORCA_SET_RESP_OK in buf:
                    return True
                if ORCA_SET_RESP_ERR in buf:
                    return False
    raise ProvisioningError(
        f"no provisioning reply on {port}; the board may run firmware "
        "without identity support."
    )


def provision_hand(
    side: str,
    hw_version: Optional[int] = None,
    port: Optional[str] = None,
    progress_callback: Optional[ProgressCallback] = None,
) -> OrcaBoardInfo:
    """Store ``side`` (and optionally ``hw_version``) on the connected OH board.

    Args:
        side: ``"left"`` or ``"right"``.
        hw_version: Hand hardware version; the board keeps its stored (or
            default) value when ``None``.
        port: OH-board CDC to use; any of the board's ports works. ``None``
            picks one automatically.
        progress_callback: Optional ``callable(dict)`` receiving
            ``{"event": ...}`` progress updates.

    Returns:
        The identity the board reports after provisioning.

    Raises:
        ProvisioningError: No board found, no reply, or the board rejected
            or did not apply the request.
        ValueError: Bad ``side``/``hw_version`` value.
    """
    side_char = {"left": "L", "right": "R", "l": "L", "r": "R"}.get(side.lower())
    if side_char is None:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    if hw_version is not None and not 0 <= int(hw_version) <= 255:
        raise ValueError(f"hw_version must be 0..255, got {hw_version!r}")

    def _emit(event: str, **payload) -> None:
        if progress_callback is None:
            return
        try:
            progress_callback({"event": event, **payload})
        except Exception:
            pass

    candidates = [port] if port is not None else oh_board_ports()
    if not candidates:
        raise ProvisioningError("no OH board found (is the hand plugged in?)")

    command = f"ORCA_SET:SIDE={side_char}"
    if hw_version is not None:
        command += f";HW={int(hw_version)}"
    command_bytes = (command + "\n").encode("ascii")

    last_error: Optional[Exception] = None
    for candidate in candidates:
        _emit("provisioning", port=candidate, command=command)
        try:
            accepted = _send_set_command(candidate, command_bytes)
        except Exception as e:
            last_error = e
            continue
        if not accepted:
            raise ProvisioningError(f"board on {candidate} rejected: {command}")

        info = probe_orca_info(candidate)
        expected = "left" if side_char == "L" else "right"
        if info is None or info.side != expected:
            raise ProvisioningError(
                f"board on {candidate} accepted the request but reports "
                f"{info.side if info else 'no identity'} instead of {expected}."
            )
        _emit("provisioned", port=candidate, side=info.side,
              hw_version=info.hw_version, serial=info.serial)
        return info

    raise ProvisioningError(f"provisioning failed on every candidate port: {last_error}")
