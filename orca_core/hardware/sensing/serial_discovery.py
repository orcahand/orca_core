"""Locate the tactile and joint-encoder serial ports.

Paxini board is identified by VID. The OH board's two CDCs share VID/PID and
are distinguished by an ``ORCA_ID?`` probe (firmware replies ``ORCA:MOTOR``
or ``ORCA:SENSOR``). When only the OH sensor CDC is present, both fields
point at it (``shared=True``).
"""

import logging
from dataclasses import dataclass
from typing import Optional

from ...constants import (
    KNOWN_VIDS,
    ORCA_ID_PROBE_BAUDRATE,
    ORCA_ID_PROBE_TIMEOUT_S,
    ORCA_ID_QUERY,
    ORCA_ID_RESP_SENSOR,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SensingPorts:
    tactile: Optional[str]
    encoder: Optional[str]

    @property
    def shared(self) -> bool:
        return (
            self.tactile is not None
            and self.encoder is not None
            and self.tactile == self.encoder
        )


def _probe_orca_id(
    port: str,
    baudrate: int = ORCA_ID_PROBE_BAUDRATE,
    timeout: float = ORCA_ID_PROBE_TIMEOUT_S,
) -> Optional[bytes]:
    """Send ORCA_ID? and return the response bytes, or None on open/timeout failure."""
    import serial

    try:
        with serial.Serial(port, baudrate=baudrate, timeout=timeout) as link:
            link.reset_input_buffer()
            link.write(ORCA_ID_QUERY)
            link.flush()
            response = link.read_until(b"\n", size=32)
            return response if response.endswith(b"\n") else None
    except (OSError, serial.SerialException) as exc:
        logger.debug("ORCA_ID? probe on %s failed: %s", port, exc)
        return None


def find_tactile_port() -> Optional[str]:
    """Return the Paxini board's device path, or None if zero or >1 match."""
    import serial.tools.list_ports

    matches = [
        p for p in serial.tools.list_ports.comports()
        if p.vid in KNOWN_VIDS["tactile_sensor"]
    ]
    if len(matches) == 1:
        return matches[0].device
    return None


def _find_oh_sensor_port() -> Optional[str]:
    import serial.tools.list_ports

    oh_candidates = [
        p for p in serial.tools.list_ports.comports()
        if p.vid in KNOWN_VIDS["oh_board"]
    ]
    for candidate in oh_candidates:
        if _probe_orca_id(candidate.device) == ORCA_ID_RESP_SENSOR:
            return candidate.device
    return None


def discover_sensing_ports() -> SensingPorts:
    """Resolve tactile and encoder ports from connected hardware.

    Paxini wins the tactile field when present. The OH sensor CDC fills
    whichever field(s) Paxini didn't take — both, if Paxini is absent
    (``shared=True``).
    """
    paxini_port = find_tactile_port()
    oh_sensor_port = _find_oh_sensor_port()

    if paxini_port is not None:
        return SensingPorts(tactile=paxini_port, encoder=oh_sensor_port)
    if oh_sensor_port is not None:
        return SensingPorts(tactile=oh_sensor_port, encoder=oh_sensor_port)
    return SensingPorts(tactile=None, encoder=None)


def resolve_sensing_ports(
    tactile_override: str = "auto",
    encoder_override: str = "auto",
) -> SensingPorts:
    """Apply per-field overrides on top of discovery.

    Each override: ``"auto"`` uses the discovered value, ``"disabled"`` forces
    None, any other string is an explicit device path. Discovery is skipped
    entirely if neither field is ``"auto"``.
    """
    needs_discovery = tactile_override == "auto" or encoder_override == "auto"
    discovered = (
        discover_sensing_ports() if needs_discovery
        else SensingPorts(tactile=None, encoder=None)
    )

    def _resolve_field(override: str, discovered_value: Optional[str]) -> Optional[str]:
        if override == "auto":
            return discovered_value
        if override == "disabled":
            return None
        return override

    return SensingPorts(
        tactile=_resolve_field(tactile_override, discovered.tactile),
        encoder=_resolve_field(encoder_override, discovered.encoder),
    )
