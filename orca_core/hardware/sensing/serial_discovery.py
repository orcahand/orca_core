"""Locate the tactile and joint-encoder serial ports.

A dedicated tactile adapter is identified by its USB vendor ID. A combined
port that carries both streams is identified with an ``ORCA_ID?`` probe; when
it is the only port present, both fields point at it (``shared=True``).
"""

import logging
import time
from dataclasses import dataclass
from typing import Optional

from ...constants import (
    KNOWN_VIDS,
    ORCA_ID_PROBE_BAUDRATE,
    ORCA_ID_PROBE_TIMEOUT_S,
    ORCA_ID_QUERY,
    ORCA_ID_RESP_MOTOR,
    ORCA_ID_RESP_SENSOR,
)
from .constants import DEFAULT_ENCODER_BAUDRATE, DEFAULT_SENSOR_BAUDRATE

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SensingPorts:
    tactile: Optional[str]
    encoder: Optional[str]
    tactile_baudrate: Optional[int] = None
    """Baud for the tactile link, or ``None`` when no tactile port is known."""

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
    """Send ORCA_ID? and return ORCA:MOTOR/ORCA:SENSOR if the bridge replies.

    Robust against an active AA A9 (or AA 56) auto-stream interleaving with
    the bridge's response: instead of stopping at the first ``\\n`` byte
    (which can land inside an auto-stream frame's LRC or payload), the read
    accumulates bytes until either the expected response substring appears
    or the timeout expires.
    """
    import serial

    try:
        with serial.Serial(port, baudrate=baudrate, timeout=0.05) as link:
            link.reset_input_buffer()
            link.write(ORCA_ID_QUERY)
            link.flush()
            deadline = time.monotonic() + timeout
            buf = bytearray()
            while time.monotonic() < deadline:
                chunk = link.read(256)
                if chunk:
                    buf.extend(chunk)
                    if ORCA_ID_RESP_MOTOR in buf:
                        return ORCA_ID_RESP_MOTOR
                    if ORCA_ID_RESP_SENSOR in buf:
                        return ORCA_ID_RESP_SENSOR
            return None
    except (OSError, serial.SerialException) as exc:
        logger.debug("ORCA_ID? probe on %s failed: %s", port, exc)
        return None


def find_tactile_port() -> Optional[str]:
    """Return the dedicated tactile adapter's device path, or None if zero or >1 match."""
    import serial.tools.list_ports

    matches = [
        p for p in serial.tools.list_ports.comports()
        if p.vid in KNOWN_VIDS["tactile_sensor"]
    ]
    if len(matches) == 1:
        return matches[0].device
    return None


TACTILE_BAUD_CANDIDATES = (DEFAULT_ENCODER_BAUDRATE, DEFAULT_SENSOR_BAUDRATE)
"""Bauds to try when detecting a tactile link, fastest first."""


def baud_for_port(port: str) -> int:
    """Detect the baud of the tactile sensor on ``port``.

    Opens the port at each candidate baud and reads a register; the first baud
    that gets a valid reply wins. Falls back to ``DEFAULT_SENSOR_BAUDRATE`` when
    no candidate answers, so a sensor that isn't currently reporting still
    connects.
    """
    for baud in TACTILE_BAUD_CANDIDATES:
        if _tactile_responds_at(port, baud):
            logger.debug("tactile baud for %s resolved to %d", port, baud)
            return baud
    logger.debug("no tactile reply on %s at any baud; defaulting", port)
    return DEFAULT_SENSOR_BAUDRATE


def _tactile_responds_at(port: str, baud: int) -> bool:
    """True if a tactile register read succeeds on ``port`` at ``baud``."""
    # Imported here to avoid a circular import at module load.
    from ..hand_serial_link import HandSerialLink
    from ..tactile_client import TactileClient
    from .constants import ADDR_CONNECTED_SENSORS_LENGTH, ADDR_CONNECTED_SENSORS_START

    link = HandSerialLink(port=port, baudrate=baud)
    try:
        link.connect()
    except Exception:
        return False
    try:
        client = TactileClient(link)
        client._connected = True
        client._read_register(ADDR_CONNECTED_SENSORS_START, ADDR_CONNECTED_SENSORS_LENGTH)
        return True
    except Exception:
        return False
    finally:
        link.disconnect()


ORCA_ID_PROBE_ATTEMPTS = 3
"""Passes over the OH-board CDCs when probing ORCA_ID?. The probe is racy on
macOS composite CDC devices (an occasional empty read), so a few passes make
detection reliable without masking a genuinely absent/silent board."""


def _find_oh_board_port(expected_resp: bytes) -> Optional[str]:
    """Return the OH-board CDC whose ``ORCA_ID?`` reply matches ``expected_resp``."""
    import serial.tools.list_ports

    oh_candidates = [
        p for p in serial.tools.list_ports.comports()
        if p.vid in KNOWN_VIDS["oh_board"]
    ]
    for _ in range(ORCA_ID_PROBE_ATTEMPTS):
        for candidate in oh_candidates:
            if _probe_orca_id(candidate.device) == expected_resp:
                return candidate.device
    return None


def find_motor_port() -> Optional[str]:
    """Return the OH-board CDC that identifies as the motor bus via ``ORCA_ID?``.

    Returns None for classic motor adapters that don't speak
    ORCA_ID?, so the caller can fall back to USB-VID matching.
    """
    return _find_oh_board_port(ORCA_ID_RESP_MOTOR)


def _find_oh_sensor_port() -> Optional[str]:
    return _find_oh_board_port(ORCA_ID_RESP_SENSOR)


def discover_sensing_ports() -> SensingPorts:
    """Resolve tactile and encoder ports from connected hardware.

    A dedicated tactile adapter claims the tactile field when present. A
    combined port fills whichever field(s) the adapter did not take — both, if
    no dedicated adapter is present (``shared=True``).
    """
    paxini_port = find_tactile_port()
    oh_sensor_port = _find_oh_sensor_port()

    if paxini_port is not None:
        return SensingPorts(
            tactile=paxini_port,
            encoder=oh_sensor_port,
            tactile_baudrate=DEFAULT_SENSOR_BAUDRATE,
        )
    if oh_sensor_port is not None:
        return SensingPorts(
            tactile=oh_sensor_port,
            encoder=oh_sensor_port,
            tactile_baudrate=DEFAULT_ENCODER_BAUDRATE,
        )
    return SensingPorts(tactile=None, encoder=None)


def resolve_sensing_ports(
    tactile_override: str = "auto",
    encoder_override: str = "auto",
    tactile_baud_override: "int | str" = "auto",
) -> SensingPorts:
    """Apply per-field overrides on top of discovery.

    Each port override: ``"auto"`` uses the discovered value, ``"disabled"``
    forces None, any other string is an explicit device path. Discovery is
    skipped entirely if neither port field is ``"auto"``.

    ``tactile_baud_override``: ``"auto"`` keeps the detected baud; any int forces
    that baud. When the tactile port is given explicitly (so discovery was
    skipped), an ``"auto"`` baud stays ``None`` for the caller to resolve with
    :func:`baud_for_port`.
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

    tactile_baud: Optional[int]
    if tactile_baud_override == "auto":
        tactile_baud = discovered.tactile_baudrate
    else:
        tactile_baud = int(tactile_baud_override)

    return SensingPorts(
        tactile=_resolve_field(tactile_override, discovered.tactile),
        encoder=_resolve_field(encoder_override, discovered.encoder),
        tactile_baudrate=tactile_baud,
    )
