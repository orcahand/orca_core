"""Locate the tactile and joint-encoder serial ports.

A dedicated tactile adapter is identified by its USB vendor ID. A combined
port that carries both streams is identified with an ``ORCA_ID?`` probe; when
it is the only port present, both fields point at it (``shared=True``). A
final fallback scans FTDI-VID ports for the encoder auto-stream directly.

All probes open ports with ``exclusive=True`` and treat a busy port as "not a
candidate", never stealing bytes from a link another client already holds.
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
    ORCA_INFO_MARKER_MOTOR,
    ORCA_INFO_MARKER_SENSOR,
    ORCA_INFO_QUERY,
)
from .constants import (
    AUTO_FRAME_META_SIZE,
    DEFAULT_ENCODER_BAUDRATE,
    DEFAULT_SENSOR_BAUDRATE,
    FTDI_VID,
    MAX_AUTO_FRAME_EFFECTIVE_LENGTH,
    PROTOCOL_HEADER_AUTO_ENC,
)
from .framing import calculate_checksum

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class OrcaBoardInfo:
    """Identity a hand's controller board reports via ``ORCA_INFO?``.

    ``serial`` is the hand's assigned serial number and ``board_id`` the
    board's immutable MCU-derived identifier; :attr:`hand_id` prefers the
    former. Boards that answer only the legacy ``ORCA_ID?`` yield a role
    with every identity field ``None``; hands that report no side are
    treated as right-handed by the callers that need one.
    """

    role: str  # "motor" | "sensor"
    side: Optional[str] = None  # "left" | "right"
    hw_version: Optional[int] = None
    fw_version: Optional[int] = None
    serial: Optional[str] = None
    board_id: Optional[str] = None

    @property
    def hand_id(self) -> Optional[str]:
        """The hand's unique identifier: its assigned serial when provisioned,
        else the board ID (which changes if the board is ever replaced)."""
        return self.serial or self.board_id


def parse_orca_info(line: bytes) -> Optional[OrcaBoardInfo]:
    """Parse one ``ORCA:<role>;K=V;...`` identity line. Unknown keys are
    ignored; malformed values yield ``None`` fields rather than an error."""
    try:
        text = line.decode("ascii").strip()
    except UnicodeDecodeError:
        return None
    tokens = text.split(";")
    role = {"ORCA:MOTOR": "motor", "ORCA:SENSOR": "sensor"}.get(tokens[0])
    if role is None:
        return None
    fields = {}
    for token in tokens[1:]:
        key, sep, value = token.partition("=")
        if sep:
            fields[key] = value
    side = {"L": "left", "R": "right"}.get(fields.get("SIDE"))

    def _int_or_none(key: str) -> Optional[int]:
        try:
            return int(fields[key])
        except (KeyError, ValueError):
            return None

    return OrcaBoardInfo(
        role=role,
        side=side,
        hw_version=_int_or_none("HW"),
        fw_version=_int_or_none("FW"),
        serial=fields.get("SN") or None,
        board_id=fields.get("BID") or None,
    )


def probe_orca_info(
    port: str,
    baudrate: int = ORCA_ID_PROBE_BAUDRATE,
    timeout: float = ORCA_ID_PROBE_TIMEOUT_S,
) -> Optional[OrcaBoardInfo]:
    """Query ``port`` for its role and hand identity.

    Sends ``ORCA_INFO?`` and parses the identity line; a board that stays
    silent (pre-identity firmware) is retried with the legacy ``ORCA_ID?``,
    yielding a role-only result. Returns ``None`` when neither answers.
    Robust against an active auto-stream the same way as the ID probe: the
    read accumulates until a complete marker...newline span appears.
    """
    import serial

    try:
        with serial.Serial(port, baudrate=baudrate, timeout=0.05, exclusive=True) as link:
            link.reset_input_buffer()
            link.write(ORCA_INFO_QUERY)
            link.flush()
            deadline = time.monotonic() + timeout
            buf = bytearray()
            while time.monotonic() < deadline:
                chunk = link.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                for marker in (ORCA_INFO_MARKER_MOTOR, ORCA_INFO_MARKER_SENSOR):
                    start = buf.find(marker)
                    if start < 0:
                        continue
                    end = buf.find(b"\n", start)
                    if end < 0:
                        break  # line still incomplete; keep reading
                    return parse_orca_info(bytes(buf[start:end]))
    except (OSError, serial.SerialException) as exc:
        logger.debug("ORCA_INFO? probe on %s failed: %s", port, exc)
        return None

    resp = _probe_orca_id(port, baudrate=baudrate, timeout=timeout)
    if resp == ORCA_ID_RESP_MOTOR:
        return OrcaBoardInfo(role="motor")
    if resp == ORCA_ID_RESP_SENSOR:
        return OrcaBoardInfo(role="sensor")
    return None


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
        # exclusive=True: skip a port another client already holds instead of
        # writing onto its live bus.
        with serial.Serial(port, baudrate=baudrate, timeout=0.05, exclusive=True) as link:
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


def oh_board_ports() -> "list[str]":
    """Device paths of every CDC presented by a hand's controller board, in
    enumeration order."""
    import serial.tools.list_ports

    return [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid in KNOWN_VIDS["oh_board"]
    ]


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
    logger.warning(
        "Could not verify sensing baud on %s: no tactile register reply at %s baud; "
        "assuming %d. If this link runs at a different baud (e.g. a 2 Mbaud "
        "encoder-only or combined link), set sensors.baudrate / encoder_baudrate "
        "explicitly in config.yaml.",
        port,
        "/".join(str(b) for b in TACTILE_BAUD_CANDIDATES),
        DEFAULT_SENSOR_BAUDRATE,
    )
    return DEFAULT_SENSOR_BAUDRATE


def _tactile_responds_at(port: str, baud: int) -> bool:
    """True if a tactile register read succeeds on ``port`` at ``baud``.

    The port is opened exclusively (see :func:`_probe_orca_id`); a busy port
    is quietly treated as "does not respond".
    """
    # Imported here to avoid a circular import at module load.
    from ..hand_serial_link import HandSerialLink
    from ..tactile_client import TactileClient
    from .constants import ADDR_CONNECTED_SENSORS_LENGTH, ADDR_CONNECTED_SENSORS_START

    link = HandSerialLink(port=port, baudrate=baud, exclusive=True)
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
"""Passes over the controller-board CDCs when probing ORCA_ID?. The probe is
racy on macOS composite CDC devices (an occasional empty read), so a few
passes make detection reliable without masking a genuinely absent or
silent board."""


def _find_oh_board_port(expected_resp: bytes) -> Optional[str]:
    """Return the controller-board CDC whose ``ORCA_ID?`` reply matches
    ``expected_resp``."""
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
    """Return the controller-board CDC that identifies as the motor bus via
    ``ORCA_ID?``.

    Returns None for classic motor adapters that don't speak
    ORCA_ID?, so the caller can fall back to USB-VID matching.
    """
    return _find_oh_board_port(ORCA_ID_RESP_MOTOR)


def _find_oh_sensor_port() -> Optional[str]:
    return _find_oh_board_port(ORCA_ID_RESP_SENSOR)


def detect_encoder_stream(
    port: str,
    baudrate: int = DEFAULT_ENCODER_BAUDRATE,
    timeout: float = 0.3,
) -> bool:
    """True iff an LRC-valid AA A9 encoder frame arrives on ``port``.

    Passive (nothing is written) and opened exclusively; a busy or
    unopenable port returns False.
    """
    import serial

    try:
        with serial.Serial(port, baudrate=baudrate, timeout=0.05, exclusive=True) as link:
            link.reset_input_buffer()
            deadline = time.monotonic() + timeout
            buf = bytearray()
            while time.monotonic() < deadline:
                chunk = link.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                if _contains_valid_encoder_frame(buf):
                    return True
            return False
    except (OSError, serial.SerialException) as exc:
        logger.debug("encoder-stream probe on %s failed: %s", port, exc)
        return False


def _contains_valid_encoder_frame(data: bytes) -> bool:
    """Scan ``data`` for one complete, LRC-valid AA A9 frame (mirrors the ``HandSerialLink`` demuxer)."""
    start = 0
    while True:
        start = data.find(PROTOCOL_HEADER_AUTO_ENC, start)
        if start < 0:
            return False
        meta_end = start + len(PROTOCOL_HEADER_AUTO_ENC) + AUTO_FRAME_META_SIZE
        if len(data) < meta_end:
            # Header at the buffer tail; the caller retries once more bytes arrive.
            return False
        effective_length = int.from_bytes(data[meta_end - 2:meta_end], "little")
        frame_end = meta_end + effective_length + 1
        if (
            0 < effective_length <= MAX_AUTO_FRAME_EFFECTIVE_LENGTH
            and len(data) >= frame_end
        ):
            frame = data[start:frame_end]
            if calculate_checksum(frame[:-1]) == frame[-1]:
                return True
        start += 1  # resync one byte forward, like the demuxer


def _find_ftdi_encoder_port() -> Optional[str]:
    """Last-resort scan: probes only FTDI-VID ports, read-only, for a live encoder stream."""
    import serial.tools.list_ports

    for candidate in serial.tools.list_ports.comports():
        if candidate.vid == FTDI_VID and detect_encoder_stream(candidate.device):
            return candidate.device
    return None


def discover_sensing_ports() -> SensingPorts:
    """Resolve tactile and encoder ports from connected hardware.

    A dedicated tactile adapter claims the tactile field when present. A
    combined port fills whichever field(s) the adapter did not take — both, if
    no dedicated adapter is present (``shared=True``).

    When neither is found, FTDI-VID ports are scanned for a live encoder
    stream as a last resort; a hit fills only the encoder field.
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
    ftdi_encoder_port = _find_ftdi_encoder_port()
    if ftdi_encoder_port is not None:
        return SensingPorts(tactile=None, encoder=ftdi_encoder_port)
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

    With all fields explicit, this function probes nothing (no port is
    opened or enumerated).
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
