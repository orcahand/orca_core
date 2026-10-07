"""Locate the tactile and joint-encoder serial ports.

A dedicated tactile adapter is identified by its USB vendor ID. A combined
port that carries both streams is identified with an ``ORCA_ID?`` probe; when
it is the only port present, both fields point at it (``shared=True``). A
final fallback scans FTDI-VID ports for the encoder auto-stream directly.

All probes open ports with ``exclusive=True`` and treat a busy port as "not a
candidate", never stealing bytes from a link another client already holds.
"""

import errno
import logging
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

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
    former. ``config`` is the provisioned sensing-config code, the hand's own
    declaration of which sensors it was built with. Boards that answer only
    the legacy ``ORCA_ID?`` yield a role with every identity field ``None``;
    hands that report no side are treated as right-handed by the callers that
    need one.
    """

    role: str  # "motor" | "sensor"
    side: Optional[str] = None  # "left" | "right"
    hw_version: Optional[int] = None
    fw_version: Optional[int] = None
    serial: Optional[str] = None
    board_id: Optional[str] = None
    config: Optional[int] = None  # sensing config code; None when unprovisioned

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
        # Firmware omits CFG entirely when unset, and reports 0 for no sensing.
        config=_int_or_none("CFG") or None,
    )


OH_BOARD_MOTOR_BAUD_RATES: "tuple[int, ...]" = (
    57600, 1_000_000, 2_000_000, 3_000_000, 4_000_000, 4_500_000)
"""Motor-bus rates an OH board will follow from the host's CDC line coding.

The board bridges USB to the motor bus and retunes the wire to match the rate
the host opened the port at -- but only for these. Anything else it ignores and
leaves the wire where it was, so a motor moved to an unlisted rate becomes
unreachable through the board: the host cannot follow it there. 9600 and 115200
are left out on purpose, because probing tools open ports at them and following
that would retune a live bus.

Mirrors ``motorBaudAllowed`` in the board firmware. A plain USB-TTL adapter has
no such filter, which is why this is a property of the transport and not of the
motor family.
"""


def _query_link(
    link,
    timeout: float,
    query: bytes,
    is_complete: "Callable[[bytearray], bool]",
) -> Optional[bytes]:
    """Send ``query`` on an open ``link`` and return the reply once ``is_complete`` accepts it, else ``None`` at the timeout."""
    link.reset_input_buffer()
    link.write(query)
    link.flush()
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = link.read(256)
        if not chunk:
            continue
        buf.extend(chunk)
        if is_complete(buf):
            return bytes(buf)
    return None


def _query_until(
    port: str,
    baudrate: int,
    timeout: float,
    query: bytes,
    is_complete: "Callable[[bytearray], bool]",
) -> Optional[bytes]:
    """Open ``port`` exclusively and query it with :func:`_query_link`."""
    import serial

    with serial.Serial(port, baudrate=baudrate, timeout=0.05, exclusive=True) as link:
        return _query_link(link, timeout, query, is_complete)


def motor_baud_rates_over_link(link, timeout: float = ORCA_ID_PROBE_TIMEOUT_S):
    """Rates the transport behind an already-open ``link`` can carry.

    Returns ``None`` when the transport imposes no limit of its own, which is
    both the plain-adapter answer and the answer when nothing identifies
    itself -- the motor family's own map is then the only bound.

    Asks in band, over the open link, because the question is worth asking
    while a session holds the port: an OH board answers ``ORCA_ID?`` from the
    bridge itself without putting it on the wire. On a plain adapter the query
    reaches the motors as unframed bytes, which they ignore, and the silence is
    the answer.
    """
    try:
        reply = _query_link(
            link, timeout, ORCA_ID_QUERY,
            lambda buf: ORCA_ID_RESP_MOTOR in buf or ORCA_ID_RESP_SENSOR in buf)
    except Exception as exc:
        logger.debug("in-band transport probe failed: %s", exc)
        return None
    # A sensor reply means the motor bus is not behind this port at all.
    if reply is not None and ORCA_ID_RESP_MOTOR in reply:
        return OH_BOARD_MOTOR_BAUD_RATES
    return None


def _info_line(buf: "bytes | bytearray") -> Optional[bytes]:
    """The first complete ``ORCA:<role>...`` line in ``buf``, or ``None``."""
    for marker in (ORCA_INFO_MARKER_MOTOR, ORCA_INFO_MARKER_SENSOR):
        start = buf.find(marker)
        if start < 0:
            continue
        end = buf.find(b"\n", start)
        if end < 0:
            return None  # line still incomplete; keep reading
        return bytes(buf[start:end])
    return None


def _info_line_ready(buf: bytearray) -> bool:
    return _info_line(buf) is not None


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
        reply = _query_until(port, baudrate, timeout, ORCA_INFO_QUERY, _info_line_ready)
    except (OSError, serial.SerialException) as exc:
        logger.debug("ORCA_INFO? probe on %s failed: %s", port, exc)
        return None
    if reply is not None:
        return parse_orca_info(_info_line(reply))

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
        reply = _query_until(
            port, baudrate, timeout, ORCA_ID_QUERY,
            lambda buf: ORCA_ID_RESP_MOTOR in buf or ORCA_ID_RESP_SENSOR in buf,
        )
    except (OSError, serial.SerialException) as exc:
        logger.debug("ORCA_ID? probe on %s failed: %s", port, exc)
        return None
    if reply is None:
        return None
    return ORCA_ID_RESP_MOTOR if ORCA_ID_RESP_MOTOR in reply else ORCA_ID_RESP_SENSOR


# EACCES is deliberately absent on POSIX: there it means missing device
# permissions, not a port another process holds. Windows has no equivalent
# EBUSY/EAGAIN signal for a busy COM port; see the platform branch below.
_PORT_BUSY_ERRNOS = frozenset({errno.EAGAIN, errno.EWOULDBLOCK, errno.EBUSY})


def port_in_use(port: str) -> bool:
    """True when ``port`` exists but another process already holds it.

    Every probe here opens exclusively, so a port held by a running client
    is silent in exactly the same way an absent board is. Callers use this
    to tell "in use" apart from "nothing there".
    """
    import serial

    try:
        with serial.Serial(port, timeout=0, exclusive=True):
            return False
    except OSError as exc:
        if exc.errno in _PORT_BUSY_ERRNOS:
            return True
        if sys.platform == "win32":
            # pyserial wraps the WinError without preserving errno; a busy
            # COM port raises PermissionError (winerror 5), an absent one
            # FileNotFoundError (winerror 2).
            return "PermissionError" in str(exc)
        return False


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

    The port is opened exclusively (see :func:`_query_until`); a busy port
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
"""Passes over the controller-board CDCs when probing ORCA_ID?, and the passes
``detect_hand`` makes with ORCA_INFO?. The probe is racy on macOS composite CDC
devices (an occasional empty read), so a few passes make detection reliable
without masking a genuinely absent or silent board."""


def _find_oh_board_port(expected_resp: bytes) -> Optional[str]:
    """Return the controller-board CDC whose ``ORCA_ID?`` reply matches
    ``expected_resp``."""
    for _ in range(ORCA_ID_PROBE_ATTEMPTS):
        for device in oh_board_ports():
            if _probe_orca_id(device) == expected_resp:
                return device
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
