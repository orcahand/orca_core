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
    PORT_BUSY_ERRNOS,
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

ORCA_ID_PROBE_ATTEMPTS = 3
"""Passes over the controller-board CDCs when probing ORCA_ID?. The probe is
racy on macOS composite CDC devices (an occasional empty read), so a few
passes make detection reliable without masking a genuinely absent or
silent board."""

_USB_UID_HEX_LEN = 32
"""Hex characters in the MCU's factory unique ID as the USB descriptor
carries it. A dual-CDC board repeats it once per interface."""

CDCS_PER_BOARD = 2
"""CDC endpoints one controller board presents: one motor, one sensing."""


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


def port_in_use(port: str) -> bool:
    """True when ``port`` exists but another process already holds it.

    Every probe here opens exclusively, so a port held by a running client
    is silent in exactly the same way an absent board is. Callers use this
    to tell a diagnostic "in use" apart from "nothing there".
    """
    import serial

    try:
        with serial.Serial(port, timeout=0, exclusive=True):
            return False
    except OSError as exc:
        return exc.errno in PORT_BUSY_ERRNOS


def board_id_from_usb_serial(usb_serial: "str | None") -> Optional[str]:
    """The board ID the firmware reports, derived from its USB descriptor.

    Both are folded from the same four MCU factory words, so a board can be
    identified without opening a port — and without the identity firmware
    that answers ``ORCA_INFO?``. The descriptor repeats the 32-hex unique ID
    once per CDC; anything not of that shape yields ``None``.
    """
    if not usb_serial:
        return None
    text = usb_serial.strip().upper()
    if len(text) == 2 * _USB_UID_HEX_LEN and text[:_USB_UID_HEX_LEN] == text[_USB_UID_HEX_LEN:]:
        text = text[:_USB_UID_HEX_LEN]
    if len(text) != _USB_UID_HEX_LEN:
        return None
    try:
        words = [int(text[i:i + 8], 16) for i in range(0, _USB_UID_HEX_LEN, 8)]
    except ValueError:
        return None
    return "%08X%08X" % (words[0] ^ words[1], words[2] ^ words[3])


@dataclass(frozen=True)
class BoardCandidate:
    """One controller board's CDCs, grouped from USB descriptors alone.

    ``ports`` are that board's device paths in enumeration order; which one
    is the motor bus and which the sensing link takes a probe
    (:func:`probe_board`).
    """

    ports: tuple[str, ...]
    usb_serial: Optional[str] = None
    location: Optional[str] = None

    @property
    def board_id(self) -> Optional[str]:
        """The board ID implied by the USB descriptor, without probing."""
        return board_id_from_usb_serial(self.usb_serial)


def oh_board_candidates() -> "list[BoardCandidate]":
    """Group every attached controller board's CDCs by physical board.

    Opens nothing, so this stays accurate while the boards are connected and
    driving — a port another client holds is still enumerated, unlike a probe.
    Boards are keyed by USB serial number, falling back to the descriptor's
    device-level location. CDCs offering neither are merged only while there
    are no more of them than one board presents; beyond that each stands
    alone, since guessing would pair a motor bus with another board's
    encoders.
    """
    import serial.tools.list_ports

    groups: "dict[tuple, list]" = {}
    unattributable = []
    for port in serial.tools.list_ports.comports():
        if port.vid not in KNOWN_VIDS["oh_board"]:
            continue
        if port.serial_number:
            key = ("sn", port.serial_number)
        elif port.location:
            # Linux reports a per-interface location (``1-3:1.0``); macOS
            # reports the device (``2-1``), where the split is a no-op.
            key = ("loc", port.location.split(":")[0])
        else:
            unattributable.append(port)
            continue
        groups.setdefault(key, []).append(port)

    candidates = [
        BoardCandidate(
            ports=tuple(p.device for p in ports),
            usb_serial=ports[0].serial_number if kind == "sn" else None,
            location=ports[0].location,
        )
        for (kind, _), ports in groups.items()
    ]

    # A CDC naming neither a serial nor a location cannot be attributed. Up to
    # one board's worth of them is still unambiguous — there is nothing to
    # cross-pair with; beyond that, each stands alone rather than risk pairing
    # one hand's motor bus with another's encoders.
    if len(unattributable) > CDCS_PER_BOARD:
        candidates.extend(BoardCandidate(ports=(p.device,)) for p in unattributable)
    elif unattributable:
        candidates.append(
            BoardCandidate(ports=tuple(p.device for p in unattributable))
        )
    return candidates


@dataclass(frozen=True)
class BoardProbe:
    """What one controller board answered when asked who it is."""

    ports: tuple[str, ...]
    usb_serial: Optional[str] = None
    board_id: Optional[str] = None
    motor_port: Optional[str] = None
    sensing_port: Optional[str] = None
    identity: Optional[OrcaBoardInfo] = None
    busy_ports: tuple[str, ...] = ()

    @property
    def hand_id(self) -> Optional[str]:
        """This hand's stable name: its assigned serial when provisioned,
        else the board ID."""
        if self.identity is not None and self.identity.hand_id:
            return self.identity.hand_id
        return self.board_id


def probe_board(
    candidate: BoardCandidate, attempts: int = ORCA_ID_PROBE_ATTEMPTS
) -> BoardProbe:
    """Resolve one board's motor and sensing CDCs, touching only its own ports.

    Both CDCs of a board answer with the same identity block, so board IDs
    that disagree across a candidate prove it holds more than one board and
    its ports are left unpaired. The descriptor's ID wins over a reply,
    because it reads the same whether or not the board answers — a hand
    already driving holds its ports and cannot be probed at all.
    """
    usb_board_id = candidate.board_id
    motor_port = sensing_port = None
    identity = None
    reported: "dict[str, str]" = {}

    for _ in range(attempts):
        for port in candidate.ports:
            if port in (motor_port, sensing_port):
                continue
            info = probe_orca_info(port)
            if info is None:
                continue
            if info.board_id is not None:
                reported[port] = info.board_id
            if info.role == "motor" and motor_port is None:
                motor_port = port
            elif info.role == "sensor" and sensing_port is None:
                sensing_port = port
            if identity is None or (identity.side is None and info.side):
                identity = info
        if motor_port is not None and sensing_port is not None:
            break

    distinct = set(reported.values())
    if usb_board_id is not None:
        # One USB serial is one device, so a disagreement here is the fold
        # and the firmware differing, not a grouping that crossed boards.
        for mismatch in sorted(distinct - {usb_board_id}):
            logger.warning(
                "a CDC of board %s reports board ID %s; trusting the USB "
                "descriptor, which reads the same whether or not the board "
                "answers.", usb_board_id, mismatch,
            )
    elif len(distinct) > 1:
        logger.error(
            "CDCs grouped as one board report different board IDs (%s), so "
            "they are separate boards. Leaving them unpaired rather than "
            "driving one hand's motors from another hand's encoders.",
            ", ".join(f"{p}={b}" for p, b in sorted(reported.items())),
        )
        motor_port = sensing_port = identity = None

    return BoardProbe(
        ports=candidate.ports,
        usb_serial=candidate.usb_serial,
        board_id=usb_board_id or (identity.board_id if identity is not None else None),
        motor_port=motor_port,
        sensing_port=sensing_port,
        identity=identity,
        busy_ports=tuple(
            port for port in candidate.ports
            if port not in (motor_port, sensing_port) and port_in_use(port)
        ),
    )


def probe_boards() -> "list[BoardProbe]":
    """Probe every attached controller board, one board at a time."""
    return [probe_board(candidate) for candidate in oh_board_candidates()]


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


def _find_oh_board_port(expected_resp: bytes) -> Optional[str]:
    """Return the controller-board CDC whose ``ORCA_ID?`` reply matches
    ``expected_resp``, or None unless exactly one board answers.

    Boards are counted by what replies, not by what enumerates: the vendor ID
    is shared with the bare controller module, so a spare board on the bench
    must not stop a lone hand resolving its ports. When two hands do answer
    there is no single right answer, and returning either hands the caller a
    port belonging to a hand it did not ask for — those callers use
    :func:`probe_boards`.
    """
    matches = []
    for candidate in oh_board_candidates():
        for _ in range(ORCA_ID_PROBE_ATTEMPTS):
            found = next(
                (p for p in candidate.ports if _probe_orca_id(p) == expected_resp),
                None,
            )
            if found is not None:
                matches.append(found)
                break

    if len(matches) == 1:
        return matches[0]
    if matches:
        logger.warning(
            "%d controller boards answered, so no single motor or sensing "
            "port can be resolved. Select a hand explicitly.", len(matches),
        )
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
