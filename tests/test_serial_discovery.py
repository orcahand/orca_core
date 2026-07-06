from __future__ import annotations

import logging
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import serial as pyserial

from orca_core.constants import ORCA_ID_RESP_MOTOR, ORCA_ID_RESP_SENSOR
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_PAYLOAD_BYTES,
    DEFAULT_ENCODER_BAUDRATE,
    DEFAULT_SENSOR_BAUDRATE,
    FTDI_VID,
    PROTOCOL_HEADER_AUTO_ENC,
)
from orca_core.hardware.sensing.framing import calculate_checksum
from orca_core.hardware.sensing.serial_discovery import (
    SensingPorts,
    _probe_orca_id,
    _tactile_responds_at,
    baud_for_port,
    detect_encoder_stream,
    discover_sensing_ports,
    find_motor_port,
    find_tactile_port,
    resolve_sensing_ports,
)
from orca_core.utils.utils import auto_detect_port


PAXINI_VID, OH_VID = 0x28E9, 0x2F5D
PAXINI_PORT = "/dev/cu.paxini"
OH_MOTOR_PORT = "/dev/cu.oh_motor"
OH_SENSOR_PORT = "/dev/cu.oh_sensor"
FTDI_PORT = "/dev/cu.ft232"
DISCOVERY = "orca_core.hardware.sensing.serial_discovery"

paxini = SimpleNamespace(device=PAXINI_PORT, vid=PAXINI_VID)
motor = SimpleNamespace(device=OH_MOTOR_PORT, vid=OH_VID)
sensor = SimpleNamespace(device=OH_SENSOR_PORT, vid=OH_VID)
ftdi = SimpleNamespace(device=FTDI_PORT, vid=FTDI_VID)
unknown = SimpleNamespace(device="/dev/cu.mystery", vid=0x1234)
OH_RESPONSES = {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR, OH_SENSOR_PORT: ORCA_ID_RESP_SENSOR}
DISCOVERED = SensingPorts("/auto/t", "/auto/e")


@pytest.mark.parametrize("tactile,encoder,expected", [
    (PAXINI_PORT, PAXINI_PORT,    True),
    (PAXINI_PORT, OH_SENSOR_PORT, False),
    (PAXINI_PORT, None,           False),
    (None,        PAXINI_PORT,    False),
    (None,        None,           False),
])
def test_sensing_ports_shared(tactile, encoder, expected):
    assert SensingPorts(tactile, encoder).shared is expected


@pytest.mark.parametrize("ports,expected", [
    pytest.param([paxini],                                                       PAXINI_PORT, id="single"),
    pytest.param([motor],                                                        None,        id="absent"),
    pytest.param([paxini, SimpleNamespace(device="/dev/cu.p2", vid=PAXINI_VID)], None,        id="ambiguous"),
    pytest.param([],                                                             None,        id="empty"),
])
def test_find_tactile_port(ports, expected):
    with patch("serial.tools.list_ports.comports", return_value=ports):
        assert find_tactile_port() == expected


@pytest.mark.parametrize("ports,probes,expected", [
    pytest.param([],                      {},                                  SensingPorts(None, None),                                            id="nothing"),
    pytest.param([paxini],                {},                                  SensingPorts(PAXINI_PORT, None, DEFAULT_SENSOR_BAUDRATE),             id="paxini_only"),
    pytest.param([motor, sensor],         OH_RESPONSES,                        SensingPorts(OH_SENSOR_PORT, OH_SENSOR_PORT, DEFAULT_ENCODER_BAUDRATE), id="oh_only_shared"),
    pytest.param([sensor, motor],         OH_RESPONSES,                        SensingPorts(OH_SENSOR_PORT, OH_SENSOR_PORT, DEFAULT_ENCODER_BAUDRATE), id="oh_reversed_order"),
    pytest.param([motor, sensor, paxini], OH_RESPONSES,                        SensingPorts(PAXINI_PORT, OH_SENSOR_PORT, DEFAULT_SENSOR_BAUDRATE),   id="paxini_plus_oh"),
    pytest.param([motor],                 {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR}, SensingPorts(None, None),                                            id="motor_only"),
    pytest.param([motor, sensor],         {},                                  SensingPorts(None, None),                                            id="oh_silent"),
    pytest.param([motor, paxini],         {OH_MOTOR_PORT: ORCA_ID_RESP_MOTOR}, SensingPorts(PAXINI_PORT, None, DEFAULT_SENSOR_BAUDRATE),             id="paxini_with_motor"),
])
def test_discover_sensing_ports(ports, probes, expected):
    with patch("serial.tools.list_ports.comports", return_value=ports), \
            patch(f"{DISCOVERY}._probe_orca_id", side_effect=lambda p, *_a, **_kw: probes.get(p)):
        assert discover_sensing_ports() == expected


@pytest.mark.parametrize("ports,probes,expected", [
    pytest.param([motor, sensor], OH_RESPONSES,                          OH_MOTOR_PORT, id="motor_and_sensor"),
    pytest.param([sensor, motor], OH_RESPONSES,                          OH_MOTOR_PORT, id="reversed_order"),
    pytest.param([sensor],        {OH_SENSOR_PORT: ORCA_ID_RESP_SENSOR}, None,          id="sensor_only"),
    pytest.param([motor, sensor], {},                                    None,          id="oh_silent"),
    pytest.param([],              {},                                    None,          id="no_oh_board"),
])
def test_find_motor_port(ports, probes, expected):
    with patch("serial.tools.list_ports.comports", return_value=ports), \
            patch(f"{DISCOVERY}._probe_orca_id", side_effect=lambda p, *_a, **_kw: probes.get(p)):
        assert find_motor_port() == expected


def test_auto_detect_port_prefers_orca_motor_bus():
    """ORCA_ID? wins over VID matching when an OH board names the motor bus."""
    with patch(f"{DISCOVERY}.find_motor_port", return_value=OH_MOTOR_PORT):
        assert auto_detect_port("dynamixel") == OH_MOTOR_PORT


def test_auto_detect_port_falls_back_to_vid_match():
    """Classic U2D2 (no ORCA_ID?) still resolves via its FTDI VID."""
    u2d2 = SimpleNamespace(device="/dev/cu.u2d2", vid=0x0403, description="U2D2")
    with patch(f"{DISCOVERY}.find_motor_port", return_value=None), \
            patch("serial.tools.list_ports.comports", return_value=[u2d2]):
        assert auto_detect_port("dynamixel") == "/dev/cu.u2d2"


def test_discover_does_not_probe_paxini():
    """ORCA_ID? on a Paxini board would hit the wrong protocol."""
    with patch("serial.tools.list_ports.comports", return_value=[paxini]), \
            patch(f"{DISCOVERY}._probe_orca_id") as probe:
        discover_sensing_ports()
    probe.assert_not_called()


@pytest.mark.parametrize("tactile,encoder,expected,expect_discovery", [
    pytest.param("auto",     "auto",      DISCOVERED,                            True,  id="auto"),
    pytest.param("auto",     "/dev/y",    SensingPorts("/auto/t", "/dev/y"),     True,  id="auto+explicit"),
    pytest.param("auto",     "disabled",  SensingPorts("/auto/t", None),         True,  id="auto+disabled"),
    pytest.param("/dev/x",   "/dev/y",    SensingPorts("/dev/x", "/dev/y"),      False, id="explicit"),
    pytest.param("/dev/s",   "/dev/s",    SensingPorts("/dev/s", "/dev/s"),      False, id="same_explicit"),
    pytest.param("disabled", "/dev/y",    SensingPorts(None, "/dev/y"),          False, id="disabled+explicit"),
    pytest.param("disabled", "disabled",  SensingPorts(None, None),              False, id="all_disabled"),
])
def test_resolve_sensing_ports(tactile, encoder, expected, expect_discovery):
    with patch(f"{DISCOVERY}.discover_sensing_ports", return_value=DISCOVERED) as discover:
        assert resolve_sensing_ports(tactile, encoder) == expected
    assert discover.called is expect_discovery


def test_resolve_passes_through_discovered_baud():
    discovered = SensingPorts("/t", "/e", DEFAULT_ENCODER_BAUDRATE)
    with patch(f"{DISCOVERY}.discover_sensing_ports", return_value=discovered):
        ports = resolve_sensing_ports("auto", "auto")
    assert ports.tactile_baudrate == DEFAULT_ENCODER_BAUDRATE


def test_resolve_baud_override_wins_over_discovery():
    discovered = SensingPorts("/t", "/e", DEFAULT_ENCODER_BAUDRATE)
    with patch(f"{DISCOVERY}.discover_sensing_ports", return_value=discovered):
        ports = resolve_sensing_ports("auto", "auto", tactile_baud_override=921600)
    assert ports.tactile_baudrate == 921600


def test_resolve_baud_unknown_for_explicit_port():
    """Explicit ports skip discovery; an auto baud stays None for the caller
    to resolve via ``baud_for_port``."""
    ports = resolve_sensing_ports("/dev/x", "/dev/y")
    assert ports.tactile_baudrate is None


@pytest.mark.parametrize("responds,expected", [
    pytest.param({DEFAULT_ENCODER_BAUDRATE: True},                            DEFAULT_ENCODER_BAUDRATE, id="2M_wins_first"),
    pytest.param({DEFAULT_ENCODER_BAUDRATE: False, DEFAULT_SENSOR_BAUDRATE: True}, DEFAULT_SENSOR_BAUDRATE, id="falls_back_to_921600"),
    pytest.param({},                                                          DEFAULT_SENSOR_BAUDRATE, id="nothing_answers_defaults"),
])
def test_baud_for_port(responds, expected):
    with patch(f"{DISCOVERY}._tactile_responds_at",
               side_effect=lambda port, baud: responds.get(baud, False)):
        assert baud_for_port("/dev/x") == expected


def test_baud_for_port_warns_on_unverified_fallback(caplog):
    with patch(f"{DISCOVERY}._tactile_responds_at", return_value=False), \
            caplog.at_level(logging.WARNING, logger=DISCOVERY):
        assert baud_for_port("/dev/x") == DEFAULT_SENSOR_BAUDRATE
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1
    message = warnings[0].getMessage()
    assert "/dev/x" in message
    assert str(DEFAULT_SENSOR_BAUDRATE) in message
    assert "config.yaml" in message


def test_baud_for_port_silent_when_verified(caplog):
    with patch(f"{DISCOVERY}._tactile_responds_at", return_value=True), \
            caplog.at_level(logging.WARNING, logger=DISCOVERY):
        assert baud_for_port("/dev/x") == DEFAULT_ENCODER_BAUDRATE
    assert not [r for r in caplog.records if r.levelno >= logging.WARNING]


# ----- Exclusive probing (busy port = not a candidate) -----------------------


def test_probe_orca_id_opens_exclusive_and_skips_busy_port():
    """A port another client already holds must never be probed."""
    with patch("serial.Serial",
               side_effect=pyserial.SerialException("Could not exclusively lock port")) as ser:
        assert _probe_orca_id(OH_MOTOR_PORT) is None
    assert ser.call_args.kwargs["exclusive"] is True


def test_tactile_probe_opens_exclusive_and_skips_busy_port():
    with patch("serial.Serial",
               side_effect=pyserial.SerialException("Could not exclusively lock port")) as ser:
        assert _tactile_responds_at("/dev/x", DEFAULT_SENSOR_BAUDRATE) is False
    assert ser.call_args.kwargs["exclusive"] is True


# ----- Encoder-stream detection (AA A9) --------------------------------------


class FakeSerial:
    """In-memory pyserial stand-in feeding a fixed byte stream in small chunks."""

    def __init__(self, data: bytes, max_chunk: int = 7):
        self._data = bytes(data)
        self._pos = 0
        self._max_chunk = max_chunk

    def read(self, n: int = 1) -> bytes:
        n = min(n, self._max_chunk)
        chunk = self._data[self._pos:self._pos + n]
        self._pos += len(chunk)
        return chunk

    def reset_input_buffer(self) -> None:
        pass

    def __enter__(self) -> "FakeSerial":
        return self

    def __exit__(self, *exc) -> bool:
        return False


def _encoder_frame(payload: bytes = bytes(AUTO_ENC_PAYLOAD_BYTES), error_byte: int = 0) -> bytes:
    """Build a wire-exact AA A9 frame: header + reserved + eff_len(LE) + err + payload + LRC."""
    body = (
        PROTOCOL_HEADER_AUTO_ENC
        + bytes([0x00])
        + (1 + len(payload)).to_bytes(2, "little")
        + bytes([error_byte])
        + payload
    )
    return body + bytes([calculate_checksum(body)])


def test_detect_encoder_stream_finds_valid_frame():
    frame = _encoder_frame(payload=bytes(range(AUTO_ENC_PAYLOAD_BYTES)))
    # Garbage prefix includes a fake AA A9 header with an implausible length,
    # then a truncated frame, then real frames.
    stream = b"\x12\x34" + PROTOCOL_HEADER_AUTO_ENC + b"\x00garbage" + frame[:10] + frame + frame
    with patch("serial.Serial", return_value=FakeSerial(stream)) as ser:
        assert detect_encoder_stream(FTDI_PORT, timeout=0.2) is True
    assert ser.call_args.kwargs["exclusive"] is True
    assert ser.call_args.kwargs["baudrate"] == DEFAULT_ENCODER_BAUDRATE


def test_detect_encoder_stream_rejects_bad_lrc():
    corrupted = bytearray(_encoder_frame())
    corrupted[-1] ^= 0xFF
    stream = b"\x00" * 4 + bytes(corrupted) + bytes(corrupted)
    with patch("serial.Serial", return_value=FakeSerial(stream)):
        assert detect_encoder_stream(FTDI_PORT, timeout=0.05) is False


def test_detect_encoder_stream_rejects_pure_garbage():
    with patch("serial.Serial", return_value=FakeSerial(b"\xaa\x55" * 40 + b"noise")):
        assert detect_encoder_stream(FTDI_PORT, timeout=0.05) is False


def test_detect_encoder_stream_busy_port_is_false():
    with patch("serial.Serial",
               side_effect=pyserial.SerialException("Could not exclusively lock port")):
        assert detect_encoder_stream(FTDI_PORT, timeout=0.05) is False


# ----- FTDI encoder-stream fallback ------------------------------------------


def test_ftdi_fallback_used_when_nothing_else_found():
    """With no dedicated tactile or combined port found, an FTDI port with a
    live encoder stream fills only the encoder field."""
    with patch("serial.tools.list_ports.comports", return_value=[ftdi, unknown]), \
            patch(f"{DISCOVERY}.detect_encoder_stream",
                  side_effect=lambda port, *a, **kw: port == FTDI_PORT) as det:
        assert discover_sensing_ports() == SensingPorts(None, FTDI_PORT)
    # Only FTDI-VID ports are probed; unknown VIDs are never touched.
    assert [c.args[0] for c in det.call_args_list] == [FTDI_PORT]


def test_ftdi_fallback_without_stream_finds_nothing():
    with patch("serial.tools.list_ports.comports", return_value=[ftdi]), \
            patch(f"{DISCOVERY}.detect_encoder_stream", return_value=False):
        assert discover_sensing_ports() == SensingPorts(None, None)


def test_ftdi_fallback_not_tried_when_paxini_present():
    with patch("serial.tools.list_ports.comports", return_value=[paxini, ftdi]), \
            patch(f"{DISCOVERY}.detect_encoder_stream") as det:
        ports = discover_sensing_ports()
    assert ports.tactile == PAXINI_PORT
    det.assert_not_called()


def test_ftdi_fallback_not_tried_when_oh_sensor_present():
    with patch("serial.tools.list_ports.comports", return_value=[sensor, ftdi]), \
            patch(f"{DISCOVERY}._probe_orca_id", side_effect=lambda p, *a, **kw: OH_RESPONSES.get(p)), \
            patch(f"{DISCOVERY}.detect_encoder_stream") as det:
        ports = discover_sensing_ports()
    assert ports.encoder == OH_SENSOR_PORT
    det.assert_not_called()


# ----- Zero-probing invariant for fully explicit configuration ---------------


def test_resolve_explicit_ports_and_baud_probe_nothing():
    """Explicit ports + explicit baud must neither open a serial port nor
    enumerate the system's ports."""
    with patch("serial.Serial") as ser, \
            patch("serial.tools.list_ports.comports") as comports:
        ports = resolve_sensing_ports("/dev/x", "/dev/y", tactile_baud_override=921600)
    assert ports == SensingPorts("/dev/x", "/dev/y", 921600)
    ser.assert_not_called()
    comports.assert_not_called()
