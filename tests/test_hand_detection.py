"""Hand autodetection: identity parsing, the detection ladder, and load_hand()."""

import logging

import pytest

import orca_core.hand_factory as hand_factory
from orca_core import OrcaHand, OrcaHandTouch, detect_hand, load_hand
from orca_core.constants import KNOWN_VIDS
from orca_core.hardware.sensing import serial_discovery
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo, parse_orca_info

from tests._helpers import fake_serial_port


# ----- identity-line parsing ------------------------------------------------

def test_parse_full_identity_line():
    info = parse_orca_info(
        b"ORCA:SENSOR;SIDE=L;HW=2;FW=1;SN=OH2-L-2628-0047;BID=0123456789ABCDEF"
    )
    assert info == OrcaBoardInfo(
        role="sensor", side="left", hw_version=2, fw_version=1,
        serial="OH2-L-2628-0047", board_id="0123456789ABCDEF",
    )
    assert info.hand_id == "OH2-L-2628-0047"


def test_parse_unprovisioned_line_has_no_side():
    info = parse_orca_info(b"ORCA:MOTOR;FW=1;BID=AABBCCDD00112233")
    assert info.role == "motor"
    assert info.side is None
    assert info.hw_version is None
    assert info.serial is None
    assert info.hand_id == "AABBCCDD00112233"


def test_parse_ignores_unknown_fields_and_junk_values():
    info = parse_orca_info(b"ORCA:MOTOR;SIDE=R;HW=x;NEW=1;FW=3")
    assert info.side == "right"
    assert info.hw_version is None
    assert info.fw_version == 3


@pytest.mark.parametrize("line", [b"", b"garbage", b"ORCA:OTHER;SIDE=L", b"\xaa\xa9\x01"])
def test_parse_rejects_non_identity_lines(line):
    assert parse_orca_info(line) is None


# ----- detection ladder -----------------------------------------------------

OH_VID = KNOWN_VIDS["oh_board"][0]

# One board's CDCs share its USB serial number; that is what pairs them.
BOARD_A = "1B46C9E850304C43552E3120FF061305" * 2
BOARD_B = "0011223344556677889AABBCCDDEEFF0" * 2


def _patch_hardware(
    monkeypatch,
    *,
    oh_ports=(),
    infos=None,
    encoder_stream=False,
    paxini_port=None,
    tactile_register=False,
    busy_ports=(),
):
    """Stand in for the USB bus at the descriptor level, so board grouping
    runs for real. An entry in ``oh_ports`` is a device path on the default
    board, or a ``(device, usb_serial)`` pair to place it on another one.
    """
    infos = infos or {}
    ports = []
    for entry in oh_ports:
        device, usb_serial = entry if isinstance(entry, tuple) else (entry, BOARD_A)
        ports.append(fake_serial_port(device, OH_VID, serial_number=usb_serial))

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: ports)
    monkeypatch.setattr(
        serial_discovery, "probe_orca_info",
        lambda port, *args, **kwargs: infos.get(port),
    )
    monkeypatch.setattr(
        serial_discovery, "port_in_use", lambda port: port in busy_ports
    )
    monkeypatch.setattr(
        hand_factory, "detect_encoder_stream", lambda port: encoder_stream
    )
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: paxini_port)
    monkeypatch.setattr(
        hand_factory, "_tactile_responds_at", lambda port, baud: tactile_register
    )


def test_detects_full_left_hand(monkeypatch):
    identity = OrcaBoardInfo(role="sensor", side="left", hw_version=2,
                             fw_version=1, serial="AB")
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={
            "/dev/cu.m": OrcaBoardInfo(role="motor", side="left"),
            "/dev/cu.s": identity,
        },
        encoder_stream=True,
        tactile_register=True,
    )
    d = detect_hand()
    assert d.model_name == "orcahand-full-left"
    assert (d.side, d.has_tactile, d.has_encoders) == ("left", True, True)
    assert d.motor_port == "/dev/cu.m"
    assert d.sensing_port == "/dev/cu.s"


def test_sideless_board_defaults_right(monkeypatch):
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={
            "/dev/cu.m": OrcaBoardInfo(role="motor"),
            "/dev/cu.s": OrcaBoardInfo(role="sensor"),
        },
        encoder_stream=True,
    )
    d = detect_hand()
    assert d.model_name == "orcahand-joint-right"
    assert d.side == "right"


def test_detects_legacy_touch_adapter(monkeypatch):
    _patch_hardware(monkeypatch, paxini_port="/dev/cu.paxini")
    d = detect_hand()
    assert d.model_name == "orcahand-touch-right"
    assert d.tactile_port == "/dev/cu.paxini"
    assert not d.has_encoders


def test_nothing_plugged_in_yields_plain_right_hand(monkeypatch):
    _patch_hardware(monkeypatch)
    d = detect_hand()
    assert d.model_name == "orcahand-right"
    assert d.motor_port is None
    assert d.sensing_port is None
    assert d.identity is None
    assert d.busy_ports == ()


# ----- ports held by another client ------------------------------------------

def test_port_held_elsewhere_is_reported_not_silently_absent(monkeypatch):
    """A CDC another process holds is silent, so it reads exactly like an
    absent board; it has to come back named rather than as a plain None."""
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={"/dev/cu.s": OrcaBoardInfo(role="sensor", side="left")},
        encoder_stream=True,
        busy_ports=("/dev/cu.m",),
    )
    d = detect_hand()
    assert d.motor_port is None
    assert d.busy_ports == ("/dev/cu.m",)


def test_ports_that_answered_are_never_reported_busy(monkeypatch):
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={
            "/dev/cu.m": OrcaBoardInfo(role="motor", side="left"),
            "/dev/cu.s": OrcaBoardInfo(role="sensor", side="left"),
        },
        busy_ports=("/dev/cu.m", "/dev/cu.s"),
    )
    assert detect_hand().busy_ports == ()


def test_load_hand_warns_that_a_busy_port_may_understate_the_hand(monkeypatch, caplog):
    """The silent failure mode: with the sensing CDC held, detection sees no
    sensors and load_hand() would hand back a plain OrcaHand without a word."""
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={"/dev/cu.m": OrcaBoardInfo(role="motor", side="right")},
        busy_ports=("/dev/cu.s",),
    )
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        hand = load_hand()
    assert type(hand) is OrcaHand
    assert "/dev/cu.s" in caplog.text


def test_load_hand_is_silent_when_every_port_answered(monkeypatch, caplog):
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m"],
        infos={"/dev/cu.m": OrcaBoardInfo(role="motor", side="right")},
    )
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        load_hand()
    assert not [r for r in caplog.records if r.levelno >= logging.WARNING]


# ----- load_hand integration -------------------------------------------------

def test_load_hand_autodetects_and_pins_ports(monkeypatch):
    detection = hand_factory.HandDetection(
        model_name="orcahand-touch-left",
        side="left",
        has_tactile=True,
        has_encoders=False,
        motor_port="/dev/cu.m",
        sensing_port="/dev/cu.s",
    )
    monkeypatch.setattr(hand_factory, "detect_hand", lambda: detection)
    hand = load_hand()
    assert type(hand) is OrcaHandTouch
    assert hand.config.type == "left"
    assert hand.config.port == "/dev/cu.m"
    assert hand.config.sensor_port == "/dev/cu.s"


def test_load_hand_detection_fallback_is_default_model(monkeypatch):
    _patch_hardware(monkeypatch)
    hand = load_hand()
    assert type(hand) is OrcaHand
    assert hand.config.type == "right"
    assert hand.config.port == "auto"


@pytest.mark.parametrize(
    "kwargs",
    [
        {"mock": True},
        {"model_name": "orcahand-left"},
        {"model_version": "v2"},
    ],
)
def test_load_hand_skips_detection_when_told_what_to_load(monkeypatch, kwargs):
    def _must_not_probe():
        raise AssertionError("detect_hand() ran despite explicit selection")

    monkeypatch.setattr(hand_factory, "detect_hand", _must_not_probe)
    load_hand(**kwargs)
