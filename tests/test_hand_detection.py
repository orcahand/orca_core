"""Hand autodetection: identity parsing, the detection ladder, and load_hand()."""

import dataclasses
import logging

import pytest

import orca_core.hand_factory as hand_factory
from orca_core import OrcaHand, OrcaHandFull, OrcaHandTouch, detect_hand, load_hand
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


def test_parse_reads_sensing_config():
    info = parse_orca_info(
        b"ORCA:SENSOR;SIDE=R;HW=2;FW=2;CFG=2500;SN=ser-0001;BID=5E170D08B3346BE3"
    )
    assert info.config == 2500


@pytest.mark.parametrize("line", [
    b"ORCA:MOTOR;SIDE=R;CFG=0",  # firmware reports 0 for "no sensing config set"
    b"ORCA:MOTOR;SIDE=R",        # field absent entirely (pre-CFG firmware)
    b"ORCA:MOTOR;SIDE=R;CFG=x",  # unparseable
])
def test_parse_treats_unset_config_as_none(line):
    assert parse_orca_info(line).config is None


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
BOARD_A = "5EED1CE500FA11EDBEEFCAFE0DDBA11D" * 2
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


# ----- the declared sensing config is ground truth ---------------------------

def test_sensing_config_caps_table_is_pinned():
    """This mapping is recorded nowhere else — the firmware only validates the
    set of codes, so a silent edit here would mis-load hands with no other
    source to check against."""
    assert hand_factory.SENSING_CONFIG_CAPS == {
        1000: (False, False),
        1500: (False, True),
        2000: (True, False),
        2500: (True, True),
    }


def test_every_declarable_config_names_a_bundled_model():
    """Guards the (tactile, encoders) tuple order: a swapped pair would still
    be a valid dict but would resolve to the wrong model."""
    for caps in hand_factory.SENSING_CONFIG_CAPS.values():
        assert caps in hand_factory._MODEL_BY_CAPS

def _declaring(config, **kwargs):
    """Patch args for a board declaring ``config`` on both its CDCs."""
    info = OrcaBoardInfo(role="sensor", side="right", serial="ser-0001",
                         config=config)
    return dict(
        oh_ports=["/dev/cu.m", "/dev/cu.s"],
        infos={
            "/dev/cu.m": dataclasses.replace(info, role="motor"),
            "/dev/cu.s": info,
        },
        **kwargs,
    )


@pytest.mark.parametrize("config,model,caps", [
    (1000, "orcahand-right", (False, False)),
    (1500, "orcahand-joint-right", (False, True)),
    (2000, "orcahand-touch-right", (True, False)),
    (2500, "orcahand-full-right", (True, True)),
])
def test_declared_config_picks_the_model(monkeypatch, config, model, caps):
    """Every CFG code maps to its model even with all sensors responding, so a
    wrong mapping can't hide behind agreeing probe results."""
    _patch_hardware(monkeypatch, **_declaring(
        config, encoder_stream=True, tactile_register=True,
    ))
    d = detect_hand()
    assert d.model_name == model
    assert (d.has_tactile, d.has_encoders) == caps
    assert d.declared_config == config


def test_declared_sensing_wins_over_silent_hardware(monkeypatch):
    """The reported failure: a full hand whose sensing link is down must stay a
    full hand rather than silently degrading to the motor-only model."""
    _patch_hardware(monkeypatch, **_declaring(2500))
    d = detect_hand()
    assert d.model_name == "orcahand-full-right"
    assert (d.has_tactile, d.has_encoders) == (True, True)
    assert (d.probed_tactile, d.probed_encoders) == (False, False)
    assert set(d.missing_capabilities) == {"tactile", "encoders"}
    assert d.undeclared_capabilities == ()


def test_partial_sensing_failure_names_only_the_dead_capability(monkeypatch):
    _patch_hardware(monkeypatch, **_declaring(2500, encoder_stream=True))
    d = detect_hand()
    assert d.missing_capabilities == ("tactile",)
    assert d.probed_encoders is True


def test_undeclared_sensors_are_flagged_and_stay_unused(monkeypatch):
    """A board provisioned as motor-only but wired with encoders: CFG still
    wins, so the extra sensing is reported rather than silently adopted."""
    _patch_hardware(monkeypatch, **_declaring(1000, encoder_stream=True))
    d = detect_hand()
    assert d.model_name == "orcahand-right"
    assert d.has_encoders is False
    assert d.undeclared_capabilities == ("encoders",)
    assert d.missing_capabilities == ()


def test_unrecognised_config_falls_back_to_probing_with_a_warning(monkeypatch, caplog):
    _patch_hardware(monkeypatch, **_declaring(1234, encoder_stream=True))
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        d = detect_hand()
    assert d.model_name == "orcahand-joint-right"
    assert d.declared_config == 1234
    assert "1234" in caplog.text


def test_undeclared_config_still_detects_by_probing(monkeypatch):
    """Older or unprovisioned boards keep the pre-CFG behaviour exactly."""
    _patch_hardware(monkeypatch, **_declaring(None, encoder_stream=True,
                                              tactile_register=True))
    d = detect_hand()
    assert d.model_name == "orcahand-full-right"
    assert d.declared_config is None
    assert (d.missing_capabilities, d.undeclared_capabilities) == ((), ())


def test_load_hand_warns_loudly_when_declared_sensing_is_dead(monkeypatch, caplog):
    _patch_hardware(monkeypatch, **_declaring(2500))
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        hand = load_hand()
    assert type(hand) is OrcaHandFull
    assert "CFG=2500" in caplog.text
    assert "ser-0001" in caplog.text


def test_load_hand_pins_the_sensing_port_of_a_declared_but_dead_link(monkeypatch):
    """connect() must fail against the real port, not against 'auto' finding
    nothing — that is what makes the error name the actual fault."""
    _patch_hardware(monkeypatch, **_declaring(2500))
    hand = load_hand()
    assert hand.config.encoder_serial_port == "/dev/cu.s"


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
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [detection])
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


def test_load_hand_skips_detection_for_a_mock(monkeypatch):
    """A mock hand has no bus, so nothing about it can come from hardware."""
    def _must_not_probe():
        raise AssertionError("the bus was probed for a mock hand")

    monkeypatch.setattr(hand_factory, "detect_hands", _must_not_probe)
    load_hand(mock=True)


@pytest.mark.parametrize(
    "kwargs", [{"model_name": "orcahand-left"}, {"model_version": "v2"}]
)
def test_a_named_model_survives_detection(monkeypatch, kwargs):
    """Detection still runs for a named model — it is what binds the store and
    pins the ports — but it says which hand this is, never which model."""
    _patch_hardware(monkeypatch)

    hand = load_hand(**kwargs)

    expected = kwargs.get("model_name", "orcahand-right")
    assert hand.config.config_path.endswith(f"{expected}/config.yaml")
