"""Hand autodetection: identity parsing, the detection ladder, and load_hand()."""

import dataclasses
import errno
import logging
import os

import pytest

import orca_core.hand_factory as hand_factory
from orca_core import OrcaHand, OrcaHandFull, OrcaHandTouch, detect_hand, load_hand
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo, parse_orca_info


# ----- identity-line parsing ------------------------------------------------

def test_parse_full_identity_line():
    info = parse_orca_info(
        b"ORCA:SENSOR;SIDE=L;HW=2;FW=1;SN=OH2-L-0000-0000;BID=0123456789ABCDEF"
    )
    assert info == OrcaBoardInfo(
        role="sensor", side="left", hw_version=2, fw_version=1,
        serial="OH2-L-0000-0000", board_id="0123456789ABCDEF",
    )
    assert info.hand_id == "OH2-L-0000-0000"


def test_parse_reads_sensing_config():
    info = parse_orca_info(
        b"ORCA:SENSOR;SIDE=R;HW=2;FW=2;CFG=2500;SN=ser-0000;BID=0123456789ABCDEF"
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

def _patch_hardware(
    monkeypatch,
    *,
    oh_ports=(),
    infos=None,
    encoder_stream=False,
    paxini_port=None,
    tactile_register=False,
    motor_family=(None, None),
    classic_ports=(),
    busy_ports=(),
    probed=None,
):
    """``motor_family`` is one answer for every port, or a per-port dict;
    ``probed`` collects the ports the family probe was asked about."""
    infos = infos or {}
    probed = probed if probed is not None else []

    def family(port):
        probed.append(port)
        if isinstance(motor_family, dict):
            return motor_family.get(port, (None, None))
        return motor_family

    monkeypatch.setattr(hand_factory, "_detect_motor_family", family)
    monkeypatch.setattr(hand_factory, "_classic_motor_ports", lambda: list(classic_ports))
    monkeypatch.setattr(hand_factory, "port_in_use", lambda port: port in busy_ports)
    monkeypatch.setattr(hand_factory, "oh_board_ports", lambda: list(oh_ports))
    monkeypatch.setattr(hand_factory, "probe_orca_info", lambda port: infos.get(port))
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
    assert (d.motor_type, d.motor_baudrate) == (None, None)
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
    info = OrcaBoardInfo(role="sensor", side="right", serial="ser-0000",
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
    assert "ser-0000" in caplog.text


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


def test_busy_bare_adapter_is_reported(monkeypatch):
    """A bare adapter held by another process is skipped by the family probe,
    so without this it would read as 'no motor found' with no reason given."""
    _patch_hardware(
        monkeypatch,
        classic_ports=["/dev/cu.motor"],
        busy_ports=("/dev/cu.motor",),
    )
    d = detect_hand()
    assert d.motor_port is None
    assert d.busy_ports == ("/dev/cu.motor",)


# ----- motor-family detection ------------------------------------------------

def test_detects_the_motor_family_on_the_motor_port(monkeypatch):
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m"],
        infos={"/dev/cu.m": OrcaBoardInfo(role="motor", side="right")},
        motor_family=("feetech", 1_000_000),
    )
    d = detect_hand()
    assert (d.motor_type, d.motor_baudrate) == ("feetech", 1_000_000)


def test_motor_family_detection_is_non_fatal(monkeypatch):
    def _boom(port, progress_callback=None):
        raise OSError("port went away")

    monkeypatch.setattr("orca_core.maintenance.motor_chain.detect_motor_type", _boom)
    assert hand_factory._detect_motor_family("/dev/cu.m") == (None, None)


def test_motor_family_falls_back_to_the_trial_probe(monkeypatch):
    """Motors already programmed into a chain no longer answer at their
    factory defaults, so the family comes from the connect-time probe."""
    monkeypatch.setattr(
        "orca_core.maintenance.motor_chain.detect_motor_type",
        lambda port, progress_callback=None: None,
    )
    monkeypatch.setattr(
        "orca_core.hardware.motor_resolution.trial_probe",
        lambda config, port: ("feetech", 1_000_000),
    )
    assert hand_factory._detect_motor_family("/dev/cu.m") == ("feetech", 1_000_000)


def test_pin_detected_ports_fills_an_unpinned_family():
    """The bundled models pin nothing, so the detected family and its baud
    rate are what the config ends up with."""
    config = hand_factory.OrcaHandConfig.from_config_path(
        model_name="orcahand-right", model_version="v2"
    )
    assert config.motor_type is None

    detection = hand_factory.HandDetection(
        model_name="orcahand-right", side="right", has_tactile=False,
        has_encoders=False, motor_port="/dev/cu.m", motor_type="feetech",
        motor_baudrate=500_000,
    )
    pinned = hand_factory._pin_detected_ports(config, detection)
    assert pinned.motor_type == "feetech"
    assert pinned.baudrate == 500_000
    assert pinned.port == "/dev/cu.m"


def test_pin_detected_ports_keeps_a_hand_written_family(caplog):
    """A family written into the config is a deliberate override: it survives a
    detection that disagrees, and the clash is logged."""
    config = dataclasses.replace(
        hand_factory.OrcaHandConfig.from_config_path(
            model_name="orcahand-right", model_version="v2"
        ),
        motor_type="dynamixel",
        baudrate=1_000_000,
    )
    detection = hand_factory.HandDetection(
        model_name="orcahand-right", side="right", has_tactile=False,
        has_encoders=False, motor_port="/dev/cu.m", motor_type="feetech",
        motor_baudrate=500_000,
    )
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        pinned = hand_factory._pin_detected_ports(config, detection)

    assert pinned.motor_type == "dynamixel"
    assert pinned.baudrate == 1_000_000
    assert any(
        "motor_type" in r.getMessage() and "feetech" in r.getMessage()
        for r in caplog.records
    ), "an override must say which field and what was detected"


def test_pin_detected_ports_keeps_a_hand_written_port(caplog):
    """A port written into the config wins over the detected one."""
    config = dataclasses.replace(
        hand_factory.OrcaHandConfig.from_config_path(
            model_name="orcahand-right", model_version="v2"
        ),
        port="/dev/cu.pinned",
    )
    detection = hand_factory.HandDetection(
        model_name="orcahand-right", side="right", has_tactile=False,
        has_encoders=False, motor_port="/dev/cu.detected",
    )
    with caplog.at_level(logging.WARNING, logger="orca_core.hand_factory"):
        pinned = hand_factory._pin_detected_ports(config, detection)

    assert pinned.port == "/dev/cu.pinned"
    assert any("port" in r.getMessage() for r in caplog.records)


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


# ----- bare USB adapters (no controller board) ------------------------------

def test_bare_adapter_becomes_the_motor_port_when_no_board_answers(monkeypatch):
    _patch_hardware(
        monkeypatch,
        classic_ports=["/dev/cu.usbserial-XXXX"],
        motor_family={"/dev/cu.usbserial-XXXX": ("feetech", 1_000_000)},
    )
    d = detect_hand()
    assert d.motor_port == "/dev/cu.usbserial-XXXX"
    assert (d.motor_type, d.motor_baudrate) == ("feetech", 1_000_000)
    assert d.model_name == "orcahand-right"


def test_bare_adapter_fallback_skips_busy_and_silent_adapters(monkeypatch):
    probed = []
    _patch_hardware(
        monkeypatch,
        classic_ports=["/dev/cu.busy", "/dev/cu.silent", "/dev/cu.motors"],
        busy_ports={"/dev/cu.busy"},
        motor_family={"/dev/cu.motors": ("dynamixel", 1_000_000)},
        probed=probed,
    )
    d = detect_hand()
    assert d.motor_port == "/dev/cu.motors"
    assert probed == ["/dev/cu.silent", "/dev/cu.motors"]


def test_bare_adapter_fallback_leaves_the_motor_port_unset_when_all_are_silent(monkeypatch):
    _patch_hardware(monkeypatch, classic_ports=["/dev/cu.usbserial-XXXX"])
    d = detect_hand()
    assert d.motor_port is None
    assert (d.motor_type, d.motor_baudrate) == (None, None)


def test_bare_adapter_fallback_never_probes_the_sensing_port(monkeypatch):
    """An FTDI sensing adapter shares a vendor ID with Dynamixel adapters."""
    probed = []
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.s"],
        infos={"/dev/cu.s": OrcaBoardInfo(role="sensor", side="left")},
        classic_ports=["/dev/cu.s"],
        motor_family={"/dev/cu.s": ("dynamixel", 1_000_000)},
        probed=probed,
    )
    d = detect_hand()
    assert d.motor_port is None
    assert probed == []


def test_controller_board_motor_port_wins_over_a_bare_adapter(monkeypatch):
    probed = []
    _patch_hardware(
        monkeypatch,
        oh_ports=["/dev/cu.m"],
        infos={"/dev/cu.m": OrcaBoardInfo(role="motor", side="right")},
        classic_ports=["/dev/cu.usbserial-XXXX"],
        motor_family=("dynamixel", 1_000_000),
        probed=probed,
    )
    d = detect_hand()
    assert d.motor_port == "/dev/cu.m"
    assert probed == ["/dev/cu.m"]


def test_classic_motor_ports_match_known_motor_vendor_ids(patch_comports):
    from types import SimpleNamespace

    patch_comports([
        SimpleNamespace(device="/dev/cu.ch340", vid=0x1A86),
        SimpleNamespace(device="/dev/cu.ftdi", vid=0x0403),
        SimpleNamespace(device="/dev/cu.paxini", vid=0x28E9),
        SimpleNamespace(device="/dev/cu.Bluetooth-Incoming-Port", vid=None),
    ])
    assert hand_factory._classic_motor_ports() == ["/dev/cu.ch340", "/dev/cu.ftdi"]


@pytest.mark.parametrize("exc, expected", [
    (OSError(errno.EBUSY, "busy"), True),
    (OSError(errno.EAGAIN, "again"), True),
    (FileNotFoundError(errno.ENOENT, "absent"), False),
    (None, False),
])
def test_port_in_use_reads_the_open_error(monkeypatch, exc, expected):
    import serial

    from orca_core.hardware.sensing.serial_discovery import port_in_use

    class FakeSerial:
        def __init__(self, port, **kwargs):
            assert kwargs.get("exclusive") is True
            if exc is not None:
                raise exc

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    monkeypatch.setattr(serial, "Serial", FakeSerial)
    assert port_in_use("/dev/cu.usbmodemXXXX") is expected


# ----- family current defaults resolve when the family is known ----------------

def test_load_hand_resolves_default_currents_from_the_detected_family(monkeypatch):
    detection = hand_factory.HandDetection(
        model_name="orcahand-right", side="right", has_tactile=False, has_encoders=False,
        motor_port="/dev/cu.usbserial-XXXX", motor_type="feetech", motor_baudrate=1_000_000,
    )
    monkeypatch.setattr(hand_factory, "detect_hand", lambda: detection)
    hand = load_hand()
    assert (hand.config.max_current, hand.config.calibration_current,
            hand.config.wrist_calibration_current) == (900, 900, 900)


def test_load_hand_leaves_default_currents_for_connect_when_no_family_is_known(monkeypatch):
    monkeypatch.setattr(hand_factory, "detect_hand", lambda: hand_factory.HandDetection(
        model_name="orcahand-right", side="right", has_tactile=False, has_encoders=False))
    hand = load_hand()
    assert hand.config.max_current == "default"
    assert not hand.config.currents_resolved


def test_load_hand_resolves_default_currents_for_a_pinned_family_mock():
    hand = load_hand(model_name="orcahand-right", mock=True)
    assert hand.config.max_current == "default"
    ok, msg = hand.connect()
    assert ok, msg
    assert (hand.config.max_current, hand.config.calibration_current) == (300, 300)
    hand.disconnect()


def test_connect_resolves_default_currents_from_the_family_on_the_bus(tmp_path):
    import shutil

    from orca_core.utils import update_yaml

    src = os.path.join(os.path.dirname(hand_factory.__file__), "models", "v2", "orcahand-right", "config.yaml")
    path = tmp_path / "config.yaml"
    shutil.copy(src, path)
    update_yaml(str(path), "motor_type", "feetech")
    hand = load_hand(config_path=str(path), mock=True)
    assert (hand.config.max_current, hand.config.calibration_current) == (900, 900)
    ok, msg = hand.connect()
    assert ok, msg
    assert hand.config.wrist_calibration_current == 900
    hand.disconnect()


def test_connect_rejects_a_pinned_max_current_below_the_family_calibration_current(tmp_path):
    import shutil

    from orca_core.utils import update_yaml

    src = os.path.join(os.path.dirname(hand_factory.__file__), "models", "v2", "orcahand-right", "config.yaml")
    path = tmp_path / "config.yaml"
    shutil.copy(src, path)
    update_yaml(str(path), "max_current", 500)
    hand = load_hand(config_path=str(path), mock=True)
    hand.config = dataclasses.replace(hand.config, motor_type="feetech")
    ok, msg = hand.connect(interactive=False)
    assert not ok
    assert "Max current" in msg and "config.yaml" in msg
    assert not hand.is_connected()
