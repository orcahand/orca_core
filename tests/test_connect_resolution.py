"""Tests for OrcaHand.connect() driver auto-detection and port helpers."""

import dataclasses
import logging
import os

from types import SimpleNamespace

import pytest

from orca_core.constants import KNOWN_VIDS
from orca_core import OrcaHand
from orca_core.utils.utils import (
    find_single_usb_serial_port,
    motor_type_for_port,
)

from tests._helpers import fake_serial_port

DYNAMIXEL_VID = KNOWN_VIDS["dynamixel"][0]
FEETECH_VID = KNOWN_VIDS["feetech"][0]


# ----- USB VID lookup helpers ---------------------------------------------

def test_motor_type_for_port_matches_known_vid(patch_comports):
    patch_comports([fake_serial_port("/dev/cu.feetech", FEETECH_VID)])
    assert motor_type_for_port("/dev/cu.feetech") == "feetech"


def test_motor_type_for_port_returns_none_for_unknown_vid(patch_comports):
    patch_comports([fake_serial_port("/dev/cu.weird", 0xDEAD)])
    assert motor_type_for_port("/dev/cu.weird") is None


def test_find_single_usb_returns_lone_adapter(patch_comports):
    patch_comports([fake_serial_port("/dev/cu.weird", 0x2F5D)])
    assert find_single_usb_serial_port() == "/dev/cu.weird"


def test_find_single_usb_returns_none_when_multiple(patch_comports):
    patch_comports(
        [
            fake_serial_port("/dev/cu.a", 0x2F5D),
            fake_serial_port("/dev/cu.b", 0x2F5D),
        ]
    )
    assert find_single_usb_serial_port() is None


def test_find_single_usb_skips_non_usb_ports(patch_comports):
    patch_comports(
        [
            SimpleNamespace(device="/dev/cu.bluetooth", vid=None, description=""),
            fake_serial_port("/dev/cu.usb", 0x2F5D),
        ]
    )
    assert find_single_usb_serial_port() == "/dev/cu.usb"


# ----- _trial_probe -------------------------------------------------------
# Use the unbound OrcaHand._trial_probe so we exercise the real implementation
# (not MockOrcaHand's synthetic override of driver resolution).

def _clear_driver(hand):
    hand.config = dataclasses.replace(hand.config, motor_type=None, baudrate=None)


def test_trial_probe_finds_feetech(mock_hand, monkeypatch):
    from orca_core.hardware import dynamixel_client, feetech_client

    _clear_driver(mock_hand)
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient,
        "probe",
        staticmethod(lambda port, baudrate, motor_ids, **k: True),
    )
    motor_type, baudrate = OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert motor_type == "feetech"
    assert baudrate == 1_000_000


def test_trial_probe_finds_dynamixel_at_3M(mock_hand, monkeypatch):
    """v1 hands run Dynamixels at 3M; probe must iterate past the 1M default."""
    seen = []

    def fake_probe(port, baudrate, motor_ids, **k):
        seen.append(baudrate)
        return baudrate == 3_000_000

    from orca_core.hardware import dynamixel_client, feetech_client

    _clear_driver(mock_hand)
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )
    motor_type, baudrate = OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert motor_type == "dynamixel"
    assert baudrate == 3_000_000
    assert seen == [1_000_000, 3_000_000]  # priority order


def test_trial_probe_returns_none_when_nothing_responds(mock_hand, monkeypatch):
    from orca_core.hardware import dynamixel_client, feetech_client

    _clear_driver(mock_hand)
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )
    assert OrcaHand._trial_probe(mock_hand, "/dev/cu.x") == (None, None)


def test_trial_probe_honours_pinned_motor_type(mock_hand, monkeypatch):
    """A pinned, responding motor_type settles the probe on its own."""
    _clear_driver(mock_hand)
    mock_hand.config = dataclasses.replace(mock_hand.config, motor_type="dynamixel")
    seen_types = set()

    def fake_feetech_probe(*a, **k):
        seen_types.add("feetech")
        return False

    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: True)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(fake_feetech_probe)
    )
    motor_type, _ = OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert motor_type == "dynamixel"
    assert "feetech" not in seen_types


def test_trial_probe_honours_pinned_baudrate(mock_hand, monkeypatch):
    """A pinned, responding baudrate is the only rate probed."""
    _clear_driver(mock_hand)
    mock_hand.config = dataclasses.replace(mock_hand.config, baudrate=3_000_000)
    seen = []

    def fake_probe(port, baudrate, motor_ids, **k):
        seen.append(baudrate)
        return True

    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(fake_probe)
    )
    OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert set(seen) == {3_000_000}


def test_trial_probe_widens_when_the_pinned_combination_is_silent(
    mock_hand, monkeypatch
):
    """A hand whose motors were swapped for another family still comes up on
    its bundled config: the pinned combination is tried first, then dropped."""
    from orca_core.hardware import dynamixel_client, feetech_client

    mock_hand.config = dataclasses.replace(
        mock_hand.config, motor_type="dynamixel", baudrate=1_000_000
    )
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: True)
    )
    assert OrcaHand._trial_probe(mock_hand, "/dev/cu.x")[0] == "feetech"


def test_resolve_motor_driver_verifies_pinned_combination(mock_hand, monkeypatch):
    """Pinned motor_type+baudrate still probe the bus, so a dead bus fails."""
    from orca_core.hardware import dynamixel_client, feetech_client

    mock_hand.config = dataclasses.replace(
        mock_hand.config, motor_type="dynamixel", baudrate=1_000_000
    )
    seen = []

    def fake_probe(port, baudrate, motor_ids, **k):
        seen.append(baudrate)
        return False

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(fake_probe)
    )
    assert not OrcaHand._resolve_motor_driver(mock_hand, "/dev/cu.x")
    assert seen[0] == 1_000_000


# ----- no write-back ------------------------------------------------------

def test_connect_never_writes_the_resolved_driver_back(mock_config_dir, monkeypatch):
    """A connect must leave config.yaml byte-identical: what the probe resolves
    is never pinned behind the operator's back, so the next connect re-probes."""
    import yaml

    from orca_core import MockOrcaHand

    class ProbedMockOrcaHand(MockOrcaHand):
        # Undo the mock's synthetic resolution so the real probe path runs.
        _resolve_motor_driver = OrcaHand._resolve_motor_driver

    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient,
        "probe",
        staticmethod(lambda port, baudrate, motor_ids: baudrate == 1_000_000),
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )

    config_path = mock_config_dir / "config.yaml"
    raw = yaml.safe_load(config_path.read_text())
    raw.pop("motor_type", None)
    raw.pop("baudrate", None)
    raw["port"] = "/dev/cu.fake"
    config_path.write_text(yaml.safe_dump(raw))
    before = config_path.read_text()

    hand = ProbedMockOrcaHand(config_path=str(config_path))
    success, msg = hand.connect()
    assert success, msg
    assert hand.config.motor_type == "dynamixel"
    assert hand.config.baudrate == 1_000_000
    hand.disconnect()

    assert config_path.read_text() == before


def test_packaged_configs_pin_no_motor_driver():
    """The shipped models must leave motor_type/baudrate unset so autodetection
    is what runs by default."""
    import glob

    import orca_core
    from orca_core.utils.utils import read_yaml

    configs = glob.glob(
        os.path.join(os.path.dirname(orca_core.__file__), "models", "v*", "*", "config.yaml")
    )
    assert configs
    for path in configs:
        raw = read_yaml(path) or {}
        assert "motor_type" not in raw, path
        assert "baudrate" not in raw, path
        assert raw.get("port", "auto") == "auto", path


def test_trial_probe_reports_via_logging_not_stdout(
        mock_hand, monkeypatch, capsys, caplog):
    from orca_core.hardware import dynamixel_client, feetech_client

    _clear_driver(mock_hand)
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )
    with caplog.at_level(logging.INFO, logger="orca_core.hardware.motor_resolution"):
        OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert capsys.readouterr().out == ""
    assert any("Probing" in record.getMessage() for record in caplog.records)


# ----- non-interactive connect ---------------------------------------------

def test_non_interactive_connect_skips_port_picker(mock_hand, monkeypatch):
    """connect(interactive=False) must fail cleanly instead of opening the
    terminal port picker when every port attempt fails."""
    import orca_core.hardware_hand as hardware_hand

    def fail_connect(self, port, base_config=None):
        raise ConnectionError("no motor responded")

    monkeypatch.setattr(OrcaHand, "_connect_on_port", fail_connect)
    monkeypatch.setattr(
        hardware_hand, "auto_detect_port", lambda *a, **k: None
    )
    # The config ships ``port: auto``; resolve it so the test exercises a
    # failing port attempt rather than the no-port-detected path.
    monkeypatch.setattr(
        hardware_hand, "find_single_usb_serial_port", lambda: "/dev/fake"
    )

    def picker_must_not_run():
        raise AssertionError("interactive picker invoked")

    monkeypatch.setattr(hardware_hand, "get_and_choose_port", picker_must_not_run)

    success, msg = OrcaHand.connect(mock_hand, interactive=False)
    assert not success
    assert "no motor responded" in msg


def test_failed_connect_restores_config(mock_hand, monkeypatch):
    """A failed connect() must not leave probed driver values in the config,
    so the next connect() re-probes and can still persist to yaml."""
    import orca_core.hardware_hand as hardware_hand

    _clear_driver(mock_hand)
    original_port = mock_hand.config.port

    def probe_then_fail(self, port, base_config=None):
        self.config = dataclasses.replace(
            base_config or self.config,
            port=port,
            motor_type="dynamixel",
            baudrate=1_000_000,
        )
        raise ConnectionError("could not open port")

    monkeypatch.setattr(OrcaHand, "_connect_on_port", probe_then_fail)
    monkeypatch.setattr(hardware_hand, "auto_detect_port", lambda *a, **k: None)
    monkeypatch.setattr(
        hardware_hand, "find_single_usb_serial_port", lambda: "/dev/fake"
    )

    success, _ = OrcaHand.connect(mock_hand, interactive=False)
    assert not success
    assert mock_hand.config.motor_type is None
    assert mock_hand.config.baudrate is None
    assert mock_hand.config.port == original_port


def test_unresolvable_auto_port_never_opens_the_literal_string(
    mock_hand, monkeypatch
):
    """``port: auto`` with nothing detectable must report the detection
    failure, not a missing-file error for a device named 'auto'."""
    import orca_core.hardware_hand as hardware_hand

    def must_not_open(self, port, base_config=None):
        raise AssertionError(f"attempted to open {port!r}")

    monkeypatch.setattr(OrcaHand, "_connect_on_port", must_not_open)
    monkeypatch.setattr(hardware_hand, "auto_detect_port", lambda *a, **k: None)
    monkeypatch.setattr(
        hardware_hand, "find_single_usb_serial_port", lambda: None
    )

    assert mock_hand.config.port == "auto"
    success, msg = OrcaHand.connect(mock_hand, interactive=False)
    assert not success
    assert "no motor bus detected" in msg
    assert "No such file or directory" not in msg


# ----- driver resolution through cli.create_hand ---------------------------

def test_mock_hand_keeps_the_family_its_config_declares(mock_config_dir):
    """The mock must not silently substitute Dynamixel semantics for a config
    that says feetech, or every Feetech branch stays untested."""
    from orca_core.hardware.mock_feetech_client import MockFeetechClient
    from orca_core.utils import cli, update_yaml

    config_path = str(mock_config_dir / "config.yaml")
    update_yaml(config_path, "motor_type", "feetech")

    hand = cli.create_hand(config_path, use_mock=True)
    assert hand.connect()[0]
    try:
        assert hand.config.motor_type == "feetech"
        assert isinstance(hand.motor_client, MockFeetechClient)
    finally:
        hand.disconnect()


def test_mock_hand_defaults_only_an_unpinned_family(mock_config_dir):
    from orca_core.utils import cli

    hand = cli.create_hand(str(mock_config_dir / "config.yaml"), use_mock=True)
    _clear_driver(hand)
    assert hand.connect()[0]
    try:
        assert hand.config.motor_type == "dynamixel"
        assert hand.config.baudrate == 1_000_000
    finally:
        hand.disconnect()


def test_connect_rechecks_control_mode_against_the_resolved_family(
    mock_config_dir, monkeypatch
):
    """multi_turn_position passes the union check while the family is unknown,
    and must be refused once the bus turns out to be Feetech."""
    from orca_core.hand_config import HandConfigValidationError
    from orca_core.utils.utils import update_yaml

    config_path = str(mock_config_dir / "config.yaml")
    update_yaml(config_path, "motor_type", None)
    update_yaml(config_path, "control_mode", "multi_turn_position")

    hand = OrcaHand(config_path=config_path)
    monkeypatch.setattr(
        OrcaHand, "_trial_probe", lambda self, port: ("feetech", 1_000_000)
    )
    with pytest.raises(HandConfigValidationError, match="feetech"):
        hand._resolve_motor_driver("/dev/cu.x")


def test_trial_probe_tries_every_family_at_a_rate_before_the_next_rate(mock_hand, monkeypatch):
    """Another family on the bus is likelier than the same family at an odd rate."""
    seen = []

    def record(family, answer_at=None):
        def probe(port, baudrate, motor_ids, **k):
            seen.append((family, baudrate))
            return baudrate == answer_at
        return staticmethod(probe)

    from orca_core.hardware import dynamixel_client, feetech_client

    _clear_driver(mock_hand)
    monkeypatch.setattr(dynamixel_client.DynamixelClient, "probe", record("dynamixel"))
    monkeypatch.setattr(feetech_client.FeetechClient, "probe", record("feetech", answer_at=500_000))
    assert OrcaHand._trial_probe(mock_hand, "/dev/cu.x") == ("feetech", 500_000)
    assert seen == [
        ("dynamixel", 1_000_000),
        ("feetech", 1_000_000),
        ("dynamixel", 3_000_000),
        ("feetech", 500_000),
    ]


def test_trial_probe_starts_with_the_family_the_adapter_vid_names(mock_hand, monkeypatch, patch_comports):
    from types import SimpleNamespace

    from orca_core.hardware import dynamixel_client, feetech_client

    patch_comports([SimpleNamespace(device="/dev/cu.x", vid=0x1A86, pid=0x55D3)])
    seen = []

    def record(family, answer):
        def probe(port, baudrate, motor_ids, **k):
            seen.append((family, baudrate))
            return answer
        return staticmethod(probe)

    _clear_driver(mock_hand)
    monkeypatch.setattr(dynamixel_client.DynamixelClient, "probe", record("dynamixel", False))
    monkeypatch.setattr(feetech_client.FeetechClient, "probe", record("feetech", True))
    assert OrcaHand._trial_probe(mock_hand, "/dev/cu.x") == ("feetech", 1_000_000)
    assert seen == [("feetech", 1_000_000)]
