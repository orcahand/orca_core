"""Tests for OrcaHand.connect() driver auto-detection and port helpers."""

import dataclasses

from types import SimpleNamespace

from orca_core.constants import KNOWN_VIDS
from orca_core import OrcaHand
from orca_core.utils.utils import (
    find_single_usb_serial_port,
    motor_type_for_port,
)

from .conftest import fake_serial_port

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
    """When motor_type is pinned in yaml, only baudrates iterate."""
    _clear_driver(mock_hand)
    mock_hand.config = dataclasses.replace(mock_hand.config, motor_type="dynamixel")
    seen_types = set()

    def fake_feetech_probe(*a, **k):
        seen_types.add("feetech")
        return False

    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(fake_feetech_probe)
    )
    OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert "feetech" not in seen_types


def test_trial_probe_honours_pinned_baudrate(mock_hand, monkeypatch):
    """When baudrate is pinned in yaml, only that rate is probed."""
    _clear_driver(mock_hand)
    mock_hand.config = dataclasses.replace(mock_hand.config, baudrate=3_000_000)
    seen = []

    def fake_probe(port, baudrate, motor_ids, **k):
        seen.append(baudrate)
        return False

    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(fake_probe)
    )
    OrcaHand._trial_probe(mock_hand, "/dev/cu.x")
    assert set(seen) == {3_000_000}


# ----- persistence ---------------------------------------------------------

def test_resolved_driver_persisted_to_yaml(mock_config_dir, monkeypatch):
    """A successful auto-probe writes motor_type/baudrate back to config.yaml."""
    import yaml

    from orca_core import MockOrcaHand
    from orca_core.utils.utils import read_yaml

    class ProbedMockOrcaHand(MockOrcaHand):
        # Undo the mock's synthetic resolution so the real probe path runs.
        _resolve_motor_driver = OrcaHand._resolve_motor_driver
        _persist_resolved_driver = OrcaHand._persist_resolved_driver

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

    hand = ProbedMockOrcaHand(config_path=str(config_path))
    success, msg = hand.connect()
    assert success, msg
    hand.disconnect()

    persisted = read_yaml(str(config_path))
    assert persisted["motor_type"] == "dynamixel"
    assert persisted["baudrate"] == 1_000_000


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

    def picker_must_not_run():
        raise AssertionError("interactive picker invoked")

    monkeypatch.setattr(hardware_hand, "get_and_choose_port", picker_must_not_run)

    success, msg = OrcaHand.connect(mock_hand, interactive=False)
    assert not success
    assert "No port selected" in msg
