"""Tests for OrcaHand.connect() driver auto-detection."""

import dataclasses
from types import SimpleNamespace

import pytest

from orca_core.constants import KNOWN_VIDS
from orca_core.hardware_hand import MockOrcaHand
from orca_core.utils.utils import (
    find_single_usb_serial_port,
    motor_type_for_port,
)


DYNAMIXEL_VID = KNOWN_VIDS["dynamixel"][0]
FEETECH_VID = KNOWN_VIDS["feetech"][0]


def _fake_port(device: str, vid: int) -> SimpleNamespace:
    return SimpleNamespace(device=device, vid=vid, description="fake")


@pytest.fixture
def patch_comports(monkeypatch):
    def _set(ports):
        import serial.tools.list_ports as ltp
        monkeypatch.setattr(ltp, "comports", lambda: ports)
    return _set


# ----- USB VID lookup helpers ---------------------------------------------

def test_motor_type_for_port_matches_known_vid(patch_comports):
    patch_comports([_fake_port("/dev/cu.feetech", FEETECH_VID)])
    assert motor_type_for_port("/dev/cu.feetech") == "feetech"


def test_motor_type_for_port_returns_none_for_unknown_vid(patch_comports):
    patch_comports([_fake_port("/dev/cu.weird", 0xDEAD)])
    assert motor_type_for_port("/dev/cu.weird") is None


def test_find_single_usb_returns_lone_adapter(patch_comports):
    patch_comports([_fake_port("/dev/cu.weird", 0x2F5D)])
    assert find_single_usb_serial_port() == "/dev/cu.weird"


def test_find_single_usb_returns_none_when_multiple(patch_comports):
    patch_comports(
        [
            _fake_port("/dev/cu.a", 0x2F5D),
            _fake_port("/dev/cu.b", 0x2F5D),
        ]
    )
    assert find_single_usb_serial_port() is None


def test_find_single_usb_skips_non_usb_ports(patch_comports):
    patch_comports(
        [
            SimpleNamespace(device="/dev/cu.bluetooth", vid=None, description=""),
            _fake_port("/dev/cu.usb", 0x2F5D),
        ]
    )
    assert find_single_usb_serial_port() == "/dev/cu.usb"


# ----- _trial_probe -------------------------------------------------------

@pytest.fixture
def mock_hand(mock_config_dir):
    return MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))


def test_trial_probe_finds_feetech(mock_hand, monkeypatch):
    from orca_core.hardware import dynamixel_client, feetech_client
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient,
        "probe",
        staticmethod(lambda port, baudrate, motor_ids, **k: True),
    )
    motor_type, baudrate = OrcaHand_trial_probe(mock_hand, "/dev/cu.x")
    assert motor_type == "feetech"
    assert baudrate == 1_000_000


def test_trial_probe_finds_dynamixel_at_3M(mock_hand, monkeypatch):
    """v1 hands run Dynamixels at 3M; probe must iterate past the 1M default."""
    seen = []
    def fake_probe(port, baudrate, motor_ids, **k):
        seen.append(baudrate)
        return baudrate == 3_000_000
    from orca_core.hardware import dynamixel_client, feetech_client
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )
    motor_type, baudrate = OrcaHand_trial_probe(mock_hand, "/dev/cu.x")
    assert motor_type == "dynamixel"
    assert baudrate == 3_000_000
    assert seen == [1_000_000, 3_000_000]  # priority order


def test_trial_probe_returns_none_when_nothing_responds(mock_hand, monkeypatch):
    from orca_core.hardware import dynamixel_client, feetech_client
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(lambda *a, **k: False)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(lambda *a, **k: False)
    )
    assert OrcaHand_trial_probe(mock_hand, "/dev/cu.x") == (None, None)


def test_trial_probe_honours_pinned_motor_type(mock_hand, monkeypatch):
    """When motor_type is pinned in yaml, only baudrates iterate."""
    mock_hand.config = dataclasses.replace(mock_hand.config, motor_type="dynamixel")
    seen_types = set()
    def fake_probe(port, baudrate, motor_ids, **k):
        return False  # never responds — we just want to see the iteration
    from orca_core.hardware import dynamixel_client, feetech_client
    def feetech_probe(*a, **k):
        seen_types.add("feetech")
        return False
    monkeypatch.setattr(
        dynamixel_client.DynamixelClient, "probe", staticmethod(fake_probe)
    )
    monkeypatch.setattr(
        feetech_client.FeetechClient, "probe", staticmethod(feetech_probe)
    )
    OrcaHand_trial_probe(mock_hand, "/dev/cu.x")
    assert "feetech" not in seen_types


def OrcaHand_trial_probe(hand, port):
    """Hop over the mock-class override to test the real implementation."""
    from orca_core.hardware_hand import OrcaHand
    return OrcaHand._trial_probe(hand, port)
