"""Mock sensing hands connect out of the box: ``'auto'`` ports pin to
in-memory links, so ``load_hand(..., mock=True).connect()`` succeeds with the
bundled configs and never enumerates or probes real serial hardware.
"""

import pytest

from orca_core import load_hand
import orca_core.hardware_hand_sensing as hardware_hand_sensing
from orca_core.hardware.sensing import serial_discovery


@pytest.fixture(autouse=True)
def _no_hardware_probing(monkeypatch):
    def _boom(*args, **kwargs):
        raise AssertionError("mock hand touched real serial-port discovery")

    monkeypatch.setattr(serial_discovery, "discover_sensing_ports", _boom)
    monkeypatch.setattr(serial_discovery, "baud_for_port", _boom)
    monkeypatch.setattr(hardware_hand_sensing, "baud_for_port", _boom)


@pytest.mark.parametrize(
    "model",
    ["orcahand-touch-right", "orcahand-joint-right", "orcahand-full-right"],
)
def test_mock_hand_connects_with_bundled_auto_config(model):
    hand = load_hand(model_name=model, mock=True)
    success, msg = hand.connect()
    try:
        assert success, msg
    finally:
        ok, _ = hand.disconnect()
        assert ok


def test_mock_touch_serves_tactile_configuration():
    hand = load_hand(model_name="orcahand-touch-right", mock=True)
    success, msg = hand.connect()
    try:
        assert success, msg
        config = hand.get_tactile_configuration()
        assert config is not None
        assert config.connected.get("index") is True
    finally:
        hand.disconnect()
