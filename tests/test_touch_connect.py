"""Connect/disconnect contract tests for ``OrcaHandTouch``: the sensor
fallback skips ports whose device never answers, a sensor failure rolls the
motor bus back, and ``disconnect()`` returns the ``(success, msg)`` tuple.
"""

from __future__ import annotations

import dataclasses
import os

import orca_core.hardware_hand_sensing as hardware_hand_sensing
from orca_core import MockOrcaHandTouch, OrcaHandTouch
from orca_core.hand_config import OrcaHandTouchConfig
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.serial_discovery import SensingPorts
from orca_core.hardware.sensing.tactile_mock import TactileMockState, install_tactile_mock

from tests._helpers import wait_until


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOUCH_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-touch-right", "config.yaml"
)


class _FakePortsHandTouch(OrcaHandTouch):
    """Production ``OrcaHandTouch`` whose tactile links are in-memory; only
    ports named in ``good_ports`` get a register responder, so any other port
    opens fine but never answers (a wrong-device stand-in)."""

    def __init__(self, config: OrcaHandTouchConfig, good_ports: set[str]):
        super().__init__(config=config)
        self._good_ports = good_ports
        self.opened_ports: list[str] = []

    def _create_tactile_link(self, port: str, baudrate: int) -> MockHandSerialLink:
        self.opened_ports.append(port)
        link = MockHandSerialLink(port=port, baudrate=baudrate)
        if port in self._good_ports:
            install_tactile_mock(link, TactileMockState())
        return link


def _touch_config(sensor_port: str) -> OrcaHandTouchConfig:
    config = OrcaHandTouchConfig.from_config_path(config_path=TOUCH_CONFIG)
    return dataclasses.replace(config, sensor_port=sensor_port)


def _install_fake_resolution(monkeypatch, auto_port: str | None):
    def fake_resolve(
        tactile_override="auto", encoder_override="auto", tactile_baud_override="auto"
    ):
        if tactile_override == "auto":
            return SensingPorts(
                tactile=auto_port, encoder=None, tactile_baudrate=921600
            )
        if tactile_override == "disabled":
            return SensingPorts(tactile=None, encoder=None)
        baud = None if tactile_baud_override == "auto" else int(tactile_baud_override)
        return SensingPorts(tactile=tactile_override, encoder=None, tactile_baudrate=baud)

    monkeypatch.setattr(hardware_hand_sensing, "resolve_sensing_ports", fake_resolve)
    monkeypatch.setattr(hardware_hand_sensing, "baud_for_port", lambda port: 921600)


def test_silent_configured_port_falls_back_to_discovered_port(monkeypatch):
    """A configured port that opens but never answers must not count as
    connected; auto-detection continues to the next candidate."""
    _install_fake_resolution(monkeypatch, auto_port="/dev/good")
    hand = _FakePortsHandTouch(_touch_config("/dev/silent"), good_ports={"/dev/good"})

    ok, msg = hand.connect_sensors_only()
    try:
        assert ok, msg
        assert hand.opened_ports == ["/dev/silent", "/dev/good"]
        assert hand.config.sensor_port == "/dev/good"
        assert hand.get_tactile_configuration() is not None
    finally:
        hand._teardown_tactile()


def test_no_responding_candidate_reports_failure(monkeypatch):
    _install_fake_resolution(monkeypatch, auto_port=None)
    hand = _FakePortsHandTouch(_touch_config("auto"), good_ports=set())

    ok, msg = hand.connect_sensors_only()
    assert not ok
    assert "no usable port" in msg
    assert hand._tactile_client is None
    assert hand._tactile_link is None


def test_touch_connect_rolls_back_motor_bus_on_sensor_failure():
    class _SensorFail(MockOrcaHandTouch):
        def _connect_sensor_with_fallback(self):
            return False, "sensor boom"

    hand = _SensorFail(config_path=TOUCH_CONFIG)
    success, msg = hand.connect()
    assert not success
    assert "sensor boom" in msg
    assert not hand.is_connected()


def test_touch_second_connect_is_noop_and_orphans_nothing():
    """A second connect() must be a no-op success — not re-open the tactile
    link and orphan the first one."""
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    hand.connect()
    try:
        link, client = hand._tactile_link, hand._tactile_client
        ok, msg = hand.connect()
        assert ok and msg == "Already connected"
        assert hand._tactile_link is link
        assert hand._tactile_client is client
    finally:
        hand.disconnect()


def test_touch_connect_after_sensors_only_keeps_tactile_link():
    """connect() after connect_sensors_only() completes the motor half but
    must not re-attach over the live tactile link."""
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    ok, _ = hand.connect_sensors_only()
    assert ok
    link, client = hand._tactile_link, hand._tactile_client
    ok, msg = hand.connect()
    try:
        assert ok
        assert "Sensor already connected" in msg
        assert hand.is_connected()
        assert hand._tactile_link is link
        assert hand._tactile_client is client
    finally:
        hand.disconnect()


def test_touch_link_health_reports_port_death():
    """The public health accessor must surface link liveness and the latched
    port-dead state without callers touching private link attributes."""
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    assert hand.get_tactile_link_health() is None
    ok, _ = hand.connect_sensors_only()
    assert ok
    try:
        health = hand.get_tactile_link_health()
        assert health.connected
        assert not health.port_dead
        assert health.port_error is None

        hand._tactile_link.simulate_port_death()
        wait_until(lambda: hand.get_tactile_link_health().port_dead)
        health = hand.get_tactile_link_health()
        assert not health.connected
        assert health.port_error
    finally:
        hand.disconnect()


def test_zero_tactile_sensors_forwards_timeout():
    """zero_tactile_sensors must pass its timeout_s through to the client's
    offset capture."""
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    ok, _ = hand.connect_sensors_only()
    assert ok
    captured = {}

    def fake_capture(num_samples=100, timeout_s=None):
        captured.update(num_samples=num_samples, timeout_s=timeout_s)
        return {}

    hand._tactile_client.capture_taxel_offsets = fake_capture
    try:
        hand.zero_tactile_sensors(num_samples=7, timeout_s=1.5)
        assert captured == {"num_samples": 7, "timeout_s": 1.5}
    finally:
        hand.disconnect()


def test_touch_disconnect_returns_lifecycle_tuple():
    hand = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    success, msg = hand.connect()
    assert success, msg

    assert hand.disconnect() == (True, "Disconnected successfully")
    assert hand._tactile_client is None
    # Tuple-shaped even when nothing was ever connected.
    never_connected = MockOrcaHandTouch(config_path=TOUCH_CONFIG)
    assert never_connected.disconnect() == (True, "Disconnected successfully")
