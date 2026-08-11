"""Interlocks that keep two attached hands from corrupting each other.

Each test here pins a failure that is silent without the guard: a hand
adopting its neighbour's bus, two hands writing one calibration file, or a
probe opening a port another client already owns.
"""

import logging
import os
from types import SimpleNamespace

import pytest

from orca_core import MockOrcaHand, OrcaHand
from orca_core.constants import PORT_BUSY_ERRNOS


# ----- port claiming -------------------------------------------------------

def test_claim_port_lock_raises_when_another_client_holds_the_port(tmp_path):
    """A held advisory lock means another client owns this bus; opening it
    anyway lets two clients read each other's replies."""
    import errno

    from orca_core.hardware.motor_client import PortHeldError, claim_port_lock

    real = tmp_path / "port"
    real.write_bytes(b"")
    first = open(real, "rb+")
    second = open(real, "rb+")
    try:
        claim_port_lock(SimpleNamespace(ser=first), "/dev/cu.fake")
        with pytest.raises(PortHeldError, match="already claimed"):
            claim_port_lock(SimpleNamespace(ser=second), "/dev/cu.fake")
    finally:
        first.close()
        second.close()

    assert errno.EAGAIN in PORT_BUSY_ERRNOS


def test_claim_port_lock_is_a_noop_for_mocked_handles():
    """Mocked ports and platforms without fcntl must connect normally."""
    from orca_core.hardware.motor_client import claim_port_lock

    claim_port_lock(SimpleNamespace(), "/dev/cu.mock")
    claim_port_lock(SimpleNamespace(ser=object()), "/dev/cu.mock")


def test_claim_port_lock_ignores_filesystems_that_cannot_lock(monkeypatch, tmp_path):
    """A lock failure that is not 'someone else holds it' stays best-effort."""
    import errno
    import fcntl

    from orca_core.hardware.motor_client import claim_port_lock

    def refuse(fd, op):
        raise OSError(errno.EINVAL, "locking not supported here")

    monkeypatch.setattr(fcntl, "flock", refuse)
    handle = open(tmp_path / "p", "wb+")
    try:
        claim_port_lock(SimpleNamespace(ser=handle), "/dev/cu.fake")
    finally:
        handle.close()


# ----- the connect cascade -------------------------------------------------

def test_explicit_port_failure_does_not_wander_to_another_bus(
        mock_config_dir, monkeypatch):
    """An explicitly configured port that fails must not fall back onto
    whatever else answers — with two hands attached that is the other hand."""
    import orca_core.hardware_hand as hardware_hand

    searched = []

    def spy_auto_detect(motor_type=None):
        searched.append(motor_type)
        return "/dev/cu.the-other-hand"

    monkeypatch.setattr(hardware_hand, "auto_detect_port", spy_auto_detect)

    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    hand.config = __import__("dataclasses").replace(hand.config, port="/dev/cu.mine")
    monkeypatch.setattr(
        hand, "_connect_on_port",
        lambda port, base: (_ for _ in ()).throw(OSError("bus down")),
    )

    ok, msg = hand.connect(interactive=False)
    assert not ok
    assert "/dev/cu.mine" in msg
    assert searched == [], "an explicit port must not trigger a port search"


def test_auto_port_failure_still_falls_back(mock_config_dir, monkeypatch):
    """The recovery cascade stays available when no port was configured.

    Uses a production OrcaHand: the mock mixin rewrites ``port: auto`` to
    ``"mock"`` before connect() runs, so it cannot reach this path.
    """
    import dataclasses

    import orca_core.hardware_hand as hardware_hand

    detected = iter(["/dev/cu.first", "/dev/cu.second"])
    monkeypatch.setattr(
        hardware_hand, "auto_detect_port",
        lambda motor_type=None: next(detected, None),
    )
    monkeypatch.setattr(hardware_hand, "find_single_usb_serial_port", lambda: None)

    hand = OrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    hand.config = dataclasses.replace(hand.config, port="auto")

    tried = []

    def record(port, base):
        tried.append(port)
        raise OSError("nope")

    monkeypatch.setattr(hand, "_connect_on_port", record)
    ok, _ = hand.connect(interactive=False)
    assert not ok
    assert tried == ["/dev/cu.first", "/dev/cu.second"], tried


# ----- driver persistence --------------------------------------------------

def test_resolved_port_is_never_written_over_an_explicit_pin(tmp_path):
    """One hand's connect must not repoint another hand's config."""
    import yaml

    from orca_core.hardware import motor_resolution

    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.safe_dump({"port": "/dev/cu.mine"}))

    existing = SimpleNamespace(
        port="/dev/cu.mine", motor_type=None, baudrate=None
    )
    resolved = SimpleNamespace(
        port="/dev/cu.the-other-hand",
        motor_type="dynamixel",
        baudrate=1_000_000,
        config_path=str(config_path),
    )
    motor_resolution.persist_resolved_driver(existing, resolved)

    data = yaml.safe_load(config_path.read_text())
    assert data["port"] == "/dev/cu.mine"
    assert data["motor_type"] == "dynamixel"


def test_trial_probe_puts_no_traffic_on_a_port_another_client_holds(monkeypatch):
    """The guard exists to keep the probe off a live bus, so pin that no probe
    ran — not merely that the call reported nothing."""
    from orca_core.hardware import motor_resolution
    from orca_core.hardware.dynamixel_client import DynamixelClient
    from orca_core.hardware.feetech_client import FeetechClient
    from orca_core.hardware.motor_client import PortHeldError
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr(serial_discovery, "port_in_use", lambda port: True)

    probed = []
    for cls in (DynamixelClient, FeetechClient):
        monkeypatch.setattr(
            cls, "probe",
            staticmethod(
                lambda port, baudrate, motor_ids, _c=cls: probed.append(
                    (_c.__name__, port, baudrate)
                )
            ),
        )

    config = SimpleNamespace(motor_type=None, baudrate=None, motor_ids=[1, 2])
    with pytest.raises(PortHeldError, match="held by another client"):
        motor_resolution.trial_probe(config, "/dev/cu.busy")

    assert probed == [], probed


def test_a_held_port_is_not_reported_as_a_wiring_fault(mock_config_dir, monkeypatch):
    """'no motor responded (check power and wiring)' is the wrong diagnosis
    when the real cause is another hand holding the bus."""
    import dataclasses

    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr(serial_discovery, "port_in_use", lambda port: True)

    hand = OrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    hand.config = dataclasses.replace(hand.config, port="/dev/cu.busy")

    ok, msg = hand.connect(interactive=False)
    assert not ok
    assert "held by another client" in msg
    assert "check power and wiring" not in msg


# ----- calibration ownership -----------------------------------------------

class _RealCalibrationMockHand(MockOrcaHand):
    """Mock transport, but the production rule that calibration reaches disk —
    so the shared-file interlock is exercised without hardware."""

    _persist_calibration = True


def test_two_connected_hands_cannot_share_one_calibration_file(mock_config_dir):
    """Two hands of one model resolve to the same packaged calibration.yaml;
    the second to calibrate would silently overwrite the first."""
    config_path = str(mock_config_dir / "config.yaml")

    first = _RealCalibrationMockHand(config_path=config_path)
    ok, msg = first.connect()
    assert ok, msg
    try:
        second = _RealCalibrationMockHand(config_path=config_path)
        with pytest.raises(ValueError, match="already in use by another hand"):
            second.connect()
    finally:
        first.disconnect()


def test_overlapping_connects_cannot_both_claim_one_calibration_file(mock_config_dir):
    """The window that matters is one hand's connect() still running when the
    other starts — the claim cannot wait for the claimant to be connected."""
    import threading
    import time

    config_path = str(mock_config_dir / "config.yaml")

    class _SlowConnect(_RealCalibrationMockHand):
        def _connect_cascade(self, interactive):
            time.sleep(0.15)
            return super()._connect_cascade(interactive)

    hands = [_SlowConnect(config_path=config_path) for _ in range(2)]
    outcomes: list = [None, None]

    def run(index):
        try:
            outcomes[index] = hands[index].connect()[0]
        except ValueError:
            outcomes[index] = "refused"

    threads = [threading.Thread(target=run, args=(i,)) for i in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    try:
        assert outcomes.count(True) == 1, outcomes
        assert outcomes.count("refused") == 1, outcomes
    finally:
        for hand in hands:
            hand.disconnect()


def test_a_failed_connect_does_not_strand_the_calibration_claim(
        mock_config_dir, monkeypatch):
    """A hand that never reached the bus holds nothing."""
    config_path = str(mock_config_dir / "config.yaml")

    first = _RealCalibrationMockHand(config_path=config_path)
    monkeypatch.setattr(
        first, "_connect_on_port",
        lambda port, base: (_ for _ in ()).throw(OSError("bus down")),
    )
    assert not first.connect(interactive=False)[0]

    second = _RealCalibrationMockHand(config_path=config_path)
    ok, msg = second.connect()
    assert ok, msg
    second.disconnect()


def test_a_dropped_hand_releases_its_calibration_claim(mock_config_dir):
    """A long-lived process that drops a connected hand without disconnect()
    must not strand its calibration file for the rest of the run."""
    import gc

    config_path = str(mock_config_dir / "config.yaml")

    hand = _RealCalibrationMockHand(config_path=config_path)
    assert hand.connect()[0]
    del hand
    gc.collect()

    second = _RealCalibrationMockHand(config_path=config_path)
    ok, msg = second.connect()
    assert ok, msg
    second.disconnect()


def test_calibration_claim_sees_through_a_symlinked_path(mock_config_dir, tmp_path):
    """Two configs reaching one calibration.yaml by different paths are still
    two hands writing one file."""
    link_dir = tmp_path / "linked"
    link_dir.symlink_to(mock_config_dir, target_is_directory=True)

    first = _RealCalibrationMockHand(config_path=str(mock_config_dir / "config.yaml"))
    second = _RealCalibrationMockHand(config_path=str(link_dir / "config.yaml"))

    assert first.connect()[0]
    try:
        with pytest.raises(ValueError, match="already in use by another hand"):
            second.connect()
    finally:
        first.disconnect()


def test_calibration_claim_is_released_when_the_owner_disconnects(mock_config_dir):
    """Sequential hands on one config are normal; only overlap is refused."""
    config_path = str(mock_config_dir / "config.yaml")

    first = _RealCalibrationMockHand(config_path=config_path)
    assert first.connect()[0]
    first.disconnect()

    second = _RealCalibrationMockHand(config_path=config_path)
    ok, msg = second.connect()
    assert ok, msg
    second.disconnect()


def test_distinct_calibration_paths_may_be_connected_together(mock_config_dir, tmp_path):
    """The interlock is on the file, not on the number of hands."""
    import shutil

    other = tmp_path / "second-hand"
    other.mkdir()
    shutil.copy(mock_config_dir / "config.yaml", other / "config.yaml")
    (other / "calibration.yaml").write_text("{}\n", encoding="utf-8")

    first = _RealCalibrationMockHand(config_path=str(mock_config_dir / "config.yaml"))
    second = _RealCalibrationMockHand(config_path=str(other / "config.yaml"))
    assert first.connect()[0]
    try:
        ok, msg = second.connect()
        assert ok, msg
        second.disconnect()
    finally:
        first.disconnect()


def test_mock_hands_do_not_claim_calibration(mock_config_dir):
    """Mocks never persist calibration, so they never contend for the file."""
    config_path = str(mock_config_dir / "config.yaml")

    first = MockOrcaHand(config_path=config_path)
    second = MockOrcaHand(config_path=config_path)
    assert first.connect()[0]
    try:
        assert second.connect()[0]
        second.disconnect()
    finally:
        first.disconnect()


# ----- detection -----------------------------------------------------------

def test_detect_hand_warns_when_more_than_one_board_is_attached(monkeypatch, caplog):
    """detect_hand() describes one hand; with two boards its motor and
    sensing ports can be claimed from different ones."""
    from orca_core import hand_factory

    monkeypatch.setattr(
        hand_factory, "oh_board_ports",
        lambda: ["/dev/cu.a1", "/dev/cu.a2", "/dev/cu.b1", "/dev/cu.b2"],
    )
    monkeypatch.setattr(hand_factory, "probe_orca_info", lambda port: None)
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda port: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)
    monkeypatch.setattr(hand_factory, "port_in_use", lambda port: False)

    with caplog.at_level(logging.WARNING):
        detection = hand_factory.detect_hand()

    assert detection.model_name == "orcahand-right"
    assert any(
        "more than one hand is plugged in" in r.getMessage() for r in caplog.records
    )


def test_detect_hand_stays_quiet_for_a_single_board(monkeypatch, caplog):
    from orca_core import hand_factory

    monkeypatch.setattr(
        hand_factory, "oh_board_ports", lambda: ["/dev/cu.a1", "/dev/cu.a2"]
    )
    monkeypatch.setattr(hand_factory, "probe_orca_info", lambda port: None)
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda port: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)
    monkeypatch.setattr(hand_factory, "port_in_use", lambda port: False)

    with caplog.at_level(logging.WARNING):
        hand_factory.detect_hand()

    assert not any(
        "more than one hand" in r.getMessage() for r in caplog.records
    )
