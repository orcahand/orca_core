"""Interlocks that keep two attached hands from corrupting each other.

Each test here pins a failure that is silent without the guard: a hand
adopting its neighbour's bus, two hands writing one calibration file, or a
probe opening a port another client already owns.
"""

import logging
from types import SimpleNamespace

import pytest

from orca_core import MockOrcaHand, OrcaHand
from orca_core.constants import KNOWN_VIDS, PORT_BUSY_ERRNOS

from tests._helpers import fake_serial_port

_OH_VID = KNOWN_VIDS["oh_board"][0]
_BOARD_A = "1B46C9E850304C43552E3120FF061305" * 2
_BOARD_B = "0011223344556677889AABBCCDDEEFF0" * 2


def _oh_port(device: str, usb_serial: str):
    return fake_serial_port(device, _OH_VID, serial_number=usb_serial)


def _patch_two_boards(monkeypatch, hand_factory):
    """Two boards whose CDCs interleave in enumeration order, so pairing by
    first-seen role would cross them."""
    from orca_core.hardware.sensing import serial_discovery
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    ports = [
        _oh_port("/dev/cu.a-sensor", _BOARD_A),
        _oh_port("/dev/cu.b-motor", _BOARD_B),
        _oh_port("/dev/cu.a-motor", _BOARD_A),
        _oh_port("/dev/cu.b-sensor", _BOARD_B),
    ]
    infos = {
        "/dev/cu.a-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0001"),
        "/dev/cu.a-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
        "/dev/cu.b-motor": OrcaBoardInfo(role="motor", side="left", serial="ser-0002"),
        "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor", side="left", serial="ser-0002"),
    }
    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: ports)
    monkeypatch.setattr(
        serial_discovery, "probe_orca_info", lambda p, *a, **k: infos.get(p)
    )
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: False)
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)


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

def test_detect_hand_warns_that_it_is_describing_only_one_of_two(monkeypatch, caplog):
    """detect_hand() describes a single hand, so with two attached it must say
    which one the caller is getting rather than silently picking."""
    from orca_core import hand_factory

    _patch_two_boards(monkeypatch, hand_factory)

    with caplog.at_level(logging.WARNING):
        detection = hand_factory.detect_hand()

    assert detection.hand_id in {"ser-0001", "ser-0002"}
    assert any(
        "2 hands are plugged in" in r.getMessage() for r in caplog.records
    )


def test_detect_hand_stays_quiet_for_a_single_board(monkeypatch, caplog):
    from orca_core import hand_factory
    from orca_core.hardware.sensing import serial_discovery
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    ports = [
        _oh_port("/dev/cu.a1", _BOARD_A),
        _oh_port("/dev/cu.a2", _BOARD_A),
    ]
    infos = {
        "/dev/cu.a1": OrcaBoardInfo(role="motor", side="right", serial="ser-0001"),
        "/dev/cu.a2": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
    }
    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: ports)
    monkeypatch.setattr(
        serial_discovery, "probe_orca_info", lambda p, *a, **k: infos.get(p)
    )
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: False)
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)

    with caplog.at_level(logging.WARNING):
        hand_factory.detect_hand()

    assert not any(
        "hands are plugged in" in r.getMessage() for r in caplog.records
    )


def test_two_boards_pair_their_own_ports_despite_interleaved_enumeration(monkeypatch):
    """The failure this prevents: claiming the first motor CDC and the first
    sensing CDC seen, which with two boards drives one hand's motors from the
    other hand's encoders."""
    from orca_core import hand_factory

    _patch_two_boards(monkeypatch, hand_factory)

    hands = {d.hand_id: d for d in hand_factory.detect_hands()}
    assert set(hands) == {"ser-0001", "ser-0002"}
    assert (hands["ser-0001"].motor_port, hands["ser-0001"].sensing_port) == (
        "/dev/cu.a-motor", "/dev/cu.a-sensor",
    )
    assert (hands["ser-0002"].motor_port, hands["ser-0002"].sensing_port) == (
        "/dev/cu.b-motor", "/dev/cu.b-sensor",
    )
    assert hands["ser-0001"].side == "right"
    assert hands["ser-0002"].side == "left"


# ----- board grouping ------------------------------------------------------

def test_board_id_is_derived_from_the_usb_descriptor():
    """The firmware folds the same four MCU words into the ID it reports, so
    a board can be named without opening a port — or having the identity
    firmware at all. Values measured on a real board."""
    from orca_core.hardware.sensing.serial_discovery import board_id_from_usb_serial

    assert board_id_from_usb_serial(_BOARD_A) == "4B7685ABAA282225"
    # A descriptor carrying the ID once folds identically.
    assert board_id_from_usb_serial(_BOARD_A[:32]) == "4B7685ABAA282225"


@pytest.mark.parametrize(
    "usb_serial",
    [None, "", "not-hex-at-all", "ABCD", "1B46C9E850304C43552E3120FF0613050000"],
)
def test_board_id_is_none_for_a_descriptor_of_the_wrong_shape(usb_serial):
    from orca_core.hardware.sensing.serial_discovery import board_id_from_usb_serial

    assert board_id_from_usb_serial(usb_serial) is None


def test_halves_that_disagree_are_not_folded():
    """Two different IDs concatenated is not one board repeating itself."""
    from orca_core.hardware.sensing.serial_discovery import board_id_from_usb_serial

    assert board_id_from_usb_serial(_BOARD_A[:32] + _BOARD_B[:32]) is None


def test_cdcs_group_by_usb_serial_not_enumeration_order(monkeypatch):
    from orca_core.hardware.sensing.serial_discovery import (
        board_id_from_usb_serial, oh_board_candidates,
    )

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a1", _BOARD_A),
        _oh_port("/dev/cu.b1", _BOARD_B),
        _oh_port("/dev/cu.a2", _BOARD_A),
        _oh_port("/dev/cu.b2", _BOARD_B),
    ])
    grouped = {c.board_id: set(c.ports) for c in oh_board_candidates()}
    assert grouped == {
        board_id_from_usb_serial(_BOARD_A): {"/dev/cu.a1", "/dev/cu.a2"},
        board_id_from_usb_serial(_BOARD_B): {"/dev/cu.b1", "/dev/cu.b2"},
    }


def test_cdcs_without_a_serial_group_by_usb_location(monkeypatch):
    """Linux reports a per-interface location; the device prefix is the board."""
    from orca_core.hardware.sensing.serial_discovery import oh_board_candidates

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        fake_serial_port("/dev/ttyACM0", _OH_VID, location="1-3:1.0"),
        fake_serial_port("/dev/ttyACM1", _OH_VID, location="1-3:1.2"),
        fake_serial_port("/dev/ttyACM2", _OH_VID, location="1-4:1.0"),
        fake_serial_port("/dev/ttyACM3", _OH_VID, location="1-4:1.2"),
    ])
    grouped = [set(c.ports) for c in oh_board_candidates()]
    assert len(grouped) == 2
    assert {"/dev/ttyACM0", "/dev/ttyACM1"} in grouped
    assert {"/dev/ttyACM2", "/dev/ttyACM3"} in grouped


def test_unattributable_cdcs_beyond_one_board_are_never_merged(monkeypatch):
    """Merging CDCs that name neither a serial nor a location would pair one
    hand's motor bus with another hand's encoders."""
    from orca_core.hardware.sensing.serial_discovery import oh_board_candidates

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        fake_serial_port(f"/dev/ttyACM{i}", _OH_VID) for i in range(4)
    ])
    assert [len(c.ports) for c in oh_board_candidates()] == [1, 1, 1, 1]


def test_one_boards_worth_of_unattributable_cdcs_still_pairs(monkeypatch):
    """There is nothing to cross-pair with, so the legacy single-board case
    keeps working."""
    from orca_core.hardware.sensing.serial_discovery import oh_board_candidates

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        fake_serial_port("/dev/ttyACM0", _OH_VID),
        fake_serial_port("/dev/ttyACM1", _OH_VID),
    ])
    candidates = oh_board_candidates()
    assert len(candidates) == 1
    assert set(candidates[0].ports) == {"/dev/ttyACM0", "/dev/ttyACM1"}


def test_a_board_reply_contradicting_the_descriptor_is_reported(monkeypatch, caplog):
    """Both CDCs of a board answer with the same identity block, so a
    disagreement proves the grouping crossed two boards."""
    from orca_core.hardware.sensing import serial_discovery
    from orca_core.hardware.sensing.serial_discovery import (
        BoardCandidate, OrcaBoardInfo, probe_board,
    )

    monkeypatch.setattr(
        serial_discovery, "probe_orca_info",
        lambda p, *a, **k: OrcaBoardInfo(role="motor", board_id="DEADBEEFDEADBEEF"),
    )
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: False)

    with caplog.at_level(logging.ERROR):
        probe_board(BoardCandidate(ports=("/dev/cu.x",), usb_serial=_BOARD_A))

    assert any("not one board" in r.getMessage() for r in caplog.records)


def test_single_port_resolvers_refuse_to_pick_between_two_boards(monkeypatch):
    """find_motor_port() has no right answer with two hands attached, and
    returning either hands the caller a port from a hand it did not ask for."""
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a1", _BOARD_A),
        _oh_port("/dev/cu.a2", _BOARD_A),
        _oh_port("/dev/cu.b1", _BOARD_B),
        _oh_port("/dev/cu.b2", _BOARD_B),
    ])
    probed = []
    monkeypatch.setattr(
        serial_discovery, "_probe_orca_id",
        lambda p, **k: probed.append(p),
    )
    assert serial_discovery.find_motor_port() is None
    assert probed == [], "an ambiguous bus must not be probed at all"
