"""Interlocks that keep two attached hands from corrupting each other.

Each test here pins a failure that is silent without the guard: a hand
adopting its neighbour's bus, two hands writing one calibration file, or a
probe opening a port another client already owns.
"""

import logging
import pathlib
from types import SimpleNamespace

import pytest

import orca_core
from orca_core import MockOrcaHand, OrcaHand
from orca_core.constants import (
    KNOWN_VIDS,
    ORCA_ID_RESP_MOTOR,
    ORCA_ID_RESP_SENSOR,
    PORT_BUSY_ERRNOS,
)
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink

from tests._helpers import fake_serial_port

_MODELS = pathlib.Path(orca_core.__file__).parent / "models"

_OH_VID = KNOWN_VIDS["oh_board"][0]
_BOARD_A = "5EED1CE500FA11EDBEEFCAFE0DDBA11D" * 2
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

    resolved = SimpleNamespace(
        port="/dev/cu.the-other-hand",
        motor_type="dynamixel",
        baudrate=1_000_000,
        config_path=str(config_path),
    )
    motor_resolution.persist_resolved_driver(resolved)

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

    opened = 0

    class _SlowConnect(_RealCalibrationMockHand):
        def _connect_cascade(self, interactive):
            nonlocal opened
            time.sleep(0.15)
            opened += 1
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
        # The claim has to be taken before the bus is touched, so the loser
        # must have been refused without ever connecting. Counting outcomes
        # alone still holds when the claim is moved after the connect.
        connected = [h for h in hands if h.is_connected()]
        assert len(connected) == 1, "both hands reached the bus"
        assert opened == 1, f"the bus was opened {opened} times for one winner"
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

    measured = "1B46C9E850304C43552E3120FF061305" * 2
    assert board_id_from_usb_serial(measured) == "4B7685ABAA282225"
    # A descriptor carrying the ID once folds identically.
    assert board_id_from_usb_serial(measured[:32]) == "4B7685ABAA282225"


@pytest.mark.parametrize(
    "usb_serial",
    [None, "", "not-hex-at-all", "ABCD", "5EED1CE500FA11EDBEEFCAFE0DDBA11D0000"],
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


def _patch_probe(monkeypatch, replies):
    """Answer each port with a fixed OrcaBoardInfo."""
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr(
        serial_discovery, "probe_orca_info", lambda p, *a, **k: replies.get(p)
    )
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: False)


def test_the_descriptor_wins_over_a_contradicting_reply(monkeypatch, caplog):
    """One USB serial is one physical device, so a reply naming a different
    board is the fold and the firmware disagreeing. The descriptor is the one
    that reads the same whether or not the board answers."""
    from orca_core.hardware.sensing.serial_discovery import (
        BoardCandidate, OrcaBoardInfo, board_id_from_usb_serial, probe_board,
    )

    _patch_probe(monkeypatch, {
        "/dev/cu.x": OrcaBoardInfo(role="motor", board_id="DEADBEEFDEADBEEF"),
    })

    with caplog.at_level(logging.WARNING):
        probe = probe_board(BoardCandidate(ports=("/dev/cu.x",), usb_serial=_BOARD_A))

    assert probe.board_id == board_id_from_usb_serial(_BOARD_A)
    assert probe.motor_port == "/dev/cu.x"
    assert any("trusting the USB descriptor" in r.getMessage() for r in caplog.records)


def test_merged_cdcs_that_turn_out_to_be_two_boards_are_left_unpaired(
        monkeypatch, caplog):
    """The one grouping that can cross boards is the descriptor-less merge, so
    that is exactly where differing replies have to be caught."""
    from orca_core.hardware.sensing.serial_discovery import (
        BoardCandidate, OrcaBoardInfo, probe_board,
    )

    _patch_probe(monkeypatch, {
        "/dev/cu.x": OrcaBoardInfo(role="motor", board_id="AAAAAAAAAAAAAAAA"),
        "/dev/cu.y": OrcaBoardInfo(role="sensor", board_id="BBBBBBBBBBBBBBBB"),
    })

    with caplog.at_level(logging.ERROR):
        probe = probe_board(BoardCandidate(ports=("/dev/cu.x", "/dev/cu.y")))

    assert probe.motor_port is None
    assert probe.sensing_port is None
    assert any("separate boards" in r.getMessage() for r in caplog.records)


def test_a_spare_board_does_not_stop_a_lone_hand_resolving_its_ports(monkeypatch):
    """The controller board shares its vendor ID with the bare module, so a
    spare on the bench enumerates as a board but answers nothing."""
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.hand-motor", _BOARD_A),
        _oh_port("/dev/cu.hand-sensor", _BOARD_A),
        _oh_port("/dev/cu.spare", _BOARD_B),
    ])
    replies = {
        "/dev/cu.hand-motor": ORCA_ID_RESP_MOTOR,
        "/dev/cu.hand-sensor": ORCA_ID_RESP_SENSOR,
    }
    monkeypatch.setattr(
        serial_discovery, "_probe_orca_id", lambda p, **k: replies.get(p)
    )
    assert serial_discovery.find_motor_port() == "/dev/cu.hand-motor"


def test_a_board_that_answers_nothing_is_not_reported_as_a_hand(monkeypatch):
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.hand-motor", _BOARD_A),
        _oh_port("/dev/cu.hand-sensor", _BOARD_A),
        _oh_port("/dev/cu.spare", _BOARD_B),
    ])
    _patch_probe(monkeypatch, {
        "/dev/cu.hand-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0001"),
        "/dev/cu.hand-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
    })
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)

    detections = hand_factory.detect_hands()
    assert [d.hand_id for d in detections] == ["ser-0001"]


def test_one_sideless_hand_among_several_is_still_warned_about(monkeypatch, caplog):
    """A hand assumed right-handed gets the wrong joint map and encoder
    polarities; with several attached the user cannot fall back on knowing
    which hand it is."""
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a-motor", _BOARD_A),
        _oh_port("/dev/cu.a-sensor", _BOARD_A),
        _oh_port("/dev/cu.b-motor", _BOARD_B),
        _oh_port("/dev/cu.b-sensor", _BOARD_B),
    ])
    _patch_probe(monkeypatch, {
        "/dev/cu.a-motor": OrcaBoardInfo(role="motor", side="left", serial="ser-A"),
        "/dev/cu.a-sensor": OrcaBoardInfo(role="sensor", side="left", serial="ser-A"),
        "/dev/cu.b-motor": OrcaBoardInfo(role="motor"),
        "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor"),
    })
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)

    with caplog.at_level(logging.WARNING):
        detections = hand_factory.detect_hands()

    assert len(detections) == 2
    assert any(
        "1 of 2 attached hands report no side" in r.getMessage()
        for r in caplog.records
    )


def test_detection_order_does_not_change_when_a_hand_connects(monkeypatch):
    """A connected hand holds its ports and cannot be probed, so an order
    taken from replies would rearrange itself the moment a hand engages."""
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a-motor", _BOARD_A),
        _oh_port("/dev/cu.a-sensor", _BOARD_A),
        _oh_port("/dev/cu.b-motor", _BOARD_B),
        _oh_port("/dev/cu.b-sensor", _BOARD_B),
    ])
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)

    answering = {
        "/dev/cu.a-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0001"),
        "/dev/cu.a-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
        "/dev/cu.b-motor": OrcaBoardInfo(role="motor", side="left", serial="ser-0002"),
        "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor", side="left", serial="ser-0002"),
    }
    _patch_probe(monkeypatch, answering)
    before = [d.board_id for d in hand_factory.detect_hands()]

    # Hand A now holds its ports, so it answers nothing and only its
    # descriptor identifies it.
    from orca_core.hardware.sensing import serial_discovery

    silent = {k: v for k, v in answering.items() if not k.startswith("/dev/cu.a")}
    _patch_probe(monkeypatch, silent)
    monkeypatch.setattr(
        serial_discovery, "port_in_use", lambda p: p.startswith("/dev/cu.a")
    )
    after = [d.board_id for d in hand_factory.detect_hands()]

    assert before == after
    assert len(after) == 2


def test_single_port_resolvers_refuse_to_pick_between_two_boards(monkeypatch):
    """find_motor_port() has no right answer when two hands answer, and
    returning either hands the caller a port from a hand it did not ask for."""
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a1", _BOARD_A),
        _oh_port("/dev/cu.a2", _BOARD_A),
        _oh_port("/dev/cu.b1", _BOARD_B),
        _oh_port("/dev/cu.b2", _BOARD_B),
    ])
    replies = {
        "/dev/cu.a1": ORCA_ID_RESP_MOTOR, "/dev/cu.a2": ORCA_ID_RESP_SENSOR,
        "/dev/cu.b1": ORCA_ID_RESP_MOTOR, "/dev/cu.b2": ORCA_ID_RESP_SENSOR,
    }
    monkeypatch.setattr(
        serial_discovery, "_probe_orca_id", lambda p, **k: replies.get(p)
    )
    assert serial_discovery.find_motor_port() is None
    assert serial_discovery._find_oh_sensor_port() is None


# ----- the in-process port registry ----------------------------------------

class _Client:
    """Stand-in owner. Claims are weak, so an owner must be weakref-able —
    a bare object() is not."""


def test_a_second_client_cannot_claim_a_port_this_process_drives():
    """Advisory file locks catch another process. Inside one process the
    second open succeeds, and both clients then read each other's replies."""
    from orca_core.hardware import port_registry

    first, second = _Client(), _Client()
    port_registry.claim("/dev/cu.bus", first)
    with pytest.raises(port_registry.PortAlreadyClaimed, match="already open"):
        port_registry.claim("/dev/cu.bus", second)

    port_registry.release("/dev/cu.bus", first)
    port_registry.claim("/dev/cu.bus", second)


def test_reclaiming_the_same_port_is_idempotent():
    from orca_core.hardware import port_registry

    owner = _Client()
    port_registry.claim("/dev/cu.bus", owner)
    port_registry.claim("/dev/cu.bus", owner)
    assert port_registry.owner_of("/dev/cu.bus") is owner


def test_releasing_a_port_you_do_not_hold_is_harmless():
    from orca_core.hardware import port_registry

    owner, other = _Client(), _Client()
    port_registry.claim("/dev/cu.bus", owner)
    port_registry.release("/dev/cu.bus", other)
    assert port_registry.owner_of("/dev/cu.bus") is owner


def test_a_dropped_client_releases_its_port():
    """A client abandoned without disconnecting must not strand its bus for
    the life of the process."""
    import gc

    from orca_core.hardware import port_registry

    owner = _Client()
    port_registry.claim("/dev/cu.bus", owner)
    del owner
    gc.collect()
    assert port_registry.owner_of("/dev/cu.bus") is None


def test_two_names_for_one_device_are_the_same_claim(tmp_path):
    """Linux ships /dev/serial/by-id symlinks beside the ttyACM they point at,
    and a config may pin either."""
    from orca_core.hardware import port_registry

    real = tmp_path / "ttyACM0"
    real.write_bytes(b"")
    link = tmp_path / "by-id-orca"
    link.symlink_to(real)

    first, second = _Client(), _Client()
    port_registry.claim(str(real), first)
    with pytest.raises(port_registry.PortAlreadyClaimed):
        port_registry.claim(str(link), second)


class _RealPortLink(MockHandSerialLink):
    """Mock serial I/O, but the real link's port-ownership hooks — a stand-in
    for a link that does drive a bus."""

    _claim_port = HandSerialLink._claim_port
    _release_port = HandSerialLink._release_port


def test_a_serial_link_claims_its_port_and_releases_it_on_disconnect():
    """Two links demuxing one board's stream would each eat frames the other
    is waiting on."""
    from orca_core.hardware import port_registry

    first = _RealPortLink(port="/dev/cu.encoders")
    first.connect()
    assert port_registry.owner_of("/dev/cu.encoders") is first

    second = _RealPortLink(port="/dev/cu.encoders")
    with pytest.raises(port_registry.PortAlreadyClaimed):
        second.connect()

    first.disconnect()
    assert port_registry.owner_of("/dev/cu.encoders") is None
    second.disconnect()

    third = _RealPortLink(port="/dev/cu.encoders")
    third.connect()
    third.disconnect()


def test_two_mock_serial_links_share_a_port_name():
    """Mock links share no bus, so the port name a mock hand is pinned to must
    not stop a second one opening."""
    first = MockHandSerialLink(port="/dev/orca-mock")
    second = MockHandSerialLink(port="/dev/orca-mock")
    first.connect()
    second.connect()

    # Sharing the name is safe because the byte streams are separate.
    first.feed_bytes(b"\xAA\xA9payload")
    assert bytes(second._injected_buffer) == b""

    second.disconnect()
    first.disconnect()


def test_a_motor_client_releases_its_port_on_a_failed_connect(monkeypatch):
    """A refused connect must not leave the bus claimed against the retry."""
    from orca_core.hardware import port_registry
    from orca_core.hardware.dynamixel_client import DynamixelClient

    client = DynamixelClient([1], port="/dev/cu.nope", baudrate=1_000_000)
    monkeypatch.setattr(client.port_handler, "openPort", lambda: False)

    with pytest.raises(OSError):
        client.connect()
    assert port_registry.owner_of("/dev/cu.nope") is None


def test_detection_tells_this_process_apart_from_a_foreign_client(monkeypatch):
    """A hand this process already connected is silent under probing for a
    reason the caller can act on, unlike a port some other program holds."""
    from orca_core import hand_factory
    from orca_core.hardware import port_registry
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.mine", _BOARD_A),
        _oh_port("/dev/cu.theirs", _BOARD_A),
    ])
    monkeypatch.setattr(serial_discovery, "probe_orca_info", lambda p, *a, **k: None)
    # True for both, as a real exclusive open reports: pyserial locks per open
    # file description, so a port THIS process holds is busy to a probe too.
    # Only the registry can tell the two apart, which is what this pins.
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: True)
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: None)

    link = MockHandSerialLink(port="/dev/cu.mine")
    link._claim_port = lambda: port_registry.claim("/dev/cu.mine", link)
    link.connect()

    detections = hand_factory.detect_hands()
    assert len(detections) == 1
    assert detections[0].owned_ports == ("/dev/cu.mine",)
    assert detections[0].busy_ports == ("/dev/cu.theirs",)


# ----- a hand that cannot name its own bus ---------------------------------

def _patch_bus(monkeypatch, ports, infos, held=(), tactile_adapter=None):
    """A bus of controller-board CDCs, some of them held by another program.

    ``infos`` answers ``ORCA_INFO?``; a port in ``held`` answers nothing and
    reports busy, exactly as an exclusive open by another client looks.
    """
    from orca_core import hand_factory
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: ports)
    monkeypatch.setattr(
        serial_discovery, "probe_orca_info",
        lambda p, *a, **k: None if p in held else infos.get(p),
    )
    monkeypatch.setattr(serial_discovery, "port_in_use", lambda p: p in held)
    monkeypatch.setattr(serial_discovery, "_probe_orca_id", lambda p, **k: (
        None if p in held or infos.get(p) is None
        else {"motor": ORCA_ID_RESP_MOTOR, "sensor": ORCA_ID_RESP_SENSOR}[infos[p].role]
    ))
    monkeypatch.setattr(hand_factory, "detect_encoder_stream", lambda p: False)
    monkeypatch.setattr(hand_factory, "_tactile_responds_at", lambda p, b: False)
    monkeypatch.setattr(hand_factory, "find_tactile_port", lambda: tactile_adapter)


@pytest.mark.parametrize("silent, resolver, neighbour", [
    ("/dev/cu.a-motor", "find_motor_port", "/dev/cu.b-motor"),
    ("/dev/cu.a-sensor", "_find_oh_sensor_port", "/dev/cu.b-sensor"),
])
def test_a_hand_silent_on_one_cdc_still_counts_as_a_second_hand(
        monkeypatch, silent, resolver, neighbour):
    """Counting only the CDCs whose reply matched the requested role hides a
    hand whose other CDC is held, leaving the neighbour as the lone match."""
    from orca_core.hardware.sensing import serial_discovery

    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: [
        _oh_port("/dev/cu.a-motor", _BOARD_A),
        _oh_port("/dev/cu.a-sensor", _BOARD_A),
        _oh_port("/dev/cu.b-motor", _BOARD_B),
        _oh_port("/dev/cu.b-sensor", _BOARD_B),
    ])
    replies = {
        "/dev/cu.a-motor": ORCA_ID_RESP_MOTOR,
        "/dev/cu.a-sensor": ORCA_ID_RESP_SENSOR,
        "/dev/cu.b-motor": ORCA_ID_RESP_MOTOR,
        "/dev/cu.b-sensor": ORCA_ID_RESP_SENSOR,
    }
    del replies[silent]
    monkeypatch.setattr(
        serial_discovery, "_probe_orca_id", lambda p, **k: replies.get(p)
    )

    resolved = getattr(serial_discovery, resolver)()
    assert resolved is None, f"resolved onto the neighbouring hand ({neighbour})"


def test_a_hand_whose_ports_are_all_held_refuses_its_neighbours_motor_bus(
        monkeypatch, tmp_path):
    """The hand is named explicitly, so handing back the only bus that
    happens to answer would drive the arm the caller ruled out."""
    from orca_core import hand_factory
    from orca_core.hand_factory import HandPortUnresolvedError, HandSelector
    from orca_core.hardware.sensing import serial_discovery
    from orca_core.hardware.sensing.serial_discovery import (
        OrcaBoardInfo, board_id_from_usb_serial,
    )

    monkeypatch.setenv("ORCA_HOME", str(tmp_path))
    _patch_bus(
        monkeypatch,
        ports=[
            _oh_port("/dev/cu.a-motor", _BOARD_A),
            _oh_port("/dev/cu.a-sensor", _BOARD_A),
            _oh_port("/dev/cu.b-motor", _BOARD_B),
            _oh_port("/dev/cu.b-sensor", _BOARD_B),
        ],
        infos={
            "/dev/cu.b-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0002"),
            "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0002"),
        },
        held=("/dev/cu.a-motor", "/dev/cu.a-sensor"),
    )
    held_hand = board_id_from_usb_serial(_BOARD_A)

    # The bus search the refusal replaces would still land on the neighbour.
    assert serial_discovery.find_motor_port() == "/dev/cu.b-motor"

    with pytest.raises(HandPortUnresolvedError, match="held by another program"):
        hand_factory.load_hand(select=HandSelector(hand_id=held_hand))


def test_one_held_hand_does_not_withhold_the_other(monkeypatch, tmp_path, caplog):
    """A hand nobody can reach is a reason to skip it, not a reason to refuse
    the arm sitting free beside it."""
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setenv("ORCA_HOME", str(tmp_path))
    _patch_bus(
        monkeypatch,
        ports=[
            _oh_port("/dev/cu.a-motor", _BOARD_A),
            _oh_port("/dev/cu.a-sensor", _BOARD_A),
            _oh_port("/dev/cu.b-motor", _BOARD_B),
            _oh_port("/dev/cu.b-sensor", _BOARD_B),
        ],
        infos={
            "/dev/cu.b-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0002"),
            "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0002"),
        },
        held=("/dev/cu.a-motor", "/dev/cu.a-sensor"),
    )

    with caplog.at_level(logging.WARNING):
        hands = hand_factory.load_hands()

    assert [h.config.port for h in hands] == ["/dev/cu.b-motor"]
    assert any("Skipping it" in r.getMessage() for r in caplog.records)


def test_a_held_sensing_cdc_disables_the_link_instead_of_discovering_one(
        monkeypatch, tmp_path):
    """A declared sensing link that did not answer must not be auto-discovered
    with a neighbour attached: discovery opens the neighbour's CDC."""
    from orca_core import hand_factory
    from orca_core.hand_factory import HandSelector
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setenv("ORCA_HOME", str(tmp_path))
    _patch_bus(
        monkeypatch,
        ports=[
            _oh_port("/dev/cu.a-motor", _BOARD_A),
            _oh_port("/dev/cu.a-sensor", _BOARD_A),
            _oh_port("/dev/cu.b-motor", _BOARD_B),
            _oh_port("/dev/cu.b-sensor", _BOARD_B),
        ],
        infos={
            "/dev/cu.a-motor": OrcaBoardInfo(
                role="motor", side="right", serial="ser-0001", config=2500),
            "/dev/cu.b-motor": OrcaBoardInfo(role="motor", side="right", serial="ser-0002"),
            "/dev/cu.b-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0002"),
        },
        held=("/dev/cu.a-sensor",),
        tactile_adapter="/dev/cu.paxini",
    )

    hand = hand_factory.load_hand(select=HandSelector(hand_id="ser-0001"))

    assert hand.config.port == "/dev/cu.a-motor"
    assert hand.config.encoder_serial_port == "disabled"
    assert hand.config.sensor_port == "disabled", (
        "an unattributed tactile adapter was auto-discovered anyway"
    )


def test_a_lone_hand_with_a_transient_probe_miss_still_resolves_at_connect(
        monkeypatch, tmp_path):
    """With one hand attached a bus search can only find that hand again, and
    it is what recovers a CDC that missed its probe. Refusing here bricks it."""
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setenv("ORCA_HOME", str(tmp_path))
    _patch_bus(
        monkeypatch,
        ports=[
            _oh_port("/dev/cu.a-motor", _BOARD_A),
            _oh_port("/dev/cu.a-sensor", _BOARD_A),
        ],
        infos={
            "/dev/cu.a-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
        },
    )

    hand = hand_factory.load_hand()
    assert hand.config.port == "auto"


def test_a_spare_board_does_not_make_a_lone_hand_unresolvable(
        monkeypatch, tmp_path):
    """A bare module answers nothing, so it is not a second hand and must not
    turn a missed probe into a refusal."""
    from orca_core import hand_factory
    from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo

    monkeypatch.setenv("ORCA_HOME", str(tmp_path))
    _patch_bus(
        monkeypatch,
        ports=[
            _oh_port("/dev/cu.a-motor", _BOARD_A),
            _oh_port("/dev/cu.a-sensor", _BOARD_A),
            _oh_port("/dev/cu.spare", _BOARD_B),
        ],
        infos={
            "/dev/cu.a-sensor": OrcaBoardInfo(role="sensor", side="right", serial="ser-0001"),
        },
    )

    hand = hand_factory.load_hand()
    assert hand.config.port == "auto"


def test_a_hand_found_only_by_its_tactile_adapter_keeps_searching_for_motors(
        monkeypatch):
    """Its motor bus really is a plain USB adapter with no controller board
    behind it, so there is no detected port to pin and nothing to refuse."""
    from orca_core.hand_config import OrcaHandConfig
    from orca_core.hand_factory import HandDetection, _pin_detected_ports

    detection = HandDetection(
        model_name="orcahand-touch-right", side="right",
        has_tactile=True, has_encoders=False, tactile_port="/dev/cu.paxini",
    )
    config = OrcaHandConfig.from_config_path(
        config_path=str(_MODELS / "v2" / "orcahand-right" / "config.yaml"),
        calibration_path=None,
    )

    assert _pin_detected_ports(config, detection, attached=2).port == "auto"
