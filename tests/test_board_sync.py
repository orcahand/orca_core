"""The calibration kept on a hand's own board, and how it meets this computer's.

The rule: on first contact, a computer with no calibration for the hand copies
the board's; after that the computer's copy wins. A newer board calibration is
offered (board_calibration_newer), never taken silently.
"""

import logging
import os
import sys

import pytest

import orca_core
from orca_core import (
    MockOrcaHand, OrcaHand, board_push, board_store, board_store_link, board_sync, hand_store,
)
from orca_core.board_store import BoardIdentity, pack
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
from orca_core.utils import read_yaml
from tests._helpers import copy_model_config

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right")
INFO = OrcaBoardInfo(role="motor", side="right", hw_version=210, serial="ser-0000",
                     board_id="0123456789ABCDEF")


class BoardedHand(MockOrcaHand):
    """Mock motors, but identity and board resolved exactly as a physical hand's are."""

    contact = board_sync.NO_BOARD
    _settle_identity = OrcaHand._settle_identity
    _settle_identity_at_construction = OrcaHand._settle_identity_at_construction

    def _contact_board(self, port):
        return type(self).contact


def board_blob(config, *, calibrated_at="2026-11-10T08:00:00Z", sequence=3, limit=0.4,
               ratios=None, joint_ranges=None):
    """What a board calibrated elsewhere holds for this hand."""
    from orca_core.calibration import joint_fingerprints

    limits = {m: [0.1, 0.1 + limit] for m in config.motor_ids}
    return pack({
        "calibrated": True, "wrist_calibrated": True,
        "motor_limits": limits,
        "joint_to_motor_ratios": ratios or {m: 99.0 for m in config.motor_ids},
        "sequence": sequence, "calibrated_at": calibrated_at,
        "fingerprint": joint_fingerprints(config),
    }, identity=BoardIdentity(side="right", hw_version=210, serial="ser-0000"), sequence=9,
        joint_ranges=joint_ranges)


@pytest.fixture
def model_dir(tmp_path):
    folder = tmp_path / "orcahand-right"
    folder.mkdir()
    copy_model_config(os.path.join(MODEL, "config.yaml"), folder / "config.yaml")
    return folder


@pytest.fixture
def connect(model_dir, monkeypatch):
    def connect(store=None):
        monkeypatch.setattr(BoardedHand, "contact", board_sync.BoardContact(info=INFO, store=store))
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"))
        assert hand.connect()[0]
        return hand
    return connect


@pytest.fixture
def config(model_dir):
    hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
    hand.config = hand.config.with_hardware_version(210)
    return hand.config


class TestFirstContact:
    def test_a_computer_with_nothing_for_the_hand_copies_the_board_s(self, connect, config):
        hand = connect(board_blob(config))
        doc = read_yaml(hand.config.calibration_path)
        assert doc["source"] == "board"
        assert doc["sequence"] == 3 and doc["calibrated_at"] == "2026-11-10T08:00:00Z"
        assert "joint_to_motor_ratios" not in doc
        assert hand.calibrated
        assert hand.board_calibration_newer() is None

    def test_the_board_s_ratios_are_never_trusted(self, connect, config):
        hand = connect(board_blob(config))
        motor = hand.config.joint_to_motor_map["index_mcp"]
        lo, hi = hand.config.joint_roms_dict["index_mcp"]
        assert hand.joint_to_motor_ratios_dict[motor] == pytest.approx(0.4 / (hi - lo))

    def test_the_store_is_kept_beside_the_hand_s_files(self, connect, config):
        blob = board_blob(config)
        hand = connect(blob)
        assert (hand_store.hand_dir("ser-0000") / "board.bin").read_bytes() == blob

    def test_afterwards_the_computer_s_copy_wins(self, connect, config):
        first = connect(board_blob(config))
        first.calibrate(persist=True)
        mine = dict(first.motor_limits_dict)
        again = connect(board_blob(config, calibrated_at="2026-11-12T08:00:00Z", limit=0.9))
        assert again.motor_limits_dict == mine

    def test_a_board_with_no_store_changes_nothing(self, connect):
        hand = connect(None)
        assert not os.path.exists(hand.config.calibration_path)
        assert hand.board_calibration_newer() is None

    def test_a_store_this_version_cannot_read_is_not_used(self, connect, caplog):
        hand = connect(b"OHC1" + b"\x00" * 40)
        assert not os.path.exists(hand.config.calibration_path)
        assert "cannot be read" in caplog.text

    def test_a_store_with_a_valid_crc_but_a_short_section_is_not_used(self, connect, caplog):
        from tests.test_board_store import _store

        hand = connect(_store((4, 1, b"\x01\x00\x00")))
        assert hand.is_connected()
        assert hand.board_calibration_newer() is None
        assert not os.path.exists(hand.config.calibration_path)
        assert "cannot be read" in caplog.text


class TestTheBanner:
    def computer_calibrated(self, connect, config, board_at):
        hand = connect(board_blob(config, calibrated_at="2026-11-01T00:00:00Z"))
        hand.calibrate(persist=True)          # the computer's now, sequence 4
        return connect(board_blob(config, calibrated_at=board_at, limit=0.9))

    def test_a_newer_board_calibration_is_reported_with_both_stamps(self, connect, config):
        hand = self.computer_calibrated(connect, config, "2099-01-01T00:00:00Z")
        newer = hand.board_calibration_newer()
        assert newer["board"] == {"sequence": 3, "calibrated_at": "2099-01-01T00:00:00Z"}
        assert newer["computer"]["sequence"] == 4
        assert newer["computer"]["calibrated_at"] == hand.calibration_info["calibrated_at"]

    def test_an_older_board_calibration_is_not(self, connect, config):
        hand = self.computer_calibrated(connect, config, "2020-01-01T00:00:00Z")
        assert hand.board_calibration_newer() is None

    def test_it_never_talks_to_the_board(self, connect, config, monkeypatch):
        hand = self.computer_calibrated(connect, config, "2099-01-01T00:00:00Z")
        import serial

        monkeypatch.setattr(serial, "Serial", lambda *a, **k: pytest.fail("opened a port"))
        monkeypatch.setattr(board_sync, "contact_board", lambda *a, **k: pytest.fail("asked"))
        assert hand.board_calibration_newer() is not None
        assert hand.board_calibration_newer() is not None

    def test_adopting_makes_the_board_s_calibration_this_computer_s(self, connect, config):
        hand = self.computer_calibrated(connect, config, "2099-01-01T00:00:00Z")
        before = hand.calibration_info
        info = hand.adopt_board_calibration()
        assert info["calibrated_at"] == "2099-01-01T00:00:00Z"
        assert info["sequence"] == before["sequence"] + 1
        assert info["previous"] == {"sequence": before["sequence"],
                                    "calibrated_at": before["calibrated_at"]}
        motor = hand.config.motor_ids[0]
        assert hand.motor_limits_dict[motor] == pytest.approx([0.1, 1.0])
        assert hand.board_calibration_newer() is None
        # And it can be taken back.
        hand.restore_previous_calibration()
        assert hand.calibration_info["calibrated_at"] == before["calibrated_at"]

    def test_adopting_keeps_this_computer_s_tactile_zero(self, connect, config):
        hand = self.computer_calibrated(connect, config, "2099-01-01T00:00:00Z")
        path = hand.config.calibration_path
        doc = read_yaml(path)
        doc["sensor_offsets"] = {"thumb": [[1.0, 2.0, 3.0]]}
        hand_store.write_doc(path, doc)
        hand.adopt_board_calibration()
        assert read_yaml(path)["sensor_offsets"] == {"thumb": [[1.0, 2.0, 3.0]]}
        assert "drive_current_measured" not in read_yaml(path), "the calibration is the board's"

    def test_adopting_over_a_stopped_run_keeps_the_last_complete_one_as_previous(
            self, connect, config):
        hand = self.computer_calibrated(connect, config, "2099-01-01T00:00:00Z")
        complete = hand.calibration_info

        def stop_after_a_step(event):
            if event["event"] == "step_done":
                hand._task_stop_event.set()

        hand.calibrate(persist=True, progress_callback=stop_after_a_step)
        hand._task_stop_event.clear()
        assert read_yaml(hand.config.calibration_path)["in_progress"] is True
        info = hand.adopt_board_calibration()
        assert info["previous"] == {"sequence": complete["sequence"],
                                    "calibrated_at": complete["calibrated_at"]}

    def test_adopting_with_nothing_on_the_board_is_refused(self, connect):
        with pytest.raises(RuntimeError):
            connect(None).adopt_board_calibration()


class TestKeepingTheComputersCalibration:
    """'Don't ask again until a calibration changes' saves the pair; a change on
    either side brings the banner back."""

    def kept(self, connect, config):
        hand = connect(board_blob(config, calibrated_at="2026-11-01T00:00:00Z"))
        hand.calibrate(persist=True)
        hand = connect(board_blob(config, calibrated_at="2099-01-01T00:00:00Z"))
        pair = hand.board_calibration_newer()
        assert hand.keep_computer_calibration() == pair
        assert hand_store.read_hand(hand.hand_dir)["board_calibration_kept"] == pair
        return hand

    def test_the_banner_stays_away_across_connects_while_nothing_changes(self, connect, config):
        self.kept(connect, config)
        again = connect(board_blob(config, calibrated_at="2099-01-01T00:00:00Z"))
        assert again.board_calibration_newer() is None

    def test_a_new_board_calibration_brings_it_back(self, connect, config):
        self.kept(connect, config)
        again = connect(board_blob(config, calibrated_at="2099-06-01T00:00:00Z", sequence=5))
        assert again.board_calibration_newer()["board"]["calibrated_at"] == "2099-06-01T00:00:00Z"

    def test_a_new_computer_calibration_brings_it_back(self, connect, config, monkeypatch):
        hand = self.kept(connect, config)
        # A calibration here that is still older than the board's.
        monkeypatch.setattr(hand_store, "now_iso", lambda: "2098-01-01T00:00:00Z")
        monkeypatch.setattr("orca_core.calibration_store.now_iso", lambda: "2098-01-01T00:00:00Z")
        hand.calibrate(persist=True, joints=["index_mcp"])
        newer = hand.board_calibration_newer()
        assert newer is not None
        assert newer["computer"]["calibrated_at"] == "2098-01-01T00:00:00Z"

    def test_both_sides_changing_brings_it_back(self, connect, config, monkeypatch):
        hand = self.kept(connect, config)
        monkeypatch.setattr("orca_core.calibration_store.now_iso", lambda: "2098-01-01T00:00:00Z")
        hand.calibrate(persist=True, joints=["index_mcp"])
        again = connect(board_blob(config, calibrated_at="2099-06-01T00:00:00Z", sequence=5))
        newer = again.board_calibration_newer()
        assert newer["board"]["calibrated_at"] == "2099-06-01T00:00:00Z"
        assert newer["computer"]["calibrated_at"] == "2098-01-01T00:00:00Z"

    def test_with_no_board_calibration_there_is_nothing_to_keep(self, connect):
        assert connect(None).keep_computer_calibration() is None


BOARD_RANGES = {"measured_at": "2026-11-20T09:30:00Z", "source": "camera rig, run2",
                "joint_roms": {"index_mcp": [-12.5, 91.25], "wrist": [-40.0, 30.0]}}


class TestTheBoardsJointRanges:
    """Ranges measured on the hand that its board holds are in force while this
    computer has no joint_ranges.yaml for it, and are never written to one."""

    def test_they_are_in_force_and_said_to_come_from_the_board(self, connect, config):
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand.config.joint_roms_dict["wrist"] == [-40.0, 30.0]
        assert hand.joint_ranges_info == {"source": "measured", "measured_at": "2026-11-20T09:30:00Z",
                                          "joints": ["index_mcp", "wrist"],
                                          "stored_on": "board", "path": None}

    def test_they_are_not_copied_into_a_file(self, connect, config):
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        assert not os.path.exists(os.path.join(hand.hand_dir, "joint_ranges.yaml"))
        assert "joint_ranges" not in read_yaml(hand.config.calibration_path)

    def test_the_ratio_is_fitted_to_them(self, connect, config):
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        motor = hand.config.joint_to_motor_map["index_mcp"]
        assert hand.joint_to_motor_ratios_dict[motor] == pytest.approx(0.4 / (91.25 + 12.5))

    def test_this_computer_s_file_wins(self, connect, config):
        hand_store.write_joint_ranges(
            hand_store.open_hand_folder("ser-0000")[0], {"index_pip": [0.0, 95.0]},
            source="camera rig", measured_at="2026-11-21T00:00:00Z", joint_ids=config.joint_ids)
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        model = config.joint_roms_dict["index_mcp"]
        assert hand.config.joint_roms_dict["index_pip"] == [0.0, 95.0]
        assert hand.config.joint_roms_dict["index_mcp"] == model
        assert hand.joint_ranges_info == {
            "source": "measured", "measured_at": "2026-11-21T00:00:00Z", "joints": ["index_pip"],
            "stored_on": "file", "path": os.path.join(hand.hand_dir, "joint_ranges.yaml")}

    def test_a_file_with_no_range_to_apply_leaves_the_board_s_in_force(self, connect, config):
        hand_store.write_doc(hand_store.open_hand_folder("ser-0000")[0] / "joint_ranges.yaml",
                             {"joint_roms": {"index_pip": [5.0, 1.0], "not_a_joint": [0, 1]}})
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand.joint_ranges_info["stored_on"] == "board"

    def test_a_board_without_them_leaves_the_model_s(self, connect, config):
        hand = connect(board_blob(config))
        assert hand.joint_ranges_info["source"] == "model"
        assert hand.config.joint_roms_dict["index_mcp"] == config.joint_roms_dict["index_mcp"]

    def test_a_board_that_no_longer_holds_them_takes_them_out_of_force(self, connect, config):
        connect(board_blob(config, joint_ranges=BOARD_RANGES))
        hand = connect(board_blob(config))
        assert hand.joint_ranges_info["source"] == "model"
        assert hand.config.joint_roms_dict["index_mcp"] == config.joint_roms_dict["index_mcp"]

    def test_a_range_this_hand_cannot_apply_is_ignored(self, connect, config, caplog):
        odd = {"measured_at": None, "source": None,
               "joint_roms": {"index_mcp": [-12.5, 91.25], "index_pip": [5.0, 1.0],
                              "not_a_joint": [0.0, 1.0]}}
        with caplog.at_level(logging.DEBUG, logger="orca_core.hand_store"):
            hand = connect(board_blob(config, joint_ranges=odd))
        assert hand.joint_ranges_info["joints"] == ["index_mcp"]
        said = {r.getMessage().split("=")[0]: r.levelno for r in caplog.records
                if r.getMessage().startswith("ignoring joint range")}
        assert said == {"ignoring joint range index_pip": logging.WARNING,
                        "ignoring joint range not_a_joint": logging.DEBUG}

    def test_a_hardware_version_change_keeps_them(self, connect, config):
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        hand.set_hardware_version(211)
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand.joint_ranges_info["stored_on"] == "board"

    def test_a_store_that_could_not_be_read_now_keeps_the_last_read_copy_s(
            self, connect, config, model_dir, monkeypatch):
        connect(board_blob(config, joint_ranges=BOARD_RANGES))
        monkeypatch.setattr(BoardedHand, "contact",
                            board_sync.BoardContact(info=INFO, store_unread=True))
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"))
        assert hand.connect()[0]
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand.joint_ranges_info["stored_on"] == "board"

    def test_a_store_this_version_cannot_decode_keeps_the_last_read_copy_s(self, connect, config):
        connect(board_blob(config, joint_ranges=BOARD_RANGES))
        hand = connect(b"OHC1" + b"\x00" * 40)
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]

    def test_a_board_that_says_it_holds_no_store_has_no_ranges(self, connect, config):
        connect(board_blob(config, joint_ranges=BOARD_RANGES))
        hand = connect(None)
        assert hand.joint_ranges_info["source"] == "model"

    def test_a_stored_hand_takes_them_from_the_store_last_read(self, connect, config, model_dir):
        connect(board_blob(config, joint_ranges=BOARD_RANGES))
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        hand._use_stored_hand("ser-0000")
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand.joint_ranges_info["stored_on"] == "board"
        assert hand._board_store.joint_ranges == BOARD_RANGES, "store and ranges agree"

    def test_a_hand_whose_board_was_asked_keeps_what_it_said(self, connect, config):
        hand = connect(board_blob(config, joint_ranges=BOARD_RANGES))
        (hand_store.hand_dir("ser-0000") / "board.bin").write_bytes(board_blob(config))
        hand._use_stored_hand("ser-0000")
        assert hand.config.joint_roms_dict["index_mcp"] == [-12.5, 91.25]
        assert hand._board_store.joint_ranges == BOARD_RANGES

    def test_a_mock_hand_drops_what_a_board_held(self, model_dir):
        hand = MockOrcaHand(config_path=str(model_dir / "config.yaml"))
        hand._board_store = board_store.unpack(board_blob(hand.config, joint_ranges=BOARD_RANGES))
        hand._board_joint_ranges = BOARD_RANGES
        hand._settle_identity(board_sync.NO_BOARD)
        assert hand._board_store is None and hand._board_joint_ranges is None
        assert hand.joint_ranges_info["source"] == "model"


class TestReadingTheBoardAtConnect:
    """An unchanged store costs one question, not a full read."""

    @pytest.fixture
    def port(self, monkeypatch):
        from tests.test_board_store_link import FakeBoard

        fake = FakeBoard(None)

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                pass

            def __enter__(self):
                return fake

            def __exit__(self, *exc):
                return False

        import serial

        monkeypatch.setattr(serial, "Serial", FakeSerial)
        return fake

    def test_a_changed_store_is_read_whole(self, port, config):
        port.store = board_blob(config)
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX", cached_store=lambda info: None)
        assert contact.store == port.store and contact.info.serial == "ser-0000"
        assert any(q.startswith(b"ORCA_CAL?0") for q in port.asked)

    def test_a_store_matching_the_cache_is_not_read_again(self, port, config):
        port.store = board_blob(config)
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX",
                                           cached_store=lambda info: port.store)
        assert contact.store == port.store
        assert not any(q.startswith(b"ORCA_CAL?0") for q in port.asked)

    def test_a_stale_cache_is_replaced_by_a_full_read(self, port, config):
        port.store = board_blob(config, sequence=5)
        stale = board_blob(config, sequence=4)
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX", cached_store=lambda info: stale)
        assert contact.store == port.store

    def test_a_lost_store_reply_at_first_contact_is_asked_again(self, port, config, monkeypatch):
        monkeypatch.setattr(board_store_link, "METADATA_TIMEOUT_S", 0.01)
        port.store = board_blob(config)
        lost = {"left": 1}
        write = type(port).write

        def lose_the_first_store_question(data):
            if bytes(data).strip() == b"ORCA_CAL?" and lost["left"]:
                lost["left"] -= 1
                return
            write(port, data)

        port.write = lose_the_first_store_question
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX").store == port.store

    def test_a_board_without_a_store_still_names_its_hand(self, port):
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX")
        assert contact.info.serial == "ser-0000" and contact.store is None
        assert not contact.store_unread

    def test_a_store_read_that_fails_is_told_from_no_store(self, port, config, monkeypatch):
        port.store = board_blob(config)

        def times_out(link, metadata):
            raise TimeoutError("no reply")

        monkeypatch.setattr(board_store_link, "read_store_over_link", times_out)
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX", cached_store=lambda info: None)
        assert contact.info.serial == "ser-0000"
        assert contact.store is None and contact.store_unread


class TestWritingTheBoard:
    @pytest.fixture
    def board(self, tmp_path, monkeypatch, config):
        """The board behind the port: what the firmware tool last wrote, once it has."""
        from tests.test_board_store_link import FakeBoard

        sent = tmp_path / "sent.bin"

        class Board(FakeBoard):
            def write(self, data):
                if sent.exists() and self.serves_what_was_sent:
                    self.store = sent.read_bytes()
                super().write(data)

        fake = Board(pack({"calibrated": True, "motor_limits": {1: [0.0, 1.0]}},
                          identity=BoardIdentity(side="right", serial="ser-0000"), sequence=6))
        fake.serves_what_was_sent = True

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                pass

            def __enter__(self):
                return fake

            def __exit__(self, *exc):
                return False

        import serial

        monkeypatch.setattr(serial, "Serial", FakeSerial)
        monkeypatch.setattr(board_store_link, "METADATA_TIMEOUT_S", 0.01)
        monkeypatch.setattr(board_store_link, "LINE_TIMEOUT_S", 0.01)
        return fake

    @pytest.fixture
    def firmware(self, tmp_path, monkeypatch, board):
        repo = tmp_path / "orca_firmware"
        tool = repo / "tools" / "bench" / "cal_store.py"
        tool.parent.mkdir(parents=True)
        log = tmp_path / "tool.log"
        tool.write_text(
            "import sys, pathlib\n"
            f"pathlib.Path({str(log)!r}).write_text(' '.join(sys.argv[1:]))\n"
            "d = sys.argv[sys.argv.index('--dir') + 1]\n"
            "b = sys.argv[sys.argv.index('--board-id') + 1]\n"
            f"pathlib.Path({str(tmp_path / 'sent.bin')!r}).write_bytes("
            "(pathlib.Path(d) / (b + '.bin')).read_bytes())\n"
            "sys.exit(int(pathlib.Path(__file__).with_name('rc').read_text() or 0)"
            " if pathlib.Path(__file__).with_name('rc').exists() else 0)\n")
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(repo))
        return tmp_path

    def calibrated(self, connect, config):
        hand = connect(board_blob(config))
        hand.calibrate(persist=True)
        hand.disconnect()
        return hand

    def test_the_hand_s_calibration_goes_onto_its_board(self, connect, config, firmware):
        hand = self.calibrated(connect, config)
        result = board_push.push_for_hand(hand)
        assert result.written and result.sequence == 7
        sent = board_store.unpack((firmware / "sent.bin").read_bytes())
        doc = read_yaml(hand.config.calibration_path)
        assert sent.identity.serial == "ser-0000", "the board keeps its identity"
        assert sent.calibration["motor_limits"] == doc["motor_limits"]
        assert sent.calibration["calibrated_at"] == doc["calibrated_at"]
        assert sent.calibration["fingerprint"] == doc["fingerprint"]
        assert sent.calibration["joint_to_motor_ratios"], "older readers still get ratios"
        # What the board now holds is remembered, so no banner for our own write.
        assert (hand_store.hand_dir("ser-0000") / "board.bin").read_bytes() == \
            (firmware / "sent.bin").read_bytes()
        assert hand.board_calibration_newer() is None

    RIG_RANGES = {"measured_at": "2026-11-20T09:30:00Z", "source": "camera rig, run2",
                  "joint_roms": {"index_mcp": [-12.5, 91.25], "wrist": [-40.0, 30.0]}}

    def test_the_hand_s_measured_ranges_go_onto_its_board(self, connect, config, firmware):
        hand = self.calibrated(connect, config)
        hand_store.write_joint_ranges(hand.hand_dir, self.RIG_RANGES["joint_roms"],
                                      source=self.RIG_RANGES["source"],
                                      measured_at=self.RIG_RANGES["measured_at"],
                                      joint_ids=hand.config.joint_ids)
        assert board_push.push_for_hand(hand).written
        sent = board_store.unpack((firmware / "sent.bin").read_bytes())
        assert sent.joint_ranges == self.RIG_RANGES
        assert "joint_ranges" not in sent.calibration

    def test_a_recalibration_keeps_the_ranges_the_board_holds(self, connect, config, firmware,
                                                              board):
        hand = self.calibrated(connect, config)
        board.store = pack({"calibrated": True, "motor_limits": {1: [0.0, 1.0]}},
                           identity=BoardIdentity(side="right", serial="ser-0000"),
                           sequence=6, joint_ranges=self.RIG_RANGES)
        assert not os.path.exists(os.path.join(hand.hand_dir, "joint_ranges.yaml"))
        assert board_push.push_for_hand(hand).written
        sent = board_store.unpack((firmware / "sent.bin").read_bytes())
        assert sent.joint_ranges == self.RIG_RANGES
        assert sent.calibration["motor_limits"] == read_yaml(hand.config.calibration_path)["motor_limits"]

    def test_a_store_that_cannot_be_read_is_not_written_over(self, connect, config, firmware,
                                                            board, monkeypatch):
        """Its ranges could not be carried forward, so the write would erase them."""
        hand = self.calibrated(connect, config)
        board.store = held = pack({"calibrated": True, "motor_limits": {1: [0.0, 1.0]}},
                                  identity=BoardIdentity(side="right", serial="ser-0000"),
                                  sequence=6, joint_ranges=self.RIG_RANGES)

        def times_out(link, metadata):
            raise TimeoutError("no reply")

        monkeypatch.setattr(board_store_link, "read_store_over_link", times_out)
        result = board_push.push_for_hand(hand)
        assert not result.written and "could not be read" in result.reason
        assert not (firmware / "tool.log").exists()
        assert board.store == held

    @pytest.mark.parametrize("measured_at", ["2026-11-20T09:30:00Z", "2026-11-22T00:00:00Z"])
    def test_the_hand_s_file_replaces_the_board_s_ranges_of_the_same_date_or_older(
            self, connect, config, firmware, board, measured_at):
        hand = self.calibrated(connect, config)
        board.store = pack({}, identity=BoardIdentity(side="right", serial="ser-0000"),
                           sequence=6, joint_ranges=self.RIG_RANGES)
        hand_store.write_joint_ranges(hand.hand_dir, {"index_pip": [0.0, 95.0]},
                                      source="camera rig, run3", measured_at=measured_at,
                                      joint_ids=hand.config.joint_ids)
        assert board_push.push_for_hand(hand).written
        sent = board_store.unpack((firmware / "sent.bin").read_bytes())
        assert sent.joint_ranges["joint_roms"] == {"index_pip": [0.0, 95.0]}
        assert sent.joint_ranges["source"] == "camera rig, run3"

    def test_the_hand_s_file_never_replaces_newer_ranges_on_the_board(self, connect, config,
                                                                      firmware, board):
        """Provisional, until the merge of file and board ranges is decided."""
        hand = self.calibrated(connect, config)
        board.store = held = pack({}, identity=BoardIdentity(side="right", serial="ser-0000"),
                                  sequence=6, joint_ranges=self.RIG_RANGES)
        hand_store.write_joint_ranges(hand.hand_dir, {"index_pip": [0.0, 95.0]},
                                      source="camera rig, run1", measured_at="2026-11-19T00:00:00Z",
                                      joint_ids=hand.config.joint_ids)
        result = board_push.push_for_hand(hand)
        assert not result.written
        assert "2026-11-20T09:30:00Z" in result.reason and "2026-11-19T00:00:00Z" in result.reason
        assert not (firmware / "tool.log").exists()
        assert board.store == held

    def test_with_ranges_nowhere_the_board_holds_none(self, connect, config, firmware):
        hand = self.calibrated(connect, config)
        assert board_push.push_for_hand(hand).written
        assert board_store.unpack((firmware / "sent.bin").read_bytes()).joint_ranges is None

    def test_without_a_firmware_checkout_nothing_is_written(self, connect, config, monkeypatch, tmp_path):
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(tmp_path / "nowhere"))
        result = board_push.push_for_hand(self.calibrated(connect, config))
        assert not result.written and "no orca_firmware checkout at" in result.reason
        assert str(tmp_path / "nowhere") in result.reason

    def test_a_checkout_without_the_store_tool_is_named_as_such(self, connect, config, monkeypatch, tmp_path):
        (tmp_path / "orca_firmware" / "tools" / "bench").mkdir(parents=True)
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(tmp_path / "orca_firmware"))
        result = board_push.push_for_hand(self.calibrated(connect, config))
        assert not result.written
        assert result.reason == (f"the orca_firmware checkout at {tmp_path / 'orca_firmware'} "
                                 "has no tools/bench/cal_store.py")

    def test_a_tool_failure_is_reported_not_raised(self, connect, config, firmware):
        (firmware / "orca_firmware" / "tools" / "bench" / "rc").write_text("3")
        result = board_push.push_for_hand(self.calibrated(connect, config))
        assert not result.written and result.reason

    def test_a_lost_reply_is_asked_again_and_the_write_beats_the_board_s_sequence(
            self, connect, config, firmware, board):
        hand = self.calibrated(connect, config)
        lost = {"left": 1}
        write = type(board).write

        def lose_the_first_store_question(data):
            if bytes(data).strip() == b"ORCA_CAL?" and lost["left"]:
                lost["left"] -= 1
                return
            write(board, data)

        board.write = lose_the_first_store_question
        result = board_push.push_for_hand(hand)
        assert result.written and result.sequence == 7

    @pytest.mark.parametrize("silent_to", [b"ORCA_CAL?", b"ORCA_INFO?"])
    def test_a_board_that_will_not_say_what_it_holds_or_who_it_is_is_not_written(
            self, connect, config, firmware, board, silent_to):
        hand = self.calibrated(connect, config)
        if silent_to == b"ORCA_INFO?":
            board.answers_info = False
            board.store = b"OHC1" + b"\x00" * 40       # unreadable: no identity in it
        write = type(board).write
        board.write = lambda data: None if bytes(data).strip() == silent_to else write(board, data)
        result = board_push.push_for_hand(hand)
        assert not result.written and result.reason
        assert not (firmware / "tool.log").exists()

    def test_a_write_the_board_does_not_serve_is_not_reported_written(
            self, connect, config, firmware, board):
        hand = self.calibrated(connect, config)
        board.serves_what_was_sent = False
        result = board_push.push_for_hand(hand)
        assert not result.written
        cache = hand_store.hand_dir("ser-0000") / "board.bin"
        assert cache.read_bytes() != (firmware / "sent.bin").read_bytes()

    def test_a_hand_without_a_board_is_skipped(self, model_dir, firmware, monkeypatch):
        monkeypatch.setattr(BoardedHand, "contact", board_sync.NO_BOARD)
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"))
        hand.connect()
        result = board_push.push_for_hand(hand)
        assert not result.written and "no board" in result.reason
        assert not (firmware / "tool.log").exists()


class TestAHandWhoseCalibrationABoardCannotHold:
    def test_it_is_said_before_the_run(self, connect, capsys):
        from orca_core.utils import cli

        from types import SimpleNamespace

        hand = connect(None)
        assert cli.board_store_problem(hand) is None
        odd = SimpleNamespace(board_hand_id="ser-0000",
                              config=SimpleNamespace(joint_ids=["index_mcp"], motor_ids=[1, 300]))
        assert "0-255" in cli.board_store_problem(odd)
        assert "this computer only" in capsys.readouterr().out
        odd.board_hand_id = None
        assert cli.board_store_problem(odd) is None, "no board, nothing to say"

    def test_a_push_that_fails_never_ends_the_run_in_a_traceback(self, connect, capsys,
                                                                  monkeypatch):
        from orca_core.utils import cli

        def broken(hand):
            raise board_store.BoardStoreError("joint name 'é' is not plain ASCII")

        monkeypatch.setattr(board_push, "push_for_hand", broken)
        assert cli.push_calibration_to_board(connect(None)) is False
        assert "this computer only" in capsys.readouterr().out


def test_calibrate_script_pushes_only_after_a_finished_real_run():
    """The rule lives in the script: --mock and an interrupted run never write a board."""
    source = open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts",
                               "calibrate.py")).read()
    assert "if finished and not args.mock:\n        push_calibration_to_board(hand)" in source
