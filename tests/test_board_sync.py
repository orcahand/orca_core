"""The calibration kept on a hand's own board, and how it meets this computer's.

The rule: on first contact, a computer with no calibration for the hand copies
the board's; after that the computer's copy wins. A newer board calibration is
offered (board_calibration_newer), never taken silently.
"""

import os
import sys

import pytest

import orca_core
from orca_core import MockOrcaHand, OrcaHand, board_push, board_store, board_sync, hand_store
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
               ratios=None):
    """What a board calibrated elsewhere holds for this hand."""
    from orca_core.calibration import joint_fingerprints

    limits = {m: [0.1, 0.1 + limit] for m in config.motor_ids}
    return pack({
        "calibrated": True, "wrist_calibrated": True,
        "motor_limits": limits,
        "joint_to_motor_ratios": ratios or {m: 99.0 for m in config.motor_ids},
        "sequence": sequence, "calibrated_at": calibrated_at,
        "fingerprint": joint_fingerprints(config),
    }, identity=BoardIdentity(side="right", hw_version=210, serial="ser-0000"), sequence=9)


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

    def test_a_board_without_a_store_still_names_its_hand(self, port):
        contact = board_sync.contact_board("/dev/cu.usbmodemXXXX")
        assert contact.info.serial == "ser-0000" and contact.store is None


class TestWritingTheBoard:
    @pytest.fixture
    def firmware(self, tmp_path, monkeypatch):
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
        monkeypatch.setattr(board_push, "board_state",
                            lambda port: (BoardIdentity(side="right", serial="ser-0000"), 6))
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

    def test_without_a_firmware_checkout_nothing_is_written(self, connect, config, monkeypatch, tmp_path):
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(tmp_path / "nowhere"))
        result = board_push.push_for_hand(self.calibrated(connect, config))
        assert not result.written and "orca_firmware" in result.reason

    def test_a_tool_failure_is_reported_not_raised(self, connect, config, firmware):
        (firmware / "orca_firmware" / "tools" / "bench" / "rc").write_text("3")
        result = board_push.push_for_hand(self.calibrated(connect, config))
        assert not result.written and result.reason

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
