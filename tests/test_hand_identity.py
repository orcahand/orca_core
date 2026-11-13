"""Which physical hand is connected, and which folder its files live in.

The hands here run the real identity path (the board's answer at connect)
over mock motors, so no serial port is opened.
"""

import os
import shutil

import pytest

import orca_core
from orca_core import MockOrcaHand, OrcaHand, board_sync, hand_store
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
from orca_core.utils import read_yaml

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right")


def board(serial="ser-0000", board_id="0123456789ABCDEF", hw=210):
    return board_sync.BoardContact(info=OrcaBoardInfo(
        role="motor", side="right", hw_version=hw, serial=serial, board_id=board_id))


class BoardedHand(MockOrcaHand):
    """Mock motors, but identity resolved exactly as a physical hand's is."""

    contact = board_sync.NO_BOARD
    _settle_identity = OrcaHand._settle_identity
    _settle_identity_at_construction = OrcaHand._settle_identity_at_construction

    def _contact_board(self, port):
        return type(self).contact


@pytest.fixture
def model_dir(tmp_path):
    folder = tmp_path / "orcahand-right"
    folder.mkdir()
    shutil.copy(os.path.join(MODEL, "config.yaml"), folder / "config.yaml")
    return folder


@pytest.fixture
def make_hand(model_dir, monkeypatch):
    def make(contact=board_sync.NO_BOARD, **kwargs):
        monkeypatch.setattr(BoardedHand, "contact", contact)
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"), **kwargs)
        assert hand.connect()[0]
        return hand
    return make


class TestIdentityAtConnect:
    def test_a_board_with_a_serial_files_the_hand_under_it(self, make_hand):
        hand = make_hand(board())
        assert hand.hand_id == hand.board_hand_id == "ser-0000"
        assert hand.hand_dir == str(hand_store.hand_dir("ser-0000"))
        assert hand.config.calibration_path == os.path.join(hand.hand_dir, "calibration.yaml")
        record = hand_store.read_hand(hand.hand_dir)
        assert record["aliases"] == {"serial": ["ser-0000"], "board_id": ["0123456789ABCDEF"]}
        assert record["model"] == "orcahand-right"

    def test_a_board_without_a_serial_files_it_under_the_board_id(self, make_hand):
        assert make_hand(board(serial=None)).hand_id == "0123456789ABCDEF"

    def test_a_hand_that_reports_nothing_shares_its_model_s_folder(self, make_hand):
        hand = make_hand()
        assert hand.hand_id == "noboard-orcahand-right"
        assert hand.board_hand_id is None
        assert hand.hand_dir == str(hand_store.hand_dir("noboard-orcahand-right"))

    def test_before_connecting_a_hand_knows_no_folder_and_reads_nothing(self, model_dir):
        (model_dir / "calibration.yaml").write_text("calibrated: true\nmotor_limits: {1: [0, 1]}\n")
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        assert hand.hand_id is None and hand.hand_dir is None
        assert hand.motor_limits_dict[1] == [None, None]

    def test_nothing_is_written_beside_the_config(self, make_hand, model_dir):
        hand = make_hand(board())
        hand.calibrate(persist=True)
        assert sorted(p.name for p in model_dir.iterdir()) == ["config.yaml"]

    def test_a_board_id_folder_moves_to_the_serial_with_its_calibration(self, make_hand):
        first = make_hand(board(serial=None))
        first.calibrate(persist=True)
        limits = dict(first.motor_limits_dict)
        second = make_hand(board())
        assert second.hand_id == "ser-0000"
        assert second.motor_limits_dict == limits
        assert not hand_store.hand_dir("0123456789ABCDEF").exists()

    def test_another_hand_on_the_same_port_switches_folder_and_calibration(self, make_hand, monkeypatch):
        hand = make_hand(board("ser-0001"))
        hand.calibrate(persist=True)
        assert hand.calibrated
        hand.disconnect()
        monkeypatch.setattr(BoardedHand, "contact", board("ser-0002", "FEDCBA9876543210"))
        hand.connect()
        assert hand.hand_id == "ser-0002"
        assert not hand.calibrated, "the other hand's calibration must not carry over"


class TestAConnectThatFailsAfterTheBoardAnswered:
    """The board names the hand before the motor client opens; if the connect
    then fails, nothing of that hand may stay behind for a later save."""

    @staticmethod
    def failing(hand, monkeypatch):
        def no_client():
            raise OSError("the port went away")
        monkeypatch.setattr(hand, "_create_motor_client", no_client)

    def test_a_first_connect_leaves_the_hand_as_it_was(self, make_hand, model_dir, monkeypatch):
        calibrated = make_hand(board("ser-0001"))
        calibrated.calibrate(persist=True)
        calibrated.disconnect()
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"))
        before = dict(hand.motor_limits_dict)
        assert before != calibrated.motor_limits_dict
        self.failing(hand, monkeypatch)
        assert not hand.connect()[0]
        assert hand.hand_id is None and hand.board_hand_id is None and hand.hand_dir is None
        assert hand.board_hardware_version is None and hand.hardware_version is None
        assert hand.motor_limits_dict == before and hand.config.calibration_path == ""

    def test_a_reconnect_to_another_hand_keeps_the_first_hand_s_state(self, make_hand,
                                                                      monkeypatch):
        hand = make_hand(board("ser-0001"))
        hand.calibrate(persist=True)
        limits = dict(hand.motor_limits_dict)
        hand.disconnect()
        monkeypatch.setattr(BoardedHand, "contact", board("ser-0002", "FEDCBA9876543210", hw=200))
        self.failing(hand, monkeypatch)
        assert not hand.connect()[0]
        assert hand.hand_id == "ser-0001"
        assert hand.hand_dir == str(hand_store.hand_dir("ser-0001"))
        assert hand.motor_limits_dict == limits and hand.hardware_version == 210
        hand.set_rom_frame("centered", persist=True)
        assert read_yaml(str(hand_store.hand_dir("ser-0001") / "calibration.yaml"))[
            "rom_frame"] == "centered"
        assert not os.path.exists(hand_store.hand_dir("ser-0002") / "calibration.yaml")


class TestTheSameHandFindsTheSameFolder:
    def test_a_second_session_loads_what_the_first_calibrated(self, make_hand):
        first = make_hand(board())
        first.calibrate(persist=True)
        second = make_hand(board())
        assert second.calibrated
        assert second.motor_limits_dict == first.motor_limits_dict

    def test_an_operation_s_fresh_hand_files_its_calibration_where_the_session_reads(
        self, make_hand, monkeypatch
    ):
        """A board-less hand: the fresh hand an operation builds from the
        session's config must land on the session's folder, and the session
        sees the result when it reconnects."""
        session = make_hand()
        session.disconnect()
        fresh = BoardedHand(config=session.config)
        assert fresh.hand_dir == session.hand_dir
        fresh.connect()
        fresh.calibrate(persist=True)
        fresh.disconnect()
        assert not session.calibrated
        session.connect()
        assert session.hand_dir == fresh.hand_dir
        assert session.calibrated
        assert session.motor_limits_dict == fresh.motor_limits_dict

    def test_a_fresh_hand_from_the_session_config_reads_the_folder_before_connecting(self, make_hand):
        session = make_hand(board())
        session.calibrate(persist=True)
        fresh = OrcaHand(config=session.config)
        assert fresh.hand_id == "ser-0000"
        assert fresh.motor_limits_dict == session.motor_limits_dict

    def test_a_hand_that_never_connected_still_writes_into_a_hand_folder(self, model_dir):
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        hand.set_rom_frame("centered")
        assert hand.hand_id == "noboard-orcahand-right"
        assert read_yaml(hand.config.calibration_path)["rom_frame"] == "centered"


class TestExplicitCalibrationPath:
    def test_an_explicit_file_is_read_and_written_while_the_folder_is_still_the_hand_s(
        self, make_hand, tmp_path
    ):
        mine = tmp_path / "mine.yaml"
        hand = make_hand(board(), calibration_path=str(mine))
        assert hand.config.calibration_path == str(mine)
        assert hand.hand_dir == str(hand_store.hand_dir("ser-0000"))
        hand.calibrate(persist=True)
        assert read_yaml(str(mine))["calibrated"] is True
        assert not os.path.exists(os.path.join(hand.hand_dir, "calibration.yaml"))


class TestLegacyFiles:
    def test_found_once_when_the_folder_is_created_and_never_touched(self, make_hand, model_dir, caplog):
        (model_dir / "calibration.yaml").write_text("calibrated: true\n")
        (model_dir / "local_config.yaml").write_text("hardware_version: 21\n")
        hand = make_hand(board())
        assert sorted(os.path.basename(p) for p in hand.legacy_files) == [
            "calibration.yaml", "local_config.yaml"]
        assert "not used" in caplog.text
        assert not hand.calibrated, "an old calibration beside the config is never read"
        assert (model_dir / "calibration.yaml").read_text() == "calibrated: true\n"
        # Still listed for whichever front-end connects next, until dismissed.
        again = make_hand(board())
        assert len(again.legacy_files) == 2
        again.dismiss_legacy_files()
        assert make_hand(board()).legacy_files == []
        assert (model_dir / "calibration.yaml").exists()

    def test_only_looked_for_when_the_folder_is_created(self, make_hand, model_dir):
        make_hand(board())
        (model_dir / "setup.yaml").write_text("{}\n")
        assert make_hand(board()).legacy_files == []

    def test_none_without_old_files(self, make_hand):
        assert make_hand(board()).legacy_files == []


class TestNickname:
    def test_a_nickname_is_kept_with_the_hand(self, make_hand):
        hand = make_hand(board())
        assert hand.nickname is None
        assert hand.set_nickname("bench left") == "bench left"
        assert make_hand(board()).nickname == "bench left"

    def test_a_hand_with_no_folder_yet_cannot_be_named(self, model_dir):
        with pytest.raises(RuntimeError):
            OrcaHand(config_path=str(model_dir / "config.yaml")).set_nickname("x")


class TestMockHands:
    def test_a_mock_never_touches_orca_home(self, model_dir):
        hand = MockOrcaHand(config_path=str(model_dir / "config.yaml"))
        hand.connect()
        hand.calibrate(persist=True)
        assert hand.hand_id == "mock-orcahand-right"
        assert not hand_store.hands_dir().exists()
        assert not hand.hand_dir.startswith(str(hand_store.orca_home()))

    def test_mocks_of_one_model_share_their_temporary_folder(self, model_dir):
        first = MockOrcaHand(config_path=str(model_dir / "config.yaml"))
        second = MockOrcaHand(config_path=str(model_dir / "config.yaml"))
        assert first.hand_dir == second.hand_dir


class TestDetectionIsReusedAtConnect:
    def test_a_plain_adapter_detection_found_is_not_asked_again(self, model_dir, monkeypatch):
        def must_not_probe(port, **kwargs):
            raise AssertionError("connect asked a plain adapter for a board")

        monkeypatch.setattr(board_sync, "contact_board", must_not_probe)
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        hand._board_hint = ("/dev/cu.usbmodemXXXX", board_sync.NO_BOARD)
        assert hand._contact_board("/dev/cu.usbmodemXXXX") == board_sync.NO_BOARD

    def test_a_board_is_asked_again_for_its_calibration_store(self, model_dir, monkeypatch):
        monkeypatch.setattr(board_sync, "contact_board", lambda port, **kw: board("ser-0009"))
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        hand._board_hint = ("/dev/cu.usbmodemXXXX", board())
        assert hand._contact_board("/dev/cu.usbmodemXXXX").info.serial == "ser-0009"


class FakeLink:
    def __init__(self, reply):
        self.reply, self.out = reply, b""

    def reset_input_buffer(self):
        self.out = b""

    def write(self, data):
        if data.strip() == b"ORCA_INFO?" and self.reply:
            self.out = self.reply

    def flush(self):
        pass

    def read(self, size):
        out, self.out = self.out, b""
        return out


@pytest.fixture
def serial_port(monkeypatch):
    import serial

    state = {"link": None, "error": None}

    class FakeSerial:
        def __init__(self, *args, **kwargs):
            if state["error"]:
                raise state["error"]

        def __enter__(self):
            return state["link"]

        def __exit__(self, *exc):
            return False

    monkeypatch.setattr(serial, "Serial", FakeSerial)
    return state


class TestAskingTheBoard:
    def test_a_board_s_answer_is_its_identity(self, serial_port):
        serial_port["link"] = FakeLink(
            b"ORCA:MOTOR;SIDE=L;HW=210;SN=ser-0000;BID=0123456789ABCDEF\n")
        info = board_sync.contact_board("/dev/cu.usbmodemXXXX").info
        assert (info.serial, info.board_id, info.side, info.hw_version) == (
            "ser-0000", "0123456789ABCDEF", "left", 210)

    def test_silence_is_no_board(self, serial_port):
        serial_port["link"] = FakeLink(None)
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX") == board_sync.NO_BOARD

    def test_a_port_that_will_not_open_is_no_board_not_a_failure(self, serial_port):
        serial_port["error"] = OSError("busy")
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX") == board_sync.NO_BOARD


INFO_LINE = b"ORCA:MOTOR;SIDE=R;HW=210;SN=ser-0000;BID=0123456789ABCDEF\n"


class FlakyLink(FakeLink):
    """A board that drops its first ``drops`` identity replies; ``legacy``
    answers only the old ORCA_ID? query."""

    def __init__(self, drops, legacy=False):
        super().__init__(INFO_LINE)
        self.drops, self.legacy, self.asked = drops, legacy, 0

    def write(self, data):
        if data.strip() == b"ORCA_ID?" and self.legacy:
            self.out = b"ORCA:MOTOR\n"
        elif data.strip() == b"ORCA_INFO?" and not self.legacy:
            self.asked += 1
            if self.asked > self.drops:
                self.out = self.reply


class TestABoardThatDoesNotSayWhoItIs:
    """A dropped identity reply must never file a boarded hand as board-less:
    that folder is shared by every board-less hand of the model."""

    @pytest.fixture
    def board_port(self, serial_port, monkeypatch):
        monkeypatch.setattr(board_sync, "is_board_port", lambda port: True)
        monkeypatch.setattr(board_sync, "IDENTITY_RETRY_S", 0.0)
        return serial_port

    def test_a_dropped_reply_is_asked_again(self, board_port):
        board_port["link"] = link = FlakyLink(drops=2)
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX").info.serial == "ser-0000"
        assert link.asked == 3

    def test_a_board_silent_every_time_is_reported_as_such(self, board_port):
        board_port["link"] = FlakyLink(drops=99)
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX").silent_board

    def test_a_board_port_that_will_not_open_is_not_taken_for_no_board(self, board_port):
        board_port["error"] = OSError("busy")
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX").silent_board

    def test_firmware_with_no_identity_to_give_is_still_a_board_less_hand(self, board_port):
        board_port["link"] = FlakyLink(drops=0, legacy=True)
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX") == board_sync.NO_BOARD

    def test_a_plain_adapter_is_asked_once(self, serial_port):
        serial_port["link"] = link = FlakyLink(drops=99)
        assert board_sync.contact_board("/dev/cu.usbmodemXXXX") == board_sync.NO_BOARD
        assert link.asked == 1

    def test_the_connect_fails_and_files_nothing(self, make_hand, model_dir, monkeypatch):
        monkeypatch.setattr(BoardedHand, "contact", board_sync.BoardContact(silent_board=True))
        hand = BoardedHand(config_path=str(model_dir / "config.yaml"))
        ok, message = hand.connect()
        assert not ok and "did not report its identity" in message
        assert hand.hand_id is None
        assert not hand_store.hand_dir("noboard-orcahand-right").exists()


class TestNowhereToKeepTheCalibration:
    """An ORCA_HOME that cannot be written is said before a calibration moves
    anything, not as an IsADirectoryError half way through it."""

    @pytest.fixture
    def unwritable_home(self, tmp_path, monkeypatch):
        blocker = tmp_path / "not-a-folder"
        blocker.write_text("")
        monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(blocker))
        return blocker

    def test_calibrating_refuses_before_any_motion(self, make_hand, unwritable_home, monkeypatch,
                                                    caplog):
        from orca_core import hardware_hand

        hand = make_hand(board())
        assert hand.hand_dir is None and "will not be kept" in caplog.text
        monkeypatch.setattr(hardware_hand, "run_calibration",
                            lambda *a, **k: pytest.fail("the routine ran"))
        with pytest.raises(OSError, match="ORCA_HOME"):
            hand.calibrate(persist=True)
        assert hand.hand_id is None, "a boarded hand is never re-filed as board-less"

    def test_a_hand_that_never_connected_says_so_too(self, model_dir, unwritable_home):
        hand = OrcaHand(config_path=str(model_dir / "config.yaml"))
        with pytest.raises(OSError, match="ORCA_HOME"):
            hand.set_rom_frame("centered")

    def test_an_explicit_file_in_a_folder_still_to_be_made_is_fine(self, make_hand, tmp_path):
        target = tmp_path / "new" / "deeper" / "mine.yaml"
        hand = make_hand(board(), calibration_path=str(target))
        hand.calibrate(persist=True)
        assert read_yaml(str(target))["calibrated"] is True
