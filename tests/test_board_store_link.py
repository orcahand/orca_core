"""Reading a hand's calibration store off its board, and mirroring it.

The board is modelled in Python here, speaking the protocol the firmware
implements (orca_firmware's cal_store.h), so the reader is driven exactly as a
hand drives it: a metadata line, then chunks with a CRC each.
"""

import pytest

from orca_core import board_store, board_store_link
from orca_core.board_store import BoardIdentity, crc16, pack


def blob(sequence=3, joints=("index_mcp", "wrist")):
    return pack(
        {
            "calibrated": True,
            "wrist_calibrated": True,
            "joint_to_motor_ratios": {1: 0.02, 2: 0.03},
            "motor_limits": {1: [-1.0, 1.0], 2: [0.5, 2.5]},
            "motor_travel_measured": {j: 120.5 for j in joints},
        },
        identity=BoardIdentity(side="right", hw_version=21, config=2500, serial="ser-0000"),
        sequence=sequence,
    )


class FakeBoard:
    """A board as far as the store reader is concerned.

    ``corrupt`` names chunk indices to answer wrongly the first time each, so
    the retry path is driven; ``always_corrupt`` never answers them cleanly.
    """

    def __init__(self, store=None, chunk_bytes=128, slot=0,
                 corrupt=(), always_corrupt=(), answers_info=True):
        self.store = store
        self.chunk_bytes = chunk_bytes
        self.slot = slot
        self.corrupt = set(corrupt)
        self.always_corrupt = set(always_corrupt)
        self.answers_info = answers_info
        self.out = bytearray()
        self.asked = []

    # the serial-port surface the reader uses
    def reset_input_buffer(self):
        self.out.clear()

    def flush(self):
        pass

    def read(self, _size):
        out, self.out = bytes(self.out), bytearray()
        return out

    def write(self, data):
        self.asked.append(bytes(data))
        line = bytes(data).strip()
        if line == b"ORCA_INFO?" and self.answers_info:
            self.out += (b"ORCA:MOTOR;SIDE=R;HW=21;FW=3;CFG=2500;SN=ser-0000;"
                         b"BID=0123456789ABCDEF\n")
            return
        if line == b"ORCA_CAL?":
            if self.store is None:
                self.out += b"ORCA:CAL;NONE\n"
                return
            self.out += (
                b"ORCA:CAL;SLOT=%d;SEQ=%d;LEN=%d;CRC=%04X;CHUNK=%d\n"
                % (self.slot,
                   int.from_bytes(self.store[8:12], "little"),
                   len(self.store), crc16(self.store), self.chunk_bytes))
            return
        if line.startswith(b"ORCA_CAL?"):
            index = int(line[len(b"ORCA_CAL?"):])
            part = self.store[index * self.chunk_bytes:(index + 1) * self.chunk_bytes]
            if index in self.always_corrupt or index in self.corrupt:
                self.corrupt.discard(index)
                part = bytes(b ^ 0xFF for b in part)       # a bad line
                self.out += b"ORCA:CALD;%d;%04X;%s\n" % (
                    index, crc16(self.store[index * self.chunk_bytes:
                                            (index + 1) * self.chunk_bytes]),
                    part.hex().upper().encode())
                return
            self.out += b"ORCA:CALD;%d;%04X;%s\n" % (
                index, crc16(part), part.hex().upper().encode())


class TestReadingTheStore:
    def test_the_metadata_says_what_the_board_holds(self):
        store = blob(sequence=7)
        meta = board_store_link.read_metadata_over_link(FakeBoard(store))
        assert meta.sequence == 7
        assert meta.length == len(store)
        assert meta.crc == crc16(store)

    def test_a_board_with_no_store_reports_none(self):
        assert board_store_link.read_metadata_over_link(FakeBoard(None)) is None
        assert board_store_link.read_store_over_link(FakeBoard(None)) is None

    def test_something_that_is_not_a_board_reports_none(self):
        """A plain adapter: the query reaches the motors, which ignore it."""
        assert board_store_link.read_metadata_over_link(FakeBoard(None, answers_info=False)) is None

    def test_the_whole_store_reads_back_byte_for_byte(self):
        store = blob()
        assert board_store_link.read_store_over_link(FakeBoard(store)) == store

    def test_it_reads_the_same_whatever_chunk_size_the_board_uses(self):
        store = blob()
        for chunk in (16, 64, 128, 4096):
            assert board_store_link.read_store_over_link(
                FakeBoard(store, chunk_bytes=chunk)) == store

    def test_a_corrupt_line_is_asked_for_again(self):
        store = blob()
        board = FakeBoard(store, chunk_bytes=64, corrupt={1})
        assert board_store_link.read_store_over_link(board) == store
        assert sum(1 for q in board.asked if q.startswith(b"ORCA_CAL?1\n")) == 2

    def test_a_line_that_never_comes_back_clean_fails_the_read(self):
        """Half a calibration must not be mistaken for the hand's."""
        board = FakeBoard(blob(), chunk_bytes=64, always_corrupt={1})
        with pytest.raises(OSError, match="chunk 1"):
            board_store_link.read_store_over_link(board)

    def test_a_store_that_does_not_match_the_reported_crc_fails_the_read(self):
        store = bytearray(blob())
        board = FakeBoard(bytes(store))
        board.store = bytes(store)
        # Report a CRC for different bytes than the chunks will carry.
        original_write = board.write

        def lying_write(data):
            original_write(data)
            if bytes(data).strip() == b"ORCA_CAL?":
                board.out = bytearray(
                    board.out.replace(b"CRC=%04X" % crc16(bytes(store)), b"CRC=0000"))

        board.write = lying_write
        with pytest.raises(OSError, match="does not match"):
            board_store_link.read_store_over_link(board)

    def test_what_is_read_unpacks_to_the_calibration_that_was_packed(self):
        store = blob()
        got = board_store.unpack(board_store_link.read_store_over_link(FakeBoard(store)))
        assert got.calibration["motor_limits"] == {1: [-1.0, 1.0], 2: [0.5, 2.5]}
        assert got.identity.serial == "ser-0000"


class TestTheMirror:
    def test_a_store_is_mirrored_as_bytes_and_as_something_readable(self, tmp_path):
        store = blob()
        board_store.write_cache(store, "0123456789ABCDEF", directory=tmp_path)
        assert (tmp_path / "0123456789ABCDEF.bin").read_bytes() == store
        readable = (tmp_path / "0123456789ABCDEF.yaml").read_text()
        assert "board_id: 0123456789ABCDEF" in readable
        assert "the board is the authority" in readable

    def test_the_mirror_reads_back_as_the_same_bytes(self, tmp_path):
        store = blob()
        board_store.write_cache(store, "ABCD", directory=tmp_path)
        assert board_store.cached_blob("ABCD", directory=tmp_path) == store

    def test_nothing_mirrored_is_not_an_error(self, tmp_path):
        assert board_store.cached_blob("NOSUCHBOARD", directory=tmp_path) is None

    def test_a_mirror_that_no_longer_checks_out_is_ignored(self, tmp_path):
        (tmp_path / "BAD.bin").write_bytes(b"OHC1" + b"\x00" * 40)
        assert board_store.cached_blob("BAD", directory=tmp_path) is None

    def test_a_broken_store_is_never_mirrored(self, tmp_path):
        with pytest.raises(board_store.BoardStoreError):
            board_store.write_cache(b"not a store", "ABCD", directory=tmp_path)
        assert list(tmp_path.iterdir()) == []

    def test_the_directory_can_be_pointed_elsewhere(self, tmp_path, monkeypatch):
        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        assert board_store.calibrations_dir() == tmp_path


class TestAHandAdoptsItsBoardsCalibration:
    """What a connect does with a board that carries a calibration."""

    @pytest.fixture()
    def board(self, monkeypatch, tmp_path):
        """Patch the serial port a connect opens to answer as a board."""
        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
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

    def hand(self):
        from orca_core.hardware_hand import MockOrcaHand

        return MockOrcaHand()

    def test_the_board_wins_over_what_was_loaded_from_disk(self, board):
        board.store = blob(sequence=5)
        hand = self.hand()
        on_disk = hand.calibration.motor_limits_dict.get(1)
        hand._adopt_board_calibration("mock")
        assert hand.calibration.motor_limits_dict[1] == [-1.0, 1.0]
        assert hand.calibration.motor_limits_dict[1] != on_disk
        assert hand._board_store.sequence == 5

    def test_a_board_with_no_store_changes_nothing(self, board):
        hand = self.hand()
        before = hand.calibration
        hand._adopt_board_calibration("mock")
        assert hand.calibration is before
        assert hand._board_store is None

    def test_a_board_that_will_not_answer_changes_nothing(self, board):
        board.answers_info = False
        hand = self.hand()
        before = hand.calibration
        hand._adopt_board_calibration("mock")
        assert hand.calibration is before

    def test_a_store_read_adopted_once_is_mirrored_for_next_time(self, board, tmp_path):
        board.store = blob()
        self.hand()._adopt_board_calibration("mock")
        assert (tmp_path / "0123456789ABCDEF.bin").read_bytes() == board.store

    def test_an_unchanged_store_is_not_pulled_again(self, board, tmp_path):
        """One round trip when the mirror already matches the board's CRC."""
        board.store = blob()
        board_store.write_cache(board.store, "0123456789ABCDEF", directory=tmp_path)
        hand = self.hand()
        hand._adopt_board_calibration("mock")
        assert hand._board_store is not None
        assert not any(q.startswith(b"ORCA_CAL?0") for q in board.asked)

    def test_a_changed_store_is_pulled_even_though_one_is_mirrored(self, board, tmp_path):
        board_store.write_cache(blob(sequence=1), "0123456789ABCDEF", directory=tmp_path)
        board.store = blob(sequence=9)
        hand = self.hand()
        hand._adopt_board_calibration("mock")
        assert hand._board_store.sequence == 9
        assert any(q.startswith(b"ORCA_CAL?0") for q in board.asked)

    def test_a_store_this_version_cannot_read_leaves_the_disk_copy(self, board, caplog):
        store = bytearray(blob())
        store[4] = 99                      # a layout from the future
        import struct
        struct.pack_into("<H", store, 14,
                         crc16(bytes(store[:14]) + bytes(store[16:])))
        board.store = bytes(store)
        hand = self.hand()
        before = hand.calibration
        with caplog.at_level("WARNING"):
            hand._adopt_board_calibration("mock")
        assert hand.calibration is before
        assert "cannot" in caplog.text

    def test_a_connect_is_never_failed_by_any_of_this(self, board):
        """A hand must still come up when the store cannot be read at all."""
        def explode(*args, **kwargs):
            raise OSError("the port went away")

        board.write = explode
        hand = self.hand()
        hand._adopt_board_calibration("mock")   # must not raise
        assert hand._board_store is None
