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


class SplittingBoard(FakeBoard):
    """A board whose replies arrive a few bytes at a time, as a real one's do.

    This is what hardware does and what a fake that answers in whole lines
    never shows: a reader that matches on a partial line decodes a truncated
    hex field, or reads CHUNK=12 out of a CHUNK=128 that has not finished
    arriving.
    """

    def __init__(self, *args, piece=7, **kwargs):
        super().__init__(*args, **kwargs)
        self.piece = piece

    def read(self, _size):
        out, self.out = bytes(self.out[:self.piece]), self.out[self.piece:]
        return out


class TestRepliesThatArriveInPieces:
    """Pinned because real hardware split them and the fakes did not."""

    def test_the_metadata_is_not_read_until_the_line_is_whole(self):
        store = blob()
        meta = board_store_link.read_metadata_over_link(SplittingBoard(store))
        assert meta is not None
        assert meta.length == len(store)
        assert meta.chunk_bytes == 128     # not 1, 12 or anything truncated

    def test_a_chunk_is_not_decoded_until_its_line_is_whole(self):
        store = blob()
        assert board_store_link.read_store_over_link(SplittingBoard(store)) == store

    def test_it_holds_up_down_to_a_byte_at_a_time(self):
        store = blob()
        assert board_store_link.read_store_over_link(
            SplittingBoard(store, piece=1)) == store


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


class TestALostOrDamagedReply:
    def test_silence_is_asked_again_and_told_apart_from_no_store(self):
        class LosesFirst(FakeBoard):
            lost = 1

            def write(self, data):
                if bytes(data).strip() == b"ORCA_CAL?" and self.lost:
                    self.lost -= 1
                    return
                super().write(data)

        store = blob()
        assert board_store_link.query_metadata(LosesFirst(store), timeout=0.01) == (
            True, board_store_link.read_metadata_over_link(FakeBoard(store)))
        assert board_store_link.query_metadata(FakeBoard(None), timeout=0.01) == (True, None)

        class Silent(FakeBoard):
            def write(self, data):
                pass

        assert board_store_link.query_metadata(Silent(store), timeout=0.01) == (False, None)

    def test_a_chunk_that_lost_a_hex_digit_is_asked_for_again(self):
        class DropsADigitOnce(FakeBoard):
            dropped = False

            def write(self, data):
                super().write(data)
                if bytes(data).startswith(b"ORCA_CAL?0") and not self.dropped:
                    self.dropped = True
                    self.out = self.out[:-2] + self.out[-1:]   # one digit short

        store = blob()
        assert board_store_link.read_store_over_link(DropsADigitOnce(store)) == store
