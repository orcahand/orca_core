"""Writing a hand's calibration back onto the hand's own board.

The write itself belongs to orca_firmware's bench tool, which is not shipped
with orca_core, so every path here has to stay sane with no checkout, no board
and no name -- none of which is an error.
"""

from pathlib import Path

import pytest

from orca_core import board_push, board_store
from orca_core.board_store import BoardIdentity


@pytest.fixture()
def fake_tool(tmp_path):
    """A stand-in for the firmware tool, in a repo-shaped directory."""

    def make(exit_code=0, body="pass"):
        tool = tmp_path / "repo" / board_push.TOOL_RELATIVE_PATH
        tool.parent.mkdir(parents=True, exist_ok=True)
        tool.write_text(f"import sys\n{body}\nsys.exit({exit_code})\n")
        return tmp_path / "repo"

    return make


class TestFindingTheTool:
    def test_a_pinned_repo_with_no_tool_is_no_tool(self, tmp_path):
        assert board_push.firmware_tool(tmp_path) is None

    def test_a_pinned_repo_is_the_only_place_looked(self, tmp_path, monkeypatch):
        """A sibling checkout must not quietly answer for a repo someone named."""
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(tmp_path))
        assert board_push.firmware_tool() is None

    def test_the_tool_is_found_where_it_is(self, fake_tool):
        repo = fake_tool()
        assert board_push.firmware_tool(repo) == repo / board_push.TOOL_RELATIVE_PATH


class TestWhatIsSkippedRatherThanFailed:
    def hand(self):
        from orca_core.hardware_hand import MockOrcaHand

        return MockOrcaHand()

    def test_a_hand_with_no_name(self):
        result = board_push.push_for_hand(self.hand())
        assert not result.written and "no id" in result.reason

    def test_a_hand_with_no_board(self):
        hand = self.hand()
        hand._hand_id = "bench-left"
        result = board_push.push_for_hand(hand)
        assert not result.written and "no board" in result.reason

    def test_a_machine_with_no_firmware_checkout(self, tmp_path, monkeypatch):
        monkeypatch.setenv(board_push.FIRMWARE_REPO_ENV, str(tmp_path))
        result = board_push.push("ser-0000", "mock", {"calibrated": True})
        assert not result.written and "checkout" in result.reason


class TestWhatIsWritten:
    @pytest.fixture(autouse=True)
    def board(self, monkeypatch):
        """A board already holding sequence 7 and an identity."""
        self.identity = BoardIdentity(
            side="right", hw_version=21, config=2500, serial="ser-0000")
        monkeypatch.setattr(board_push, "board_state",
                            lambda port, **kw: (self.identity, 7))

    def written_blob(self, directory, hand_id="ser-0000"):
        return board_store.unpack((directory / f"{hand_id}.bin").read_bytes())

    def test_the_board_s_identity_is_carried_into_the_write(self, fake_tool, tmp_path):
        """A blob packed without it would make the board forget which hand it is."""
        repo = fake_tool()
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert result.written
        assert self.written_blob(tmp_path).identity == self.identity

    def test_it_is_written_one_past_what_the_board_has(self, fake_tool, tmp_path):
        """An equal or lower sequence would leave the board on its older slot."""
        repo = fake_tool()
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert result.sequence == 8
        assert self.written_blob(tmp_path).sequence == 8

    def test_the_calibration_is_what_goes_down(self, fake_tool, tmp_path):
        repo = fake_tool()
        board_push.push("ser-0000", "mock",
                        {"calibrated": True, "motor_limits": {1: [-2.0, 2.0]}},
                        directory=tmp_path, repo=repo)
        stored = self.written_blob(tmp_path).calibration
        assert stored["motor_limits"][1] == [-2.0, 2.0]
        assert stored["calibrated"] is True


class TestAWriteThatDoesNotLand:
    """The mirror must never claim something the board does not hold."""

    @pytest.fixture(autouse=True)
    def board(self, monkeypatch):
        monkeypatch.setattr(board_push, "board_state",
                            lambda port, **kw: (BoardIdentity(), 1))

    def test_the_mirror_it_made_is_taken_back(self, fake_tool, tmp_path):
        repo = fake_tool(exit_code=1)
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert not result.written
        assert not (tmp_path / "ser-0000.bin").exists()

    def test_the_mirror_that_was_there_is_put_back(self, fake_tool, tmp_path):
        repo = fake_tool(exit_code=1)
        kept = board_store.pack({"calibrated": False}, sequence=3)
        (tmp_path / "ser-0000.bin").write_bytes(kept)
        board_push.push("ser-0000", "mock", {"calibrated": True},
                        directory=tmp_path, repo=repo)
        assert (tmp_path / "ser-0000.bin").read_bytes() == kept

    def test_what_the_tool_said_is_what_is_reported(self, fake_tool, tmp_path):
        repo = fake_tool(exit_code=1, body="print('no firmware answered', file=sys.stderr)")
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert "no firmware answered" in result.reason

    def test_a_board_that_will_not_answer_writes_nothing(self, fake_tool, tmp_path,
                                                         monkeypatch):
        repo = fake_tool()
        monkeypatch.setattr(board_push, "board_state",
                            lambda port, **kw: (_ for _ in ()).throw(OSError("no port")))
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert not result.written and "did not answer" in result.reason
        assert not (tmp_path / "ser-0000.bin").exists()


class TestAPortTheHandStillHas:
    """The tool opens the port itself, so a connected hand blocks the write.

    Worth its own message: it is the one failure a front-end can do something
    about, by writing the board once the hand has let go.
    """

    def test_a_locked_port_says_what_to_do_about_it(self, fake_tool, tmp_path,
                                                    monkeypatch):
        repo = fake_tool()
        monkeypatch.setattr(
            board_push, "board_state",
            lambda port, **kw: (_ for _ in ()).throw(
                OSError("Could not exclusively lock port /dev/null")))
        result = board_push.push("ser-0000", "mock", {"calibrated": True},
                                 directory=tmp_path, repo=repo)
        assert not result.written
        assert "once the hand has disconnected" in result.reason
