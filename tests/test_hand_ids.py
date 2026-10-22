"""Which hands a machine knows, and the menu that manages them.

A hand with no ORCA board cannot name itself, so the user does. These are the
rules for those names and the picker that adds, chooses and forgets them —
driven by feeding it answers, the way a person would type them.
"""

from pathlib import Path

import pytest

from orca_core import hand_ids
from orca_core.hand_ids import HandIdError


class Typist:
    """Answers a prompt from a script, and records what it was asked."""

    def __init__(self, *answers):
        self.answers = list(answers)
        self.prompts = []
        self.shown = []

    def __call__(self, prompt):
        self.prompts.append(prompt)
        if not self.answers:
            raise EOFError("nothing more was typed")
        return self.answers.pop(0)

    def show(self, line):
        self.shown.append(line)

    @property
    def screen(self):
        return "\n".join(self.shown)


class TestWhatCanBeAName:
    @pytest.mark.parametrize("name", ["ser-0000", "a", "my.hand_2", "A" * 20])
    def test_accepted(self, name):
        assert hand_ids.valid_id(name)

    @pytest.mark.parametrize("name", ["", "-", "._-", "A" * 21, "has space",
                                      "sl/ash", "uniçode"])
    def test_refused(self, name):
        assert not hand_ids.valid_id(name)

    def test_the_refusal_says_what_the_rule_is(self):
        with pytest.raises(HandIdError, match=r"1-20 characters"):
            hand_ids.check_id("has space")

    def test_the_rule_is_the_one_a_flashable_serial_follows(self):
        """So an id a user invents and a serial we assign are the same kind."""
        assert hand_ids.valid_id("ser-0000")
        assert hand_ids.valid_id("0123456789ABCDEF")      # a board id, too


class TestWhatThisMachineKnows:
    def test_nothing_at_first(self, tmp_path):
        assert hand_ids.known_ids(tmp_path) == []

    def test_a_directory_that_is_not_there_is_not_an_error(self, tmp_path):
        assert hand_ids.known_ids(tmp_path / "nope") == []

    def test_an_id_can_exist_before_any_calibration_does(self, tmp_path):
        """Adding is how a second board-less hand gets told from the first."""
        hand_ids.add_id("left-bench", tmp_path)
        assert hand_ids.known_ids(tmp_path) == ["left-bench"]

    def test_a_calibration_filed_under_a_name_makes_it_known(self, tmp_path):
        (tmp_path / "ser-0001.bin").write_bytes(b"")
        assert hand_ids.known_ids(tmp_path) == ["ser-0001"]

    def test_the_same_hand_is_not_listed_once_per_file(self, tmp_path):
        (tmp_path / "ser-0001.bin").write_bytes(b"")
        (tmp_path / "ser-0001.yaml").write_text("")
        assert hand_ids.known_ids(tmp_path) == ["ser-0001"]

    def test_they_come_back_sorted(self, tmp_path):
        """Because auto picks the first, which has to mean something stable."""
        for name in ("zeta", "alpha", "middle"):
            hand_ids.add_id(name, tmp_path)
        assert hand_ids.known_ids(tmp_path) == ["alpha", "middle", "zeta"]

    def test_adding_twice_is_not_an_error(self, tmp_path):
        hand_ids.add_id("bench", tmp_path)
        hand_ids.add_id("bench", tmp_path)
        assert hand_ids.known_ids(tmp_path) == ["bench"]

    def test_adding_does_not_disturb_a_calibration_already_filed(self, tmp_path):
        (tmp_path / "bench.bin").write_bytes(b"a calibration")
        hand_ids.add_id("bench", tmp_path)
        assert (tmp_path / "bench.bin").read_bytes() == b"a calibration"

    def test_the_marker_is_written_even_beside_a_calibration(self, tmp_path):
        """It is what makes the name a chosen one, which is what gets offered."""
        (tmp_path / "bench.bin").write_bytes(b"a calibration")
        hand_ids.add_id("bench", tmp_path)
        assert (tmp_path / f"bench{hand_ids.MARKER_SUFFIX}").exists()
        assert hand_ids.assigned_ids(tmp_path) == ["bench"]

    def test_a_name_that_cannot_be_one_is_refused(self, tmp_path):
        with pytest.raises(HandIdError):
            hand_ids.add_id("has space", tmp_path)


class TestForgettingOne:
    def test_everything_filed_under_it_goes(self, tmp_path):
        (tmp_path / "bench.bin").write_bytes(b"")
        (tmp_path / "bench.yaml").write_text("")
        removed = hand_ids.delete_id("bench", tmp_path)
        assert {path.name for path in removed} == {"bench.bin", "bench.yaml"}
        assert hand_ids.known_ids(tmp_path) == []

    def test_a_marker_only_one_goes_too(self, tmp_path):
        hand_ids.add_id("named-only", tmp_path)
        removed = hand_ids.delete_id("named-only", tmp_path)
        assert [path.suffix for path in removed] == [hand_ids.MARKER_SUFFIX]

    def test_other_hands_are_left_alone(self, tmp_path):
        hand_ids.add_id("keep", tmp_path)
        hand_ids.add_id("drop", tmp_path)
        hand_ids.delete_id("drop", tmp_path)
        assert hand_ids.known_ids(tmp_path) == ["keep"]

    def test_one_that_is_not_there_says_so(self, tmp_path):
        with pytest.raises(HandIdError, match="knows no hand"):
            hand_ids.delete_id("ghost", tmp_path)


class TestWhatAutoResolvesTo:
    def test_the_serial_first(self, tmp_path):
        assert hand_ids.auto_id("ser-0000", "0123456789ABCDEF", tmp_path) == "ser-0000"

    def test_then_the_board_id(self, tmp_path):
        assert hand_ids.auto_id(None, "0123456789ABCDEF", tmp_path) == "0123456789ABCDEF"

    def test_then_the_first_name_this_machine_knows(self, tmp_path):
        hand_ids.add_id("zeta", tmp_path)
        hand_ids.add_id("alpha", tmp_path)
        assert hand_ids.auto_id(None, None, tmp_path) == "alpha"

    def test_and_nothing_when_there_is_nothing(self, tmp_path):
        assert hand_ids.auto_id(None, None, tmp_path) is None

    def test_a_board_that_reports_an_unusable_serial_falls_through(self, tmp_path):
        assert hand_ids.auto_id("-", "0123456789ABCDEF", tmp_path) == "0123456789ABCDEF"


class TestThePicker:
    """Same shape as the port picker: numbered, with q to give up."""

    def pick(self, typist, tmp_path):
        return hand_ids.choose_id(ask=typist, show=typist.show, directory=tmp_path)

    def test_choosing_by_number(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        hand_ids.add_id("beta", tmp_path)
        typist = Typist("2")
        assert self.pick(typist, tmp_path) == "beta"

    def test_the_list_is_shown_with_numbers(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        typist = Typist("1")
        self.pick(typist, tmp_path)
        assert " 1. alpha" in typist.screen

    def test_q_gives_up(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        assert self.pick(Typist("q"), tmp_path) is None

    def test_a_closed_input_gives_up_rather_than_looping(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        assert self.pick(Typist(), tmp_path) is None

    def test_a_number_out_of_range_asks_again(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        typist = Typist("7", "1")
        assert self.pick(typist, tmp_path) == "alpha"
        assert "Enter a number" in typist.screen

    def test_adding_from_the_menu_then_choosing_it(self, tmp_path):
        typist = Typist("a", "right-bench", "1")
        assert self.pick(typist, tmp_path) == "right-bench"
        assert hand_ids.known_ids(tmp_path) == ["right-bench"]

    def test_a_name_that_cannot_be_one_is_refused_and_asked_again(self, tmp_path):
        typist = Typist("a", "has space", "a", "fine-name", "1")
        assert self.pick(typist, tmp_path) == "fine-name"
        assert "1-20 characters" in typist.screen

    def test_it_says_so_when_there_is_nothing_to_choose_from(self, tmp_path):
        typist = Typist("q")
        assert self.pick(typist, tmp_path) is None
        assert "no registered hand ids yet" in typist.screen

    def test_deleting_asks_before_it_does_it(self, tmp_path):
        hand_ids.add_id("doomed", tmp_path)
        typist = Typist("d", "1", "y", "q")
        assert self.pick(typist, tmp_path) is None
        assert hand_ids.known_ids(tmp_path) == []
        assert any("Forget doomed" in prompt for prompt in typist.prompts)

    def test_saying_no_to_the_confirmation_keeps_it(self, tmp_path):
        hand_ids.add_id("spared", tmp_path)
        typist = Typist("d", "1", "n", "1")
        assert self.pick(typist, tmp_path) == "spared"
        assert hand_ids.known_ids(tmp_path) == ["spared"]

    def test_anything_but_yes_keeps_it(self, tmp_path):
        """A calibration is thrown away here, so silence must mean no."""
        for answer in ("", "no", "maybe", "Y E S"):
            hand_ids.add_id("spared", tmp_path)
            typist = Typist("d", "1", answer, "1")
            assert self.pick(typist, tmp_path) == "spared"
            assert hand_ids.known_ids(tmp_path) == ["spared"]

    def test_deleting_nothing_in_particular_is_refused(self, tmp_path):
        typist = Typist("d", "q")
        assert self.pick(typist, tmp_path) is None
        assert "nothing to delete" in typist.screen

    def test_the_whole_flow_add_two_delete_one_choose_the_other(self, tmp_path):
        typist = Typist("a", "left", "a", "right", "d", "1", "y", "1")
        assert self.pick(typist, tmp_path) == "right"
        assert hand_ids.known_ids(tmp_path) == ["right"]


class TestSettlingWhichHandThisIs:
    """What a script does at connect, which is nothing unless it has to ask."""

    def hand(self, port="/dev/null"):
        from orca_core.hardware_hand import MockOrcaHand

        hand = MockOrcaHand()
        hand.config = __import__("dataclasses").replace(hand.config, port=port)
        return hand

    def settle(self, hand, typist=None, interactive=True):
        from orca_core.utils.cli import settle_hand_id

        return settle_hand_id(hand, interactive=interactive,
                              ask=typist or Typist())

    def test_a_board_that_names_itself_is_never_asked_about(self, monkeypatch):
        from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
        import orca_core.utils.cli as cli

        monkeypatch.setattr(cli, "board_info_for_port", lambda port: OrcaBoardInfo(
            role="motor", serial="ser-0000", board_id="0123456789ABCDEF"))
        typist = Typist()
        assert self.settle(self.hand(), typist) == "ser-0000"
        assert typist.prompts == []

    def test_a_board_with_no_serial_is_named_by_its_board_id(self, monkeypatch):
        from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
        import orca_core.utils.cli as cli

        monkeypatch.setattr(cli, "board_info_for_port", lambda port: OrcaBoardInfo(
            role="motor", serial=None, board_id="0123456789ABCDEF"))
        assert self.settle(self.hand()) == "0123456789ABCDEF"

    def test_a_board_less_hand_is_asked(self, monkeypatch, tmp_path):
        import orca_core.utils.cli as cli
        from orca_core import board_store

        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
        hand_ids.add_id("bench-left", tmp_path)
        typist = Typist("1")
        assert self.settle(self.hand(), typist) == "bench-left"

    def test_a_board_less_hand_nobody_can_answer_for_keeps_the_old_path(
        self, monkeypatch, tmp_path
    ):
        """Headless: no prompt, no id, and the model directory still works."""
        import orca_core.utils.cli as cli
        from orca_core import board_store

        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
        hand = self.hand()
        assert self.settle(hand, interactive=False) is None
        assert hand._hand_id is None

    def test_giving_up_at_the_picker_keeps_the_old_path(self, monkeypatch, tmp_path):
        import orca_core.utils.cli as cli
        from orca_core import board_store

        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
        hand = self.hand()
        assert self.settle(hand, Typist("q")) is None
        assert hand._hand_id is None

    def test_an_id_already_settled_is_not_asked_about_again(self, monkeypatch):
        import orca_core.utils.cli as cli

        monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
        hand = self.hand()
        hand._hand_id = "already-known"
        typist = Typist()
        assert self.settle(hand, typist) == "already-known"
        assert typist.prompts == []

    def test_the_picker_can_add_a_hand_mid_prompt(self, monkeypatch, tmp_path):
        """A second board-less hand gets told from the first here."""
        import orca_core.utils.cli as cli
        from orca_core import board_store

        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
        assert self.settle(self.hand(), Typist("a", "bench-2", "1")) == "bench-2"
        assert hand_ids.known_ids(tmp_path) == ["bench-2"]


class TestThePickerCannotBeMadeToSpin:
    """An answerer that never gives a usable answer must end the menu.

    Pinned because it did not: a caller passing one fixed answer -- which is
    how the hardware-version prompt is driven in tests -- hung the whole suite.
    """

    def test_a_fixed_unusable_answer_ends_it(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        typist = Typist(*(["21"] * 500))
        assert hand_ids.choose_id(ask=typist, show=typist.show,
                                  directory=tmp_path) is None
        assert len(typist.prompts) <= hand_ids.MAX_ATTEMPTS
        assert "Giving up" in typist.screen

    def test_a_terminal_user_still_gets_plenty_of_tries(self, tmp_path):
        hand_ids.add_id("alpha", tmp_path)
        typist = Typist("x", "y", "z", "99", "1")
        assert hand_ids.choose_id(ask=typist, show=typist.show,
                                  directory=tmp_path) == "alpha"

    def test_connecting_does_not_ask_which_hand_this_is(self):
        """Every script connects; only some care. The ask is theirs to make."""
        import inspect

        from orca_core.utils import cli

        assert "settle_hand_id" not in inspect.getsource(cli.connect_hand)


class TestACalibrationRunWritesTheHandsOwnFile:
    """Where a run persists, and what it must not lose on the way.

    Before a run, a hand reads and writes the model directory exactly as it
    always did. A run gives a named hand a file of its own, carrying across
    whatever was in the model directory first -- tactile offsets most of all,
    which no calibration run would put back.
    """

    def hand(self, tmp_path, monkeypatch, hand_id="ser-0000"):
        from orca_core import board_store
        from orca_core.hardware_hand import MockOrcaHand

        monkeypatch.setenv(board_store.CALIBRATIONS_DIR_ENV, str(tmp_path))
        hand = MockOrcaHand()
        hand._hand_id = hand_id
        return hand

    def test_connecting_alone_moves_nothing(self, tmp_path, monkeypatch):
        hand = self.hand(tmp_path, monkeypatch)
        before = hand.config.calibration_path
        assert "models" in before
        assert list(tmp_path.iterdir()) == []
        assert hand.config.calibration_path == before

    def test_a_run_gives_the_hand_a_file_of_its_own(self, tmp_path, monkeypatch):
        hand = self.hand(tmp_path, monkeypatch)
        hand._claim_hand_file()
        assert hand.config.calibration_path == str(tmp_path / "ser-0000.yaml")

    def test_what_was_in_the_model_directory_comes_across(self, tmp_path, monkeypatch):
        """Including everything a calibration run does not own.

        A touch hand's tactile offsets live in the same document and no
        calibration run would put them back, so they have to be carried.
        """
        import dataclasses

        from orca_core.utils.utils import read_yaml

        hand = self.hand(tmp_path, monkeypatch)
        legacy = tmp_path / "legacy-model-dir.yaml"
        legacy.write_text(
            "calibrated: true\n"
            "motor_limits:\n  1: [1.0, 2.0]\n"
            "sensor_offsets:\n  thumb:\n  - [1.0, 2.0, 3.0]\n"
            "taxel_noise_gates:\n  thumb: [0.1]\n"
        )
        hand.config = dataclasses.replace(hand.config, calibration_path=str(legacy))

        hand._claim_hand_file()
        carried = read_yaml(hand.config.calibration_path) or {}
        assert carried["sensor_offsets"] == {"thumb": [[1.0, 2.0, 3.0]]}
        assert carried["taxel_noise_gates"] == {"thumb": [0.1]}
        assert carried["motor_limits"] == {1: [1.0, 2.0]}

    def test_the_model_directory_copy_is_left_alone(self, tmp_path, monkeypatch):
        """So another machine, or an older core, still reads it."""
        import dataclasses

        hand = self.hand(tmp_path, monkeypatch)
        legacy = tmp_path / "legacy-model-dir.yaml"
        legacy.write_text("calibrated: true\n")
        hand.config = dataclasses.replace(hand.config, calibration_path=str(legacy))
        before = legacy.read_bytes()
        hand._claim_hand_file()
        assert legacy.read_bytes() == before

    def test_claiming_twice_does_not_overwrite_the_hands_file(
        self, tmp_path, monkeypatch
    ):
        from orca_core.utils.utils import read_yaml

        hand = self.hand(tmp_path, monkeypatch)
        hand._claim_hand_file()
        own = Path(hand.config.calibration_path)
        own.write_text("calibrated: true\nmotor_limits: {1: [9.0, 9.5]}\n")
        hand._claim_hand_file()
        assert (read_yaml(str(own)) or {})["motor_limits"][1] == [9.0, 9.5]

    def test_a_hand_with_no_name_keeps_writing_where_it_always_did(
        self, tmp_path, monkeypatch
    ):
        hand = self.hand(tmp_path, monkeypatch, hand_id=None)
        before = hand.config.calibration_path
        hand._claim_hand_file()
        assert hand.config.calibration_path == before
        assert list(tmp_path.iterdir()) == []

    def test_a_run_persists_to_the_hands_own_file(self, tmp_path, monkeypatch):
        """End to end: calibrate a mock hand, and see where it lands."""
        from orca_core.utils.utils import read_yaml

        hand = self.hand(tmp_path, monkeypatch)
        hand.connect()
        hand.calibrate(persist=True)
        hand.disconnect()

        own = tmp_path / "ser-0000.yaml"
        assert own.exists()
        document = read_yaml(str(own)) or {}
        assert document["calibrated"] is True
        assert document["motor_limits"]


class TestOnlyChosenNamesAreOffered:
    """A picker lists names somebody gave, not names the hardware reports.

    A serial or a board id is detected. Listing it would invite someone to pick
    the hand they already have, and for a board-less hand it would offer
    another hand's calibration.
    """

    def test_a_name_somebody_added_is_offered(self, tmp_path):
        hand_ids.add_id("left-bench", tmp_path)
        assert hand_ids.assigned_ids(tmp_path) == ["left-bench"]

    def test_a_file_keyed_by_a_serial_is_not(self, tmp_path):
        (tmp_path / "ser-0001.yaml").write_text("calibrated: true\n")
        assert hand_ids.known_ids(tmp_path) == ["ser-0001"]
        assert hand_ids.assigned_ids(tmp_path) == []

    def test_a_file_keyed_by_a_board_id_is_not(self, tmp_path):
        (tmp_path / "0123456789ABCDEF.bin").write_bytes(b"")
        assert hand_ids.assigned_ids(tmp_path) == []

    def test_a_name_is_marked_even_beside_a_calibration_that_exists(self, tmp_path):
        """Which is how a hand whose board died gets picked again by name."""
        (tmp_path / "ser-0001.yaml").write_text("calibrated: true\n")
        hand_ids.add_id("ser-0001", tmp_path)
        assert hand_ids.assigned_ids(tmp_path) == ["ser-0001"]
        assert (tmp_path / "ser-0001.yaml").read_text() == "calibrated: true\n"

    def test_auto_only_falls_back_to_a_chosen_name(self, tmp_path):
        """Another hand's serial must never be handed to a board-less one."""
        (tmp_path / "ser-0001.yaml").write_text("calibrated: true\n")
        assert hand_ids.auto_id(None, None, tmp_path) is None
        hand_ids.add_id("my-hand", tmp_path)
        assert hand_ids.auto_id(None, None, tmp_path) == "my-hand"

    def test_the_board_still_wins_over_any_chosen_name(self, tmp_path):
        hand_ids.add_id("my-hand", tmp_path)
        assert hand_ids.auto_id("ser-0000", None, tmp_path) == "ser-0000"

    def test_the_picker_offers_only_chosen_names(self, tmp_path):
        (tmp_path / "ser-0001.yaml").write_text("calibrated: true\n")
        hand_ids.add_id("chosen", tmp_path)
        typist = Typist("1")
        assert hand_ids.choose_id(ask=typist, show=typist.show,
                                  directory=tmp_path) == "chosen"
        assert "ser-0001" not in typist.screen


class TestARegisterThatCannotBeWritten:
    """An unwritable calibrations directory must not end the run.

    The hand cannot be named, but the calibration still has somewhere to go --
    the model directory, exactly as before hand ids existed -- so this reports
    and steps aside rather than raising into a script mid-prompt.
    """

    @pytest.fixture()
    def unwritable(self, tmp_path):
        locked = tmp_path / "locked"
        locked.mkdir(mode=0o500)
        yield locked / "calibrations"
        locked.chmod(0o700)

    def test_adding_says_what_went_wrong_instead_of_raising(self, unwritable):
        shown = []
        answers = iter(["a", "bench-left"])
        assert hand_ids.choose_id(ask=lambda *a: next(answers, "q"),
                                  directory=unwritable,
                                  show=shown.append) is None
        assert any("Could not register" in line for line in shown)

    def test_listing_an_unreachable_register_is_not_an_error(self, unwritable):
        assert hand_ids.known_ids(unwritable) == []
        assert hand_ids.assigned_ids(unwritable) == []
        assert hand_ids.auto_id(directory=unwritable) is None
