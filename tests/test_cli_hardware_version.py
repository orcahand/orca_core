"""A hand of unknown hardware version (plain adapter, no pin) is asked for it
by the shared CLI helpers, and the answer is kept in its local_config.yaml --
the same place the console's picker writes."""

import glob
import os
import shutil
from argparse import Namespace

import orca_core
from orca_core.constants import HARDWARE_VERSION_V21
from orca_core.utils import cli

MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")
LEFT = sorted(glob.glob(os.path.join(MODELS, "*-left", "config.yaml")))[0]
THUMB = ("thumb_mcp", "thumb_dip")


def _mock_hand(tmp_path, **args):
    shutil.copy(LEFT, tmp_path / "config.yaml")
    ns = Namespace(config_path=str(tmp_path / "config.yaml"), mock=True, model_name=None,
                   engage_feedback=False, hardware_version=None)
    for k, v in args.items():
        setattr(ns, k, v)
    return cli.create_hand_from_args(ns, engage_sensors=False)


def _flipped(hand):
    return [hand.config.joint_inversion_dict[j] for j in THUMB]


def test_an_unknown_version_is_asked_for_and_the_answer_is_kept(tmp_path):
    hand = _mock_hand(tmp_path)
    base = _flipped(hand)
    asked = []
    try:
        cli.connect_hand(hand, ask=lambda prompt: asked.append(prompt) or "210")
        assert len(asked) == 1
        assert hand.config.hardware_version == HARDWARE_VERSION_V21
        assert _flipped(hand) == [not f for f in base], "the v2.1 left map is in force at once"
    finally:
        hand.disconnect()
    assert (tmp_path / "local_config.yaml").read_text().strip() == "hardware_version: 210"
    assert (tmp_path / "config.yaml").read_text() == open(LEFT, encoding="utf-8").read()


def test_enter_keeps_the_version_unknown_and_writes_nothing(tmp_path):
    hand = _mock_hand(tmp_path)
    try:
        cli.connect_hand(hand, ask=lambda prompt: "")
        assert hand.config.hardware_version is None
    finally:
        hand.disconnect()
    assert not (tmp_path / "local_config.yaml").exists()


def test_a_non_interactive_run_never_asks(tmp_path):
    hand = _mock_hand(tmp_path)
    try:
        cli.connect_hand(hand, interactive=False, ask=lambda prompt: (_ for _ in ()).throw(AssertionError("asked")))
        assert hand.config.hardware_version is None
    finally:
        hand.disconnect()


def test_a_known_version_is_not_asked_for(tmp_path):
    hand = _mock_hand(tmp_path, hardware_version=200)
    try:
        cli.connect_hand(hand, ask=lambda prompt: (_ for _ in ()).throw(AssertionError("asked")))
        assert hand.config.hardware_version == 200
    finally:
        hand.disconnect()


def test_the_flag_pins_for_the_run_and_for_the_next_one(tmp_path):
    hand = _mock_hand(tmp_path, hardware_version=210)
    hand.disconnect()
    assert hand.config.hardware_version == HARDWARE_VERSION_V21
    assert (tmp_path / "local_config.yaml").read_text().strip() == "hardware_version: 210"
    again = _mock_hand(tmp_path)   # no flag this time: the sidecar carries it
    try:
        assert again.config.hardware_version == HARDWARE_VERSION_V21
    finally:
        again.disconnect()


def test_a_bad_answer_is_asked_again(tmp_path):
    hand = _mock_hand(tmp_path)
    answers = iter(["twenty-one", "210"])
    try:
        cli.connect_hand(hand, ask=lambda prompt: next(answers))
        assert hand.config.hardware_version == HARDWARE_VERSION_V21
    finally:
        hand.disconnect()


def test_without_a_terminal_the_default_prompt_is_skipped(tmp_path, monkeypatch):
    """Piped or captured stdin cannot answer, so the run continues as unknown."""
    hand = _mock_hand(tmp_path)
    monkeypatch.setattr(cli.sys.stdin, "isatty", lambda: False)
    try:
        cli.connect_hand(hand)
        assert hand.config.hardware_version is None
    finally:
        hand.disconnect()


def test_the_board_decides_before_anyone_is_asked(tmp_path, monkeypatch):
    """A config path skips core's detection; connect_hand asks the board behind
    the motor port instead of the operator, and writes nothing."""
    from types import SimpleNamespace

    hand = _mock_hand(tmp_path)
    monkeypatch.setattr(cli, "board_info_for_port", lambda port: SimpleNamespace(hw_version=210))
    try:
        cli.connect_hand(hand, ask=lambda prompt: (_ for _ in ()).throw(AssertionError("asked")))
        assert hand.config.hardware_version == HARDWARE_VERSION_V21
    finally:
        hand.disconnect()
    assert not (tmp_path / "local_config.yaml").exists()


def test_a_pin_beats_the_board_and_the_clash_is_logged(tmp_path, monkeypatch, caplog):
    from types import SimpleNamespace

    hand = _mock_hand(tmp_path, hardware_version=200)
    monkeypatch.setattr(cli, "board_info_for_port", lambda port: SimpleNamespace(hw_version=210))
    try:
        with caplog.at_level("WARNING"):
            cli.connect_hand(hand, ask=lambda prompt: (_ for _ in ()).throw(AssertionError("asked")))
        assert hand.config.hardware_version == 200
        assert "pins hardware_version=200" in caplog.text and "board's 210" in caplog.text
    finally:
        hand.disconnect()


def test_no_board_still_falls_through_to_the_prompt(tmp_path, monkeypatch):
    hand = _mock_hand(tmp_path)
    monkeypatch.setattr(cli, "board_info_for_port", lambda port: None)
    try:
        cli.connect_hand(hand, ask=lambda prompt: "210")
        assert hand.config.hardware_version == HARDWARE_VERSION_V21
    finally:
        hand.disconnect()
