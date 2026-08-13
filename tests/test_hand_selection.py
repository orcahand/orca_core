"""Choosing between attached hands, and keeping their state apart."""

import logging
import os

import pytest

from orca_core import (
    AmbiguousHandError,
    HandNotFoundError,
    HandSelector,
    OrcaHand,
    hand_store,
    load_hand,
    load_hands,
)
from orca_core.hand_factory import HandDetection, select_hand


def _detection(hand_id, side="right", model=None, **kwargs):
    return HandDetection(
        model_name=model or f"orcahand-{side}",
        side=side,
        has_tactile=False,
        has_encoders=False,
        hand_id=hand_id,
        board_id=f"BID-{hand_id}",
        side_source="board",
        **kwargs,
    )


TWO_RIGHT = [_detection("ser-0001"), _detection("ser-0002")]


# ----- selection -----------------------------------------------------------

def test_one_attached_hand_needs_no_selector():
    only = _detection("ser-0001")
    assert select_hand([only]) is only


def test_two_hands_refuse_to_be_guessed_between():
    with pytest.raises(AmbiguousHandError) as excinfo:
        select_hand(TWO_RIGHT)
    message = str(excinfo.value)
    assert "ser-0001" in message and "ser-0002" in message, (
        "an ambiguity error that does not name the candidates just moves the "
        "problem to the user"
    )


@pytest.mark.parametrize("selector,expected", [
    (HandSelector(hand_id="ser-0002"), "ser-0002"),
    (HandSelector(board_id="BID-ser-0001"), "ser-0001"),
])
def test_a_selector_picks_one_hand(selector, expected):
    assert select_hand(TWO_RIGHT, selector).hand_id == expected


def test_side_cannot_separate_two_hands_of_the_same_side():
    """Two right hands are the case the store exists for; side is no help."""
    with pytest.raises(AmbiguousHandError):
        select_hand(TWO_RIGHT, HandSelector(side="right"))


def test_side_separates_a_bimanual_pair():
    hands = [_detection("ser-0001", side="right"), _detection("ser-0002", side="left")]
    assert select_hand(hands, HandSelector(side="left")).hand_id == "ser-0002"


def test_a_selector_matching_nothing_names_what_is_attached():
    with pytest.raises(HandNotFoundError) as excinfo:
        select_hand(TWO_RIGHT, HandSelector(hand_id="ser-9999"))
    assert "ser-0001" in str(excinfo.value)


def test_no_hand_attached_says_so_plainly():
    with pytest.raises(HandNotFoundError, match="no hand is attached"):
        select_hand([], HandSelector(side="left"))


def test_load_hand_still_works_with_nothing_attached(monkeypatch):
    """Inspecting a config off the bench must not require hardware."""
    import orca_core.hand_factory as hand_factory

    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [])
    hand = load_hand()
    assert type(hand) is OrcaHand
    assert hand.config.port == "auto"


def test_load_hand_refuses_to_pick_between_two_attached_hands(monkeypatch):
    import orca_core.hand_factory as hand_factory

    monkeypatch.setattr(hand_factory, "detect_hands", lambda: TWO_RIGHT)
    with pytest.raises(AmbiguousHandError):
        load_hand()


def test_load_hands_builds_every_attached_hand(monkeypatch, tmp_path):
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: TWO_RIGHT)

    hands = load_hands()
    assert len(hands) == 2
    assert all(type(h) is OrcaHand for h in hands)


def test_load_hands_detects_once_however_many_hands(monkeypatch, tmp_path):
    """Probing opens ports, so a per-hand re-probe would multiply that cost
    and race with the hands it just described."""
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    calls = []

    def counted():
        calls.append(1)
        return TWO_RIGHT

    monkeypatch.setattr(hand_factory, "detect_hands", counted)
    load_hands()
    assert len(calls) == 1


# ----- per-hand state ------------------------------------------------------

def test_two_identical_hands_get_separate_calibration(monkeypatch, tmp_path):
    """The whole point: two right hands resolve to one packaged model, so
    without a per-hand store the second to calibrate overwrites the first."""
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: TWO_RIGHT)

    first, second = load_hands()
    assert first.config.calibration_path != second.config.calibration_path
    assert "ser-0001" in first.config.calibration_path
    assert "ser-0002" in second.config.calibration_path


def test_calibration_lands_outside_the_installed_package(monkeypatch, tmp_path):
    """A wheel upgrade replaces the package; a read-only install cannot be
    written at all. Neither may cost a hand its calibration."""
    import orca_core
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [TWO_RIGHT[0]])

    hand = load_hand()
    package_dir = os.path.dirname(orca_core.__file__)
    assert not hand.config.calibration_path.startswith(package_dir)


def test_an_explicit_calibration_path_beats_the_store(monkeypatch, tmp_path):
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [TWO_RIGHT[0]])

    mine = tmp_path / "mine.yaml"
    mine.write_text("{}\n")
    hand = load_hand(calibration_path=str(mine))
    assert hand.config.calibration_path == str(mine)


def test_a_hand_keeps_the_calibration_it_had_before_the_store(monkeypatch, tmp_path):
    """Adopting an existing calibration matters more than tidiness: losing it
    means re-running a hardstop sweep on a built hand."""
    import yaml

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))

    legacy_model = tmp_path / "model"
    legacy_model.mkdir()
    (legacy_model / "calibration.yaml").write_text(
        yaml.safe_dump({"calibrated": True, "motor_limits": {1: [0.0, 1.0]}})
    )
    resolved = hand_store.resolve_calibration_path(
        "ser-0001", str(legacy_model / "calibration.yaml")
    )
    assert resolved == hand_store.calibration_path("ser-0001")
    assert yaml.safe_load(open(resolved))["calibrated"] is True


def test_the_store_wins_once_it_holds_a_calibration(tmp_path, monkeypatch):
    import yaml

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    stored = hand_store.calibration_path("ser-0001")
    os.makedirs(os.path.dirname(stored), exist_ok=True)
    with open(stored, "w") as f:
        yaml.safe_dump({"calibrated": True, "source": "store"}, f)

    legacy = tmp_path / "model" / "calibration.yaml"
    os.makedirs(legacy.parent, exist_ok=True)
    legacy.write_text(yaml.safe_dump({"calibrated": True, "source": "package"}))

    resolved = hand_store.resolve_calibration_path("ser-0001", str(legacy))
    assert yaml.safe_load(open(resolved))["source"] == "store"


def test_the_store_records_which_hand_a_directory_belongs_to(monkeypatch, tmp_path):
    import yaml

    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [TWO_RIGHT[0]])
    load_hand()

    identity = tmp_path / "hands" / "ser-0001" / "identity.yaml"
    recorded = yaml.safe_load(identity.read_text())
    assert recorded["board_id"] == "BID-ser-0001"
    assert recorded["side"] == "right"


def test_a_hand_id_cannot_escape_the_store_directory(monkeypatch, tmp_path):
    """Ids come from a board's own reply, so they are not a trusted path."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    hostile = hand_store.hand_dir("../../etc/evil")
    assert os.path.dirname(os.path.realpath(hostile)) == os.path.realpath(
        hand_store.hands_root()
    )


def test_orca_home_can_be_moved_off_the_invoking_users_home(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path / "rig"))
    assert hand_store.hands_root().startswith(str(tmp_path / "rig"))
