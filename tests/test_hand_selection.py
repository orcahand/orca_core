"""Choosing between attached hands, and keeping their state apart."""

import logging
import os

import pytest

from orca_core import (
    AmbiguousHandError,
    HandNotFoundError,
    HandSelector,
    MockOrcaHand,
    OrcaHand,
    hand_store,
    load_hand,
    load_hands,
)
from orca_core.constants import KNOWN_VIDS
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


def test_every_field_of_a_selector_must_match():
    """A caller pinning side as a cross-check against hand_id gets the wrong
    arm if the fields are ORed instead of ANDed."""
    with pytest.raises(HandNotFoundError):
        select_hand(TWO_RIGHT, HandSelector(hand_id="ser-0001", side="left"))


def test_a_selector_can_name_a_hand_by_its_motor_port():
    hands = [
        _detection("ser-0001", motor_port="/dev/cu.a-motor"),
        _detection("ser-0002", motor_port="/dev/cu.b-motor"),
    ]
    picked = select_hand(hands, HandSelector(motor_port="/dev/cu.b-motor"))
    assert picked.hand_id == "ser-0002"


def test_a_selection_failure_carries_what_was_attached():
    """A front-end renders its own advice from these rather than probing the
    bus a second time."""
    with pytest.raises(AmbiguousHandError) as ambiguous:
        select_hand(TWO_RIGHT)
    assert [d.hand_id for d in ambiguous.value.detections] == ["ser-0001", "ser-0002"]

    with pytest.raises(HandNotFoundError) as missing:
        select_hand(TWO_RIGHT, HandSelector(hand_id="ser-0009"))
    assert [d.hand_id for d in missing.value.detections] == ["ser-0001", "ser-0002"]


def test_load_hands_builds_only_the_hands_a_selector_matches(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, TWO_RIGHT)

    hands = load_hands(select=HandSelector(hand_id="ser-0002"))

    assert len(hands) == 1
    assert "ser-0002" in hands[0].config.calibration_path


def test_load_hands_refuses_a_selector_matching_nothing(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, TWO_RIGHT)

    with pytest.raises(HandNotFoundError):
        load_hands(select=HandSelector(hand_id="ser-0009"))


def test_load_hand_forwards_its_selector(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, TWO_RIGHT)

    hand = load_hand(select=HandSelector(hand_id="ser-0001"))

    assert "ser-0001" in hand.config.calibration_path


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


# ----- HandFleet -------------------------------------------------------

def test_fleet_only_returns_the_one_hand():
    from orca_core import HandFleet

    hand = object()
    assert HandFleet((hand,)).only() is hand


def test_fleet_only_refuses_zero_hands():
    from orca_core import HandFleet

    with pytest.raises(HandNotFoundError):
        HandFleet(()).only()


def test_fleet_only_refuses_several_hands():
    from orca_core import HandFleet

    with pytest.raises(AmbiguousHandError):
        HandFleet((object(), object())).only()


def test_fleet_run_keys_results_by_id_and_isolates_exceptions():
    from orca_core import HandFleet

    fleet = HandFleet((object(), object()), ("a", "b"))
    boom = ValueError("nope")

    def fn(hand):
        if hand is fleet.hands[1]:
            raise boom
        return "ok"

    for parallel in (False, True):
        results = fleet.run(fn, parallel=parallel)
        assert results == {"a": "ok", "b": boom}


def test_fleet_run_calls_every_hand_even_in_parallel():
    from orca_core import HandFleet

    hands = tuple(object() for _ in range(4))
    fleet = HandFleet(hands)
    seen = []

    results = fleet.run(lambda hand: seen.append(hand) or hand, parallel=True)
    assert sorted(id(h) for h in seen) == sorted(id(h) for h in hands)
    assert set(results.values()) == set(hands)


def test_load_hands_returns_a_fleet_iterable_and_sized_like_a_list(monkeypatch, tmp_path):
    import orca_core.hand_factory as hand_factory
    from orca_core import HandFleet

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: TWO_RIGHT)

    fleet = load_hands()
    assert isinstance(fleet, HandFleet)
    assert len(fleet) == 2
    assert fleet.ids == ("ser-0001", "ser-0002")
    assert [type(h) for h in fleet] == [OrcaHand, OrcaHand]


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


# ----- explicit configs -----------------------------------------------------

def test_load_hands_configs_never_probes_the_bus(monkeypatch, tmp_path):
    """configs= names each hand explicitly, so there is nothing to detect."""
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))

    def explode():
        raise AssertionError("must not probe hardware")

    monkeypatch.setattr(hand_factory, "detect_hands", explode)

    fleet = load_hands(configs=[_model(), _model()], mock=True)

    assert len(fleet) == 2
    assert [type(h) for h in fleet] == [MockOrcaHand, MockOrcaHand]


def test_load_hands_configs_has_no_upper_bound():
    fleet = load_hands(configs=[_model()] * 5, mock=True)
    assert len(fleet) == 5


def test_load_hands_configs_and_select_are_mutually_exclusive():
    with pytest.raises(ValueError, match="configs"):
        load_hands(configs=[_model()], select=HandSelector(hand_id="ser-0001"))


def test_load_hands_configs_resolve_each_hands_own_pinned_port(monkeypatch, tmp_path):
    """configs= is built on the existing single-hand load_hand(), so two real
    legacy hands detect_hands() can't tell apart (see
    test_two_ambiguous_legacy_hands_still_load_from_their_own_pinned_configs)
    still resolve cleanly from their own pinned ports — nothing new to build
    for real hardware disambiguation."""
    from tests._helpers import fake_serial_port

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    feetech_vid = KNOWN_VIDS["feetech"][0]
    monkeypatch.setattr(
        "serial.tools.list_ports.comports",
        lambda: [
            fake_serial_port("/dev/cu.a-motor", feetech_vid),
            fake_serial_port("/dev/cu.b-motor", feetech_vid),
        ],
    )

    fleet = load_hands(configs=[
        _pinned_config(tmp_path, "/dev/cu.a-motor"),
        _pinned_config(tmp_path, "/dev/cu.b-motor"),
    ])

    assert [h.config.port for h in fleet] == ["/dev/cu.a-motor", "/dev/cu.b-motor"]


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


def test_a_legacy_hand_selects_and_binds_its_own_store(monkeypatch, tmp_path):
    """A legacy hand's HandDetection is unremarkable — motor_port set, no
    identity — so it must flow through load_hand()/the store exactly like
    any provisioned hand, with no special-casing anywhere downstream."""
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    legacy = HandDetection(
        model_name="orcahand-touch-right",
        side="right",
        has_tactile=True,
        has_encoders=False,
        motor_port="/dev/cu.feetech",
        hand_id="legacy-5B79030513",
        board_id="legacy-5B79030513",
        side_source="default",
    )
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [legacy])

    hand = load_hand(select=HandSelector(hand_id="legacy-5B79030513"))

    assert hand.config.port == "/dev/cu.feetech"
    assert "legacy-5B79030513" in hand.config.calibration_path


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


def test_the_suite_never_writes_into_the_invoking_users_home(monkeypatch, tmp_path):
    """The store is keyed on ids the fixtures invent, and a physically
    attached hand may answer to one of them. Nothing here sets ORCA_HOME, so
    this passes only while the autouse fence in conftest does."""
    import orca_core.hand_factory as hand_factory

    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [_detection("ser-0001")])

    load_hand()

    leaked = sorted(str(f.relative_to(home)) for f in home.rglob("*") if f.is_file())
    assert not leaked, leaked


# ----- a named config still names only the model ---------------------------

def _detected(monkeypatch, detections):
    import orca_core.hand_factory as hand_factory

    monkeypatch.setattr(hand_factory, "detect_hands", lambda: list(detections))


def _model(name="orcahand-right"):
    import orca_core

    return os.path.join(
        os.path.dirname(orca_core.__file__), "models", "v2", name, "config.yaml"
    )


def test_a_named_config_still_binds_the_hand_store(monkeypatch, tmp_path):
    """orca_ui names a config at every call site. Without this its
    calibrations land beside the packaged model, where the next hand of the
    same model reads them as its own."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [_detection("ser-0001")])

    hand = load_hand(config_path=_model())

    assert "ser-0001" in hand.config.calibration_path
    assert str(tmp_path) in hand.config.calibration_path


def test_a_named_config_does_not_license_guessing_between_hands(
        monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, TWO_RIGHT)

    with pytest.raises(AmbiguousHandError):
        load_hand(config_path=_model())


def test_select_is_honoured_next_to_a_named_config(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, TWO_RIGHT)

    hand = load_hand(config_path=_model(), select=HandSelector(hand_id="ser-0002"))

    assert "ser-0002" in hand.config.calibration_path


def test_select_cannot_be_combined_with_a_mock_hand(monkeypatch):
    with pytest.raises(ValueError, match="mock"):
        load_hand(mock=True, select=HandSelector(hand_id="ser-0001"))


def test_a_named_model_does_not_come_back_as_the_detected_one(
        monkeypatch, tmp_path):
    """Detection resolves the hand, not the model — otherwise naming a model
    would be silently overruled by whatever is plugged in."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [_detection("ser-0001", model="orcahand-full-right")])

    hand = load_hand(model_name="orcahand-right")

    assert hand.config.config_path.endswith("orcahand-right/config.yaml")


def _pinned_config(tmp_path, port):
    config = tmp_path / f"pinned{port.replace('/', '_')}"
    config.mkdir()
    text = open(_model(), encoding="utf-8").read().replace(
        "port: auto", f"port: {port}", 1
    )
    (config / "config.yaml").write_text(text, encoding="utf-8")
    return str(config / "config.yaml")


def test_a_pinned_port_is_not_overwritten_by_detection(monkeypatch, tmp_path):
    """Pinning port: is the documented way to nail a hand to a bus; detection
    replacing it swaps one silent wrong behaviour for another."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [_detection("ser-0001", motor_port="/dev/cu.pinned")])

    hand = load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.pinned"))

    assert hand.config.port == "/dev/cu.pinned"


def test_a_pin_contradicting_the_selected_hand_is_refused(monkeypatch, tmp_path):
    """Keeping either one crosses the pair: the pinned bus with the selected
    hand's calibration, or the selected hand driven through a bus it does not
    own."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [
        _detection("ser-0001", motor_port="/dev/cu.a-motor"),
        _detection("ser-0002", motor_port="/dev/cu.b-motor"),
    ])

    with pytest.raises(ValueError, match="one hand's motors under the other"):
        load_hand(
            config_path=_pinned_config(tmp_path, "/dev/cu.b-motor"),
            select=HandSelector(hand_id="ser-0001"),
        )


def test_a_pinned_port_selects_the_hand_whose_bus_it_is(monkeypatch, tmp_path):
    """Otherwise the motors come from one hand and the calibration from
    whichever hand detection happened to pick."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [
        _detection("ser-0001", motor_port="/dev/cu.a-motor"),
        _detection("ser-0002", motor_port="/dev/cu.b-motor"),
    ])

    hand = load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.b-motor"))

    assert hand.config.port == "/dev/cu.b-motor"
    assert "ser-0002" in hand.config.calibration_path


def test_a_pinned_port_that_is_nobodys_bus_is_a_configuration_error(
        monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [_detection("ser-0001", motor_port="/dev/cu.a-motor")])

    with pytest.raises(HandNotFoundError, match="/dev/cu.gone"):
        load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.gone"))


def test_two_ambiguous_legacy_hands_still_load_from_their_own_pinned_configs(
        monkeypatch, tmp_path):
    """Two bare Feetech/Dynamixel adapters with no controller board can't be
    told apart by detect_hands() (see test_hand_detection.py), which leaves
    both undetected rather than guess. An operator who wired each one up
    themselves and pinned its port in its own config.yaml has already
    resolved that ambiguity by hand — refusing them too would make two old
    hands unusable together even though nothing is actually at risk of being
    crossed."""
    from tests._helpers import fake_serial_port

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    feetech_vid = KNOWN_VIDS["feetech"][0]
    monkeypatch.setattr(
        "serial.tools.list_ports.comports",
        lambda: [
            fake_serial_port("/dev/cu.a-motor", feetech_vid),
            fake_serial_port("/dev/cu.b-motor", feetech_vid),
        ],
    )

    hand_a = load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.a-motor"))
    hand_b = load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.b-motor"))

    assert hand_a.config.port == "/dev/cu.a-motor"
    assert hand_b.config.port == "/dev/cu.b-motor"


def test_a_pinned_port_naming_nothing_plugged_in_is_still_refused(monkeypatch, tmp_path):
    """The graceful fallback only covers a port detection left ambiguous —
    not a config pinning a port with nothing there at all."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [])

    with pytest.raises(HandNotFoundError, match="no hand is attached"):
        load_hand(config_path=_pinned_config(tmp_path, "/dev/cu.gone"))


def test_a_caller_supplied_motor_port_selector_gets_the_same_trust(monkeypatch, tmp_path):
    """orca_ui's own connect ladder (orca_ui/hand/sessions.py _build_hand)
    never relies on _pinned_port inference — it always passes
    select=HandSelector(motor_port=...) itself, once its own hardware probe
    (which reads the same pinned config field) has resolved a port. That
    caller-supplied selector deserves exactly the same trust as one derived
    internally from a config's own port: pin — both name an exact device
    path the operator already resolved by hand, and detect_hands() can leave
    it unconfirmed for the same reason (an ambiguous legacy sibling)."""
    from tests._helpers import fake_serial_port

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    feetech_vid = KNOWN_VIDS["feetech"][0]
    monkeypatch.setattr(
        "serial.tools.list_ports.comports",
        lambda: [
            fake_serial_port("/dev/cu.a-motor", feetech_vid),
            fake_serial_port("/dev/cu.b-motor", feetech_vid),
        ],
    )

    hand = load_hand(
        config_path=_pinned_config(tmp_path, "/dev/cu.a-motor"),
        select=HandSelector(motor_port="/dev/cu.a-motor"),
    )

    assert hand.config.port == "/dev/cu.a-motor"


def test_a_motor_port_selector_contradicted_by_a_detected_hand_still_raises(
        monkeypatch, tmp_path):
    """A detection did answer at this exact port, just for a hand that some
    other selector field rules out — that is a genuine conflict, not the
    ambiguous-legacy-sibling case, and must not be papered over."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    _detected(monkeypatch, [_detection("ser-0001", motor_port="/dev/cu.a-motor")])

    with pytest.raises(HandNotFoundError):
        load_hand(
            config_path=_model(),
            select=HandSelector(motor_port="/dev/cu.a-motor", hand_id="ser-0001", side="left"),
        )


# ----- adoption is a one-shot, recorded event -------------------------------

def _plausible_calibration(path):
    """A calibration complete enough to survive OrcaHand._sanity_check.

    A skeletal ``{calibrated: true}`` is demoted for its missing motor limits,
    so it would make these tests pass for the wrong reason.
    """
    import yaml

    path.write_text(yaml.dump({
        "motor_limits": {m: [0.0, 3.0] for m in range(1, 18)},
        "joint_to_motor_ratios": {m: 0.02 for m in range(1, 18)},
        "wrist_calibrated": True,
        "calibrated": True,
    }), encoding="utf-8")
    return str(path)


def test_only_the_first_hand_adopts_the_calibration_beside_the_model(
        monkeypatch, tmp_path):
    """It was measured on one physical hand. A second hand inheriting it
    reports itself calibrated and skips its own hardstop sweep."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path / "store"))
    legacy = _plausible_calibration(tmp_path / "calibration.yaml")

    first = hand_store.resolve_calibration_path("ser-0001", legacy)
    second = hand_store.resolve_calibration_path("ser-0002", legacy)

    assert "ser-0001" in first
    assert os.path.exists(first)
    assert not os.path.exists(second), "the second hand inherited the first's limits"


def test_a_second_hand_is_not_marked_calibrated_by_the_first(
        monkeypatch, tmp_path):
    import orca_core.hand_factory as hand_factory

    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path / "store"))
    model = tmp_path / "model"
    model.mkdir()
    import shutil
    shutil.copy(_model(), model / "config.yaml")
    _plausible_calibration(model / "calibration.yaml")
    config_path = str(model / "config.yaml")

    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [_detection("ser-0001")])
    first = load_hand(config_path=config_path)
    monkeypatch.setattr(hand_factory, "detect_hands", lambda: [_detection("ser-0002")])
    second = load_hand(config_path=config_path)

    assert first.calibration.calibrated is True
    assert second.calibration.calibrated is False


def test_adoption_survives_the_board_being_given_a_serial(monkeypatch, tmp_path):
    """Provisioning changes the hand id from the board id to the serial. The
    calibration measured before that must not be orphaned."""
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path / "store"))
    board = "BID-0001"

    hand_store.record_identity(board, {"board_id": board})
    _plausible_calibration(
        __import__("pathlib").Path(hand_store.calibration_path(board))
    )

    resolved = hand_store.resolve_calibration_path(
        "ser-0001", str(tmp_path / "absent.yaml"), board_id=board
    )

    assert "ser-0001" in resolved, "re-keying the hand orphaned its calibration"
    assert os.path.exists(resolved)
    assert os.path.exists(hand_store.calibration_path(board)), "adoption moved it"


def test_ids_that_differ_only_in_punctuation_keep_separate_directories(
        monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    dirs = {hand_store.hand_dir(i) for i in ("ser 1", "ser/1", "ser+1")}
    assert len(dirs) == 3


def test_a_plain_id_names_its_directory_as_it_reads(monkeypatch, tmp_path):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    assert os.path.basename(hand_store.hand_dir("ser-0001")) == "ser-0001"
