"""A later hardware revision of a model may carry a second joint-to-motor map.

v2.1 left hands route the thumb MCP and DIP tendons the other way, so those two
motors turn opposite to v2.0 for the same joint motion. The config carries both
maps; the board's provisioned hardware version decides which is in force, once,
when the config is built -- so nothing downstream has to know a revision exists.
"""

import glob
import os
from types import SimpleNamespace

import pytest
import yaml

from orca_core import hand_factory, load_hand
from orca_core.constants import HARDWARE_VERSION_V21
from orca_core.hand_config import (
    HandConfigValidationError,
    OrcaHandConfig,
    OrcaHandTouchConfig,
    write_hardware_version,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V2_MODELS = os.path.join(REPO_ROOT, "orca_core", "models", "v2")
LEFT = sorted(glob.glob(os.path.join(V2_MODELS, "*-left", "config.yaml")))
RIGHT = sorted(glob.glob(os.path.join(V2_MODELS, "*-right", "config.yaml")))
FLIPPED = {"thumb_mcp", "thumb_dip"}


def _config_with(tmp_path, source, **overrides):
    with open(source, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    raw.update(overrides)
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return str(path)


# --- what the packaged configs say ---------------------------------------
# These are the guard against the mistake a revision invites: an edit to one
# model's map that quietly drives the wrong motor on every hand of that kind.


def test_there_are_left_and_right_models_to_check():
    assert len(LEFT) == 4 and len(RIGHT) == 4


@pytest.mark.parametrize("path", LEFT, ids=[os.path.basename(os.path.dirname(p)) for p in LEFT])
def test_every_left_model_carries_a_v21_map(path):
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.joint_to_motor_map_v21, "left model has no v2.1 map"


@pytest.mark.parametrize("path", LEFT, ids=[os.path.basename(os.path.dirname(p)) for p in LEFT])
def test_the_v21_map_differs_from_the_base_map_only_in_the_thumb_signs(path):
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert set(config.joint_to_motor_map_v21) == set(config.joint_to_motor_map)
    for joint, signed in config.joint_to_motor_map_v21.items():
        assert abs(signed) == config.joint_to_motor_map[joint], joint
    resolved = config.with_hardware_version(HARDWARE_VERSION_V21)
    differing = {j for j in config.joint_ids
                 if resolved.joint_inversion_dict[j] != config.joint_inversion_dict[j]}
    assert differing == FLIPPED


@pytest.mark.parametrize("path", RIGHT, ids=[os.path.basename(os.path.dirname(p)) for p in RIGHT])
def test_no_right_model_carries_a_v21_map(path):
    """The wiring change is a left-hand change. A right config gaining this key
    would be an accident, and would silently apply on any right board
    provisioned as v2.1."""
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.joint_to_motor_map_v21 == {}
    resolved = config.with_hardware_version(HARDWARE_VERSION_V21)
    assert resolved.joint_inversion_dict == config.joint_inversion_dict


def test_every_model_ships_the_same_baseline_and_only_for_v21():
    """v2.0 hands have no travel baseline at all; every v2.1 variant shares one."""
    baselines = []
    for path in LEFT + RIGHT:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        assert "joint_motor_travel" not in raw, path
        baselines.append(raw["joint_motor_travel_v21"])
    assert all(b == baselines[0] for b in baselines)


# --- the travel features follow the version ------------------------------


@pytest.mark.parametrize("path", [LEFT[0], RIGHT[0]], ids=["left", "right"])
@pytest.mark.parametrize("version", [None, 2])
def test_an_older_or_unknown_hand_has_no_travel_baseline(path, version):
    config = OrcaHandConfig.from_config_path(config_path=path).with_hardware_version(version)
    assert not config.uses_motor_travel
    assert config.expected_motor_travel_deg("index_mcp") is None
    assert config.expected_motor_travel_deg("index_mcp", any_hardware=True) > 0


@pytest.mark.parametrize("path", [LEFT[0], RIGHT[0]], ids=["left", "right"])
def test_a_v21_hand_uses_the_packaged_baseline(path):
    config = OrcaHandConfig.from_config_path(config_path=path).with_hardware_version(
        HARDWARE_VERSION_V21)
    assert config.uses_motor_travel
    assert config.expected_motor_travel_deg("index_mcp") == pytest.approx(139.39)


def test_a_hands_own_measurement_wins_over_the_packaged_baseline(tmp_path):
    path = _config_with(tmp_path, RIGHT[0], hardware_version=21,
                        joint_motor_travel={"index_mcp": 120.0})
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.expected_motor_travel_deg("index_mcp") == pytest.approx(120.0)
    assert config.expected_motor_travel_deg("index_pip") == pytest.approx(131.04)


def test_a_pin_is_in_force_without_load_hand(tmp_path):
    """A hand built straight from the path still gets the pinned revision's map."""
    path = _config_with(tmp_path, LEFT[0], hardware_version=21)
    config = OrcaHandConfig.from_config_path(config_path=path)
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    assert all(config.joint_inversion_dict[j] != base.joint_inversion_dict[j] for j in FLIPPED)


def test_a_pin_is_in_force_for_a_tactile_config_built_from_the_path(tmp_path):
    source = os.path.join(V2_MODELS, "orcahand-touch-left", "config.yaml")
    path = _config_with(tmp_path, source, hardware_version=21)
    config = OrcaHandTouchConfig.from_config_path(config_path=path)
    base = OrcaHandTouchConfig.from_config_path(config_path=source)
    assert all(config.joint_inversion_dict[j] != base.joint_inversion_dict[j] for j in FLIPPED)


@pytest.mark.parametrize("has_line", [True, False], ids=["with auto line", "without the key"])
def test_a_written_pin_lives_beside_the_config_and_leaves_it_untouched(tmp_path, has_line):
    """The pin goes into local_config.yaml: config.yaml is never rewritten, so a
    packaged model stays exactly as shipped and a dev checkout stays clean."""
    original = open(RIGHT[0], encoding="utf-8").read()
    if not has_line:
        original = original.replace("hardware_version: auto\n", "")
    path = tmp_path / "config.yaml"
    path.write_text(original, encoding="utf-8")

    written = write_hardware_version(str(path), 21)
    assert written == str(tmp_path / "local_config.yaml")
    assert path.read_text(encoding="utf-8") == original
    assert yaml.safe_load(open(written)) == {"hardware_version": 21}
    assert OrcaHandConfig.from_config_path(config_path=str(path)).hardware_version == 21

    write_hardware_version(str(path), None)
    assert not os.path.exists(written), "an empty local config is removed"
    assert OrcaHandConfig.from_config_path(config_path=str(path)).hardware_version is None
    assert path.read_text(encoding="utf-8") == original


def test_the_local_config_beats_an_explicit_yaml_value_and_says_so(tmp_path, caplog):
    path = _config_with(tmp_path, LEFT[0], hardware_version=2)
    write_hardware_version(path, HARDWARE_VERSION_V21)
    with caplog.at_level("WARNING"):
        config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.hardware_version == HARDWARE_VERSION_V21
    assert "local_config.yaml sets hardware_version=21" in caplog.text


def test_the_local_config_ignores_keys_it_does_not_own(tmp_path, caplog):
    path = _config_with(tmp_path, RIGHT[0])
    (tmp_path / "local_config.yaml").write_text(
        yaml.safe_dump({"hardware_version": 21, "max_current": 50}), encoding="utf-8")
    with caplog.at_level("WARNING"):
        config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.hardware_version == 21
    assert config.max_current != 50
    assert "ignoring ['max_current']" in caplog.text


def test_the_packaged_configs_carry_no_local_config():
    for path in LEFT + RIGHT:
        assert not os.path.exists(os.path.join(os.path.dirname(path), "local_config.yaml")), path


# --- selecting the map --------------------------------------------------


def test_v21_hardware_puts_the_v21_map_in_force():
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    resolved = base.with_hardware_version(HARDWARE_VERSION_V21)

    assert resolved.hardware_version == HARDWARE_VERSION_V21
    # Same motor for every joint; only the two thumb directions changed.
    assert resolved.joint_to_motor_map == base.joint_to_motor_map
    for joint in base.joint_ids:
        flipped = joint in FLIPPED
        assert resolved.joint_inversion_dict[joint] == (
            base.joint_inversion_dict[joint] != flipped), joint


def test_v20_hardware_keeps_the_base_map():
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    resolved = base.with_hardware_version(2)
    assert resolved.hardware_version == 2
    assert resolved.joint_inversion_dict == base.joint_inversion_dict


def test_an_unknown_hardware_version_changes_nothing():
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    assert base.with_hardware_version(None) is base


def test_v21_on_a_model_without_the_map_only_records_the_version():
    base = OrcaHandConfig.from_config_path(config_path=RIGHT[0])
    resolved = base.with_hardware_version(HARDWARE_VERSION_V21)
    assert resolved.hardware_version == HARDWARE_VERSION_V21
    assert resolved.joint_inversion_dict == base.joint_inversion_dict


# --- shape of the v21 map -------------------------------------------------




def test_the_chosen_map_passes_the_same_checks_as_the_base_map(tmp_path):
    """Applying the v2.1 map rebuilds the config, so the ordinary checks run
    on it: a motor the hand does not have is caught then, like in any map."""
    with open(LEFT[0], encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    v21 = dict(raw["joint_to_motor_map_v21"])
    v21["thumb_dip"] = -99
    path = _config_with(tmp_path, LEFT[0], joint_to_motor_map_v21=v21)
    config = OrcaHandConfig.from_config_path(config_path=path)   # loads: not in force yet
    with pytest.raises(HandConfigValidationError, match="not in the motor IDs list"):
        config.with_hardware_version(HARDWARE_VERSION_V21)


def test_a_v21_map_that_is_not_a_mapping_is_rejected(tmp_path):
    path = _config_with(tmp_path, LEFT[0], joint_to_motor_map_v21=[1, 2, 3])
    with pytest.raises(HandConfigValidationError, match="must be a mapping"):
        OrcaHandConfig.from_config_path(config_path=path)


def test_a_pinned_hardware_version_selects_the_map(tmp_path):
    """A hand on a plain adapter has no board to ask; its owner pins the
    revision in their config copy, as they already pick the side by model."""
    path = _config_with(tmp_path, LEFT[0], hardware_version=21)
    hand = load_hand(config_path=path, mock=True)
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    assert hand.config.hardware_version == 21
    for joint in FLIPPED:
        assert hand.config.joint_inversion_dict[joint] != base.joint_inversion_dict[joint]


@pytest.mark.parametrize("spelling", ["auto", "Auto", " AUTO "])
def test_auto_means_not_pinned(tmp_path, spelling):
    path = _config_with(tmp_path, LEFT[0], hardware_version=spelling)
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.hardware_version is None


def test_every_packaged_config_ships_on_auto():
    for path in LEFT + RIGHT:
        assert OrcaHandConfig.from_config_path(config_path=path).hardware_version is None, path


@pytest.mark.parametrize("value", [-1, "x", True, 2.5, [21]])
def test_a_hardware_version_that_is_not_auto_or_a_whole_number_is_rejected(tmp_path, value):
    path = _config_with(tmp_path, LEFT[0], hardware_version=value)
    with pytest.raises(HandConfigValidationError, match="hardware_version"):
        OrcaHandConfig.from_config_path(config_path=path)


# --- how load_hand resolves it -------------------------------------------
# Precedence: a yaml pin, else the board (read by detection or handed in by a
# caller that skipped it -- the same source two ways), else nothing.


def _detection(hw):
    return SimpleNamespace(identity=SimpleNamespace(hw_version=hw))


def _pinned(hw):
    return SimpleNamespace(hardware_version=hw)


def test_the_board_is_used_when_nothing_is_pinned():
    assert hand_factory._resolve_hardware_version(_pinned(None), _detection(21), None) == 21


def test_a_handed_in_board_value_ranks_as_detection():
    """The console loads by config_path, skips detection, and hands the board
    value in. That is the board speaking through a different door, not an
    override, so a pin still wins over it."""
    assert hand_factory._resolve_hardware_version(_pinned(None), None, 21) == 21
    assert hand_factory._resolve_hardware_version(_pinned(2), None, 21) == 2


def test_a_pin_wins_over_the_board_and_says_so(caplog):
    with caplog.at_level("WARNING", logger="orca_core.hand_factory"):
        assert hand_factory._resolve_hardware_version(_pinned(21), _detection(2), None) == 21
    assert "pins hardware_version=21" in caplog.text
    assert "detected 2" in caplog.text


def test_a_pin_that_agrees_with_the_board_is_quiet(caplog):
    with caplog.at_level("WARNING", logger="orca_core.hand_factory"):
        assert hand_factory._resolve_hardware_version(_pinned(21), _detection(21), None) == 21
    assert "hardware_version" not in caplog.text


def test_a_pin_with_no_board_is_quiet(caplog):
    """The pin exists for hands with no board. Warning that 'the detected None
    is not used' would nag exactly the people using it as intended."""
    with caplog.at_level("WARNING", logger="orca_core.hand_factory"):
        assert hand_factory._resolve_hardware_version(_pinned(21), None, None) == 21
        assert hand_factory._resolve_hardware_version(
            _pinned(21), SimpleNamespace(identity=None), None) == 21
    assert "hardware_version" not in caplog.text


def test_nothing_known_resolves_to_nothing():
    assert hand_factory._resolve_hardware_version(_pinned(None), None, None) is None
    assert hand_factory._resolve_hardware_version(
        _pinned(None), SimpleNamespace(identity=None), None) is None


def test_load_hand_applies_a_handed_in_version_to_the_mock_hand():
    hand = load_hand(config_path=LEFT[0], mock=True,
                     detected_hardware_version=HARDWARE_VERSION_V21)
    assert hand.config.hardware_version == HARDWARE_VERSION_V21
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    for joint in FLIPPED:
        assert hand.config.joint_inversion_dict[joint] != base.joint_inversion_dict[joint]


def test_load_hand_without_a_version_keeps_the_base_map():
    hand = load_hand(config_path=LEFT[0], mock=True)
    assert hand.config.hardware_version is None
    base = OrcaHandConfig.from_config_path(config_path=LEFT[0])
    assert hand.config.joint_inversion_dict == base.joint_inversion_dict


@pytest.mark.parametrize("path", LEFT + RIGHT)
def test_only_a_v21_dynamixel_hand_calibrates_per_group(path):
    import dataclasses

    base = OrcaHandConfig.from_config_path(config_path=path)

    def resolved(version, family):
        config = base.with_hardware_version(version)
        return dataclasses.replace(config, motor_type=family, calibration_current=300,
                                   wrist_calibration_current=280)

    v21 = resolved(HARDWARE_VERSION_V21, "dynamixel")
    assert [v21.calibration_current_for(j) for j in
            ("index_abd", "thumb_cmc", "index_mcp", "thumb_mcp", "index_pip", "thumb_dip", "wrist")] \
        == [300, 300, 500, 500, 500, 500, 280]
    assert v21.retry_current_for_attempt(1, "index_mcp") == 550
    for config in (resolved(2, "dynamixel"), resolved(None, "dynamixel"),
                   resolved(HARDWARE_VERSION_V21, "feetech")):
        assert {config.calibration_current_for(j) for j in config.joint_ids if j != "wrist"} == {300}
