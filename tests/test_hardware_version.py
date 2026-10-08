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
from orca_core.hand_config import HandConfigValidationError, OrcaHandConfig

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


def test_the_packaged_left_configs_ship_on_auto():
    for path in LEFT:
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
