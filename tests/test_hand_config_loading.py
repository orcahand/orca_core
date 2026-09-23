"""Config-loading edge cases: empty config files and sensors.baudrate values."""

import os
import shutil

import pytest
import yaml

from orca_core.hand_config import (
    HandConfigValidationError,
    OrcaHandConfig,
    OrcaHandTouchConfig,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOUCH_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-touch-right", "config.yaml"
)


def test_empty_config_yaml_raises_clear_error(tmp_path):
    config_path = tmp_path / "config.yaml"
    config_path.write_text("", encoding="utf-8")
    with pytest.raises(HandConfigValidationError, match="is empty"):
        OrcaHandConfig.from_config_path(config_path=str(config_path))


def test_comments_only_config_yaml_raises_clear_error(tmp_path):
    config_path = tmp_path / "config.yaml"
    config_path.write_text("# truncated by an interrupted write\n", encoding="utf-8")
    with pytest.raises(HandConfigValidationError, match="config.yaml"):
        OrcaHandTouchConfig.from_config_path(config_path=str(config_path))


def _touch_config_with_baudrate(tmp_path, baudrate):
    config_path = tmp_path / "config.yaml"
    shutil.copy(TOUCH_CONFIG, config_path)
    with open(config_path) as f:
        doc = yaml.safe_load(f)
    doc.setdefault("sensors", {})["baudrate"] = baudrate
    with open(config_path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return config_path


def test_sensors_baudrate_auto_loads_as_auto(tmp_path):
    config_path = _touch_config_with_baudrate(tmp_path, "auto")
    config = OrcaHandTouchConfig.from_config_path(config_path=str(config_path))
    assert config.sensor_baudrate == "auto"


def test_sensors_baudrate_int_loads_as_int(tmp_path):
    config_path = _touch_config_with_baudrate(tmp_path, 921600)
    config = OrcaHandTouchConfig.from_config_path(config_path=str(config_path))
    assert config.sensor_baudrate == 921600


def test_write_yaml_atomic_exported_from_utils():
    from orca_core.utils import write_yaml_atomic
    from orca_core.utils.utils import write_yaml_atomic as impl

    assert write_yaml_atomic is impl


# ----- goal-current defaults ------------------------------------------------

RIGHT_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-right", "config.yaml"
)


def _right_config_with(tmp_path, **overrides):
    with open(RIGHT_CONFIG, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    raw.update(overrides)
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return str(path)


class FamilyClient:
    """Stand-in for a motor client class: the family's current defaults."""

    default_max_current_ma = 640
    default_calibration_current_ma = 610


def test_currents_default_to_the_family_sentinel():
    import dataclasses

    defaults = {f.name: f.default for f in dataclasses.fields(OrcaHandConfig)}
    assert defaults["max_current"] == defaults["calibration_current"] == "default"
    assert defaults["wrist_calibration_current"] is None


def test_packaged_v2_configs_leave_currents_to_the_family():
    import glob

    for path in sorted(glob.glob(os.path.join(REPO_ROOT, "orca_core", "models", "v2", "*", "config.yaml"))):
        config = OrcaHandConfig.from_config_path(config_path=path)
        assert (config.max_current, config.calibration_current, config.wrist_calibration_current) == (
            "default", "default", "default"), path
        assert not config.currents_resolved


@pytest.mark.parametrize("spelling", ["default", "Default", " DEFAULT "])
def test_default_is_read_case_insensitively(tmp_path, spelling):
    config = OrcaHandConfig.from_config_path(
        config_path=_right_config_with(tmp_path, max_current=spelling))
    assert config.max_current == "default"


@pytest.mark.parametrize("value", ["lots", "300mA", None, [300]])
def test_a_current_that_is_neither_a_number_nor_default_is_rejected(tmp_path, value):
    path = _right_config_with(tmp_path, max_current=value)
    with pytest.raises(HandConfigValidationError, match="number of mA or 'default'"):
        OrcaHandConfig.from_config_path(config_path=path)


def test_wrist_calibration_current_defaults_to_the_finger_value(tmp_path):
    config = OrcaHandConfig.from_config_path(
        config_path=_right_config_with(tmp_path, calibration_current=280))
    assert config.wrist_calibration_current == config.calibration_current == 280


def test_family_currents_fill_every_default(tmp_path):
    config = OrcaHandConfig.from_config_path(config_path=_right_config_with(tmp_path))
    resolved = config.with_family_currents(FamilyClient)
    assert (resolved.max_current, resolved.calibration_current, resolved.wrist_calibration_current) == (640, 610, 610)
    assert resolved.currents_resolved
    assert resolved.with_family_currents(FamilyClient) is resolved


def test_pinned_currents_survive_family_resolution(tmp_path):
    config = OrcaHandConfig.from_config_path(
        config_path=_right_config_with(tmp_path, max_current=700, wrist_calibration_current=350))
    resolved = config.with_family_currents(FamilyClient)
    assert (resolved.max_current, resolved.calibration_current, resolved.wrist_calibration_current) == (700, 610, 350)


def test_a_pinned_max_current_below_the_family_calibration_current_is_rejected_on_resolution(tmp_path):
    config = OrcaHandConfig.from_config_path(
        config_path=_right_config_with(tmp_path, max_current=500))
    with pytest.raises(HandConfigValidationError, match="Max current"):
        config.with_family_currents(FamilyClient)


def test_pinned_max_current_below_pinned_calibration_current_is_rejected_at_load(tmp_path):
    path = _right_config_with(tmp_path, max_current=200, calibration_current=250)
    with pytest.raises(HandConfigValidationError, match="Max current"):
        OrcaHandConfig.from_config_path(config_path=path)


def test_explicit_wrist_calibration_current_is_kept(tmp_path):
    path = _right_config_with(tmp_path, wrist_calibration_current=250)
    config = OrcaHandConfig.from_config_path(config_path=path)
    assert config.wrist_calibration_current == 250


@pytest.mark.parametrize(
    "field, value",
    [("calibration_current", 0), ("wrist_calibration_current", -5), ("max_current", 0)],
)
def test_non_positive_currents_are_rejected(tmp_path, field, value):
    path = _right_config_with(tmp_path, **{field: value})
    with pytest.raises(HandConfigValidationError, match="positive number of mA"):
        OrcaHandConfig.from_config_path(config_path=path)
