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


def test_current_defaults_come_from_constants():
    import dataclasses
    from orca_core.constants import DEFAULT_CALIBRATION_CURRENT_MA, DEFAULT_MAX_CURRENT_MA

    defaults = {f.name: f.default for f in dataclasses.fields(OrcaHandConfig)}
    assert defaults["max_current"] == DEFAULT_MAX_CURRENT_MA == 300
    assert defaults["calibration_current"] == DEFAULT_CALIBRATION_CURRENT_MA == 300
    assert defaults["wrist_calibration_current"] is None


def test_wrist_calibration_current_defaults_to_the_finger_value(tmp_path):
    config = OrcaHandConfig.from_config_path(config_path=_right_config_with(tmp_path))
    assert config.wrist_calibration_current == config.calibration_current == 300


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
