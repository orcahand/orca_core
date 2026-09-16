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
