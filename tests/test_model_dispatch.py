from pathlib import Path

import pytest

from orca_core.constants import DEFAULT_MODEL_NAME, LATEST
from orca_core.hand_config import BaseHandConfig, OrcaHandConfig
from orca_core.utils import get_model_path


REPO_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = REPO_ROOT / "orca_core" / "models"


def test_latest_is_hand_selected_default_version():
    assert LATEST == "v1"


def test_default_hand_is_right():
    assert DEFAULT_MODEL_NAME == "orcahand_right"


def test_get_model_path_defaults_to_latest_right_hand():
    assert Path(get_model_path()) == MODELS_DIR / LATEST / DEFAULT_MODEL_NAME


def test_get_model_path_dispatches_explicit_version_and_model():
    assert Path(get_model_path(model_version="v1", model_name="orcahand_left")) == (
        MODELS_DIR / "v1" / "orcahand_left"
    )


def test_base_hand_config_dispatches_to_default_config_file():
    config = BaseHandConfig.from_config_path()
    assert Path(config.config_path) == MODELS_DIR / LATEST / DEFAULT_MODEL_NAME / "config.yaml"


def test_base_hand_config_dispatches_to_explicit_config_file():
    config = BaseHandConfig.from_config_path(
        model_version="v1",
        model_name="orcahand_left",
    )
    assert Path(config.config_path) == MODELS_DIR / "v1" / "orcahand_left" / "config.yaml"


def test_orca_hand_config_dispatches_calibration_path_for_selected_model():
    config = OrcaHandConfig.from_config_path(
        model_version="v1",
        model_name="orcahand_left",
    )
    assert Path(config.calibration_path) == (
        MODELS_DIR / "v1" / "orcahand_left" / "calibration.yaml"
    )


def test_unknown_model_raises_file_not_found():
    with pytest.raises(FileNotFoundError, match="Available models"):
        get_model_path(model_version="v1", model_name="missing_hand")
