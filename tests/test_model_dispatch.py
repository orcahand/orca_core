import tempfile
from pathlib import Path

import pytest
import yaml

from orca_core import LATEST_VERSION, canonical_joint_ids
from orca_core.constants import DEFAULT_MODEL_NAME
from orca_core.hand_config import BaseHandConfig, OrcaHandConfig
from orca_core.utils import get_model_path
from orca_core.utils.utils import read_yaml


REPO_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = REPO_ROOT / "orca_core" / "models"


def test_latest_is_hand_selected_default_version():
    assert LATEST_VERSION == "v1"


def test_default_hand_is_right():
    assert DEFAULT_MODEL_NAME == "orcahand_right"


@pytest.mark.parametrize(
    ("kwargs", "expected_path"),
    [
        ({}, MODELS_DIR / LATEST_VERSION / DEFAULT_MODEL_NAME),
        (
            {"model_version": "v1", "model_name": "orcahand_left"},
            MODELS_DIR / "v1" / "orcahand_left",
        ),
    ],
)
def test_get_model_path_dispatches_model_selection(kwargs, expected_path):
    assert Path(get_model_path(**kwargs)) == expected_path


@pytest.mark.parametrize(
    ("kwargs", "expected_path"),
    [
        ({}, MODELS_DIR / LATEST_VERSION / DEFAULT_MODEL_NAME / "config.yaml"),
        (
            {"model_version": "v1", "model_name": "orcahand_left"},
            MODELS_DIR / "v1" / "orcahand_left" / "config.yaml",
        ),
    ],
)
def test_base_hand_config_dispatches_config_file_path(kwargs, expected_path):
    config = BaseHandConfig.from_config_path(**kwargs)
    assert Path(config.config_path) == expected_path


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


@pytest.mark.parametrize(
    ("kwargs", "expected_version", "expected_model_name"),
    [
        ({}, LATEST_VERSION, DEFAULT_MODEL_NAME),
        ({"version": "v1", "type": "left"}, "v1", "orcahand_left"),
        ({"version": "v2", "type": "right"}, "v2", "orcahand_right"),
        ({"version": LATEST_VERSION}, LATEST_VERSION, DEFAULT_MODEL_NAME),
        ({"type": "right"}, LATEST_VERSION, "orcahand_right"),
    ],
)
def test_canonical_joint_ids_dispatches_to_selected_model(kwargs, expected_version, expected_model_name):
    config_path = MODELS_DIR / expected_version / expected_model_name / "config.yaml"
    config = read_yaml(str(config_path))
    assert canonical_joint_ids(**kwargs) == tuple(config["joint_ids"])


@pytest.fixture
def patched_models_dir(monkeypatch):
    with tempfile.TemporaryDirectory() as temp_dir:
        models_dir = Path(temp_dir) / "models"
        (models_dir / "v1" / "orcahand_right").mkdir(parents=True)
        (models_dir / "v1" / "orcahand_left").mkdir(parents=True)
        (models_dir / "v2" / "orcahand_right").mkdir(parents=True)

        def write_config(path: Path, joint_ids: list[str]) -> None:
            with open(path, "w", encoding="utf-8") as file:
                yaml.safe_dump({"joint_ids": joint_ids}, file, sort_keys=False)

        write_config(models_dir / "v1" / "orcahand_right" / "config.yaml", ["v1_right_joint"])
        write_config(models_dir / "v1" / "orcahand_left" / "config.yaml", ["v1_left_joint"])
        write_config(models_dir / "v2" / "orcahand_right" / "config.yaml", ["v2_right_joint"])

        monkeypatch.setattr("orca_core.utils.utils._get_models_dir", lambda: str(models_dir))
        yield models_dir, write_config


@pytest.mark.parametrize(
    ("kwargs", "expected"),
    [
        ({}, ("v1_right_joint",)),
        ({"type": "left"}, ("v1_left_joint",)),
        ({"version": "v2", "type": "right"}, ("v2_right_joint",)),
    ],
)
def test_canonical_joint_ids_tracks_underlying_config_contents(patched_models_dir, kwargs, expected):
    assert canonical_joint_ids(**kwargs) == expected


def test_canonical_joint_ids_reflects_updated_config_contents(patched_models_dir):
    models_dir, write_config = patched_models_dir
    write_config(
        models_dir / "v1" / "orcahand_right" / "config.yaml",
        ["updated_joint_a", "updated_joint_b"],
    )
    assert canonical_joint_ids() == ("updated_joint_a", "updated_joint_b")
