"""Config-driven taxel coordinates for tactile sensors.

Coordinates are loaded from sensor model config files in the models/ directory.
The finger-to-model mapping is configured in models/sensor_models.yaml.
"""

import os
from typing import TypedDict
import yaml

from orca_core.hardware.sensing.constants import FINGER_MODELS


class TaxelCoord(TypedDict):
    x: float
    y: float
    z: float


MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")

_model_cache: dict[str, list[TaxelCoord]] = {}


def _load_model_coordinates(model_name: str) -> list[TaxelCoord]:
    """Load coordinates for a sensor model from its config.yaml."""
    if model_name in _model_cache:
        return _model_cache[model_name]

    config_path = os.path.join(MODELS_DIR, model_name, "config.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    coords = [TaxelCoord(x=c["x"], y=c["y"], z=c["z"]) for c in config["coordinates"]]
    _model_cache[model_name] = coords
    return coords


def get_coordinates(finger: str) -> list[TaxelCoord]:
    """Get taxel coordinates for a finger.

    Args:
        finger: Finger name ('thumb', 'index', 'middle', 'ring', 'pinky')

    Returns:
        List of coordinate dicts with 'x', 'y', 'z' keys (in mm)
    """
    mapping = FINGER_MODELS
    model_name = mapping.get(finger)
    if model_name is None:
        return []
    return _load_model_coordinates(model_name)


def get_all_coordinates() -> dict[str, list[TaxelCoord]]:
    """Get taxel coordinates for all fingers.

    Returns:
        Dict mapping finger name to list of coordinate dicts
    """
    mapping = FINGER_MODELS
    return {finger: _load_model_coordinates(model) for finger, model in mapping.items()}


def get_taxel_counts() -> dict[str, int]:
    """Get the taxel count for each finger based on the current model mapping.

    Returns:
        Dict mapping finger name to number of taxels
    """
    mapping = FINGER_MODELS
    return {finger: len(_load_model_coordinates(model)) for finger, model in mapping.items()}
