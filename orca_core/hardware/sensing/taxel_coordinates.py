"""Config-driven taxel coordinates for tactile sensors.

Coordinates are loaded from sensor model config files in the models/ directory.
The finger-to-model mapping is configured in models/sensor_models.yaml.
"""

import os
from typing import TypedDict, Optional
import yaml


class TaxelCoord(TypedDict):
    x: float
    y: float
    z: float


MODELS_DIR = os.path.join(os.path.dirname(__file__), "models")
SENSOR_MODELS_CONFIG = os.path.join(MODELS_DIR, "sensor_models.yaml")

_model_cache: dict[str, list[TaxelCoord]] = {}
_finger_mapping_cache: Optional[dict[str, str]] = None


def _load_finger_mapping() -> dict[str, str]:
    """Load the finger-to-model mapping from sensor_models.yaml."""
    global _finger_mapping_cache
    if _finger_mapping_cache is not None:
        return _finger_mapping_cache

    with open(SENSOR_MODELS_CONFIG, "r") as f:
        config = yaml.safe_load(f)

    _finger_mapping_cache = config["finger_models"]
    return _finger_mapping_cache


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
    mapping = _load_finger_mapping()
    model_name = mapping.get(finger)
    if model_name is None:
        return []
    return _load_model_coordinates(model_name)


def get_all_coordinates() -> dict[str, list[TaxelCoord]]:
    """Get taxel coordinates for all fingers.

    Returns:
        Dict mapping finger name to list of coordinate dicts
    """
    mapping = _load_finger_mapping()
    return {finger: _load_model_coordinates(model) for finger, model in mapping.items()}


def get_taxel_counts() -> dict[str, int]:
    """Get the taxel count for each finger based on the current model mapping.

    Returns:
        Dict mapping finger name to number of taxels
    """
    mapping = _load_finger_mapping()
    return {finger: len(_load_model_coordinates(model)) for finger, model in mapping.items()}
