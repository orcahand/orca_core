from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .utils import read_yaml


PROFILE_SCHEMA_VERSION = "1.0"
DEFAULT_PROFILE_NAME = "orcahand_v1_right"


@dataclass(frozen=True)
class DriverConfig:
    kind: str
    port: str | None = None
    baudrate: int = 3000000
    options: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class HandProfile:
    profile_id: str
    name: str
    schema_version: str
    hand_type: str | None
    control_mode: str
    max_current: int
    calib_current: int
    wrist_calib_current: int
    calib_step_size: float
    calib_step_period: float
    calib_threshold: float
    calib_num_stable: int
    neutral_position: dict[str, float]
    motor_ids: list[int]
    joint_ids: list[str]
    joint_to_motor_map: dict[str, int]
    joint_roms: dict[str, list[float]]
    calib_sequence: list[dict[str, Any]]
    driver: DriverConfig
    source_path: str | None = None


def built_in_models_dir() -> Path:
    return Path(__file__).resolve().parent / "models"


def list_builtin_profiles() -> list[str]:
    models_dir = built_in_models_dir()
    if not models_dir.exists():
        return []
    return sorted(path.name for path in models_dir.iterdir() if path.is_dir())


def builtin_profile_path(name: str) -> Path:
    path = built_in_models_dir() / name
    if not path.is_dir():
        available = ", ".join(list_builtin_profiles())
        raise FileNotFoundError(
            f"Built-in profile '{name}' not found."
            + (f" Available profiles: {available}" if available else "")
        )
    return path


def load_profile(name: str = DEFAULT_PROFILE_NAME) -> HandProfile:
    return load_profile_from_path(builtin_profile_path(name))


def load_profile_from_path(path: str | Path) -> HandProfile:
    path = Path(path).expanduser().resolve()
    profile_dir = path if path.is_dir() else path.parent
    config_path = path if path.is_file() else path / "config.yaml"

    if not config_path.exists():
        raise FileNotFoundError(f"config.yaml not found at {config_path}")

    config = read_yaml(str(config_path)) or {}
    return _parse_profile(config, profile_dir=profile_dir)


def load_legacy_calibration_data(path: str | Path) -> dict[str, Any]:
    profile_dir = Path(path).expanduser().resolve()
    if profile_dir.is_file():
        profile_dir = profile_dir.parent
    calibration_path = profile_dir / "calibration.yaml"
    if not calibration_path.exists():
        return {}
    return read_yaml(str(calibration_path)) or {}


def _parse_profile(config: dict[str, Any], profile_dir: Path) -> HandProfile:
    required = (
        "motor_ids",
        "joint_ids",
        "joint_to_motor_map",
        "joint_roms",
        "neutral_position",
        "calib_sequence",
    )
    missing = [field_name for field_name in required if not config.get(field_name)]
    if missing:
        raise ValueError(f"Profile is missing required fields: {', '.join(missing)}")

    driver_data = dict(config.get("driver") or {})
    driver_kind = driver_data.pop("kind", None) or config.get("motor_type", "dynamixel")
    driver_port = driver_data.pop("port", None)
    if driver_port is None:
        driver_port = config.get("port")
    driver_baudrate = int(driver_data.pop("baudrate", config.get("baudrate", 3000000)))

    return HandProfile(
        profile_id=config.get("profile_id") or profile_dir.name,
        name=config.get("name") or profile_dir.name,
        schema_version=str(config.get("schema_version", PROFILE_SCHEMA_VERSION)),
        hand_type=config.get("type"),
        control_mode=config.get("control_mode", "current_based_position"),
        max_current=int(config.get("max_current", 300)),
        calib_current=int(config.get("calib_current", 200)),
        wrist_calib_current=int(config.get("wrist_calib_current", 100)),
        calib_step_size=float(config.get("calib_step_size", 0.1)),
        calib_step_period=float(config.get("calib_step_period", 0.01)),
        calib_threshold=float(config.get("calib_threshold", 0.01)),
        calib_num_stable=int(config.get("calib_num_stable", 20)),
        neutral_position={k: float(v) for k, v in dict(config.get("neutral_position", {})).items()},
        motor_ids=[int(motor_id) for motor_id in list(config.get("motor_ids", []))],
        joint_ids=[str(joint_id) for joint_id in list(config.get("joint_ids", []))],
        joint_to_motor_map={str(k): int(v) for k, v in dict(config.get("joint_to_motor_map", {})).items()},
        joint_roms={
            str(k): [float(v[0]), float(v[1])]
            for k, v in dict(config.get("joint_roms", {})).items()
        },
        calib_sequence=list(config.get("calib_sequence", [])),
        driver=DriverConfig(
            kind=str(driver_kind),
            port=str(driver_port) if driver_port is not None else None,
            baudrate=driver_baudrate,
            options=driver_data,
        ),
        source_path=str(profile_dir),
    )
