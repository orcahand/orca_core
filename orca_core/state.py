from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path

from .profile import HandProfile
from .utils import read_yaml, write_yaml


DEFAULT_STATE_ENV_VAR = "ORCA_CORE_STATE_DIR"


@dataclass
class HandState:
    profile_id: str
    profile_schema_version: str
    package_version: str | None = None
    port: str | None = None
    calibrated: bool = False
    wrist_calibrated: bool = False
    motor_limits: dict[int, list[float | None]] = field(default_factory=dict)
    joint_to_motor_ratios: dict[int, float] = field(default_factory=dict)

    @classmethod
    def for_profile(cls, profile: HandProfile, package_version: str | None = None) -> "HandState":
        return cls(
            profile_id=profile.profile_id,
            profile_schema_version=profile.schema_version,
            package_version=package_version,
            port=profile.driver.port,
            calibrated=False,
            wrist_calibrated=False,
            motor_limits={motor_id: [None, None] for motor_id in profile.motor_ids},
            joint_to_motor_ratios={motor_id: 0.0 for motor_id in profile.motor_ids},
        )

    @classmethod
    def from_dict(
        cls,
        data: dict | None,
        profile: HandProfile,
        package_version: str | None = None,
    ) -> "HandState":
        state = cls.for_profile(profile, package_version=package_version)
        if not data:
            return state

        state.package_version = data.get("package_version", package_version)
        state.port = data.get("port", state.port)
        state.calibrated = bool(data.get("calibrated", False))
        state.wrist_calibrated = bool(data.get("wrist_calibrated", False))

        raw_limits = data.get("motor_limits", {})
        for motor_id in profile.motor_ids:
            raw_value = raw_limits.get(motor_id, raw_limits.get(str(motor_id)))
            if raw_value is None:
                continue
            state.motor_limits[motor_id] = [
                None if raw_value[0] is None else float(raw_value[0]),
                None if raw_value[1] is None else float(raw_value[1]),
            ]

        raw_ratios = data.get("joint_to_motor_ratios", {})
        for motor_id in profile.motor_ids:
            raw_value = raw_ratios.get(motor_id, raw_ratios.get(str(motor_id)))
            if raw_value is None:
                continue
            state.joint_to_motor_ratios[motor_id] = float(raw_value)

        return state

    def to_dict(self) -> dict:
        return {
            "profile_id": self.profile_id,
            "profile_schema_version": self.profile_schema_version,
            "package_version": self.package_version,
            "port": self.port,
            "calibrated": self.calibrated,
            "wrist_calibrated": self.wrist_calibrated,
            "motor_limits": {str(k): v for k, v in self.motor_limits.items()},
            "joint_to_motor_ratios": {str(k): v for k, v in self.joint_to_motor_ratios.items()},
        }


class StateStore(ABC):
    @abstractmethod
    def load(self, profile: HandProfile) -> HandState:
        raise NotImplementedError

    @abstractmethod
    def save(self, profile: HandProfile, state: HandState) -> None:
        raise NotImplementedError


class FileStateStore(StateStore):
    def __init__(self, base_dir: str | Path | None = None):
        if base_dir is None:
            base_dir = _default_state_dir()
        self.base_dir = Path(base_dir).expanduser().resolve()
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def path_for(self, profile: HandProfile) -> Path:
        return self.base_dir / f"{profile.profile_id}.yaml"

    def load(self, profile: HandProfile) -> HandState:
        path = self.path_for(profile)
        if not path.exists():
            return HandState.for_profile(profile)
        return HandState.from_dict(read_yaml(str(path)), profile=profile)

    def save(self, profile: HandProfile, state: HandState) -> None:
        write_yaml(str(self.path_for(profile)), state.to_dict())


class InMemoryStateStore(StateStore):
    def __init__(self):
        self._states: dict[str, dict] = {}

    def load(self, profile: HandProfile) -> HandState:
        return HandState.from_dict(self._states.get(profile.profile_id), profile=profile)

    def save(self, profile: HandProfile, state: HandState) -> None:
        self._states[profile.profile_id] = state.to_dict()


def _default_state_dir() -> Path:
    import os

    override = os.environ.get(DEFAULT_STATE_ENV_VAR)
    if override:
        return Path(override)
    return Path.home() / ".orca_core" / "state"
