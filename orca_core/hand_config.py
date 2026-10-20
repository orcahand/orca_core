# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import dataclasses
import logging
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional

from .constants import (
    CALIBRATION_CURRENT_V21,
    CALIBRATION_GROUPS,
    CALIBRATION_RETRY_STEP_MA,
    CONTROL_MODES,
    DEFAULT_CALIBRATION_TRAVEL_RETRIES,
    DEFAULT_MODEL_NAME,
    DEFAULT_RETRY_CURRENT_SCALE,
    FINGER_NAMES,
    JOINT_IDS,
    JOINT_MOTOR_TRAVEL,
    JOINT_MOTOR_TRAVEL_V21,
    JOINT_ROM_DICT,
    HARDWARE_VERSION,
    HARDWARE_VERSION_V21,
    JOINT_TO_MOTOR_MAP,
    JOINT_TO_MOTOR_MAP_V21,
    MOTOR_IDS,
    SUPPORTED_MOTOR_TYPES,
    WRIST,
)
from .control.constants import (
    DEFAULT_CORRECTION_MAX_DEG,
    DEFAULT_JOINT_GAIN_OVERRIDES,
    DEFAULT_KI,
    DEFAULT_KP,
)
from .hardware.sensing.constants import (
    VALID_SENSOR_IDS,
    DEFAULT_ENCODER_BAUDRATE,
    DEFAULT_SENSOR_PORT,
    DEFAULT_FINGER_TO_SENSOR_ID,
)
from .joint_position import OrcaJointPositions
from .utils.utils import get_model_path, read_yaml, write_yaml_atomic


# What a current limit says in config.yaml to take the motor family's own value.
_FAMILY_DEFAULT = "default"


def _current_setting(value, name: str) -> "int | str":
    """A current limit from config.yaml: a whole number of mA, or ``default``."""
    if isinstance(value, str) and value.strip().lower() == _FAMILY_DEFAULT:
        return _FAMILY_DEFAULT
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise HandConfigValidationError(
            f"{name} must be a number of mA or 'default', got {value!r}."
        )
    try:
        return int(value)
    except ValueError:
        raise HandConfigValidationError(
            f"{name} must be a number of mA or 'default', got {value!r}."
        ) from None


class HandConfigValidationError(ValueError):
    """Raised when a hand configuration is structurally invalid."""


@dataclass(frozen=True)
class JointGains:
    """Outer-loop PI gains for one joint of the closed-loop joint controller.

    ``kp`` is dimensionless: the correction is mapped to motor space through
    the calibrated joint-to-motor ratio, so one degree of correction nominally
    produces one degree of joint motion. Joints that respond fast (a small
    motor move producing a large joint move) have the least stability margin
    and want a lower ``kp`` than the rest.
    """

    kp: float = DEFAULT_KP
    ki: float = DEFAULT_KI
    correction_max_deg: float = DEFAULT_CORRECTION_MAX_DEG

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "correction_max_deg"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise HandConfigValidationError(
                    f"JointGains.{name} must be a number, got {value!r}"
                )
            if not math.isfinite(value):
                raise HandConfigValidationError(
                    f"JointGains.{name} must be finite, got {value}"
                )
            if value < 0:
                raise HandConfigValidationError(
                    f"JointGains.{name} must be non-negative, got {value}"
                )

    @classmethod
    def from_dict(
        cls, raw: Dict[str, float], base: "JointGains | None" = None
    ) -> "JointGains":
        """Build from a YAML mapping, inheriting unspecified fields from
        ``base`` (or the package defaults). Lets a per-joint override name
        only the gain it changes."""
        base = base or cls()
        if not isinstance(raw, dict):
            raise HandConfigValidationError(
                f"joint gain entry must be a mapping, got {raw!r}"
            )
        unknown = set(raw) - {"kp", "ki", "correction_max_deg"}
        if unknown:
            raise HandConfigValidationError(
                f"unknown joint gain key(s) {sorted(unknown)}; "
                "expected kp, ki, or correction_max_deg"
            )

        # float() would silently coerce bools and crash on None; reject both
        # here so __post_init__ only ever sees genuine numbers.
        def _coerce(key: str) -> float:
            value = raw[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise HandConfigValidationError(
                    f"JointGains.{key} must be a number, got {value!r}"
                )
            return float(value)

        return cls(
            kp=_coerce("kp") if "kp" in raw else base.kp,
            ki=_coerce("ki") if "ki" in raw else base.ki,
            correction_max_deg=(
                _coerce("correction_max_deg")
                if "correction_max_deg" in raw
                else base.correction_max_deg
            ),
        )

    def as_dict(self) -> Dict[str, float]:
        return dataclasses.asdict(self)


def _resolve_model_name_from_type(hand_type: str | None) -> str:
    """Resolve a packaged model directory name from a hand type string."""
    if hand_type is None:
        return DEFAULT_MODEL_NAME

    normalized_type = hand_type.strip().lower()
    if not normalized_type:
        return DEFAULT_MODEL_NAME
    if normalized_type not in {"left", "right"}:
        raise ValueError(f"Unsupported hand type: {hand_type!r}. Expected 'left' or 'right'.")

    return f"orcahand-{normalized_type}"


def _resolve_config_path(
    config_path: str | None,
    model_version: str | None = None,
    model_name: str | None = None,
) -> str:
    """Resolve the canonical ``config.yaml`` path."""
    if config_path is None:
        resolved = os.path.join(
            get_model_path(model_version=model_version, model_name=model_name),
            "config.yaml",
        )
    elif os.path.basename(config_path) != "config.yaml":
        raise ValueError("config_path must point to a config.yaml file.")
    else:
        resolved = os.path.abspath(config_path)

    if not os.path.exists(resolved):
        raise FileNotFoundError(f"config.yaml not found: {resolved}")

    return resolved


def _resolve_calibration_path(config_path: str, calibration_path: str | None) -> str:
    """Resolve the companion ``calibration.yaml`` path for a config file."""
    if calibration_path is not None:
        return os.path.abspath(calibration_path)
    return os.path.join(os.path.dirname(config_path), "calibration.yaml")


def canonical_joint_ids(
    version: str | None = None,
    type: str | None = None,
) -> tuple[str, ...]:
    """Return the canonical joint ordering for a packaged ORCA hand model."""
    config = BaseHandConfig.from_config_path(
        model_version=version,
        model_name=_resolve_model_name_from_type(type),
    )

    return tuple(config.joint_ids)


def _canonical_joint_to_motor_map(
    raw_joint_to_motor_map: Dict[str, int],
) -> tuple[Dict[str, int], Dict[str, bool]]:
    """Normalize signed YAML motor mappings into absolute IDs plus inversion flags."""
    normalized_map: Dict[str, int] = {}
    inversion_dict: Dict[str, bool] = {}
    for joint, motor_id in raw_joint_to_motor_map.items():
        motor_id = int(motor_id)
        inversion_dict[joint] = motor_id < 0
        normalized_map[joint] = abs(motor_id)
    return normalized_map, inversion_dict


LOCAL_CONFIG_FILE = "local_config.yaml"
# Keys a hand's local_config.yaml may override. Everything else in it is ignored
# with a warning: the file is for what belongs to one hand, not for config edits.
LOCAL_CONFIG_KEYS = (HARDWARE_VERSION,)


def local_config_path(config_path: str) -> str:
    """``local_config.yaml`` beside ``config_path``: a hand's own overrides,
    kept out of the packaged config so the package stays pristine and git
    clean. Written by :func:`write_hardware_version`; read at every load."""
    return os.path.join(os.path.dirname(os.path.abspath(config_path)), LOCAL_CONFIG_FILE)


def read_local_config(config_path: str) -> Dict:
    """The overrides in force for ``config_path``'s hand (whitelisted keys only)."""
    raw = read_yaml(local_config_path(config_path)) or {}
    if not isinstance(raw, dict):
        logging.getLogger(__name__).warning(
            "%s is not a mapping; ignoring it", local_config_path(config_path))
        return {}
    unknown = sorted(set(raw) - set(LOCAL_CONFIG_KEYS))
    if unknown:
        logging.getLogger(__name__).warning(
            "%s: ignoring %s (only %s may be overridden there)",
            local_config_path(config_path), unknown, ", ".join(LOCAL_CONFIG_KEYS))
    return {k: raw[k] for k in LOCAL_CONFIG_KEYS if k in raw}


def _with_local_overrides(config: Dict, config_path: str) -> Dict:
    """``config`` with the hand's local overrides on top. A local value that
    displaces an explicit one in config.yaml is logged: the yaml no longer
    says what is in force."""
    local = read_local_config(config_path)
    for key, value in local.items():
        current = config.get(key)
        if current not in (None, "auto") and current != value:
            logging.getLogger(__name__).warning(
                "%s sets %s=%r, so config.yaml's %r is not used",
                local_config_path(config_path), key, value, current)
    return {**config, **local}


def format_hardware_version(version: int | None) -> str:
    """``21`` -> ``"v2.1"``, ``2`` -> ``"v2.0"``; the board stores major*10 + minor,
    and a single digit is a bare major."""
    if version is None:
        return "an unknown version"
    major, minor = (version, 0) if version < 10 else divmod(version, 10)
    return f"v{major}.{minor}"


def write_hardware_version(config_path: str, version: int | None) -> str:
    """Pin ``hardware_version`` for the hand at ``config_path``, or lift the
    pin for ``None``. The pin lives in the hand's ``local_config.yaml`` beside
    the config; config.yaml itself is never touched, so a packaged model stays
    as shipped. Returns the path written."""
    if version is not None and (isinstance(version, bool) or not isinstance(version, int) or version < 0):
        raise HandConfigValidationError(
            f"{HARDWARE_VERSION} must be a non-negative integer or None, got {version!r}"
        )
    path = local_config_path(config_path)
    local = read_yaml(path) or {}
    if not isinstance(local, dict):
        local = {}
    if version is None:
        local.pop(HARDWARE_VERSION, None)
    else:
        local[HARDWARE_VERSION] = int(version)
    if local:
        write_yaml_atomic(path, local)
    elif os.path.exists(path):
        os.remove(path)
    return path


def calibration_group(joint: str) -> Optional[str]:
    """``abd`` (the abductions and thumb_cmc), ``mcp``, ``pip`` (the PIPs and thumb_dip), or None for the wrist."""
    if joint == WRIST:
        return None
    if joint.endswith("_abd") or joint == "thumb_cmc":
        return "abd"
    return "mcp" if joint.endswith("_mcp") else "pip"


def _parse_group_currents(raw, key: str) -> Dict[str, Dict[str, int]]:
    raw = raw or {}
    if not isinstance(raw, dict):
        raise HandConfigValidationError(
            f"{key} must map a motor family to per-group currents in mA, got {raw!r}"
        )
    by_family = {}
    for family, groups in raw.items():
        if not isinstance(groups, dict) or not set(groups) <= set(CALIBRATION_GROUPS):
            raise HandConfigValidationError(
                f"{key}[{family}] must map {', '.join(CALIBRATION_GROUPS)} to a current in mA, "
                f"got {groups!r}"
            )
        for group, current in groups.items():
            if isinstance(current, bool) or not isinstance(current, int) or current <= 0:
                raise HandConfigValidationError(
                    f"{key}[{family}][{group}] must be a positive number of mA, got {current!r}"
                )
        by_family[str(family)] = {str(g): int(c) for g, c in groups.items()}
    return by_family


def _parse_motor_travel(raw, key: str) -> Dict[str, float]:
    raw = raw or {}
    if not isinstance(raw, dict):
        raise HandConfigValidationError(
            f"{key} must be a mapping of joint name to motor travel in degrees, got {raw!r}"
        )
    travel_by_joint = {}
    for joint, travel in raw.items():
        if travel is None or isinstance(travel, bool) or not isinstance(travel, (int, float)):
            raise HandConfigValidationError(
                f"{key}[{joint}] must be a number of degrees, got {travel!r}"
            )
        travel_by_joint[str(joint)] = float(travel)
    return travel_by_joint


def _validated_gain_partial(raw, where: str) -> Dict[str, float]:
    """Check one partial gain mapping and return it as plain floats.

    Kept partial rather than resolved so each layer can override only the keys
    it names; building a JointGains from it here surfaces bad keys and negative
    values at config-load time instead of at connect.
    """
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise HandConfigValidationError(f"{where} must be a mapping, got {raw!r}")
    JointGains.from_dict(raw)  # validates keys and values; result discarded
    return {key: float(value) for key, value in raw.items()}


def _parse_joint_control_gains(
    raw: Dict[str, dict] | None,
) -> tuple[Dict[str, float], Dict[str, Dict[str, float]]]:
    """Parse the config.yaml ``joint_control_gains`` block.

    The block holds overrides only — the defaults in ``control.constants`` are
    the baseline, so an absent block (or an absent key within it) means "use
    the shipped default". Four layers, each overriding only the gains it
    names::

        control/constants.py  DEFAULT_KP / DEFAULT_KI / DEFAULT_CORRECTION_MAX_DEG
          └─ DEFAULT_JOINT_GAIN_OVERRIDES   shipped per-joint defaults
               └─ config.yaml  all:         this hand, every joint
                    └─ config.yaml  joints: this hand, the joints it names

    Expected shape, with every key optional::

        joint_control_gains:
          all: {kp: 0.4}
          joints:
            index_mcp: {kp: 0.25}

    Returns the raw partial overrides; :meth:`OrcaHandConfig.joint_gains_for`
    layers them.
    """
    if raw is None:
        return {}, {}
    if not isinstance(raw, dict):
        raise HandConfigValidationError(
            f"joint_control_gains must be a mapping, got {raw!r}"
        )
    unknown = set(raw) - {"all", "joints"}
    if unknown:
        raise HandConfigValidationError(
            f"unknown joint_control_gains key(s) {sorted(unknown)}; "
            "expected 'all' and/or 'joints'"
        )

    gains_all = _validated_gain_partial(raw.get("all"), "joint_control_gains.all")

    raw_joints = raw.get("joints") or {}
    if not isinstance(raw_joints, dict):
        raise HandConfigValidationError(
            f"joint_control_gains.joints must be a mapping, got {raw_joints!r}"
        )
    overrides = {
        str(joint): _validated_gain_partial(
            entry, f"joint_control_gains.joints.{joint}"
        )
        for joint, entry in raw_joints.items()
    }
    return gains_all, overrides


def _shared_hand_kwargs(config: dict) -> dict:
    """The joint-space fields every hand config carries, taken from a parsed ``config.yaml``."""
    kwargs = {}
    if "type" in config:
        kwargs["type"] = config["type"]
    if JOINT_IDS in config:
        kwargs["joint_ids"] = list(config[JOINT_IDS])
    if JOINT_ROM_DICT in config:
        kwargs["joint_roms_dict"] = dict(config[JOINT_ROM_DICT])
    if "neutral_position" in config:
        kwargs["neutral_position"] = dict(config["neutral_position"])
    return kwargs


@dataclass(frozen=True)
class BaseHandConfig:
    """Base joint-space configuration for a hand model."""

    config_path: str
    type: Literal["left", "right"] | None = None
    joint_ids: List[str] = field(default_factory=list)
    joint_roms_dict: Dict[str, List[float]] = field(default_factory=dict)
    neutral_position: Dict[str, float] = field(default_factory=dict)

    @property
    def model_path(self) -> str:
        """Directory containing the hand assets described by ``config_path``."""
        return os.path.dirname(self.config_path)

    @classmethod
    def from_config_path(
        cls,
        config_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
    ) -> "BaseHandConfig":
        """Load shared hand metadata from ``config.yaml``."""
        resolved_config_path = _resolve_config_path(
            config_path,
            model_version=model_version,
            model_name=model_name,
        )
        config = _with_local_overrides(read_yaml(resolved_config_path) or {}, resolved_config_path)

        return cls(config_path=resolved_config_path, **_shared_hand_kwargs(config))

    def validate(self) -> None:
        """Validate shared joint-space configuration."""
        if not self.joint_ids:
            raise HandConfigValidationError("At least one joint must be configured.")

        for joint in self.joint_ids:
            if joint not in self.joint_roms_dict:
                raise HandConfigValidationError(f"ROM for joint {joint} is not defined.")

        for joint, rom in self.joint_roms_dict.items():
            if joint not in self.joint_ids:
                raise HandConfigValidationError(f"Joint {joint} in ROMs is not defined.")
            if len(rom) != 2 or rom[1] - rom[0] <= 0:
                raise HandConfigValidationError(f"ROM {rom} for joint {joint} is not valid.")

        for joint in self.neutral_position:
            if joint not in self.joint_ids:
                raise HandConfigValidationError(
                    f"Neutral position entry for joint {joint} is not defined."
                )

    def clamp_joint_positions(
        self,
        joint_pos: OrcaJointPositions,
    ) -> OrcaJointPositions:
        """Clamp joint positions to the configured ROM bounds."""
        if not isinstance(joint_pos, OrcaJointPositions):
            raise HandConfigValidationError(
                "joint_pos must be an OrcaJointPositions instance."
            )

        normalized = {}
        for joint, pos in joint_pos:
            if joint not in self.joint_ids:
                raise HandConfigValidationError(
                    f"Joint {joint} is not defined among the hand's joint IDs: {self.joint_ids}"
                )
            normalized[joint] = self._clamp_joint_value(joint, pos)

        return OrcaJointPositions.from_dict(normalized)

    def _clamp_joint_value(self, joint_name: str, joint_pos: float) -> float:
        min_pos, max_pos = self.joint_roms_dict[joint_name]
        return max(min_pos, min(max_pos, float(joint_pos)))


@dataclass(frozen=True)
class OrcaHandConfig(BaseHandConfig):
    """ORCA hand configuration layered on top of the shared base spec."""

    calibration_path: str = ""
    # None = auto-detect at connect time (probed and persisted to config.yaml).
    baudrate: int | None = None
    port: str = "auto"
    # A current limit in mA, or "default": the motor family's own value, filled
    # in once the family is known (load_hand when detected or pinned, else connect).
    max_current: int | str = _FAMILY_DEFAULT
    control_mode: str = "current_based_position"
    motor_type: str | None = None
    motor_ids: List[int] = field(default_factory=list)
    joint_to_motor_map: Dict[str, int] = field(default_factory=dict)
    joint_inversion_dict: Dict[str, bool] = field(default_factory=dict)
    # The hand's hardware version (2 = v2.0, 21 = v2.1). It selects which
    # joint_to_motor_map is in force and gates the v2.1-only features. Pinned in the yaml by a hand whose motor
    # bus has no controller board to report it; otherwise `auto`, and load_hand
    # fills it from the board. A pin beats the board and the clash is logged,
    # as for port and motor_type. None means not known.
    hardware_version: int | None = None
    # The signed map a v2.1 hand uses instead of joint_to_motor_map, kept raw
    # (sign included). Absent on models whose wiring did not change.
    # with_hardware_version applies it; the chosen map then passes the same
    # checks the base map does.
    joint_to_motor_map_v21: Dict[str, int] = field(default_factory=dict)
    calibration_current: int | str = _FAMILY_DEFAULT
    # None follows calibration_current: the wrist gets what the fingers get
    # unless a config says otherwise.
    wrist_calibration_current: int | str | None = None
    # v2.1 only: per motor family, a calibration current per joint group (see
    # calibration_group) that replaces calibration_current for those joints.
    calibration_current_v21_dict: Dict[str, Dict[str, int]] = field(default_factory=dict)
    calibration_step_size: float = 0.1  # rad
    calibration_step_period: float = 0.01  # s
    calibration_threshold: float = 0.01  # rad
    calibration_num_stable: int = 20
    calibration_sequence: List[dict] = field(default_factory=list)
    # Motor-shaft travel between a joint's two hardstops, in degrees, keyed by
    # joint name. The travel a joint's motor makes is set by that joint's
    # tendon spool diameter, so it belongs to the joint and is resolved to a
    # motor only where it is used - motor IDs differ per hand. A hand's own
    # measurement (``joint_motor_travel``) wins over the packaged v2.1 baseline.
    joint_motor_travel_dict: Dict[str, float] = field(default_factory=dict)
    joint_motor_travel_v21_dict: Dict[str, float] = field(default_factory=dict)
    # Fraction of the expected travel a measured travel may deviate by before
    # the calibration routine re-drives the joint at a higher current.
    calibration_travel_margin: float = 0.25
    # Current the re-drive uses (mA). None derives it from calibration_current.
    calibration_retry_current: int | None = None
    calibration_travel_retries: int = DEFAULT_CALIBRATION_TRAVEL_RETRIES
    # Ceiling for the re-drive only (mA), independent of max_current: getting a
    # joint past an over-tensioned tendon takes more torque than the hand is
    # allowed to use in normal operation, and the routine restores max_current
    # as soon as it finishes. Unset (with calibration_retry_current unset too)
    # steps each re-drive CALIBRATION_RETRY_STEP_MA above the last, so the top
    # attempt is retries x step above the nominal calibration current.
    calibration_max_current: int | None = None
    use_joint_feedback: bool | None = None
    # Joint names or the ["all"] sentinel (every slotted joint, wrist
    # included); an explicit finger-only list leaves the wrist open-loop.
    joint_encoder_joints: List[str] | None = None
    encoder_serial_port: str = "auto"
    encoder_baudrate: int = DEFAULT_ENCODER_BAUDRATE
    # Partial outer-loop PI gain overrides from the config.yaml
    # 'joint_control_gains' block; the defaults live in control.constants.
    # See _parse_joint_control_gains() for the layering.
    joint_gains_all: Dict[str, float] = field(default_factory=dict)
    joint_gains_overrides: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def joint_gains_for(self, joint: str) -> JointGains:
        """Gains the closed-loop controller uses for ``joint``, with every
        override layer applied in order. Each layer sets only the gains it
        names; the rest fall through to the layer above."""
        layered: Dict[str, float] = dict(DEFAULT_JOINT_GAIN_OVERRIDES.get(joint, {}))
        layered.update(self.joint_gains_all)
        layered.update(self.joint_gains_overrides.get(joint, {}))
        return JointGains.from_dict(layered)

    @property
    def joint_gains_baseline(self) -> JointGains:
        """What a joint with no per-joint override resolves to — the package
        defaults with this hand's ``all:`` block applied."""
        return JointGains.from_dict(self.joint_gains_all)

    @property
    def joint_gains_dict(self) -> Dict[str, JointGains]:
        """Resolved gains for every configured joint, overrides applied."""
        return {joint: self.joint_gains_for(joint) for joint in self.joint_ids}

    @property
    def calibration_group_currents(self) -> Dict[str, int]:
        """The per-group calibration currents in force: v2.1 hardware with an
        entry for this hand's motor family, else none."""
        if not self.uses_motor_travel:
            return {}
        return dict(self.calibration_current_v21_dict.get(str(self.motor_type), {}))

    def calibration_current_for(self, joint: Optional[str]) -> float:
        """The current (mA) ``joint`` is driven onto its hardstops with."""
        if joint == WRIST:
            return self.wrist_calibration_current
        group = calibration_group(joint) if joint else None
        return self.calibration_group_currents.get(group, self.calibration_current)

    def calibration_retry_current_for(self, joint: Optional[str] = None) -> int:
        """Current (mA) a short-travel ``joint`` is re-driven at.

        An over-tensioned tendon stalls the drive before the hardstop, so the
        re-drive needs more torque than the nominal calibration pass. Config
        may pin it outright; otherwise it scales off the joint's own current.
        """
        if self.calibration_retry_current is not None:
            return self.calibration_retry_current
        return int(round(self.calibration_current_for(joint) * DEFAULT_RETRY_CURRENT_SCALE))

    @property
    def calibration_retry_current_resolved(self) -> int:
        """Current (mA) a short-travel joint on the plain calibration current is re-driven at."""
        return self.calibration_retry_current_for(None)

    @property
    def retry_steps_by_default(self) -> bool:
        """Re-drives step up by CALIBRATION_RETRY_STEP_MA, nothing being pinned."""
        return self.calibration_retry_current is None and self.calibration_max_current is None

    def retry_current_for_attempt(self, attempt: int, joint: Optional[str] = None) -> float:
        """Current (mA) for 1-based re-drive ``attempt`` of ``joint``.

        With neither ``calibration_retry_current`` nor ``calibration_max_current``
        pinned, each attempt adds CALIBRATION_RETRY_STEP_MA to the nominal
        current. Otherwise:

        Successive attempts ramp evenly from the nominal calibration current up
        to :attr:`calibration_retry_current_resolved`, so a hand that only
        needs a nudge is not driven at the full retry current on the first try.
        With the default single retry there is no room to ramp and the one
        attempt goes straight to the top — set ``calibration_travel_retries``
        above 1 to get the gradual approach.

        Capped at :attr:`calibration_max_current`, *not* at ``max_current``.
        The whole point of the re-drive is to break a joint past whatever
        stalled it, which routinely needs more torque than the hand is allowed
        in normal operation; capping at ``max_current`` would make the retry
        repeat the pass that already failed on any hand whose
        ``calibration_current`` equals its ceiling. The elevated current is
        bounded in time — the routine restores ``max_current`` when
        calibration ends — so it never leaks into ordinary motor commands.
        """
        if attempt < 1 or attempt > self.calibration_travel_retries:
            raise ValueError(
                f"attempt {attempt} is outside 1..{self.calibration_travel_retries}"
            )
        base = float(self.calibration_current_for(joint))
        if self.retry_steps_by_default:
            return base + CALIBRATION_RETRY_STEP_MA * attempt
        top = float(self.calibration_retry_current_for(joint))
        ramped = base + (top - base) * attempt / self.calibration_travel_retries
        ceiling = (
            self.calibration_max_current
            if self.calibration_max_current is not None
            else base
        )
        return min(ramped, float(ceiling))

    @property
    def uses_motor_travel(self) -> bool:
        """Whether the travel-based features (calibration's short-travel
        re-drive, guided tensioning) apply: v2.1 hardware only."""
        return self.hardware_version == HARDWARE_VERSION_V21

    def expected_motor_travel_deg(self, joint: str, *, any_hardware: bool = False) -> float | None:
        """Baseline motor travel for ``joint``, or ``None`` if there is none.

        ``None`` on anything but v2.1 hardware unless ``any_hardware`` is set,
        which an operator's explicit override of the version gate uses.
        """
        if not (self.uses_motor_travel or any_hardware):
            return None
        return self.joint_motor_travel_dict.get(joint, self.joint_motor_travel_v21_dict.get(joint))

    @property
    def motor_id_to_idx_dict(self) -> Dict[int, int]:
        """Map motor ID to its index in ``motor_ids``."""
        return {motor_id: idx for idx, motor_id in enumerate(self.motor_ids)}

    @property
    def motor_to_joint_dict(self) -> Dict[int, str]:
        """Map motor ID to joint name."""
        return {motor_id: joint for joint, motor_id in self.joint_to_motor_map.items()}

    @property
    def has_joint_encoders(self) -> bool:
        """Whether this hand declares any joint-encoder hardware."""
        return bool(self.joint_encoder_joints)

    @property
    def joint_feedback_enabled(self) -> bool:
        """Whether the closed-loop joint-feedback controller should engage.

        ``use_joint_feedback`` is tri-state: ``None`` (unset) defers to the
        presence of encoder hardware, so a hand that declares
        ``joint_encoder_joints`` runs closed-loop by default. An explicit
        ``False`` forces plain motor-only control even when encoders exist.
        """
        if self.use_joint_feedback is None:
            return self.has_joint_encoders
        return self.use_joint_feedback

    @classmethod
    def _kwargs_from_config_path(
        cls,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
    ) -> "tuple[dict, dict]":
        """Constructor kwargs and the parsed yaml document for a hardware-backed hand."""
        resolved_config_path = _resolve_config_path(
            config_path,
            model_version=model_version,
            model_name=model_name,
        )
        resolved_calibration_path = _resolve_calibration_path(
            resolved_config_path, calibration_path
        )

        config = _with_local_overrides(read_yaml(resolved_config_path) or {}, resolved_config_path)
        if not config:
            raise HandConfigValidationError(
                f"config.yaml at {resolved_config_path} is empty. Restore it "
                "from the bundled model or your hand's backup."
            )

        kwargs = {
            "config_path": resolved_config_path,
            "calibration_path": resolved_calibration_path,
            **_shared_hand_kwargs(config),
        }
        if "baudrate" in config:
            kwargs["baudrate"] = int(config["baudrate"])
        if "port" in config:
            kwargs["port"] = config["port"]
        if "max_current" in config:
            kwargs["max_current"] = _current_setting(config["max_current"], "max_current")
        if "control_mode" in config:
            kwargs["control_mode"] = config["control_mode"]
        if "motor_type" in config:
            kwargs["motor_type"] = config["motor_type"]
        if MOTOR_IDS in config:
            kwargs["motor_ids"] = [int(motor_id) for motor_id in config[MOTOR_IDS]]
        if JOINT_TO_MOTOR_MAP in config:
            joint_to_motor_map, joint_inversion_dict = _canonical_joint_to_motor_map(
                dict(config[JOINT_TO_MOTOR_MAP])
            )
            kwargs["joint_to_motor_map"] = joint_to_motor_map
            kwargs["joint_inversion_dict"] = joint_inversion_dict
        if JOINT_TO_MOTOR_MAP_V21 in config:
            raw_v21 = config[JOINT_TO_MOTOR_MAP_V21]
            if not isinstance(raw_v21, dict):
                raise HandConfigValidationError(
                    f"{JOINT_TO_MOTOR_MAP_V21} must be a mapping of joint name to "
                    f"signed motor ID, got {raw_v21!r}"
                )
            kwargs["joint_to_motor_map_v21"] = {
                str(joint): int(motor_id) for joint, motor_id in raw_v21.items()
            }
        raw_hw = config.get(HARDWARE_VERSION)
        if raw_hw is not None and not (isinstance(raw_hw, str) and raw_hw.strip().lower() == "auto"):
            if isinstance(raw_hw, bool) or not isinstance(raw_hw, int) or raw_hw < 0:
                raise HandConfigValidationError(
                    f"{HARDWARE_VERSION} must be 'auto' or a non-negative integer "
                    f"(2 for v2.0, 21 for v2.1), got {raw_hw!r}"
                )
            kwargs["hardware_version"] = raw_hw
        if "calibration_current" in config:
            kwargs["calibration_current"] = _current_setting(
                config["calibration_current"], "calibration_current"
            )
        if config.get("wrist_calibration_current") is not None:
            kwargs["wrist_calibration_current"] = _current_setting(
                config["wrist_calibration_current"], "wrist_calibration_current"
            )
        if "calibration_step_size" in config:
            kwargs["calibration_step_size"] = float(config["calibration_step_size"])
        if "calibration_step_period" in config:
            kwargs["calibration_step_period"] = float(config["calibration_step_period"])
        if "calibration_threshold" in config:
            kwargs["calibration_threshold"] = float(config["calibration_threshold"])
        if "calibration_num_stable" in config:
            kwargs["calibration_num_stable"] = int(config["calibration_num_stable"])
        if "calibration_sequence" in config:
            kwargs["calibration_sequence"] = list(config["calibration_sequence"])
        if CALIBRATION_CURRENT_V21 in config:
            kwargs["calibration_current_v21_dict"] = _parse_group_currents(
                config[CALIBRATION_CURRENT_V21], CALIBRATION_CURRENT_V21
            )
        for key, kwarg in ((JOINT_MOTOR_TRAVEL, "joint_motor_travel_dict"),
                           (JOINT_MOTOR_TRAVEL_V21, "joint_motor_travel_v21_dict")):
            if key in config:
                kwargs[kwarg] = _parse_motor_travel(config[key], key)
        if "calibration_travel_margin" in config:
            kwargs["calibration_travel_margin"] = float(
                config["calibration_travel_margin"]
            )
        if "calibration_retry_current" in config:
            raw_retry = config["calibration_retry_current"]
            kwargs["calibration_retry_current"] = (
                None if raw_retry is None else int(raw_retry)
            )
        if "calibration_travel_retries" in config:
            kwargs["calibration_travel_retries"] = int(
                config["calibration_travel_retries"]
            )
        if "calibration_max_current" in config:
            raw_ceiling = config["calibration_max_current"]
            kwargs["calibration_max_current"] = (
                None if raw_ceiling is None else int(raw_ceiling)
            )
        if "use_joint_feedback" in config:
            raw_ujf = config["use_joint_feedback"]
            kwargs["use_joint_feedback"] = None if raw_ujf is None else bool(raw_ujf)
        if "joint_encoder_joints" in config:
            raw = config["joint_encoder_joints"]
            kwargs["joint_encoder_joints"] = (
                None if raw is None else [str(j) for j in raw]
            )
        if "encoder_serial_port" in config:
            kwargs["encoder_serial_port"] = str(config["encoder_serial_port"])
        if "encoder_baudrate" in config:
            kwargs["encoder_baudrate"] = int(config["encoder_baudrate"])
        if "joint_control_gains" in config:
            gains_all, overrides = _parse_joint_control_gains(
                config["joint_control_gains"]
            )
            kwargs["joint_gains_all"] = gains_all
            kwargs["joint_gains_overrides"] = overrides

        return kwargs, config

    @classmethod
    def from_config_path(
        cls,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
    ) -> "OrcaHandConfig":
        """Load hardware-backed ORCA hand configuration from canonical YAML keys."""
        kwargs, _ = cls._kwargs_from_config_path(
            config_path, calibration_path, model_version, model_name
        )
        # A pinned version is in force from load, so a hand built straight from
        # the path (not through load_hand) still gets that revision's map.
        return cls(**kwargs).with_hardware_version(kwargs.get("hardware_version"))

    def validate_control_mode(self) -> None:
        """Check ``control_mode`` against what the motor family accepts.

        An unpinned ``motor_type`` is only checked against the union of modes
        across families; connect re-runs this once the family is resolved.
        """
        if self.motor_type is None:
            if self.control_mode not in CONTROL_MODES:
                raise HandConfigValidationError("Invalid control mode.")
            return

        from .hardware.motor_factory import motor_client_class

        supported = motor_client_class(self.motor_type).supported_modes
        if self.control_mode not in supported:
            raise HandConfigValidationError(
                f"control_mode {self.control_mode!r} is not supported by "
                f"{self.motor_type} motors; supported: {sorted(supported)}."
            )

    def validate_config(self) -> None:
        """Validate the ORCA hand configuration."""
        super().validate()

        if len(self.motor_ids) != len(self.joint_ids):
            raise HandConfigValidationError(
                "Number of motor IDs and joints do not match."
            )

        if len(self.motor_ids) != len(self.joint_to_motor_map):
            raise HandConfigValidationError(
                "Number of motor IDs and joint-to-motor mappings do not match."
            )

        if self.motor_type is not None and self.motor_type not in SUPPORTED_MOTOR_TYPES:
            raise HandConfigValidationError(
                f"Unknown motor_type: {self.motor_type!r}. "
                f"Expected one of {SUPPORTED_MOTOR_TYPES} or omit for auto-detection."
            )

        self.validate_control_mode()

        for name in ("max_current", "calibration_current", "wrist_calibration_current"):
            value = getattr(self, name)
            if value == _FAMILY_DEFAULT:
                continue
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0):
                raise HandConfigValidationError(
                    f"{name} must be a positive number of mA or 'default', got {value!r}."
                )

        if self.currents_resolved and self.max_current < self.calibration_current:
            raise HandConfigValidationError(
                "Max current should be greater than the calibration current."
            )

        self._validate_travel_check()

        for joint, motor_id in self.joint_to_motor_map.items():
            if joint not in self.joint_ids:
                raise HandConfigValidationError(f"Joint {joint} is not defined.")
            if motor_id not in self.motor_ids:
                raise HandConfigValidationError(
                    f"Motor ID {motor_id} is not in the motor IDs list."
                )

        for step in self.calibration_sequence:
            joints = step.get("joints")
            if not isinstance(joints, dict):
                raise HandConfigValidationError(
                    "Each calibration step must contain a 'joints' mapping."
                )
            for joint, direction in joints.items():
                if joint not in self.joint_ids:
                    raise HandConfigValidationError(f"Joint {joint} is not defined.")
                if direction not in ["flex", "extend"]:
                    raise HandConfigValidationError(
                        f"Invalid direction for joint {joint}."
                    )

        # A gain override for an unknown joint would silently never apply.
        unknown_gain_joints = set(self.joint_gains_overrides) - set(self.joint_ids)
        if unknown_gain_joints:
            raise HandConfigValidationError(
                f"joint_control_gains.joints names unknown joint(s) "
                f"{sorted(unknown_gain_joints)}; expected joints from joint_ids."
            )

        if self.use_joint_feedback is True and not self.has_joint_encoders:
            raise HandConfigValidationError(
                "use_joint_feedback: true requires joint_encoder_joints to be set."
            )

        if self.joint_encoder_joints is not None:
            from .hardware.sensing.constants import (
                ENCODER_JOINTS_ALL,
                JOINT_TO_ENCODER_SLOT,
            )
            for joint in self.joint_encoder_joints:
                if str(joint).lower() == ENCODER_JOINTS_ALL:
                    continue
                if joint not in JOINT_TO_ENCODER_SLOT:
                    raise HandConfigValidationError(
                        f"joint_encoder_joints contains {joint!r}, which has no encoder slot."
                    )

    def _validate_travel_check(self) -> None:
        """Validate the dynamic short-travel re-drive settings."""
        for key, travel_by_joint in ((JOINT_MOTOR_TRAVEL, self.joint_motor_travel_dict),
                                     (JOINT_MOTOR_TRAVEL_V21, self.joint_motor_travel_v21_dict)):
            for joint, travel in travel_by_joint.items():
                if joint not in self.joint_ids:
                    raise HandConfigValidationError(
                        f"{key} names joint {joint!r}, which is not in joint_ids."
                    )
                if not math.isfinite(travel) or travel <= 0:
                    raise HandConfigValidationError(
                        f"{key}[{joint}] must be a positive number of degrees, "
                        f"got {travel!r}"
                    )

        margin = self.calibration_travel_margin
        if not math.isfinite(margin) or not 0 < margin < 1:
            raise HandConfigValidationError(
                "calibration_travel_margin must be a fraction in (0, 1), got "
                f"{margin!r}"
            )

        if self.calibration_travel_retries < 0:
            raise HandConfigValidationError(
                "calibration_travel_retries must be non-negative, got "
                f"{self.calibration_travel_retries!r}"
            )

        # The re-drive ceiling has to leave room above the nominal pass, or
        # every attempt clamps back onto the current that already stalled.
        if (
            self.currents_resolved
            and self.calibration_max_current is not None
            and self.calibration_max_current < self.calibration_current
        ):
            raise HandConfigValidationError(
                "calibration_max_current must be at least calibration_current "
                f"({self.calibration_current}), got "
                f"{self.calibration_max_current}."
            )

        # A retry current at or below the nominal one would re-drive with no
        # more torque than the pass that already stalled short.
        if (
            self.currents_resolved
            and self.calibration_retry_current is not None
            and self.calibration_retry_current <= self.calibration_current
        ):
            raise HandConfigValidationError(
                "calibration_retry_current must exceed calibration_current "
                f"({self.calibration_current}), got "
                f"{self.calibration_retry_current}."
            )

    def __post_init__(self) -> None:
        if self.wrist_calibration_current is None:
            object.__setattr__(self, "wrist_calibration_current", self.calibration_current)
        self.validate_config()

    @property
    def currents_resolved(self) -> bool:
        """Whether every current limit is a number rather than ``default``."""
        return _FAMILY_DEFAULT not in (
            self.max_current, self.calibration_current, self.wrist_calibration_current
        )

    def with_family_currents(self, client_cls) -> "OrcaHandConfig":
        """This config with each ``default`` current replaced by ``client_cls``'s own.

        ``client_cls`` is the motor client class of the connected family; its
        ``default_max_current_ma`` fills ``max_current`` and its
        ``default_calibration_current_ma`` fills ``calibration_current`` and a
        wrist value that was left to follow it. The result is validated, so a
        pinned ``max_current`` below the family's calibration current raises.
        """
        if self.currents_resolved:
            return self

        def pick(value, family_value):
            return int(family_value) if value == _FAMILY_DEFAULT else value

        return dataclasses.replace(
            self,
            max_current=pick(self.max_current, client_cls.default_max_current_ma),
            calibration_current=pick(
                self.calibration_current, client_cls.default_calibration_current_ma
            ),
            wrist_calibration_current=pick(
                self.wrist_calibration_current, client_cls.default_calibration_current_ma
            ),
        )


    def with_hardware_version(self, hardware_version: int | None) -> "OrcaHandConfig":
        """This config as the given hardware revision of the hand sees it.

        Picks the map for the revision. For one that carries its own,
        ``joint_to_motor_map`` and ``joint_inversion_dict`` are rebuilt from it,
        exactly as they are built from the base map at load; every consumer
        then reads it with no idea a revision exists. A model without a map
        for the revision keeps the base one. ``hardware_version`` is recorded
        either way, the same way a detected ``motor_type`` is.

        ``None`` means the version is not known, and returns self untouched.
        """
        if hardware_version is None:
            return self
        if hardware_version == HARDWARE_VERSION_V21 and self.joint_to_motor_map_v21:
            joint_to_motor_map, joint_inversion_dict = _canonical_joint_to_motor_map(
                self.joint_to_motor_map_v21
            )
            return dataclasses.replace(
                self,
                hardware_version=hardware_version,
                joint_to_motor_map=joint_to_motor_map,
                joint_inversion_dict=joint_inversion_dict,
            )
        return dataclasses.replace(self, hardware_version=hardware_version)


def _sensor_kwargs(sensors: dict) -> dict:
    """Touch-config constructor kwargs from the ``sensors:`` block."""
    sensor_kwargs = {}
    if "port" in sensors:
        sensor_kwargs["sensor_port"] = sensors["port"]
    if "baudrate" in sensors:
        raw_baud = sensors["baudrate"]
        sensor_kwargs["sensor_baudrate"] = raw_baud if raw_baud == "auto" else int(raw_baud)
    if "finger_to_sensor_id" in sensors:
        sensor_kwargs["finger_to_sensor_id"] = dict(sensors["finger_to_sensor_id"])
    return sensor_kwargs


@dataclass(frozen=True)
class OrcaHandTouchConfig(OrcaHandConfig):
    """ORCA hand configuration with tactile sensor support."""

    sensor_port: str = DEFAULT_SENSOR_PORT
    sensor_baudrate: "int | str" = "auto"
    """``"auto"`` detects the baud from the connected sensor. Set an int in
    ``sensors.baudrate`` only to force a specific rate."""
    finger_to_sensor_id: Dict[str, int] = field(
        default_factory=lambda: dict(DEFAULT_FINGER_TO_SENSOR_ID)
    )

    @classmethod
    def from_config_path(
        cls,
        config_path: str | None = None,
        calibration_path: str | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
    ) -> "OrcaHandTouchConfig":
        kwargs, config = cls._kwargs_from_config_path(
            config_path, calibration_path, model_version, model_name
        )
        try:
            sensor_kwargs = _sensor_kwargs(config.get("sensors", {}))
        except Exception:
            OrcaHandConfig(**kwargs)  # a base-field error is reported ahead of a sensors one
            raise

        return cls(**kwargs, **sensor_kwargs).with_hardware_version(kwargs.get("hardware_version"))

    def validate_config(self) -> None:
        super().validate_config()

        if set(self.finger_to_sensor_id.keys()) != set(FINGER_NAMES):
            raise HandConfigValidationError(
                f"finger_to_sensor_id must contain exactly {FINGER_NAMES}, "
                f"got {sorted(self.finger_to_sensor_id.keys())}"
            )

        ids = set(self.finger_to_sensor_id.values())
        if ids != VALID_SENSOR_IDS:
            raise HandConfigValidationError(
                f"finger_to_sensor_id values must be 0-4 with no duplicates, "
                f"got {sorted(self.finger_to_sensor_id.values())}"
            )
