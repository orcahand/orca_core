# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Per-hand absolute sensor-calibration storage.

Absolute joint-sensor calibration (per-joint encoder zero + scale from an
external fixture) is device state, not session state: it travels with the
hand, changes rarely, and outlives any single ``calibration.yaml``. A
:class:`SensorCalStore` provides it to the hand at connect time; the
hardstop-anchored entries in ``calibration.yaml`` remain the fallback for
hands that don't have one.

:class:`FileSensorCalStore` is the host-side backend (also used as the
backup target once calibration lives on the hand itself); a device-blob
backend reading the connector board's flash will join it.
"""

import os
from abc import ABC, abstractmethod
from typing import Dict, Optional

import yaml

from .calibration import (
    JointEncoderCal,
    _parse_joint_encoder_entry,
    joint_encoder_calibration_to_yaml,
)
from .utils.utils import read_yaml

SENSOR_CAL_FILE_VERSION = 1


class SensorCalStore(ABC):
    """Source/sink for a hand's absolute joint-sensor calibration."""

    provenance: str = "unknown"

    @abstractmethod
    def load(self) -> Optional[Dict[str, JointEncoderCal]]:
        """Return the stored per-joint calibration, or ``None`` when the
        store has nothing usable."""

    @abstractmethod
    def store(
        self,
        joint_encoder_calibration_dict: Dict[str, JointEncoderCal],
        hand_serial: str | None = None,
    ) -> None:
        """Persist the given per-joint calibration, replacing any prior
        content."""


class FileSensorCalStore(SensorCalStore):
    """YAML-file backend (``sensor_calibration.yaml`` next to
    ``calibration.yaml`` by default)."""

    provenance = "file"

    def __init__(self, path: str):
        self.path = path

    def load(self) -> Optional[Dict[str, JointEncoderCal]]:
        if not os.path.exists(self.path):
            return None
        data = read_yaml(self.path) or {}

        version = data.get("version", SENSOR_CAL_FILE_VERSION)
        if version > SENSOR_CAL_FILE_VERSION:
            print(
                f"\033[93mWarning: {self.path} has sensor-calibration version "
                f"{version} (this build understands ≤{SENSOR_CAL_FILE_VERSION}); "
                f"attempting to read the known fields.\033[0m"
            )

        loaded = {}
        for joint, entry in (data.get("joints", {}) or {}).items():
            cal = _parse_joint_encoder_entry(joint, entry)
            if cal is not None:
                loaded[joint] = cal
        return loaded or None

    def store(
        self,
        joint_encoder_calibration_dict: Dict[str, JointEncoderCal],
        hand_serial: str | None = None,
    ) -> None:
        payload = {
            "version": SENSOR_CAL_FILE_VERSION,
            "joints": joint_encoder_calibration_to_yaml(
                joint_encoder_calibration_dict
            ),
        }
        if hand_serial is not None:
            payload["hand_serial"] = hand_serial
        with open(self.path, "w") as f:
            yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=True)
