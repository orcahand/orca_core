# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Forward kinematics for the ORCA hand from packaged URDF-derived constants.

Loads per-hand kinematic chains and tactile sensor mount poses from
``data/v*_kinematics.yaml`` (see that file's header for provenance) and
computes fingertip / sensor poses in the palm or base frame from orca_core
joint angles (degrees, orca_core joint ids and sign conventions).
"""
from __future__ import annotations

import os
from collections.abc import Mapping
from functools import lru_cache

import numpy as np
import yaml

from orca_core.kinematics import frames
from orca_core.kinematics.transforms import Transform, rotation_about_axis

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def _joint_transform(entry: dict, joint_pos_deg: Mapping[str, float]) -> Transform:
    origin = Transform.from_xyz_rpy(entry["xyz"], entry["rpy"])
    if "joint" not in entry:  # fixed
        return origin
    angle_deg = joint_pos_deg.get(entry["joint"], 0.0) or 0.0
    angle = entry["sign"] * np.deg2rad(angle_deg)
    rotation = rotation_about_axis(np.array(entry["axis"]), angle)
    return origin @ Transform.from_rotation_translation(rotation, np.zeros(3))


class HandKinematics:
    """FK and sensor mounts for one hand (type + model version)."""

    def __init__(self, chains: dict, sensor_mounts: dict[str, dict] | None, hand_type: str):
        self._chains = chains
        self._sensor_mounts = {
            finger: Transform.from_xyz_rpy(mount["xyz"], mount["rpy"])
            for finger, mount in (sensor_mounts or {}).items()
        }
        self.hand_type = hand_type

    @classmethod
    @lru_cache(maxsize=None)
    def load(cls, hand_type: str, version: int = 2) -> HandKinematics:
        """Load packaged kinematics for ``hand_type`` (``"right"``/``"left"``)."""
        path = os.path.join(DATA_DIR, f"v{version}_kinematics.yaml")
        with open(path) as f:
            data = yaml.safe_load(f)
        if hand_type not in data:
            raise KeyError(f"no kinematics for hand type {hand_type!r} in {path}")
        side = data[hand_type]
        return cls(side["chains"], side.get("sensor_mounts"), hand_type)

    @property
    def sensor_mounts(self) -> dict[str, Transform]:
        """``{finger: T_fingertip_sensor}``; the sensor's fixed pose in its distal link."""
        if not self._sensor_mounts:
            raise NotImplementedError(
                f"sensor mounts are not available for the {self.hand_type} hand yet"
            )
        return dict(self._sensor_mounts)

    def fingertip_poses(
        self,
        joint_pos_deg: Mapping[str, float],
        in_frame: str = frames.BASE,
        fingers: tuple[str, ...] = FINGERS,
    ) -> dict[str, Transform]:
        """``{finger: T_frame_fingertip}`` for the given joint angles (degrees).

        ``in_frame`` is ``"palm"`` (carpals, wrist excluded) or ``"base"``
        (static root, wrist included). Missing joints default to 0.
        """
        if in_frame == frames.PALM:
            root = Transform.identity()
        elif in_frame == frames.BASE:
            root = Transform.identity()
            for entry in self._chains["base_chain"]:
                root = root @ _joint_transform(entry, joint_pos_deg)
        else:
            raise ValueError(f"in_frame must be 'palm' or 'base', got {in_frame!r}")

        poses = {}
        for finger in fingers:
            pose = root
            for entry in self._chains["fingers"][finger]:
                pose = pose @ _joint_transform(entry, joint_pos_deg)
            poses[finger] = pose
        return poses

    def sensor_poses(
        self,
        joint_pos_deg: Mapping[str, float],
        in_frame: str = frames.BASE,
        fingers: tuple[str, ...] = FINGERS,
    ) -> dict[str, Transform]:
        """``{finger: T_frame_sensor}``: fingertip pose composed with the sensor mount."""
        mounts = self.sensor_mounts
        fingers = tuple(f for f in fingers if f in mounts)
        tips = self.fingertip_poses(joint_pos_deg, in_frame=in_frame, fingers=fingers)
        return {finger: tips[finger] @ mounts[finger] for finger in fingers}
