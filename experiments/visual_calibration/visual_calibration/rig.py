"""The rig as geometry: a fixed camera, a plate turning about the rig's z axis,
the hand standing on the plate.

Rig frame: z up along the turntable axis, origin on the plate's top face at
the axis. Plate frame: the rig frame turned by the stop angle about z.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from orca_core.kinematics.transforms import Transform, rotation_about_axis

from visual_calibration.camera import Camera
from visual_calibration.hand_model import HandModel

# The URDF base (forearm) has its long axis along +y and its bottom face at
# y = -58 mm: stand it up on the plate (forearm y -> rig z, forearm z -> rig -y).
NOMINAL_HAND_MOUNT = Transform.from_rotation_translation(
    np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]), [0.0, 0.0, 0.058])


@dataclass(frozen=True)
class Rig:
    camera: Camera
    hand_mount: Transform = NOMINAL_HAND_MOUNT    # hand base in the plate frame

    @staticmethod
    def plate(stop_deg: float) -> Transform:
        return Transform.from_rotation_translation(
            rotation_about_axis(np.array([0.0, 0.0, 1.0]), np.deg2rad(stop_deg)), np.zeros(3))

    def hand_in_rig(self, stop_deg: float) -> Transform:
        return self.plate(stop_deg) @ self.hand_mount

    def project(self, model: HandModel, q_deg: Mapping[str, float], stop_deg: float,
                points: list[tuple[str, np.ndarray]]) -> np.ndarray:
        """Pixels of (link, point in link) pairs at one stop, (n, 2)."""
        return self.project_poses(model.link_poses(q_deg), stop_deg, points)

    def project_poses(self, poses: Mapping[str, Transform], stop_deg: float,
                      points: list[tuple[str, np.ndarray]]) -> np.ndarray:
        hand = self.hand_in_rig(stop_deg)
        world = np.array([(hand @ poses[link]).apply_to_points(p) for link, p in points])
        return self.camera.project(world)
