"""Joint angles from 2-D observations of known points on the hand.

An observation is one point on a link (in that link's frame, e.g. the
fingertip) seen at one pixel at one plate stop. The rig (camera, axis, hand
mount) is held; only the named joints move, the others stay at ``q_fixed``.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from visual_calibration.hand_model import HandModel
from visual_calibration.rig import Rig
from visual_calibration.solve import Fit, least_squares


@dataclass(frozen=True)
class Observation:
    stop_deg: float
    link: str
    point: np.ndarray          # in the link frame, metres
    uv: np.ndarray             # pixel


def predict(rig: Rig, model: HandModel, q_deg: Mapping[str, float],
            observations: Sequence[Observation]) -> np.ndarray:
    poses = model.link_poses(q_deg)
    return np.vstack([rig.project_poses(poses, o.stop_deg, [(o.link, o.point)]) for o in observations])


def fit_joints(rig: Rig, model: HandModel, observations: Sequence[Observation], joints: Sequence[str],
               q_start: Mapping[str, float], q_fixed: Mapping[str, float] | None = None) -> tuple[dict, Fit]:
    """Least squares on the pixel error; joints bounded by the URDF limits.
    Returns ({joint: angle deg}, Fit with std in degrees, rms in px)."""
    base = dict(q_fixed or {})
    uv = np.array([o.uv for o in observations])

    def residual(x):
        return (predict(rig, model, base | dict(zip(joints, x)), observations) - uv).ravel()

    limits = [model.joints[j].limits_deg or (-180.0, 180.0) for j in joints]
    fit = least_squares(residual, [q_start[j] for j in joints],
                        [lo for lo, _ in limits], [hi for _, hi in limits], step=1e-3)
    return dict(zip(joints, fit.x)), fit
