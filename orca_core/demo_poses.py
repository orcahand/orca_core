# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Packaged demo poses, shared by the bundled examples and external front-ends.

Data, not behaviour: each pose maps joint name to a fraction of that joint's
ROM, which :meth:`~orca_core.base_hand.BaseHand.pose_from_fractions` turns into
joint angles for a specific hand. Playing a sequence is the caller's job

Deliberately not re-exported from the ``orca_core`` namespace: demo content is
not part of the hand-control API. Import it explicitly::

    from orca_core.demo_poses import load_demo_poses
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache
from types import MappingProxyType
from typing import Mapping

from .utils.utils import read_yaml

DATA_DIR = os.path.join(os.path.dirname(__file__), "data")


class DemoPoseError(ValueError):
    """Raised when the packaged demo-pose data is malformed."""


@dataclass(frozen=True)
class DemoPoses:
    """One named demo: its poses, and the order they are played in.

    Attributes:
        pose_fractions: Maps pose name → ``{joint: fraction of ROM}``.
        sequence: Pose names in playback order. Every name is a key of
            :attr:`pose_fractions`.
    """

    pose_fractions: Mapping[str, Mapping[str, float]]
    sequence: tuple[str, ...]


@lru_cache(maxsize=None)
def load_demo_poses() -> Mapping[str, DemoPoses]:
    """Return the packaged demos keyed by name (``"main"``, ``"abduction"``).

    The result is cached and read-only; copy it before mutating.
    """
    raw = read_yaml(os.path.join(DATA_DIR, "demo_poses.yaml")) or {}

    demos: dict[str, DemoPoses] = {}
    for name, entry in raw.items():
        poses = entry.get("poses") or {}
        sequence = tuple(entry.get("sequence") or ())
        unknown = [pose for pose in sequence if pose not in poses]
        if unknown:
            raise DemoPoseError(f"demo {name!r} sequences undefined poses: {unknown}")
        demos[name] = DemoPoses(
            pose_fractions=MappingProxyType(
                {pose: MappingProxyType(dict(fractions)) for pose, fractions in poses.items()}
            ),
            sequence=sequence,
        )
    return MappingProxyType(demos)
