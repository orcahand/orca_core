# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import time
from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence

import numpy as np

from .demo_presets import DEMO_POSE_FRACTIONS, DEMO_SEQUENCES
from .hand_config import BaseHandConfig
from .joint_position import OrcaJointPositions

from .constants import STEPS_TO_NEUTRAL, STEP_SIZE_NEUTRAL


class BaseHand(ABC):
    """Abstract base class defining the shared joint-space interface for all ORCA hand backends.

    Concrete subclasses implement :meth:`_get_joint_positions` and
    :meth:`_set_joint_positions`.
    All higher-level motion helpers (interpolation, normalisation, neutral
    position) live here so they are available regardless of backend.

    On construction the hand loads its :class:`~orca_core.BaseHandConfig`,
    validates it, and registers its default joint ordering with
    :class:`~orca_core.OrcaJointPosition`.

    The class exposes methods for:
    - Setting joint positions, thus reaching arbitrary hand specifications
    - Recording and replaying named positions
    - Moving to the neutral position defined in the hand configuration file, ``config.yaml``.

    Args:
        config_path: Path to a ``config.yaml`` file. When ``None`` the default
            model bundled with the package is used.
    """

    config_cls = BaseHandConfig

    def __init__(
        self,
        config_path: str | None = None,
        config: BaseHandConfig | None = None,
        model_version: str | None = None,
        model_name: str | None = None,
        **config_kwargs,
    ):
        self.config = (
            config
            if config is not None
            else self.config_cls.from_config_path(
                config_path=config_path,
                model_version=model_version,
                model_name=model_name,
                **config_kwargs,
            )
        )
        self.config.validate()
        OrcaJointPositions.register_joint_names(self.config.joint_ids)

        self.recorded_positions: dict[str, OrcaJointPositions] = {}

    @abstractmethod
    def _get_joint_positions(self) -> OrcaJointPositions:
        """Return the current joint configuration."""
        pass

    @abstractmethod
    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        """Apply a joint-space command. Return ``True`` if the command was applied."""
        pass

    def _coerce_joint_positions(
        self,
        joint_pos: OrcaJointPositions | Mapping[str, float | None] | np.ndarray,
    ) -> OrcaJointPositions:
        if isinstance(joint_pos, OrcaJointPositions):
            return joint_pos

        if isinstance(joint_pos, Mapping):
            return OrcaJointPositions.from_dict(dict(joint_pos))

        if isinstance(joint_pos, np.ndarray):
            return OrcaJointPositions.from_ndarray(joint_pos, joint_ids=self.config.joint_ids)

        raise TypeError(
            "joint_pos must be an OrcaJointPositions instance, a mapping, or a 1-D numpy array."
        )

    def pose_from_fractions(
        self,
        fractions: Mapping[str, float],
    ) -> OrcaJointPositions:
        """Build a pose by specifying joints as fractions of their ROM, read from config."""
        pose_dict = {joint: 0.0 for joint in self.config.joint_ids}
        pose_dict.update(self.config.neutral_position)

        for joint, fraction in fractions.items():
            if joint not in self.config.joint_roms_dict:
                continue

            joint_min, joint_max = self.config.joint_roms_dict[joint]
            pose_dict[joint] = joint_min + float(fraction) * (joint_max - joint_min)

        return self.config.clamp_joint_positions(OrcaJointPositions.from_dict(pose_dict))

    def set_joint_positions(
        self,
        joint_pos: OrcaJointPositions | Mapping[str, float | None] | np.ndarray,
        num_steps: int = 1,
        step_size: float = 1e-2,
    ):
        """Command the hand to a target joint configuration.

        Positions are clamped to configured ROM bounds before being sent to
        the hardware for increased safety.
        When *num_steps* > 1 the motion is linearly interpolated
        from the current position, with a ``time.sleep(step_size)`` pause
        between each intermediate waypoint.

        Args:
            joint_pos: Target joint positions as an
                :class:`~orca_core.OrcaJointPosition`, a ``dict``, or a 1-D
                ``np.ndarray`` aligned with :attr:`joint_ids`.
            num_steps: Number of interpolation steps. Use ``1`` for an
                immediate move (default). Simulation hands should always use ``1``.
            step_size: Sleep duration in seconds between interpolated steps.
                Ignored when *num_steps* is ``1``.
        """
        joint_pos = self._coerce_joint_positions(joint_pos)
        joint_pos = self.config.clamp_joint_positions(joint_pos)
        waypoints = self._linear_waypoints_to(joint_pos, num_steps)
        for i, wp in enumerate(waypoints):
            self._set_joint_positions(wp)
            if i < len(waypoints) - 1:
                time.sleep(step_size)

    def get_joint_position(self) -> OrcaJointPositions:
        return self._get_joint_positions()

    def _linear_waypoints_to(
        self, target: OrcaJointPositions, num_steps: int
    ) -> list[OrcaJointPositions]:
        """Return linear waypoints from current pose to *target*."""
        # TODO(fracapuano): Move this to a stateless util function
        if num_steps <= 1:
            return [target]

        current = self._get_joint_positions()
        out: list[OrcaJointPositions] = []
        for step in range(num_steps + 1):
            t = step / num_steps
            out.append(
                OrcaJointPositions.from_dict({
                    joint: current.data[joint] * (1 - t)
                    + target.data.get(joint, current.data[joint]) * t
                    for joint in current.data
                })
            )
        return out

    def register_position(self, name: str, joint_pos: OrcaJointPositions):
        self.recorded_positions[name] = self.config.clamp_joint_positions(joint_pos)

    def remove_position(self, name: str):
        try:
            del self.recorded_positions[name]
        except KeyError:
            pass  # position was not among recorded positions

    def set_named_position(self, name: str, num_steps: int = 1, step_size: float = 1.0):
        self.set_joint_positions(self.recorded_positions[name], num_steps=num_steps, step_size=step_size)

    def play_named_positions(
        self,
        names: Sequence[str],
        cycles: int = 1,
        num_steps: int = 1,
        step_size: float = 1.0,
        return_to_neutral: bool = False,
        neutral_num_steps: int | None = None,
        neutral_step_size: float | None = None,
    ) -> None:
        if cycles < 0:
            raise ValueError("cycles must be non-negative.")

        for _ in range(cycles):
            for name in names:
                self.set_named_position(name, num_steps=num_steps, step_size=step_size)
            if return_to_neutral:
                self.set_neutral_position(
                    num_steps=num_steps if neutral_num_steps is None else neutral_num_steps,
                    step_size=step_size if neutral_step_size is None else neutral_step_size,
                )

    def run_demo(
        self,
        demo_name: str = "main",
        cycles: int = 1,
        num_steps: int = STEPS_TO_NEUTRAL,
        step_size: float = STEP_SIZE_NEUTRAL,
        return_to_neutral: bool = True,
    ) -> tuple[str, ...]:
        if demo_name not in DEMO_POSE_FRACTIONS or demo_name not in DEMO_SEQUENCES:
            available = ", ".join(sorted(DEMO_SEQUENCES))
            raise ValueError(f"Unknown demo '{demo_name}'. Available demos: {available}.")

        poses = {
            name: self.pose_from_fractions(fractions)
            for name, fractions in DEMO_POSE_FRACTIONS[demo_name].items()
        }
        sequence = DEMO_SEQUENCES[demo_name]

        for _ in range(cycles):
            for name in sequence:
                self.set_joint_positions(poses[name], num_steps=num_steps, step_size=step_size)
            
            if return_to_neutral:
                self.set_neutral_position(num_steps=num_steps, step_size=step_size)

        return sequence

    def set_neutral_position(self, num_steps: int = STEPS_TO_NEUTRAL, step_size: float = STEP_SIZE_NEUTRAL):
        """Move hand to neutral position."""
        self.set_joint_positions(
            OrcaJointPositions.from_dict(self.config.neutral_position),
            num_steps=num_steps,
            step_size=step_size,
        )

    def set_zero_position(self, num_steps: int = STEPS_TO_NEUTRAL, step_size: float = STEP_SIZE_NEUTRAL):
        self.set_joint_positions(
            OrcaJointPositions.from_dict(
                {joint: 0.0 for joint in self.config.joint_ids}
            ),
            num_steps=num_steps,
            step_size=step_size,
        )
