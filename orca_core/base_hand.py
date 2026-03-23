# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import time
from abc import ABC, abstractmethod

from .hand_config import HandConfig
from .joint_position import OrcaJointPositions


class BaseHand(ABC):
    """Abstract base class defining the shared joint-space interface for all ORCA hand backends.

    Concrete subclasses provide hardware-specific implementations of
    :meth:`_get_joint_pos_impl` and :meth:`_set_joint_pos_impl`.
    All higher-level motion helpers (interpolation, normalisation, neutral
    position) live here so they are available regardless of backend.

    On construction the hand loads its :class:`~orca_core.HandConfig`,
    validates it, and registers its default joint ordering with
    :class:`~orca_core.OrcaJointPosition`.

    Args:
        config_path: Path to a ``config.yaml`` file. When ``None`` the default
            model bundled with the package is used.
    """

    def __init__(
        self,
        config_path: str | None = None,
    ):
        self.config = HandConfig.from_config_path(config_path=config_path)
        self.config.validate()
        
        self.positions: dict[str, OrcaJointPositions] = {}
    
    def set_joint_positions(
        self,
        joint_pos: OrcaJointPositions,
        num_steps: int = 1,
        step_size: float = 1e-2,
):
        # TODO(fracapuano): Move to hardare hand. The logic for setting joint positions along a trajectory is specific to the real-world hardware
        """Command the hand to a target joint configuration.

        Positions are clamped to configured ROM bounds before being sent to
        the hardware. 
        When *num_steps* > 1 the motion is linearly interpolated
        from the current position, with a ``time.sleep(step_size)`` pause
        between each intermediate waypoint.

        Args:
            joint_pos: Target joint positions as an
                :class:`~orca_core.OrcaJointPosition`, a ``dict``, or a 1-D
                ``np.ndarray`` aligned with :attr:`joint_ids`.
            num_steps: Number of interpolation steps. Use ``1`` for an
                immediate move (default).
            step_size: Sleep duration in seconds between interpolated steps.
                Ignored when *num_steps* is ``1``.
        """
        joint_pos = self.config.clamp_joint_positions(joint_pos)

        if num_steps > 1:
            current_pos = self._get_joint_positions()
            for step in range(num_steps + 1):
                t = step / num_steps
                interpolated = OrcaJointPositions.from_dict({
                    joint: current_pos.data[joint] * (1 - t) + joint_pos.data.get(joint, current_pos.data[joint]) * t
                    for joint in current_pos.data
                })
                self._set_joint_positions(interpolated)
                
                if step < num_steps:
                    time.sleep(step_size)
            return

        self._set_joint_positions(joint_pos)
    
    def register_position(self, name: str, joint_pos: OrcaJointPositions):
        self.positions[name] = joint_pos
    
    def remove_position(self, name: str):
        try:
            del self.positions[name]
        except KeyError:
            pass  # position was not among registered
    
    def set_named_position(self, name: str, num_steps: int = 1, step_size: float = 1.0):
        self.set_joint_positions(self.positions[name], num_steps=num_steps, step_size=step_size)

    def set_neutral_position(self, num_steps: int = 25, step_size: float = 0.001):
        """Move hand to neutral position."""
        self.set_joint_positions(
            OrcaJointPositions.from_dict(self.config.neutral_position),
            num_steps=num_steps,
            step_size=step_size,
        )

    @abstractmethod
    def _get_joint_positions(self) -> OrcaJointPositions:
        """Return the current joint positions as an :class:`~orca_core.OrcaJointPosition`."""

    @abstractmethod
    def _set_joint_positions(self, joint_pos: OrcaJointPositions) -> bool:
        """Apply a joint command. Returns ``True`` if the command was successful, ``False`` otherwise."""
