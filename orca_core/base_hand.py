# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Union

from .hand_spec import HandSpec


class BaseHand(ABC):
    """Shared joint-space interface for ORCA hand backends."""

    def __init__(self, model_path: str | None = None):
        self.spec = HandSpec.from_model_path(model_path)
        self.spec.validate_shared()
        self.spec.apply_to_instance(self)

    def get_joint_pos(self, as_list: bool = True) -> Union[dict, list]:
        joint_pos = self._get_joint_pos_impl()
        if as_list:
            return [joint_pos[joint] for joint in self.joint_ids]
        return joint_pos

    def set_joint_pos(self, joint_pos: Union[dict, list], num_steps: int = 1, step_size: float = 1.0):
        normalized_command = self.spec.normalize_joint_command(joint_pos)
        if not normalized_command:
            return

        if num_steps > 1:
            current_positions = self.get_joint_pos(as_list=False)
            target_positions = {
                joint: normalized_command.get(joint, current_positions[joint]) for joint in self.joint_ids
            }

            for step in range(num_steps + 1):
                t = step / num_steps
                interpolated_positions = {}
                for joint in self.joint_ids:
                    current_pos = current_positions[joint]
                    target_pos = target_positions[joint]
                    if current_pos is None or target_pos is None:
                        interpolated_positions[joint] = None
                        continue
                    interpolated_positions[joint] = current_pos * (1 - t) + target_pos * t

                self._set_joint_pos_impl(interpolated_positions)
                if step < num_steps:
                    time.sleep(step_size)
            return

        self._set_joint_pos_impl(normalized_command)

    def set_zero_position(self, num_steps: int = 25, step_size: float = 0.001):
        self.set_joint_pos({joint: 0.0 for joint in self.joint_ids}, num_steps=num_steps, step_size=step_size)

    def set_neutral_position(self, num_steps: int = 25, step_size: float = 0.001):
        if self.neutral_position is None:
            raise ValueError("Neutral position is not set. Please set the neutral position in the config.yaml file.")
        self.set_joint_pos(self.neutral_position, num_steps=num_steps, step_size=step_size)

    @abstractmethod
    def _get_joint_pos_impl(self) -> dict:
        """Return the current joint positions as a full dict."""

    @abstractmethod
    def _set_joint_pos_impl(self, joint_pos: dict) -> None:
        """Apply a joint command dict. Partial dicts must be supported."""
