# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from .base_hand import BaseHand
from .hardware_hand import HardwareOrcaHand, MockOrcaHand
from .joint_position import OrcaJointPosition


class OrcaHand(HardwareOrcaHand):
    """Primary entry point for controlling an ORCA robotic hand.

    ``OrcaHand`` is a thin alias for :class:`~orca_core.HardwareOrcaHand` kept
    for backward compatibility. All functionality is inherited unchanged;
    refer to :class:`~orca_core.HardwareOrcaHand` for the full API reference.

    Example:
        >>> hand = OrcaHand()
        >>> hand.connect()
        >>> hand.init_joints()
        >>> hand.set_joint_pos({"index_mcp": 0.8, "thumb_pip": 0.4})
        >>> hand.disconnect()
    """
    pass


if __name__ == "__main__":
    hand = OrcaHand()
    status = hand.connect()
    hand.enable_torque()
    hand.calibrate()
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disable_torque()
    hand.disconnect()
