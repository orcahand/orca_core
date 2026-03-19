# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

from .base_hand import BaseHand
from .hardware_hand import HardwareOrcaHand, MockOrcaHand


class OrcaHand(HardwareOrcaHand):
    """Backward-compatible hardware-backed ORCA hand entry point."""


if __name__ == "__main__":
    hand = OrcaHand()
    status = hand.connect()
    hand.enable_torque()
    hand.calibrate()
    hand.set_joint_pos({joint: 0 for joint in hand.joint_ids})
    hand.disable_torque()
    hand.disconnect()
