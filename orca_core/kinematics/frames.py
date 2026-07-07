# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Frame names used by the tactile/kinematics API.

The frame stack, innermost first:

* ``sensor`` — a tactile sensor's own frame; taxel positions and forces are
  natively expressed here. One per finger.
* ``fingertip`` — the distal link frame (at the distal revolute joint). The
  sensor sits at a fixed mount pose in this frame. One per finger.
* ``palm`` — the carpals link; moves with the wrist.
* ``base`` — the static root of the hand (forearm structure the hand is
  mounted by).
* ``world`` — ``base`` composed with a user-supplied hand pose
  (identity unless set).
"""
from __future__ import annotations

SENSOR = "sensor"
FINGERTIP = "fingertip"
PALM = "palm"
BASE = "base"
WORLD = "world"

FRAMES = (SENSOR, FINGERTIP, PALM, BASE, WORLD)
