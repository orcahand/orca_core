# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Guarded motor reads shared by the maintenance routines."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from ..constants import TINY_SLEEP

if TYPE_CHECKING:
    from ..hardware_hand import OrcaHand


def read_motor_pos_checked(
    hand: "OrcaHand", retries: int = 5, retry_interval: float = TINY_SLEEP
) -> np.ndarray:
    """Read motor positions, rejecting a bulk read the bus never answered.

    A dropped status packet leaves ``get_motor_pos`` returning the stale
    cache, which reads as zero motion (a false stall) and, at limit capture,
    as the tension-biased pre-release position. Retry, and raise rather than
    let a caller act on a stale sample.
    """
    for _ in range(retries):
        with hand._motor_lock:
            motor_pos = hand.get_motor_pos()
            read_ok = hand.motor_client.last_read_ok
        if read_ok:
            return motor_pos
        time.sleep(retry_interval)
    raise RuntimeError(
        "motor position read failed: the motor bus returned no status packets"
    )
