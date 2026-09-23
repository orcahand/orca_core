"""The HLS register map is pinned to the vendor memory table by literal value.

Every other test compares addresses against the map's own names, which
cannot catch a wrong entry; these literals can.
"""

import numpy as np

from orca_core.hardware.feetech_registers import HLS


def test_register_addresses_match_the_vendor_memory_table():
    assert (HLS.PROTECTION_CURRENT, HLS.GOAL_CURRENT, HLS.TORQUE_LIMIT, HLS.PRESENT_CURRENT) == (28, 44, 48, 69)
    assert (HLS.ID, HLS.BAUD_RATE, HLS.MODE, HLS.LOCK) == (5, 6, 33, 55)
    assert (HLS.TORQUE_ENABLE, HLS.ACC, HLS.GOAL_POSITION, HLS.GOAL_SPEED) == (40, 41, 42, 46)
    assert (HLS.PRESENT_POSITION, HLS.PRESENT_SPEED, HLS.PRESENT_TEMPERATURE, HLS.MOVING) == (56, 58, 63, 66)
    assert HLS.POSITION_PROFILE_LEN == 7 == HLS.GOAL_SPEED + 2 - HLS.ACC


def test_goal_current_scale_and_range_match_the_vendor_memory_table():
    assert HLS.CURRENT_SCALE_MA == 6.5
    assert HLS.GOAL_CURRENT_MAX_RAW == 2047
    # 300 mA -> 46 units (299 mA); 650 -> 100; 500 -> 76; full scale is 13305.5 mA.
    assert [int(ma / HLS.CURRENT_SCALE_MA) for ma in (300, 650, 500)] == [46, 100, 76]
    assert np.isclose(HLS.GOAL_CURRENT_MAX_RAW * HLS.CURRENT_SCALE_MA, 13305.5)
