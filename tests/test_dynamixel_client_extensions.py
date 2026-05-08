"""Tests for the host-side joint loop's motor-client additions:
``sync_write_current`` (mA-typed) and ``set_operating_mode_per_motor``.
"""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.hardware.dynamixel_client import (
    DEFAULT_CUR_SCALE,
    currents_mA_to_dxl_units,
)
from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient


@pytest.fixture
def mock_client():
    client = MockDynamixelClient(motor_ids=[1, 2, 3], port="/dev/null")
    client.connect()
    try:
        yield client
    finally:
        client.disconnect()


def test_currents_mA_to_dxl_units_truncates_toward_zero():
    units = currents_mA_to_dxl_units(
        np.array([0.0, 134.0, -134.0, 1.0, -1.0]), DEFAULT_CUR_SCALE
    )
    assert units.dtype.kind == "i"
    assert units.tolist() == [0, 100, -100, 0, 0]


def test_sync_write_current_records_mA_per_motor(mock_client):
    mock_client.sync_write_current([1, 2, 3], np.array([10.0, -20.5, 100.0]))
    assert mock_client.get_last_currents_mA() == {1: 10.0, 2: -20.5, 3: 100.0}


def test_set_operating_mode_per_motor_updates_each(mock_client):
    mock_client.set_operating_mode_per_motor([1, 2, 3], [0, 5, 0])
    assert mock_client.get_operating_modes() == {1: 0, 2: 5, 3: 0}
