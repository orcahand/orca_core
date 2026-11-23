"""A present position written back as a goal ("hold where you are") must land on
the same encoder step; one step off, the motor pushes against it and heats up."""

import numpy as np

from orca_core.hardware import dynamixel_client, feetech_client
from orca_core.hardware.dynamixel_client import DynamixelClient

STEPS = range(-4096, 4097)


def test_feetech_writes_a_read_back_position_on_its_own_step():
    scale = feetech_client.DEFAULT_POS_SCALE
    off = [raw for raw in STEPS
           if feetech_client._rad_to_raw_units(feetech_client._raw_units_to_rad(raw, scale), scale) != raw]
    assert off == []


def test_dynamixel_writes_a_read_back_position_on_its_own_step():
    client = DynamixelClient.__new__(DynamixelClient)
    client._goal_positions = {}
    written = []
    client.sync_write = lambda ids, values, address, size: written.extend(int(v) for v in values)
    raws = list(STEPS)
    read_back = np.array([float(raw) * dynamixel_client.DEFAULT_POS_SCALE for raw in raws])

    client.write_desired_pos(list(range(len(raws))), read_back)

    assert [w for w, raw in zip(written, raws) if w != raw] == []
