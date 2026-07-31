"""Cross-family contract tests for ``MotorClient`` implementations.

The mock must fail exactly where the real clients fail: any bus method
called before ``connect()`` raises, unknown motor IDs are logged and
reported (never a mock-only exception), and ``set_torque_enabled`` shares
one finite-retry signature across the whole family so a dead motor can
never wedge the hand.
"""

import inspect

import numpy as np
import pytest

from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient
from orca_core.hardware.motor_client import MotorClient


@pytest.fixture
def disconnected_mock():
    client = MockDynamixelClient([1, 2], port="mock")
    yield client
    MockDynamixelClient.OPEN_CLIENTS.discard(client)


@pytest.fixture
def connected_mock():
    client = MockDynamixelClient([1, 2], port="mock")
    client.connect()
    yield client
    client.disconnect()
    MockDynamixelClient.OPEN_CLIENTS.discard(client)


@pytest.mark.parametrize(
    "call",
    [
        pytest.param(lambda c: c.read_position_velocity_current(), id="read_pos_vel_cur"),
        pytest.param(lambda c: c.set_torque_enabled([1], True), id="set_torque_enabled"),
        pytest.param(lambda c: c.set_operating_mode([1], 3), id="set_operating_mode"),
        pytest.param(lambda c: c.write_desired_pos([1], np.zeros(1)), id="write_desired_pos"),
        pytest.param(lambda c: c.write_desired_current([1], np.zeros(1)), id="write_desired_current"),
        pytest.param(lambda c: c.write_profile_velocity([1], np.zeros(1)), id="write_profile_velocity"),
        pytest.param(lambda c: c.read_temperature(), id="read_temperature"),
        pytest.param(lambda c: c.read_status_is_done_moving(), id="read_status_is_done_moving"),
        pytest.param(lambda c: c.sync_write([1], [0], 116, 4), id="sync_write"),
        pytest.param(lambda c: c.write_byte([1], 0, 64), id="write_byte"),
    ],
)
def test_mock_bus_methods_raise_when_disconnected(disconnected_mock, call):
    with pytest.raises(OSError, match="Must call connect"):
        call(disconnected_mock)


def test_mock_bus_methods_work_when_connected(connected_mock):
    assert connected_mock.set_torque_enabled([1, 2], True) == []
    assert all(connected_mock._torque_enabled.values())
    read = connected_mock.read_position_velocity_current()
    assert read.position.shape == (2,)


def test_mock_set_torque_enabled_returns_unknown_ids_as_failed(connected_mock):
    assert connected_mock.set_torque_enabled([1, 99], True) == [99]
    assert connected_mock._torque_enabled[1] is True


def test_mock_set_operating_mode_skips_unknown_ids(connected_mock):
    connected_mock.set_operating_mode([1, 99], 5)
    assert connected_mock._operating_mode[1] == 5


def test_mock_writes_skip_unknown_ids(connected_mock):
    connected_mock.write_desired_pos([1, 99], np.array([0.5, 0.5]))
    assert connected_mock._pos[1] == pytest.approx(0.5)
    assert 99 not in connected_mock._pos


def test_set_torque_enabled_signature_is_uniform_across_family():
    """One retry contract for the whole family: finite default, 0 = single
    attempt, and the failed IDs returned so callers can react."""
    base = inspect.signature(MotorClient.set_torque_enabled).parameters
    assert base["retries"].default == 3
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient):
        params = inspect.signature(cls.set_torque_enabled).parameters
        assert params["retries"].default == 3, cls.__name__
        assert params["retry_interval"].default == 0.25, cls.__name__
