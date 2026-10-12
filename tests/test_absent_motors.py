"""Motors that never answer the connect-time ping are left out of every read
and write, so a hand with fingers missing from the bus reads fresh and fast
instead of timing out on each silent ID."""

import sys

import pytest

from orca_core.hardware.dynamixel_client import DynamixelClient
from tests.test_dynamixel_client_locking import COMM_RX_FAIL, COMM_SUCCESS, FakeBus, make_fake_sdk


@pytest.fixture
def partial_client(monkeypatch, request):
    bus = FakeBus()
    bus.ping_hook = lambda motor_id: COMM_RX_FAIL if motor_id == 2 else COMM_SUCCESS
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", make_fake_sdk(bus))
    client = DynamixelClient([1, 2, 3], port="/dev/fake", baudrate=57600)
    client.port_handler.is_open = True

    def cleanup():
        client.port_handler.is_open = False
        DynamixelClient.OPEN_CLIENTS.discard(client)

    request.addfinalizer(cleanup)
    client._read_model_numbers()
    client._adopt_responding()
    return client, bus


def test_the_silent_motor_is_known_and_the_rest_answer(partial_client):
    client, _ = partial_client
    assert client.responding_motor_ids() == [1, 3]
    assert client.absent_motor_ids() == [2]


def test_readers_are_scoped_to_the_motors_that_answered(partial_client):
    client, bus = partial_client
    assert client._pos_vel_cur_reader.active_motor_ids == [1, 3]
    bus.log.clear()
    pos, _vel, _cur = client.read_position_velocity_current()
    assert client.last_read_ok, "a silent motor must not spoil a fresh read of the others"
    assert len(pos) == 3, "data stays in config order, with a slot for the silent motor"


def test_writes_skip_the_silent_motor(partial_client):
    client, bus = partial_client
    bus.log.clear()
    assert client.write_byte([1, 2, 3], 1, 64) == []
    assert client.set_torque_enabled([1, 2, 3], True, retries=0) == []
    assert all("2" not in str(entry) for entry in bus.log if isinstance(entry, tuple))


def test_a_hardware_error_read_of_the_silent_motor_costs_no_bus_time(partial_client):
    client, bus = partial_client
    bus.log.clear()
    assert client.read_hardware_error(2) is None
    assert bus.log == []


def test_before_connect_every_motor_counts(monkeypatch):
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", make_fake_sdk(FakeBus()))
    client = DynamixelClient([4, 5], port="/dev/fake", baudrate=57600)
    assert client.responding_motor_ids() == [4, 5]
    assert client.absent_motor_ids() == []


def test_the_mock_client_can_declare_silent_motors():
    from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient

    client = MockDynamixelClient([1, 2, 3], port="/dev/mock", baudrate=1_000_000)
    client.silent_motor_ids = {3}
    assert client.responding_motor_ids() == [1, 2]
    assert client.absent_motor_ids() == [3]
    client.connect()
    try:
        assert client.read_hardware_error(3) is None
    finally:
        client.disconnect()
