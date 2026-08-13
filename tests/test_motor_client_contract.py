"""Cross-family contract tests for ``MotorClient`` implementations.

The mock must fail exactly where the real clients fail: any bus method
called before ``connect()`` raises, unknown motor IDs are logged and
reported (never a mock-only exception), and ``set_torque_enabled`` shares
one finite-retry signature across the whole family so a dead motor can
never wedge the hand. The ``OPEN_CLIENTS`` exit-cleanup registry follows
the same lifecycle everywhere: a client registers on successful
``connect()`` — never on construction or failed connect — and is
deregistered by ``disconnect()`` even when its torque-off raises.
"""

import inspect
import types

import numpy as np
import pytest

import orca_core.hardware.mock_dynamixel_client as mock_dynamixel_client_module
import orca_core.hardware.mock_feetech_client as mock_feetech_client_module
from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
from orca_core.hardware.mock_dynamixel_client import (
    MockDynamixelClient,
    dynamixel_cleanup_handler as mock_cleanup_handler,
)
from orca_core.hardware.mock_feetech_client import (
    MockFeetechClient,
    feetech_cleanup_handler as mock_feetech_cleanup_handler,
)
from orca_core.hardware.motor_client import MotorClient

# (mock class, its module, its atexit cleanup handler) for every family.
MOCK_FAMILIES = [
    pytest.param(
        (MockDynamixelClient, mock_dynamixel_client_module, mock_cleanup_handler),
        id="dynamixel",
    ),
    pytest.param(
        (MockFeetechClient, mock_feetech_client_module, mock_feetech_cleanup_handler),
        id="feetech",
    ),
]
MOCK_CLASSES = [pytest.param(p.values[0][0], id=p.id) for p in MOCK_FAMILIES]


@pytest.fixture(params=MOCK_CLASSES)
def disconnected_mock(request):
    client = request.param([1, 2], port="mock")
    yield client
    request.param.OPEN_CLIENTS.discard(client)


@pytest.fixture(params=MOCK_CLASSES)
def connected_mock(request):
    client = request.param([1, 2], port="mock")
    client.connect()
    yield client
    client.disconnect()
    request.param.OPEN_CLIENTS.discard(client)


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
    ],
)
def test_mock_bus_methods_raise_when_disconnected(disconnected_mock, call):
    with pytest.raises(OSError, match="Must call connect"):
        call(disconnected_mock)


@pytest.mark.parametrize(
    "call",
    [
        pytest.param(lambda c: c.sync_write([1], [0], 116, 4), id="sync_write"),
        pytest.param(lambda c: c.write_byte([1], 0, 64), id="write_byte"),
    ],
)
def test_dxl_register_level_mock_methods_raise_when_disconnected(call):
    """Register-level helpers exist on the Dynamixel mock only."""
    client = MockDynamixelClient([1, 2], port="mock")
    try:
        with pytest.raises(OSError, match="Must call connect"):
            call(client)
    finally:
        MockDynamixelClient.OPEN_CLIENTS.discard(client)


def test_mock_bus_methods_work_when_connected(connected_mock):
    assert connected_mock.set_torque_enabled([1, 2], True) == []
    assert all(connected_mock._torque_enabled.values())
    read = connected_mock.read_position_velocity_current()
    assert read.position.shape == (2,)


def test_mock_set_torque_enabled_returns_unknown_ids_as_failed(connected_mock):
    assert connected_mock.set_torque_enabled([1, 99], True) == [99]
    assert connected_mock._torque_enabled[1] is True


def test_mock_set_operating_mode_skips_unknown_ids(connected_mock):
    # Velocity mode survives both families' mode mapping unchanged.
    connected_mock.set_operating_mode([1, 99], 1)
    assert connected_mock._operating_mode[1] == 1
    assert 99 not in connected_mock._operating_mode


def test_mock_writes_skip_unknown_ids(connected_mock):
    connected_mock.write_desired_pos([1, 99], np.array([-0.5, -0.5]))
    assert connected_mock._pos[1] == pytest.approx(-0.5)
    assert 99 not in connected_mock._pos


# ----- OPEN_CLIENTS registry lifecycle --------------------------------------


class _RankOrderedSet(set):
    """Set whose iteration yields clients in ``_cleanup_rank`` order, so the
    cleanup-handler test deterministically hits the failing client first."""

    def __iter__(self):
        return iter(
            sorted(set.__iter__(self), key=lambda c: getattr(c, "_cleanup_rank", 1))
        )


@pytest.fixture(params=MOCK_FAMILIES)
def mock_family(request, monkeypatch):
    """A mock class with an isolated registry, plus its module and handler."""
    cls, module, cleanup_handler = request.param
    registry = _RankOrderedSet()
    monkeypatch.setattr(cls, "OPEN_CLIENTS", registry)
    return types.SimpleNamespace(
        cls=cls, module=module, cleanup=cleanup_handler, registry=registry
    )


def test_mock_construction_does_not_register(mock_family):
    mock_family.cls([1, 2], port="mock")
    assert mock_family.registry == set()


def test_mock_connect_registers_and_disconnect_deregisters(mock_family):
    client = mock_family.cls([1, 2], port="mock")
    client.connect()
    assert client in mock_family.registry
    client.disconnect()
    assert mock_family.registry == set()
    assert not client.is_connected


def test_mock_failed_connect_leaves_registry_empty(mock_family, monkeypatch):
    client = mock_family.cls([1, 2], port="mock")

    def boom(*args, **kwargs):
        raise OSError("simulated port failure")

    # Registration must be the last step of connect(): a failure anywhere
    # earlier (here, the first thing connect does) leaves the registry empty.
    monkeypatch.setattr(
        mock_family.module, "logging", types.SimpleNamespace(info=boom)
    )
    with pytest.raises(OSError):
        client.connect()
    assert not client.is_connected
    assert mock_family.registry == set()


def test_mock_disconnect_deregisters_even_when_torque_off_raises(
        mock_family, monkeypatch):
    client = mock_family.cls([1, 2], port="mock")
    client.connect()

    def boom(*args, **kwargs):
        raise OSError("serial link lost")

    monkeypatch.setattr(client, "set_torque_enabled", boom)
    with pytest.raises(OSError):
        client.disconnect()
    assert not client.is_connected
    assert mock_family.registry == set()


def test_mock_cleanup_handler_survives_a_failing_client(mock_family):
    bad = mock_family.cls([1], port="bad")
    good = mock_family.cls([1], port="good")
    bad.connect()
    good.connect()
    bad._cleanup_rank, good._cleanup_rank = 0, 1

    def boom():
        raise OSError("port died")

    bad.disconnect = boom
    try:
        mock_family.cleanup()
    finally:
        del bad.disconnect  # restore the real method for __del__

    assert not good.is_connected, "good client must still be disconnected"
    assert good not in mock_family.registry


def test_set_torque_enabled_signature_is_uniform_across_family():
    """One retry contract for the whole family: finite default, 0 = single
    attempt, and the failed IDs returned so callers can react."""
    base = inspect.signature(MotorClient.set_torque_enabled).parameters
    assert base["retries"].default == 3
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        params = inspect.signature(cls.set_torque_enabled).parameters
        assert params["retries"].default == 3, cls.__name__
        assert params["retry_interval"].default == 0.25, cls.__name__
