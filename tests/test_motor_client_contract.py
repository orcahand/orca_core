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
import logging
import types

import numpy as np
import pytest

import orca_core.hardware.mock_motor_client as mock_motor_client_module
from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient
from orca_core.hardware.mock_feetech_client import MockFeetechClient
from orca_core.hardware.motor_client import MotorClient, ServoGains

MOCK_CLASSES = [
    pytest.param(MockDynamixelClient, id="dynamixel"),
    pytest.param(MockFeetechClient, id="feetech"),
]


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
        pytest.param(lambda c: c.read_temperature(), id="read_temperature"),
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


@pytest.fixture(params=MOCK_CLASSES)
def mock_family(request, monkeypatch):
    """A mock class, with the shared exit-cleanup registry swapped for an empty one."""
    registry = set()
    monkeypatch.setattr(MotorClient, "OPEN_CLIENTS", registry)
    return types.SimpleNamespace(cls=request.param, registry=registry)


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
        mock_motor_client_module, "logging", types.SimpleNamespace(info=boom)
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


def test_wait_for_motion_complete_signature_is_uniform_across_family():
    for cls in (MotorClient, DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        params = inspect.signature(cls.wait_for_motion_complete).parameters
        assert list(params)[1:] == ["timeout", "poll_interval"], cls.__name__


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


# ----- goal-current contract -------------------------------------------------


REAL_AND_MOCK = [
    pytest.param((DynamixelClient, MockDynamixelClient), id="dynamixel"),
    pytest.param((FeetechClient, MockFeetechClient), id="feetech"),
]


@pytest.mark.parametrize("pair", REAL_AND_MOCK)
def test_every_family_declares_its_goal_current_register(pair):
    real, mock = pair
    for cls in (real, mock):
        assert cls.current_scale_ma > 0 and np.isfinite(cls.current_scale_ma)
        assert cls.max_current_ma > 0 and np.isfinite(cls.max_current_ma)
    assert mock.current_scale_ma == real.current_scale_ma
    assert mock.max_current_ma == real.max_current_ma


def test_abc_leaves_the_goal_current_attributes_to_the_family():
    assert not hasattr(MotorClient, "current_scale_ma")
    assert not hasattr(MotorClient, "max_current_ma")


def test_read_current_limits_reports_every_motor(connected_mock):
    limits = connected_mock.read_current_limits()
    assert set(limits) == {1, 2}
    assert all(limit == type(connected_mock).max_current_ma for limit in limits.values())


@pytest.mark.parametrize("value", [-1.0, float("nan")])
def test_mock_write_desired_current_rejects_bad_values(connected_mock, value):
    with pytest.raises(ValueError, match="non-negative finite"):
        connected_mock.write_desired_current([1], np.array([value]))


# ----- family current defaults ------------------------------------------------

@pytest.mark.parametrize("real, mock, expected", [
    pytest.param(DynamixelClient, MockDynamixelClient, (300, 300), id="dynamixel"),
    pytest.param(FeetechClient, MockFeetechClient, (900, 900), id="feetech"),
])
def test_every_family_declares_its_current_defaults(real, mock, expected):
    for cls in (real, mock):
        assert (cls.default_max_current_ma, cls.default_calibration_current_ma) == expected
        assert isinstance(cls.default_max_current_ma, int)
        assert isinstance(cls.default_calibration_current_ma, int)
    assert (mock.default_max_current_ma, mock.default_calibration_current_ma) == (
        real.default_max_current_ma, real.default_calibration_current_ma)


def test_read_hardware_errors_is_uniform_across_family():
    """Every client answers for a whole chain in one call.

    The bus is half-duplex, so a caller sweeping latched errors must not be
    forced into a round trip per motor. Families that cannot batch inherit the
    ABC's per-motor fallback, but the entry point is the same everywhere.
    """
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        assert hasattr(cls, "read_hardware_errors"), cls.__name__

    # A family with no batch primitive still answers, via the ABC default.
    per_motor = types.SimpleNamespace(
        read_hardware_error=lambda motor_id: 0x20 if motor_id == 2 else 0,
    )
    fallback = MotorClient.read_hardware_errors(per_motor, [1, 2])
    assert fallback == {1: 0, 2: 0x20}
    assert MotorClient.read_hardware_errors(per_motor, []) == {}


def test_take_hardware_alerts_is_uniform_across_family():
    """Noticing a latched fault is free; acting on it is the caller's call.

    Every client offers the same drain, so a front-end never has to know which
    family it is talking to. Families that cannot see the byte report nothing
    rather than making the caller pay for a second read.
    """
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        assert hasattr(cls, "take_hardware_alerts"), cls.__name__

    blind = types.SimpleNamespace()
    assert MotorClient.take_hardware_alerts(blind) == {}


def test_servo_gains_are_uniform_across_family():
    """Every client answers the same way, so a front-end never branches on
    motor family. Families that cannot reach the registers report None
    rather than raising."""
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        assert hasattr(cls, "read_servo_gains"), cls.__name__
        assert hasattr(cls, "write_servo_gains"), cls.__name__

    blind = types.SimpleNamespace()
    assert MotorClient.read_servo_gains(blind, [1, 2]) == {1: None, 2: None}
    assert MotorClient.write_servo_gains(blind, {1: ServoGains(kp=1)}) is None


def test_a_family_without_servo_registers_says_so_instead_of_dropping_the_write(caplog):
    blind = types.SimpleNamespace()
    with caplog.at_level(logging.WARNING, logger="orca_core.hardware.motor_client"):
        MotorClient.write_servo_gains(blind, {1: ServoGains(kp=1)})
        MotorClient.write_servo_profile(blind, {})
    assert caplog.text.count("was ignored") == 1


def test_partial_gain_writes_leave_the_other_fields_alone(connected_mock):
    """A caller nudging one gain must not silently zero the rest."""
    if not isinstance(connected_mock, MockDynamixelClient):
        pytest.skip("gain registers are Dynamixel-only for now")
    before = connected_mock.read_servo_gains([1])[1]
    connected_mock.write_servo_gains({1: ServoGains(kp=1234)})
    after = connected_mock.read_servo_gains([1])[1]
    assert after.kp == 1234
    assert (after.ki, after.kd, after.ff_1st, after.ff_2nd) == (
        before.ki, before.kd, before.ff_1st, before.ff_2nd)


def test_only_dynamixel_declares_a_return_delay_to_program():
    from orca_core.constants import DYNAMIXEL_RETURN_DELAY_TIME_US

    assert DynamixelClient.return_delay_time_us == DYNAMIXEL_RETURN_DELAY_TIME_US
    assert FeetechClient.return_delay_time_us is None


@pytest.mark.parametrize("delay_us, register", [(20, 10), (0, 0), (508, 254)])
def test_dynamixel_return_delay_is_written_in_two_us_units(delay_us, register):
    from orca_core.hardware.dynamixel_client import ADDR_RETURN_DELAY_TIME

    writes = []
    stub = types.SimpleNamespace(
        set_torque_enabled=lambda ids, enabled: None,
        write_byte=lambda ids, value, address: writes.append((ids, value, address)) or [],
    )

    assert DynamixelClient.change_return_delay_time(stub, 5, delay_us) is True
    assert writes == [([5], register, ADDR_RETURN_DELAY_TIME)]


@pytest.mark.parametrize("delay_us", [-2, 7, 510])
def test_dynamixel_rejects_an_unencodable_return_delay(delay_us):
    stub = types.SimpleNamespace(write_byte=lambda *a: pytest.fail("wrote an invalid delay"))
    assert DynamixelClient.change_return_delay_time(stub, 5, delay_us) is False


def test_reboot_is_declared_uniformly_across_family():
    """Clearing a latched fault in band is the same call on every client.

    A caller offering recovery has to know whether the family can do it at
    all before promising the operator anything, so the capability is declared
    rather than discovered by catching an exception.
    """
    for cls in (DynamixelClient, FeetechClient, MockDynamixelClient,
                MockFeetechClient):
        assert cls.supports_reboot is True, cls.__name__
        assert callable(getattr(cls, "reboot_motor", None)), cls.__name__

    # A family without the instruction says so instead of silently no-opping:
    # a caller that believed a fault was cleared would drive a dead motor.
    assert MotorClient.supports_reboot is False
    with pytest.raises(NotImplementedError, match="power-cycled"):
        MotorClient.reboot_motor(types.SimpleNamespace(), 1)


@pytest.mark.parametrize("make", [
    lambda: MockDynamixelClient([1, 2], port="mock", baudrate=1000000),
    lambda: MockFeetechClient([1, 2], port="mock", baudrate=1000000),
])
def test_a_reboot_clears_that_motors_latch_and_leaves_the_others(make):
    client = make()
    client.connect()
    try:
        client.latched_errors = {1: 0x20, 2: 0x04}

        client.reboot_motor(1)

        assert client.read_hardware_error(1) == 0
        assert client.read_hardware_error(2) == 0x04, "rebooted the wrong motor"
        assert client.reboot_count[1] == 1
    finally:
        client.disconnect()


def test_feetech_records_the_status_byte_it_already_receives():
    """Every reply on that family carries the byte, so noticing a fault is
    free - but logging it is not reporting it. A fault raised while the hand
    is being driven must not be lost because nothing happened to be polling
    the error register.
    """
    # The real client, unconnected: recording is bookkeeping on a byte the
    # caller already has, so it needs no bus.
    client = FeetechClient([1, 2], port="/dev/null", baudrate=1000000)
    # Only the log line needs the handler; the recording is pure bookkeeping.
    client.packet_handler = types.SimpleNamespace(
        getRxPacketError=lambda error: f"status 0x{error:02X}")
    assert client.take_hardware_alerts() == {}

    client._log_status_error(1, 0x04, "sync read")
    client._log_status_error(1, 0x04, "sync read")  # log throttled, still recorded
    client._log_status_error(2, 0x20, "sync read")

    assert client.take_hardware_alerts() == {1: 0x04, 2: 0x20}
    assert client.take_hardware_alerts() == {}, "draining clears"


def test_a_short_ping_reply_is_not_a_crash():
    """A motor part way through a reboot answers the ping and then has nothing
    to say about its model number. Indexing that blindly turned every
    liveness check into an IndexError, which aborted a reboot before its RAM
    was restored and left the motor at its protection current."""
    from orca_core.hardware.feetech.protocol_packet_handler import (
        protocol_packet_handler,
    )

    handler = protocol_packet_handler(portHandler=None, protocol_end=0)
    handler.txRxPacket = lambda packet: ([], 0, 0)

    for short in ([], [7], None):
        handler.readTxRx = lambda scs_id, address, length: (short, 0, 0)
        model, result, error = handler.ping(1)
        assert model == 0, short
        assert result == 0, "the ping itself answered; only the model is unknown"

    # A full reply still reports the model.
    handler.readTxRx = lambda scs_id, address, length: ([0x34, 0x12], 0, 0)
    model, result, _ = handler.ping(1)
    assert model == 0x1234


# Methods the fault policy calls on a live client. Checked against the real
# classes, not the mocks: a mock cannot catch a real client calling a helper
# that no longer exists, which is how a renamed connection check shipped a
# policy that died on its first poll and recovered nothing.
_FAULT_POLICY_METHODS = (
    "read_stop_mask",
    "read_temperature_limits",
    "hold_motor_off",
    "release_motor",
    "reboot_motor",
    "read_hardware_error",
    "take_hardware_alerts",
)


@pytest.mark.parametrize("cls", [DynamixelClient, FeetechClient])
def test_the_fault_policy_methods_only_call_helpers_that_exist(cls):
    """Every ``self.x`` these methods reach for must be on the class.

    This is the one bug a mock cannot find: the method is never executed
    against a real client in the suite, so a helper renamed underneath it
    only shows up as an AttributeError on hardware.
    """
    import inspect
    import re

    # An unconnected instance, so attributes set in __init__ count as real.
    # Nothing here is called; only the names are checked.
    client = cls([1, 2], port="/dev/null", baudrate=1000000)

    missing = []
    for name in _FAULT_POLICY_METHODS:
        method = getattr(cls, name, None)
        if method is None or not callable(method):
            continue
        try:
            source = inspect.getsource(method)
        except (OSError, TypeError):
            continue
        for attribute in sorted(set(re.findall(r"self\.([A-Za-z_][A-Za-z0-9_]*)",
                                               source))):
            if not hasattr(client, attribute):
                missing.append(f"{cls.__name__}.{name} -> self.{attribute}")
    assert not missing, "calls that do not exist: " + ", ".join(missing)
