"""OPEN_CLIENTS registry lifecycle for the real motor clients.

A client must appear in its family's exit-cleanup registry only while its
port is actually open: registered on successful connect(), never on
construction or failed connect, and removed on disconnect(). The atexit
cleanup handlers must survive one client failing so the remaining clients
still get their torque disabled.

The same lifecycle owns the in-process port registry: connect() claims the
port and disconnect() releases it, including when a failed transaction left
the vendor SDK's in-flight flag latched.

Also pins the motor-family factory: every family resolves to a real and a
mock client, and an unknown family is a ValueError rather than a silent
Dynamixel fallback.
"""

import sys
import types

import numpy as np
import pytest

import orca_core.hardware.feetech_client as feetech_client_module
from orca_core.hardware import port_registry
from orca_core.hardware.dynamixel_client import (
    DynamixelClient,
    dynamixel_cleanup_handler,
)
from orca_core.hardware.feetech_client import (
    COMM_SUCCESS,
    FeetechClient,
    feetech_cleanup_handler,
)
from orca_core.hardware.motor_factory import (
    create_mock_motor_client,
    mock_motor_client_class,
    motor_client_class,
)

# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


class _BadFirstSet(set):
    """Set whose iteration yields clients in ``_cleanup_rank`` order, so the
    cleanup-handler tests deterministically hit the failing client first."""

    def __iter__(self):
        return iter(
            sorted(set.__iter__(self), key=lambda c: getattr(c, "_cleanup_rank", 1))
        )


def _make_fake_dxl_sdk():
    class FakeSerial:
        def reset_input_buffer(self):
            pass

    class PortHandler:
        open_result = True
        baud_result = True

        def __init__(self, port):
            self.port_name = port
            self.is_open = False
            self.is_using = False
            self.ser = FakeSerial()

        def openPort(self):
            self.is_open = self.open_result
            return self.open_result

        def setBaudRate(self, baudrate):
            self.baudrate = baudrate
            return self.baud_result

        def closePort(self):
            self.is_open = False

    class PacketHandler:
        def __init__(self, protocol_version):
            pass

        def write1ByteTxRx(self, port, motor_id, address, value):
            return 0, 0

        def read1ByteTxRx(self, port, motor_id, address):
            return 0, 0, 0

        def getTxRxResult(self, comm_result):
            return str(comm_result)

        def getRxPacketError(self, dxl_error):
            return str(dxl_error)

    class GroupBulkRead:
        def __init__(self, port, packet_handler):
            self.data_dict = {}

        def addParam(self, motor_id, address, size):
            self.data_dict[motor_id] = [None, address, size]
            return True

    class GroupSyncWrite:
        def __init__(self, port, packet_handler, address, size):
            pass

        def addParam(self, motor_id, value):
            return True

        def txPacket(self):
            return 0

        def clearParam(self):
            pass

    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.COMM_RX_FAIL = -3001
    sdk.COMM_NOT_AVAILABLE = -3002
    sdk.PortHandler = PortHandler
    sdk.PacketHandler = PacketHandler
    sdk.GroupBulkRead = GroupBulkRead
    sdk.GroupSyncWrite = GroupSyncWrite
    return sdk


class FakeFeetechPortHandler:
    open_result = True

    def __init__(self, port):
        self.port_name = port
        self.is_open = False
        self.is_using = False
        self.ser = object()

    def openPort(self):
        self.is_open = self.open_result
        return self.open_result

    def closePort(self):
        self.is_open = False


class FakeSmsSts:
    def __init__(self, port_handler):
        pass

    def write1ByteTxRx(self, motor_id, address, value):
        return COMM_SUCCESS, 0


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def dxl_registry(monkeypatch):
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", _make_fake_dxl_sdk())
    registry = _BadFirstSet()
    monkeypatch.setattr(DynamixelClient, "OPEN_CLIENTS", registry)
    return registry


@pytest.fixture
def feetech_registry(monkeypatch):
    monkeypatch.setattr(feetech_client_module, "PortHandler", FakeFeetechPortHandler)
    monkeypatch.setattr(feetech_client_module, "sms_sts", FakeSmsSts)
    registry = _BadFirstSet()
    monkeypatch.setattr(FeetechClient, "OPEN_CLIENTS", registry)
    return registry


def _make_dxl(port="/dev/fake"):
    return DynamixelClient([1, 2], port=port, baudrate=57600)


def _make_feetech(port="/dev/fake"):
    return FeetechClient([1, 2], port=port)


# ---------------------------------------------------------------------------
# DynamixelClient
# ---------------------------------------------------------------------------


def test_dxl_construction_does_not_register(dxl_registry):
    _make_dxl()
    assert dxl_registry == set()


def test_dxl_connect_registers_and_disconnect_deregisters(dxl_registry):
    client = _make_dxl()
    client.connect()
    assert client in dxl_registry
    client.disconnect()
    assert dxl_registry == set()
    assert not client.is_connected


def test_dxl_failed_connect_leaves_registry_empty(dxl_registry):
    client = _make_dxl()
    client.port_handler.open_result = False
    with pytest.raises(OSError):
        client.connect()
    assert dxl_registry == set()


def test_dxl_failure_after_open_closes_port_before_raising(dxl_registry):
    client = _make_dxl()
    client.port_handler.baud_result = False
    with pytest.raises(OSError):
        client.connect()
    assert not client.port_handler.is_open, "mid-connect failure left the port open"
    assert not client.is_connected
    assert dxl_registry == set()


def test_dxl_repeated_failed_connects_do_not_accumulate(dxl_registry):
    for _ in range(5):
        client = _make_dxl()
        client.port_handler.open_result = False
        with pytest.raises(OSError):
            client.connect()
    assert dxl_registry == set()


def test_dxl_cleanup_handler_survives_a_failing_client(dxl_registry):
    bad, good = _make_dxl("/dev/bad"), _make_dxl("/dev/good")
    bad.connect()
    good.connect()
    bad._cleanup_rank, good._cleanup_rank = 0, 1

    def boom():
        raise OSError("port died")

    bad.disconnect = boom
    try:
        dynamixel_cleanup_handler()
    finally:
        del bad.disconnect  # restore the real method for __del__

    assert not good.port_handler.is_open, "good client must still be closed"
    assert good not in dxl_registry


# ---------------------------------------------------------------------------
# FeetechClient
# ---------------------------------------------------------------------------


def test_feetech_construction_does_not_register(feetech_registry):
    _make_feetech()
    assert feetech_registry == set()


def test_feetech_connect_registers_and_disconnect_deregisters(feetech_registry):
    client = _make_feetech()
    client.connect()
    assert client in feetech_registry
    client.disconnect()
    assert feetech_registry == set()
    assert not client.is_connected


def test_feetech_failed_connect_leaves_registry_empty(feetech_registry):
    client = _make_feetech()
    client.port_handler.open_result = False
    with pytest.raises(OSError):
        client.connect()
    assert feetech_registry == set()


def test_feetech_failure_after_open_closes_port_before_raising(
        feetech_registry, monkeypatch):
    class RaisingSmsSts:
        def __init__(self, port_handler):
            raise OSError("packet handler init failed")

    monkeypatch.setattr(feetech_client_module, "sms_sts", RaisingSmsSts)
    client = _make_feetech()
    with pytest.raises(OSError):
        client.connect()
    assert not client.port_handler.is_open, "mid-connect failure left the port open"
    assert not client.is_connected
    assert feetech_registry == set()


def test_feetech_scan_for_motors_is_an_instance_method(feetech_registry):
    """maintenance.motor_chain.scan_motors calls it on a constructed client."""
    client = _make_feetech()
    assert client.scan_for_motors(port="/dev/fake", id_range=(1, 1),
                                  baud_rates=[]) == []


def test_feetech_cleanup_handler_survives_a_failing_client(feetech_registry):
    bad, good = _make_feetech("/dev/bad"), _make_feetech("/dev/good")
    bad.connect()
    good.connect()
    bad._cleanup_rank, good._cleanup_rank = 0, 1

    def boom():
        raise OSError("port died")

    bad.disconnect = boom
    try:
        feetech_cleanup_handler()
    finally:
        del bad.disconnect  # restore the real method for __del__

    assert not good.port_handler.is_open, "good client must still be closed"
    assert good not in feetech_registry


# ---------------------------------------------------------------------------
# In-process port ownership
# ---------------------------------------------------------------------------


def test_dxl_connect_claims_the_port_and_disconnect_releases_it(dxl_registry):
    """Without the claim, a second client opens the same bus and the two read
    each other's replies."""
    first = _make_dxl()
    first.connect()
    assert port_registry.owner_of("/dev/fake") is first

    second = _make_dxl()
    with pytest.raises(port_registry.PortAlreadyClaimed):
        second.connect()

    first.disconnect()
    assert port_registry.owner_of("/dev/fake") is None
    second.connect()
    second.disconnect()


def test_feetech_connect_claims_the_port_and_disconnect_releases_it(feetech_registry):
    first = _make_feetech()
    first.connect()
    assert port_registry.owner_of("/dev/fake") is first

    second = _make_feetech()
    with pytest.raises(port_registry.PortAlreadyClaimed):
        second.connect()

    first.disconnect()
    assert port_registry.owner_of("/dev/fake") is None
    second.connect()
    second.disconnect()


def test_dxl_disconnect_releases_the_port_after_a_latched_transaction(dxl_registry):
    """The vendor SDK leaves ``is_using`` set when a transaction raises — a
    yanked cable mid-read is enough. Bailing out of disconnect there strands
    the claim for the life of the process, so no later client can connect."""
    client = _make_dxl()
    client.connect()
    client.port_handler.is_using = True

    client.disconnect()

    assert port_registry.owner_of("/dev/fake") is None
    assert dxl_registry == set()
    assert not client.port_handler.is_open

    replacement = _make_dxl()
    replacement.connect()
    replacement.disconnect()


def test_feetech_disconnect_releases_the_port_after_a_latched_transaction(
        feetech_registry):
    client = _make_feetech()
    client.connect()
    client.port_handler.is_using = True

    client.disconnect()

    assert port_registry.owner_of("/dev/fake") is None
    assert feetech_registry == set()
    assert not client.port_handler.is_open

    replacement = _make_feetech()
    replacement.connect()
    replacement.disconnect()


# ---------------------------------------------------------------------------
# Motor-family factory
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "motor_type, real_name, mock_name",
    [
        ("dynamixel", "DynamixelClient", "MockDynamixelClient"),
        ("feetech", "FeetechClient", "MockFeetechClient"),
    ],
)
def test_factory_resolves_every_family(motor_type, real_name, mock_name):
    assert motor_client_class(motor_type).__name__ == real_name
    assert mock_motor_client_class(motor_type).__name__ == mock_name


@pytest.mark.parametrize(
    "factory", [motor_client_class, mock_motor_client_class]
)
def test_factory_rejects_an_unknown_family(factory):
    with pytest.raises(ValueError, match="Unknown motor_type"):
        factory("stepper")


def test_create_mock_motor_client_builds_without_connecting():
    client = create_mock_motor_client("feetech", [1, 2], "mock", 1_000_000)
    try:
        assert not client.is_connected
        assert client.motor_ids == [1, 2]
        assert client.port_name == "mock"
        assert client.baudrate == 1_000_000
    finally:
        type(client).OPEN_CLIENTS.discard(client)


def test_mock_clients_carry_their_family_capabilities():
    """A mock hand must behave like the family it stands in for."""
    for motor_type in ("dynamixel", "feetech"):
        real = motor_client_class(motor_type)
        mock = mock_motor_client_class(motor_type)
        for attribute in (
            "motor_type",
            "waits_for_motion",
            "supports_multi_turn",
            "supported_modes",
            "position_range_rad",
            "factory_default_id",
            "factory_default_baudrate",
            "baud_rate_map",
            "requires_unpowered_hotplug",
        ):
            assert getattr(mock, attribute) == getattr(real, attribute), (
                f"{mock.__name__}.{attribute}"
            )


def test_mock_feetech_clamps_commands_into_its_position_range():
    client = create_mock_motor_client("feetech", [1], "mock", 1_000_000)
    with client:
        low, high = client.position_range_rad
        client.write_desired_pos([1], np.array([high + 1.0]))
        assert client._pos[1] == pytest.approx(high)
        client.write_desired_pos([1], np.array([low - 1.0]))
        assert client._pos[1] == pytest.approx(low)
