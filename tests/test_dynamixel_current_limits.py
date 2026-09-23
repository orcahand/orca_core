"""Dynamixel goal-current writes: mA in, register units out, and a motor
whose model has no current register is skipped rather than written to."""

from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from orca_core.hardware.dynamixel_client import (
    ADDR_GOAL_CURRENT,
    DynamixelClient,
)

XC330 = 1220
XC430 = 1080


def _make_sdk(models: dict[int, int], writes: list):
    class FakeSerial:
        def reset_input_buffer(self):
            pass

    class PortHandler:
        def __init__(self, port):
            self.is_open = False
            self.is_using = False
            self.ser = FakeSerial()

        def openPort(self):
            self.is_open = True
            return True

        def setBaudRate(self, baudrate):
            return True

        def closePort(self):
            self.is_open = False

    class PacketHandler:
        def __init__(self, protocol_version):
            pass

        def ping(self, port, motor_id):
            if motor_id in models:
                return models[motor_id], 0, 0
            return 0, -3001, 0

        def read1ByteTxRx(self, port, motor_id, address):
            return 0, 0, 0

        def write1ByteTxRx(self, port, motor_id, address, value):
            return 0, 0

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
            self.address = address
            self.params = {}

        def addParam(self, motor_id, value):
            self.params[motor_id] = int.from_bytes(value, "little")
            return True

        def txPacket(self):
            writes.append((self.address, dict(self.params)))
            return 0

        def clearParam(self):
            self.params = {}

    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.COMM_RX_FAIL = -3001
    sdk.COMM_NOT_AVAILABLE = -3002
    sdk.PortHandler = PortHandler
    sdk.PacketHandler = PacketHandler
    sdk.GroupBulkRead = GroupBulkRead
    sdk.GroupSyncWrite = GroupSyncWrite
    return sdk


@pytest.fixture
def hand_bus(monkeypatch):
    """Two finger motors and a wrist motor with no current register."""
    writes: list = []
    models = {1: XC330, 2: XC330, 3: XC430}
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", _make_sdk(models, writes))
    client = DynamixelClient([1, 2, 3], port="/dev/fake", baudrate=57600)
    client.connect()
    yield client, writes
    client.port_handler.is_open = False
    DynamixelClient.OPEN_CLIENTS.discard(client)


def _goal_current_writes(writes):
    return [params for address, params in writes if address == ADDR_GOAL_CURRENT]


def test_connect_learns_each_motors_model(hand_bus):
    client, _ = hand_bus
    assert client._model_numbers == {1: XC330, 2: XC330, 3: XC430}


def test_current_limits_are_none_where_the_model_has_no_register(hand_bus):
    client, _ = hand_bus
    assert client.read_current_limits() == {1: 910.0, 2: 910.0, 3: None}


def test_write_desired_current_writes_milliamps_and_skips_the_wrist(hand_bus):
    client, writes = hand_bus
    client.write_desired_current([1, 2, 3], np.array([300.0, 450.0, 300.0]))
    assert _goal_current_writes(writes) == [{1: 300, 2: 450}]


def test_write_desired_current_clamps_to_the_register_range(hand_bus, caplog):
    client, writes = hand_bus
    client.write_desired_current([1], np.array([5000.0]))
    assert _goal_current_writes(writes) == [{1: 910}]
    assert "clamped" in caplog.text


def test_write_desired_current_validates_before_touching_the_bus(hand_bus):
    client, writes = hand_bus
    with pytest.raises(ValueError, match="non-negative finite"):
        client.write_desired_current([1, 2], np.array([300.0, float("nan")]))
    assert _goal_current_writes(writes) == []


def test_unknown_model_is_treated_as_limitable(monkeypatch):
    writes: list = []
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", _make_sdk({1: 9999}, writes))
    client = DynamixelClient([1], port="/dev/fake", baudrate=57600)
    client.connect()
    try:
        assert client.read_current_limits() == {1: 910.0}
    finally:
        client.port_handler.is_open = False
        DynamixelClient.OPEN_CLIENTS.discard(client)
