"""Unit tests for ``FeetechClient`` bus behavior.

Covers read caching and ``last_read_ok`` (a motor missing from a partial
read must keep its cached value and flag the read as not ok), stale-RX
flushing after failed transactions, finite torque/mode retries, the bus
lock held across every port transaction, and the ``calibrate_offset``
success/failure contract.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace

import numpy as np
import pytest

import orca_core.hardware.feetech_client as feetech_client_module
from orca_core.hardware.feetech_client import (
    COMM_SUCCESS,
    FeetechClient,
)
from orca_core.hardware.feetech import (
    SMS_STS_MODE,
    SMS_STS_MOVING,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
    SMS_STS_TORQUE_ENABLE,
)


def _lock_held_by_another_thread(lock) -> bool:
    """True when ``lock`` is currently held by a thread other than the probe."""
    result = {}

    def probe():
        acquired = lock.acquire(blocking=False)
        if acquired:
            lock.release()
        result["held"] = not acquired

    t = threading.Thread(target=probe)
    t.start()
    t.join()
    return result["held"]


class FakeSerial:
    """Records RX flushes into the shared op log."""

    def __init__(self, log: list):
        self.log = log

    def reset_input_buffer(self):
        self.log.append("flush")


class FakePacketHandler:
    """Serves raw register values per motor; ``unavailable_ids`` simulate
    motors that drop their reply. Records every port operation in ``log``
    and, when ``client`` is set, whether the bus lock was held for it."""

    def __init__(self, positions: dict[int, int]):
        self.positions = dict(positions)
        self.unavailable_ids: set[int] = set()
        self.sync_fails = False
        self.log: list[str] = []
        self.write1_hook = None  # callable(motor_id, address, value) -> (result, error)
        self.ofs_hook = None     # callable(motor_id, position) -> (result, error)
        self.client: FeetechClient | None = None
        self.lock_checks: list[bool] = []

    def _record(self, name):
        self.log.append(name)
        if self.client is not None:
            self.lock_checks.append(
                _lock_held_by_another_thread(self.client._bus_lock)
            )

    def scs_tohost(self, raw, bit):
        return raw

    def read2ByteTxRx(self, motor_id, address):
        self._record("read2")
        if motor_id in self.unavailable_ids:
            return 0, 1, 0
        if address == SMS_STS_PRESENT_POSITION_L:
            return self.positions[motor_id], COMM_SUCCESS, 0
        return 0, COMM_SUCCESS, 0

    def write1ByteTxRx(self, motor_id, address, value):
        self._record("write1")
        if self.write1_hook is not None:
            return self.write1_hook(motor_id, address, value)
        return COMM_SUCCESS, 0

    def reOfsCal(self, motor_id, position):
        self._record("ofs_cal")
        if self.ofs_hook is not None:
            return self.ofs_hook(motor_id, position)
        return COMM_SUCCESS, 0


class FakeGroupSyncRead:
    def __init__(self, packet_handler, address, size):
        self.handler = packet_handler

    def addParam(self, motor_id):
        return True

    def txRxPacket(self):
        self.handler._record("sync_tx")
        return 1 if self.handler.sync_fails else COMM_SUCCESS

    def isAvailable(self, motor_id, address, size):
        return motor_id not in self.handler.unavailable_ids, 0

    def getData(self, motor_id, address, size):
        if address == SMS_STS_PRESENT_POSITION_L:
            return self.handler.positions[motor_id]
        assert address in (
            SMS_STS_PRESENT_SPEED_L, SMS_STS_PRESENT_CURRENT_L, SMS_STS_MOVING)
        return 0


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(feetech_client_module, "GroupSyncRead", FakeGroupSyncRead)
    feetech = FeetechClient(motor_ids=[1, 2, 3], port="/dev/null")
    handler = FakePacketHandler({1: 100, 2: 200, 3: 300})
    feetech.packet_handler = handler
    feetech._connected = True
    yield feetech, handler
    feetech._connected = False
    FeetechClient.OPEN_CLIENTS.discard(feetech)


def _patch_sleep(monkeypatch):
    sleeps = []
    monkeypatch.setattr(feetech_client_module.time, "sleep", lambda s: sleeps.append(s))
    return sleeps


def _expected_pos(client: FeetechClient, raw: int) -> float:
    return FeetechClient._raw_to_rad(raw, client.pos_scale)


def test_full_sync_read_reports_ok(client):
    feetech, _ = client
    read = feetech.read_position_velocity_current()
    assert feetech.last_read_ok is True
    np.testing.assert_allclose(
        read.position,
        [_expected_pos(feetech, raw) for raw in (100, 200, 300)],
        rtol=1e-6,
    )


def test_partial_sync_read_keeps_cache_and_flags_not_ok(client):
    feetech, handler = client
    feetech.read_position_velocity_current()

    handler.positions = {1: 110, 2: 999, 3: 310}
    handler.unavailable_ids = {2}
    read = feetech.read_position_velocity_current()

    assert feetech.last_read_ok is False
    assert read.position[0] == pytest.approx(_expected_pos(feetech, 110))
    # The dropped motor keeps its previous value instead of reading 0.0.
    assert read.position[1] == pytest.approx(_expected_pos(feetech, 200))
    assert read.position[2] == pytest.approx(_expected_pos(feetech, 310))

    handler.unavailable_ids = set()
    read = feetech.read_position_velocity_current()
    assert feetech.last_read_ok is True
    assert read.position[1] == pytest.approx(_expected_pos(feetech, 999))


def test_per_motor_fallback_keeps_cache_and_flags_not_ok(client):
    feetech, handler = client
    feetech.read_position_velocity_current()

    handler.positions = {1: 110, 2: 999, 3: 310}
    handler.sync_fails = True
    handler.unavailable_ids = {2}
    read = feetech.read_position_velocity_current()

    assert feetech.last_read_ok is False
    assert read.position[0] == pytest.approx(_expected_pos(feetech, 110))
    assert read.position[1] == pytest.approx(_expected_pos(feetech, 200))
    assert read.position[2] == pytest.approx(_expected_pos(feetech, 310))


def test_fallback_with_all_motors_answering_reports_ok(client):
    feetech, handler = client
    handler.sync_fails = True
    read = feetech.read_position_velocity_current()
    assert feetech.last_read_ok is True
    np.testing.assert_allclose(
        read.position,
        [_expected_pos(feetech, raw) for raw in (100, 200, 300)],
        rtol=1e-6,
    )


# ----- stale-RX flushing ----------------------------------------------------

def test_failed_sync_read_flushes_rx_before_per_motor_fallback(client):
    feetech, handler = client
    feetech.port_handler.ser = FakeSerial(handler.log)
    handler.sync_fails = True
    feetech.read_position_velocity_current()
    assert handler.log[:3] == ["sync_tx", "flush", "read2"]


def test_failed_per_motor_reads_flush_rx_each(client):
    feetech, handler = client
    feetech.port_handler.ser = FakeSerial(handler.log)
    handler.sync_fails = True
    handler.unavailable_ids = {2}
    feetech.read_position_velocity_current()
    # One flush after the failed sync read, one per failed pos/vel/cur read.
    assert handler.log.count("flush") == 4


def test_partial_sync_read_flushes_rx(client):
    feetech, handler = client
    feetech.port_handler.ser = FakeSerial(handler.log)
    handler.unavailable_ids = {2}
    feetech.read_position_velocity_current()
    assert handler.log.count("flush") == 1


def test_partial_moving_status_read_flushes_rx(client, monkeypatch):
    feetech, handler = client
    feetech.port_handler.ser = FakeSerial(handler.log)
    _patch_sleep(monkeypatch)
    # One poll (0.1 < deadline 0.5), whose partial read must flush, then time out.
    ticks = iter([0.0, 0.1, 1.0])
    monkeypatch.setattr(feetech_client_module.time, "monotonic",
                        lambda: next(ticks))
    handler.unavailable_ids = {2}
    with pytest.raises(feetech_client_module.MotionTimeoutError):
        feetech.wait_for_motion_complete(timeout=0.5)
    assert handler.log == ["sync_tx", "flush"]


def test_failed_torque_write_flushes_rx_between_retries(client, monkeypatch):
    feetech, handler = client
    feetech.port_handler.ser = FakeSerial(handler.log)
    _patch_sleep(monkeypatch)
    handler.write1_hook = lambda motor_id, address, value: (1, 0)
    feetech.set_torque_enabled([1], True, retries=1)
    assert handler.log == ["write1", "flush", "write1", "flush"]


# ----- finite retries -------------------------------------------------------

def test_set_torque_enabled_success_returns_empty_without_sleeping(
        client, monkeypatch):
    feetech, handler = client
    sleeps = _patch_sleep(monkeypatch)
    result = feetech.set_torque_enabled([1, 2, 3], True)
    assert result == []
    assert sleeps == []
    assert handler.log.count("write1") == 3


def test_set_torque_enabled_default_retries_are_finite(client, monkeypatch):
    feetech, handler = client
    sleeps = _patch_sleep(monkeypatch)
    handler.write1_hook = lambda motor_id, address, value: (1, 0)
    result = feetech.set_torque_enabled([1], True)
    assert result == [1]
    assert handler.log.count("write1") == 4  # first attempt + 3 retries
    assert sleeps == [0.25, 0.25, 0.25]


def test_set_torque_enabled_retries_zero_is_single_attempt(client, monkeypatch):
    feetech, handler = client
    sleeps = _patch_sleep(monkeypatch)
    handler.write1_hook = lambda motor_id, address, value: (1, 0)
    result = feetech.set_torque_enabled([1, 2, 3], True, retries=0)
    assert result == [1, 2, 3]
    assert sleeps == []
    assert handler.log.count("write1") == 3


def test_set_torque_enabled_returns_only_still_failed_ids(client, monkeypatch):
    feetech, handler = client
    sleeps = _patch_sleep(monkeypatch)
    handler.write1_hook = lambda motor_id, address, value: (
        (1, 0) if motor_id == 2 else (COMM_SUCCESS, 0))
    result = feetech.set_torque_enabled([1, 2, 3], True, retries=2,
                                        retry_interval=0.1)
    assert result == [2]
    assert sleeps == [0.1, 0.1]


def test_set_operating_mode_is_bounded_on_dead_motor(client, monkeypatch):
    feetech, handler = client
    _patch_sleep(monkeypatch)
    handler.write1_hook = lambda motor_id, address, value: (1, 0)
    feetech.set_operating_mode([1], 5)  # must return, never spin forever
    # Torque-off (1+3 retries) only: the unacked motor gets no mode write
    # and no torque re-enable.
    assert handler.log.count("write1") == 4


def test_set_operating_mode_skips_motors_that_did_not_ack_torque_off(
        client, monkeypatch):
    feetech, handler = client
    _patch_sleep(monkeypatch)
    calls = []

    def hook(motor_id, address, value):
        calls.append((motor_id, address, value))
        if motor_id == 2 and address == SMS_STS_TORQUE_ENABLE:
            return 1, 0
        return COMM_SUCCESS, 0

    handler.write1_hook = hook
    feetech.set_operating_mode([1, 2, 3], 5)

    mode_writes = [mid for mid, addr, _ in calls if addr == SMS_STS_MODE]
    assert mode_writes == [1, 3], "unacked motor 2 must get no mode write"
    reenabled = [mid for mid, addr, val in calls
                 if addr == SMS_STS_TORQUE_ENABLE and val == 1]
    assert reenabled == [1, 3], "unacked motor 2 must not be torque re-enabled"


# ----- calibrate_offset contract -------------------------------------------

def test_calibrate_offset_reports_failure(client):
    feetech, handler = client
    handler.ofs_hook = lambda motor_id, position: (1, 0)
    assert feetech.calibrate_offset(2, upper=True) is False


def test_calibrate_offset_reports_success(client):
    feetech, _ = client
    assert feetech.calibrate_offset(2, upper=True) is True


# ----- bus lock -------------------------------------------------------------

def test_reads_hold_the_bus_lock(client):
    feetech, handler = client
    handler.client = feetech
    feetech.read_position_velocity_current()
    assert handler.lock_checks and all(handler.lock_checks)


def test_torque_and_mode_writes_hold_the_bus_lock(client, monkeypatch):
    feetech, handler = client
    _patch_sleep(monkeypatch)
    handler.client = feetech
    feetech.set_torque_enabled([1], True, retries=0)
    feetech.set_operating_mode([1, 2], 3)
    assert handler.lock_checks and all(handler.lock_checks)


def test_calibrate_offset_holds_the_bus_lock(client):
    feetech, handler = client
    handler.client = feetech
    feetech.calibrate_offset(1)
    assert handler.lock_checks and all(handler.lock_checks)


def test_lock_probe_detects_unheld_lock(client):
    feetech, handler = client
    assert _lock_held_by_another_thread(feetech._bus_lock) is False


# ----- connect-time advisory lock ------------------------------------------

class FakeConnectPortHandler:
    def __init__(self):
        self.is_open = False
        self.is_using = False
        self.baudrate = None
        self.ser = SimpleNamespace(fileno=lambda: 42)

    def openPort(self):
        self.is_open = True
        return True

    def closePort(self):
        self.is_open = False


def test_connect_takes_advisory_lock_on_port(monkeypatch):
    fcntl = pytest.importorskip("fcntl")
    flock_calls = []
    monkeypatch.setattr(fcntl, "flock",
                        lambda fd, op: flock_calls.append((fd, op)))
    monkeypatch.setattr(feetech_client_module, "sms_sts",
                        lambda port_handler: FakePacketHandler({}))

    feetech = FeetechClient(motor_ids=[], port="/dev/fake")
    feetech.port_handler = FakeConnectPortHandler()
    try:
        feetech.connect()
        assert flock_calls == [(42, fcntl.LOCK_EX | fcntl.LOCK_NB)]
        assert feetech.is_connected
    finally:
        feetech._connected = False
        FeetechClient.OPEN_CLIENTS.discard(feetech)


def test_connect_succeeds_when_flock_unavailable(monkeypatch):
    fcntl = pytest.importorskip("fcntl")

    def failing_flock(fd, op):
        raise OSError("resource temporarily unavailable")

    monkeypatch.setattr(fcntl, "flock", failing_flock)
    monkeypatch.setattr(feetech_client_module, "sms_sts",
                        lambda port_handler: FakePacketHandler({}))

    feetech = FeetechClient(motor_ids=[], port="/dev/fake")
    feetech.port_handler = FakeConnectPortHandler()
    try:
        feetech.connect()
        assert feetech.is_connected
    finally:
        feetech._connected = False
        FeetechClient.OPEN_CLIENTS.discard(feetech)
