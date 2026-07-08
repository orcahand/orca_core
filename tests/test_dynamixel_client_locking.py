"""Tests for DynamixelClient bus serialization and stale-RX flushing.

Uses a fake dynamixel_sdk (patched into sys.modules) so no hardware or serial
port is touched. The fake bus records every port operation and detects when a
transaction from one thread interleaves with an open transaction from another.
"""

import logging
import sys
import threading
import time
import types

import numpy as np
import pytest

from orca_core.hardware.dynamixel_client import DynamixelClient

COMM_SUCCESS = 0
COMM_RX_FAIL = -3001
COMM_NOT_AVAILABLE = -3002


class FakeBus:
    """Shared state of the fake wire.

    A bulk read is a multi-step transaction: one txPacket followed by one
    readRx per motor. While it is open, any port operation from a different
    thread counts as a violation (this is exactly what collapses the real
    bus).
    """

    def __init__(self):
        self._meta = threading.Lock()
        self._owner = None
        self._reads_left = 0
        self.violations = 0
        self.log = []
        self.op_delay = 0.0
        # Failure injection.
        self.bulk_tx_results = []  # queue of comm results for bulk txPacket
        self.write1_hook = None    # callable(motor_id) -> (comm, err)
        self.read1_hook = None     # callable(motor_id) -> (value, comm, err)
        self.read2_hook = None     # callable(motor_id, address) -> (value, comm, err)
        self.read4_hook = None     # callable(motor_id, address) -> (value, comm, err)

    def _check_owner(self):
        me = threading.get_ident()
        if self._owner is not None and self._owner != me:
            self.violations += 1

    def instant(self, name):
        with self._meta:
            self._check_owner()
            self.log.append(name)
        if self.op_delay:
            time.sleep(self.op_delay)

    def bulk_tx(self, result, expected_reads):
        with self._meta:
            self._check_owner()
            self.log.append('bulk_tx')
            if result == COMM_SUCCESS and expected_reads > 0:
                self._owner = threading.get_ident()
                self._reads_left = expected_reads
        if self.op_delay:
            time.sleep(self.op_delay)

    def bulk_rx(self):
        with self._meta:
            self._check_owner()
            self.log.append('bulk_rx')
            self._reads_left -= 1
            if self._reads_left <= 0:
                self._owner = None
        if self.op_delay:
            time.sleep(self.op_delay)

    def flush(self):
        with self._meta:
            self.log.append('flush')

    def events(self, *names):
        return [e for e in self.log if e in names]


def make_fake_sdk(bus):
    class FakeSerial:
        def reset_input_buffer(self):
            bus.flush()

    class PortHandler:
        def __init__(self, port):
            self.port_name = port
            self.is_open = False
            self.is_using = False
            self.ser = FakeSerial()

        def openPort(self):
            self.is_open = True
            return True

        def setBaudRate(self, baudrate):
            self.baudrate = baudrate
            return True

        def closePort(self):
            self.is_open = False

    class PacketHandler:
        def __init__(self, protocol_version):
            self.protocol_version = protocol_version

        def write1ByteTxRx(self, port, motor_id, address, value):
            bus.instant('write1')
            if bus.write1_hook is not None:
                return bus.write1_hook(motor_id)
            return COMM_SUCCESS, 0

        def read1ByteTxRx(self, port, motor_id, address):
            bus.instant('read1')
            if bus.read1_hook is not None:
                return bus.read1_hook(motor_id)
            return 0, COMM_SUCCESS, 0

        def read2ByteTxRx(self, port, motor_id, address):
            bus.instant('read2')
            if bus.read2_hook is not None:
                return bus.read2_hook(motor_id, address)
            return 0, COMM_SUCCESS, 0

        def read4ByteTxRx(self, port, motor_id, address):
            bus.instant('read4')
            if bus.read4_hook is not None:
                return bus.read4_hook(motor_id, address)
            return 0, COMM_SUCCESS, 0

        def reboot(self, port, motor_id):
            bus.instant('reboot')
            return COMM_SUCCESS, 0

        def readRx(self, port, motor_id, length):
            bus.bulk_rx()
            return bytes(length), COMM_SUCCESS, 0

        def getTxRxResult(self, comm_result):
            return 'comm_result={}'.format(comm_result)

        def getRxPacketError(self, dxl_error):
            return 'dxl_error={}'.format(dxl_error)

    class GroupBulkRead:
        def __init__(self, port, packet_handler):
            self.port = port
            self.ph = packet_handler
            self.data_dict = {}
            self.last_result = False

        def addParam(self, motor_id, address, size):
            self.data_dict[motor_id] = [None, address, size]
            return True

        def txPacket(self):
            if bus.bulk_tx_results:
                result = bus.bulk_tx_results.pop(0)
            else:
                result = COMM_SUCCESS
            bus.bulk_tx(result, expected_reads=len(self.data_dict))
            return result

        def isAvailable(self, motor_id, address, size):
            return True

        def getData(self, motor_id, address, size):
            return 0

    class GroupSyncWrite:
        def __init__(self, port, packet_handler, address, size):
            self.params = {}

        def addParam(self, motor_id, value):
            self.params[motor_id] = value
            return True

        def txPacket(self):
            bus.instant('sync_tx')
            return COMM_SUCCESS

        def clearParam(self):
            self.params = {}

    sdk = types.ModuleType('dynamixel_sdk')
    sdk.COMM_SUCCESS = COMM_SUCCESS
    sdk.COMM_RX_FAIL = COMM_RX_FAIL
    sdk.COMM_NOT_AVAILABLE = COMM_NOT_AVAILABLE
    sdk.PortHandler = PortHandler
    sdk.PacketHandler = PacketHandler
    sdk.GroupBulkRead = GroupBulkRead
    sdk.GroupSyncWrite = GroupSyncWrite
    return sdk


@pytest.fixture
def bus():
    return FakeBus()


@pytest.fixture
def client(bus, monkeypatch, request):
    monkeypatch.setitem(sys.modules, 'dynamixel_sdk', make_fake_sdk(bus))
    dxl_client = DynamixelClient([1, 2], port='/dev/fake', baudrate=57600)
    dxl_client.port_handler.is_open = True  # pretend connected

    def cleanup():
        dxl_client.port_handler.is_open = False
        DynamixelClient.OPEN_CLIENTS.discard(dxl_client)

    request.addfinalizer(cleanup)
    return dxl_client


def _patch_sleep(monkeypatch):
    import orca_core.hardware.dynamixel_client as dxl_mod
    sleeps = []
    monkeypatch.setattr(dxl_mod.time, 'sleep', lambda s: sleeps.append(s))
    return sleeps


class _NoopLock:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def _run_read_write_race(client, read_iterations):
    errors = []
    reads_done = threading.Event()

    def reader():
        try:
            for _ in range(read_iterations):
                client.read_position_velocity_current()
        except Exception as exc:  # pragma: no cover - failure reporting
            errors.append(exc)
        finally:
            reads_done.set()

    def writer():
        try:
            while not reads_done.is_set():
                client.write_desired_pos([1, 2], np.zeros(2))
        except Exception as exc:  # pragma: no cover - failure reporting
            errors.append(exc)

    threads = [threading.Thread(target=reader), threading.Thread(target=writer)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30)
        assert not t.is_alive(), 'race test thread did not finish'
    assert not errors, errors


def test_concurrent_read_and_sync_write_never_interleave(client, bus):
    bus.op_delay = 0.0005
    _run_read_write_race(client, read_iterations=60)
    assert bus.violations == 0


def test_guard_detects_unserialized_access(client, bus):
    """Sanity check: without the bus lock the same workload does interleave.

    Proves the violation detector in the previous test is not vacuous.
    """
    client._bus_lock = _NoopLock()
    bus.op_delay = 0.002
    _run_read_write_race(client, read_iterations=30)
    assert bus.violations > 0


def test_failed_read_flushes_rx_before_retry(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL]
    reader = client._pos_vel_cur_reader
    reader.read(retries=1)
    assert reader.last_read_ok is True
    assert bus.events('bulk_tx', 'flush') == ['bulk_tx', 'flush', 'bulk_tx']


def test_read_flushes_rx_after_final_failure(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL, COMM_RX_FAIL]
    # Per-motor fallback also fails, so the cache stays stale.
    bus.read4_hook = lambda motor_id, address: (0, COMM_RX_FAIL, 0)
    bus.read2_hook = lambda motor_id, address: (0, COMM_RX_FAIL, 0)
    reader = client._pos_vel_cur_reader
    reader.read(retries=1)
    assert reader.last_read_ok is False
    assert bus.events('bulk_tx', 'flush') == [
        'bulk_tx', 'flush', 'bulk_tx', 'flush']


def test_bulk_failure_recovers_via_per_motor_fallback(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL, COMM_RX_FAIL]
    bus.read4_hook = lambda motor_id, address: (2048, COMM_SUCCESS, 0)
    bus.read2_hook = lambda motor_id, address: (100, COMM_SUCCESS, 0)
    reader = client._pos_vel_cur_reader
    positions, velocities, currents = reader.read(retries=1)
    assert reader.last_read_ok is True
    assert np.all(positions > 0)
    assert np.all(currents > 0)
    # One 4-byte read for pos and vel each, per motor.
    assert len(bus.events('read4')) == 2 * len(reader.motor_ids)


def test_partial_fallback_failure_marks_read_not_ok(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL, COMM_RX_FAIL]
    # Motor 1 answers individually; motor 2 stays silent.
    bus.read4_hook = lambda motor_id, address: (
        (2048, COMM_SUCCESS, 0) if motor_id == 1 else (0, COMM_RX_FAIL, 0))
    bus.read2_hook = lambda motor_id, address: (
        (100, COMM_SUCCESS, 0) if motor_id == 1 else (0, COMM_RX_FAIL, 0))
    reader = client._pos_vel_cur_reader
    positions, _, _ = reader.read(retries=1)
    assert reader.last_read_ok is False
    assert positions[0] > 0  # motor 1 refreshed even though the read is flagged


def test_set_torque_enabled_does_not_sleep_on_immediate_success(
        client, bus, monkeypatch):
    sleeps = _patch_sleep(monkeypatch)
    result = client.set_torque_enabled([1, 2], True)
    assert result == []
    assert sleeps == []
    assert bus.events('write1') == ['write1', 'write1']


def test_set_torque_enabled_default_retries_are_finite(
        client, bus, monkeypatch):
    sleeps = _patch_sleep(monkeypatch)
    bus.write1_hook = lambda motor_id: (COMM_RX_FAIL, 0)
    result = client.set_torque_enabled([1], True)
    assert result == [1]
    assert len(bus.events('write1')) == 4  # first attempt + 3 retries
    assert sleeps == [0.25, 0.25, 0.25]
    assert 'flush' in bus.log


def test_set_torque_enabled_returns_failed_ids_and_sleeps_between_retries(
        client, bus, monkeypatch):
    sleeps = _patch_sleep(monkeypatch)
    bus.write1_hook = lambda motor_id: (
        (COMM_RX_FAIL, 0) if motor_id == 2 else (COMM_SUCCESS, 0))
    result = client.set_torque_enabled([1, 2], True, retries=2,
                                       retry_interval=0.1)
    assert result == [2]
    assert sleeps == [0.1, 0.1]


def test_set_torque_enabled_retries_zero_is_single_attempt(
        client, bus, monkeypatch):
    sleeps = _patch_sleep(monkeypatch)
    bus.write1_hook = lambda motor_id: (COMM_RX_FAIL, 0)
    result = client.set_torque_enabled([1, 2], True, retries=0)
    assert result == [1, 2]
    assert sleeps == []
    assert bus.events('write1') == ['write1', 'write1']


def test_set_torque_enabled_negative_retries_still_means_forever(
        client, bus, monkeypatch):
    sleeps = _patch_sleep(monkeypatch)
    attempts = {'count': 0}

    def flaky(motor_id):
        attempts['count'] += 1
        if attempts['count'] <= 5:
            return COMM_RX_FAIL, 0
        return COMM_SUCCESS, 0

    bus.write1_hook = flaky
    result = client.set_torque_enabled([1], True, retries=-1,
                                       retry_interval=0.01)
    assert result == []
    assert attempts['count'] == 6
    assert len(sleeps) == 5


def test_read_hardware_error_returns_none_on_comm_failure(client, bus):
    bus.read1_hook = lambda motor_id: (0, COMM_RX_FAIL, 0)
    assert client.read_hardware_error(1) is None
    assert 'flush' in bus.log


def test_read_hardware_error_returns_value_on_success(client, bus):
    bus.read1_hook = lambda motor_id: (0x20, COMM_SUCCESS, 0)
    assert client.read_hardware_error(1) == 0x20


def test_connect_takes_advisory_lock_on_port(client, monkeypatch):
    fcntl = pytest.importorskip('fcntl')
    flock_calls = []
    monkeypatch.setattr(fcntl, 'flock',
                        lambda fd, op: flock_calls.append((fd, op)))
    client.port_handler.is_open = False
    client.port_handler.ser.fileno = lambda: 42
    client.connect()
    assert flock_calls == [(42, fcntl.LOCK_EX | fcntl.LOCK_NB)]
    assert client.is_connected


def test_connect_succeeds_when_flock_fails(client, monkeypatch):
    fcntl = pytest.importorskip('fcntl')

    def failing_flock(fd, op):
        raise OSError('resource temporarily unavailable')

    monkeypatch.setattr(fcntl, 'flock', failing_flock)
    client.port_handler.is_open = False
    client.port_handler.ser.fileno = lambda: 42
    client.connect()
    assert client.is_connected


def test_connect_succeeds_without_fileno(client):
    # FakeSerial has no fileno(); the AttributeError must be swallowed.
    client.port_handler.is_open = False
    client.connect()
    assert client.is_connected


def test_check_overload_retries_once_then_skips_on_no_reply(
        client, bus, caplog):
    calls = []

    def failing(motor_id):
        calls.append(motor_id)
        return 0, COMM_RX_FAIL, 0

    bus.read1_hook = failing
    with caplog.at_level(logging.WARNING):
        rebooted = client.check_overload_and_reboot([1])
    assert rebooted == []
    assert calls == [1, 1]  # one retry, then gives up
    assert any('skipping overload check' in record.getMessage()
               for record in caplog.records)
    assert not bus.events('reboot')


def test_full_fallback_is_rate_limited(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL] * 4  # two reads x two attempts
    bus.read4_hook = lambda motor_id, address: (2048, COMM_SUCCESS, 0)
    bus.read2_hook = lambda motor_id, address: (100, COMM_SUCCESS, 0)
    reader = client._pos_vel_cur_reader
    reader.read(retries=1)
    first_sweep_reads = len(bus.events('read4'))
    assert first_sweep_reads > 0
    # Immediately after, another total bulk failure must NOT sweep again.
    reader.read(retries=1)
    assert len(bus.events('read4')) == first_sweep_reads
    assert reader.last_read_ok is False


def test_failed_motor_enters_cooldown(client, bus):
    bus.bulk_tx_results = [COMM_RX_FAIL, COMM_RX_FAIL]
    bus.read4_hook = lambda motor_id, address: (0, COMM_RX_FAIL, 0)
    bus.read2_hook = lambda motor_id, address: (0, COMM_RX_FAIL, 0)
    reader = client._pos_vel_cur_reader
    reader.read(retries=1)
    assert set(reader._fallback_skip_until) == set(reader.motor_ids)
