"""Feetech bus behaviour that only ever got exercised on Dynamixel hands.

Connecting must not mutate motor state, EEPROM writes must be bracketed by the
unlock/lock pair and skipped when nothing changes, torque limits are per motor
and reach the bus on their own, a latched status flag qualifies the motor
without invalidating the reply it came with, and a baud-rate change must be
followed to the new rate before the EEPROM is re-locked.

Also covers the connect-time driver probe falling back to a full sweep when
the pinned family stays silent.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

import orca_core.hardware.feetech_client as feetech_client_module
from orca_core.hardware import motor_resolution
from orca_core.hardware.feetech_client import (
    COMM_SUCCESS,
    DEFAULT_TORQUE_LIMIT,
    FeetechClient,
)
from orca_core.hardware.feetech import (
    SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L,
    SMS_STS_GOAL_TIME_L,
    SMS_STS_LOCK,
    SMS_STS_MODE,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_TORQUE_ENABLE,
)


# ----- fakes ----------------------------------------------------------------


class FakePortHandler:
    """Port handler recording every baud rate it was opened at."""

    def __init__(self, port: str):
        self.port_name = port
        self.is_open = False
        self.is_using = False
        self.baudrate = 1_000_000
        self.ser = None
        self.baud_history: list[int] = []
        self.set_baud_result = True

    def openPort(self) -> bool:
        self.is_open = True
        self.baud_history.append(self.baudrate)
        return True

    def setBaudRate(self, baudrate: int) -> bool:
        if not self.set_baud_result:
            return False
        self.baudrate = baudrate
        self.baud_history.append(baudrate)
        return True

    def closePort(self) -> None:
        self.is_open = False


class FakeGroupSyncWrite:
    """The packet handler's own position sync-write group."""

    def __init__(self, handler):
        self.handler = handler

    def clearParam(self):
        pass

    def txPacket(self):
        return COMM_SUCCESS


class FakeSmsSts:
    """Records every transaction a FeetechClient issues, in order."""

    def __init__(self, port_handler=None):
        self.port_handler = port_handler
        self.writes: list[tuple[int, int, int]] = []   # (motor_id, address, value)
        self.sync_writes: list[tuple[int, list[int]]] = []  # (address, params)
        self.pos_writes: list[tuple] = []              # SyncWritePosEx arguments
        self.pings: list[int] = []
        self.write_hook = None
        self.ping_result = COMM_SUCCESS
        self.modes: dict[int, int] = {}  # mode register per motor; 0 = servo
        self.groupSyncWrite = FakeGroupSyncWrite(self)

    def read1ByteTxRx(self, motor_id, address):
        if address == SMS_STS_MODE:
            return self.modes.get(motor_id, 0), COMM_SUCCESS, 0
        return 0, COMM_SUCCESS, 0

    def write1ByteTxRx(self, motor_id, address, value):
        self.writes.append((motor_id, address, value))
        if self.write_hook is not None:
            return self.write_hook(motor_id, address, value)
        return COMM_SUCCESS, 0

    def unLockEprom(self, motor_id):
        return self.write1ByteTxRx(motor_id, SMS_STS_LOCK, 0)

    def LockEprom(self, motor_id):
        return self.write1ByteTxRx(motor_id, SMS_STS_LOCK, 1)

    def ping(self, motor_id):
        self.pings.append(motor_id)
        return 0, self.ping_result, 0

    def syncWriteTxOnly(self, start_address, data_length, param, param_length):
        self.sync_writes.append((start_address, list(param)))
        return COMM_SUCCESS

    def SyncWritePosEx(self, motor_id, position, speed, acc, torque):
        self.pos_writes.append((motor_id, position, speed, acc, torque))
        return True

    def scs_toscs(self, value, bit):
        return value

    def scs_tohost(self, value, bit):
        return value

    def scs_lobyte(self, word):
        return word & 0xFF

    def scs_hibyte(self, word):
        return (word >> 8) & 0xFF

    def getRxPacketError(self, error):
        return f"error 0x{error:02X}"


class FakeSyncRead:
    """Sync read whose per-motor status byte is scripted via ``status``."""

    status: dict[int, int] = {}
    positions: dict[int, int] = {}

    def __init__(self, packet_handler, address, size):
        self.address = address

    def addParam(self, motor_id):
        return True

    def txRxPacket(self):
        return COMM_SUCCESS

    def isAvailable(self, motor_id, address, size):
        return True, FakeSyncRead.status.get(motor_id, 0)

    def getData(self, motor_id, address, size):
        if address == SMS_STS_PRESENT_POSITION_L:
            return FakeSyncRead.positions.get(motor_id, 0)
        return 0


@pytest.fixture
def client(monkeypatch):
    """A connected FeetechClient whose bus is a recording fake."""
    monkeypatch.setattr(feetech_client_module, "PortHandler", FakePortHandler)
    monkeypatch.setattr(feetech_client_module, "sms_sts", FakeSmsSts)
    feetech = FeetechClient(motor_ids=[1, 2, 3], port="/dev/fake")
    feetech.connect()
    yield feetech, feetech.packet_handler
    feetech._connected = False
    FeetechClient.OPEN_CLIENTS.discard(feetech)


def _decode(param: list[int], data_length: int) -> "dict[int, list[int]]":
    """Split a sync-write param block into {motor_id: data bytes}."""
    stride = 1 + data_length
    return {
        param[i]: param[i + 1:i + stride]
        for i in range(0, len(param), stride)
    }


# ----- connect must not mutate motor state ----------------------------------


def test_connect_writes_no_mode_register(client):
    feetech, handler = client
    assert [w for w in handler.writes if w[1] == SMS_STS_MODE] == []
    assert handler.writes == [], "connect() must not write any motor register"


# ----- EEPROM-bracketed mode writes -----------------------------------------


def test_set_operating_mode_brackets_the_mode_write(client):
    feetech, handler = client
    handler.modes = {1: 1}  # motor left in wheel mode, so the mode must change
    feetech.set_operating_mode([1], 5)

    eeprom = [(addr, value) for mid, addr, value in handler.writes
              if mid == 1 and addr in (SMS_STS_LOCK, SMS_STS_MODE)]
    assert eeprom == [(SMS_STS_LOCK, 0), (SMS_STS_MODE, 0), (SMS_STS_LOCK, 1)]


def test_set_operating_mode_skips_motors_whose_mode_write_failed(client):
    feetech, handler = client

    def hook(motor_id, address, value):
        if motor_id == 2 and address == SMS_STS_MODE:
            return -3, 0  # comm failure: the motor never answered
        return COMM_SUCCESS, 0

    handler.write_hook = hook
    handler.modes = {1: 1, 2: 1, 3: 1}
    feetech.set_operating_mode([1, 2, 3], 5)

    reenabled = [mid for mid, addr, value in handler.writes
                 if addr == SMS_STS_TORQUE_ENABLE and value == 1]
    assert reenabled == [1, 3], "a motor with an unacked mode write must stay off"
    acc_params = [p for addr, p in handler.sync_writes if addr == SMS_STS_ACC]
    assert list(_decode(acc_params[0], 1)) == [1, 3]


def test_set_operating_mode_writes_the_motion_profile_once(client):
    feetech, handler = client
    feetech.write_desired_current([1, 2, 3], np.full(3, 400.0))
    handler.sync_writes.clear()
    feetech.set_operating_mode([1, 2, 3], 5)

    assert [addr for addr, _ in handler.sync_writes] == [
        SMS_STS_ACC, SMS_STS_GOAL_TIME_L]
    # Nothing moves while the torque limit reads zero, so it is established here.
    motion = _decode([p for addr, p in handler.sync_writes
                      if addr == SMS_STS_GOAL_TIME_L][0], 4)
    assert motion[1][:2] == [400 & 0xFF, 400 >> 8]


def test_status_error_on_mode_write_does_not_fail_the_motor(client):
    feetech, handler = client
    handler.write_hook = lambda motor_id, address, value: (COMM_SUCCESS, 0x20)
    handler.modes = {1: 1}
    feetech.set_operating_mode([1], 5)

    reenabled = [mid for mid, addr, value in handler.writes
                 if addr == SMS_STS_TORQUE_ENABLE and value == 1]
    assert reenabled == [1], "a latched status flag is not a failed transaction"


def test_read_hardware_error_returns_the_status_byte(client):
    feetech, handler = client
    assert feetech.read_hardware_error(1) == 0
    handler.ping_result = -3
    assert feetech.read_hardware_error(1) is None


# ----- per-motor torque limits ----------------------------------------------


def test_write_desired_current_keeps_one_torque_per_motor(client):
    feetech, handler = client
    feetech.write_desired_current([1, 2], np.array([300.0, 700.0]))

    assert feetech._motor_torque[1] == 300
    assert feetech._motor_torque[2] == 700
    assert feetech._motor_torque[3] == DEFAULT_TORQUE_LIMIT


def test_write_desired_current_reaches_the_bus_immediately(client):
    feetech, handler = client
    feetech.write_desired_current([1, 2], np.array([300.0, 700.0]))

    torque_writes = [p for addr, p in handler.sync_writes if addr == SMS_STS_GOAL_TIME_L]
    assert len(torque_writes) == 1, "the torque limit must not wait for a position"
    assert _decode(torque_writes[0], 2) == {1: [300 & 0xFF, 300 >> 8],
                                            2: [700 & 0xFF, 700 >> 8]}


def test_write_positions_sync_composes_per_motor_torque(client):
    feetech, handler = client
    feetech.write_desired_current([1, 2], np.array([300.0, 700.0]))
    feetech.write_positions_sync([1, 2], np.zeros(2))

    assert [(mid, torque) for mid, _, _, _, torque in handler.pos_writes] == [
        (1, 300), (2, 700)]


def test_explicit_torque_overrides_the_stored_limit(client):
    feetech, handler = client
    feetech.write_desired_current([1, 2], np.array([300.0, 700.0]))
    feetech.write_positions_sync([1, 2], np.zeros(2), torque=100)

    assert {torque for _, _, _, _, torque in handler.pos_writes} == {100}


# ----- position-only hot path -----------------------------------------------


def test_write_desired_pos_does_not_rearm_the_motion_profile(client):
    feetech, handler = client
    feetech.write_desired_pos([1, 2], np.zeros(2))

    assert handler.pos_writes == [], "the profile must not be re-sent per command"
    assert [addr for addr, _ in handler.sync_writes] == [SMS_STS_GOAL_POSITION_L]


def test_write_desired_pos_with_explicit_speed_uses_the_profile_packet(client):
    feetech, handler = client
    feetech.write_desired_pos([1, 2], np.zeros(2), speed=200)

    assert [speed for _, _, speed, _, _ in handler.pos_writes] == [200, 200]


# ----- out-of-range commands are observable ---------------------------------


def test_out_of_range_command_is_counted_and_warned(client, caplog):
    feetech, handler = client
    # POSITION_DIRECTION makes the motor frame negative, so a positive radian
    # command lands below POS_MIN and clamps.
    with caplog.at_level("WARNING"):
        feetech.write_desired_pos([1], np.array([1.0]))

    assert feetech.clamped_command_counts()[1] == 1
    assert any("outside" in record.getMessage() for record in caplog.records)


def test_repeated_out_of_range_commands_warn_at_a_bounded_rate(client, caplog):
    feetech, handler = client
    with caplog.at_level("WARNING"):
        for _ in range(5):
            feetech.write_desired_pos([1], np.array([1.0]))

    assert feetech.clamped_command_counts()[1] == 5
    warnings = [r for r in caplog.records if "outside" in r.getMessage()]
    assert len(warnings) == 1


# ----- read freshness --------------------------------------------------------


def test_sync_read_keeps_a_flagged_motors_fresh_sample(client, monkeypatch):
    """A latched overload is the motor's condition, not a failed read: tensioning
    stalls the motors on purpose and must still see them move."""
    feetech, handler = client
    monkeypatch.setattr(feetech_client_module, "GroupSyncRead", FakeSyncRead)
    FakeSyncRead.positions = {1: 100, 2: 200, 3: 300}
    FakeSyncRead.status = {}

    feetech.read_position_velocity_current()
    assert feetech.last_read_ok is True

    FakeSyncRead.status = {2: 0x20}  # motor 2 reports an overload flag
    FakeSyncRead.positions = {1: 100, 2: 250, 3: 300}
    read = feetech.read_position_velocity_current()
    assert feetech.last_read_ok is True
    assert read.position[1] == feetech._raw_to_rad(250, feetech.pos_scale)

    FakeSyncRead.status = {}


def test_sync_read_flags_a_motor_that_did_not_answer(client, monkeypatch):
    feetech, handler = client

    class MissingMotorSyncRead(FakeSyncRead):
        def isAvailable(self, motor_id, address, size):
            return motor_id != 2, 0

    monkeypatch.setattr(feetech_client_module, "GroupSyncRead", MissingMotorSyncRead)
    feetech.read_position_velocity_current()
    assert feetech.last_read_ok is False


def test_set_operating_mode_does_not_rewrite_an_unchanged_mode(client):
    """The mode register is EEPROM: re-selecting the mode a motor is already in
    must not spend a write cycle on it."""
    feetech, handler = client
    handler.modes = {1: 0, 2: 0, 3: 0}
    feetech.set_operating_mode([1, 2, 3], 5)

    assert [w for w in handler.writes if w[1] in (SMS_STS_MODE, SMS_STS_LOCK)] == []
    reenabled = [mid for mid, addr, value in handler.writes
                 if addr == SMS_STS_TORQUE_ENABLE and value == 1]
    assert reenabled == [1, 2, 3], "torque must still come back on"


def test_set_operating_mode_writes_once_across_repeated_calls(client):
    feetech, handler = client
    handler.modes = {1: 1}
    feetech.set_operating_mode([1], 5)
    handler.writes.clear()
    feetech.set_operating_mode([1], 5)

    assert [w for w in handler.writes if w[1] == SMS_STS_MODE] == []


# ----- baud-rate changes ------------------------------------------------------


def test_change_motor_baudrate_relocks_eeprom_at_the_new_baud(client):
    feetech, handler = client
    assert feetech.change_motor_baudrate(1, 500_000) is True

    assert feetech.port_handler.baudrate == 500_000
    assert feetech.baudrate == 500_000
    # The lock write and the confirming ping must both follow the baud switch.
    lock_index = max(i for i, (_, addr, value) in enumerate(handler.writes)
                     if addr == SMS_STS_LOCK and value == 1)
    baud_index = next(i for i, (_, addr, _) in enumerate(handler.writes)
                      if addr == feetech_client_module.SMS_STS_BAUD_RATE)
    assert lock_index > baud_index
    assert handler.pings == [1]


def test_change_motor_baudrate_fails_when_the_motor_goes_silent(client):
    feetech, handler = client
    handler.ping_result = -3
    assert feetech.change_motor_baudrate(1, 500_000) is False


# ----- connect-time driver probe ---------------------------------------------


def _probe_config(motor_type=None, baudrate=None):
    return SimpleNamespace(
        motor_type=motor_type, baudrate=baudrate, motor_ids=[1, 2])


def test_trial_probe_falls_back_to_a_full_sweep_when_the_pin_is_silent(monkeypatch):
    """A Feetech hand on a config pinning dynamixel must still come up."""
    from orca_core.hardware import dynamixel_client, feetech_client

    monkeypatch.setattr(dynamixel_client.DynamixelClient, "probe",
                        staticmethod(lambda *a, **k: False))
    monkeypatch.setattr(feetech_client.FeetechClient, "probe",
                        staticmethod(lambda port, baudrate, motor_ids: baudrate == 500_000))

    config = _probe_config(motor_type="dynamixel", baudrate=1_000_000)
    assert motor_resolution.trial_probe(config, "/dev/cu.x") == ("feetech", 500_000)


def test_trial_probe_tries_the_pinned_combination_first(monkeypatch):
    from orca_core.hardware import dynamixel_client, feetech_client

    seen = []

    def dxl_probe(port, baudrate, motor_ids):
        seen.append(("dynamixel", baudrate))
        return baudrate == 1_000_000

    monkeypatch.setattr(dynamixel_client.DynamixelClient, "probe",
                        staticmethod(dxl_probe))
    monkeypatch.setattr(feetech_client.FeetechClient, "probe",
                        staticmethod(lambda *a, **k: False))

    config = _probe_config(motor_type="dynamixel", baudrate=1_000_000)
    assert motor_resolution.trial_probe(config, "/dev/cu.x") == ("dynamixel", 1_000_000)
    assert seen == [("dynamixel", 1_000_000)]


def test_trial_probe_sweeps_the_supported_baudrate_tail(monkeypatch):
    """Rates outside MOTOR_BAUD_RATES are still reachable, after it."""
    from orca_core.hardware import dynamixel_client, feetech_client

    seen = []

    def feetech_probe(port, baudrate, motor_ids):
        seen.append(baudrate)
        return baudrate == 115_200

    monkeypatch.setattr(dynamixel_client.DynamixelClient, "probe",
                        staticmethod(lambda *a, **k: False))
    monkeypatch.setattr(feetech_client.FeetechClient, "probe",
                        staticmethod(feetech_probe))

    motor_type, baudrate = motor_resolution.trial_probe(_probe_config(), "/dev/cu.x")
    assert (motor_type, baudrate) == ("feetech", 115_200)
    assert seen[0] == 1_000_000, "the priority rate is still tried first"
    assert seen[1] == 500_000, "the tail follows in descending order"
