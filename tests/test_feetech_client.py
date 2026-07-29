"""Unit tests for ``FeetechClient`` read caching and ``last_read_ok``.

A motor missing from a partial sync read (or failing a per-motor fallback
read) must keep its previously cached value — never a spurious 0.0 — and
the read must be reported as not ok so callers can discard or retry it.
"""

from __future__ import annotations

import numpy as np
import pytest

import orca_core.hardware.feetech_client as feetech_client_module
from orca_core.hardware.feetech_client import (
    COMM_SUCCESS,
    FeetechClient,
)
from orca_core.hardware.feetech import (
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
)


class FakePacketHandler:
    """Serves raw register values per motor; ``unavailable_ids`` simulate
    motors that drop their reply."""

    def __init__(self, positions: dict[int, int]):
        self.positions = dict(positions)
        self.unavailable_ids: set[int] = set()
        self.sync_fails = False

    def scs_tohost(self, raw, bit):
        return raw

    def read2ByteTxRx(self, motor_id, address):
        if motor_id in self.unavailable_ids:
            return 0, 1, 0
        if address == SMS_STS_PRESENT_POSITION_L:
            return self.positions[motor_id], COMM_SUCCESS, 0
        return 0, COMM_SUCCESS, 0


class FakeGroupSyncRead:
    def __init__(self, packet_handler, address, size):
        self.handler = packet_handler

    def addParam(self, motor_id):
        return True

    def txRxPacket(self):
        return 1 if self.handler.sync_fails else COMM_SUCCESS

    def isAvailable(self, motor_id, address, size):
        return motor_id not in self.handler.unavailable_ids, 0

    def getData(self, motor_id, address, size):
        if address == SMS_STS_PRESENT_POSITION_L:
            return self.handler.positions[motor_id]
        assert address in (SMS_STS_PRESENT_SPEED_L, SMS_STS_PRESENT_CURRENT_L)
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
