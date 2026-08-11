"""Tests for the encoder slot-health/identity preamble (maintenance routine)."""

import os
import shutil
import time

import numpy as np
import pytest

from orca_core import MockOrcaHand
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.types import EncoderReading
from orca_core.maintenance import encoder_slot_check
from orca_core.maintenance.encoder_slot_check import run_slot_check

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand-full-right")

COUNTS_PER_MOTOR_RAD = 800  # fake gearing: 0.35 rad wiggle -> ~280 counts ~ 6 deg


@pytest.fixture
def hand(tmp_path):
    shutil.copy(os.path.join(MODEL_DIR, "config.yaml"), tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    success, msg = hand.connect()
    assert success, msg
    try:
        yield hand
    finally:
        hand.disconnect()


class FakeEncoderClient:
    """Encoder stream whose counts track the mock hand's motor positions.

    ``slot_map`` overrides joint->slot wiring to simulate crossed connectors;
    ``flagged_slots`` assert the chip angle-error bit on every frame;
    ``dead_joints`` produce no count response to their motor.
    """

    def __init__(self, hand, slot_map=None, flagged_slots=(), dead_joints=()):
        self._hand = hand
        self._slot_map = dict(slot_map or JOINT_TO_ENCODER_SLOT)
        self._flagged = set(flagged_slots)
        self._dead = set(dead_joints)
        self._tick = 0

    def get_latest(self):
        self._tick += 1
        motor_pos = self._hand.get_motor_pos()
        counts = np.full(AUTO_ENC_NUM_JOINTS, 2000, dtype=np.uint16)
        for joint, slot in self._slot_map.items():
            if joint in self._dead:
                continue
            motor_id = self._hand.config.joint_to_motor_map[joint]
            idx = self._hand.config.motor_id_to_idx_dict[motor_id]
            counts[slot] = int(
                2000 + COUNTS_PER_MOTOR_RAD * motor_pos[idx] + (self._tick % 2)
            ) % 16384
        angle_error = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool)
        for slot in self._flagged:
            angle_error[slot] = True
        return EncoderReading(
            raw_counts=counts,
            parity_ok=np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool),
            angle_error=angle_error,
            error_byte=0,
            timestamp=time.monotonic() + self._tick * 1e-7,
        )


@pytest.fixture(autouse=True)
def fast_settling(monkeypatch):
    monkeypatch.setattr(encoder_slot_check, "HEALTH_SAMPLE_S", 0.15)
    monkeypatch.setattr(encoder_slot_check, "HEALTH_MIN_FRAMES", 10)
    monkeypatch.setattr(encoder_slot_check, "SETTLE_S", 0.0)
    monkeypatch.setattr(encoder_slot_check, "SAMPLE_FRAMES", 5)


def test_healthy_and_correctly_mapped_passes(hand):
    result = run_slot_check(hand, FakeEncoderClient(hand))
    assert result is not None
    assert result.passed
    assert all(h.verdict == "ok" for h in result.health)
    verdicts = {i.joint: i.verdict for i in result.identity}
    assert set(verdicts.values()) == {"ok"}
    assert len(verdicts) == len(hand._encoder_backed_joints())


def test_crossed_slots_detected_as_mismapped(hand):
    crossed = dict(JOINT_TO_ENCODER_SLOT)
    crossed["middle_abd"], crossed["ring_abd"] = crossed["ring_abd"], crossed["middle_abd"]
    result = run_slot_check(hand, FakeEncoderClient(hand, slot_map=crossed))
    assert result is not None and not result.passed
    verdicts = {i.joint: i.verdict for i in result.identity}
    assert verdicts["middle_abd"] == "mismapped"
    assert verdicts["ring_abd"] == "mismapped"
    assert verdicts["index_abd"] == "ok"
    mism = next(i for i in result.identity if i.joint == "middle_abd")
    assert mism.argmax_slot == JOINT_TO_ENCODER_SLOT["ring_abd"]


def test_flagged_slot_fails_health_and_skips_identity(hand):
    flagged_slot = JOINT_TO_ENCODER_SLOT["middle_pip"]
    result = run_slot_check(hand, FakeEncoderClient(hand, flagged_slots={flagged_slot}))
    assert result is not None and not result.passed
    health = {h.joint: h.verdict for h in result.health}
    assert health["middle_pip"] == "flagged"
    assert all(v == "ok" for j, v in health.items() if j != "middle_pip")
    identity = {i.joint: i.verdict for i in result.identity}
    assert identity["middle_pip"] == "skipped-flagged"
    assert identity["index_mcp"] == "ok"


def test_dead_slot_reports_no_response(hand):
    result = run_slot_check(hand, FakeEncoderClient(hand, dead_joints={"pinky_pip"}))
    assert result is not None and not result.passed
    identity = {i.joint: i.verdict for i in result.identity}
    assert identity["pinky_pip"] == "no-response"


def test_health_only_skips_motion(hand):
    moves = []
    original = hand._set_motor_pos
    hand._set_motor_pos = lambda *a, **k: moves.append(a) or original(*a, **k)
    result = run_slot_check(hand, FakeEncoderClient(hand), identity=False)
    assert result is not None and result.passed
    assert result.identity == []
    assert moves == []


def test_should_stop_aborts_and_releases(hand):
    calls = {"n": 0}

    def stop():
        calls["n"] += 1
        return calls["n"] > 1  # let the first joint start, then abort

    result = run_slot_check(hand, FakeEncoderClient(hand), should_stop=stop)
    assert result is None
