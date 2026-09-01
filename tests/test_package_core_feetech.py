"""Hand behaviour that differs on a single-turn motor family (Feetech).

Exercised through a mock hand whose config pins ``motor_type: feetech``, so
the family-specific branches run without Dynamixel hardware.
"""

import os
import shutil
import threading

import pytest

from orca_core import MockOrcaHand
from orca_core.constants import CURRENT_BASED_POSITION, MULTI_TURN_POSITION, POSITION, WRIST
from orca_core.hardware.mock_feetech_client import MockFeetechClient
from orca_core.utils import update_yaml

MODEL_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "orca_core", "models", "v2", "orcahand-right",
)


@pytest.fixture
def feetech_hand(tmp_path):
    """Connected MockOrcaHand whose config declares Feetech motors."""
    config_path = tmp_path / "config.yaml"
    shutil.copy(os.path.join(MODEL_DIR, "config.yaml"), config_path)
    (tmp_path / "calibration.yaml").write_text("{}\n", encoding="utf-8")
    update_yaml(str(config_path), "motor_type", "feetech")

    hand = MockOrcaHand(config_path=str(config_path))
    success, msg = hand.connect()
    assert success, msg
    try:
        yield hand
    finally:
        hand.stop_task()
        hand.disconnect()


def test_feetech_config_gets_a_feetech_mock_client(feetech_hand):
    assert isinstance(feetech_hand.motor_client, MockFeetechClient)
    assert feetech_hand.config.motor_type == "feetech"


def test_dynamixel_stays_the_default_for_unpinned_configs(mock_config_dir):
    from orca_core.hardware.mock_dynamixel_client import MockDynamixelClient

    update_yaml(str(mock_config_dir / "config.yaml"), "motor_type", None)
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    assert hand.connect()[0]
    try:
        assert isinstance(hand.motor_client, MockDynamixelClient)
    finally:
        hand.disconnect()


# ----- wrap offsets ---------------------------------------------------------

def test_bounded_client_gets_no_wrap_offsets(feetech_hand):
    """A single-turn motor cannot wrap, so a +/-2pi shift would only saturate
    it against an end stop for the rest of the session."""
    feetech_hand._compute_wrap_offsets_dict()
    assert set(feetech_hand._wrap_offsets_dict) == set(feetech_hand.config.motor_ids)
    assert set(feetech_hand._wrap_offsets_dict.values()) == {0.0}


def test_bounded_client_skips_the_out_of_bounds_read(feetech_hand, monkeypatch):
    """Out-of-limits on a bounded client is a fault, not a wrap: no read at all."""
    def _must_not_read(*args, **kwargs):
        raise AssertionError("wrap detection read positions on a bounded client")

    monkeypatch.setattr(
        type(feetech_hand), "_read_motor_pos_for_offsets", _must_not_read
    )
    feetech_hand._compute_wrap_offsets_dict()


def test_wrapping_client_still_detects_wraps(connected_mock_hand):
    import numpy as np

    hand = connected_mock_hand
    motor_id = hand.config.motor_ids[0]
    idx = hand.config.motor_id_to_idx_dict[motor_id]
    pos = np.zeros(len(hand.config.motor_ids))
    pos[idx] = hand.motor_limits_dict[motor_id][1] + np.pi
    monkey = lambda *a, **k: pos  # noqa: E731
    hand._read_motor_pos_for_offsets = monkey

    hand._compute_wrap_offsets_dict()
    assert hand._wrap_offsets_dict[motor_id] == pytest.approx(2 * np.pi)


# ----- control modes --------------------------------------------------------

def test_wrist_keeps_requested_mode_without_multi_turn(feetech_hand, caplog):
    import logging

    wrist_motor = feetech_hand.config.joint_to_motor_map[WRIST]
    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand"):
        feetech_hand.set_control_mode(CURRENT_BASED_POSITION)

    modes = feetech_hand.motor_client._operating_mode
    assert len(set(modes.values())) == 1, "wrist diverged from the requested mode"
    assert modes[wrist_motor] == modes[feetech_hand.config.motor_ids[0]]
    assert any("multi-turn" in r.getMessage() for r in caplog.records)


def test_multi_turn_family_still_pins_the_wrist(initialized_mock_hand):
    from orca_core.constants import WRIST_MODE_VALUE

    hand = initialized_mock_hand
    hand.set_control_mode(CURRENT_BASED_POSITION)
    wrist_motor = hand.config.joint_to_motor_map[WRIST]
    assert hand.motor_client._operating_mode[wrist_motor] == WRIST_MODE_VALUE


def test_unsupported_mode_is_rejected_by_family(feetech_hand):
    with pytest.raises(ValueError, match="feetech"):
        feetech_hand.set_control_mode(MULTI_TURN_POSITION)


def test_supported_mode_is_accepted(feetech_hand):
    """Feetech collapses position control onto servo mode (0), as the real client does."""
    feetech_hand.set_control_mode(POSITION)
    assert set(feetech_hand.motor_client._operating_mode.values()) == {0}


# ----- motion waits ---------------------------------------------------------

class _WaitRecorder:
    """Records the order of wait_for_motion and set_control_mode calls."""

    def __init__(self, hand):
        self.calls = []
        self.hand = hand
        hand.motor_client.waits_for_motion = True
        hand.wait_for_motion = lambda timeout=5.0: self.calls.append("wait")
        original = hand.set_control_mode
        hand.set_control_mode = lambda mode, motor_ids=None: (
            self.calls.append(f"mode:{mode}"), original(mode, motor_ids)
        )[1]


def test_init_joints_waits_before_the_mode_switch(feetech_hand):
    recorder = _WaitRecorder(feetech_hand)
    feetech_hand.init_joints(force_calibrate=False)
    assert "wait" in recorder.calls
    assert recorder.calls.index("wait") < len(recorder.calls) - 1
    assert recorder.calls[-1].startswith("mode:")


def test_set_neutral_position_waits_before_the_mode_switch(feetech_hand):
    feetech_hand.init_joints(move_to_neutral=False)
    recorder = _WaitRecorder(feetech_hand)
    feetech_hand.set_neutral_position(num_steps=2)
    assert recorder.calls[0] == f"mode:{POSITION}"
    assert recorder.calls[1] == "wait"
    assert recorder.calls[2].startswith("mode:")


def _lock_is_free(lock) -> bool:
    """Whether another thread can take ``lock`` right now."""
    if lock.acquire(timeout=0.5):
        lock.release()
        return True
    return False


def test_wait_for_motion_does_not_hold_the_motor_lock(feetech_hand):
    """Holding it would stall the 100 Hz joint loop for the whole timeout."""
    free = []

    def _probe_lock_from_other_thread(timeout=5.0):
        # The motor lock is re-entrant, so it must be probed off this thread.
        probe = threading.Thread(
            target=lambda: free.append(_lock_is_free(feetech_hand._motor_lock))
        )
        probe.start()
        probe.join()

    feetech_hand.motor_client.waits_for_motion = True
    feetech_hand.motor_client.wait_for_motion_complete = _probe_lock_from_other_thread
    feetech_hand.wait_for_motion()
    assert free == [True]


# ----- last_read_ok ---------------------------------------------------------

def test_last_read_ok_forwards_the_client_flag(feetech_hand, monkeypatch):
    assert feetech_hand.last_read_ok is True
    monkeypatch.setattr(
        type(feetech_hand.motor_client),
        "last_read_ok",
        property(lambda self: False),
    )
    assert feetech_hand.last_read_ok is False


# ----- motor-chain planning -------------------------------------------------

_CHAIN_CFG = {"motor_ids": list(range(1, 18)), "joint_to_motor_map": {"wrist": 1}}


@pytest.mark.parametrize("motor_type", ["dynamixel", "feetech"])
def test_unpinned_baudrate_comes_from_the_motor_family(motor_type):
    from orca_core.constants import MOTOR_BAUD_RATES
    from orca_core.maintenance import motor_chain

    plan = motor_chain.plan_motor_chain(_CHAIN_CFG, "/dev/fake", motor_type)
    assert plan.target_baud == MOTOR_BAUD_RATES[motor_type][0]


def test_a_baudrate_the_family_rejects_is_refused():
    from orca_core.maintenance import motor_chain

    cfg = {**_CHAIN_CFG, "baudrate": 3_000_000}
    with pytest.raises(motor_chain.MotorChainError, match="invalid baud rate"):
        motor_chain.plan_motor_chain(cfg, "/dev/fake", "feetech")


def test_an_unrecognised_model_number_is_reported_not_refused(monkeypatch):
    """A new production batch of the same servo answers with the client's
    generic family label; polling forever on it would strand the build."""
    from orca_core.maintenance import motor_chain

    plan = motor_chain.plan_motor_chain(_CHAIN_CFG, "/dev/fake", "feetech")
    found = {"id": 1, "model_name": "Feetech", "model_number": 4321, "baud_rate": 1_000_000}
    monkeypatch.setattr(motor_chain, "scan_motors", lambda *a, **k: [found])

    events = []
    assert motor_chain.find_default_motor(plan, "HLS3915", events.append) is found
    assert events[0]["event"] == "unrecognised_motor_model"
    assert events[0]["model_number"] == 4321


def test_a_known_wrong_model_is_still_refused(monkeypatch):
    from orca_core.maintenance import motor_chain

    plan = motor_chain.plan_motor_chain(_CHAIN_CFG, "/dev/fake", "feetech")
    monkeypatch.setattr(
        motor_chain, "scan_motors",
        lambda *a, **k: [{"id": 1, "model_name": "HLS3930", "baud_rate": 1_000_000}],
    )
    with pytest.raises(motor_chain.MotorChainError, match="wrong motor type"):
        motor_chain.find_default_motor(plan, "HLS3915")


def test_no_expected_model_skips_the_check(monkeypatch):
    from orca_core.maintenance import motor_chain

    plan = motor_chain.plan_motor_chain(_CHAIN_CFG, "/dev/fake", "feetech")
    found = {"id": 1, "model_name": "HLS3930", "baud_rate": 1_000_000}
    monkeypatch.setattr(motor_chain, "scan_motors", lambda *a, **k: [found])
    assert motor_chain.find_default_motor(plan, None) is found
