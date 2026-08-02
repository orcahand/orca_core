"""Failure-path tests for calibration routine internals: guarded motor reads,
offset-calibration failures, and the atomic YAML persistence helper."""

import os
import shutil
import threading

import numpy as np
import pytest
import yaml

from orca_core import MockOrcaHand
from orca_core.maintenance import calibration_routine
from orca_core.utils.utils import read_yaml, write_yaml_atomic

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand-right")


@pytest.fixture
def calib_dir(tmp_path):
    shutil.copy(os.path.join(MODEL_DIR, "config.yaml"), tmp_path / "config.yaml")
    return tmp_path


@pytest.fixture
def connected_hand(calib_dir):
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, msg
    try:
        yield hand
    finally:
        hand.disconnect()


# ---------------------------------------------------------------------------
# _read_motor_pos_checked
# ---------------------------------------------------------------------------


class _StubReadHand:
    """Hand stand-in whose bulk read reports stale for the first N attempts."""

    def __init__(self, failing_reads):
        self.reads = 0
        self._motor_lock = threading.RLock()
        stub = self

        class _Client:
            @property
            def last_read_ok(self):
                return stub.reads > failing_reads

        self.motor_client = _Client()

    def get_motor_pos(self):
        self.reads += 1
        return np.array([0.1, 0.2])


def test_read_motor_pos_checked_retries_until_fresh():
    hand = _StubReadHand(failing_reads=2)
    pos = calibration_routine._read_motor_pos_checked(hand, retries=5)
    assert hand.reads == 3
    np.testing.assert_array_equal(pos, np.array([0.1, 0.2]))


def test_read_motor_pos_checked_raises_on_persistent_stale_reads():
    hand = _StubReadHand(failing_reads=99)
    with pytest.raises(RuntimeError, match="no status packets"):
        calibration_routine._read_motor_pos_checked(hand, retries=3)
    assert hand.reads == 3


# ---------------------------------------------------------------------------
# calibrate_offset failure handling
# ---------------------------------------------------------------------------


def _force_offset_calibration(hand, monkeypatch, offset_stub):
    monkeypatch.setattr(
        type(hand.motor_client),
        "requires_offset_calibration",
        property(lambda self: True),
    )
    monkeypatch.setattr(hand.motor_client, "calibrate_offset", offset_stub)


def test_initial_offset_failure_skips_joint_and_keeps_limits(
    connected_hand, calib_dir, monkeypatch
):
    """A failed pre-drive offset calibration must not drive the joint nor
    persist limits for it."""
    import dataclasses as dc

    hand = connected_hand
    pre = {mid: [-0.7, 0.7] for mid in hand.config.motor_ids}
    ratios = {mid: 0.02 for mid in hand.config.motor_ids}
    hand.calibration = dc.replace(
        hand.calibration, motor_limits_dict=pre, joint_to_motor_ratios_dict=ratios
    )

    _force_offset_calibration(hand, monkeypatch, lambda motor_id, upper=True: False)

    target = "thumb_cmc"
    target_motor = hand.config.joint_to_motor_map[target]
    events = []
    hand.calibrate(joints=[target], progress_callback=events.append, persist=True)

    assert hand.motor_limits_dict[target_motor] == pre[target_motor]
    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert raw["motor_limits"][target_motor] == pre[target_motor]
    failures = [e for e in events if e["event"] == "offset_calibration_failed"]
    assert failures and all(e["motor"] == target_motor for e in failures)
    assert not any(e["event"] == "limit_recorded" for e in events)
    assert not any(e["event"] == "joint_calibrated" for e in events)


def test_wrist_offset_failure_keeps_wrist_uncalibrated_and_recoverable(
    connected_hand, calib_dir, monkeypatch
):
    """A wrist step skipped by a failed offset calibration must not persist
    wrist_calibrated, and a later run with working offsets must calibrate it."""
    import dataclasses as dc

    hand = connected_hand
    wrist_motor = hand.config.joint_to_motor_map["wrist"]
    limits = {mid: list(l) for mid, l in hand.calibration.motor_limits_dict.items()}
    limits[wrist_motor] = [None, None]
    hand.calibration = dc.replace(
        hand.calibration, motor_limits_dict=limits, calibrated=False
    )
    assert not hand.calibration.wrist_calibrated

    _force_offset_calibration(
        hand, monkeypatch, lambda motor_id, upper=True: motor_id != wrist_motor
    )

    events = []
    hand.calibrate(joints=["wrist"], progress_callback=events.append, persist=True)

    assert not hand.calibration.wrist_calibrated
    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert raw["wrist_calibrated"] is False
    assert raw["calibrated"] is False
    assert any(
        e["event"] == "offset_calibration_failed" and e["motor"] == wrist_motor
        for e in events
    )

    monkeypatch.setattr(
        hand.motor_client, "calibrate_offset", lambda motor_id, upper=True: True
    )
    events = []
    hand.calibrate(joints=["wrist"], progress_callback=events.append, persist=True)

    assert not any(e["event"] == "wrist_skipped" for e in events)
    assert hand.calibration.wrist_calibrated
    assert all(l is not None for l in hand.motor_limits_dict[wrist_motor])
    assert read_yaml(str(calib_dir / "calibration.yaml"))["wrist_calibrated"] is True


def test_stale_wrist_flag_without_limits_recalibrates(connected_hand, calib_dir):
    """A persisted wrist_calibrated flag with no stored wrist limits must not
    skip the wrist; the run recalibrates it and leaves a consistent state."""
    import dataclasses as dc

    hand = connected_hand
    wrist_motor = hand.config.joint_to_motor_map["wrist"]
    limits = {mid: list(l) for mid, l in hand.calibration.motor_limits_dict.items()}
    limits[wrist_motor] = [None, None]
    hand.calibration = dc.replace(
        hand.calibration,
        motor_limits_dict=limits,
        wrist_calibrated=True,
        calibrated=False,
    )

    events = []
    hand.calibrate(joints=["wrist"], progress_callback=events.append, persist=True)

    assert not any(e["event"] == "wrist_skipped" for e in events)
    assert hand.calibration.wrist_calibrated
    assert all(l is not None for l in hand.motor_limits_dict[wrist_motor])
    assert read_yaml(str(calib_dir / "calibration.yaml"))["wrist_calibrated"] is True


def test_final_offset_failure_records_no_limit(connected_hand, monkeypatch):
    """A failed post-release offset calibration must not record a limit taken
    in the un-shifted motor frame."""
    hand = connected_hand
    calls = {}

    def offset_stub(motor_id, upper=True):
        calls[motor_id] = calls.get(motor_id, 0) + 1
        return calls[motor_id] == 1

    _force_offset_calibration(hand, monkeypatch, offset_stub)

    target = "thumb_cmc"
    target_motor = hand.config.joint_to_motor_map[target]
    pre = list(hand.motor_limits_dict[target_motor])
    events = []
    hand.calibrate(joints=[target], progress_callback=events.append)

    assert hand.motor_limits_dict[target_motor] == pre
    assert any(e["event"] == "offset_calibration_failed" for e in events)
    assert not any(e["event"] == "limit_recorded" for e in events)
    assert not any(e["event"] == "joint_calibrated" for e in events)


# ---------------------------------------------------------------------------
# Torque-release failure handling
# ---------------------------------------------------------------------------


def test_failed_torque_release_skips_limit_capture(
    connected_hand, calib_dir, monkeypatch
):
    """A pre-capture torque release the bus rejected must not record a limit
    taken under tendon tension; the prior limit stays in place."""
    import dataclasses as dc

    hand = connected_hand
    pre = {mid: [-0.7, 0.7] for mid in hand.config.motor_ids}
    ratios = {mid: 0.02 for mid in hand.config.motor_ids}
    hand.calibration = dc.replace(
        hand.calibration, motor_limits_dict=pre, joint_to_motor_ratios_dict=ratios
    )

    target = "thumb_cmc"
    target_motor = hand.config.joint_to_motor_map[target]
    orig_disable = hand.disable_torque

    def failing_disable(motor_ids=None):
        result = orig_disable(motor_ids)
        if motor_ids == [target_motor]:
            return [target_motor]
        return result

    monkeypatch.setattr(hand, "disable_torque", failing_disable)

    events = []
    hand.calibrate(joints=[target], progress_callback=events.append, persist=True)

    assert hand.motor_limits_dict[target_motor] == pre[target_motor]
    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert raw["motor_limits"][target_motor] == pre[target_motor]
    failures = [e for e in events if e["event"] == "torque_release_failed"]
    assert failures and all(e["motor"] == target_motor for e in failures)
    assert not any(e["event"] == "limit_recorded" for e in events)
    assert not any(e["event"] == "joint_calibrated" for e in events)


# ---------------------------------------------------------------------------
# write_yaml_atomic
# ---------------------------------------------------------------------------


def test_write_yaml_atomic_round_trips_and_replaces(tmp_path):
    path = tmp_path / "calibration.yaml"
    write_yaml_atomic(str(path), {"calibrated": False})
    write_yaml_atomic(
        str(path), {"calibrated": True, "motor_limits": {1: [-1.0, 1.0]}}
    )
    assert read_yaml(str(path)) == {
        "calibrated": True,
        "motor_limits": {1: [-1.0, 1.0]},
    }
    assert [p.name for p in tmp_path.iterdir()] == ["calibration.yaml"]


def test_write_yaml_atomic_failed_write_keeps_original(tmp_path, monkeypatch):
    path = tmp_path / "calibration.yaml"
    write_yaml_atomic(str(path), {"calibrated": False})

    def boom(*args, **kwargs):
        raise RuntimeError("disk full")

    monkeypatch.setattr(yaml, "dump", boom)
    with pytest.raises(RuntimeError, match="disk full"):
        write_yaml_atomic(str(path), {"calibrated": True})

    assert read_yaml(str(path)) == {"calibrated": False}
    assert [p.name for p in tmp_path.iterdir()] == ["calibration.yaml"]


def test_write_yaml_atomic_preserves_permissions(tmp_path):
    path = tmp_path / "calibration.yaml"
    path.write_text("calibrated: false\n")
    os.chmod(path, 0o664)
    write_yaml_atomic(str(path), {"calibrated": True})
    assert os.stat(path).st_mode & 0o777 == 0o664


def test_write_yaml_atomic_preserves_symlink(tmp_path):
    target = tmp_path / "real_calibration.yaml"
    target.write_text("calibrated: false\n")
    link = tmp_path / "calibration.yaml"
    link.symlink_to(target)

    write_yaml_atomic(str(link), {"calibrated": True})

    assert link.is_symlink(), "symlink was replaced by a regular file"
    assert os.readlink(str(link)) == str(target), "symlink was repointed"
    assert read_yaml(str(target)) == {"calibrated": True}
    assert read_yaml(str(link)) == {"calibrated": True}
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        "calibration.yaml",
        "real_calibration.yaml",
    ]


def test_write_yaml_atomic_converts_numpy_arrays(tmp_path):
    path = tmp_path / "data.yaml"
    write_yaml_atomic(
        str(path),
        {"limits": np.array([1.0, 2.0]), "per_motor": {1: np.array([0.5, 0.6])}},
    )
    assert read_yaml(str(path)) == {
        "limits": [1.0, 2.0],
        "per_motor": {1: [0.5, 0.6]},
    }
