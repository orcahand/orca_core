import dataclasses
import logging
import os
import shutil

import numpy as np
import pytest
from orca_core import MockOrcaHand, OrcaHand
from orca_core.calibration import JointEncoderCal
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS
from orca_core.utils import read_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand-right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")
JOINT_MODEL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def mock_hand():
    hand = MockOrcaHand(config_path=REAL_CONFIG)
    success, msg = hand.connect()
    assert success, f"Failed to connect: {msg}"
    yield hand
    hand.stop_task()
    hand.disconnect()


@pytest.fixture
def calibrated_hand(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect: {msg}"
    hand.calibrate()
    yield hand
    hand.stop_task()
    hand.disconnect()


def test_connect_disconnect_updates_connection_state(mock_hand):
    assert mock_hand.is_connected()
    success, _ = mock_hand.disconnect()
    assert success
    assert not mock_hand.is_connected()


def test_set_control_mode_preserves_wrist_special_case(mock_hand):
    wrist_motor_id = mock_hand.config.joint_to_motor_map.get("wrist")
    mock_hand.set_control_mode("current_based_position")
    for motor_id in mock_hand.config.motor_ids:
        expected_mode = 4 if motor_id == wrist_motor_id else 5
        assert mock_hand._motor_client._operating_mode[motor_id] == expected_mode


def test_set_max_current_supports_scalar_and_list(mock_hand):
    mock_hand.set_max_current(123.0)
    assert all(current == 123.0 for current in mock_hand._motor_client._cur.values())
    desired_currents = [float(idx) for idx in range(len(mock_hand.config.motor_ids))]
    mock_hand.set_max_current(desired_currents)
    for motor_id, desired in zip(mock_hand.config.motor_ids, desired_currents):
        assert mock_hand._motor_client._cur[motor_id] == desired


def test_disconnect_disables_torque_and_discards_client(mock_hand):
    client = mock_hand._motor_client
    mock_hand.disconnect()
    assert all(not enabled for enabled in client._torque_enabled.values())
    assert mock_hand._motor_client is None


def test_wait_for_motion_skips_the_bus_for_non_waiting_clients(mock_hand):
    """Clients that report no motion wait must not reach the motor lock."""
    calls = []
    mock_hand._motor_client.wait_for_motion_complete = lambda **kw: calls.append(kw)

    assert not mock_hand._motor_client.waits_for_motion
    mock_hand.wait_for_motion(timeout=0.1)
    assert calls == []


def test_wait_for_motion_delegates_for_waiting_clients(mock_hand):
    calls = []
    mock_hand._motor_client.wait_for_motion_complete = lambda **kw: calls.append(kw)
    mock_hand._motor_client.waits_for_motion = True

    mock_hand.wait_for_motion(timeout=0.1)
    assert calls == [{"timeout": 0.1}]


def test_init_joints_can_skip_neutral_move(calibrated_hand):
    before = np.copy(calibrated_hand.get_motor_pos())
    calibrated_hand.init_joints(move_to_neutral=False)
    np.testing.assert_allclose(calibrated_hand.get_motor_pos(), before)


def test_calibrated_joint_command_round_trips_through_motor_mapping(calibrated_hand):
    target = {}
    for joint in calibrated_hand.config.joint_ids[:3]:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        target[joint] = (min_pos + max_pos) / 2.0
    calibrated_hand.set_joint_positions(target)
    actual = calibrated_hand.get_joint_position().as_dict()
    for joint, expected in target.items():
        assert actual[joint] == pytest.approx(expected, abs=1e-5)


def test_joint_to_motor_pos_applies_the_wrap_offset(calibrated_hand):
    """A wrapped motor's command is shifted by its recorded offset.

    Without this the write path and the closed-loop path disagree by a full
    revolution on any motor whose encoder has wrapped.
    """
    joint = calibrated_hand.config.joint_ids[0]
    motor_id = calibrated_hand.config.joint_to_motor_map[joint]
    idx = calibrated_hand.config.motor_id_to_idx_dict[motor_id]
    target = {joint: sum(calibrated_hand.config.joint_roms_dict[joint]) / 2.0}

    calibrated_hand._wrap_offsets_dict[motor_id] = 0.0
    unwrapped = calibrated_hand._joint_to_motor_pos(target)[idx]

    calibrated_hand._wrap_offsets_dict[motor_id] = 2 * np.pi
    assert calibrated_hand._joint_to_motor_pos(target)[idx] == pytest.approx(
        unwrapped + 2 * np.pi
    )


def test_calibrated_list_command_round_trips(calibrated_hand):
    command = []
    for joint in calibrated_hand.config.joint_ids:
        min_pos, max_pos = calibrated_hand.config.joint_roms_dict[joint]
        command.append((min_pos + max_pos) / 4.0)
    calibrated_hand.set_joint_positions(np.array(command))
    actual = calibrated_hand.get_joint_position().as_array(calibrated_hand.config.joint_ids)
    np.testing.assert_allclose(actual, np.array(command), atol=1e-6)


# ---------------------------------------------------------------------------
# Construction side effects (_sanity_check must not create/rewrite files)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("hand_cls", [OrcaHand, MockOrcaHand])
def test_construction_leaves_config_dir_untouched(tmp_path, hand_cls):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand_cls(config_path=str(tmp_path / "config.yaml"))
    assert sorted(p.name for p in tmp_path.iterdir()) == ["config.yaml"]


def test_sanity_check_demotes_stale_calibrated_flag_in_existing_file(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    calib_path = tmp_path / "calibration.yaml"
    calib_path.write_text("calibrated: true\n", encoding="utf-8")

    hand = OrcaHand(config_path=str(tmp_path / "config.yaml"))

    assert not hand.calibrated
    assert read_yaml(str(calib_path))["calibrated"] is False


def test_sanity_check_does_not_rewrite_already_uncalibrated_file(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    calib_path = tmp_path / "calibration.yaml"
    original = "calibrated: false\noperator_note: keep me\n"
    calib_path.write_text(original, encoding="utf-8")

    OrcaHand(config_path=str(tmp_path / "config.yaml"))

    assert calib_path.read_text(encoding="utf-8") == original


def test_sanity_check_demote_write_is_gated_on_persist_calibration(tmp_path, monkeypatch):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    calib_path = tmp_path / "calibration.yaml"
    original = "calibrated: true\n"
    calib_path.write_text(original, encoding="utf-8")
    monkeypatch.setattr(OrcaHand, "_persist_calibration", False)

    hand = OrcaHand(config_path=str(tmp_path / "config.yaml"))

    assert not hand.calibrated, "stale flag must still be demoted in memory"
    assert calib_path.read_text(encoding="utf-8") == original


def test_mock_construction_never_rewrites_stale_calibration_file(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    calib_path = tmp_path / "calibration.yaml"
    original = "calibrated: true\n"
    calib_path.write_text(original, encoding="utf-8")

    MockOrcaHand(config_path=str(tmp_path / "config.yaml"))

    assert calib_path.read_text(encoding="utf-8") == original


# ---------------------------------------------------------------------------
# connect()/disconnect() idempotence and failure cleanup
# ---------------------------------------------------------------------------


def test_second_connect_is_noop_returning_success(mock_hand):
    first_client = mock_hand._motor_client
    success, msg = mock_hand.connect()
    assert success, msg
    assert mock_hand._motor_client is first_client, "repeat connect() must not open a second client"


def test_second_disconnect_succeeds_without_touching_the_bus(mock_hand):
    success, _ = mock_hand.disconnect()
    assert success

    # The client is discarded on disconnect, so a second disconnect has no
    # bus to touch and must still succeed.
    assert mock_hand._motor_client is None
    success, msg = mock_hand.disconnect()
    assert success, msg


def test_disconnect_with_raising_torque_off_still_closes_and_allows_reconnect(mock_hand):
    client = mock_hand._motor_client

    def boom(*args, **kwargs):
        raise OSError("serial link lost")

    client.set_torque_enabled = boom
    success, msg = mock_hand.disconnect()

    assert not success
    assert "torque disable failed" in msg
    assert not mock_hand.is_connected()
    assert mock_hand._motor_client is None
    assert not client.is_connected, "dead client left marked connected"

    success, msg = mock_hand.connect()
    assert success, msg
    assert mock_hand.is_connected()
    assert mock_hand._motor_client is not client, "reconnect reused the dead client"


def test_del_releases_hands_that_hold_no_motor_client(mock_hand):
    """Sensing variants can hold live links with the motor client unset (e.g.
    a sensors-only connect), so __del__ must always route through disconnect."""
    calls = []

    class _RecordingHand(MockOrcaHand):
        def disconnect(self):
            calls.append(True)
            return super().disconnect()

    hand = _RecordingHand(config_path=REAL_CONFIG)
    assert hand._motor_client is None
    hand.__del__()
    assert calls == [True], "__del__ skipped the teardown of a live hand"


def test_del_on_a_half_constructed_hand_does_not_raise():
    hand = OrcaHand.__new__(OrcaHand)
    hand.__del__()


def test_disconnect_reports_unacked_torque_disable_but_still_closes(mock_hand):
    client = mock_hand._motor_client
    client.set_torque_enabled = lambda *args, **kwargs: [5, 7]

    success, msg = mock_hand.disconnect()

    assert not success
    assert "[5, 7]" in msg
    assert not mock_hand.is_connected()
    assert mock_hand._motor_client is None


def test_torque_toggles_return_failed_ids_and_warn(mock_hand, caplog):
    assert mock_hand.enable_torque() == []
    assert mock_hand.disable_torque() == []

    def hand_warnings():
        return [
            record.getMessage()
            for record in caplog.records
            if record.name == "orca_core.hardware_hand"
            and "not acknowledged" in record.getMessage()
        ]

    valid_id = mock_hand.config.motor_ids[0]
    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand"):
        failed = mock_hand.enable_torque([valid_id, 99])
    assert failed == [99]
    assert any("99" in message for message in hand_warnings())

    caplog.clear()
    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand"):
        failed = mock_hand.disable_torque([valid_id, 99])
    assert failed == [99]
    assert any("99" in message for message in hand_warnings())


def test_failed_connect_after_client_open_closes_client(tmp_path, monkeypatch):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))

    created = []
    orig_create = hand._create_motor_client

    def create_and_record():
        client = orig_create()
        created.append(client)
        return client

    def failing_persist(existing):
        raise RuntimeError("post-connect failure")

    monkeypatch.setattr(hand, "_create_motor_client", create_and_record)
    monkeypatch.setattr(hand, "_persist_resolved_driver", failing_persist)
    monkeypatch.setattr(
        "orca_core.hardware_hand.auto_detect_port", lambda *a, **k: None
    )

    success, _ = hand.connect(interactive=False)

    assert not success
    assert hand._motor_client is None
    assert created and not created[0].is_connected, "connected client leaked open"


def test_failed_connect_logs_instead_of_printing(tmp_path, monkeypatch, capsys, caplog):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))

    def failing_persist(existing):
        raise RuntimeError("post-connect failure")

    monkeypatch.setattr(hand, "_persist_resolved_driver", failing_persist)
    monkeypatch.setattr(
        "orca_core.hardware_hand.auto_detect_port", lambda *a, **k: None
    )

    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand"):
        success, _ = hand.connect(interactive=False)

    assert not success
    assert any(
        "Connection failed on" in record.getMessage() for record in caplog.records
    )
    assert "Connection failed" not in capsys.readouterr().out


# ---------------------------------------------------------------------------
# Mock calibration: in-memory by default, coherent calibrated flag,
# synthesized limits inside the mock motors' travel
# ---------------------------------------------------------------------------


@pytest.fixture
def fresh_mock_dir(tmp_path):
    """Config-only model dir, as in a fresh checkout or wheel install."""
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    return tmp_path


def test_mock_calibrate_defaults_to_in_memory_only(fresh_mock_dir):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()
    hand.calibrate()
    assert hand.calibrated
    assert not (fresh_mock_dir / "calibration.yaml").exists(), (
        "mock calibrate() persisted synthesized values to disk"
    )


def test_mock_calibrate_persist_true_opts_into_disk_write(fresh_mock_dir):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(persist=True)
    raw = read_yaml(str(fresh_mock_dir / "calibration.yaml"))
    assert raw["calibrated"] is True


def test_init_joints_on_synthesized_mock_skips_calibration(fresh_mock_dir, monkeypatch):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()

    calls = []
    monkeypatch.setattr(hand, "calibrate", lambda *a, **k: calls.append(1))
    hand.init_joints()

    assert not calls, "init_joints() ran calibrate() on a fully synthesized mock"
    assert not (fresh_mock_dir / "calibration.yaml").exists()


def test_init_joints_force_calibrate_on_mock_leaves_disk_untouched(fresh_mock_dir):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()
    hand.init_joints(force_calibrate=True)
    assert hand.calibrated
    assert not (fresh_mock_dir / "calibration.yaml").exists()


def test_synthesized_mock_limits_fit_mock_motor_travel(fresh_mock_dir):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()
    low = hand._motor_client._min_pos
    high = hand._motor_client._max_motor_pos
    for motor_id, limits in hand.motor_limits_dict.items():
        assert low <= limits[0] < limits[1] <= high, (
            f"motor {motor_id} synthesized limits {limits} exceed mock travel"
        )


def test_widest_joint_rom_round_trips_on_synthesized_mock(fresh_mock_dir):
    hand = MockOrcaHand(config_path=str(fresh_mock_dir / "config.yaml"))
    hand.connect()
    roms = hand.config.joint_roms_dict
    joint = max(roms, key=lambda j: roms[j][1] - roms[j][0])
    for target in roms[joint]:
        hand.set_joint_positions({joint: float(target)})
        actual = hand.get_joint_position().as_dict()[joint]
        assert actual == pytest.approx(float(target), abs=1e-6), (
            f"{joint} saturated inside the mock plant at {target}"
        )


# ---------------------------------------------------------------------------
# Uncalibrated warn-once state and encoder decoding errors
# ---------------------------------------------------------------------------


def test_applying_a_calibration_rearms_uncalibrated_warnings(mock_hand, caplog):
    with caplog.at_level(logging.WARNING, logger="orca_core.hardware_hand"):
        mock_hand._warn_uncalibrated(3, "index_mcp", "motor limits")
        mock_hand._warn_uncalibrated(3, "index_mcp", "motor limits")
        assert len(caplog.records) == 1, "warn-once must not repeat"

        mock_hand.calibration = dataclasses.replace(mock_hand.calibration)
        mock_hand._warn_uncalibrated(3, "index_mcp", "motor limits")
        assert len(caplog.records) == 2, "new calibration must re-arm the warning"


def test_raw_to_joint_angle_without_validated_side_raises_actionable_error(tmp_path):
    shutil.copy(REAL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    hand.calibration = dataclasses.replace(
        hand.calibration,
        joint_encoder_calibration_dict={
            "index_mcp": JointEncoderCal(enc_at_anchor_count=100)
        },
    )
    hand.config = dataclasses.replace(hand.config, type=None)

    with pytest.raises(ValueError, match="polarity"):
        hand._raw_to_joint_angle(np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.int64))


# ---------------------------------------------------------------------------
# Background tasks
# ---------------------------------------------------------------------------


def test_stop_task_reports_and_logs_a_well_formed_message(mock_hand, caplog):
    def wait_for_stop():
        mock_hand._task_stop_event.wait(5.0)

    with caplog.at_level(logging.INFO, logger="orca_core.hardware_hand"):
        mock_hand._start_task(wait_for_stop)
        assert mock_hand.stop_task(timeout=5.0)
        assert not mock_hand.task_running

    # getMessage() raises when a record's args don't match its format string.
    assert any("task stopped" in record.getMessage() for record in caplog.records)


# ---------------------------------------------------------------------------
# Public accessors
# ---------------------------------------------------------------------------


def test_encoder_backed_joints_public_accessor(tmp_path):
    shutil.copy(JOINT_MODEL_CONFIG, tmp_path / "config.yaml")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    assert hand.encoder_backed_joints, "joint model should expose encoder-backed joints"
    assert hand.encoder_backed_joints == hand._encoder_backed_joints()
