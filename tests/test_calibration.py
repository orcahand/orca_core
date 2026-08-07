import os
import shutil

import numpy as np
import pytest
import yaml
from orca_core.calibration import (
    CalibrationResult,
    JointEncoderCal,
    joint_encoder_calibration_to_yaml,
)
from orca_core.constants import JOINT_ENCODER_CALIBRATION
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_NUM_JOINTS,
    ENCODER_LSB_DEG,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core import MockOrcaHand
from orca_core.utils import read_yaml, update_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right")
REAL_CONFIG = os.path.join(MODEL_DIR, "config.yaml")

EXPECTED_LIMITS = [-1.0, 1.0]


@pytest.fixture
def calib_dir(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return tmp_path


def check_calibrated(hand, calib_path):
    assert hand.calibrated, "Hand should be marked as calibrated"

    calib = read_yaml(str(calib_path))

    motor_limits_file = calib.get('motor_limits', {})
    motor_limits = hand.motor_limits_dict
    assert motor_limits == motor_limits_file, "Motor limits do not match between hand and calibration file"

    joint_to_motor_ratios_file = calib.get('joint_to_motor_ratios', {})
    joint_to_motor_ratios = hand.joint_to_motor_ratios_dict
    assert joint_to_motor_ratios == joint_to_motor_ratios_file, "Joint to motor ratios do not match between hand and calibration file"

    for mid, ratio in joint_to_motor_ratios.items():
        assert ratio != 0, f"Joint to motor ratio for motor {mid} should not be 0"
        assert motor_limits[mid][0] >= EXPECTED_LIMITS[0], f"Motor {mid} min limit is below expected"
        assert motor_limits[mid][1] <= EXPECTED_LIMITS[1], f"Motor {mid} max limit is above expected"


def test_calibration_yaml_missing(calib_dir):
    calib_path = calib_dir / "calibration.yaml"
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    assert hand.calibrated, "Synthesized mock calibration should count as calibrated in memory"
    assert not calib_path.exists(), "Construction must not create calibration.yaml"

    hand.calibrate(persist=True)

    assert calib_path.exists(), "calibration.yaml should be created"

    check_calibrated(hand, calib_path)


# ---------------------------------------------------------------------------
# JointEncoderCal + YAML round-trip
# ---------------------------------------------------------------------------


def test_calibration_result_empty_has_empty_encoder_dict():
    result = CalibrationResult.empty([1, 2, 3])
    assert result.joint_encoder_calibration_dict == {}


def test_joint_encoder_calibration_yaml_round_trip(tmp_path):
    calib_path = tmp_path / "calibration.yaml"
    calib_path.touch()
    cals = {
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=12345),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=4321),
    }

    update_yaml(
        str(calib_path),
        JOINT_ENCODER_CALIBRATION,
        joint_encoder_calibration_to_yaml(cals),
    )

    result = CalibrationResult.from_calibration_path(str(calib_path), motor_ids=[1, 2])
    assert result.joint_encoder_calibration_dict == cals


def test_calibration_result_loads_missing_encoder_block_as_empty(tmp_path):
    calib_path = tmp_path / "calibration.yaml"
    with open(calib_path, "w") as f:
        yaml.safe_dump({"calibrated": False}, f)

    result = CalibrationResult.from_calibration_path(str(calib_path), motor_ids=[1])
    assert result.joint_encoder_calibration_dict == {}


# ---------------------------------------------------------------------------
# is_calibrated(use_joint_feedback=...) extension
# ---------------------------------------------------------------------------


def _populate_motor_calibration(hand):
    """Fill in valid motor limits + ratios so the joint-encoder check is the
    only thing left to gate ``is_calibrated``."""
    import dataclasses as dc
    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    hand.calibration = dc.replace(
        hand.calibration,
        motor_limits_dict=motor_limits,
        joint_to_motor_ratios_dict=ratios,
        calibrated=True,
        wrist_calibrated=True,
    )


def test_is_calibrated_open_loop_ignores_encoder_block(calib_dir):
    """Explicit use_joint_feedback=False bypasses the encoder check even when
    the loaded config requests joint feedback."""
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    _populate_motor_calibration(hand)
    assert hand.is_calibrated(use_joint_feedback=False) is True


def test_is_calibrated_with_joint_feedback_requires_encoder_block(calib_dir):
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    _populate_motor_calibration(hand)

    assert hand.is_calibrated(use_joint_feedback=True) is False, (
        "Empty encoder block must fail the joint-feedback gate"
    )

    full_dict = {
        joint: JointEncoderCal(enc_at_anchor_count=0)
        for joint in hand._encoder_backed_joints()
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=full_dict
    )
    assert hand.is_calibrated(use_joint_feedback=True) is True

    partial_dict = dict(full_dict)
    partial_dict.pop(next(iter(partial_dict)))
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=partial_dict
    )
    assert hand.is_calibrated(use_joint_feedback=True) is False


def test_is_calibrated_joint_feedback_requires_wrist_anchor(calib_dir):
    """A calibration made before the wrist joined the loop (finger anchors
    only) reports uncalibrated — the signal to recalibrate and capture the
    wrist anchor."""
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    _populate_motor_calibration(hand)
    finger_dict = {
        joint: JointEncoderCal(enc_at_anchor_count=0)
        for joint in hand._encoder_backed_joints()
        if joint != "wrist"
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=finger_dict
    )
    assert hand.is_calibrated(use_joint_feedback=True) is False


# ---------------------------------------------------------------------------
# OrcaHand._raw_to_joint_angle
# ---------------------------------------------------------------------------


def test_raw_to_joint_angle_at_anchor_returns_anchor_angle(calib_dir):
    """raw == enc_at_anchor_count decodes to the joint's ROM upper (the
    pose the calibration sweep stalls the motor at)."""
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    encoder_cal = {
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=1000),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=8000),
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )

    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[JOINT_TO_ENCODER_SLOT["thumb_cmc"]] = 1000
    raw[JOINT_TO_ENCODER_SLOT["index_mcp"]] = 8000

    angles = hand._raw_to_joint_angle(raw)
    assert set(angles.keys()) == {"thumb_cmc", "index_mcp"}
    assert angles["thumb_cmc"] == pytest.approx(
        hand.config.joint_roms_dict["thumb_cmc"][1], abs=1e-9
    )
    assert angles["index_mcp"] == pytest.approx(
        hand.config.joint_roms_dict["index_mcp"][1], abs=1e-9
    )


def test_raw_to_joint_angle_polarity_and_wrap(calib_dir, monkeypatch):
    """Polarity flips the sign of the encoder delta; the 14-bit wrap is
    handled at the ±180° boundary."""
    import dataclasses as dc

    from orca_core.hardware.sensing import constants as sensing_constants

    monkeypatch.setitem(sensing_constants.JOINT_ENCODER_POLARITY, "thumb_cmc", 1)
    monkeypatch.setitem(sensing_constants.JOINT_ENCODER_POLARITY, "index_mcp", -1)

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    encoder_cal = {
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=10),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=10),
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )

    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[JOINT_TO_ENCODER_SLOT["thumb_cmc"]] = 30  # +20 LSB, polarity +1
    raw[JOINT_TO_ENCODER_SLOT["index_mcp"]] = 16380  # wraps to -14 LSB, polarity -1

    rom_thumb = hand.config.joint_roms_dict["thumb_cmc"][1]
    rom_index = hand.config.joint_roms_dict["index_mcp"][1]
    angles = hand._raw_to_joint_angle(raw)
    assert angles["thumb_cmc"] == pytest.approx(rom_thumb + 20 * ENCODER_LSB_DEG, abs=1e-12)
    assert angles["index_mcp"] == pytest.approx(rom_index + 14 * ENCODER_LSB_DEG, abs=1e-12)


def test_raw_to_joint_angle_empty_dict_returns_empty(calib_dir):
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    assert hand._raw_to_joint_angle(raw) == {}


def test_raw_to_joint_angle_wrong_shape_raises(calib_dir):
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    with pytest.raises(ValueError, match=str(AUTO_ENC_NUM_JOINTS)):
        hand._raw_to_joint_angle(np.zeros(AUTO_ENC_NUM_JOINTS - 1, dtype=np.uint16))


# ---------------------------------------------------------------------------
# Calibrate(joint_encoder_client=...) integration
# ---------------------------------------------------------------------------


def _enable_joint_feedback(config_path):
    """Append ``use_joint_feedback: true`` to a config.yaml in place."""
    update_yaml(str(config_path), "use_joint_feedback", True)


def test_calibrate_with_joint_feedback_persists_encoder_block(calib_dir):
    from tests._encoder_helpers import MockJointEncoderSource

    config_path = calib_dir / "config.yaml"
    _enable_joint_feedback(config_path)

    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    hand.calibrate(joint_encoder_client=encoder, persist=True)

    calib_path = calib_dir / "calibration.yaml"
    raw = read_yaml(str(calib_path))
    assert "joint_encoder_calibration" in raw

    encoder_backed = hand._encoder_backed_joints()
    assert set(raw["joint_encoder_calibration"].keys()) == set(encoder_backed)
    assert hand.is_calibrated(use_joint_feedback=True) is True

    sample_entry = next(iter(raw["joint_encoder_calibration"].values()))
    assert set(sample_entry.keys()) == {"enc_at_anchor_count"}


# ---------------------------------------------------------------------------
# joint_encoder_joints subset behaviour
# ---------------------------------------------------------------------------


def test_encoder_backed_joints_respects_config_subset(calib_dir):
    """An explicit joint_encoder_joints list (the bring-up/debug override)
    narrows encoder-backed joints to that subset (∩ motor map ∩ slot map)."""
    config_path = calib_dir / "config.yaml"
    update_yaml(str(config_path), "joint_encoder_joints", ["ring_mcp", "ring_pip"])
    hand = MockOrcaHand(config_path=str(config_path))
    assert set(hand._encoder_backed_joints()) == {"ring_mcp", "ring_pip"}


def test_encoder_backed_joints_all_selects_every_slotted_motor_joint(calib_dir):
    """The ``["all"]`` default expands to every joint that has both an encoder
    slot and a driving motor, wrist included."""
    from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT

    config_path = calib_dir / "config.yaml"
    update_yaml(str(config_path), "joint_encoder_joints", ["all"])
    hand = MockOrcaHand(config_path=str(config_path))
    expected = {
        j for j in JOINT_TO_ENCODER_SLOT if j in hand.config.joint_to_motor_map
    }
    assert set(hand._encoder_backed_joints()) == expected
    assert "wrist" in hand._encoder_backed_joints()


def test_encoder_backed_joints_empty_when_field_unset(tmp_path):
    """Unset / null joint_encoder_joints yields an empty set."""
    src_config = os.path.join(MODEL_DIR, "config.yaml")
    config_path = tmp_path / "config.yaml"
    shutil.copy(src_config, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", None)
    # Clearing the encoder hardware also clears the feedback opt-in; an
    # explicit use_joint_feedback: true with no encoders is now rejected.
    update_yaml(str(config_path), "use_joint_feedback", None)
    hand = MockOrcaHand(config_path=str(config_path))
    assert hand._encoder_backed_joints() == []


def test_calibrate_writes_encoder_block_only_for_configured_subset(calib_dir):
    """Encoder pass runs only for joints listed in joint_encoder_joints."""
    from tests._encoder_helpers import MockJointEncoderSource

    config_path = calib_dir / "config.yaml"
    update_yaml(str(config_path), "joint_encoder_joints", ["ring_mcp", "ring_pip"])
    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    hand.calibrate(joint_encoder_client=encoder, persist=True)

    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert set(raw["joint_encoder_calibration"].keys()) == {"ring_mcp", "ring_pip"}


# ---------------------------------------------------------------------------
# Partial calibration: joints=... filter and pending_limits invariant
# ---------------------------------------------------------------------------


def test_calibrate_with_joints_filter_only_touches_listed_joints(calib_dir):
    """calibrate(joints=[...]) writes motor_limits only for listed joints."""
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    pre_existing = {
        mid: [-0.7, 0.7] for mid in hand.config.motor_ids
    }
    hand.calibration = dc.replace(
        hand.calibration, motor_limits_dict=pre_existing
    )

    target_joint = "thumb_cmc"
    target_motor = hand.config.joint_to_motor_map[target_joint]
    hand.calibrate(joints=[target_joint])

    for motor_id, limits in hand.motor_limits_dict.items():
        if motor_id == target_motor:
            assert limits != pre_existing[motor_id], (
                f"target motor {motor_id} should have new limits"
            )
        else:
            assert limits == pre_existing[motor_id], (
                f"untouched motor {motor_id} lost its previous limits: {limits}"
            )


def test_calibrate_partial_single_direction_preserves_motor_limits(calib_dir):
    """A run that only completes one direction for a joint must NOT clobber
    that joint's existing both-directions motor_limits."""
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    target = "thumb_cmc"
    target_motor = hand.config.joint_to_motor_map[target]
    pre_existing = list(hand.motor_limits_dict.get(target_motor) or [None, None])
    if pre_existing[0] is None or pre_existing[1] is None:
        pre_existing = [-0.6, 0.6]
    motor_limits_pre = {
        mid: list(hand.motor_limits_dict.get(mid) or [None, None])
        for mid in hand.config.motor_ids
    }
    motor_limits_pre[target_motor] = list(pre_existing)
    hand.calibration = dc.replace(hand.calibration, motor_limits_dict=motor_limits_pre)

    seq = list(hand.config.calibration_sequence)
    flex_only = [s for s in seq if s["joints"] == {target: "flex"}]
    assert flex_only, "expected a flex-only step for the target joint"
    hand.config = dc.replace(hand.config, calibration_sequence=flex_only)

    hand.calibrate(joints=[target])

    assert hand.motor_limits_dict[target_motor] == pre_existing, (
        "single-direction partial run clobbered both-directions limits"
    )


# ---------------------------------------------------------------------------
# Encoder anchors survive failed samples and aborted runs
# ---------------------------------------------------------------------------


def _seed_anchor_block(calib_dir, anchors):
    """Persist a joint_encoder_calibration block before the hand is built."""
    update_yaml(
        str(calib_dir / "calibration.yaml"),
        JOINT_ENCODER_CALIBRATION,
        {j: {"enc_at_anchor_count": c} for j, c in anchors.items()},
    )


def test_failed_anchor_sample_keeps_previous_anchor(calib_dir, monkeypatch):
    """A failed encoder anchor sample must keep the joint's previously
    persisted anchor instead of deleting it."""
    from orca_core.hardware.joint_encoder_client import JointEncoderCalibrationError
    from orca_core.maintenance import calibration_routine

    config_path = calib_dir / "config.yaml"
    update_yaml(str(config_path), "joint_encoder_joints", ["ring_mcp", "ring_pip"])
    _seed_anchor_block(calib_dir, {"ring_mcp": 111, "ring_pip": 222})

    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()

    failing_slot = JOINT_TO_ENCODER_SLOT["ring_pip"]
    seen_slots = set()

    def fake_sample(client, *, slot, **kwargs):
        if slot == failing_slot:
            raise JointEncoderCalibrationError("no frames")
        # First sample (flex hardstop) 5000, second (extend) far away, so the
        # healthy slot passes the sweep-tracking guard.
        if slot in seen_slots:
            return 9000
        seen_slots.add(slot)
        return 5000

    monkeypatch.setattr(
        calibration_routine, "sample_anchor_count_from_client", fake_sample
    )

    events = []
    hand.calibrate(
        joint_encoder_client=object(), progress_callback=events.append, persist=True
    )

    block = read_yaml(str(calib_dir / "calibration.yaml"))["joint_encoder_calibration"]
    assert block["ring_pip"]["enc_at_anchor_count"] == 222, "previous anchor lost"
    assert block["ring_mcp"]["enc_at_anchor_count"] == 5000, "anchor not re-sampled"
    assert (
        hand.calibration.joint_encoder_calibration_dict["ring_pip"].enc_at_anchor_count
        == 222
    )
    assert any(
        e["event"] == "encoder_anchor_failed" and e["joint"] == "ring_pip"
        for e in events
    )


def test_aborted_run_preserves_previous_anchors_on_disk(calib_dir, monkeypatch):
    """Aborting after the first step must leave previously persisted anchors
    of not-yet-visited joints in calibration.yaml."""
    from orca_core.maintenance import calibration_routine

    config_path = calib_dir / "config.yaml"
    update_yaml(str(config_path), "joint_encoder_joints", ["ring_mcp", "ring_pip"])
    _seed_anchor_block(calib_dir, {"ring_mcp": 111, "ring_pip": 222})

    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    monkeypatch.setattr(
        calibration_routine,
        "sample_anchor_count_from_client",
        lambda client, *, slot, **kwargs: 5000,
    )

    events = []

    def stop_after_first_step(event):
        events.append(event)
        if event["event"] == "step_done":
            hand._task_stop_event.set()

    hand.calibrate(
        joint_encoder_client=object(),
        progress_callback=stop_after_first_step,
        persist=True,
    )

    kinds = [e["event"] for e in events]
    assert kinds[-1] == "calibration_aborted"
    assert kinds.count("step_done") == 1, "expected an abort right after step 1"
    block = read_yaml(str(calib_dir / "calibration.yaml"))["joint_encoder_calibration"]
    assert block == {
        "ring_mcp": {"enc_at_anchor_count": 111},
        "ring_pip": {"enc_at_anchor_count": 222},
    }


# ---------------------------------------------------------------------------
# Persistence: persist=False seam and full-document write
# ---------------------------------------------------------------------------


def test_run_calibration_persist_false_leaves_disk_untouched(calib_dir):
    from orca_core.maintenance.calibration_routine import run_calibration

    calib_path = calib_dir / "calibration.yaml"
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    before = read_yaml(str(calib_path))

    result = run_calibration(hand, persist=False)

    assert result is not None and result.calibrated
    assert hand.calibration.calibrated
    assert read_yaml(str(calib_path)) == before, "persist=False wrote to disk"


def test_per_step_persist_preserves_unrelated_keys(calib_dir):
    calib_path = calib_dir / "calibration.yaml"
    update_yaml(str(calib_path), "operator_note", "check tendon 3")

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(persist=True)

    raw = read_yaml(str(calib_path))
    assert raw["operator_note"] == "check tendon 3"
    assert raw["calibrated"] is True


# ---------------------------------------------------------------------------
# Events replace stdout reporting
# ---------------------------------------------------------------------------


def test_calibrate_reports_limits_via_events_not_stdout(calib_dir, capsys):
    """Every motor's captured limits arrive as limit_recorded events; the
    routine itself prints no operator-facing progress to stdout."""
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    capsys.readouterr()

    events = []
    hand.calibrate(progress_callback=events.append)

    bounds_by_motor = {}
    for e in events:
        if e["event"] == "limit_recorded":
            bounds_by_motor.setdefault(e["motor"], set()).add(e["bound"])
    assert set(bounds_by_motor) == set(hand.config.motor_ids)
    assert all(b == {"lower", "upper"} for b in bounds_by_motor.values())

    out = capsys.readouterr().out
    for fragment in ("reached the limit", "Joint calibrated", "Enabling torque"):
        assert fragment not in out


def test_second_calibrate_skips_wrist_with_event(calib_dir):
    """No encoder client → the anchor-needed override stays inert and a
    calibrated wrist is skipped as before."""
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(joints=["wrist"])
    assert hand.calibration.wrist_calibrated

    events = []
    hand.calibrate(joints=["thumb_cmc"], progress_callback=events.append)
    assert any(e["event"] == "wrist_skipped" for e in events)


def test_calibrate_records_wrist_anchor_with_event(calib_dir):
    """A full calibrate with the encoder pass active anchors the wrist at
    its FLEX hardstop and persists the anchor."""
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )

    events = []
    hand.calibrate(
        joint_encoder_client=encoder,
        progress_callback=events.append,
        persist=True,
    )

    wrist_events = [
        e
        for e in events
        if e["event"] == "encoder_anchor_recorded" and e["joint"] == "wrist"
    ]
    assert len(wrist_events) == 1
    assert wrist_events[0]["anchor_angle_deg"] == pytest.approx(
        hand.config.joint_roms_dict["wrist"][1]
    )
    anchor = hand.calibration.joint_encoder_calibration_dict["wrist"]
    assert anchor.enc_at_anchor_count != 0, (
        "anchor must come from the synthesized slot, not a dead default"
    )


def test_wrist_steps_rerun_when_anchor_missing(calib_dir):
    """A motor-calibrated wrist without an encoder anchor must not be
    skipped while the encoder pass is active: the steps re-run to capture
    the anchor."""
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(joints=["wrist"])
    assert hand.calibration.wrist_calibrated
    assert "wrist" not in hand.calibration.joint_encoder_calibration_dict

    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    events = []
    hand.calibrate(joint_encoder_client=encoder, progress_callback=events.append)

    assert not any(e["event"] == "wrist_skipped" for e in events)
    anchor = hand.calibration.joint_encoder_calibration_dict["wrist"]
    assert anchor.enc_at_anchor_count != 0


def test_subset_calibrate_emits_wrist_skipped_when_wrist_excluded(calib_dir):
    """A joints= restriction that excludes the wrist keeps the plain skip
    (with its event) even while the wrist anchor is missing."""
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(joints=["wrist"])
    assert "wrist" not in hand.calibration.joint_encoder_calibration_dict

    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    events = []
    hand.calibrate(
        joints=["thumb_cmc"],
        joint_encoder_client=encoder,
        progress_callback=events.append,
    )
    assert any(e["event"] == "wrist_skipped" for e in events)
    assert "wrist" not in hand.calibration.joint_encoder_calibration_dict


def test_dead_slot_gets_no_anchor_and_loses_a_stale_one(calib_dir):
    """A slot that reads a constant through the sweep passes the parity check
    but must not be anchored: the wrist (no prior anchor) stays unanchored,
    and a stale anchor on a dead finger slot is dropped, both with a failure
    event. Healthy slots anchor normally."""
    import dataclasses as dc

    from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibration = dc.replace(
        hand.calibration,
        joint_encoder_calibration_dict={
            "ring_pip": JointEncoderCal(enc_at_anchor_count=7)
        },
    )
    encoder = MockJointEncoderSource(
        hand._motor_client,
        hand.config.joint_to_motor_map,
        dead_slots={
            JOINT_TO_ENCODER_SLOT["wrist"], JOINT_TO_ENCODER_SLOT["ring_pip"]
        },
    )

    events = []
    hand.calibrate(
        joint_encoder_client=encoder,
        progress_callback=events.append,
        persist=True,
    )

    anchored = hand.calibration.joint_encoder_calibration_dict
    assert "wrist" not in anchored
    assert "ring_pip" not in anchored
    assert "ring_mcp" in anchored
    rejected = {
        e["joint"]
        for e in events
        if e["event"] == "encoder_anchor_failed" and "did not track" in e["error"]
    }
    assert rejected == {"wrist", "ring_pip"}
    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert "ring_pip" not in raw["joint_encoder_calibration"]


def test_wrist_skipped_when_anchor_already_present(calib_dir):
    """Once the anchor exists, a calibrated wrist is skipped again and the
    anchor is left untouched."""
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    hand.calibrate(joint_encoder_client=encoder)
    anchor = hand.calibration.joint_encoder_calibration_dict["wrist"]

    events = []
    hand.calibrate(
        joints=["thumb_cmc"],
        joint_encoder_client=encoder,
        progress_callback=events.append,
    )
    assert any(e["event"] == "wrist_skipped" for e in events)
    assert hand.calibration.joint_encoder_calibration_dict["wrist"] == anchor


# ---------------------------------------------------------------------------
# Validator
# ---------------------------------------------------------------------------


def test_validator_rejects_unknown_joint_in_joint_encoder_joints(tmp_path):
    from orca_core.hand_config import HandConfigValidationError

    src_config = os.path.join(MODEL_DIR, "config.yaml")
    config_path = tmp_path / "config.yaml"
    shutil.copy(src_config, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", ["not_a_joint"])

    with pytest.raises(HandConfigValidationError, match="not_a_joint"):
        MockOrcaHand(config_path=str(config_path))


def test_validator_accepts_wrist_in_joint_encoder_joints(tmp_path):
    """The wrist has an encoder slot and may be selected explicitly."""
    src_config = os.path.join(MODEL_DIR, "config.yaml")
    config_path = tmp_path / "config.yaml"
    shutil.copy(src_config, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", ["wrist"])

    hand = MockOrcaHand(config_path=str(config_path))
    assert hand._encoder_backed_joints() == ["wrist"]




# ---------------------------------------------------------------------------
# Progress callback + stop safety
# ---------------------------------------------------------------------------


def test_calibrate_emits_progress_events(calib_dir):
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    events = []
    hand.calibrate(progress_callback=events.append)

    kinds = [e["event"] for e in events]
    assert kinds[0] == "calibration_started"
    assert kinds[-1] == "calibration_done"
    assert kinds.count("step_done") == events[0]["steps"]
    ratios = {
        e["joint"]: e["ratio"] for e in events if e["event"] == "joint_calibrated"
    }
    assert ratios and all(r != 0.0 for r in ratios.values())


def test_calibrate_broken_progress_callback_does_not_abort(calib_dir):
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    def boom(_event):
        raise RuntimeError("bad callback")

    hand.calibrate(progress_callback=boom)
    assert hand.calibrated


def test_calibrate_stop_mid_drive_loop_persists_nothing(calib_dir):
    """A stop landing inside the hardstop-drive loop must abort BEFORE the
    encoder pass / limit capture samples — and persists — values taken at an
    arbitrary pose (the joints are not at their hardstops)."""
    calib_path = calib_dir / "calibration.yaml"
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()

    pre_limits = {
        mid: list(limits)
        for mid, limits in hand.calibration.motor_limits_dict.items()
    }
    pre_ratios = dict(hand.calibration.joint_to_motor_ratios_dict)
    pre_encoders = dict(hand.calibration.joint_encoder_calibration_dict)

    events = []
    orig_set_motor_pos = hand._set_motor_pos

    def set_and_stop(*args, **kwargs):
        # First drive-loop write: request a stop mid-drive.
        hand._task_stop_event.set()
        return orig_set_motor_pos(*args, **kwargs)

    hand._set_motor_pos = set_and_stop
    hand.calibrate(progress_callback=events.append, persist=True)

    kinds = [e["event"] for e in events]
    assert kinds[-1] == "calibration_aborted"
    assert "joint_calibrated" not in kinds
    assert "step_done" not in kinds
    assert hand.calibration.motor_limits_dict == pre_limits
    assert hand.calibration.joint_to_motor_ratios_dict == pre_ratios
    assert hand.calibration.joint_encoder_calibration_dict == pre_encoders
    assert not calib_path.exists(), "aborted mid-drive run must not write calibration"


# ---------------------------------------------------------------------------
# Encoder-measured joint ROMs
# ---------------------------------------------------------------------------


def _calibrate_with_measured_roms(calib_dir, span_scale=1.0, dead_slots=frozenset()):
    """Run a full mock calibration whose synthetic encoder measures each
    joint's configured ROM span scaled by ``span_scale``."""
    from tests._encoder_helpers import (
        MockJointEncoderSource,
        rom_consistent_counts_per_rad,
    )

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client,
        hand.config.joint_to_motor_map,
        counts_per_rad=rom_consistent_counts_per_rad(
            hand.config.joint_roms_dict, span_scale
        ),
        dead_slots=dead_slots,
    )
    events = []
    hand.calibrate(
        joint_encoder_client=encoder,
        progress_callback=events.append,
        persist=True,
    )
    return hand, events


def test_measured_rom_matches_config_when_travel_is_nominal(calib_dir):
    """A hand whose hardstops sit exactly at the configured ROM measures that
    ROM back, so the map is unchanged."""
    hand, _ = _calibrate_with_measured_roms(calib_dir)

    measured = hand.calibration.joint_roms_measured_dict
    assert set(measured) == set(hand.encoder_backed_joints)
    for joint, rom in measured.items():
        assert rom == pytest.approx(hand.config.joint_roms_dict[joint], abs=0.05)


def test_measured_rom_moves_only_the_lower_endpoint(calib_dir):
    """The anchor pose (flex hardstop) defines the encoder frame's upper
    endpoint, so a short-travelling joint moves its lower endpoint only."""
    hand, _ = _calibrate_with_measured_roms(calib_dir, span_scale=0.95)

    for joint, (lower, upper) in hand.calibration.joint_roms_measured_dict.items():
        config_lower, config_upper = hand.config.joint_roms_dict[joint]
        assert upper == pytest.approx(config_upper, abs=1e-9)
        expected_span = (config_upper - config_lower) * 0.95
        assert upper - lower == pytest.approx(expected_span, abs=0.05)
        assert lower > config_lower


def test_measured_rom_sets_the_ratio_and_the_map_frame(calib_dir):
    """The ratio is fitted against the measured span, so commanding a measured
    ROM endpoint lands on the corresponding motor limit."""
    hand, _ = _calibrate_with_measured_roms(calib_dir, span_scale=0.95)

    joint = "index_pip"
    motor_id = hand.config.joint_to_motor_map[joint]
    lower, upper = hand.calibration.joint_roms_measured_dict[joint]
    motor_lower, motor_upper = hand.motor_limits_dict[motor_id]

    ratio = hand.joint_to_motor_ratios_dict[motor_id]
    assert ratio == pytest.approx((motor_upper - motor_lower) / (upper - lower))

    idx = hand.config.motor_id_to_idx_dict[motor_id]
    inverted = hand.config.joint_inversion_dict.get(joint, False)
    at_lower = hand._joint_to_motor_pos({joint: lower})[idx]
    at_upper = hand._joint_to_motor_pos({joint: upper})[idx]
    assert at_lower == pytest.approx(motor_upper if inverted else motor_lower)
    assert at_upper == pytest.approx(motor_lower if inverted else motor_upper)


def test_measured_rom_beyond_tolerance_is_rejected(calib_dir):
    """A span implying a hardstop far off nominal is more likely a fault than
    a tolerance: the config ROM is kept and the ratio falls back to it."""
    hand, events = _calibrate_with_measured_roms(calib_dir, span_scale=0.5)

    assert hand.calibration.joint_roms_measured_dict == {}
    assert hand.effective_joint_roms_dict == hand.config.joint_roms_dict
    rejected = {e["joint"] for e in events if e["event"] == "measured_rom_rejected"}
    assert rejected == set(hand.encoder_backed_joints)

    joint = "index_pip"
    motor_id = hand.config.joint_to_motor_map[joint]
    config_lower, config_upper = hand.config.joint_roms_dict[joint]
    motor_lower, motor_upper = hand.motor_limits_dict[motor_id]
    assert hand.joint_to_motor_ratios_dict[motor_id] == pytest.approx(
        (motor_upper - motor_lower) / (config_upper - config_lower)
    )


def test_measured_rom_persists_and_reloads(calib_dir):
    """The measured ROMs survive a round trip through calibration.yaml."""
    hand, _ = _calibrate_with_measured_roms(calib_dir, span_scale=0.95)

    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert set(raw["joint_roms_measured"]) == set(
        hand.calibration.joint_roms_measured_dict
    )

    reloaded = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    assert (
        reloaded.calibration.joint_roms_measured_dict
        == hand.calibration.joint_roms_measured_dict
    )
    assert reloaded.effective_joint_roms_dict == hand.effective_joint_roms_dict


def test_measured_rom_dropped_for_a_slot_that_fails_the_sweep(calib_dir):
    """A dead slot yields no anchor, and no measured ROM from the same pair
    of samples either."""
    hand, _ = _calibrate_with_measured_roms(
        calib_dir, dead_slots={JOINT_TO_ENCODER_SLOT["ring_pip"]}
    )

    assert "ring_pip" not in hand.calibration.joint_roms_measured_dict
    assert "ring_mcp" in hand.calibration.joint_roms_measured_dict
    assert hand.effective_joint_roms_dict["ring_pip"] == (
        hand.config.joint_roms_dict["ring_pip"]
    )


def test_slot_going_dead_drops_its_stale_measured_rom(calib_dir):
    """A slot that measured a ROM once and later fails the sweep loses that
    ROM: the map must not keep trusting a span from an encoder now proven
    dead, the same way the stale anchor is dropped."""
    from tests._encoder_helpers import (
        MockJointEncoderSource,
        rom_consistent_counts_per_rad,
    )

    hand, _ = _calibrate_with_measured_roms(calib_dir, span_scale=0.95)
    assert "ring_pip" in hand.calibration.joint_roms_measured_dict

    encoder = MockJointEncoderSource(
        hand._motor_client,
        hand.config.joint_to_motor_map,
        counts_per_rad=rom_consistent_counts_per_rad(
            hand.config.joint_roms_dict, 0.95
        ),
        dead_slots={JOINT_TO_ENCODER_SLOT["ring_pip"]},
    )
    hand.calibrate(joint_encoder_client=encoder, persist=True)

    assert "ring_pip" not in hand.calibration.joint_roms_measured_dict
    assert "ring_pip" not in hand.calibration.joint_encoder_calibration_dict
    assert "ring_pip" not in read_yaml(str(calib_dir / "calibration.yaml"))[
        "joint_roms_measured"
    ]
    assert "middle_pip" in hand.calibration.joint_roms_measured_dict


def test_partial_recalibration_keeps_untouched_measured_roms(calib_dir):
    """Recalibrating a subset rewrites only those joints' measured ROMs."""
    from tests._encoder_helpers import (
        MockJointEncoderSource,
        rom_consistent_counts_per_rad,
    )

    hand, _ = _calibrate_with_measured_roms(calib_dir, span_scale=0.95)
    before = dict(hand.calibration.joint_roms_measured_dict)

    encoder = MockJointEncoderSource(
        hand._motor_client,
        hand.config.joint_to_motor_map,
        counts_per_rad=rom_consistent_counts_per_rad(
            hand.config.joint_roms_dict, 0.98
        ),
    )
    hand.calibrate(joint_encoder_client=encoder, joints=["index_pip"], persist=True)

    after = hand.calibration.joint_roms_measured_dict
    assert after["index_pip"] != before["index_pip"]
    assert after["middle_pip"] == before["middle_pip"]
    assert read_yaml(str(calib_dir / "calibration.yaml"))["joint_roms_measured"][
        "middle_pip"
    ] == pytest.approx(before["middle_pip"])


def test_open_loop_calibration_records_no_measured_roms(calib_dir):
    """Without an encoder client there is nothing to measure the span with, so
    the map stays on the config ROMs."""
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    hand.calibrate(persist=True)

    assert hand.calibration.joint_roms_measured_dict == {}
    assert hand.effective_joint_roms_dict == hand.config.joint_roms_dict
    assert "joint_roms_measured" not in read_yaml(
        str(calib_dir / "calibration.yaml")
    )


def test_calibration_result_drops_malformed_measured_rom_entries(tmp_path):
    path = tmp_path / "calibration.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "joint_roms_measured": {
                    "index_pip": [-12.0, 107.0],
                    "middle_pip": [107.0],
                    "ring_pip": "nonsense",
                }
            }
        )
    )
    result = CalibrationResult.from_calibration_path(str(path), motor_ids=[1])
    assert result.joint_roms_measured_dict == {"index_pip": [-12.0, 107.0]}
