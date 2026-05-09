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
    ENCODER_LSB_RAD,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware_hand import MockOrcaHand
from orca_core.utils import read_yaml, update_yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(REPO_ROOT, "orca_core", "models", "v2", "orcahand_right")
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

    assert not hand.calibrated, "Hand should not be marked as calibrated before calibration"

    hand.calibrate()

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
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=12345, anchor_angle_rad=0.0),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=4321, anchor_angle_rad=1.5),
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
        joint: JointEncoderCal(enc_at_anchor_count=0, anchor_angle_rad=0.0)
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


# ---------------------------------------------------------------------------
# OrcaHand._raw_to_joint_angle
# ---------------------------------------------------------------------------


def test_raw_to_joint_angle_at_anchor_returns_anchor_angle(calib_dir):
    import dataclasses as dc

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    encoder_cal = {
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=1000, anchor_angle_rad=0.25),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=8000, anchor_angle_rad=1.4),
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )

    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[JOINT_TO_ENCODER_SLOT["thumb_cmc"]] = 1000
    raw[JOINT_TO_ENCODER_SLOT["index_mcp"]] = 8000

    angles = hand._raw_to_joint_angle(raw)
    assert set(angles.keys()) == {"thumb_cmc", "index_mcp"}
    assert angles["thumb_cmc"] == pytest.approx(0.25, abs=1e-9)
    assert angles["index_mcp"] == pytest.approx(1.4, abs=1e-9)


def test_raw_to_joint_angle_polarity_and_wrap(calib_dir, monkeypatch):
    import dataclasses as dc

    from orca_core.hardware.sensing import constants as sensing_constants

    monkeypatch.setitem(sensing_constants.JOINT_ENCODER_POLARITY, "thumb_cmc", 1)
    monkeypatch.setitem(sensing_constants.JOINT_ENCODER_POLARITY, "index_mcp", -1)

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    encoder_cal = {
        "thumb_cmc": JointEncoderCal(enc_at_anchor_count=10, anchor_angle_rad=0.0),
        "index_mcp": JointEncoderCal(enc_at_anchor_count=10, anchor_angle_rad=0.0),
    }
    hand.calibration = dc.replace(
        hand.calibration, joint_encoder_calibration_dict=encoder_cal
    )

    raw = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    raw[JOINT_TO_ENCODER_SLOT["thumb_cmc"]] = 30  # +20 LSB, polarity +1
    raw[JOINT_TO_ENCODER_SLOT["index_mcp"]] = 16380  # wraps to -14 LSB, polarity -1

    angles = hand._raw_to_joint_angle(raw)
    assert angles["thumb_cmc"] == pytest.approx(20 * ENCODER_LSB_RAD, abs=1e-12)
    assert angles["index_mcp"] == pytest.approx(14 * ENCODER_LSB_RAD, abs=1e-12)


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
    hand.calibrate(joint_encoder_client=encoder)

    calib_path = calib_dir / "calibration.yaml"
    raw = read_yaml(str(calib_path))
    assert "joint_encoder_calibration" in raw

    encoder_backed = hand._encoder_backed_joints()
    assert set(raw["joint_encoder_calibration"].keys()) == set(encoder_backed)
    assert hand.is_calibrated(use_joint_feedback=True) is True

    sample_entry = next(iter(raw["joint_encoder_calibration"].values()))
    assert set(sample_entry.keys()) == {"enc_at_anchor_count", "anchor_angle_rad"}


def test_calibrate_with_joint_feedback_round_trips_through_raw_to_joint_angle(
    calib_dir,
):
    from tests._encoder_helpers import MockJointEncoderSource

    config_path = calib_dir / "config.yaml"
    _enable_joint_feedback(config_path)

    hand = MockOrcaHand(config_path=str(config_path))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    hand.calibrate(joint_encoder_client=encoder)

    reading = encoder.get_latest_encoder_reading()
    angles = hand._raw_to_joint_angle(reading.raw_counts)
    assert set(angles.keys()) == set(hand._encoder_backed_joints())
    for joint, angle in angles.items():
        rom_min, rom_max = hand.config.joint_roms_dict[joint]
        slack = 0.2  # mock motor quantisation slack
        assert rom_min - slack <= angle <= rom_max + slack, (
            f"{joint} angle {angle} outside ROM [{rom_min}, {rom_max}]"
        )


# ---------------------------------------------------------------------------
# joint_encoder_joints subset behaviour
# ---------------------------------------------------------------------------


def test_encoder_backed_joints_respects_config_subset(calib_dir):
    """Only joints in joint_encoder_joints (∩ motor map ∩ protocol slot map)
    are reported as encoder-backed."""
    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    expected = set(hand.config.joint_encoder_joints or [])
    assert set(hand._encoder_backed_joints()) == expected


def test_encoder_backed_joints_empty_when_field_unset(tmp_path):
    """Unset / null joint_encoder_joints yields an empty set."""
    src_config = os.path.join(MODEL_DIR, "config.yaml")
    config_path = tmp_path / "config.yaml"
    shutil.copy(src_config, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", None)
    hand = MockOrcaHand(config_path=str(config_path))
    assert hand._encoder_backed_joints() == []


def test_calibrate_writes_encoder_block_only_for_configured_subset(calib_dir):
    """Encoder pass runs only for joints listed in joint_encoder_joints."""
    from tests._encoder_helpers import MockJointEncoderSource

    hand = MockOrcaHand(config_path=str(calib_dir / "config.yaml"))
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map
    )
    hand.calibrate(joint_encoder_client=encoder)

    raw = read_yaml(str(calib_dir / "calibration.yaml"))
    assert set(raw["joint_encoder_calibration"].keys()) == set(
        hand.config.joint_encoder_joints
    )


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


def test_validator_rejects_wrist_in_joint_encoder_joints(tmp_path):
    from orca_core.hand_config import HandConfigValidationError

    src_config = os.path.join(MODEL_DIR, "config.yaml")
    config_path = tmp_path / "config.yaml"
    shutil.copy(src_config, config_path)
    update_yaml(str(config_path), "joint_encoder_joints", ["wrist"])

    with pytest.raises(HandConfigValidationError, match="wrist"):
        MockOrcaHand(config_path=str(config_path))
