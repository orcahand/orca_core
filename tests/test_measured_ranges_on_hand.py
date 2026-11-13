"""Ranges measured on one hand, put in force on a calibrated hand with
``hand.config = hand.config.with_measured_ranges(...)``: the derived
joint-to-motor ratio, the open-loop and the closed-loop maps all put the
measured range's ends exactly on the calibrated motor limits, whatever an
encoder-measured span, a hardware version or an old calibration file says."""

import dataclasses
import os

import numpy as np
import pytest
import yaml

import orca_core
from orca_core import MockOrcaHand, OrcaHandConfig, load_config
from orca_core.calibration import JointEncoderCal
from orca_core.control import JointController, JointLoopThread
from orca_core.hardware.sensing.constants import JOINT_TO_ENCODER_SLOT
from tests._helpers import copy_model_config

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-joint-right",
                     "config.yaml")
MEASURED = [-30.0, 110.0]
NARROWER = [-10.0, 90.0]
# Motor limits a calibration measured, in rad: off zero so a dropped lower limit shows.
LIMITS = [0.4, 2.6]


def _config(tmp_path, **overrides):
    path = tmp_path / "config.yaml"
    if overrides:
        raw = load_config(MODEL)
        raw.update(overrides)
        path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    else:
        copy_model_config(MODEL, path)
    return str(path)


def _calibrated_hand(tmp_path, **overrides):
    """A connected mock whose every motor was calibrated to ``LIMITS`` under the model's ranges."""
    hand = MockOrcaHand(config_path=_config(tmp_path, **overrides))
    hand.connect()
    roms = hand.config.joint_roms_dict
    hand.calibration = dataclasses.replace(
        hand.calibration,
        motor_limits_dict={mid: list(LIMITS) for mid in hand.config.motor_ids},
        joint_to_motor_ratios_dict={
            mid: (LIMITS[1] - LIMITS[0]) / (roms[j][1] - roms[j][0])
            for j, mid in hand.config.joint_to_motor_map.items()
        },
        calibrated=True,
        wrist_calibrated=True,
    )
    return hand


def _motor(hand, joint):
    return hand.config.joint_to_motor_map[joint]


def _ratio(hand, joint):
    return hand.calibration.joint_to_motor_ratios_dict[_motor(hand, joint)]


def _motor_at(hand, joint, angle):
    motor_id = _motor(hand, joint)
    return hand._joint_to_motor_pos({joint: angle})[hand.config.motor_id_to_idx_dict[motor_id]]


def _joint_at(hand, joint, motor_pos):
    motor_pos_all = [0.0] * len(hand.config.motor_ids)
    motor_pos_all[hand.config.motor_id_to_idx_dict[_motor(hand, joint)]] = motor_pos
    return hand._motor_to_joint_pos(motor_pos_all)[joint]


def _assert_ends_on_limits(hand, joint, rom):
    """``rom``'s ends land exactly on the motor limits and map back."""
    lo, hi = rom
    assert _ratio(hand, joint) == pytest.approx((LIMITS[1] - LIMITS[0]) / (hi - lo))
    flex_end, extend_end = (lo, hi) if hand.config.joint_inversion_dict[joint] else (hi, lo)
    if hand._wrap_offsets_dict is None:
        hand._compute_wrap_offsets_dict()
    offset = hand._wrap_offsets_dict.get(_motor(hand, joint), 0.0)
    assert _motor_at(hand, joint, flex_end) == pytest.approx(LIMITS[1] + offset)
    assert _motor_at(hand, joint, extend_end) == pytest.approx(LIMITS[0] + offset)
    assert _joint_at(hand, joint, LIMITS[1] + offset) == pytest.approx(flex_end)
    assert _joint_at(hand, joint, LIMITS[0] + offset) == pytest.approx(extend_end)


@pytest.mark.parametrize("joint", ["index_mcp", "ring_mcp", "wrist"])
def test_a_measured_range_lands_exactly_on_the_motor_limits(tmp_path, joint):
    hand = _calibrated_hand(tmp_path)
    assert hand.config.joint_inversion_dict[joint] is (joint != "index_mcp")
    assert hand.config.joint_roms_dict[joint] != MEASURED
    hand.config = hand.config.with_measured_ranges({joint: MEASURED})
    assert hand.config.joint_roms_dict[joint] == MEASURED
    assert hand.effective_joint_roms_dict[joint] == MEASURED
    _assert_ends_on_limits(hand, joint, MEASURED)


def test_a_narrower_measured_range_lands_on_the_limits_too(tmp_path):
    hand = _calibrated_hand(tmp_path)
    hand.config = hand.config.with_measured_ranges({"index_mcp": NARROWER})
    _assert_ends_on_limits(hand, "index_mcp", NARROWER)


def test_a_measured_range_wins_over_an_encoder_measured_span(tmp_path):
    hand = _calibrated_hand(tmp_path)
    encoder_span = [-20.0, 100.0]
    hand.calibration = dataclasses.replace(
        hand.calibration, joint_roms_measured_dict={"index_mcp": encoder_span})
    hand.config = hand.config.with_measured_ranges({"middle_mcp": MEASURED})
    assert hand.effective_joint_roms_dict["index_mcp"] == encoder_span, \
        "without a measured range, the encoder span is the frame"
    assert _ratio(hand, "index_mcp") == pytest.approx((LIMITS[1] - LIMITS[0]) / 120.0)

    hand.config = hand.config.with_measured_ranges({"index_mcp": MEASURED})
    assert hand.effective_joint_roms_dict["index_mcp"] == MEASURED
    _assert_ends_on_limits(hand, "index_mcp", MEASURED)


def test_the_joint_loop_puts_the_measured_ends_on_the_motor_limits(tmp_path):
    hand = _calibrated_hand(tmp_path)
    joints = [j for j in hand._encoder_backed_joints() if j in JOINT_TO_ENCODER_SLOT]
    hand.calibration = dataclasses.replace(
        hand.calibration,
        joint_encoder_calibration_dict={j: JointEncoderCal(enc_at_anchor_count=0) for j in joints})
    hand.config = hand.config.with_measured_ranges({"index_mcp": MEASURED})
    hand._compute_wrap_offsets_dict()
    loop = JointLoopThread(hand, None, JointController(num_joints=len(joints)))
    loop._snapshot_calibration()

    i = loop._joint_names.index("index_mcp")
    offset = hand._wrap_offsets_dict[_motor(hand, "index_mcp")]
    command = np.array([0.0] * len(joints))
    command[i] = MEASURED[1]
    assert loop._joint_to_motor_pos(command)[i] == pytest.approx(LIMITS[1] + offset)
    command[i] = MEASURED[0]
    assert loop._joint_to_motor_pos(command)[i] == pytest.approx(LIMITS[0] + offset)


def test_a_hardware_version_change_keeps_the_measured_range(tmp_path):
    v211 = {"index_mcp": [-1.0, 99.0], "index_pip": [-2.0, 98.0]}
    hand = _calibrated_hand(tmp_path, joint_roms_v211=v211)
    hand.config = hand.config.with_measured_ranges({"index_mcp": MEASURED})

    assert hand.set_hardware_version(211) == 211
    assert hand.config.joint_roms_dict["index_pip"] == v211["index_pip"], "v2.1.1's ranges apply"
    assert hand.config.joint_roms_dict["index_mcp"] == MEASURED
    _assert_ends_on_limits(hand, "index_mcp", MEASURED)

    hand.set_hardware_version(200)
    assert hand.config.joint_roms_dict["index_pip"] != v211["index_pip"]
    _assert_ends_on_limits(hand, "index_mcp", MEASURED)

    # The config alone carries them across versions, and never into its base.
    config = hand.config
    model_rom = load_config(MODEL)["joint_roms"]["index_mcp"]
    assert config.base_joint_roms_dict["index_mcp"] == model_rom
    assert config.with_hardware_version(211).joint_roms_dict["index_mcp"] == MEASURED
    assert config.without_hardware_version().joint_roms_dict["index_mcp"] == MEASURED
    fresh = OrcaHandConfig.from_config_path(config_path=MODEL).with_measured_ranges({"index_mcp": MEASURED})
    assert fresh.base_joint_roms_dict["index_mcp"] == model_rom
    assert dataclasses.replace(fresh, measured_joint_roms_dict={}).without_hardware_version() \
        .joint_roms_dict["index_mcp"] == model_rom


def test_an_old_calibration_file_re_derives_to_the_measured_span(tmp_path):
    """A 0.13 file (ratios stored, fitted to the model's 126.44 deg index_mcp
    span) named by path maps the measured range, not the stored ratio."""
    config_path = _config(tmp_path)
    model_rom = load_config(MODEL)["joint_roms"]["index_mcp"]
    assert model_rom[1] - model_rom[0] == pytest.approx(126.44)
    probe = MockOrcaHand(config_path=config_path)
    calibration = tmp_path / "calibration.yaml"
    roms = probe.config.joint_roms_dict
    calibration.write_text(yaml.safe_dump({
        "joint_to_motor_ratios": {
            mid: (LIMITS[1] - LIMITS[0]) / (roms[j][1] - roms[j][0])
            for j, mid in probe.config.joint_to_motor_map.items()},
        "motor_limits": {mid: list(LIMITS) for mid in probe.config.motor_ids},
        "calibrated": True,
        "wrist_calibrated": True,
    }), encoding="utf-8")

    hand = MockOrcaHand(config_path=config_path, calibration_path=str(calibration))
    hand.connect()
    assert _ratio(hand, "index_mcp") == pytest.approx((LIMITS[1] - LIMITS[0]) / 126.44)
    hand.config = hand.config.with_measured_ranges({"index_mcp": MEASURED})
    _assert_ends_on_limits(hand, "index_mcp", MEASURED)


def test_a_calibration_run_with_encoders_fits_the_measured_range(tmp_path):
    """The run's own ratio takes the measured range over the span the encoders
    measured, the same order the hand derives at load."""
    from tests._encoder_helpers import MockJointEncoderSource, rom_consistent_counts_per_rad

    hand = MockOrcaHand(config_path=_config(tmp_path))
    lo, hi = hand.config.joint_roms_dict["index_pip"]
    # Close enough to the model's for the encoder span to pass its tolerance check.
    measured = [lo - 0.5, hi + 0.5]
    hand.config = hand.config.with_measured_ranges({"index_pip": measured})
    hand.connect()
    encoder = MockJointEncoderSource(
        hand._motor_client, hand.config.joint_to_motor_map,
        counts_per_rad=rom_consistent_counts_per_rad(load_config(MODEL)["joint_roms"], 0.98))
    hand.calibrate(joint_encoder_client=encoder, persist=False)

    assert "index_pip" in hand.calibration.joint_roms_measured_dict
    motor_lower, motor_upper = hand.motor_limits_dict[_motor(hand, "index_pip")]
    expected = (motor_upper - motor_lower) / (measured[1] - measured[0])
    assert _ratio(hand, "index_pip") == pytest.approx(expected)
    hand._derive_calibration()
    assert _ratio(hand, "index_pip") == pytest.approx(expected)
