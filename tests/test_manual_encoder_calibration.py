"""Manual joint-encoder calibration: re-anchoring one joint at an
operator-verified pose. Covers the pure inverse math, the hand-level method
(validation, calibration update, persistence), and the live hot-swap into a
running joint loop.
"""

from __future__ import annotations

import os
import shutil

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import (
    ENCODER_COUNTS_PER_REV,
    JOINT_ENCODER_POLARITY,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.encoder_protocol import (
    anchor_count_for_joint_angle,
    encoder_to_joint_angle,
)
from orca_core.utils.utils import read_yaml

from tests._encoder_helpers import with_even_parity
from tests._hand_feedback_helpers import make_calibrated_joint_feedback_hand


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def joint_feedback_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return str(config_path)


# ----- pure math -------------------------------------------------------------


@pytest.mark.parametrize("polarity", [1, -1])
@pytest.mark.parametrize("count", [0, 7, 8191, 16383])
@pytest.mark.parametrize("angle", [-30.0, 0.0, 47.3, 95.0])
def test_anchor_count_inverts_decode(polarity, count, angle):
    anchor_angle = 95.0
    anchor = anchor_count_for_joint_angle(count, polarity, angle, anchor_angle)
    assert 0 <= anchor < ENCODER_COUNTS_PER_REV
    decoded = encoder_to_joint_angle(
        np.array([count]), np.array([anchor]),
        np.array([polarity]), np.array([anchor_angle]),
    )[0]
    # Exact up to the rounding of the anchor to a whole count (one LSB).
    assert decoded == pytest.approx(angle, abs=360.0 / ENCODER_COUNTS_PER_REV)


# ----- hand-level method -----------------------------------------------------


def _connected_hand(config_path, joint="index_mcp", raw_count=1234):
    raw_counts = np.zeros(17, dtype=np.uint16)
    raw_counts[JOINT_TO_ENCODER_SLOT[joint]] = raw_count
    hand = make_calibrated_joint_feedback_hand(
        config_path, raw_counts=with_even_parity(raw_counts)
    )
    ok, msg = hand.connect(interactive=False)
    assert ok, msg
    return hand


def test_manual_calibration_reanchors_only_that_joint(joint_feedback_config):
    joint, raw_count, true_angle = "index_mcp", 1234, 30.0
    hand = _connected_hand(joint_feedback_config, joint, raw_count)
    try:
        before = dict(hand.calibration.joint_encoder_calibration_dict)
        anchor = hand.calibrate_joint_encoder_manual(joint, true_angle)

        expected = anchor_count_for_joint_angle(
            raw_count,
            JOINT_ENCODER_POLARITY[joint],
            true_angle,
            hand.config.joint_roms_dict[joint][1],
        )
        assert anchor == expected
        after = hand.calibration.joint_encoder_calibration_dict
        assert after[joint].enc_at_anchor_count == anchor
        for other in before:
            if other != joint:
                assert after[other] == before[other]

        # The decode path now reads the pumped counts as the true angle.
        frame = hand._encoder_client.get_latest_unfiltered()
        angles = hand._raw_to_joint_angle(frame.raw_counts)
        assert angles[joint] == pytest.approx(true_angle, abs=0.05)
    finally:
        hand.disconnect()


def test_manual_calibration_hot_swaps_running_loop(joint_feedback_config):
    joint, true_angle = "index_mcp", 30.0
    hand = _connected_hand(joint_feedback_config, joint, raw_count=1234)
    try:
        assert joint in hand.loop_joint_names
        hand.calibrate_joint_encoder_manual(joint, true_angle)
        # update_anchor rebases synchronously, so the loop's measured view is
        # already in the corrected frame.
        assert hand.get_measured_joints()[joint] == pytest.approx(
            true_angle, abs=0.05
        )
    finally:
        hand.disconnect()


def test_manual_calibration_persists_yaml(joint_feedback_config, tmp_path):
    joint = "index_mcp"
    hand = _connected_hand(joint_feedback_config, joint, raw_count=1234)
    try:
        anchor = hand.calibrate_joint_encoder_manual(joint, 30.0, persist=True)
        doc = read_yaml(hand.config.calibration_path)
        assert (
            doc["joint_encoder_calibration"][joint]["enc_at_anchor_count"]
            == anchor
        )
    finally:
        hand.disconnect()


def test_manual_calibration_rejects_bad_inputs(joint_feedback_config):
    hand = _connected_hand(joint_feedback_config)
    try:
        with pytest.raises(ValueError, match="not encoder-backed"):
            hand.calibrate_joint_encoder_manual("no_such_joint", 0.0)
        rom = hand.config.joint_roms_dict["index_mcp"]
        with pytest.raises(ValueError, match="outside the"):
            hand.calibrate_joint_encoder_manual("index_mcp", rom[1] + 20.0)
    finally:
        hand.disconnect()


def test_manual_calibration_requires_encoder_client(joint_feedback_config):
    raw_counts = np.zeros(17, dtype=np.uint16)
    hand = make_calibrated_joint_feedback_hand(
        joint_feedback_config, raw_counts=raw_counts
    )
    with pytest.raises(RuntimeError, match="no joint encoder client"):
        hand.calibrate_joint_encoder_manual("index_mcp", 0.0)


# ----- centered ROM frame ----------------------------------------------------


def test_centered_rom_frame_splits_delta_and_keeps_decode_consistent(
    joint_feedback_config,
):
    import dataclasses as dc

    joint, raw_count = "index_mcp", 1234
    hand = _connected_hand(joint_feedback_config, joint, raw_count)
    try:
        cl, cu = hand.config.joint_roms_dict[joint]
        # Pretend the sweep measured 4° more travel than the config nominal.
        hand.calibration = dc.replace(
            hand.calibration,
            joint_roms_measured_dict={joint: [cl - 4.0, cu]},
        )
        assert hand.rom_frame == "anchor"
        assert hand.effective_joint_roms_dict[joint] == [cl - 4.0, cu]

        hand.set_rom_frame("centered", persist=False)
        assert hand.effective_joint_roms_dict[joint] == pytest.approx(
            [cl - 2.0, cu + 2.0]
        )

        # Manual anchoring and decode share the frame: an anchor set at 30°
        # decodes back to 30° in the centered frame too.
        hand.calibrate_joint_encoder_manual(joint, 30.0)
        frame = hand._encoder_client.get_latest_unfiltered()
        assert hand._raw_to_joint_angle(frame.raw_counts)[joint] == pytest.approx(
            30.0, abs=0.05
        )

        with pytest.raises(ValueError, match="unknown rom frame"):
            hand.set_rom_frame("sideways")
    finally:
        hand.disconnect()


def test_rom_frame_persists_to_yaml(joint_feedback_config):
    from orca_core.utils.utils import read_yaml

    hand = _connected_hand(joint_feedback_config)
    try:
        hand.set_rom_frame("centered", persist=True)
        assert read_yaml(hand.config.calibration_path)["rom_frame"] == "centered"
    finally:
        hand.disconnect()

    rehydrated = make_calibrated_joint_feedback_hand(joint_feedback_config)
    assert rehydrated.rom_frame == "centered"
