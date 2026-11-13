"""Joint ranges measured on one hand (joint_ranges.yaml in its folder) are in
force for both the clamp and the calibration mapping of that hand.

OrcaHandConfig.with_measured_ranges lands with the shared-model-data change;
until both are merged these tests stand in a minimal version of it.
"""

import dataclasses
import os

import pytest

import orca_core
from orca_core import MockOrcaHand, hand_config, hand_store
from tests._helpers import copy_model_config

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-left", "config.yaml")
MEASURED = [-10.0, 80.0]


def _stand_in(self, roms):
    return dataclasses.replace(self, joint_roms_dict={**self.joint_roms_dict, **roms})


@pytest.fixture
def ranges_supported(monkeypatch):
    if not hasattr(hand_config.OrcaHandConfig, "with_measured_ranges"):
        monkeypatch.setattr(hand_config.OrcaHandConfig, "with_measured_ranges", _stand_in,
                            raising=False)


@pytest.fixture
def hand_with_ranges(tmp_path, ranges_supported):
    folder = tmp_path / "orcahand-left"
    folder.mkdir()
    copy_model_config(MODEL, folder / "config.yaml")
    mock_folder = hand_store.mock_hand_dir(hand_store.mock_hand_id("orcahand-left"))
    hand_store.write_doc(mock_folder / "joint_ranges.yaml", {
        "measured_at": "2026-11-01T10:00:00Z", "source": "camera rig",
        "joint_roms": {"index_mcp": MEASURED, "not_a_joint": [0, 1], "index_pip": [5, 1]},
    })
    return MockOrcaHand(config_path=str(folder / "config.yaml"))


def test_measured_ranges_are_in_force_and_said(hand_with_ranges):
    hand = hand_with_ranges
    assert hand.config.joint_roms_dict["index_mcp"] == MEASURED
    assert hand.joint_ranges_info == {
        "source": "measured", "measured_at": "2026-11-01T10:00:00Z", "joints": ["index_mcp"]}


def test_the_ratio_is_fitted_to_the_measured_range(hand_with_ranges):
    hand = hand_with_ranges
    motor = hand.config.joint_to_motor_map["index_mcp"]
    lo, hi = hand.motor_limits_dict[motor]
    assert hand.joint_to_motor_ratios_dict[motor] == pytest.approx((hi - lo) / (MEASURED[1] - MEASURED[0]))


def test_a_hardware_version_change_keeps_them(hand_with_ranges):
    hand = hand_with_ranges
    hand.set_hardware_version(211)
    assert hand.config.joint_roms_dict["index_mcp"] == MEASURED
    hand.set_hardware_version(None)
    assert hand.config.joint_roms_dict["index_mcp"] == MEASURED


def test_they_outrank_an_encoder_measured_span(hand_with_ranges):
    hand = hand_with_ranges
    hand.calibration = dataclasses.replace(
        hand.calibration, joint_roms_measured_dict={"index_mcp": [-40.0, 100.0]})
    assert hand.effective_joint_roms_dict["index_mcp"] == MEASURED


def test_ranges_taken_out_of_the_file_go_back_to_the_model_s(hand_with_ranges, tmp_path):
    hand = hand_with_ranges
    model = hand_config.OrcaHandConfig.from_config_path(config_path=MODEL).joint_roms_dict["index_mcp"]
    os.remove(os.path.join(hand.hand_dir, "joint_ranges.yaml"))
    hand._use_folder(hand.hand_dir, reload=True)
    assert hand.config.joint_roms_dict["index_mcp"] == model
    assert hand.joint_ranges_info["source"] == "model"


def test_another_hand_s_ranges_do_not_follow_into_this_one(hand_with_ranges, tmp_path):
    hand = hand_with_ranges
    model = hand_config.OrcaHandConfig.from_config_path(config_path=MODEL).joint_roms_dict["index_mcp"]
    other = tmp_path / "another-hand"
    other.mkdir()
    hand._use_folder(str(other))
    assert hand.config.joint_roms_dict["index_mcp"] == model


def test_a_changed_range_replaces_the_old_one(hand_with_ranges):
    hand = hand_with_ranges
    hand_store.write_doc(os.path.join(hand.hand_dir, "joint_ranges.yaml"),
                         {"joint_roms": {"index_pip": [0.0, 90.0]}})
    hand._use_folder(hand.hand_dir, reload=True)
    model = hand_config.OrcaHandConfig.from_config_path(config_path=MODEL).joint_roms_dict
    assert hand.config.joint_roms_dict["index_pip"] == [0.0, 90.0]
    assert hand.config.joint_roms_dict["index_mcp"] == model["index_mcp"]


def test_without_a_file_the_model_s_ranges_are_in_force(tmp_path):
    folder = tmp_path / "orcahand-left"
    folder.mkdir()
    copy_model_config(MODEL, folder / "config.yaml")
    hand = MockOrcaHand(config_path=str(folder / "config.yaml"))
    assert hand.joint_ranges_info == {"source": "model", "measured_at": None, "joints": []}
