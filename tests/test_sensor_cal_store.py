"""Tests for ``SensorCalStore``/``FileSensorCalStore`` and the connect-time
install of absolute sensor calibration on ``OrcaHandJointFeedback``.
"""

from __future__ import annotations

import os
import shutil

import pytest
import yaml

from orca_core.calibration import JointEncoderCal
from orca_core.sensor_cal_store import FileSensorCalStore

from tests._hand_feedback_helpers import make_calibrated_joint_feedback_hand

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


# ---------------------------------------------------------------------------
# FileSensorCalStore
# ---------------------------------------------------------------------------


def test_file_store_round_trip(tmp_path):
    path = tmp_path / "sensor_calibration.yaml"
    store = FileSensorCalStore(str(path))
    assert store.load() is None
    assert store.provenance == "file"

    cals = {
        "ring_mcp": JointEncoderCal(
            zero_count=123, zero_angle_deg=95.5, scale=1.02, source="fixture"
        ),
        "ring_pip": JointEncoderCal(zero_count=456, zero_angle_deg=100.0),
    }
    store.store(cals, hand_serial="OH-0042")
    assert store.load() == cals

    raw = yaml.safe_load(path.read_text())
    assert raw["version"] == 1
    assert raw["hand_serial"] == "OH-0042"


def test_file_store_empty_or_malformed_returns_none(tmp_path, capsys):
    path = tmp_path / "sensor_calibration.yaml"
    with open(path, "w") as f:
        yaml.safe_dump({"version": 1, "joints": {"ring_mcp": {"bogus": 1}}}, f)

    assert FileSensorCalStore(str(path)).load() is None
    assert "dropping unrecognized" in capsys.readouterr().out


def test_file_store_future_version_warns_but_reads(tmp_path, capsys):
    path = tmp_path / "sensor_calibration.yaml"
    with open(path, "w") as f:
        yaml.safe_dump(
            {
                "version": 99,
                "joints": {"ring_mcp": {"zero_count": 1, "zero_angle_deg": 90.0}},
            },
            f,
        )

    loaded = FileSensorCalStore(str(path)).load()
    assert loaded == {"ring_mcp": JointEncoderCal(zero_count=1, zero_angle_deg=90.0)}
    assert "version 99" in capsys.readouterr().out


# ---------------------------------------------------------------------------
# Connect-time install
# ---------------------------------------------------------------------------


@pytest.fixture
def feedback_config(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    return config_path


def test_connect_installs_file_sensor_cal_over_hardstop_entries(feedback_config, tmp_path):
    hand = make_calibrated_joint_feedback_hand(str(feedback_config))
    joints = hand._encoder_backed_joints()
    fixture_cal = JointEncoderCal(
        zero_count=777, zero_angle_deg=88.8, scale=0.99, source="fixture"
    )
    FileSensorCalStore(str(tmp_path / "sensor_calibration.yaml")).store(
        {joints[0]: fixture_cal}
    )

    hand.connect()
    try:
        assert hand.calibration.joint_encoder_calibration_dict[joints[0]] == fixture_cal
        assert hand.sensor_cal_provenance == "file"
        # Joints without a fixture entry keep their hardstop-anchored records.
        assert (
            hand.calibration.joint_encoder_calibration_dict[joints[1]].source
            == "hardstop"
        )
    finally:
        hand.disconnect()


def test_connect_without_file_keeps_calibration_yaml_provenance(feedback_config):
    hand = make_calibrated_joint_feedback_hand(str(feedback_config))
    hand.connect()
    try:
        assert hand.sensor_cal_provenance == "calibration.yaml"
    finally:
        hand.disconnect()
