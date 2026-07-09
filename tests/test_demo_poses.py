"""Tests for the packaged demo-pose data and its accessor."""

import pytest

from orca_core.demo_poses import DemoPoseError, DemoPoses, load_demo_poses
from orca_core import MockOrcaHand


def test_packaged_demos_load():
    demos = load_demo_poses()
    assert set(demos) == {"main", "abduction"}
    assert all(isinstance(d, DemoPoses) for d in demos.values())


def test_every_sequenced_pose_is_defined():
    for name, demo in load_demo_poses().items():
        assert demo.sequence, f"{name} has an empty sequence"
        for pose in demo.sequence:
            assert pose in demo.pose_fractions, f"{name}: {pose} undefined"


def test_fractions_are_within_unit_range():
    for name, demo in load_demo_poses().items():
        for pose, fractions in demo.pose_fractions.items():
            for joint, fraction in fractions.items():
                assert 0.0 <= fraction <= 1.0, f"{name}/{pose}/{joint} = {fraction}"


def test_returned_data_is_read_only():
    demo = load_demo_poses()["main"]
    with pytest.raises(TypeError):
        demo.pose_fractions["open_hand"]["wrist"] = 0.0


def test_poses_resolve_against_a_real_hand_config(mock_config_dir):
    """Every demo pose must turn into in-ROM joint angles on a real config."""
    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    for name, demo in load_demo_poses().items():
        for pose_name, fractions in demo.pose_fractions.items():
            pose = hand.pose_from_fractions(fractions)
            for joint, value in pose.as_dict().items():
                low, high = hand.config.joint_roms_dict[joint]
                assert low - 1e-9 <= value <= high + 1e-9, f"{name}/{pose_name}/{joint}"


def test_malformed_demo_data_raises(tmp_path, monkeypatch):
    import orca_core.demo_poses as module

    (tmp_path / "demo_poses.yaml").write_text(
        "broken:\n  sequence: [missing_pose]\n  poses:\n    real_pose: {wrist: 0.5}\n"
    )
    monkeypatch.setattr(module, "DATA_DIR", str(tmp_path))
    module.load_demo_poses.cache_clear()
    try:
        with pytest.raises(DemoPoseError, match="undefined poses"):
            module.load_demo_poses()
    finally:
        module.load_demo_poses.cache_clear()
