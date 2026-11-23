"""The fitted hard stops recorded as the hand's joint_ranges.yaml on this computer."""
import pytest
from orca_core import hand_store

from hand_study import hardstops


def test_identifiable_stops_go_to_the_hand_s_joint_ranges(tmp_path, monkeypatch):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    with pytest.raises(ValueError, match="connect it once"):
        hardstops.hand_folder("ser-0001")
    hand_store.hand_dir("ser-0001").mkdir(parents=True)
    stops = {"index_mcp": {"new": [-12.0, 95.5], "identifiable": True},
             "index_pip": {"new": [-15.0, 107.0], "identifiable": False}}
    path = hardstops.record_ranges("ser-0001", stops, "camera rig, sessions s1, calibration run2")
    doc = hand_store.read_doc(path, "joint_ranges")
    assert path == hand_store.hand_dir("ser-0001") / "joint_ranges.yaml"
    assert doc["joint_roms"] == {"index_mcp": [-12.0, 95.5]} and doc["source"].startswith("camera rig")
    assert hardstops.record_ranges("ser-0001", {"index_pip": stops["index_pip"]}, "x") is None
