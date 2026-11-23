"""The fitted hard stops recorded as the hand's joint_ranges.yaml on this computer."""
import json

import numpy as np
import pytest
import yaml
from orca_core import hand_store

from hand_study import hardstops, paths
from hand_study.mapping import HAND_MAP, LinearMap
from visual_calibration.rig_config import RigConfig

MODEL = "orcahand-right"
LIMITS = {m: [-1.0 + 0.1 * m, 1.5 + 0.1 * m] for m in range(1, 18)}


def test_identifiable_stops_go_to_the_hand_s_joint_ranges(tmp_path, monkeypatch):
    monkeypatch.setenv(hand_store.ORCA_HOME_ENV, str(tmp_path))
    with pytest.raises(ValueError, match="connect it once"):
        hardstops.hand_folder("ser-0001")
    hand_store.hand_dir("ser-0001").mkdir(parents=True)
    stops = {"index_mcp": {"new": [-12.0, 95.5], "identifiable": True},
             "index_pip": {"new": [-15.0, 107.0], "identifiable": False}}
    rec = hardstops.record_ranges("ser-0001", stops, "camera rig, sessions s1, calibration run2")
    doc = hand_store.read_doc(rec["path"], "joint_ranges")
    assert rec["path"] == str(hand_store.hand_dir("ser-0001") / "joint_ranges.yaml") and rec["recorded"] == ["index_mcp"]
    assert doc["joint_roms"] == {"index_mcp": [-12.0, 95.5]} and doc["source"].startswith("camera rig")
    assert hardstops.record_ranges("ser-0001", {"index_pip": stops["index_pip"]}, "x")["path"] is None


def test_a_fit_that_is_not_a_range_is_left_out_and_the_file_says_what_it_lost(capsys):
    """lo >= hi would make orca_core refuse the whole file: that joint is left out with a warning.
    The file is replaced whole, and the joints it no longer holds are named."""
    hand_store.hand_dir("ser-0001").mkdir(parents=True)
    hardstops.record_ranges("ser-0001", {"index_mcp": {"new": [-12.0, 95.5], "identifiable": True},
                                         "ring_mcp": {"new": [-20.0, 100.0], "identifiable": True}}, "first")
    rec = hardstops.record_ranges("ser-0001", {"index_mcp": {"new": [-10.0, 90.0], "identifiable": True},
                                               "wrist": {"new": [30.0, 30.0], "identifiable": True}}, "second")
    assert rec["recorded"] == ["index_mcp"] and list(rec["rejected"]) == ["wrist"] and rec["gone"] == ["ring_mcp"]
    assert "wrist left out" in capsys.readouterr().err
    summary = hardstops.ranges_summary(rec)
    assert "left out: wrist" in summary and "no longer in the file (back to the model's): ring_mcp" in summary


def test_the_fitted_stops_become_the_hand_s_joint_ranges(orca_home, monkeypatch):
    """hardstops on a run archived from a 0.14 hand folder (no config.yaml): the identifiable
    stops go to the hand's joint_ranges.yaml, its source naming the rig and the run."""
    folder = hand_store.hand_dir("ser-0001")
    folder.mkdir(parents=True)
    (folder / "calibration.yaml").write_text(yaml.safe_dump({"schema": 1, "motor_limits": LIMITS}))
    run = paths.new_run(RigConfig.load(rig_id="rig-1"), "ser-0001")
    calib = run / "calib_q1"
    calib.mkdir()
    (calib / "calibration.yaml").write_text((folder / "calibration.yaml").read_text())
    L = LinearMap.of_hand(folder, MODEL)
    (calib / HAND_MAP).write_text(yaml.safe_dump({"hand": "ser-0001", "model": MODEL, "joints": {
        j: {"motor": L.motor[j], "inverted": L.inverted[j], "rom": list(L.rom[j])} for j in L.joints}}))
    session = run / "sessions" / "t-N07-short"
    ids = sorted(L.limits)
    joint_of = {L.motor[j]: j for j in L.joints}
    for k, t in enumerate(np.linspace(0.0, 1.0, 5)):
        pos = [L.limits[m][0] + t * (L.limits[m][1] - L.limits[m][0]) for m in ids]
        assumed = L.joints_of(ids, pos)
        lo, hi = -15.0, 95.0                                    # index_mcp's true stops
        measured = {**assumed, "index_mcp": lo + t * (hi - lo)}
        d = session / "configs" / f"p{k}"
        d.mkdir(parents=True)
        b = {"motor_ids": ids, "motor_pos": pos, "motor_joint": [joint_of[m] for m in ids], "current_ma": [0.0] * 17}
        (d / "meta.json").write_text(json.dumps({"q": assumed, "before": b, "after": b, "views": [], "calibration": "q1"}))
        (d / "est.json").write_text(json.dumps({"joints": measured, "assumed": assumed, "std": {}, "rms_px": 5.0,
                                                "view_rms_px": [], "angles": []}))
    (session / "plan.json").write_text(json.dumps({"calibration": "q1"}))
    monkeypatch.setattr(paths, "STUDY", run)
    monkeypatch.setattr(hardstops, "STUDY", run)
    monkeypatch.setattr(hardstops, "OUT", run / "proposed")
    out = hardstops.main([session], "q1", hand_id="ser-0001")
    doc = hand_store.read_doc(folder / "joint_ranges.yaml", "joint_ranges")
    assert doc["joint_roms"]["index_mcp"] == [-15.0, 95.0] and out["joint_ranges"]["path"] == str(folder / "joint_ranges.yaml")
    saved = json.loads((run / "proposed" / "hardstops.json").read_text())
    assert saved["joint_ranges"]["recorded"] == out["joint_ranges"]["recorded"]
    assert doc["source"].startswith(f"camera rig rig-1, run {run.name}, sessions t-N07-short, calibration q1")
    assert "joint_roms.index_mcp[0]: -22.75 -> -15" in (run / "proposed" / "config.diff.txt").read_text()
