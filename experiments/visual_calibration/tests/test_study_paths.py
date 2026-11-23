"""The hand study's folders: a run's under the rig that measured the hand, else the shared one."""

import pytest

from hand_study import paths
from visual_calibration.rig_config import RigConfig


def test_a_run_is_filed_under_its_rig_and_hand(orca_home, monkeypatch):
    rig = RigConfig.load(rig_id="rig-1")
    run = paths.new_run(rig, "ser-0001", protocol="short")
    assert run.parent == orca_home / "rigs" / "rig-1" / "runs" / "ser-0001"
    assert len(run.name) == len("20261123-201500") and run.name[8] == "-"
    info = paths.run_info(run)
    assert (info["rig"], info["hand"], info["protocol"]) == ("rig-1", "ser-0001", "short")
    monkeypatch.setenv(paths.RUN_ENV, str(run))
    assert paths.study_dir() == run
    monkeypatch.delenv(paths.RUN_ENV)
    assert paths.study_dir() == paths.SHARED == paths.UI_STATE / "visual_calibration" / "hand_study"
    with pytest.raises(ValueError):
        paths.new_run(rig, "../ser-0001")


def test_two_runs_in_one_second_get_their_own_folders(orca_home):
    rig = RigConfig.load(rig_id="rig-1")
    runs = {paths.new_run(rig, "ser-0001") for _ in range(3)}
    assert len(runs) == 3 and all(paths.run_info(r)["hand"] == "ser-0001" for r in runs)
