"""A hand measured on the rig: the run plans over the ranges the hand has, from a copy of its model."""
import pytest
import yaml
from orca_core import hand_store, load_config

from hand_study import run_hand

MODEL = "orcahand-right"


def test_the_run_plans_over_the_ranges_the_hand_has(tmp_path):
    folder = hand_store.hand_dir("ser-0001")
    folder.mkdir(parents=True)
    (folder / "hand.yaml").write_text(yaml.safe_dump({"schema": 1, "id": "ser-0001", "model": MODEL}))
    (folder / "joint_ranges.yaml").write_text(yaml.safe_dump({"schema": 1, "joint_roms": {"index_mcp": [-10.0, 90.0]}}))
    hand, model, roms = run_hand.the_hand("ser-0001")
    assert (hand, model, roms["index_mcp"]) == ("ser-0001", MODEL, [-10.0, 90.0])
    cfg = yaml.safe_load(run_hand.run_config(tmp_path, model, roms).read_text())
    packaged = load_config(model_name=MODEL)["joint_roms"]
    assert cfg["joint_roms"]["index_mcp"] == [-10.0, 90.0] and packaged["index_mcp"] != [-10.0, 90.0]
    assert cfg["joint_roms"]["wrist"] == packaged["wrist"] and cfg["joint_to_motor_map"]
    with pytest.raises(SystemExit, match="connect it once"):
        run_hand.the_hand("ser-0002")


def test_a_hand_whose_board_holds_its_ranges_is_planned_over_them():
    from orca_core import board_store
    from orca_core.board_store import BoardIdentity
    folder = hand_store.hand_dir("ser-0001")
    folder.mkdir(parents=True)
    (folder / "hand.yaml").write_text(yaml.safe_dump({"schema": 1, "id": "ser-0001", "model": MODEL}))
    (folder / hand_store.BOARD_STORE_FILE).write_bytes(board_store.pack(
        {"motor_limits": {1: [0.0, 1.0]}}, identity=BoardIdentity(side="right", serial="ser-0001"),
        joint_ranges={"joint_roms": {"index_mcp": [-12.5, 91.25]}}))
    assert run_hand.the_hand("ser-0001")[2]["index_mcp"] == [-12.5, 91.25]


def test_preflight_needs_the_rig_calibration_orca_ui_uses(monkeypatch):
    from hand_study import night
    from visual_calibration.rig_config import RigConfig
    rig = RigConfig.load(rig_id="rig-1")
    from hand_study import paths
    monkeypatch.setattr(paths, "run_info", lambda study=None: {"rig": "rig-1"})
    ui = {}
    monkeypatch.setattr(night, "call", lambda path, *a, **k: ui)
    n = night.Night.__new__(night.Night)
    with pytest.raises(night.Abort, match="no rig calibration"):
        n.check_rig()
    rig.save_calibration({"calibrated_at": "2026-11-20T10:00:00+00:00"})
    ui.update(calibration_path=rig.calibration_path, calibrated_at="2026-11-19T10:00:00+00:00")
    with pytest.raises(night.Abort, match="restart orca-ui"):
        n.check_rig()
    ui["calibrated_at"] = "2026-11-20T10:00:00+00:00"
    assert n.check_rig() == "rig calibrated 2026-11-20T10:00:00+00:00"


def test_the_run_reaches_every_process_of_it(tmp_path, orca_home):
    """The workers and the fit, started with the run's environment, work in the run's folder."""
    import subprocess
    import sys
    run = tmp_path / "run"
    cfg = run_hand.run_config(run, MODEL, {})
    env = run_hand.run_env(run, cfg)
    assert env["ORCA_HOME"] == str(orca_home) and env["STUDY_ROM_CONFIG"] == str(cfg)
    out = subprocess.run([sys.executable, "-c", "from hand_study import worker, hardstops, paths; "
                          "print(worker.STUDY, hardstops.OUT, paths.SHARED, sep='\\n')"],
                         cwd=run_hand.HERE, env=env, capture_output=True, text=True, check=True).stdout.split("\n")
    assert out[:2] == [str(run), str(run / "proposed")] and out[2].startswith(str(orca_home))
