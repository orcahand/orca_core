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
