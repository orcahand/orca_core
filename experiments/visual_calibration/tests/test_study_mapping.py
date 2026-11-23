"""hand_study.mapping: orca_core's linear map from an archived run, its ratios from orca_core."""

import math

import pytest
import yaml
from orca_core.hand_config import OrcaHandConfig

from hand_study.mapping import HAND_MAP, LinearMap, hand_map

MODEL = "orcahand-right"


def _info(folder="/x/hands/ser-0001", source="model", roms=None):
    """orca-ui's /api/hand/info for a right hand: the packaged map, with `roms` in force."""
    c = OrcaHandConfig.from_config_path(model_name=MODEL)
    return {"model_name": MODEL,
            "hand": {"id": "ser-0001", "folder": folder, "joint_ranges": {"source": source}},
            "joints": [{"id": j, "motor_id": c.joint_to_motor_map[j], "inverted": c.joint_inversion_dict[j],
                        "rom": (roms or {}).get(j, c.joint_roms_dict[j])} for j in c.joint_ids]}


def _limits():
    return {m: [0.1 * m, 0.1 * m + 3.0] for m in range(1, 18)}


def _archive(path, roms=None, **extra):
    path.mkdir(parents=True, exist_ok=True)
    (path / "calibration.yaml").write_text(yaml.safe_dump({"schema": 1, "motor_limits": _limits(), **extra}))
    if roms:
        (path / "joint_ranges.yaml").write_text(yaml.safe_dump({"schema": 1, "joint_roms": roms}))
    (path / HAND_MAP).write_text(yaml.safe_dump(hand_map(_info(source="measured" if roms else "model", roms=roms))))
    return path


def test_hand_map_from_orca_ui():
    m = hand_map(_info(source="measured"))
    assert m["hand"] == "ser-0001" and m["model"] == MODEL and m["joint_ranges"] == "measured"
    assert m["joints"]["ring_mcp"]["motor"] == 7 and m["joints"]["ring_mcp"]["inverted"] is True


def test_ratios_come_from_orca_core_and_follow_the_measured_ranges(tmp_path):
    """orca_core 0.14 stores no ratios: the motor's travel over the range in force."""
    L = LinearMap(_archive(tmp_path / "a"))
    lo, hi = L.rom["index_mcp"]
    assert L.ratio[3] == pytest.approx(3.0 / (hi - lo))
    assert L.to_joint("index_mcp", 0.3) == pytest.approx(lo) and L.to_joint("index_mcp", 3.3) == pytest.approx(hi)
    assert L.to_joint("ring_mcp", 0.7) == pytest.approx(L.rom["ring_mcp"][1])    # inverted: the lower limit is the top
    assert L.to_motor("index_mcp", L.to_joint("index_mcp", 2.2)) == pytest.approx(2.2)
    M = LinearMap(_archive(tmp_path / "b", roms={"index_mcp": [-10.0, 90.0]}))
    assert M.rom["index_mcp"] == (-10.0, 90.0) and M.ratio[3] == pytest.approx(3.0 / 100.0)
    assert M.to_joint("index_mcp", 3.3) == pytest.approx(90.0) and M.ratio[7] == pytest.approx(L.ratio[7])


def test_an_archive_orca_core_would_map_otherwise_is_refused(tmp_path):
    """The joint ranges the hand ran with, but no joint_ranges.yaml archived: the frame is unknown."""
    d = _archive(tmp_path / "a", roms={"index_mcp": [-10.0, 90.0]})
    (d / "joint_ranges.yaml").unlink()
    with pytest.raises(ValueError, match="index_mcp"):
        LinearMap(d)


def test_an_old_archive_s_ratios_come_from_orca_core_too(tmp_path):
    """A run archived before 0.14 (full config.yaml): the ratios it stored are not read."""
    (tmp_path / "calibration.yaml").write_text(yaml.safe_dump({"motor_limits": {3: [1.0, 4.0], 7: [-2.0, 0.5]},
                                                               "joint_to_motor_ratios": {3: 0.5, 7: 0.5}}))
    (tmp_path / "config.yaml").write_text(yaml.safe_dump({
        "joint_ids": ["index_mcp", "ring_mcp"], "joint_to_motor_map": {"index_mcp": 3, "ring_mcp": -7},
        "joint_roms": {"index_mcp": [-25, 100], "ring_mcp": [-20, 105]}}))
    L = LinearMap(tmp_path)
    assert L.ratio == pytest.approx({3: 3.0 / 125, 7: 2.5 / 125}) and L.inverted == {"index_mcp": False, "ring_mcp": True}
    assert L.to_joint("index_mcp", 1.0 + 0.024 * 10) == pytest.approx(-15.0)


def test_a_packaged_config_copy_is_refused_clearly(tmp_path):
    """A 0.14 packaged config.yaml has no joint_roms (they come from shared.yaml)."""
    (tmp_path / "calibration.yaml").write_text(yaml.safe_dump({"motor_limits": _limits()}))
    (tmp_path / "config.yaml").write_text(yaml.safe_dump({"joint_ids": ["index_mcp"],
                                                          "joint_to_motor_map": {"index_mcp": 3}}))
    with pytest.raises(ValueError, match="hand_map.yaml"):
        LinearMap(tmp_path)


def test_unwrap_takes_a_turn_off():
    L = LinearMap.__new__(LinearMap)
    L.limits, L.motor = {3: (1.0, 4.0)}, {"index_mcp": 3}
    assert L.unwrap("index_mcp", 1.0 + 2 * math.pi) == pytest.approx(1.0)


def test_night_archives_the_hand_folder_and_checks_it(tmp_path, monkeypatch):
    """calibrate/snapshot on orca_core 0.14: the files come from the folder orca-ui names."""
    from hand_study import night
    folder = _archive(tmp_path / "hands" / "ser-0001", roms={"index_mcp": [-10.0, 90.0]})
    (folder / HAND_MAP).unlink()
    info = _info(folder=str(folder), source="measured", roms={"index_mcp": [-10.0, 90.0]})
    L = LinearMap.of_hand(folder, MODEL)
    motors = {"motors": [{"id": m, "position": 0.1 * m + 1.5} for m in range(1, 18)]}
    replies = {"/hand/info": info, "/motors/direct": motors}
    monkeypatch.setattr(night, "call", lambda path, *a, **k: replies[path])
    live = L.joints_of(list(range(1, 18)), [0.1 * m + 1.5 for m in range(1, 18)])
    assert live["index_mcp"] == pytest.approx(40.0)
    monkeypatch.setattr(night, "Hand", lambda stream: type("H", (), {"estimate": lambda self: dict(live)})())
    n = night.Night.__new__(night.Night)
    n.stream = None
    out = tmp_path / "calib_x"
    n.archive_hand(out)
    assert {p.name for p in out.iterdir()} == {"calibration.yaml", "joint_ranges.yaml", HAND_MAP}
    n.check_archive(out)                                   # matches the live estimate
    live["index_mcp"] = 43.0
    with pytest.raises(night.Abort, match="live estimate"):
        n.check_archive(out)


def _stored_hand(hid, cal_extra=None, roms=None):
    """A right hand filed under `hid`, calibrated (fingerprint of the packaged map), `roms` measured."""
    from orca_core import hand_store
    from orca_core.calibration import joint_fingerprints
    f = hand_store.hand_dir(hid)
    f.mkdir(parents=True)
    (f / "hand.yaml").write_text(yaml.safe_dump({"schema": 1, "id": hid, "model": MODEL}))
    limits = {m: [-1.0 + 0.1 * m, 1.6 + 0.13 * m] for m in range(1, 18)}
    fingerprint = joint_fingerprints(OrcaHandConfig.from_config_path(model_name=MODEL))
    (f / "calibration.yaml").write_text(yaml.safe_dump({"schema": 1, "calibrated": True, "wrist_calibrated": True,
                                                        "motor_limits": limits, "fingerprint": fingerprint,
                                                        **(cal_extra or {})}))
    if roms:
        hand_store.write_joint_ranges(f, roms, source="test")
    return f, limits


@pytest.mark.parametrize("cal_extra, roms", [
    (None, None),
    (None, {"index_mcp": [-12.0, 95.0], "ring_pip": [0.0, 100.0]}),
    ({"joint_roms_measured": {"index_mcp": [-20.0, 80.0]}}, None),
    ({"joint_roms_measured": {"index_mcp": [-20.0, 80.0]}, "rom_frame": "centered"}, None),
    ({"fingerprint": {"index_mcp": "another mapping"}}, None),
], ids=["model", "measured ranges", "encoder span", "encoder span centered", "a joint's mapping changed"])
def test_the_map_is_orca_core_s_own(cal_extra, roms):
    """LinearMap gives what the hand itself reads from the same motor positions (OrcaHand's map)."""
    import numpy as np
    from orca_core.hardware_hand import MockOrcaHand
    folder, limits = _stored_hand("ser-0001", cal_extra, roms)
    if cal_extra and "fingerprint" in cal_extra:            # a fingerprint for every joint, one of them changed
        doc = yaml.safe_load((folder / "calibration.yaml").read_text())
        from orca_core.calibration import joint_fingerprints
        doc["fingerprint"] = {**joint_fingerprints(OrcaHandConfig.from_config_path(model_name=MODEL)),
                              "index_mcp": "another mapping"}
        (folder / "calibration.yaml").write_text(yaml.safe_dump(doc))
    hand = MockOrcaHand(model_name=MODEL)
    hand._use_stored_hand("ser-0001")
    hand._wrap_offsets_dict = {}
    L = LinearMap.of_hand(folder, MODEL)
    ids = hand.config.motor_ids
    for t in (0.0, 0.3, 0.9):
        pos = np.array([limits[m][0] + t * (limits[m][1] - limits[m][0]) for m in ids])
        runtime = {j: v for j, v in hand._motor_to_joint_pos(pos).items() if v is not None}
        mine = L.joints_of(ids, list(pos))
        assert set(mine) == set(runtime)
        assert max(abs(mine[j] - runtime[j]) for j in runtime) < 1e-9
