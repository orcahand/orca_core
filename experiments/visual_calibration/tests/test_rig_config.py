"""The camera rig's config.yaml, turntable.yaml, calibration.yaml (visual_calibration.rig_config)."""

import shutil

import numpy as np
import pytest
import yaml

from visual_calibration.rig_config import RigConfig, RigConfigError, get_rig_path


@pytest.fixture
def rig_copy(tmp_path):
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path)
    (path / "calibration.yaml").write_text("calibrated_at: null\nhistory: []\n")
    return path


def _edit(path, change):
    """Edit config.yaml, with turntable.yaml under its "turntable" key."""
    config = yaml.safe_load((path / "config.yaml").read_text())
    config["turntable"] = yaml.safe_load((path / "turntable.yaml").read_text())
    change(config)
    (path / "turntable.yaml").write_text(yaml.safe_dump(config.pop("turntable")))
    (path / "config.yaml").write_text(yaml.safe_dump(config))


def test_packaged_rig_loads():
    rig = RigConfig.load()
    assert (rig.version, rig.name) == ("v1", "camera-rig")
    assert rig.hard_stop.arm_sector == (125.0, 145.0)
    assert rig.travel == rig.hard_stop.travel == 320.0
    assert rig.stops.angles[0] == 0.0 and rig.stops.angles[-1] == 315.0


def test_wider_fin_costs_travel(rig_copy):
    _edit(rig_copy, lambda c: c["turntable"]["hard_stop"]["fin"].update(fin_angle=25.0))
    with pytest.raises(RigConfigError, match="travel 320.0° does not match the hard stop's 315.0°"):
        RigConfig.load(path=str(rig_copy))


def test_fin_contact_must_meet_the_arm(rig_copy):
    _edit(rig_copy, lambda c: c["turntable"]["hard_stop"]["fin"].update(fin_contact_angle=140.0))
    with pytest.raises(RigConfigError, match="must meet the arm's edge at 145.0°"):
        RigConfig.load(path=str(rig_copy))


def test_stops_stay_inside_the_travel(rig_copy):
    _edit(rig_copy, lambda c: c["turntable"]["stops"].update(count=23))
    with pytest.raises(RigConfigError, match="leave the travel"):
        RigConfig.load(path=str(rig_copy))


def test_search_needs_one_sigma_per_direction(rig_copy):
    _edit(rig_copy, lambda c: c["solver"]["search"]["tower"]["tilt"].update(sigma=[0.3]))
    with pytest.raises(RigConfigError, match="solver.search.tower.tilt"):
        RigConfig.load(path=str(rig_copy))


def test_unknown_stop_key_is_reported(rig_copy):
    _edit(rig_copy, lambda c: c["turntable"]["stops"].update(steps=15))
    with pytest.raises(RigConfigError, match="malformed"):
        RigConfig.load(path=str(rig_copy))


def test_save_calibration_keeps_history(rig_copy):
    rig = RigConfig.load(path=str(rig_copy))
    assert rig.load_calibration()["history"] == []
    first = dict(rig.load_calibration(), calibrated_at="2026-11-06T12:00:00", axis={"tilt": [0.1, 0.0], "offset": [0.2, 0.0]})
    rig.save_calibration(first)
    assert rig.load_calibration()["history"] == []          # the empty template is not a run
    rig.save_calibration(dict(first, calibrated_at="2026-11-07T12:00:00"))
    saved = rig.load_calibration()
    assert saved["calibrated_at"] == "2026-11-07T12:00:00"
    assert [h["calibrated_at"] for h in saved["history"]] == ["2026-11-06T12:00:00"]
    assert saved["history"][0]["axis"]["tilt"] == [0.1, 0.0]


def test_camera_pose_follows_the_mount_holes(rig_copy):
    rig = RigConfig.load()
    assert rig.camera_nominal["position"] == [246.5, 0.0, 269.25]      # the 270 hole
    assert abs(rig.camera_nominal["look_at"][2] - (269.25 - 246.5 * np.tan(np.radians(rig.raw["camera"]["mount"]["tilt"])))) < 0.01
    _edit(rig_copy, lambda c: c["camera"]["mount"].update(distance=230.0))
    assert RigConfig.load(path=str(rig_copy)).camera_nominal["position"] == [236.5, 0.0, 269.25]
    _edit(rig_copy, lambda c: c["camera"]["mount"].update(distance=235.0))
    with pytest.raises(RigConfigError, match="not a hole"):
        RigConfig.load(path=str(rig_copy))
