"""The rig's own files live in its folder under ORCA_HOME, named by its board; the config stays in the repo."""
import logging
import shutil

import pytest
import yaml

from visual_calibration import rig_home
from visual_calibration.rig_config import RigConfig, get_rig_path
from visual_calibration.turntable import Turntable


def test_rig_id_is_the_rig_id_else_the_usb_serial(orca_home):
    assert rig_home.rig_id("0123456789ABCDEF", "rig-1") == "rig-1"
    assert rig_home.rig_id("0123456789ABCDEF", "ser-0001") == "0123456789ABCDEF"     # a hand's serial is no rig id
    assert rig_home.remember("0123456789ABCDEF", "rig-2") == "rig-2"
    assert rig_home.rig_id("0123456789ABCDEF") == "rig-2"                            # offline, from the pinned serial
    assert rig_home.rig_dir("rig-2") == orca_home / "rigs" / "rig-2"
    with pytest.raises(rig_home.RigIdError):
        rig_home.rig_id(None, "ser-0001")
    with pytest.raises(rig_home.RigIdError):
        rig_home.rig_dir("../x")


def test_the_rig_s_calibrations_go_to_its_folder(orca_home, tmp_path, caplog):
    rig = RigConfig.load()
    serial = rig.raw["turntable"]["board"]["usb_serial"]
    assert rig.home == str(orca_home / "rigs" / serial)
    assert RigConfig.load(rig_id="rig-3").home == str(orca_home / "rigs" / "rig-3")
    checkout = tmp_path / "checkout"
    shutil.copytree(get_rig_path(), checkout)
    (checkout / "calibration.yaml").write_text("calibrated_at: '2026-11-01T00:00:00'\nhistory: []\n")
    rig = RigConfig.load(path=str(checkout))
    assert rig.calibration_path == str(checkout / "calibration.yaml")      # a rig copy keeps its own files
    rig.home = str(orca_home / "rigs" / "rig-3")
    with caplog.at_level(logging.WARNING):
        assert rig.load_calibration()["calibrated_at"] == "2026-11-01T00:00:00"     # left in the checkout: read
    assert "move it there" in caplog.text
    rig.save_calibration({"calibrated_at": "2026-11-23T00:00:00"})
    assert rig.calibration_path == str(orca_home / "rigs" / "rig-3" / "calibration.yaml")
    assert yaml.safe_load((checkout / "calibration.yaml").read_text())["calibrated_at"] == "2026-11-01T00:00:00"
    assert rig.load_calibration()["history"][0]["calibrated_at"] == "2026-11-01T00:00:00"


def test_the_motor_calibration_is_saved_to_the_rig_s_folder(tmp_path):
    left, own = tmp_path / "left.yaml", tmp_path / "rig" / "turntable_calibration.yaml"
    left.write_text("zero_in_turn_deg: 10.0\n")
    t = Turntable.open(RigConfig.load().raw["turntable"], mock=True, calibration_path=str(left), save_path=str(own))
    try:
        cal = t.calibrate(save=True)
    finally:
        t.close()
    assert yaml.safe_load(own.read_text())["zero_motor_deg"] == cal["zero_motor_deg"]
    assert left.read_text() == "zero_in_turn_deg: 10.0\n"
