import shutil

import pytest

from visual_calibration.rig_config import RigConfig, get_rig_path


@pytest.fixture
def uncalibrated(tmp_path):
    """A copy of the rig with nothing solved yet (the real one may be calibrated)."""
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference.yaml"))
    (path / "calibration.yaml").write_text("calibrated_at: null\nhistory: []\n")
    return RigConfig.load(path=str(path))
