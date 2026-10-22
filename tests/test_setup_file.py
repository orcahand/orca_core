"""setup.yaml keeps every setup record in one file; a section never written
is still read from the separate file it used to live in."""

from types import SimpleNamespace

from orca_core.maintenance import guided_tension, setup_file
from orca_core.utils.utils import read_yaml, write_yaml_atomic


def test_sections_share_one_file_and_legacy_files_still_read(tmp_path):
    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    write_yaml_atomic(str(tmp_path / "tension_progress.yaml"), {"hands": {"h1": {"rounds": 2}}})
    assert guided_tension.read_progress(config, "h1") == {"rounds": 2}

    guided_tension.record_hold_tension(config)
    setup_file.write_section(config, "slack_tests", {"t": 1})
    data = read_yaml(str(tmp_path / "setup.yaml"))
    assert data["last_tension"]["mode"] == "hold" and data["slack_tests"] == {"t": 1}

    guided_tension.clear_progress(config, "h1")
    assert guided_tension.read_progress(config, "h1") is None
    assert read_yaml(str(tmp_path / "tension_progress.yaml"))["hands"]["h1"] == {"rounds": 2}
