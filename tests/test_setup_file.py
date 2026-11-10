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


def test_the_file_is_parsed_once_until_it_changes(tmp_path, monkeypatch):
    """The console reads sections on every refresh; a file grown by recorded
    sweeps must not be parsed again each time."""
    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    setup_file.write_section(config, "slack_tests", {"t": 1})
    parses = []
    real_load = setup_file.yaml.load
    monkeypatch.setattr(
        setup_file.yaml, "load", lambda *a, **k: parses.append(1) or real_load(*a, **k))

    assert setup_file.read_section(config, "slack_tests") == {"t": 1}
    assert setup_file.read_section(config, "slack_tests") == {"t": 1}
    assert len(parses) == 1

    setup_file.write_section(config, "slack_tests", {"t": 2})
    assert setup_file.read_section(config, "slack_tests") == {"t": 2}
    # Another process (a script beside the console) rewriting the file.
    write_yaml_atomic(str(tmp_path / "setup.yaml"), {"slack_tests": {"t": 3}})
    assert setup_file.read_section(config, "slack_tests") == {"t": 3}


def test_a_returned_section_is_the_callers_own_copy(tmp_path):
    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    setup_file.write_section(config, "slack_tests", {"runs": [1]})

    setup_file.read_section(config, "slack_tests")["runs"].append(2)
    assert setup_file.read_section(config, "slack_tests") == {"runs": [1]}
