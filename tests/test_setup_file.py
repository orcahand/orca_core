"""setup.yaml keeps every setup record in one file; a section never written
is still read from the separate file it used to live in."""

import os
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


def test_a_writer_waits_for_another_process_and_keeps_its_section(tmp_path):
    """Another process holding the lock while it rewrites the file must not
    have its section overwritten by a writer that read the file before it."""
    import threading
    import time

    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    path = setup_file.setup_path(config)
    with open(os.path.realpath(path) + setup_file.LOCK_SUFFIX, "a+b") as other:
        assert setup_file._try_lock(other.fileno())
        writer = threading.Thread(
            target=setup_file.write_section, args=(config, "slack_tests", {"t": 1}))
        writer.start()
        time.sleep(0.3)
        assert writer.is_alive(), "the writer did not wait for the lock"
        write_yaml_atomic(path, {"sweeps": {"s": 1}})
        setup_file._unlock(other.fileno())
    writer.join(timeout=5)

    assert read_yaml(path) == {"sweeps": {"s": 1}, "slack_tests": {"t": 1}}


def test_a_lock_held_too_long_raises_instead_of_waiting_forever(tmp_path, monkeypatch):
    import pytest

    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    path = setup_file.setup_path(config)
    monkeypatch.setattr(setup_file, "LOCK_TIMEOUT_S", 0.2)
    with open(os.path.realpath(path) + setup_file.LOCK_SUFFIX, "a+b") as other:
        assert setup_file._try_lock(other.fileno())
        with pytest.raises(TimeoutError, match="locked"):
            setup_file.write_section(config, "slack_tests", {"t": 1})
        setup_file._unlock(other.fileno())


def test_a_lock_file_this_user_cannot_write_still_locks(tmp_path):
    """A lock file left by another user (say a script run with sudo) is read-only here."""
    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    lock = os.path.realpath(setup_file.setup_path(config)) + setup_file.LOCK_SUFFIX
    open(lock, "w").close()
    os.chmod(lock, 0o444)
    try:
        setup_file.write_section(config, "slack_tests", {"t": 1})
    finally:
        os.chmod(lock, 0o644)
    assert setup_file.read_section(config, "slack_tests") == {"t": 1}


def test_a_file_system_that_cannot_lock_still_saves(tmp_path, monkeypatch, caplog):
    import errno

    def no_locks(fd):
        raise OSError(errno.ENOLCK, "No locks available")

    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    monkeypatch.setattr(setup_file, "_try_lock", no_locks)
    setup_file.write_section(config, "slack_tests", {"t": 1})
    assert setup_file.read_section(config, "slack_tests") == {"t": 1}
    assert "without the cross-process lock" in caplog.text


def test_a_writer_waiting_for_another_process_does_not_hold_up_reads(tmp_path, monkeypatch):
    import threading
    import time

    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    path = setup_file.setup_path(config)
    setup_file.write_section(config, "sweeps", {"s": 1})
    waiting = threading.Event()
    real_try = setup_file._try_lock

    def try_lock(fd):
        locked = real_try(fd)
        if not locked:
            waiting.set()
        return locked

    monkeypatch.setattr(setup_file, "_try_lock", try_lock)
    with open(os.path.realpath(path) + setup_file.LOCK_SUFFIX, "a+b") as other:
        assert real_try(other.fileno())
        writer = threading.Thread(
            target=setup_file.write_section, args=(config, "slack_tests", {"t": 1}))
        writer.start()
        assert waiting.wait(5)
        started = time.monotonic()
        assert setup_file.read_section(config, "sweeps") == {"s": 1}
        assert time.monotonic() - started < 1.0
        assert writer.is_alive()
        setup_file._unlock(other.fileno())
    writer.join(timeout=5)
    assert setup_file.read_section(config, "slack_tests") == {"t": 1}


def test_writers_in_separate_processes_lose_no_update(tmp_path):
    import subprocess
    import sys

    config = SimpleNamespace(calibration_path=str(tmp_path / "calibration.yaml"))
    script = (
        "import sys; from types import SimpleNamespace;"
        "from orca_core.maintenance import setup_file;"
        "c = SimpleNamespace(calibration_path=sys.argv[1]);"
        "[setup_file.update_section(c, 'count', lambda n: (n or 0) + 1) for _ in range(40)];"
        "setup_file.write_section(c, sys.argv[2], True)"
    )
    workers = [
        subprocess.Popen([sys.executable, "-c", script, config.calibration_path, name])
        for name in ("sweeps", "slack_tests")
    ]
    assert all(w.wait(timeout=60) == 0 for w in workers)

    assert read_yaml(setup_file.setup_path(config)) == {
        "count": 80, "sweeps": True, "slack_tests": True}
