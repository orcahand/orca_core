"""Which sensing streams ``scripts/monitor_sensors.py`` opens, and on which ports.

The dashboard reads the hand's own declaration: a config with a ``sensors``
block but no ``joint_encoder_joints`` opens no encoder stream and vice versa,
and the ports come from the same resolver the hand classes use. The Tkinter
view itself is not covered.
"""

import dataclasses
import importlib.util
import os
import sys
from types import SimpleNamespace
from typing import ClassVar
from unittest.mock import patch

import pytest

import orca_core
from orca_core import OrcaHandConfig, OrcaHandTouchConfig
from orca_core.hardware.sensing.constants import (
    DEFAULT_ENCODER_BAUDRATE,
    DEFAULT_SENSOR_BAUDRATE,
    JOINT_TO_ENCODER_SLOT,
)
from orca_core.hardware.sensing.serial_discovery import SensingPorts
from orca_core.utils.utils import read_yaml

pytest.importorskip("tkinter", reason="monitor_sensors is a Tkinter script")

MODELS = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2")
SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts", "monitor_sensors.py",
)


def _load_script():
    """Import ``scripts/monitor_sensors.py``, which is not on the package path."""
    spec = importlib.util.spec_from_file_location("monitor_sensors", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


monitor = _load_script()


def _config(model_name):
    """The config class load_hand() would pick for a bundled model."""
    path = os.path.join(MODELS, model_name, "config.yaml")
    declares_tactile = "sensors" in (read_yaml(path) or {})
    cls = OrcaHandTouchConfig if declares_tactile else OrcaHandConfig
    return cls.from_config_path(path)


def _manager(model_name, port=None, baud=None, encoder_joints=None):
    """A manager for a bundled model, with probing and board reads stubbed out."""
    config = _config(model_name)
    if encoder_joints is None and config.has_joint_encoders:
        encoder_joints = ALL_ENCODER_JOINTS
    manager = monitor.ConnectionManager(
        config, encoder_joints, port, baud, "Resultant"
    )
    manager._board_identity = lambda port: ""
    return manager


ALL_ENCODER_JOINTS = [
    j for j in JOINT_TO_ENCODER_SLOT
    if j in _config("orcahand-right").joint_to_motor_map
]


def _discovered(tactile=None, encoder=None, tactile_baud=None):
    """Stand in for live discovery, keeping resolve_sensing_ports' own logic."""
    return patch(
        "orca_core.hardware.sensing.serial_discovery.discover_sensing_ports",
        return_value=SensingPorts(tactile, encoder, tactile_baud),
    )


@pytest.fixture
def links(monkeypatch):
    """Swap HandSerialLink for a recorder; yields every link the script builds."""
    built = []

    class FakeLink:
        def __init__(self, port, baudrate, **kwargs):
            self.port, self.baudrate = port, baudrate
            self.connected = self.closed = False
            self.fail_connect = False
            built.append(self)

        def connect(self):
            if self.fail_connect:
                raise OSError("busy")
            self.connected = True

        def disconnect(self):
            self.closed = True

    monkeypatch.setattr(monitor, "HandSerialLink", FakeLink)
    return built


@pytest.fixture
def clients(monkeypatch):
    """Stub the stream clients so a connect attempt speaks no wire protocol."""

    class FakeEncoder:
        def __init__(self, link):
            self.link = link

        def connect(self):
            pass

        def disconnect(self):
            pass

        def start_stream(self, **kwargs):
            pass

    class FakeTactile(FakeEncoder):
        def __init__(self, link, finger_to_sensor_id=None):
            super().__init__(link)
            self.wiring = finger_to_sensor_id

        def stop_stream(self):
            pass

        def get_tactile_configuration(self):
            return None

    monkeypatch.setattr(monitor, "JointEncoderClient", FakeEncoder)
    monkeypatch.setattr(monitor, "TactileClient", FakeTactile)


# ----- which streams a hand declares ----------------------------------------


@pytest.mark.parametrize("model, tactile, encoders", [
    ("orcahand-touch-right", True, False),
    ("orcahand-joint-right", False, True),
    ("orcahand-full-left", True, True),
])
def test_declared_streams_follow_the_config(model, tactile, encoders):
    manager = _manager(model)
    assert (manager.has_tactile, manager.has_encoders) == (tactile, encoders)


def test_a_config_declaring_no_sensing_keeps_looking_for_both():
    """Detection may have missed a board that is plugged in later."""
    manager = _manager("orcahand-right")
    assert manager.has_tactile and manager.has_encoders
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        manager._resolve_targets(pinned=True)
    kwargs = resolve.call_args.kwargs
    assert kwargs["tactile_override"] == "auto"
    assert kwargs["encoder_override"] == "auto"


def test_an_explicit_port_declares_both_streams():
    """--port is the bring-up override: it opens both on a config that declares neither."""
    manager = _manager("orcahand-right", port="/dev/x", baud=DEFAULT_ENCODER_BAUDRATE)
    assert manager.has_tactile and manager.has_encoders


# ----- port resolution ------------------------------------------------------


def test_touch_config_resolves_tactile_only():
    with _discovered(tactile="/dev/t", tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        enc, tac = _manager("orcahand-touch-right")._resolve_targets(pinned=True)
    assert enc is None
    assert tac == ("/dev/t", DEFAULT_SENSOR_BAUDRATE)


def test_joint_config_resolves_encoder_only():
    with _discovered(encoder="/dev/e"):
        enc, tac = _manager("orcahand-joint-right")._resolve_targets(pinned=True)
    assert enc == ("/dev/e", DEFAULT_ENCODER_BAUDRATE)
    assert tac is None


def test_full_config_resolves_both_streams():
    with _discovered(tactile="/dev/t", encoder="/dev/e", tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        enc, tac = _manager("orcahand-full-right")._resolve_targets(pinned=True)
    assert enc == ("/dev/e", DEFAULT_ENCODER_BAUDRATE)
    assert tac == ("/dev/t", DEFAULT_SENSOR_BAUDRATE)


def test_an_undeclared_stream_is_never_probed_for():
    """A hand without encoders must not have its ports scanned for one."""
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        _manager("orcahand-touch-right")._resolve_targets(pinned=True)
    assert resolve.call_args.kwargs["encoder_override"] == "disabled"


def test_packaged_configs_leave_the_ports_to_discovery():
    """Nothing is pinned in a shipped model, so every port stays autodetected."""
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        _manager("orcahand-full-right")._resolve_targets(pinned=True)
    kwargs = resolve.call_args.kwargs
    assert kwargs["tactile_override"] == "auto"
    assert kwargs["encoder_override"] == "auto"


def _pinned_manager(model, port):
    config = dataclasses.replace(
        _config(model), sensor_port=port, encoder_serial_port=port
    )
    manager = monitor.ConnectionManager(
        config, ALL_ENCODER_JOINTS, None, None, "Resultant"
    )
    manager._board_identity = lambda port: ""
    return manager


def test_a_port_named_in_the_config_is_used_before_discovery():
    manager = _pinned_manager("orcahand-full-right", "/dev/pinned")
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        manager._resolve_targets(pinned=True)
    kwargs = resolve.call_args.kwargs
    assert kwargs["tactile_override"] == "/dev/pinned"
    assert kwargs["encoder_override"] == "/dev/pinned"


def test_the_fallback_attempt_rediscovers_instead_of_reusing_the_named_port():
    """A board that comes back on a new path must still be found."""
    manager = _pinned_manager("orcahand-full-right", "/dev/gone")
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        manager._resolve_targets(pinned=False)
    kwargs = resolve.call_args.kwargs
    assert kwargs["tactile_override"] == "auto"
    assert kwargs["encoder_override"] == "auto"


def test_the_fallback_attempt_still_skips_an_undeclared_stream():
    """Falling back must not start probing for encoders a touch hand lacks."""
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        _manager("orcahand-touch-right")._resolve_targets(pinned=False)
    assert resolve.call_args.kwargs["encoder_override"] == "disabled"


def test_an_explicit_tactile_port_gets_its_baud_probed():
    """Discovery is skipped for a named port, so the baud has to be detected."""
    with _discovered(), \
            patch.object(monitor, "resolve_sensing_ports",
                         return_value=SensingPorts("/dev/t", None, None)), \
            patch.object(monitor, "baud_for_port", return_value=DEFAULT_SENSOR_BAUDRATE) as probe:
        _, tac = _manager("orcahand-touch-right")._resolve_targets(pinned=True)
    probe.assert_called_once_with("/dev/t")
    assert tac == ("/dev/t", DEFAULT_SENSOR_BAUDRATE)


def test_baud_flag_overrides_every_configured_rate():
    with _discovered(tactile="/dev/t", encoder="/dev/e", tactile_baud=DEFAULT_SENSOR_BAUDRATE):
        enc, tac = _manager("orcahand-full-right", baud=115200)._resolve_targets(pinned=True)
    assert enc[1] == 115200 and tac[1] == 115200


def test_port_flag_puts_both_streams_on_one_port():
    with patch.object(monitor, "resolve_sensing_ports") as resolve:
        enc, tac = _manager(
            "orcahand-touch-right", port="/dev/x", baud=DEFAULT_ENCODER_BAUDRATE
        )._resolve_targets(pinned=True)
    resolve.assert_not_called()
    assert enc == tac == ("/dev/x", DEFAULT_ENCODER_BAUDRATE)


# ----- sessions -------------------------------------------------------------


def test_streams_on_one_port_share_a_single_link(links):
    target = ("/dev/s", DEFAULT_ENCODER_BAUDRATE)
    session = monitor.SensorSession(target, target)
    assert len(links) == 1
    assert session.enc is not None and session.tac is not None


def test_streams_on_separate_ports_get_a_link_each(links):
    session = monitor.SensorSession(
        ("/dev/e", DEFAULT_ENCODER_BAUDRATE), ("/dev/t", DEFAULT_SENSOR_BAUDRATE)
    )
    assert sorted(link.port for link in links) == ["/dev/e", "/dev/t"]
    assert session.enc is not None and session.tac is not None


def test_a_shared_port_opens_at_the_encoder_baud(links):
    """The encoder stream is the baud-critical one on a link carrying both."""
    session = monitor.SensorSession(
        ("/dev/s", DEFAULT_ENCODER_BAUDRATE), ("/dev/s", DEFAULT_SENSOR_BAUDRATE)
    )
    assert [link.baudrate for link in links] == [DEFAULT_ENCODER_BAUDRATE]
    assert session.where == f"/dev/s @ {DEFAULT_ENCODER_BAUDRATE}"


def test_an_undeclared_stream_gets_no_client_and_no_link(links):
    session = monitor.SensorSession(("/dev/e", DEFAULT_ENCODER_BAUDRATE), None)
    assert session.tac is None
    assert [link.port for link in links] == ["/dev/e"]


def test_the_tactile_client_gets_the_configs_finger_wiring(links, monkeypatch):
    """Sensor ids are per hand: a wrong map labels every finger's readings wrong."""
    wiring = {"thumb": 4, "index": 3, "middle": 2, "ring": 1, "pinky": 0}
    seen = {}
    monkeypatch.setattr(
        monitor, "TactileClient",
        lambda link, finger_to_sensor_id=None: seen.update(wiring=finger_to_sensor_id),
    )
    monitor.SensorSession(None, ("/dev/t", DEFAULT_SENSOR_BAUDRATE), wiring)
    assert seen["wiring"] == wiring


def test_the_manager_takes_the_finger_wiring_from_the_config():
    manager = _manager("orcahand-touch-left")
    assert manager._finger_to_sensor_id == _config("orcahand-touch-left").finger_to_sensor_id
    assert _manager("orcahand-joint-right")._finger_to_sensor_id is None


def test_closing_tolerates_a_link_that_fails_to_close(links):
    session = monitor.SensorSession(
        ("/dev/e", DEFAULT_ENCODER_BAUDRATE), ("/dev/t", DEFAULT_SENSOR_BAUDRATE)
    )
    enc_link, tac_link = session.links["/dev/e"], session.links["/dev/t"]
    enc_link.disconnect = lambda: (_ for _ in ()).throw(OSError("gone"))

    session.close()

    assert tac_link.closed


def _session_with_dead_port(dead: str):
    """Build sessions whose link on *dead* refuses to open."""
    original = monitor.SensorSession

    def build(enc, tac, wiring=None):
        session = original(enc, tac, wiring)
        if dead in session.links:
            session.links[dead].fail_connect = True
        return session

    return patch.object(monitor, "SensorSession", build)


def test_a_port_that_will_not_open_costs_only_its_own_stream(links, clients):
    """A busy tactile adapter must not take the encoder panel down with it."""
    manager = _manager("orcahand-full-right")
    with _discovered(tactile="/dev/t", encoder="/dev/e",
                     tactile_baud=DEFAULT_SENSOR_BAUDRATE), \
            _session_with_dead_port("/dev/t"):
        manager._try_connect()

    snap = manager.snapshot()
    assert snap.connected and snap.enc_open and not snap.tac_open
    assert manager._session.tac is None
    # The port that failed is closed, not left held open.
    assert [link.port for link in links if link.closed] == ["/dev/t"]
    assert "/dev/t" not in snap.where


def test_every_port_failing_leaves_nothing_connected(links, clients):
    manager = _manager("orcahand-touch-right")
    with _discovered(tactile="/dev/t", tactile_baud=DEFAULT_SENSOR_BAUDRATE), \
            _session_with_dead_port("/dev/t"):
        manager._try_connect()

    assert manager._session is None
    assert not manager.snapshot().connected
    assert all(link.closed for link in links)


def test_a_config_with_no_named_port_discovers_once_per_attempt(links, clients):
    """A second sweep costs a full round of serial probes for the same answer."""
    manager = _manager("orcahand-full-right")
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)) as resolve:
        manager._try_connect()
    assert resolve.call_count == 1


def test_a_named_port_that_fails_falls_back_to_the_discovered_one(links, clients, capsys):
    """The re-plug case: the configured path is gone, the board is on a new one."""
    manager = _pinned_manager("orcahand-full-right", "/dev/gone")

    def resolve(**kwargs):
        if kwargs["encoder_override"] == "auto":
            return SensingPorts("/dev/found", "/dev/found", DEFAULT_ENCODER_BAUDRATE)
        return SensingPorts("/dev/gone", "/dev/gone", DEFAULT_ENCODER_BAUDRATE)

    with patch.object(monitor, "resolve_sensing_ports", side_effect=resolve), \
            _session_with_dead_port("/dev/gone"):
        manager._try_connect()

    snap = manager.snapshot()
    assert snap.connected and "/dev/found" in snap.where
    # Dropping a configured port silently is indistinguishable from broken
    # autodetection, so the override has to be named on the way past.
    out = capsys.readouterr().out
    assert "falling back to autodetection" in out and "/dev/gone" in out


def test_nothing_plugged_in_keeps_searching(links, clients):
    manager = _manager("orcahand-touch-right")
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)):
        manager._try_connect()
    assert links == []
    assert manager._session is None
    assert "searching" in manager.snapshot().message


# ----- servicing a session with only one stream ------------------------------


class FakeStats:
    frames_ok = 7
    stream_rearms = 0
    bad_header_resyncs = 3
    frames_bad_lrc: ClassVar[dict[int, int]] = {0: 2}


class FakeClient:
    """Encoder/tactile client stand-in with no serial traffic behind it."""

    def __init__(self, reading="frame"):
        self._reading = reading

    def get_stats(self):
        return FakeStats()

    def get_latest(self):
        return self._reading

    def start_stream(self, **kwargs):
        pass

    def stop_stream(self):
        pass

    def _get_configuration(self):
        return None


def _serviceable(manager, enc=None, tac=None):
    session = monitor.SensorSession(
        ("/dev/e", DEFAULT_ENCODER_BAUDRATE) if enc else None,
        ("/dev/t", DEFAULT_SENSOR_BAUDRATE) if tac else None,
    )
    for link in session.links.values():
        link.is_connected, link.is_port_dead = True, False
        link.get_link_stats = FakeStats
    session.enc = FakeClient() if enc else None
    session.tac = FakeClient() if tac else None
    manager._session = session
    manager._last_rescan = 0.0
    return session


@pytest.mark.parametrize("enc, tac", [(True, False), (False, True)])
def test_servicing_a_one_stream_session_does_not_crash(links, enc, tac):
    """The manager thread dies silently if it touches a client that isn't there."""
    manager = _manager("orcahand-full-right")
    _serviceable(manager, enc=enc, tac=tac)

    manager._service_session()

    snap = manager.snapshot()
    assert (snap.enc_reading is not None) is enc
    assert (snap.tac_reading is not None) is tac
    assert snap.link_resyncs == FakeStats.bad_header_resyncs
    # A stream that was never opened must not report a fault it cannot have.
    assert snap.tac_status == "" or tac


def test_dropping_a_session_clears_what_it_reported(links, clients):
    """A stale 'streaming resultant' would outlive the stream that produced it."""
    manager = _manager("orcahand-full-right")
    with _discovered(tactile="/dev/s", encoder="/dev/s",
                     tactile_baud=DEFAULT_ENCODER_BAUDRATE):
        manager._try_connect()
    manager._set(tac_status="streaming resultant")

    manager._drop_session("link lost")

    snap = manager.snapshot()
    assert snap.tac_status == ""
    assert not (snap.enc_open or snap.tac_open or snap.connected)
    assert snap.where == ""


# ----- what the health score grades ------------------------------------------


def test_health_total_counts_only_the_streams_in_play():
    assert monitor.health_total(17, 17, 5) == 39
    assert monitor.health_total(17, 0, 5) == 22
    assert monitor.health_total(17, 0, 0) == 17


def test_health_total_never_divides_by_zero():
    assert monitor.health_total(0, 0, 0) == 1


def test_encoder_grading_follows_the_configs_joint_list():
    """A config naming three encoder joints must not be scored out of 17."""
    subset = ["index_mcp", "middle_mcp", "wrist"]
    manager = _manager("orcahand-joint-right", encoder_joints=subset)
    assert manager.encoder_joints == subset
    assert monitor.health_total(17, len(manager.encoder_joints), 0) == 20


def test_a_hand_declaring_no_encoders_grades_every_slot_it_finds():
    """Nothing declared means nothing known, so a stream found anyway is graded whole."""
    manager = _manager("orcahand-right")
    assert manager.encoder_joints is None
    assert manager.declares_encoders is False and manager.has_encoders is True


def test_the_fallback_warning_is_not_repeated_every_second(links, clients, capsys):
    manager = _pinned_manager("orcahand-full-right", "/dev/gone")
    with patch.object(monitor, "resolve_sensing_ports",
                      return_value=SensingPorts(None, None)):
        manager._try_connect()
        manager._try_connect()
    assert capsys.readouterr().out.count("falling back to autodetection") == 1


# ----- panel layout ----------------------------------------------------------


def test_the_panel_layout_comes_from_the_configs_joint_list():
    """A hand built with fewer joints gets fewer rows, not blank hardcoded ones."""
    layout = monitor.finger_joints(
        ["wrist", "index_mcp", "index_abd", "thumb_dip", "thumb_cmc"]
    )
    assert layout == {
        "thumb": ["thumb_cmc", "thumb_dip"],
        "index": ["index_abd", "index_mcp"],
        "wrist": ["wrist"],
    }


def test_the_layout_orders_each_finger_proximal_to_distal():
    layout = monitor.finger_joints(
        ["index_pip", "index_abd", "index_mcp", "thumb_mcp", "thumb_cmc", "thumb_abd"]
    )
    assert layout["index"] == ["index_abd", "index_mcp", "index_pip"]
    assert layout["thumb"] == ["thumb_cmc", "thumb_abd", "thumb_mcp"]


def test_a_wristless_hand_gets_no_wrist_column():
    assert "wrist" not in monitor.finger_joints(["index_mcp"])


def test_the_layout_matches_every_bundled_model():
    """Deriving the columns must not change what a shipped hand shows."""
    for model in ("orcahand-right", "orcahand-full-left"):
        config = _config(model)
        layout = monitor.finger_joints(config.joint_ids)
        assert sorted(j for js in layout.values() for j in js) == sorted(config.joint_ids)


def _grading(manager, rows):
    """Run the UI's grading rule without building a Tkinter window."""
    view = SimpleNamespace(manager=manager, joint_rows=dict.fromkeys(rows))
    return monitor.SensorMonitorUI._graded_joints(view)


def test_a_joint_with_no_encoder_slot_is_never_graded():
    """Grading a joint the wire protocol has no slot for raises mid-refresh."""
    manager = _manager("orcahand-right")
    assert manager.encoder_joints is None
    assert _grading(manager, ["index_mcp", "palm_flex"]) == ["index_mcp"]


def test_grading_follows_the_config_over_the_rows_on_screen():
    manager = _manager("orcahand-joint-right", encoder_joints=["index_mcp", "wrist"])
    assert _grading(manager, ["index_mcp", "wrist", "thumb_mcp"]) == ["index_mcp", "wrist"]
