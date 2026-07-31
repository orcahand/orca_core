"""Smoke tests for the HTTP API: every route exercised against a MockOrcaHand."""

import os
import shutil

import pytest

pytest.importorskip("httpx", reason="fastapi.testclient requires httpx")

from fastapi.testclient import TestClient

import orca_core
from orca_core import MockOrcaHand, OrcaHand
import orca_core.api.api as api


@pytest.fixture(scope="module")
def api_env(tmp_path_factory):
    """TestClient bound to a connected, calibrated MockOrcaHand swapped into the app."""
    config_dir = tmp_path_factory.mktemp("api_model")
    packaged = os.path.join(
        os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right", "config.yaml"
    )
    shutil.copy(packaged, config_dir / "config.yaml")
    (config_dir / "calibration.yaml").write_text("{}\n", encoding="utf-8")

    hand = MockOrcaHand(config_path=str(config_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"
    hand.init_joints(force_calibrate=True)

    original_hand = api.hand
    api.hand = hand
    try:
        yield TestClient(api.app), hand, config_dir
    finally:
        api.hand = original_hand
        hand.stop_task()
        hand.disconnect()


def test_status(api_env):
    client, _, _ = api_env
    resp = client.get("/status")
    assert resp.status_code == 200
    assert resp.json() == {"connected": True, "calibrated": True}


def test_connect_when_already_connected(api_env):
    client, _, _ = api_env
    resp = client.post("/connect")
    assert resp.status_code == 200
    assert "already connected" in resp.json()["message"].lower()


def test_torque_enable_disable(api_env):
    client, hand, _ = api_env
    assert client.post("/torque/enable").status_code == 200
    assert client.post(
        "/torque/enable", json={"motor_ids": hand.config.motor_ids[:2]}
    ).status_code == 200
    assert client.post("/torque/disable").status_code == 200


def test_set_max_current(api_env):
    client, _, _ = api_env
    resp = client.post("/current/max", json={"current": 300.0})
    assert resp.status_code == 200


def test_motor_state_endpoints(api_env):
    client, hand, _ = api_env
    n_motors = len(hand.config.motor_ids)

    resp = client.get("/motors/position")
    assert resp.status_code == 200
    assert len(resp.json()["positions"]) == n_motors

    resp = client.get("/motors/current")
    assert resp.status_code == 200
    assert len(resp.json()["currents"]) == n_motors

    resp = client.get("/motors/temperature")
    assert resp.status_code == 200
    assert len(resp.json()["temperatures"]) == n_motors


def test_get_joint_position(api_env):
    client, hand, _ = api_env
    resp = client.get("/joints/position")
    assert resp.status_code == 200
    positions = resp.json()["positions"]
    assert set(positions) == set(hand.config.joint_ids)
    assert all(isinstance(v, float) for v in positions.values())


def test_set_joint_position(api_env):
    client, hand, _ = api_env
    joint = hand.config.joint_ids[0]
    rom_min, rom_max = hand.config.joint_roms_dict[joint]
    target = (rom_min + rom_max) / 2

    resp = client.post("/joints/position", json={"positions": {joint: target}})
    assert resp.status_code == 200

    readback = client.get("/joints/position").json()["positions"]
    assert readback[joint] == pytest.approx(target, abs=1.0)


def test_calibrate_status(api_env):
    client, _, _ = api_env
    resp = client.get("/calibrate/status")
    assert resp.status_code == 200
    assert resp.json() == {"calibrated": True}


def test_calibrate(api_env):
    client, _, _ = api_env
    resp = client.post("/calibrate")
    assert resp.status_code == 200
    body = resp.json()
    assert body["calibrated"] is True


def test_disconnect_and_reconnect(api_env):
    client, _, _ = api_env
    resp = client.post("/disconnect")
    assert resp.status_code == 200

    resp = client.post("/disconnect")
    assert resp.status_code == 200
    assert "already disconnected" in resp.json()["message"].lower()

    resp = client.post("/connect")
    assert resp.status_code == 200


def test_set_config(api_env):
    client, mock_hand, config_dir = api_env
    resp = client.post("/config", json=str(config_dir / "config.yaml"))
    assert resp.status_code == 200
    assert isinstance(api.hand, OrcaHand)
    api.hand = mock_hand


@pytest.fixture
def disconnected_client(tmp_path, monkeypatch):
    """TestClient whose hand is a constructed-but-never-connected MockOrcaHand."""
    packaged = os.path.join(
        os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right", "config.yaml"
    )
    shutil.copy(packaged, tmp_path / "config.yaml")
    (tmp_path / "calibration.yaml").write_text("{}\n", encoding="utf-8")
    hand = MockOrcaHand(config_path=str(tmp_path / "config.yaml"))
    monkeypatch.setattr(api, "hand", hand)
    return TestClient(api.app)


def test_mutating_endpoints_busy_return_409(api_env):
    """A lifecycle endpoint arriving while another holds the lock is rejected."""
    client, _, _ = api_env
    assert api._hand_lock.acquire(blocking=False)
    try:
        for path, body in (
            ("/connect", None),
            ("/disconnect", None),
            ("/config", "/some/config.yaml"),
            ("/calibrate", None),
        ):
            resp = client.post(path, json=body) if body else client.post(path)
            assert resp.status_code == 409, path
            assert "in progress" in resp.json()["detail"]
    finally:
        api._hand_lock.release()


def test_read_endpoints_stay_responsive_while_locked(api_env):
    client, _, _ = api_env
    assert api._hand_lock.acquire(blocking=False)
    try:
        assert client.get("/status").status_code == 200
        assert client.get("/motors/position").status_code == 200
        assert client.get("/calibrate/status").status_code == 200
    finally:
        api._hand_lock.release()


def test_connect_failure_detail_is_not_rewrapped(monkeypatch):
    class _FailingConnectHand:
        def is_connected(self):
            return False

        def connect(self, interactive=True):
            return False, "boom"

    monkeypatch.setattr(api, "hand", _FailingConnectHand())
    client = TestClient(api.app)
    resp = client.post("/connect")
    assert resp.status_code == 500
    assert resp.json()["detail"] == "Connection failed: boom"


def test_motor_state_endpoints_return_null_when_disconnected(disconnected_client):
    for path, key in (
        ("/motors/position", "positions"),
        ("/motors/current", "currents"),
        ("/motors/temperature", "temperatures"),
    ):
        resp = disconnected_client.get(path)
        assert resp.status_code == 200, path
        assert resp.json() == {key: None}


def test_control_endpoints_return_409_when_disconnected(disconnected_client):
    assert disconnected_client.post("/torque/enable").status_code == 409
    assert disconnected_client.post("/torque/disable").status_code == 409
    assert disconnected_client.post("/current/max", json={"current": 300.0}).status_code == 409
    assert disconnected_client.get("/joints/position").status_code == 409
    resp = disconnected_client.post("/joints/position", json={"positions": {"wrist": 0.0}})
    assert resp.status_code == 409


def test_set_config_does_not_build_default_hand(monkeypatch):
    """POST /config on a fresh server builds only the requested hand."""
    constructed = []

    def _factory(*args, **kwargs):
        constructed.append(kwargs)
        return MockOrcaHand.__new__(MockOrcaHand)

    monkeypatch.setattr(api, "hand", None)
    monkeypatch.setattr(api, "OrcaHand", _factory)
    client = TestClient(api.app)
    resp = client.post("/config", json="/some/config.yaml")
    assert resp.status_code == 200
    assert constructed == [{"config_path": "/some/config.yaml"}]


def test_hand_is_constructed_lazily_on_first_use(monkeypatch):
    created = []

    class _StubHand:
        def is_connected(self):
            return False

        def is_calibrated(self):
            return False

    def _factory(*args, **kwargs):
        stub = _StubHand()
        created.append(stub)
        return stub

    monkeypatch.setattr(api, "hand", None)
    monkeypatch.setattr(api, "OrcaHand", _factory)
    client = TestClient(api.app)
    resp = client.get("/status")
    assert resp.status_code == 200
    assert resp.json() == {"connected": False, "calibrated": False}
    assert len(created) == 1 and api.hand is created[0]
