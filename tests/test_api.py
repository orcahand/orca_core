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
