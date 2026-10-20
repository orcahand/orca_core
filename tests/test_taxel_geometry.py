"""Tests for static taxel geometry loading and its wiring into the tactile config."""

import os

import numpy as np
import pytest
import yaml

from orca_core.constants import FINGER_NAMES
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import DEFAULT_TAXEL_COUNTS
from orca_core.hardware.sensing import taxel_geometry
from orca_core.hardware.sensing.taxel_geometry import (
    SENSOR_FRAME,
    load_all_taxel_geometry,
    load_taxel_geometry,
)
from orca_core.hardware.sensing.tactile_mock import (
    TactileMockState,
    feed_taxels_frame,
    install_tactile_mock,
)
from orca_core.hardware.tactile_client import TactileClient


def _make_client(connected_fingers=None):
    state = TactileMockState()
    if connected_fingers is not None:
        state.connected_fingers = list(connected_fingers)
    link = MockHandSerialLink()
    install_tactile_mock(link, state)
    link.connect()
    client = TactileClient(link, finger_to_sensor_id=state.finger_to_sensor_id)
    return client, link, state


def _sensor_models():
    """One geometry per sensor model; index, middle and ring share one."""
    return list({geom.model: geom for geom in load_all_taxel_geometry().values()}.values())


def _nearest_neighbours(positions, k=6):
    """Indices of each taxel's ``k`` nearest other taxels."""
    distances = np.linalg.norm(positions[:, None] - positions[None], axis=2)
    return np.argsort(distances, axis=1, kind="stable")[:, 1 : k + 1]


def _angles_deg(a, b):
    return np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", a, b), -1.0, 1.0)))


def _yaml_axes(model):
    """Each taxel's ``(x_axis, y_axis, z_axis)`` rows as written in the model YAML."""
    with open(os.path.join(taxel_geometry.MODELS_DIR, model, "config.yaml")) as f:
        rows = yaml.load(f, Loader=getattr(yaml, "CSafeLoader", yaml.SafeLoader))["coordinates"]
    return np.array([[r["x_axis"], r["y_axis"], r["z_axis"]] for r in rows], dtype=float)


class TestLoadGeometry:
    def test_counts_match_taxel_counts(self):
        for finger, expected in DEFAULT_TAXEL_COUNTS.items():
            geom = load_taxel_geometry(finger)
            assert geom.num_taxels == expected, (
                f"{finger}: expected {expected} positions, got {geom.num_taxels}"
            )

    def test_positions_shape_and_dtype(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.positions.shape == (geom.num_taxels, 3)
            assert geom.positions.dtype == np.float64

    def test_frame_is_sensor(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.frame == SENSOR_FRAME

    def test_positions_are_meters(self):
        # Sensor bodies are a few cm long; mm values would exceed this by 1000x.
        for geom in load_all_taxel_geometry().values():
            assert np.all(np.abs(geom.positions) < 0.05)
            assert np.max(np.abs(geom.positions)) > 0.005

    def test_known_first_taxel_converted(self):
        positions = load_taxel_geometry("index").positions
        np.testing.assert_allclose(
            positions[0], [4.03569563e-3, 28.08244307e-3, 3.72577291e-3]
        )

    def test_cache_returns_the_same_array_object(self):
        assert (
            load_taxel_geometry("index").positions
            is load_taxel_geometry("index").positions
        )

    def test_cached_positions_are_read_only(self):
        """Callers share the cached array, so it must not be writable."""
        with pytest.raises(ValueError):
            load_taxel_geometry("index").positions[0, 0] = 1.0


class TestTaxelAxes:
    def test_orientations_shape_and_dtype(self):
        for geom in load_all_taxel_geometry().values():
            assert geom.orientations.shape == (geom.num_taxels, 3, 3)
            assert geom.orientations.dtype == np.float64

    def test_normals_match_the_local_surface(self):
        # A plane through each taxel and its six nearest neighbours, signed by the
        # neighbours' normals; correct axes stay within about 20 deg of it.
        for geom in _sensor_models():
            nb = _nearest_neighbours(geom.positions)
            patch = np.concatenate([geom.positions[:, None], geom.positions[nb]], axis=1)
            fitted = np.linalg.svd(patch - patch.mean(axis=1, keepdims=True))[2][:, 2]
            fitted *= np.sign(np.einsum("ij,ij->i", fitted, geom.normals[nb].sum(axis=1)))[:, None]
            off = _angles_deg(fitted, geom.normals)
            assert off.max() < 30.0, (
                f"{geom.model}: taxel {off.argmax()} normal is {off.max():.0f} deg off its surface"
            )

    def test_tangent_axes_follow_their_neighbours(self):
        # Each taxel's x axis against its neighbours' x axes projected onto its surface.
        for geom in _sensor_models():
            nb = _nearest_neighbours(geom.positions)
            x_axes, normals = geom.orientations[:, :, 0], geom.normals
            mean_x = x_axes[nb].sum(axis=1)
            mean_x -= np.einsum("ij,ij->i", mean_x, normals)[:, None] * normals
            mean_x /= np.linalg.norm(mean_x, axis=1, keepdims=True)
            off = _angles_deg(mean_x, x_axes)
            assert off.max() < 45.0, (
                f"{geom.model}: taxel {off.argmax()} x axis is {off.max():.0f} deg off its neighbours'"
            )

    def test_top_of_pad_faces_up(self):
        for geom in load_all_taxel_geometry().values():
            top = np.argmax(geom.positions[:, 2])
            assert geom.normals[top][2] > 0.85

    def test_known_first_taxel_normal(self):
        normal = load_taxel_geometry("index").normals[0]
        np.testing.assert_allclose(normal, [0.53679367, 0.82664428, 0.16885436], atol=1e-6)

    def test_cached_orientations_are_read_only(self):
        geom = load_taxel_geometry("index")
        assert geom.orientations is load_taxel_geometry("index").orientations
        with pytest.raises(ValueError):
            geom.orientations[0, 0, 0] = 1.0


class TestRotateForces:
    def test_local_components_land_on_the_yaml_axes(self):
        for geom in _sensor_models():
            axes = _yaml_axes(geom.model)
            local = np.tile([0.7, -0.4, 2.5], (geom.num_taxels, 1))
            expected = 0.7 * axes[:, 0] - 0.4 * axes[:, 1] + 2.5 * axes[:, 2]
            np.testing.assert_allclose(geom.rotate_forces(local), expected, atol=1e-6)

    def test_magnitudes_are_preserved(self):
        geom = load_taxel_geometry("middle")
        local = np.random.default_rng(0).normal(size=(geom.num_taxels, 3))
        np.testing.assert_allclose(
            np.linalg.norm(geom.rotate_forces(local), axis=1),
            np.linalg.norm(local, axis=1),
            atol=1e-12,
        )

    def test_rejects_wrong_taxel_count(self):
        geom = load_taxel_geometry("index")
        with pytest.raises(ValueError):
            geom.rotate_forces(np.zeros((geom.num_taxels - 1, 3)))


def _write_model(models_dir, name, coordinate_rows):
    model_dir = models_dir / name
    model_dir.mkdir(parents=True)
    rows = "\n".join(f"- {row}" for row in coordinate_rows)
    (model_dir / "config.yaml").write_text(
        f"name: {name}\nframe: sensor\nunits: mm\n"
        f"num_taxels: {len(coordinate_rows)}\ncoordinates:\n{rows}\n"
    )


class TestAxesValidation:
    @pytest.fixture
    def models_dir(self, tmp_path, monkeypatch):
        monkeypatch.setattr(taxel_geometry, "MODELS_DIR", str(tmp_path))
        return tmp_path

    def test_loads_and_snaps_rounded_axes(self, models_dir):
        _write_model(models_dir, "axes-rounded", [
            "{x: 0, y: 0, z: 1, x_axis: [1, 0, 0.00001], y_axis: [0, 1, 0], z_axis: [0, 0, 1]}",
        ])
        _, orientations = taxel_geometry._load_model("axes-rounded")
        np.testing.assert_allclose(orientations[0].T @ orientations[0], np.eye(3), atol=1e-12)

    def test_rejects_missing_axes(self, models_dir):
        _write_model(models_dir, "axes-missing", ["{x: 0, y: 0, z: 1}"])
        with pytest.raises(ValueError, match="x_axis"):
            taxel_geometry._load_model("axes-missing")

    def test_rejects_non_orthonormal_axes(self, models_dir):
        _write_model(models_dir, "axes-skewed", [
            "{x: 0, y: 0, z: 1, x_axis: [1, 0.2, 0], y_axis: [0, 1, 0], z_axis: [0, 0, 1]}",
        ])
        with pytest.raises(ValueError, match="orthonormal"):
            taxel_geometry._load_model("axes-skewed")

    def test_rejects_left_handed_axes(self, models_dir):
        _write_model(models_dir, "axes-mirrored", [
            "{x: 0, y: 0, z: 1, x_axis: [1, 0, 0], y_axis: [0, 1, 0], z_axis: [0, 0, -1]}",
        ])
        with pytest.raises(ValueError, match="right-handed"):
            taxel_geometry._load_model("axes-mirrored")


class TestConfigWiring:
    def test_geometry_present_for_connected_fingers(self):
        client, link, _ = _make_client()
        client.connect()
        geometry = client.get_taxel_geometry()
        assert set(geometry) == set(FINGER_NAMES)
        for finger in FINGER_NAMES:
            assert geometry[finger].num_taxels == DEFAULT_TAXEL_COUNTS[finger]
        link.disconnect()

    def test_only_connected_fingers_have_geometry(self):
        client, link, _ = _make_client(connected_fingers=["index", "thumb"])
        client.connect()
        assert set(client.get_taxel_geometry()) == {"index", "thumb"}
        link.disconnect()

    def test_geometry_count_matches_taxel_forces(self):
        client, link, state = _make_client(connected_fingers=["index"])
        client.connect()
        client.start_stream(resultant=False, taxels=True)
        taxels = {"index": [[0.1, 0.2, 0.3]] * DEFAULT_TAXEL_COUNTS["index"]}
        feed_taxels_frame(link, taxels, state.active_sensors)
        client.wait_for_first_frame()
        reading = client.get_latest_taxels()
        positions = client.get_taxel_geometry()["index"].positions
        assert len(positions) == len(reading["index"])
        link.disconnect()

    def test_empty_before_connect(self):
        client, link, _ = _make_client()
        assert client.get_taxel_geometry() == {}
        link.disconnect()
