"""Tests for the frame-aware taxel API on OrcaHandTouch (mock tactile client)."""

import os

import numpy as np
import pytest

import orca_core
from orca_core import OrcaHandTouch, Transform, frames
from orca_core.hardware.mock_tactile_client import MockTactileClient
from orca_core.hardware.sensing.constants import FINGER_NAMES

CONFIG_V2_TOUCH_RIGHT = os.path.join(
    os.path.dirname(orca_core.__file__), "models", "v2", "orcahand_touch_right", "config.yaml"
)

BENT_POSE = {"wrist": -20.0, "index_mcp": 40.0, "index_pip": 30.0, "thumb_cmc": -10.0}


@pytest.fixture
def hand():
    hand = OrcaHandTouch(config_path=CONFIG_V2_TOUCH_RIGHT)
    client = MockTactileClient(connected_sensors=list(FINGER_NAMES))
    client.connect()
    client.start_auto_stream(resultant=False, taxels=True)
    client.wait_for_first_frame(timeout=2.0)
    hand._tactile_client = client
    yield hand
    client.stop_auto_stream()
    client.disconnect()


class TestGetTaxelData:
    def test_sensor_frame_matches_geometry_and_stream(self, hand):
        data = hand.get_taxel_data(frame=frames.SENSOR)
        geometry = hand.get_taxel_geometry()
        reading = hand.get_tactile_taxels()
        assert set(data) == set(FINGER_NAMES)
        for finger, taxel_data in data.items():
            assert taxel_data.frame == frames.SENSOR
            np.testing.assert_allclose(taxel_data.positions, geometry[finger].positions)
            assert taxel_data.num_taxels == len(reading.as_array(finger))

    def test_fingertip_frame_applies_mount(self, hand):
        sensor = hand.get_taxel_data(frame=frames.SENSOR)
        fingertip = hand.get_taxel_data(frame=frames.FINGERTIP)
        mounts = hand.kinematics.sensor_mounts
        for finger in FINGER_NAMES:
            np.testing.assert_allclose(
                fingertip[finger].positions,
                mounts[finger].apply_to_points(sensor[finger].positions),
                atol=1e-12,
            )

    def test_forces_are_rotated_not_translated(self, hand):
        sensor = hand.get_taxel_data(frame=frames.SENSOR)
        base = hand.get_taxel_data(frame=frames.BASE, joint_pos=BENT_POSE)
        for finger in FINGER_NAMES:
            np.testing.assert_allclose(
                np.linalg.norm(base[finger].forces, axis=1),
                np.linalg.norm(sensor[finger].forces, axis=1),
                atol=1e-9,
            )

    def test_base_frame_uses_fk(self, hand):
        base = hand.get_taxel_data(frame=frames.BASE, joint_pos=BENT_POSE)
        sensor_poses = hand.kinematics.sensor_poses(BENT_POSE, in_frame=frames.BASE)
        geometry = hand.get_taxel_geometry()
        for finger in FINGER_NAMES:
            np.testing.assert_allclose(
                base[finger].positions,
                sensor_poses[finger].apply_to_points(geometry[finger].positions),
                atol=1e-12,
            )

    def test_world_frame_composes_base_pose(self, hand):
        pose = Transform.from_xyz_rpy([0.5, -0.2, 1.0], [0.1, 0.2, 0.3])
        hand.set_base_pose(pose)
        base = hand.get_taxel_data(frame=frames.BASE, joint_pos=BENT_POSE)
        world = hand.get_taxel_data(frame=frames.WORLD, joint_pos=BENT_POSE)
        for finger in FINGER_NAMES:
            np.testing.assert_allclose(
                world[finger].positions,
                pose.apply_to_points(base[finger].positions),
                atol=1e-12,
            )

    def test_world_frame_defaults_to_identity(self, hand):
        base = hand.get_taxel_data(frame=frames.BASE, joint_pos=BENT_POSE)
        world = hand.get_taxel_data(frame=frames.WORLD, joint_pos=BENT_POSE)
        for finger in FINGER_NAMES:
            np.testing.assert_allclose(world[finger].positions, base[finger].positions)

    def test_invalid_frame_raises(self, hand):
        with pytest.raises(ValueError):
            hand.get_taxel_data(frame="fingernail")

    def test_none_before_first_stream_frame(self):
        hand = OrcaHandTouch(config_path=CONFIG_V2_TOUCH_RIGHT)
        client = MockTactileClient(connected_sensors=["index"])
        client.connect()
        hand._tactile_client = client
        assert hand.get_taxel_data() is None
        client.disconnect()


class TestSensorTransforms:
    def test_sensor_frame_is_identity(self, hand):
        for transform in hand.get_sensor_transforms(frames.SENSOR).values():
            np.testing.assert_array_equal(transform.matrix, np.eye(4))

    def test_fingertip_transforms_are_mounts(self, hand):
        mounts = hand.kinematics.sensor_mounts
        for finger, transform in hand.get_sensor_transforms(frames.FINGERTIP).items():
            np.testing.assert_array_equal(transform.matrix, mounts[finger].matrix)

    def test_palm_differs_from_base_by_wrist_chain(self, hand):
        palm = hand.get_sensor_transforms(frames.PALM, joint_pos=BENT_POSE)
        base = hand.get_sensor_transforms(frames.BASE, joint_pos=BENT_POSE)
        # relative transform between the two must be the same for every finger
        rels = [
            (base[f] @ palm[f].inverse()).matrix for f in sorted(palm)
        ]
        for rel in rels[1:]:
            np.testing.assert_allclose(rel, rels[0], atol=1e-12)
