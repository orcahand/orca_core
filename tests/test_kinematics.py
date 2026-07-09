"""Tests for rigid transforms, packaged kinematic chains, and hand FK."""

import numpy as np
import pytest

from orca_core.kinematics import FINGERS, HandKinematics, Transform, frames, rotation_about_axis

RIGHT = HandKinematics.load("right")
LEFT = HandKinematics.load("left")

# Golden fingertip positions (base frame, zero pose) computed from the raw v2
# URDFs with an independent FK implementation (max deviation ~1e-9).
ZERO_POSE_TIPS_BASE = {
    "right": {
        "thumb": [-0.045445, 0.09231, -0.091682],
        "index": [-0.047527, 0.195899, -0.069292],
        "middle": [-0.015863, 0.208526, -0.07525],
        "ring": [0.014056, 0.201213, -0.072041],
        "pinky": [0.042716, 0.177709, -0.065517],
    },
    "left": {
        "thumb": [0.025475, 0.144874, -0.075099],
        "index": [0.027517, 0.21691, 0.002662],
        "middle": [-0.004133, 0.230669, 0.005023],
        "ring": [-0.034053, 0.222838, 0.003457],
        "pinky": [-0.062714, 0.199842, -0.004679],
    },
}

BENT_POSE = {
    "wrist": -30.0,
    "index_abd": 10.0,
    "index_mcp": 45.0,
    "index_pip": 60.0,
    "thumb_cmc": -20.0,
    "thumb_abd": 30.0,
    "thumb_mcp": 25.0,
    "thumb_dip": 40.0,
}
BENT_POSE_GOLDENS_RIGHT = {
    ("base", "thumb"): [-0.068198, 0.155653, -0.058498],
    ("base", "index"): [-0.035444, 0.206012, -0.033167],
    ("palm", "thumb"): [-0.058198, 0.052754, 0.06821],
    ("palm", "index"): [-0.025444, 0.02313, 0.11617],
}


class TestTransform:
    def test_identity(self):
        assert np.array_equal(Transform.identity().matrix, np.eye(4))

    def test_from_xyz_rpy_pure_yaw(self):
        t = Transform.from_xyz_rpy([1.0, 2.0, 3.0], [0.0, 0.0, np.pi / 2])
        np.testing.assert_allclose(t.apply_to_points([1.0, 0.0, 0.0]), [1.0, 3.0, 3.0], atol=1e-12)

    def test_rpy_is_extrinsic_xyz(self):
        # roll then pitch about fixed axes: R = Ry @ Rx
        t = Transform.from_xyz_rpy(rpy=[np.pi / 2, np.pi / 2, 0.0])
        expected = rotation_about_axis(np.array([0, 1, 0]), np.pi / 2) @ rotation_about_axis(
            np.array([1, 0, 0]), np.pi / 2
        )
        np.testing.assert_allclose(t.rotation, expected, atol=1e-12)

    def test_compose_and_inverse_roundtrip(self):
        rng = np.random.default_rng(0)
        t = Transform.from_xyz_rpy(rng.normal(size=3), rng.normal(size=3))
        roundtrip = t @ t.inverse()
        np.testing.assert_allclose(roundtrip.matrix, np.eye(4), atol=1e-12)

    def test_apply_to_vectors_ignores_translation_and_preserves_norm(self):
        t = Transform.from_xyz_rpy([10.0, -5.0, 2.0], [0.3, -0.8, 1.1])
        vectors = np.random.default_rng(1).normal(size=(7, 3))
        rotated = t.apply_to_vectors(vectors)
        np.testing.assert_allclose(
            np.linalg.norm(rotated, axis=1), np.linalg.norm(vectors, axis=1), atol=1e-12
        )
        np.testing.assert_allclose(rotated, t.apply_to_points(vectors) - t.translation, atol=1e-12)

    def test_matrix_is_immutable(self):
        t = Transform.identity()
        with pytest.raises(ValueError):
            t.matrix[0, 3] = 1.0

    def test_bad_shape_raises(self):
        with pytest.raises(ValueError):
            Transform(np.eye(3))

    def test_rotation_about_axis_matches_rpy(self):
        np.testing.assert_allclose(
            rotation_about_axis(np.array([0.0, 0.0, 1.0]), 0.7),
            Transform.from_xyz_rpy(rpy=[0.0, 0.0, 0.7]).rotation,
            atol=1e-12,
        )


class TestPackagedKinematics:
    @pytest.mark.parametrize("kin", [RIGHT, LEFT], ids=["right", "left"])
    def test_zero_pose_fingertips_match_urdf(self, kin):
        tips = kin.fingertip_poses({}, in_frame=frames.BASE)
        for finger, expected in ZERO_POSE_TIPS_BASE[kin.hand_type].items():
            np.testing.assert_allclose(tips[finger].translation, expected, atol=1e-6)

    def test_bent_pose_goldens(self):
        for in_frame in (frames.BASE, frames.PALM):
            tips = RIGHT.fingertip_poses(BENT_POSE, in_frame=in_frame)
            for finger in ("thumb", "index"):
                np.testing.assert_allclose(
                    tips[finger].translation,
                    BENT_POSE_GOLDENS_RIGHT[(in_frame, finger)],
                    atol=1e-6,
                )

    def test_base_composes_palm(self):
        base_tips = RIGHT.fingertip_poses(BENT_POSE, in_frame=frames.BASE)
        palm_tips = RIGHT.fingertip_poses(BENT_POSE, in_frame=frames.PALM)
        wrist_chain = Transform.identity()
        from orca_core.kinematics.hand_kinematics import _joint_transform

        for entry in RIGHT._chains["base_chain"]:
            wrist_chain = wrist_chain @ _joint_transform(entry, BENT_POSE)
        for finger in FINGERS:
            np.testing.assert_allclose(
                (wrist_chain @ palm_tips[finger]).matrix, base_tips[finger].matrix, atol=1e-12
            )

    def test_missing_joints_default_to_zero(self):
        assert np.allclose(
            RIGHT.fingertip_poses({}, in_frame=frames.BASE)["index"].matrix,
            RIGHT.fingertip_poses(
                {"index_abd": 0.0, "index_mcp": 0.0, "index_pip": 0.0, "wrist": 0.0},
                in_frame=frames.BASE,
            )["index"].matrix,
        )

    def test_invalid_frame_raises(self):
        with pytest.raises(ValueError):
            RIGHT.fingertip_poses({}, in_frame="fingertip")

    def test_index_abd_sign_is_inverted(self):
        index_abd = next(
            e for e in RIGHT._chains["fingers"]["index"] if e["joint"] == "index_abd"
        )
        assert index_abd["sign"] == -1

    @pytest.mark.parametrize("kin", [RIGHT, LEFT], ids=["right", "left"])
    def test_abduction_axes_are_mutually_parallel(self, kin):
        """The four finger abduction joints all rotate about the palm normal, so
        their sign-applied axes must be parallel once expressed in the palm frame.
        A single flipped ``sign`` shows up here as an anti-parallel axis."""
        signed_axes = {}
        for finger in ("index", "middle", "ring", "pinky"):
            pose = Transform.identity()
            for entry in kin._chains["fingers"][finger]:
                pose = pose @ Transform.from_xyz_rpy(entry["xyz"], entry["rpy"])
                if entry.get("joint", "").endswith("_abd"):
                    axis = pose.rotation @ np.array(entry["axis"])
                    signed_axes[finger] = entry["sign"] * axis / np.linalg.norm(axis)
                    break

        reference = signed_axes["index"]
        for finger, axis in signed_axes.items():
            assert np.dot(reference, axis) > 0.9, (
                f"{finger}_abd rotates opposite to index_abd "
                f"(dot={np.dot(reference, axis):.3f}); its sign is likely flipped"
            )

    @pytest.mark.parametrize("kin", [RIGHT, LEFT], ids=["right", "left"])
    def test_positive_abduction_moves_every_fingertip_toward_the_pinky(self, kin):
        """Sign convention, anchored on ``index_abd`` whose sign is forced by the
        asymmetric URDF/ROM limit pair. Catches a global as well as a per-joint flip."""
        fingers = ("index", "middle", "ring", "pinky")
        rest = kin.fingertip_poses({}, in_frame=frames.PALM, fingers=fingers)
        lateral = rest["pinky"].translation - rest["index"].translation
        lateral /= np.linalg.norm(lateral)

        for finger in fingers:
            tip = kin.fingertip_poses(
                {f"{finger}_abd": 20.0}, in_frame=frames.PALM, fingers=(finger,)
            )[finger]
            displacement = np.dot(tip.translation - rest[finger].translation, lateral)
            assert displacement > 0.005, (
                f"+20 deg of {finger}_abd moved the fingertip {displacement * 1000:+.1f} mm "
                f"along the index->pinky axis; positive abduction must move it toward the pinky"
            )


class TestSensorMounts:
    def test_right_mounts_present_for_all_fingers(self):
        assert set(RIGHT.sensor_mounts) == set(FINGERS)

    def test_ring_shares_middle_mount(self):
        mounts = RIGHT.sensor_mounts
        np.testing.assert_allclose(mounts["ring"].matrix, mounts["middle"].matrix)

    def test_mount_translations_within_fingertip(self):
        for finger, mount in RIGHT.sensor_mounts.items():
            assert np.all(np.abs(mount.translation) < 0.03), finger

    def test_sensor_poses_compose_mount(self):
        tips = RIGHT.fingertip_poses(BENT_POSE, in_frame=frames.BASE)
        sensors = RIGHT.sensor_poses(BENT_POSE, in_frame=frames.BASE)
        mounts = RIGHT.sensor_mounts
        for finger in FINGERS:
            np.testing.assert_allclose(
                sensors[finger].matrix, (tips[finger] @ mounts[finger]).matrix, atol=1e-12
            )

    def test_left_mounts_equal_right(self):
        # The left fingertip meshes are the same geometry as the right (hand
        # mirroring happens upstream in the kinematic tree), so the sensor
        # sits at the identical pose in the distal link frame.
        right, left = RIGHT.sensor_mounts, LEFT.sensor_mounts
        assert set(left) == set(right)
        for finger in right:
            np.testing.assert_allclose(left[finger].matrix, right[finger].matrix)
