"""Tests for rigid transforms, packaged kinematic chains, and hand FK."""

import numpy as np
import pytest

from orca_core.kinematics import FINGERS, HandKinematics, Transform, frames, rotation_about_axis

RIGHT = HandKinematics.load("right")
LEFT = HandKinematics.load("left")

# Golden fingertip positions (base frame, zero pose): the right chain with the
# wrist-origin roll removed, and its exact sagittal-plane mirror for the left.
ZERO_POSE_TIPS_BASE = {
    "right": {
        "thumb": [-0.045445, 0.144895, -0.075095],
        "index": [-0.047527, 0.216908, 0.002661],
        "middle": [-0.015863, 0.230669, 0.005023],
        "ring": [0.014056, 0.222837, 0.003457],
        "pinky": [0.042716, 0.199842, -0.004680],
    },
    "left": {
        "thumb": [0.045445, 0.144895, -0.075095],
        "index": [0.047527, 0.216908, 0.002661],
        "middle": [0.015863, 0.230669, 0.005023],
        "ring": [-0.014056, 0.222837, 0.003457],
        "pinky": [-0.042716, 0.199842, -0.004680],
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
    ("base", "thumb"): [-0.068198, 0.177748, -0.011581],
    ("base", "index"): [-0.035444, 0.204471, 0.038054],
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

    def test_equality_is_a_bool_and_exact(self):
        a = Transform.from_xyz_rpy([1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
        b = Transform.from_xyz_rpy([1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
        c = Transform.from_xyz_rpy([1.0, 2.0, 3.1], [0.1, 0.2, 0.3])
        assert (a == b) is True
        assert (a == c) is False
        assert a != c
        assert not (a != b)

    def test_equality_with_non_transform_is_false_not_an_error(self):
        assert (Transform.identity() == 4) is False
        assert (Transform.identity() == None) is False  # noqa: E711
        assert Transform.identity() != "not a transform"

    def test_equal_transforms_collapse_in_sets_and_dicts(self):
        a = Transform.from_xyz_rpy([0.5, 0.0, 0.0])
        b = Transform.from_xyz_rpy([0.5, 0.0, 0.0])
        c = Transform.identity()
        assert hash(a) == hash(b)
        assert {a, b, c} == {a, c}
        d = {a: "first", c: "identity"}
        d[b] = "second"
        assert d[a] == "second"
        assert len(d) == 2

    def test_signed_zero_compares_and_hashes_alike(self):
        matrix = np.eye(4)
        matrix[0, 3] = -0.0
        a, b = Transform(np.eye(4)), Transform(matrix)
        assert a == b
        assert hash(a) == hash(b)


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

    def test_left_mounts_mirror_right(self):
        # The left hand is the exact sagittal (x=0) reflection of the right, so
        # each sensor mount is the right mount reflected across that plane.
        reflect = np.diag([-1.0, 1.0, 1.0, 1.0])
        right, left = RIGHT.sensor_mounts, LEFT.sensor_mounts
        assert set(left) == set(right)
        for finger in right:
            np.testing.assert_allclose(
                left[finger].matrix, reflect @ right[finger].matrix @ reflect, atol=1e-9
            )


class TestLeftRightMirror:
    """The left hand is generated as the exact sagittal reflection of the right,
    so every fingertip and sensor pose must mirror across the x=0 plane."""

    REFLECT = np.diag([-1.0, 1.0, 1.0, 1.0])
    POSES = [
        {},
        {"wrist": -25.0, "index_mcp": 40.0, "thumb_cmc": -15.0, "pinky_abd": 20.0},
        {"index_abd": -20.0, "ring_abd": 15.0, "middle_mcp": 30.0, "thumb_abd": 25.0},
    ]

    @pytest.mark.parametrize("pose", POSES)
    @pytest.mark.parametrize("frame", [frames.PALM, frames.BASE])
    def test_fingertips_and_sensors_mirror(self, pose, frame):
        for getter in ("fingertip_poses", "sensor_poses"):
            right = getattr(RIGHT, getter)(pose, in_frame=frame)
            left = getattr(LEFT, getter)(pose, in_frame=frame)
            for finger in right:
                np.testing.assert_allclose(
                    left[finger].matrix, self.REFLECT @ right[finger].matrix @ self.REFLECT,
                    atol=1e-7,
                )
