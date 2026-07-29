# Tactile Data in 3D Frames

`OrcaHandTouch` can express per-taxel tactile data — 3D taxel positions and
force vectors — in a set of coordinate frames, from the raw sensor frame up to
a user-defined world frame.

## The frame stack

| Frame | Description |
|---|---|
| `sensor` | A tactile sensor's own frame. Origin on the sensor mounting plane at the connector end; X across the sensor width, Y along the sensor toward the fingertip, Z outward through the sensing surface. Taxel positions and forces are natively measured here. |
| `fingertip` | The distal link frame of the finger (located at the distal revolute joint). The sensor sits at a fixed, factory-known mount pose in this frame. |
| `palm` | The palm (carpals) link; moves with the wrist. |
| `base` | The static root of the hand (the forearm structure it is mounted by). |
| `world` | `base` composed with a user-supplied hand pose (identity unless set). |

Frame names are available as constants in `orca_core.frames`.

## Getting taxel data

```python
from orca_core import OrcaHandTouch, frames

hand = OrcaHandTouch("path/to/config.yaml")
hand.connect()
hand.start_tactile_stream(resultant=False, taxels=True)

data = hand.get_taxel_data(frame=frames.PALM)
index = data["index"]
index.positions   # (n, 3) taxel positions in meters, palm frame
index.forces      # (n, 3) taxel forces in Newtons, palm frame
```

Row `i` of `positions` and `forces` describe the same taxel. Positions get the
full rigid transform; forces are free vectors and are only rotated, so their
magnitudes are preserved in every frame.

Frames beyond `fingertip` require joint angles for the forward kinematics. By
default the hand's current joint positions are used; pass `joint_pos` (degrees,
orca_core joint ids) to override:

```python
data = hand.get_taxel_data(frame=frames.BASE, joint_pos={"index_mcp": 40.0})
```

For the world frame, tell the hand where it is whenever its mount moves (e.g.
the robot arm's end-effector pose):

```python
from orca_core import Transform

hand.set_base_pose(Transform.from_xyz_rpy([0.4, 0.0, 0.8], [0.0, 0.0, 1.57]))
data = hand.get_taxel_data(frame=frames.WORLD)
```

## Working with raw transforms

`get_sensor_transforms(frame, joint_pos=None)` returns `{finger: Transform}`
mapping sensor-frame data into `frame`, for users who want to transform their
own quantities. `hand.kinematics` exposes the underlying `HandKinematics`
(fingertip poses, sensor mount poses, per-finger chains).

```python
transforms = hand.get_sensor_transforms(frames.BASE)
contact_world = transforms["index"].apply_to_points(my_sensor_frame_points)
```

## Notes

- The kinematic constants (joint origins/axes, sensor mount poses) are packaged
  with orca_core for the v2 hand models and derived from the official
  `orcahand_description` v2 URDFs.
- Joint angles used for FK are motor-derived estimates unless your hand has
  joint encoders; accuracy of `palm`/`base`/`world` positions follows the
  accuracy of the joint angles.
- Run `examples/taxel_frames.py --mock --frame base` for a no-hardware
  demo.
