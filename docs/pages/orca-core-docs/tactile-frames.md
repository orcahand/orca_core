# Tactile Data in 3D Frames

`OrcaHandTouch` can express per-taxel tactile data — 3D taxel positions and
force vectors — in a set of coordinate frames, from the raw sensor frame up to
a user-defined world frame.

## The frame stack

| Frame | Description |
|---|---|
| `sensor` | A tactile sensor's own frame. Origin on the sensor mounting plane at the connector end; X across the sensor width, Y along the sensor toward the fingertip, Z outward through the sensing surface. Taxel positions are natively expressed here; per-taxel forces are rotated into it from each taxel's own axes (see below). |
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

## Taxel axes

Each taxel measures force in its own local axes: `z` along its outward surface
normal (the pressure, always ≥ 0) and `x`, `y` tangent to the surface (signed
shear). On the curved fingertip those axes point a different way for every
taxel; on the sides of the pad a taxel's `z` is nearly horizontal in the sensor
frame. So the raw values from `get_tactile_taxels()` are **not** sensor-frame
vectors.

`get_taxel_data()` handles this: it rotates every taxel's force out of its
local axes into the sensor frame before applying the frame stack above. To do
it yourself, use the per-taxel orientations in the geometry:

```python
geom = hand.get_taxel_geometry()["index"]
geom.orientations   # (n, 3, 3) local-to-sensor rotations; columns = local x, y, z
geom.normals        # (n, 3) outward surface normals (local z axes)

local = hand.get_tactile_taxels().as_array("index")   # (n, 3), taxel axes
forces = geom.rotate_forces(local)                    # (n, 3), sensor frame
```

A press straight into taxel `i` therefore shows up as `fz * geom.normals[i]`.
The axes ship with the sensor models' positions in
`orca_core/hardware/sensing/models/*/config.yaml`.

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

## Readings: frame numbers, fixed arrays, the force ceiling

`get_tactile_data()`, `get_tactile_forces()` and `get_tactile_taxels()` return
the latest frame (50 Hz), in the sensor's own axes rather than a 3D frame.

```python
reading = hand.get_tactile_data()
reading.seq              # frame number: same number = same frame, a jump = frames missed
reading.forces.array     # (5, 3) newtons, rows thumb → pinky, NaN for an absent finger
reading.taxels.array     # (363, 3), finger rows at TAXEL_SLICES[finger], NaN if absent
reading.forces.saturated # fingers whose normal force reached the 25.5 N ceiling
```

`seq` counts the frames the computer received since the stream started; it
can't see frames lost before they arrive. The arrays are read-only and shared
by every caller of the same frame: `.copy()` before writing. The sensor caps
the resultant normal force at 25.5 N, so a saturated finger's real force is at
least that; its taxels keep rising past the cap.

## Zeroing, fingertip health, reboot

```python
hand.zero_tactile_sensors()      # fingertips unloaded: nothing may touch them
hand.get_fingertip_health()      # {finger: FingertipHealth} or None
hand.reboot_tactile_board()      # restarts the sensing board, re-arms the stream
```

On connector firmware 1.0 and later, `zero_tactile_sensors()` first has the
fingertips zero themselves, then captures a host baseline; older boards get the
host baseline only. The fingertip zero also clears a phantom left by powering
up under load. Zeroing under load leaves a phantom after release: zero again
unloaded, or power-cycle the hand.

`get_fingertip_health()` returns `None` before firmware 1.0. From 1.0 each
finger reports an error code (`gone`, `resultant_read_failed`,
`taxel_read_failed`; 0 = healthy, a good reply clears it) and counters since
the board started: good polls, timeouts, bad frames, disconnects.

`reboot_tactile_board()` stops the joint-encoder stream for about 20 ms, so a
hand refuses it while its joint loop runs. Only an ORCA connector board is
rebooted (Paxini's own comm board drops off USB), and the reboot counts only
once the board's counters show it happened.

## Notes

- The kinematic constants (joint origins/axes, sensor mount poses) are packaged
  with orca_core for the v2 hand models and derived from the official
  `orcahand_description` v2 URDFs.
- Joint angles used for FK are motor-derived estimates unless your hand has
  joint encoders; accuracy of `palm`/`base`/`world` positions follows the
  accuracy of the joint angles.
- Run `examples/taxel_frames.py --mock --frame base` for a no-hardware
  demo.
