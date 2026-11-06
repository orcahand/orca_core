# Plan: visual calibration of the hand (one camera, rotating hand)

Goal: measure the hand's 17 joint angles from photos, then correct the motors
until the hand matches a target pose.

Setup (decided):
- One camera, fixed to the rig frame. It never moves with the hand.
- The hand and its base turn on a motor around a vertical axis (z up, so
  gravity pulls the same way in every photo).
- A marker board fixed to the hand's base turns with the hand, so every photo
  measures the base pose directly.

Steps: calibrate the camera → calibrate the camera to the rig → least squares
on the hand.

## 1. Build the rig

The rig must be rigid. It doesn't need to be precisely machined: markers
measure the hand-camera position better than it can be built in.

- [ ] Turntable with a motor, axis vertical; the hand's base bolted to it
- [ ] Camera on the fixed frame, roughly perpendicular to the axis, tilted
      20-30° down to see the fingertips and palm
- [ ] Cables: slip ring, or limit the sweep to ±180° (still covers every angle)
- [ ] ChArUco board on the base, in a printed pocket at a known CAD position.
      Large or two-faced, so it is seen at every stop (a small flat board can
      flip between two near-equal poses)
- [ ] One fixed tag on the rig frame, in view at every stop (detects a bumped camera)
- [ ] Small markers on each finger link
- [ ] Confirm z is the forearm's long axis (`orca_core/kinematics/frames.py`)

## 2. Calibrate the camera (once)

- [ ] Lock focus, zoom and resolution; autofocus and stabilization off.
      Changing any of these invalidates the calibration
- [ ] Photograph a ChArUco board across the whole frame, corners included, at
      different tilts; solve `fx, fy, cx, cy` and distortion
- [ ] Target: under ~0.5 px reprojection error on images left out of the fit

## 3. Calibrate the camera to the rig

The board on the base gives the base pose in every photo, so the joint fit
never depends on the motor angle. This step measures the rotation axis to
check the rig and to predict the base pose when the board is hidden.

- [ ] Turn a full circle in 24 stops (15°), one photo per stop
- [ ] Per photo, `solvePnP` on the board → board pose in the camera frame `T_i`
- [ ] Fit the axis: the relative rotations `T_i⁻¹·T_j` all turn about one axis;
      the board origin traces a circle around it. Fit axis direction + point
      and the angle of each stop
- [ ] Compare fitted angles with the motor angles → motor angle error
- [ ] Residuals from the circle → axis wobble. Above ~0.1 mm: fix the rig
      before going on
- [ ] Hand to board: from the CAD position of the pocket. Check it by
      overlaying the rendered base/forearm outline on the photos (rigid parts
      only, joints out of the fit)
- [ ] Save: lens calibration, axis (in the camera frame), board→hand transform,
      fixed-tag pose

How to read the overlay of the rigid parts:

| What the overlay shows | Likely cause |
|---|---|
| Same offset at every stop | Camera pose, or a pixel convention (half-pixel shift, flipped axis) |
| Error changes with the stop | Axis wobble or stop angle error |
| Error grows toward the frame edges | Lens calibration |

## 4. Collect data

- [ ] About 20 poses across each joint's range, each joint visible at some stop
      (a tucked thumb or tight fist can stay hidden at every angle)
- [ ] Per pose: 12-24 stops, fingers held still
- [ ] Check the fingers held: compare the first and a repeated last photo

## 5. Least squares on the hand

Unknowns per pose: 17 joint angles. The base pose per photo comes from step 3.
Forward kinematics: `HandKinematics` (`orca_core/kinematics/hand_kinematics.py`).

- [ ] Undistort each photo; project the model's link markers with the
      calibrated lens values (never a nominal field of view)
- [ ] Start from the motor angles
- [ ] Solve each finger separately: four 3-D problems + one 4-D (thumb).
      Grid-search around the motor estimate (±10° in 1° steps), then refine
- [ ] Refine all 17 together with robust least squares:
      `pixel error² + w_q·(q − q_motor)²`
- [ ] Later, from many poses: link-length and joint-axis errors (CAD vs the real
      hand). Not earlier, or they absorb camera errors

## 6. Check for wrong answers

- [ ] Photos left out fit about as well as the ones used
- [ ] ~1 px per photo, with no pattern as the hand turns
- [ ] Uncertainty from the Jacobian: large = that joint is poorly seen in that pose
- [ ] Different start guesses give the same answer
- [ ] A joint at a hardstop solves to that hardstop's angle

## 7. Correct the hand

- [ ] Measure the pose
- [ ] Send a motor correction of 50-100% of the error
- [ ] Re-measure; stop under tolerance (e.g. 1°)
- [ ] Always approach a pose from the same side (tendon hysteresis)
- [ ] Save per-joint offsets as the calibration

## 8. Maintain

- [ ] Each session: check the fixed tag. If the camera moved, redo step 3

## Later: live tracking

2-3 fixed cameras, 60-90° apart; markers, or markerless (SAM 3 to find and
track on downscaled frames, BiRefNet on full-resolution crops for edges and
cables). Untested on our clips.

## Reference numbers

Estimates at 4K, 60° field of view, 0.5 m, 40 mm link:
- 1 px ≈ 0.15 mm
- In-plane motion ≈ 0.2° per pixel
- Motion toward the camera: ±5° shifts only ~1 px. Why one view is not
  enough, and why the hand turns

## Sources

- [SAM 3 paper](https://arxiv.org/html/2511.16719v2)
- [MatAnyone 2 paper](https://arxiv.org/html/2512.11782v2)
- [Matting library survey](https://ice-ice-bear.github.io/posts/2026-04-13-matting-libraries)
