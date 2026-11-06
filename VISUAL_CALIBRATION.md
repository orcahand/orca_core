# Visual calibration (branch `visual-calibration`)

Goal: measure the hand's 17 joint angles from photos, then correct the motors
until the hand matches a target pose.

Branched from Diego's `feature/improve-calibration` (2026-11-06). Plan only so
far: no code yet.

## Setup (decided)

- **One camera**, fixed to the rig frame. It never moves with the hand.
- **The hand turns** on a motor around a vertical axis (z up, so gravity pulls
  the same way in every photo). A mechanical hardstop sets 0°; the motor angle
  is known well.
- **Simple rig**: no glued QR codes or stickers as the reference. Clemens wants
  as few setup parts as possible.
- **The tower (the hand's base) is the reference**: rigid, in every photo, and
  its geometry is known from the CAD.

Steps: calibrate the lens → fit camera + axis on the tower → least squares on
the fingers.

## Decisions and why

- **Why the camera must stay still**: if it turned with the hand, every photo
  would be the same view. A fixed camera and a turning hand equals a camera
  walking around the hand.
- **Why not a sticker board as the reference**: its position on the rig comes
  from the CAD plus how it was stuck on. A stuck-on error (~0.2 mm or 0.1°
  ≈ 1 px) shifts every joint angle and the solver can't see it.
- **Why the tower instead**: printed features sit at their CAD positions by
  construction, nothing to stick. Residual risk: print tolerance
  (±0.1-0.2 mm), so the tower's critical features must be accurate.
- **Why the solver also fits small rig errors**: the axis can be tilted or
  offset from the CAD, and the angle at each stop can be off slightly. Fitting
  them with priors, instead of trusting the CAD, removes the need for a precise
  rig.
- **Why rigid parts first**: a small camera error can look like every finger
  being a few degrees off, with a low pixel error. Fitting the camera and axis
  on the tower before the fingers stops the joints from absorbing it.

## Unknowns in the solver

| Unknown | Count | Prior / constraint |
|---|---|---|
| Camera pose (relative to the rig) | 6 | per session; from the tower fit |
| Axis tilt | 2 | small, ~0 |
| Axis offset | 2 | small, ~0 |
| Angle correction per stop | 1 per stop | small, ~0; the hardstop stop pinned at 0 |
| Joint angles | 17 per pose | start from the motor angles |
| Later: link lengths, joint axes | many | only from many poses |

Identifiability: camera pose and axis offset can only be told apart because the
hand turns. Use a wide arc (≥ 120°, better 360°). A global angle offset trades
off with the camera's rotation about the axis, which is why the hardstop stop is
pinned.

## 1. Build the rig

- [ ] Turntable + motor, axis vertical, hardstop at 0°; the tower bolted on,
      in a repeatable position (dowel pins)
- [ ] Camera on the fixed frame, roughly perpendicular to the axis, tilted
      20-30° down to see the fingertips and palm
- [ ] Cables: slip ring, or limit the sweep to ±180°
- [ ] Tower: add printed fiducials (dots, holes or engraved ArUco tags) on
      several faces, so some are visible at every stop. Silhouette/edges alone
      work but are less precise and can have symmetric look-alike poses
- [ ] Small markers on each finger link (or markerless later)
- [ ] Confirm z is the forearm's long axis (`orca_core/kinematics/frames.py`)

## 2. Calibrate the lens (once)

- [ ] Lock focus, zoom and resolution; autofocus and stabilization off
- [ ] ChArUco board across the whole frame, corners included, at different
      tilts → `fx, fy, cx, cy` + distortion. A loose board, only for this step
- [ ] Target: < ~0.5 px on images left out of the fit

## 3. Fit camera + axis on the tower (each session, automatic)

- [ ] Sweep 24 stops (15°) with any finger pose
- [ ] Fit camera pose + axis tilt/offset + per-stop angle corrections to the
      tower's fiducials (or edges) across all stops, joints out of the fit
- [ ] Rough start: CAD camera position, or one photo of a loose board
- [ ] Check: overlay the rendered tower on each stop

| What the overlay shows | Likely cause |
|---|---|
| Same offset at every stop | Camera pose, or a pixel convention (half-pixel shift, flipped axis) |
| Error changes with the stop | Axis wobble or stop angle error |
| Error grows toward the frame edges | Lens calibration |

## 4. Collect data

- [ ] ~20 poses across each joint's range, each joint visible at some stop
      (a tucked thumb or tight fist can stay hidden at every angle)
- [ ] Per pose: 12-24 stops, fingers held still; repeat the first stop at the
      end to check they held

## 5. Least squares on the fingers

Forward kinematics: `HandKinematics` (`orca_core/kinematics/hand_kinematics.py`).

- [ ] Undistort; project with the calibrated lens values (never a nominal field
      of view)
- [ ] Per finger first (four 3-D problems + one 4-D thumb), grid-search ±10° in
      1° steps around the motor angles, then refine
- [ ] Then all together, camera/axis held near the tower fit:
      `pixel error² + w_q·(q − q_motor)² + w_rig·(rig − rig_tower)²`
- [ ] Later, from many poses: link lengths and joint axes (CAD vs real hand)

## 6. Check for wrong answers

- [ ] Photos left out fit about as well as the ones used
- [ ] ~1 px per photo, with no pattern as the hand turns
- [ ] Jacobian uncertainty: large = that joint is poorly seen in that pose
- [ ] Different start guesses give the same answer
- [ ] A joint at a hardstop solves to that hardstop's angle
- [ ] The rig values moved no more than the tower fit's noise in the final solve

## 7. Rig error model (over many sessions)

- [ ] Log the fitted axis tilt/offset and per-stop angle corrections each session
- [ ] Consistent across sessions → bake them into a fixed rig model (constants),
      and keep only a small prior in the solver
- [ ] A pattern over the stop angle (e.g. sinusoidal = bearing wobble or encoder
      eccentricity) → model it as a few Fourier terms in θ
- [ ] Drifting between sessions → keep fitting per session; check the camera
      mount and the tower mounting

## 8. Correct the hand

- [ ] Measure → motor correction of 50-100% of the error → re-measure; stop
      under tolerance (e.g. 1°)
- [ ] Always approach a pose from the same side (tendon hysteresis)
- [ ] Save per-joint offsets as the calibration

## Later: live tracking

2-3 fixed cameras, 60-90° apart; markers, or markerless (SAM 3 on downscaled
frames to find and track, BiRefNet on full-resolution crops for edges and
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
