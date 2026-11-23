# Visual calibration (experiment, branch `visual-calibration`)

Self-contained: nothing in the `orca_core` package or its calibration is
changed; this folder only imports `orca_core` (kinematics transforms, YAML
helpers). Move pieces into `orca_core` once they work.

    cd experiments/visual_calibration
    uv run --with opencv-contrib-python-headless pytest tests -q
    uv run python -m visual_calibration.e0

Goal: measure the hand's 17 joint angles from photos, then correct the motors
until the hand matches a target pose.

Branched from Diego's `feature/improve-calibration` (2026-11-06). So far: this
plan and the rig's config, `rigs/v1/camera-rig/config.yaml` (hard stop,
travel, stops, solver search bounds and priors per direction) +
`calibration.yaml` (the solver's results, with a history of runs), loaded by
`visual_calibration/rig_config.py`.

**Where the rig's own files live:** `calibration.yaml` and `turntable_calibration.yaml`
are the rig's, not the repo's: they live in `<ORCA_HOME>/rigs/<rig id>/` (default
`~/.orca`), next to the rig's photo sessions and its hand runs
(`runs/<hand id>/<YYYYMMDD-HHMMSS>/`), see `visual_calibration/rig_home.py`. The rig id
is the rig board's rig ID (`rig-...`) if it has one, else its USB serial (pinned in
`turntable.yaml`). A file still left in `rigs/v1/camera-rig/` is read, with a warning,
until the rig has its own; nothing moves it.

**Measuring a hand** (orca-ui running with the hand connected, the rig calibrated):
`python -m hand_study.run_hand` makes a run folder for the connected hand, plans over its
ranges in force (a standalone copy of its model, the packaged config is never edited),
archives its calibration from its folder in the hand store, collects the short block and the
validation set with the solve workers beside it, and records the fitted hard stops as the
hand's `joint_ranges.yaml` (in force at its next connect). `--dry` builds and lists the run;
`--hand-id` names the hand instead.

## Step 1 in two parts (2026-11-06)

The motor turns the plate (`visual_calibration/turntable.py`: XC430, multi-turn
position mode with a PWM cap; homing ramps into the hard stop until the goal runs
ahead of the plate, that is 0°; moves are slow ramps, a blocked plate is an error).

The turntable has its own OH board with one motor and its own config,
`rigs/v1/camera-rig/turntable.yaml` (board, motor, hard stop, travel, stops),
driven through orca_core's motor client. `port: auto` takes the board whose bus
has exactly one motor, so the hand's board (17 motors) can stay plugged in.
**Motor calibration** (`python -m visual_calibration.turntable calibrate`, or
*Calibrate motor* on the page): home 3× (spread ≤ 0.5°), run to the stop's other
side (travel within 5° of the design, else gear_ratio is wrong; a wrong direction
still measures right, the photos catch it: the sides come round in reverse),
record every stop's motor position -> `turntable_calibration.yaml` (zero, in-turn
ticks, travel, board USB serial). After that the motor's in-turn position gives
the plate angle at connect, no homing needed (direct drive, travel < 1 turn).

Safety (the XC430-T240BB-T has no current control): slow and soft by default
(10°/s, homing 5°/s, PWM 0.2, stall at 2° lag on 3 ticks); homing gives up after
the travel + 20°; any error or Ctrl-C releases the torque (the zero is kept).
Position P gain 400: the factory 700 hunts ±0.2° through the drive's backlash
while holding (1-2 px of motion in the photos).

**Camera controls and the sweep CLI** (`uvc.py`, `sweep.py`): OpenCV on macOS
cannot reach focus or exposure, so `uvcc` (node, `npx uvcc`) sets the BRIO's UVC
controls to `camera.uvc` and reads them back. `python -m visual_calibration.sweep
OUT --step 7.5` does that before the motor moves, then checks one frame at the
parked stop (90°) against `camera_reference.yaml` (tile sharpness, scale = focus,
brightness, clipping; `--save-reference` stores a new one), the zero (homing),
every stop (photo once two frames agree within 0.3 px) and the first again, and
reads the controls again at the end. `--hand`: the hand on the plate, controls
only. Logi Tune must be closed (it rewrites the controls).

**Mounting** (`visual_calibration/mounting.py`): which way the tower sits on the
plate is not assumed. Found from the photos (which sides are seen at which stop;
in 1b from the solved camera positions), snapped to `calibration_object.mountings`
(0/90/180/270°, the connector's bolt pattern) within 15°, else used as found with a warning; recorded in
calibration.yaml (`mounting: yaw, side0_stop`) and used by the overlay. The stop
plan is checked to show sides 0, 2 and 1 or 3 for every mounting.

- **1a camera** (`calibrate_camera`): a sweep, every photo with its own free
  object pose (plate angles unused) -> lens + tile poses.
- **1b rotation** (`calibrate_rig`, started from 1a's lens): a new homed sweep ->
  camera pose in the rig, axis tilt/offset, stop corrections, refining lens and
  tiles with them. Holding 1a's lens fixed costs ~3 px on 11 stops (1a's free
  tiles absorb a little lens error), so 1b refines it.
- The camera's CAD pose (`camera.nominal_pose`, from the mount setting
  `camera.mount`) is the prior: the first photo's camera must be within
  `solver.search.camera` of it (else a clear error: the mount setting is wrong),
  and the fit is held near it by its sigma (10 mm: weak, does not drag it).

## Two cameras (branch `two-camera`, 2026-11-09)

Cam A is the camera above (back at the 260 hole, level); cam B is a second BRIO on a side rail
(rig +y, r ~220, z ~470, 45° down, portrait, aimed at z 250), for every finger. config.yaml
`cameras: {a, b}` (an old `camera:` block loads as a; b is `enabled: false` until plugged in),
`tower: v1 | v2` picks the tile seats (v2: 12 tiles, row 0 37.5° up, gable roof on sides 1/3,
same marker ids). Each photo has a camera (`Photo.camera`; session photos `stop_<deg>_b`);
1a and 1b solve a lens and a pose per camera and share the object, axis, tower, stops and
tiles; cam A holds the azimuth gauge. calibration.yaml keeps cam A at the top level (every
single-camera reader unchanged), the others under `cameras:`. Sweeps grab every enabled
camera at each stop (`camera_reference_b.yaml` for b); two BRIOs are told apart by serial
(`uvc.device.serial`; `python -m visual_calibration.sweep --list-cameras`). Before a hand
session `sweep --quick` checks one frame per camera at the park stop: over 1 px, a 5-stop
rotation session with the lens and tiles held (`Session.create(..., "rotation", hold=True)`).
The hand fit (live_fit, hand_study) adds every calibrated camera's views (meta.json `camera`).

## Hand model (starting point)

The source of truth is **`orcahand_description`** (`../orcahand_description`,
`main`, Diego's v2 fix of 2026-11-04/05: masses, joint positions, readable
names):

- `v2/models/urdf/orcahand_{right,left}.urdf`: 19 links, 17 revolute joints +
  `<side>_tower_fixed` (forearm → tower → wrist → carpals → `<finger>_abd/mcp/pip`,
  thumb `cmc/abd/mcp/dip`). Metres, radians.
- `v2/models/assets/{right,left}/*.stl`: one mesh per link (`I-PP.stl`,
  `I-FingerTipAssembly.stl`, ...), plus `*_Skin.stl` for the skins.
- `v2/models/mjcf/`: the same hand for MuJoCo; `v2/name_changes.csv`: old → new
  names.

Stale copies (older than that fix, MCP offsets up to 0.6 mm off, e.g.
`index_mcp` x -19.15 vs -18.75 mm):
- orca_core `orca_core/kinematics/data/v2_kinematics.yaml` (`HandKinematics`):
  regenerate with `tools/extract_urdf_kinematics.py` before fitting.
- orca_ui `frontend/public/assembly/movement/*/hand.urdf`: fine for the UI
  overlays, not for the solver.

## Hand CAD (in orca_ui) and the UI page

Rough CAD of the hand, for the solver's model and the overlays, lives in the
sibling repo `../orca_ui` (Diego's `merge/assembly-calibration-with-main`):

- `frontend/public/assembly/movement/{left,right}/`: `hand.urdf` + one GLB per
  link in `meshes/` (`link_tower.glb`, `link_forearm.glb`, `link_carpals.glb`,
  `link_<finger>_pp/ap/fingertip_assembly.glb`, thumb `tp/pp/ap/dp`),
  `manifest.json` (joints, links, rest-pose offsets; built from
  `orcahand_description`), `joint_map.yaml`. The kinematic model to project.
- `orca_ui/models/hand_v2/{left,right}/tower_spools.glb` + `tower_faces.json`:
  the tower, the reference for the camera/axis fit.
- `orca_ui/models/assembly/*.zip` (restored to `frontend/public/assembly/parts/
  <model>/assembly.glb`): full assembly GLBs, imported from STEP.
- STEP sources: `../orca_files/orca_v2/base/Orcahand_BaseV2_{Left,Right}.zip`,
  `../orca_files/orca_v2/touch/TouchV2_Assembly_{Left,Right}.step.zip`
  (orca_ui `HANDOVER.md`, "Model archive").

UI: orca_ui branch `visual-calibration`, Setup › Visual calibration
(`frontend/src/components/visual/VisualCalibrationView.tsx`), two step tabs:
**1 · Rig** (calibration object, no hand) and **2 · Hand**. For now each shows the
live camera only (browser `getUserMedia`, the BRIO by default, highest resolution).
Capture for calibration will need the backend (full-resolution stills, focus
and exposure locked over UVC).

Camera: Logitech BRIO turned on its side (portrait, 2160×3840) for the tall
hand + forearm, looking down 15-25° (sees curled fingers and the thumb; not
steeper than ~30°, or the hand hides the base); calibrate the lens as mounted.
The UI turns the stream 90° by default (0/90/180/270 buttons).

## Setup (decided)

- **One camera**, fixed to the rig frame. It never moves with the hand.
- **The hand turns** on a motor around a vertical axis (z up, so gravity pulls
  the same way in every photo). A mechanical hardstop sets 0°; the motor angle
  is known well.
- **Simple rig**: no glued QR codes or stickers as the reference. Clemens wants
  as few setup parts as possible.
- **Two steps, never combined** (2026-11-06):
  1. **Rig, without the hand**: a calibration object stands on the plate where
     the hand will stand, on the same dowels. Turn through every stop; fit the
     lens, camera pose, axis and each stop's angle. Once, and again whenever the
     camera is touched (compare to the last run).
  2. **Hand**: swap the object for the hand. Fit only the hand mount on the
     plate (6 DoF, small, from the tower) once per mounting, then the joints.
  Why: the object stands in the hand's place, so it is in the close camera frame
  and in focus, and it never hides the fingers; each step has few unknowns.
  Conditions: the camera stays put between the steps (rigid mount, focus and
  zoom locked); the hand's mount is repeatable (dowels on the tower).
- **The calibration object**: a printed square tube, 80 × 80 mm (about the
  palm's width), ~360 mm tall, in two halves (spigot joint, one-way key), on the
  hand's rods (Ø5.2 slip holes over both M5x80 rods, 4 radial M3x12). 8
  separate two-colour ChArUco tiles, each screwed on with 3 M3x12 (2 above the
  pattern, 1 below): sides 0 and 2 carry rows 0-2 (wrist, knuckles,
  fingertips), sides 1 and 3 row 1 only. The fingertip row leans back 25° on
  wedges, so not every tile normal is horizontal: with only vertical faces the
  focal length and the camera tilt trade off. Tiles 78 × 78 × 5 mm, 5 × 5
  squares of 14 mm, `DICT_4X4_250` 10.5 mm markers, 16 corners each
  (`tile_r<row>_s<side>`). Each tile's back carries a letter A-H and its seat
  on the tube the same letter (engraved, mirrored from outside): letter to
  letter, so a tile can't land in the wrong seat. Generator: `make_board.py`
  (printed set: `orca_files/calibration_rig/tiles/`, `set.yaml`). The tube:
  Shea, in the rig design.
- **Step 1 fits the lens too** (no loose board): lens + camera pose + stop
  angles + each tile's pose on the object (CAD start, ±1 mm/1°;
  `tile_r0_s0` held, it defines the object frame) in one bundle adjustment,
  ~22 stops × the visible tiles × 16 corners. Screwed tiles are not glued stickers: their
  placement error is fitted, not trusted. The lens is calibrated where the
  object turns, which is where the hand will be; the frame's edges are not
  (they never see the hand). The loose board stays as a fallback.
- **The axis lives in the dowels**: the object sits on the hand's dowels, so the
  axis relative to the dowels (tilt, offset) carries over to the hand.
- **The tower (the hand's base) is the check**: render it over the photos; if it
  lines up, the hand mount and the CAD agree.

Steps: rig with the calibration object (lens, camera, axis, stops) → hand on:
mount from the tower → least squares on the fingers.

## Decisions and why

- **Why the camera must stay still**: if it turned with the hand, every photo
  would be the same view. A fixed camera and a turning hand equals a camera
  walking around the hand.
- **Why not a sticker board as the reference**: its position on the rig comes
  from the CAD plus how it was stuck on. A stuck-on error (~0.2 mm or 0.1°
  ≈ 1 px) shifts every joint angle and the solver can't see it.
- **Why printed tiles instead**: printed features sit at their CAD positions
  by construction, nothing to stick. A large flat grid with many corners gives
  a far better pose than the tower's few, curved, partly hidden features.
  Residual risk: print scale (0.1-0.3 %) and flatness, measured once.
- **Why the solver also fits small rig errors**: the axis can be tilted or
  offset from the CAD, and the angle at each stop can be off slightly. Fitting
  them with priors, instead of trusting the CAD, removes the need for a precise
  rig.
- **Why rigid parts first**: a small camera error can look like every finger
  being a few degrees off, with a low pixel error. Fitting the camera and axis
  in step 1, without the hand, stops the joints from absorbing it.

## Unknowns in the solver

| Unknown | Count | Prior / constraint |
|---|---|---|
| Lens (fx, fy, cx, cy, k1, k2, p1, p2, k3) | 9 | step 1; held in step 2 |
| Camera pose (relative to the rig) | 6 | step 1, calibration object; held in step 2 |
| Tile poses on the object | 6 per tile, 7 tiles | step 1, ±1 mm/1° from the CAD; tile_r0_s0 held |
| Hand mount on the plate | 6 | step 2, small, from the tower; once per mounting |
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

## 2. Lock the camera

- [x] Lock focus (at the hand's distance), zoom, exposure and resolution;
      autofocus and stabilization off (`camera.uvc`, set by every sweep)
- [ ] The lens itself is fitted in step 3 with the calibration object; target
      < ~0.5 px on stops left out of the fit. Fallback: a loose ChArUco board
      across the frame at different tilts

## 3. Fit camera + axis on the calibration object (step 1 of the UI)

- [ ] Sweep 22 stops (0-315°, 15°) with any finger pose
- [ ] Fit lens + camera pose + axis tilt/offset + per-stop angle corrections
      + tile poses to the tile corners at every stop (4-6 tiles seen each); no
      hand on the plate
- [ ] Rough start: CAD camera position
- [ ] Then the hand on (step 2 of the UI): fit its mount from the tower, check
      by overlaying the rendered tower on each stop

| What the overlay shows | Likely cause |
|---|---|
| Same offset at every stop | Camera pose, or a pixel convention (half-pixel shift, flipped axis) |
| Error changes with the stop | Axis wobble or stop angle error |
| Error grows toward the frame edges | Lens calibration |

## First experiments: can we match one fingertip?

Each one answers one question, in order. Stop at the first that fails.

| # | Setup | Unknowns | Question |
|---|---|---|---|
| E0 | Synthetic: render the index finger from the URDF + STLs with a known camera, add 0.5 px noise | `index_mcp`, `index_pip` | Does the pipeline (FK, projection, conventions, solver) get back the angles we rendered? |
| E1 | Real, one photo, hand still at all-zero, plate at 0° | none (overlay only) | Does the projected URDF land on the real finger? The baseline offset in px |
| E2 | Real, plate still, the index finger side-on; sweep `index_pip` only, −10 to 100° in 10° steps | `index_pip` per photo | Fitted vs commanded angle: offset, gain, hysteresis (up vs down) |
| E3 | Same as E2 with 8 plate stops per pose | `index_mcp`, `index_pip`, then `index_abd` | Does turning the plate pin the joint toward the camera? |
| E4 | E3 with the silhouette/edges instead of one point | same | Does the automatic feature match the clicked one? |

- **The fingertip as one point** for E1-E3: click it by hand in the photos (a
  tiny UI tool). That keeps detection errors out of the geometry test; the
  detector comes in E4.
- **Expected precision**: at ~0.11 mm/px, a tip 40 mm from the PIP moves
  ~0.16° per px; a clicked point is good to ~1-2 px, so ~0.3° per photo.
- **E2 is the key test**: if fitted vs commanded is a straight line with a
  stable offset, joint calibration from photos works; the offset is the joint's
  zero error and a slope off 1 is a gain or tendon error.
- Code: `visual_calibration/`: `hand_model.py` (the orcahand_description
  URDF + STLs, FK, fingertip point), `camera.py` (pinhole + Brown distortion,
  OpenCV conventions; nominal BRIO portrait), `rig.py` (plate stops, hand mount:
  forearm +y up), `solve.py` (bounded Levenberg-Marquardt, numpy),
  `fit.py` (joints from point observations), `e0.py`
  (`uv run python -m visual_calibration.e0`, results to
  `~/Downloads/visual_calibration/e0/`). Test: `tests/test_visual_calibration.py`.

**E0 result (2026-11-06, 100 trials per row, occlusion ignored):**
- Works: bent finger, 8 stops, tip only, 0.5 px → mcp/pip RMS ≤ 0.07°;
  2 px (a clicked tip) → ≤ 0.3°.
- **The tip alone fails on a straight finger**: mcp and pip trade off (near
  straight, bending one way at the MCP and back at the PIP moves the tip
  almost not at all), up to 15° wrong, even with 22 stops. One more point (the
  PIP knuckle) fixes it: ≤ 0.07° at 0.5 px, ≤ 0.3° at 2 px, abd included.
- **Abduction of a curled finger** from the tip alone is weak (the tip lies
  near the abd axis): up to 4.5° at 2 px; with the PIP point ≤ 0.4°.
- **One side-on photo** works for a bent finger (≤ 0.5°), not for a straight one.
- **Rig error carries straight into the joints**: the camera off by 0.5 mm and
  0.1° (not fitted) → 0.1-0.3° on mcp/pip, 0.9° on a curled abd, and a
  7-9 px fit residual. Step 1 must pin the camera well below that.
- So for E1-E3: click **two points per finger** (tip + PIP knuckle), several stops.

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
