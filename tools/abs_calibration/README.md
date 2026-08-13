# Absolute calibration: synthetic study (Phase 0.7) + camera layer (Phase 1)

Answers, before any rig is built: **can the laptop-tier pipeline (removable
dots + webcams + CPU solver) recover each joint's encoder calibration to
budget — and what do the injected model-error classes cost?** Then builds
the Phase-1 camera layer that turns real webcam images into the solver's
session format, validated end-to-end on rendered synthetic images.

Design doc: `plans/orca_external_encoder_calibration.md` (§9 Q2, §11).

## What it simulates

A perturbed "as-printed" truth generates the v1 observation classes; the
estimator only ever sees the **nominal** model, so every recovered-parameter
bias is caused by an injected error class:

| Class | Injection |
|---|---|
| Print geometry | joint-axis tilt, origin shifts, per-link direction bias, fingertip-point bias, palm-plane warp |
| Measurement systematics | global scale error + smooth 3D error field (distortion/reconstruction proxy), iid noise |
| Encoder | offset + gain + first two INL harmonics, 14-bit quantization |
| Prototype quirks | gross hardstop-init error on selected joints (de-novo init path) |

Scenario A injects nothing and must recover ≪0.1° — it is the machinery
self-test. B–F inject one class at a time, then combined, then combined with
gross init error.

## Deliberate simplifications

- **Observations live at the 3D level** (triangulated dots, measured link
  directions), not 2D pixels. The scale + smooth-field terms stand in for
  what the camera layer contributes systematically; the study targets *bias
  propagation through the estimator*, which is scale-preserved by this
  abstraction. The production estimator adds the 2D layer later.
- All dots visible in all frames (occlusion mostly reduces data volume, not
  structure).
- Plate contacts are placed observations, not simulated physical contact.

## Run

```bash
uv run python tools/abs_calibration/study_synthetic.py            # full
uv run python tools/abs_calibration/study_synthetic.py --quick    # smoke
uv run python tools/abs_calibration/study_synthetic.py --scenario E-webcam-combined
```

Results land in `tools/abs_calibration/results/results.yaml`; the summary
table prints per scenario with per-joint-class budget verdicts and absolute
fingertip error (which includes the *uncorrected* geometry error — the honest
v1 number).

## Findings (2026-08-11, full run)

- **A-ideal:** machinery recovers to ≪0.1° (wrist 0.008° mean), fingertip
  0.05 mm — self-test passed.
- **E-webcam-combined:** per-joint class means 0.24–0.84°, fingertip mean
  1.18 mm / p95 2.03 mm — matches the design doc's v1 targets; the 1 mm
  per-joint budgets are 3–7× exceeded by model error, not by conditioning.
- **B vs C:** print geometry alone costs ~0.9 mm fingertip; webcam-tier
  measurement systematics alone ~0.44 mm.
- **D vs E:** offsets-only trails offset+curve by only ~10% at this tier —
  geometry bias dominates INL here; the curve's value grows as the rig
  improves.
- **F:** de-novo init fully recovers 20°/12° gross hardstop-init errors
  (identical solution to E).
- **Base∘wrist degeneracy found and fixed:** no distal observation can split
  the wrist offset from base pose (constant rotation about the world-fixed
  wrist axis ⇒ absorbed by base). A mesh-referenced frame anchor on the
  forearm link is mandatory; with it, scenario A's wrist error fell 0.39° →
  0.008°.

## Axis diagnostic (resolved limitation)

The first implementation had a ~0.6° noise floor (per-frame Kabsch). The
current one expresses each dot's sweep track in the solved model's
reference-link frame (held-joint drift is visible to the encoders, so the
model reference is smooth) and fits one common axis to all tracks jointly
(smallest eigenvector of summed track covariances). Flexion sweeps resolve
~0.1°; short abduction arcs with near-parallel dot tangents are honest only
to ~0.5–1°, and the diagnostic **reports its own per-joint noise floor**
(eigen-perturbation) so a reading is only ever interpreted against it. Rig
protocol note: sweeping abd at 2–3 different flexion postures diversifies
the tangents and would bring abd floors down to the flexion class.

`sigma_deg` from the solver is statistical only (data sufficiency /
convergence gate); the synthetic study is what quantifies model-error bias,
which dominates it.

## Phase-1 camera layer + Phase-2 anchors

Turns raw camera data into the session format and measures the absolute
anchors. The full hardware flow:

```bash
# 1. shoot ChArUco calibration images (interactive; board fixed for
#    extrinsics; a hand-held assist camera gets an intrinsic-only dir)
uv run python tools/abs_calibration/record_calibration.py CALIB_DIR \
    --camera cam0=0 --camera cam1=1 --camera cam2=2

# 2. calibrate the rig (world frame = the fixed board; camera dirs without
#    extrinsic/ become assist cameras: per-shot board self-localisation)
uv run python tools/abs_calibration/calibrate_rig.py CALIB_DIR --out rig.yaml

# 3. capture a session: sweeps + anchor poses + encoders; --assist-camera
#    prompts hand-held shots at each anchor pose  [NOT yet hardware-tested]
uv run python tools/abs_calibration/record_session.py CONFIG --rig rig.yaml \
    --out CAPTURE_DIR --camera cam0=0 --camera cam1=1 --camera cam2=2

# 4. detect/track/triangulate/assign -> session dir (dots only)
uv run python tools/abs_calibration/build_session.py CAPTURE_DIR \
    --rig rig.yaml --out SESSION_DIR

# 5. detect the Phase-2 anchors into the session (dir_obs + frame_axes,
#    with per-anchor sigmas; preliminary dots-only solve for prediction)
uv run python tools/abs_calibration/detect_anchors.py SESSION_DIR \
    --rig rig.yaml [--assist-camera assist0] [--priors priors.yaml]

# 6. (optional) plate contacts at several wrist angles [NOT yet hardware-tested]
uv run python tools/abs_calibration/record_plate.py CONFIG --rig rig.yaml \
    --plate plate.yaml --out PLATE_SESSION --camera cam0=0
#    (print the plate: python plate.py --scad plate.scad --yaml plate.yaml)

# 7. merge whatever sessions exist (camera + contact + plate) and solve
uv run python tools/abs_calibration/merge_sessions.py MERGED \
    SESSION_DIR PLATE_SESSION CONTACT_SESSION --same-mount
uv run python tools/abs_calibration/solve_session.py MERGED \
    --plate-holdout 0.3
```

Design points, each load-bearing:

- **The sweep protocol carries dot identity** (`sweep_plan.py`): dots are
  uncoded, so identity lives on temporal continuity. Every sweep is an
  eased triangle from/to the held pose (velocity zero at reversals, steps
  sized per joint by lever arm) so consecutive frames are always one small
  step apart and a constant-velocity tracker never has to guess across a
  jump. Static-step capture (settle, then read encoders + grab all cameras)
  is what makes unsynchronized webcams mutually consistent.
- **Track-level association**: 2D tracks are matched across cameras by
  90th-percentile Sampson distance over shared frames, grown best-first
  under two vetoes — per-camera temporal exclusivity, and 3D continuity
  with the component's triangulated anchor track (epipolar consistency
  alone cannot separate two dots when one is occluded).
- **Identity swaps are killed by rigid-group voting** (`rigidity_filter`):
  during one joint's sweep the hand is exactly two rigid bodies, so all
  pairwise dot distances within each side are constant. Per-frame votes
  catch transient mismatches; per-epoch votes catch both cameras handing a
  track to the same neighbouring dot at once (self-occlusion events are
  pose-correlated across cameras — those runs are smooth, epipolar-clean,
  and wrong in depth).
- **Links are identified by motion signature** (`assign.py`): the set of
  sweeps that move a track equals its link's ancestor set — mesh-free and
  calibration-free. Sweeps a track didn't properly see (occlusion, or too
  little of the joint's travel) are unknowns; only links whose full
  signature was exercised are candidates; a chain-ambiguous track (dot
  hidden during exactly its own distal sweeps — fingers curl their dots
  away) is parked on the most proximal equivalent link with the
  non-equivalent frames masked. Static tracks (forearm, board features,
  mount clutter) all land harmlessly on the forearm link, where an unknown
  static layout is pure gauge.
- The board defines the world frame and the metric scale — verify the
  printed square size with calipers and record it as
  ``measured_square_m`` in a board YAML.

### Files

- `board.py` / `cameras.py` — ChArUco spec/detection/intrinsics/pose;
  camera + rig model, YAML persistence, triangulation, epipolar geometry
  (`python board.py --out board.svg` writes a print-exact board with a
  caliper ruler)
- `dots.py` — blob detection, per-camera tracking, association,
  triangulation, rigidity/despike filters
- `assign.py` — motion-signature link assignment
- `sweep_plan.py` — the shared sweep protocol + anchor-pose plan
  (renderer + hardware)
- `anchors.py` — Phase-2 anchor detection: mesh-guided contour refinement
  (below)
- `detect_anchors.py` — anchors into a session: preliminary solve,
  detection, de-novo wide scan, assist-shot self-localisation
- `plate.py`, `record_plate.py` — dimple plate: printable geometry, tag
  pose, contact capture (hardware capture untested, like its siblings)
- `merge_sessions.py` — contact + camera + plate sessions into one dataset
- `calibrate_rig.py`, `build_session.py` — offline CLIs
- `record_calibration.py`, `record_session.py` — hardware capture
  (**written against the public hand API, not yet run on hardware**)
- `render_synthetic.py`, `validate_camera_layer.py` — image-level synthetic
  validation (below)

## Phase-2 anchor detection (`anchors.py`)

Dots alone leave every per-joint offset as pure gauge (a constant joint
shift is absorbed exactly by the unknown dot layout). The anchors pin them
against the printed surface itself: the URDF mesh, posed by a preliminary
dots-only solve (offsets held by priors — all a prediction needs), is
aligned to the observed silhouette edges. One mechanism serves every
anchor class — model-guided sparse-contour pose refinement:

- **Rim extraction**: mesh points with view-grazing normals, z-buffer
  visible, on the actual occluding contour (stepping outward must leave
  the body's own projected footprint — a flat face seen edge-on passes
  the grazing band but is not contour), backdrop far enough behind
  (another body within ~12 mm makes the edge ambiguous), silicone-skin
  zones excluded.
- **Edge search** along each contour normal, sub-pixel, polarity-free,
  with an ambiguity veto — and with the board's checker/marker gradients
  masked out: the hand stands in front of the board, but the board pose
  is exactly known (it IS the world frame), so its texture transitions
  are predictable clutter.
- **Damped multi-view Gauss-Newton** over the group pose with three
  nuisance classes: radial inflation (print oversize / blooming — one
  scalar, else the solver converts it into depth drift along the weakest
  direction), per-view 2D offsets (assist shots carry the planar-target
  tilt ambiguity of board self-localisation; fixed cameras get a tight
  prior), and priors that hold the directions contour cannot see
  (cylinder roll, axis slide when the end edges are occluded).
- **Honest failure**: edge-count, fit-rms and rotation-information gates
  fail an anchor into NaN with a reason — never a silent prediction. Each
  surviving anchor ships its own sigma (`dirs_sigma_*.npy`), which the
  estimator uses as the residual weight (floored at the 0.1 deg class
  default), so a marginal anchor informs instead of dominating.
- **De-novo wide scan** (priors `mode: off`): a grossly wrong offset —
  e.g. a misplaced encoder sensor — puts the predicted contour outside
  the search window or onto a neighbour's edges; a 1-D scan of the offset
  against edge support finds the basin before refinement.

Anchor classes produced: `dir_obs` (phalanx link z directions, all anchor
poses), palm dorsal-shell frame anchor (per anchor pose), and the forearm
tower frame anchor (mandatory — the only observation that splits the wrist
offset from base pose; static, so measured once over views pooled across
anchor frames).

### Rig geometry (validated in the synthetic scene)

Board upright BEHIND the hand facing the cameras (world frame + metric
scale; the printed face must face them — ArUco is not mirror-invariant),
hand ~28 cm in front so the wrist sweep clears it. Four cameras 52–60 cm
from the hand on a wide arc, one of them **below-front looking up** —
curling fingers turn their dots floor-ward during exactly their own
sweeps. Dots on phalanx sides as well as dorsal (an axis-parallel normal
is flexion-invariant), none on skin. Capture held pose parks the thumb
clear of the fingers (`record_session.py --park`): at the natural neutral
thumb and finger dots pass within ~2 mm and steal identities.

## Camera-free contact mode (no webcams at all)

Every session observation class is optional; `record_contacts.py` fills a
session with **contact events** instead of dots — same solver, same
outputs, per-joint `sigma_deg` reporting honestly what the data could
determine:

- **abd-block** — park one finger at its abduction stop (the stop is
  stiffness, nothing more: both encoders are READ at contact, no hardstop
  angle is assumed), drive the neighbour in under a weak current limit
  until its encoder stalls; the pair then sits on a mesh contact manifold
  precomputed from the description URDF (`build_contact_manifold.py`,
  table committed as `contact_manifolds_right.yaml`; skin surfaces, sigma
  = fit + print + skin compliance ≈ 1.6–3.4°). Pairwise both directions
  at TWO flexion postures — one contact curve pins only an offset
  combination; the second posture's differing slope separates the pair
  (pinned in tests). Coarse next to the camera tier — and coarser than a
  healthy hardstop anchor (~1° bias class), so its role is
  **verification, not replacement**: it independently measures
  encoder-reported angle against mechanical truth and flags anything in
  the multi-degree class, e.g. a misplaced encoder magnet/board (the
  `ring_abd` +25° reading on ser-9964 is believed to be exactly that —
  a sensor-placement issue, to be fixed at the source).
- **tip-press** — full hands: torque off, press the prompted fingertip
  pair together by hand; auto-captures when both pads localise >0.35 N
  with still encoders (per-pad contact centroid + forces). Pad-point
  coincidence residual with force-dependent sigma — the Phase-3 residual
  pulled forward, hand-guided.
- **The wrist is constrained by neither** (no self-contact crosses it);
  it keeps the hardstop calibration and its sigma says so. Only the
  camera tier's forearm anchor reaches the wrist.

```bash
uv run python tools/abs_calibration/record_contacts.py CONFIG --out SESSION \
    --modes abd,tip        # --dry-run first: prints the derived directions
uv run python tools/abs_calibration/solve_session.py SESSION \
    --manifolds tools/abs_calibration/contact_manifolds_right.yaml
```

Approach directions are derived from the packaged kinematic model per
pair (the dry-run prints them; on the right hand, toward-the-thumb is
the decreasing-angle direction for every finger, matching the hardstop
calibration's first drive). Also **not yet run on hardware**; the stall
thresholds are the expected first-session tweak (single constants, loud
failures).

### Validation on rendered images (`validate_camera_layer.py`)

Renders ChArUco calibration views and dot sweep frames through cameras with
realistic focal lengths and lens distortion (plus the board and a crude
hand body in-frame as clutter), then runs the *production* pipeline on the
images and scores every stage against truth: recovered intrinsics/
extrinsics, triangulated dots vs truth (rms/p95/coverage), link-assignment
integrity, and estimator offset recovery on the built session (with
truth-derived Phase-2 anchors standing in). `--quick` runs a reduced
wrist+index+thumb scene and gates only pipeline mechanics — 1-2 surviving
dots per link make its solve statistically thin by construction.

**Findings (2026-08-12, two validation tiers).** The scene renders the
hand from the REAL URDF meshes (skin zones excluded from dot placement)
with z-buffered self-occlusion — finger-over-finger, palm-over-thumb, the
works.

*Tier A — heuristic visibility (dot normals only): PASS.* At the physical
rig geometry: intrinsics 0.10-0.14 px, extrinsics within 0.25 deg/3.3 mm,
dots 0.99 mm rms / 0.87 mm p95 at 0.54 coverage, zero misassigned,
end-to-end estimator recovery **0.135 deg mean / 0.31 deg max** — the
camera-only error budget holds with margin.

*Tier B — true mesh self-occlusion: geometry and identity hold, coverage
does not (OPEN).* Dot geometry stays at 0.6 mm rms with at most one
misassigned column, but deep flexion hides each finger's dots from every
static camera in turn: tracks fragment at every occlusion blink, distal
joints starve, and their solve degrades (per-joint coverage is the
binding constraint, not accuracy). Probe-driven search over camera sets
showed **no 4-camera arrangement keeps flexed fingertips covered; a fifth
camera nearly under the hand looking up is decisive** (mean own-sweep
coverage 0.51 vs <=0.34 for every 4-camera set) and is now the prescribed
rig. 7 dots/link is the measured optimum (10 crowds the image). Remaining
mitigations, in order: a hand-held assist camera during flexion sweeps
(per-frame board self-localisation — pipeline extension), model-informed
gap-bridging in association, and assist postures (repeat pip sweeps at a
second wrist angle to change the occlusion geometry).

**Camera count is a dial, best-effort by construction** (probe-measured
own-sweep coverage / dead joints): 2 cams 0.15/9 (use contact mode
instead), 3 cams [side-profile + under + below-front] 0.30/0 — the
sensible minimum, 4 (+thumb-side) 0.34/0, 5 (full arc, the benchmark
layout) 0.44/0. A side-profile camera earns its place in every small
set. Measured negative: the one-finger-at-a-time protocol
(`record_session.py --clear-fingers`, park other fingers curled) sounds
right but is net-worse — parked fists block the under camera and no side
camera sees through the finger row; the held spread already separates
the fingers. Kept as a flag, off by default.

Rig-design lessons baked into the protocol, the scene, and the design
doc: board 28 cm behind the hand (the wrist sweep swings fingertips
~15 cm backward), cameras per the dial above, ~6 mm dots 6-8 per link on
the phalanx sides as well as dorsal (axis-parallel normals are
flexion-invariant), thumb parked clear of the fingers, clutter test
points on the actual mount surfaces. Hardware guide with mesh-rendered
expected views:
https://claude.ai/code/artifact/e5d349ab-e3cf-4ed6-ab9a-56dc0a354656

Results: `results/camera_validation.yaml` (gates included; regenerate with
`uv run python tools/abs_calibration/validate_camera_layer.py`).

### Phase-2 findings on rendered images (2026-08-13, splat tier, seeds 0-2)

The validator now runs REAL anchor detection (no truth stand-ins) and
scores it against truth. Ranges below span three full-scene seeds —
mesh-tier realizations vary substantially and one roll is never the
number:

- **The anchors transformed the open coverage problem's solve**: with
  detected anchors + per-anchor sigma weighting the full-tier solve
  lands at **0.49-1.1 deg mean / 1.4-5.0 deg max** on clean-assignment
  realizations (seeds 0, 2), against 5.6 / 48 recorded with the old
  truth stand-ins on the same open misassignment/coverage issue.
  Misassigned columns are 0-1 with the solid-splat renders (was 0-3),
  always the adjacent-finger class. The degree-class joints are exactly
  the sigma-flagged ones (pips with 1-3 dot columns and no surviving
  anchors; starved abds), which the solve_session sigma gate refuses to
  persist. **The remaining bite of the open association item, measured**
  (seed 1): a stolen column on a chain whose dir anchors ALSO starved
  (index got 0 this realization) still blows that chain up (index_abd
  30 deg at overconfident sigma — a wrong local minimum's Jacobian
  says nothing) — anchors rescue misassignment where anchor coverage
  exists (seed 0's pinky recovered exactly so), which sharpens the
  queued mitigations (gap-bridging, assist postures) as the remaining
  coverage work.
- **Splat-tier detection floors** (the render's ragged, background-
  dependent silhouette bounds edge detection at the degree class; real
  optics are sub-pixel): dir anchors 2.8-4.1 deg mean at ~0.25
  availability (mid-flexion frames lose distal links to occlusion —
  extended anchor rows carry most survivors); palm frame anchor 0.5-2.4
  deg over 8-10 frames; assist self-localisation 0.31-0.54 deg /
  1.0-1.9 mm max over 30/30 shots at full resolution (the quick tier's
  1.1 deg was a resolution artifact).
- **The tower anchor inherits the preliminary base-wrist split at this
  tier**: seed 0 measured 1.32 deg error about an axis at |cos| 0.997 to
  the wrist axis — identically across every view-pooling variant; seed 1
  split 1.9 deg, seed 2 only 0.3 deg (and its solve hit 0.49/1.4 — the
  doc's target class, proving the mechanism delivers when the split is
  small). Mechanism: dots leave base∘wrist gauge, the prelim splits it
  arbitrarily within priors, and the contour signal of a rotation about
  the tower's own long axis (~2-3 px) sits below this tier's edge noise
  (~4 px rms, correlated), so the refinement holds its prior. When the
  tower anchor is MISSING entirely the solve degrades to 3.3/30 (seed 1
  before the rms-gate fix) — the doc's "forearm anchor is mandatory"
  measured directly. Sub-pixel hardware edges resolve the about-axis
  component 4-8x better; the 0.2-0.4 deg tower class stands as the
  hardware expectation. Detected sigmas remain STATISTICAL (doc §12) —
  the truth-comparison is what measures bias, and at this tier bias
  dominates sigma for the tower.
- Detection runs on CPU in ~2-4 min per session on top of the dot
  pipeline; the de-novo wide scan is exercised in unit tests (20 deg
  gross-offset recovery) and stands ready for the ring_abd-class faults.

## Production-facing pieces

- `dataset.py` — the session on-disk format: the seam between data producers
  (synthetic sim and the camera layer) and the solver. Occlusion (NaN) is
  part of the format.
- `solve_session.py` — CLI: session dir → `vision_calibration.yaml`
  (offset + per-ROM poly + sigma per joint) + `calibration_diagnostics.yaml`
  (axis disagreements with floors, solver info), with the per-joint prior
  lever as a YAML and a sigma persistence gate.
- Load-bearing behavior is pinned in `tests/test_abs_calibration.py`
  (masked VarPro, session round-trip, axis floor/recovery, sigma, de-novo)
  and `tests/test_camera_layer.py` (triangulation, board pose, tracking
  identity rules, association vetoes, rigidity/despike filters, signature
  assignment, sweep-protocol continuity).

## Structure (Phase-0.7 study)

- `model.py` — perturbable chain FK (all link frames) + encoder truth model
- `sim.py` — scenario definitions and dataset generation
- `estimator.py` — the v1 estimator shape: base pose + per-joint
  `b0 + b1·u + b2·u²` curves, dot layouts eliminated by variable projection,
  per-joint MAP-prior lever, de-novo 1-D init, §4b axis diagnostic
- `study_synthetic.py` — scenario runner, metrics, results
