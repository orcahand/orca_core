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

## Phase-1 camera layer

Turns raw camera data into the session format. The full hardware flow:

```bash
# 1. shoot ChArUco calibration images (interactive; board fixed for extrinsics)
uv run python tools/abs_calibration/record_calibration.py CALIB_DIR \
    --camera cam0=0 --camera cam1=1 --camera cam2=2

# 2. calibrate the rig (world frame = the fixed board)
uv run python tools/abs_calibration/calibrate_rig.py CALIB_DIR --out rig.yaml

# 3. capture a session: sweeps + frames + encoders  [NOT yet hardware-tested]
uv run python tools/abs_calibration/record_session.py CONFIG --rig rig.yaml \
    --out CAPTURE_DIR --camera cam0=0 --camera cam1=1 --camera cam2=2

# 4. detect/track/triangulate/assign -> session dir
uv run python tools/abs_calibration/build_session.py CAPTURE_DIR \
    --rig rig.yaml --out SESSION_DIR

# 5. solve
uv run python tools/abs_calibration/solve_session.py SESSION_DIR
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
- `sweep_plan.py` — the shared sweep protocol (renderer + hardware)
- `calibrate_rig.py`, `build_session.py` — offline CLIs
- `record_calibration.py`, `record_session.py` — hardware capture
  (**written against the public hand API, not yet run on hardware**)
- `render_synthetic.py`, `validate_camera_layer.py` — image-level synthetic
  validation (below)

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
  (pinned in tests). Coarse next to the camera tier, and exactly enough
  to catch the known +25° `ring_abd` hardstop error.
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
    --modes abd,tip        # verify ABD_TOWARD_PINKY_SIGN once on hardware
uv run python tools/abs_calibration/solve_session.py SESSION \
    --manifolds tools/abs_calibration/contact_manifolds_right.yaml
```

Also **not yet run on hardware**; the stall thresholds and the
approach-sign table are the expected first-session tweaks (single
constants, loud failures).

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

**Findings (2026-08-12, full run at the physical rig geometry — PASS):**
intrinsics 0.10-0.14 px rms, focal < 0.4%; extrinsics within 0.25 deg /
3.3 mm (the farthest camera carries a small systematic at its 0.65 m board
distance — the end-to-end solve absorbs it, and the solve gates carry the
requirement); triangulated dots 0.99 mm rms / 0.87 mm p95 vs truth at
0.54 coverage with **zero misassigned columns**; end-to-end estimator
recovery (truth anchors) **0.135 deg mean / 0.31 deg max** — camera-only
error sits below the Phase-0.7 webcam-tier model-error band (E-class
means 0.24-0.84 deg), as budgeted. Rig-design lessons the failures
forced, now baked into the protocol, the scene, and the design doc: the
board stands 28 cm behind the hand (the wrist sweep swings fingertips
~15 cm backward), one camera must look from below-front (a curling finger
turns its dots floor-ward during exactly its own sweep), dots are ~6 mm
and 6-8 per link (density is what gives the rigidity voting its quorum —
identity theft concentrates on links with 1-2 surviving columns), dots go
on the phalanx sides (an axis-parallel normal is flexion-invariant), and
the capture held pose parks the thumb clear of the fingers (at the
natural neutral, thumb and finger dots pass within ~2 mm and steal
identities). Hardware guide with rendered expected views:
https://claude.ai/code/artifact/e5d349ab-e3cf-4ed6-ab9a-56dc0a354656

Results: `results/camera_validation.yaml` (gates included; regenerate with
`uv run python tools/abs_calibration/validate_camera_layer.py`).

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
