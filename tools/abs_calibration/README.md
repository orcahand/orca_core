# Absolute-calibration synthetic study (Phase 0.7)

Answers, before any rig is built: **can the laptop-tier pipeline (removable
dots + webcams + CPU solver) recover each joint's encoder calibration to
budget — and what do the injected model-error classes cost?**

Design doc: `plans/orca_external_encoder_calibration.md` (§9 Q2, §11 Phase 0.7).

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

## Production-facing pieces

- `dataset.py` — the session on-disk format: the seam between data producers
  (synthetic sim today, the camera/detection layer at Phase 1) and the
  solver. Occlusion (NaN) is part of the format.
- `solve_session.py` — CLI: session dir → `vision_calibration.yaml`
  (offset + per-ROM poly + sigma per joint) + `calibration_diagnostics.yaml`
  (axis disagreements with floors, solver info), with the per-joint prior
  lever as a YAML and a sigma persistence gate.
- Load-bearing behavior is pinned in `tests/test_abs_calibration.py`
  (masked VarPro, session round-trip, axis floor/recovery, sigma, de-novo).

## Structure

- `model.py` — perturbable chain FK (all link frames) + encoder truth model
- `sim.py` — scenario definitions and dataset generation
- `estimator.py` — the v1 estimator shape: base pose + per-joint
  `b0 + b1·u + b2·u²` curves, dot layouts eliminated by variable projection,
  per-joint MAP-prior lever, de-novo 1-D init, §4b axis diagnostic
- `study_synthetic.py` — scenario runner, metrics, results
