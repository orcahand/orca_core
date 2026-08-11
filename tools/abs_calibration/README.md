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

## Structure

- `model.py` — perturbable chain FK (all link frames) + encoder truth model
- `sim.py` — scenario definitions and dataset generation
- `estimator.py` — the v1 estimator shape: base pose + per-joint
  `b0 + b1·u + b2·u²` curves, dot layouts eliminated by variable projection,
  per-joint MAP-prior lever, de-novo 1-D init, §4b axis diagnostic
- `study_synthetic.py` — scenario runner, metrics, results
