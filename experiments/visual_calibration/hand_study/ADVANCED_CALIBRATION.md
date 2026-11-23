# Advanced calibration: a physical model (planned, not run yet)

Planned on 2026-11-08, to run at a later stage on the overnight study's data
(`~/.orca_ui/visual_calibration/hand_study/`). It is compared with the three models
already fitted.

## What is fitted today

| Model | What it is | Held-out RMS (V0) |
|---|---|---|
| a) old config | orca_core's linear map between the calibrated motor stops and the configured ROM | 3.9° |
| b) new hard stops | the same linear map, with the two stop angles per joint refitted by least squares (config change only) | 3.0° |
| per-joint map | a 1-D piecewise-linear curve per joint, own command only (`joint_map.json`, used by the orca-ui dashboard toggle) | — |
| c) LUT | per joint: a 1-D table over its own command (9 knots), plus additive 1-D tables over its partners (same finger, wrist, significant couplings; 4 knots each), plus a 2-D table over the finger's flexion pair. Fitted by regularised least squares and inverted for control (`lut.py`) | 2.5° |

Hysteresis is not modelled. It is only avoided: every pose is approached from the same side,
through a via pose 8° below.

## Proposal: a small physical model with few parameters per joint

The aim is to generalise better than big tables, and to refit each hand quickly.

1. **Per joint:** a smooth, gently non-linear curve from motor to joint angle, because the
   pulley and tendon moment arms change with the angle. About 3–4 parameters, e.g. a
   low-order spline or a polynomial.
2. **Tendon coupling matrix:** the wrist changes the tendon path length of every finger
   flexor, and mcp and pip share tendons. Take the coupling ratios from the CAD (moment arms)
   and fit only the gains. The controller then compensates the finger motors when the wrist
   moves. This would also avoid tonight's wrist jam at −65° with all fingers at their
   extension stops.
3. **Load and compliance term:** use the motor current as a measure of tendon force, and
   model the stretch as Δq ≈ k_j · current. This covers pinches, presses and holds.
4. **Hysteresis as a backlash (play) operator:** a deadband h_j per joint (measured 3.5–7° on
   index_mcp, thumb_mcp, thumb_dip and middle_pip). The controller compensates it by the
   direction of the last motion, or always finishes an approach from the same side.
5. **Calibration per hand:**
   - Factory: a short camera run (~15–30 min) fits the ~6–8 parameters per joint.
   - Customer, no camera: orca_core's hard-stop calibration, plus the tactile touch
     calibration (thumb to each fingertip, contact onset from the zeroed tactile signal,
     "Super Advanced Calibration") to refit offsets and drift.

## How to evaluate

- The same held-out V-blocks as the study (V0–V6): RMS and max per joint, and fingertip
  error in mm.
- The Kapandji test: distance, touch yes/no, force.
- Compare with a), b) and c), and with the number of parameters.
- Robustness: refit on the dense data only and test on V3–V5 (after the wear, the load and
  the recalibration).
- Decide whether the tactile-only customer recalibration keeps the accuracy without the
  camera.
