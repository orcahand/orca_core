# Hand calibration study — overnight run, ser-4485 (orcahand-touch-right), 8–9 Nov 2026

Code: `experiments/visual_calibration/hand_study/` (run with orca_ui's venv,
`PYTHONPATH=<experiments/visual_calibration>`). Data: `~/.orca_ui/visual_calibration/hand_study/`
(`sessions/20261108-night-N##-*`, `night/20261108-night.{json,state.json,log}`, `calib_run*`,
`proposed/`). Report page: `python -m hand_study.server` → http://127.0.0.1:5010.
orca_core itself (its calibration and core code) is untouched; its standard calibration is only
*run* (through orca-ui), and the proposed config is a copy.

## Goals

1. **Motor → joint, precisely**: every joint over its full range including the hard stops,
   couplings (which joint depends on which), hysteresis on a subset.
2. **New hard stops** (the customer deliverable): keep orca_core's linear map, change only the
   config's `joint_roms` (the angles the map spans), fitted per joint by least squares on the
   true (vision) angles. Accuracy per joint and overall (RMS, max) for a) old config,
   b) new stops (linear), c) full LUT with couplings; one closed-loop check with the new stops.
3. **Robustness / drift**: how often must a customer recalibrate? Wear phases, validation after
   each, orca_core's standard recalibration, then a short (~50 min) visual calibration.
4. **Rig / vision learnings**: hard-to-see joints and views, camera, URDF fixes, shading/creases.
5. **Kapandji test**: thumb tip to each fingertip under old / new-stop / LUT maps: vision
   distance, tactile contact and force.
6. **Super Advanced Calibration (SAC)**, only after a complete main study with a good fit:
   thumb–finger contacts found by touch as closure constraints in the fit.

## Protocol (night/20261108-night.json, built by `python -m hand_study.night_plan 20261108-night`)

| start | block | what | poses | min |
|---|---|---|---|---|
| 0:00 | calibrate-start | orca_core standard calibration (unchanged) → `calib_run2` | | 1 |
| 0:01 | N00-baseline | zero pose | 1 | 1 |
| 0:02 | N01-dense | 17 joints swept over the full ROM (7–9 points, ends = configured stops); grids mcp×pip (4 fingers), thumb mcp×dip, cmc×abd, wrist×index_mcp, wrist×middle_pip, abd×mcp (index, ring); hysteresis subset (3 references × below/above × 3 visits); 22 random | 263 | ~180 |
| ~3:00 | N02-V0 | validation set (below) | 25 | 17 |
| | wear-cycle-1 | 30 min open/close cycling (fist, open, spread, open, thumb in, open; 60°/s) | | 30 |
| | N03-V1 | validation | 25 | 17 |
| | wear-cycle-2 | 30 min cycling | | 30 |
| | N04-V2 | validation | 25 | 17 |
| | wear-loaded | 1 h loaded: thumb-to-finger pinches pressed 6° further, neighbouring fingers squeezed (abd), every joint held 1° inside its configured stops (4 s holds) | | 60 |
| | N05-V3 | validation | 25 | 17 |
| | recalibrate | orca_core standard calibration again → `calib_run3` | | 1 |
| | N06-V4 | validation | 25 | 17 |
| | N07-short | short visual calibration: 4 points per joint + 6 random | 72 | ~50 |
| | N08-V5 | validation | 25 | 17 |
| | N09-V6 | closed loop: the validation set commanded through the NEW hard stops (fitted on N01) | 25 | 17 |
| | N11-kapandji | 4 touches × 3 maps (old, new stops, LUT), 5°/s final approach, tactile, torque off after each | 12 | 11 |
| | N12-sac | SAC contacts (4 pairs × 5 approaches) + fit + Kapandji under the SAC stops | ~24 | 25 |
| | N10-extra | random poses until the deadline | ≤150 | rest |

**Validation set** (identical in every V block, `night_plan.validation`): open ×2, 12 random
(seed 99), 8 at configured stops (abd fans at an end, single flexors at the top), the thumb at
its stops, 3 references (pinch, mid, mix).

**Same approach side**: every pose (except the hysteresis repeats, which come from both sides)
is reached through a via pose 8° below it on every joint (4° if that collides).

**Views**: 5 plate stops (45, 112.5, 180, 225, 292.5°), stop-and-shoot at 10°/s, alternating
direction. Continuous capture was *not* used: the frame latency is unknown and 1° of plate angle
is ~1.7 mm at the fingertips, more than the vision noise; to be tested in daytime.

**Camera**: UVC controls locked to the rig config (manual exposure 100, WB 5000 K, focus 60)
before every block. Until 8 Nov the camera ran on auto exposure / white balance; the locked
preflight zero pose fits as before (8.15 vs 8.07 px, same mask quality).

**Mount**: the pooled mount over all earlier poses (`mount_bundle.json`) held fixed.

## Closed loop (N09-V6) without touching orca_core

The new config is a copy (`proposed/config.yaml`). Instead of reloading orca-ui with it, every
pose is commanded so the motor goes exactly where orca_core with the new config would send it
after its calibration: `m = m0 + t (m1 - m0)`, `t = (q - lo') / (hi' - lo')`, with this run's
motor limits (m0, m1); the command sent through the unchanged config is `q_old = map_old(m)`.
This is the customer case "new config file + standard calibration" exactly (orca_core
recomputes the ratio as (m1 - m0)/(hi' - lo') in its calibration).

Important for customers: the new numbers act fully only after a calibration with the new config;
without one only the offset moves (the stored ratio stays).

## Safety (unattended)

- Hand and plate never move together; plate torque off after every move (lease), hand torque
  off after every block, on any error and at the end.
- Guard before every pose / every 60 cycles: torque still on (else: e-stop → abort), no motor
  hardware error, hottest motor < 65 °C (shutdown at 70, wrist 80), > 8 GB disk free.
  ≥ 60 °C: torque off until ≤ 45 °C (max 30 min, else abort).
- A 409 on a joint target (torque gone / control taken) aborts; other errors are retried 3×
  (60 s apart, turntable reconnected if it was the plate). An abort is never retried.
- `watchdog.py`: torque off (hand + plate) when the runner's process is gone or its heartbeat
  is older than 180 s. The orca-ui E-STOP works as always.
- `caffeinate -dims -w <runner pid>` keeps the Mac awake.

Dry runs: mock orca-ui (all step types, watchdog after kill -9); real rig (calibration, dense
and V poses with via, wear bursts, closed loop, Kapandji, SAC, temperature guard, watchdog).

## Analysis plan (morning)

- Solves: `hand_study.worker` (5 processes, started with the runner).
- Per joint motor vs joint: measured, old linear, new-stop linear, LUT.
- `hardstops.py` → `proposed/{config.yaml, config.diff.txt, hardstops.json}`, accuracy a/b/c on
  the dense block (train) and on V0 (held out); V6 = closed-loop accuracy of b.
- Drift: per joint error of each map on V0…V6 (same poses; V4+ after recalibration).
- Recalibration question: V3 → V4 for a) and b): does orca_core's calibration restore accuracy
  and do the new stops still hold after it?
- Short vs long: LUT / stops fitted on N07 vs on N01, both evaluated on V5.
- Couplings heatmap, hysteresis (repeats), per-joint visibility (view-split noise per view).
- Kapandji: thumb-tip ↔ fingertip distance from the solved joints (URDF), tactile contact/force.
- SAC (`sac_fit.py`): stops refitted with the contacts; with vs without on V blocks, leave-one-out
  over contacts, Kapandji under SAC.

## Results

_(filled in the morning)_

### New hard stops (old → new)

### Accuracy a / b / c

### Drift V0 … V6

### Kapandji

### Super Advanced Calibration

### Rig / vision / URDF recommendations
