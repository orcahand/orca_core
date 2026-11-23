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

Numbers: `report/results.json`, `report/findings.md`, CSVs in `report/` (`python -m hand_study.report
20261108-night --loo`). 660 poses solved; 10 vision blunders (fit > 9.5 px) and 3 suspect masks left
out; a joint whose half-view solves differ > 12° is not counted. Errors are true (vision) − predicted,
all joints pooled: RMS / max, and median / p95 where the max is a vision failure.

**Run as executed.** 02:05–06:50 as planned up to N04-V2; a wrist overload at 06:50 (after
v-stop-flex-lo) aborted the runner; resumed 09:12. At 10:30 the wrist jammed again in N05-V3; its
reboot lost a turn (extended position mode, the motor sat past 2π), so the poses after it were taken
in a broken frame: moved to `faulted/20261108-night-N05-V3-wrap/` and left out. The Mac then rebooted
(10:34, Jetsam: the macOS video decoder at 81 GB, see the camera note below). Restarted 10:49 with the
recalibration (run3) *before* finishing V3, so V3 is split: **V3-pre** (20 poses, run2, after all
wear) and **V3-post** (5 poses, run3). v-stop-flex-lo (wrist −65° with every finger at its extension
stop) jammed the wrist in 5 of 5 V blocks and was dropped from V4–V6. N10-extra was stopped at
130/150 poses by request. 6 motor faults, all wrist, all on that pose or right after it.

### New hard stops (old → new)

Fitted on N01-dense (`proposed/config.yaml`, `config.diff.txt`); 14 of 17 joints move ≥ 1°:
thumb_cmc [−45, 33]→[−46.0, 33.0], thumb_abd [−18, 55]→[−17.1, 59.7], thumb_mcp [−25, 100]→[−21.5, 102.1],
thumb_dip [−15, 107]→[−11.2, 108.2], index_abd [−25, 30]→[−21.3, 27.0], index_mcp [−25, 100]→[−35.9, 103.8],
index_pip [−15, 107]→[−18.8, 113.9], middle_mcp [−25, 100]→[−27.8, 103.8], ring_abd [−27, 27]→[−28.5, 26.6],
ring_mcp [−25, 100]→[−22.4, 102.1], ring_pip [−15, 107]→[−16.3, 108.3], pinky_abd [−30, 30]→[−34.1, 29.6],
pinky_mcp [−25, 100]→[−19.9, 106.1], pinky_pip [−15, 107]→[−14.3, 103.3].
Fitted on the 50 min short block instead (4 points per joint, after the recalibration) most ends
agree within 3°; the flexion ends differ most (middle_mcp 5.3°, thumb_mcp 5.0°, index_pip 4.6°,
pinky_mcp 4.5°) — yet on V5 the short stops are as accurate (2.76 vs 2.88).

### Accuracy a / b / c

| set | a) old config | b) new stops | c) LUT | b + SAC |
|---|---|---|---|---|
| N01-dense (train) | 3.22 / 17.9 | 1.73 / 16.4 | 1.38 / 10.7 | 1.72 |
| N02-V0 (held out) | 3.90 | 2.96 | 2.49 | 2.93 |
| V0 median / p95 | 2.18 / 8.4 | 1.06 / 6.7 | 0.99 / 5.5 | 1.06 / 6.3 |
| N10-extra, 124 random poses | 4.22 / 28.9 | 3.84 / 29.0 | 3.93 / 36.0 | 3.78 |
| N10-extra, maps fitted on N07-short | | 3.61 | 3.41 | |
| N09-V6 closed loop (true vs wanted) | | 3.43 (median 1.57) | | |

- **The new stops are the deliverable**: −24 % RMS and half the median error on held-out poses,
  in every block, before and after recalibration. Biggest gains pinky_mcp (−4.0°), thumb_mcp (−2.6°),
  index/ring_mcp (−1.5°); no joint gets worse by more than 0.25° (index_pip).
- **Closed loop** (orca_core with the new config, emulated): 3.43° RMS from the wanted angle, vs
  3.79 for the old map on the same poses; the 0.5° beyond b's prediction is the motor not reaching
  its target under load.
- **The LUT is tied to its calibration run**: best while run2 holds (V0–V3 2.5–2.8), worse than the
  stops after orca_core's recalibration (V4–V6 3.2–3.4) because it maps commanded angles. Refitted on
  the short block (same run) it is best again (V5 2.59, N10 3.41). A LUT is only worth shipping if it
  is refit with every calibration, or rewritten in motor space.
- **Random poses (N10-extra) are harder for every map** (3.6–4.2): combined flexion with occlusion;
  92 of 2108 joint observations are unseen and a few mcp/pip swaps of the vision fit cause the maxima.

### Drift V0 … V6

RMS (b, new stops): V0 2.96 → V1 3.06 → V2 3.02 (two 30 min cycling phases) → **V3-pre 3.42** (after
1 h loaded) → recalibration → V3-post 2.35, V4 2.96, V5 2.88, V6 2.95. a) old: 3.90, 3.95, 3.92, 4.25 →
3.15, 3.71, 3.64.
- Cycling: no measurable drift (±0.1°). Loaded holds at the stops: +0.45° RMS.
- orca_core's standard recalibration (22 s) restores it fully; the new stops keep their gain after it.
- Customer advice: recalibrate after heavy loaded use; the stops need no refit after a recalibration.
- Short (72 poses, ~50 min) vs long (263 poses) on V5: stops 2.76 vs 2.88, LUT 2.59 vs 3.38 (the long
  LUT is from the other calibration run). The short calibration is enough.

### Kapandji

Thumb tip to each fingertip, 5°/s final approach, 4 maps. Vision distance (mm) / tactile contact:

| finger | old | new stops | LUT | SAC |
|---|---|---|---|---|
| index | 0.09 / yes | 0.76 / yes | 0.26 / yes | 0.67 / yes |
| middle | 0.42 / yes | 0.31 / yes | 0.23 / yes | 0.11 / yes |
| ring | 0.49 / yes | 0.12 / yes | 0.18 / yes | 0.20 / yes |
| pinky | 0.88 / no | 0.26 / no | 4.02 / no | 0.24 / no |

Index, middle and ring touch under every map. The pinky never registers on the tactile sensors,
even at 0.24 mm vision distance: the contact lands outside the taxels (or below the 0.12 threshold).
The old map presses hardest (middle: tactile force 2.08 vs 1.09 with the new stops).

### Super Advanced Calibration

16 contacts (pinky 1 of 5). Mean closure gap 0.26 mm with the new stops; the SAC refit gives 0.31 mm,
leave-one-out 0.32 mm; on the V blocks it changes RMS by ≤ 0.06°. **Not worth it at this precision**:
the new stops already close the thumb–finger loops to a quarter millimetre.

### Couplings, hysteresis, visibility

- Couplings (partial correlation of a joint's error with another joint): the wrist drives the abd
  joints (thumb_abd, ring_abd, index_abd, pinky_abd; t 10–13) and the mcp's (−); index mcp↔pip and
  middle abd↔mcp within a finger. The LUT captures these; the linear map cannot.
- Hysteresis (3 references, from below / above, 3 visits): median 0.24°, worst thumb_dip 6.0° (pinch)
  and middle_pip 4.4° (mid); repeat SD median 0.87°.
- Hardest for vision (median half-view split): thumb_mcp 3.3°, index_abd 3.2°, thumb_dip 2.8°.

### Rig / vision / URDF recommendations

- Camera (root cause of the 10:34 crash): the overnight decoder ran 24 % CPU for 9.7 h, i.e. a client
  decoded all night, most likely the browser page left streaming; browser clients asking for other
  resolutions also switched the shared camera under the runner (the 10:51 "no 4K frame" failures).
  Fixed: one shared capture per process (`visual_calibration/capture.py`), browser streams stop in
  hidden tabs and request the rig's 4K mode, no access probe once granted, reopen when another
  client switches the mode. Soak 45 min, 837 stills alongside a 4K browser stream: our process and
  the decoder flat (203 / 105 MB), decoder back to 4 MB on close.
- Wrist: never command wrist −65° with the fingers at their extension stops (jams, 5/5); a reboot of
  a motor that sits past 2π in extended position mode loses a turn — the study now detects the jump,
  never reboots past a boundary and recalibrates (worth an orca_core issue; orca_core untouched here).
- Vision: add a view for the thumb mcp / index abd; random poses with deep flexion need a
  finger-level consistency check (an unseen mcp makes its pip suspect).
- Tactile: pinky–thumb contacts are not sensed; check taxel coverage at the pinky tip.
