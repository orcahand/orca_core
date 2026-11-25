# Handover: testing the rig's hand measurement (2026-11-25)

For Clemens, or the coding agent he works with. Paste this file as the prompt, or read it as a checklist.

## What this is

Your `visual-calibration` branches, carried on as `internal/rig-calibration` in orca_core and orca_ui
(internal, not for main). The goal is that in-house assemblers measure each right hand's joint ranges
on your two-camera turntable rig from orca-ui, and get per-hand accuracy numbers.

**Status: built and tested in software only (orca_core 2388 tests, experiments 120, orca_ui 565), never
run on the real rig, cameras or a hand.** Expect bugs on hardware. That is what this test is for.

**Out of scope for this test:** flashing firmware and writing to the hand's board. Leave
`ORCA_FIRMWARE_REPO` unset; the console then hides "Write to board".

## What changed from your branch

- Adapted to orca_core 0.14 / orca_ui 0.6. The ranges in force come from orca_core (hand file > board >
  package), with no copy of the hand's config.
- **The rig's own files moved out of the repo** into `~/.orca/rigs/<rig id>/`: `calibration.yaml`,
  `turntable_calibration.yaml`, photo sessions and hand runs (`runs/<hand id>/<YYYYMMDD-HHMMSS>/`). The rig
  id is the rig board's rig ID, else its USB serial. A file still in `rigs/v1/camera-rig/` is read, with
  a warning, until the rig has its own. See `visual_calibration/rig_home.py`.
- **Two cameras required; the Python backend owns both** (`visual_calibration/rig_cameras.py`, served
  under `/api/visual-calibration/cameras`). The browser only previews. A preflight proves which camera
  is which: each one's exposure is cut through uvcc and exactly one stream must darken. With the tower on
  the plate, each camera's height must match its mount (A ~250 mm, B ~470 mm).
- The 12-tile tube (tower v2) is the configured calibration object.
- The rig board is found only by its USB serial or its rig identity, never "a free board with one motor".
- `python -m hand_study.run_hand`: one run per hand. It plans over the ranges in force, collects the
  72-pose block plus the 25-pose check with the solve workers beside it, records the fitted hard stops as
  the hand's `joint_ranges.yaml`, and computes accuracy before and after (`proposed/accuracy.json`).
- orca-ui › Setup › Visual calibration › "Measure joint ranges": connect → normal calibration if needed →
  mount → Measure (progress, Stop, E-STOP kills the run's whole process group) → Results. While a run
  is active, the rig controls it doesn't use answer 409.
- Your extras (Study tab, study maps, ghosts, sandbox, live estimate) are kept but hidden: start
  `orca-ui --research-ui` to get them back.

## Setup

1. **Checkouts:**
   - orca_core and orca_ui on `internal/rig-calibration`;
   - orcahand_description on main, next to orca_core (else set `ORCAHAND_DESCRIPTION=<path>`).
2. **orca-ui on this orca_core:** in orca_ui, run `./dev local <orca_core checkout>`. The banner's `core`
   line must show the local checkout. If orca_core isn't at `../orca_core`, set
   `ORCA_VISUAL_CALIBRATION=<orca_core>/experiments/visual_calibration`.
3. **The console's Python needs:** OpenCV with aruco (tested with 4.13), scipy, trimesh, torch and
   transformers (BiRefNet `ZhengPeng7/BiRefNet_dynamic`, ~900 MB). Load the model once before the first
   run so the download doesn't happen mid-run. You also need node/npx for `uvcc`.
4. **Cameras:**
   - Put each camera's USB serial into `rigs/v1/camera-rig/config.yaml` under
     `cameras.a.uvc.device.serial` and `cameras.b.uvc.device.serial`. Run
     `python -m visual_calibration.sweep --list-cameras` to see them. Without a serial the preflight
     refuses.
   - Close Logi Tune.
   - Not BRIOs? On 2026-11-24 the rig Mac had two "USB 2.0 Camera" units (0c45:6361, 1080p) that both
     report the serial `SN20221220`. Serials can't tell those apart. `uvc.find_device` also filters on
     `location`, but telling them apart by port is untested.
5. **The tube:** check the J/K tile letters. orca_files `set.yaml` and the rig config disagree on which
   letter the tile with marker ids 36–47 has.

## The test, in order

1. orca-ui with the hand plugged in → Setup › Visual calibration → "Check cameras" passes.
2. Rig steps 1a and 1b with both cameras, then save to rig. Check that the result lands in
   `~/.orca/rigs/<rig id>/`.
3. Dry run: `python -m hand_study.run_hand --dry` (from `experiments/visual_calibration`, with orca-ui
   running and the hand connected). It builds the run and lists its steps; nothing moves.
4. In the console: connect → calibrate → mount → Measure (~65 min) → Results. Results shows per-joint
   ranges and accuracy before/after.
5. E-STOP mid-block, and Stop mid-plate-move: no process left (`ps -g <pgid>`), hand and plate limp. Quit
   orca-ui with one Ctrl+C, never two: the run is killed and torque goes off.

**Note:** a finished run writes the hand's `~/.orca/hands/<hand id>/joint_ranges.yaml` on this computer,
and those ranges are in force from the next connect. Move the file away to go back to the previous ranges.

## Please confirm or report back

- The run's defaults: segmenter `birefnet`, 2 workers, 3 h deadline (`run_hand --help`).
- Quality gate: for now a joint gets a measured range if the fit's coverage check (`MIN_SPAN`) passes.
  Diego still decides the real gate.
- Unverified on the rig:
  - OpenCV's camera order with the MacBook's own camera present;
  - how long the preflight takes;
  - whether camera B sees enough tiles at the park angle;
  - how far the solves lag behind collection against the 3 h deadline.
- Send back the run folder's `proposed/accuracy.json`, `hardstops.json` and the run's log.

## Tests

From the orca_core checkout. Set `HOME` and `ORCA_HOME` to a scratch folder so tests never touch the
real `~/.orca`:
- `pytest -q` for the package;
- `PYTHONPATH=.:experiments/visual_calibration pytest -q -o addopts="" experiments/visual_calibration/tests`
  for the rig.

Known failures, not caused by this work:
- `test_rig_config::test_save_calibration_keeps_history`, already failing on `visual-calibration`;
- `test_tile_detector_finds_a_rendered_tile` (OpenCV, 0.5 px);
- `test_live_fit::test_joints_palm_on` (tolerance).

orca_ui: `pytest -q` with `ORCA_RIG_AUTODETECT=0`.

Working on this branch: commit on `internal/rig-calibration`; never merge into main.
