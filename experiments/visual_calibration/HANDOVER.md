# Handover: testing the rig's hand measurement (2026-11-25)

For Clemens, or the coding agent he works with. Paste this file as the prompt, or read it as a checklist.

## What this is

Your `visual-calibration` branches, carried on as `internal/rig-calibration` in orca_core and orca_ui
(internal, not for main). The goal is that in-house assemblers measure each right hand's joint ranges
on your two-camera turntable rig from orca-ui, and get per-hand accuracy numbers.

**Status: built and tested in software only (orca_core 2388 tests, experiments 120, orca_ui 565), never
run on the real rig, cameras or a hand.** Expect bugs on hardware. That is what this test is for.

**Writing to the hand's board is a separate, last part.** It needs the firmware branch
`internal/board-calibration-store` (its `HANDOVER.md`), with the board flashed to FW 4. Without
`ORCA_FIRMWARE_REPO` pointing at that checkout, the console hides "Write to board" and everything else
works.

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

1. **Checkouts, side by side in one folder:** orca_core and orca_ui on `internal/rig-calibration`,
   orca_firmware on `internal/board-calibration-store` (optional, for board writes), orcahand_description
   on main. Elsewhere: set `ORCA_UI`, `ORCA_FIRMWARE_REPO`, `ORCAHAND_DESCRIPTION`.
2. **Install:** `uv` and node (`brew install uv node`), then from orca_core run
   `experiments/visual_calibration/setup_rig.sh`. It makes orca_ui's environment on Python 3.12 with
   orca_core's `rig` extra (OpenCV with aruco, torch, transformers and what BiRefNet needs), points orca_ui
   at this orca_core (`./dev local`), downloads the BiRefNet weights (~450 MB), runs orca_firmware's
   `./setup.sh` (`--no-firmware` skips it), checks uvcc and Rosetta, and ends with the doctor. Run it
   again whenever: it only does what is missing.
3. **Doctor:** `python -m visual_calibration.doctor` (from `experiments/visual_calibration`, with
   orca_ui's `.venv/bin/python`) checks all of this and the rig's hardware, and prints the fix for each
   problem. It opens nothing.
4. **Cameras:** plug both in, close Logi Tune, then from `experiments/visual_calibration`:
   `python -m visual_calibration.sweep --assign-cameras`. It writes each camera's USB identity into
   `rigs/v1/camera-rig/config.yaml` (a tracked file). Two cameras that report the same serial (the rig
   Mac's two "USB 2.0 Camera" units, 0c45:6361, both `SN20221220`) are told apart by their USB port, so
   keep each in its port. Which is A and which B is a guess until "Check cameras" passes with the tower
   on the plate; if it says the cameras are swapped, run `... --assign-cameras --swap`.
5. **Start the console:** `experiments/visual_calibration/rig-console.sh` (orca-ui with the rig's paths
   set). The banner's `core` line must show the local checkout.
6. **The tube:** check the J/K tile letters. orca_files `set.yaml` and the rig config disagree on which
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
6. Optional, with the board on FW 4 and `ORCA_FIRMWARE_REPO` set, then orca-ui restarted:
   - "Write to board" → the read-back confirms.
   - Unplug and replug: the ranges are still in force.
   - Start orca-ui with a fresh `ORCA_HOME` (a new folder): the board's calibration is copied on
     first contact, and the hand menu says the ranges come from the board.

**Note:** a finished run writes the hand's `~/.orca/hands/<hand id>/joint_ranges.yaml` on this computer,
and those ranges are in force from the next connect. Move the file away to go back to the previous ranges.

## Please confirm or report back

- The run's defaults: segmenter `birefnet`, 2 workers, 3 h deadline (`run_hand --help`).
- Quality gate: for now a joint gets a measured range if the fit's coverage check (`MIN_SPAN`) passes.
  Diego still decides the real gate.
- Unverified on the rig:
  - OpenCV's camera order with the MacBook's own camera present;
  - two cameras with one serial, told apart by USB port (tested on fakes only);
  - with the "USB 2.0 Camera" units instead of BRIOs: config.yaml's resolution (3840×2160) and UVC
    controls (focus, exposure, white balance) are the BRIO's and may not fit them;
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
