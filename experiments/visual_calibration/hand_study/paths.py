"""Where the hand study keeps its files.

A run measures one hand on the camera rig. Everything it writes (plans, photos, solves, the
archived calibration, the fit) goes to its own folder, named to every process of the run by
``$STUDY_RUN``::

    <ORCA_HOME>/rigs/<rig id>/runs/<hand id>/<YYYYMMDD-HHMMSS>/     (visual_calibration.rig_home)
      run.yaml        which rig and hand, when, which protocol

Without a run, the study's shared folder in orca-ui's state,
``<ORCA_HOME>/ui/visual_calibration/hand_study`` (``orca_ui.home.ui_home``): the overnight
studies and the maps orca-ui offers per hand.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path

from orca_core import hand_store
from orca_core.utils.utils import read_yaml, write_yaml_atomic

RUN_ENV = "STUDY_RUN"
RUN_FILE = "run.yaml"
UI_STATE = hand_store.orca_home() / "ui"
SHARED = UI_STATE / "visual_calibration" / "hand_study"


def study_dir() -> Path:
    run = os.environ.get(RUN_ENV)
    return Path(run).expanduser().absolute() if run else SHARED


STUDY = study_dir()


def new_run(rig, hand_id: str, **info) -> Path:
    """A new, empty run folder for `hand_id` on `rig` (a RigConfig), its run.yaml naming both."""
    started = datetime.now(timezone.utc).astimezone()
    d = Path(rig.own_file("runs")) / hand_store.hand_dir(hand_id).name / started.strftime("%Y%m%d-%H%M%S")
    d.mkdir(parents=True)
    write_yaml_atomic(d / RUN_FILE, {"rig": Path(rig.home).name, "hand": hand_id,
                                     "started": started.isoformat(timespec="seconds"), **info})
    return d


def run_info(study: Path | None = None) -> dict:
    """The run.yaml of a run folder (default: this process's); {} outside a run."""
    return read_yaml((study or STUDY) / RUN_FILE) or {}
