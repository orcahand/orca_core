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

import json
import os
from datetime import datetime, timezone
from pathlib import Path

from orca_core import hand_store
from orca_core.utils.utils import read_yaml, write_yaml_atomic

RUN_ENV = "STUDY_RUN"
RUN_FILE = "run.yaml"
CONSOLE_ENV = "ORCA_UI_PID"     # set by orca-ui when it starts a run: its pid (the run ends without it)
RIG_ENV = "ORCA_RIG_ID"         # set by orca-ui when it starts a run: the rig it has connected
UI_STATE = hand_store.orca_home() / "ui"
SHARED = UI_STATE / "visual_calibration" / "hand_study"


def study_dir() -> Path:
    run = os.environ.get(RUN_ENV)
    return Path(run).expanduser().absolute() if run else SHARED


STUDY = study_dir()


def new_run(rig, hand_id: str, **info) -> Path:
    """A new, empty run folder for `hand_id` on `rig` (a RigConfig), its run.yaml naming both."""
    started = datetime.now(timezone.utc).astimezone()
    stamp = started.strftime("%Y%m%d-%H%M%S")
    hand = Path(rig.own_file("runs")) / hand_store.hand_dir(hand_id).name
    hand.mkdir(parents=True, exist_ok=True)
    for k in range(1, 1000):                    # two runs in one second each get their own
        d = hand / (stamp if k == 1 else f"{stamp}-{k}")
        try:
            d.mkdir()
            break
        except FileExistsError:
            continue
    else:
        raise FileExistsError(f"no free run folder for {stamp} in {hand}")
    write_yaml_atomic(d / RUN_FILE, {"rig": Path(rig.home).name, "hand": hand_id,
                                     "started": started.isoformat(timespec="seconds"), **info})
    return d


def run_info(study: Path | None = None) -> dict:
    """The run.yaml of a run folder (default: this process's); {} outside a run."""
    return read_yaml((study or STUDY) / RUN_FILE) or {}


def run_rig_id() -> str | None:
    """The rig this process works on: its run's (run.yaml), else the one orca-ui named (RIG_ENV),
    else None (RigConfig: the one turntable.yaml pins). The planner, the night and the solve
    workers of a run all read the same rig folder and calibration."""
    return run_info().get("rig") or os.environ.get(RIG_ENV) or None


def read_json(path: Path):
    """A JSON file's content, {} when it is missing or not whole yet (written by another process)."""
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def write_json(path: Path, data) -> None:
    """Write `path` whole or not at all (a temp file replaced): a reader never sees half of it."""
    path = Path(path)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(json.dumps(data, indent=1))
    os.replace(tmp, path)
