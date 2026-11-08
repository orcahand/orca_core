"""Solve every collected configuration that has no estimate yet (several of these run side
by side; a lock file per configuration keeps them apart).

    python -m hand_study.worker SESSION_DIR [SESSION_DIR ...] [--follow]
"""

from __future__ import annotations

import argparse
import json
import os
import time
import traceback
from pathlib import Path

import numpy as np

from hand_study.collect import write_json
from hand_study.estimate import StudyFit, load_frames
from hand_study.mapping import LinearMap

STUDY = Path.home() / ".orca_ui/visual_calibration/hand_study"


def claim(d: Path) -> bool:
    try:
        os.close(os.open(d / "est.lock", os.O_CREAT | os.O_EXCL | os.O_WRONLY))
        return True
    except FileExistsError:
        return False


def pending(sessions: list[Path]):
    for s in sessions:
        for d in sorted((s / "configs").glob("*")):
            if (d / "meta.json").exists() and not (d / "est.json").exists() and not (d / "est.lock").exists():
                yield s, d


def solve_one(fit: StudyFit, session: Path, d: Path) -> None:
    plan = json.loads((session / "plan.json").read_text())
    meta = json.loads((d / "meta.json").read_text())
    L = LinearMap(STUDY / f"calib_{plan['calibration']}")
    assumed = L.joints_of(meta["before"]["motor_ids"], meta["before"]["motor_pos"])
    mount = np.array(plan["mount"]) if plan.get("mount") else None
    out = fit.solve(load_frames(d), assumed, mount)
    fit.save_views(d / "views.npz")
    out["assumed"] = assumed
    out["commanded"] = meta["q"]
    write_json(d / "est.json", out)


def main(sessions: list[Path], follow: bool) -> None:
    fit = StudyFit()
    while True:
        did = False
        for s, d in pending(sessions):
            if not claim(d):
                continue
            did = True
            try:
                solve_one(fit, s, d)
                print(d.name, "ok", flush=True)
            except Exception:
                (d / "est.error").write_text(traceback.format_exc())
                print(d.name, "error", flush=True)
            finally:
                (d / "est.lock").unlink(missing_ok=True)
        if not did:
            if not follow:
                return
            time.sleep(5)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("sessions", type=Path, nargs="+")
    p.add_argument("--follow", action="store_true")
    a = p.parse_args()
    main(a.sessions, a.follow)
