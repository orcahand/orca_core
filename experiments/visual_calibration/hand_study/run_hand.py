"""Measure one hand on the camera rig, end to end: a new run folder under the rig
(``<ORCA_HOME>/rigs/<rig id>/runs/<hand id>/<YYYYMMDD-HHMMSS>/``, hand_study.paths), the hand's
config in force written there (orca_core's standalone copy of its model with the hand's measured
ranges: the plans span it; the packaged config is never edited), the quick run (night_quick)
built and run with its solve workers, and the fitted hard stops recorded as the hand's
joint_ranges.yaml in its folder (in force at its next connect).

    python -m hand_study.run_hand [--hand-id ID] [--tag TAG] [--workers 2] [--limit N] [--dry]
    python -m hand_study.run_hand --resume RUN_FOLDER [--workers 2]     # go on where it stopped

The hand is the one connected to orca-ui (/api/hand/info), or --hand-id, a hand with a folder in
orca_core's hand store on this computer (preflight then checks it is the one connected). The
plans are built in a child process: they read the run's config when imported (STUDY_ROM_CONFIG).
A run that stopped resumes with --resume (the command it prints).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
from pathlib import Path

from orca_core import hand_store, write_standalone_config
from orca_core.utils.utils import read_yaml, write_yaml_atomic

from hand_study import paths
from visual_calibration.rig_config import RigConfig

HERE = Path(__file__).parents[1]


def the_hand(hand_id: str | None) -> tuple[str, str, dict[str, list[float]]]:
    """(id, model, joint ranges in force) of the hand: the one orca-ui has, or `hand_id`'s folder."""
    if hand_id is None:
        from hand_study.hw import call
        info = call("/hand/info")
        hand_id = (info.get("hand") or {}).get("id")
        if not hand_id:
            raise SystemExit("orca-ui has no hand: connect it, or name it with --hand-id")
        return hand_id, info["model_name"], {j["id"]: list(j["rom"]) for j in info["joints"]}
    from hand_study.mapping import orca_core_config, stored_ranges
    folder = hand_store.hand_dir(hand_id)
    model = (read_yaml(folder / hand_store.HAND_FILE) or {}).get("model")
    if not model:
        raise SystemExit(f"no hand {hand_id} with a model on this computer ({folder}): connect it once first")
    config = orca_core_config(model, stored_ranges(folder, model))
    return hand_id, model, {j: list(r) for j, r in config.joint_roms_dict.items()}


def run_config(run: Path, model: str, roms: dict[str, list[float]]) -> Path:
    """The model's complete config in the run, with the hand's ranges in force."""
    path = Path(write_standalone_config(str(run / "bench" / model / "config.yaml"), model_name=model))
    doc = read_yaml(path)
    doc["joint_roms"] = {**doc["joint_roms"], **roms}
    write_yaml_atomic(path, doc)
    return path


def run_env(run: Path, cfg: Path) -> dict:
    """The environment of every process of the run: its folder, its config, this ORCA_HOME (absolute,
    so a child started elsewhere agrees on it) and this experiment on the path."""
    return {**os.environ, paths.RUN_ENV: str(run), "STUDY_ROM_CONFIG": str(cfg),
            hand_store.ORCA_HOME_ENV: str(hand_store.orca_home()),
            "PYTHONPATH": os.pathsep.join(filter(None, [str(HERE), os.environ.get("PYTHONPATH")]))}


def main(hand_id: str | None, tag: str | None, workers: int, segmenter: str, motors: str, hours: float,
         limit: int | None, dry: bool) -> Path:
    hand, model, roms = the_hand(hand_id)
    run = paths.new_run(RigConfig.load(), hand, protocol="night_quick", model=model)
    tag = tag or f"{hand}-{run.name}"
    write_yaml_atomic(run / paths.RUN_FILE, {**paths.run_info(run), "tag": tag})
    cfg = run_config(run, model, roms)
    env = run_env(run, cfg)
    subprocess.run([sys.executable, "-m", "hand_study.night_quick", tag, "--run", "q" + run.name, "--bench",
                    str(cfg.parent), "--hand", hand, "--motors", motors], cwd=HERE, env=env, check=True)
    spec_path = run / "night" / f"{tag}.json"
    spec = json.loads(spec_path.read_text())
    if limit:                                   # a dry run: a few poses per block
        for s in spec["steps"]:
            if s["kind"] == "collect" and s["name"] != "N00-baseline":
                s["limit"] = limit
        spec_path.write_text(json.dumps(spec, indent=1))
    print(f"run {run}\ntag {tag}", flush=True)
    if dry:
        subprocess.run([sys.executable, "-m", "hand_study.night", tag, "--dry"], cwd=HERE, env=env, check=True)
        return run
    return go(run, workers, segmenter, hours)


def go(run: Path, workers: int, segmenter: str, hours: float) -> Path:
    """Run (or resume) the run's night with its solve workers; then say what came of it."""
    info = paths.run_info(run)
    tag = info["tag"]
    env = run_env(run, run / "bench" / info["model"] / "config.yaml")
    spec = json.loads((run / "night" / f"{tag}.json").read_text())
    procs = [subprocess.Popen([sys.executable, "-m", "hand_study.worker", *spec["sessions"], "--follow"], cwd=HERE,
                              env={**env, "STUDY_SEGMENTER": segmenter}, stderr=subprocess.STDOUT,
                              stdout=(run / "night" / f"{tag}-worker{i}.log").open("a"))
             for i in range(workers)]
    try:
        subprocess.run([sys.executable, "-m", "hand_study.night", tag, "--hours", str(hours)], cwd=HERE, env=env)
    finally:
        for p in procs:
            p.send_signal(signal.SIGTERM)
    state = read_json(run / "night" / f"{tag}.state.json")
    out = read_json(run / "proposed" / "hardstops.json")
    print(f"night {state.get('state')}" + (f": {state.get('error')}" if state.get("error") else ""))
    if out.get("joint_ranges"):
        from hand_study.hardstops import ranges_summary
        print(ranges_summary(out["joint_ranges"]))
    else:
        print("joint ranges: none recorded")
    if state.get("state") != "done":
        print(f"resume: cd {HERE} && {hand_store.ORCA_HOME_ENV}={env[hand_store.ORCA_HOME_ENV]} "
              f"PYTHONPATH={env['PYTHONPATH']} {sys.executable} -m hand_study.run_hand --resume {run} "
              f"--workers {workers} --segmenter {segmenter}")
    return run


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--hand-id", help="default: the hand connected to orca-ui")
    p.add_argument("--tag", help="default: <hand id>-<run folder>")
    p.add_argument("--workers", type=int, default=2, help="solve workers beside the runner")
    p.add_argument("--segmenter", default="birefnet", help="STUDY_SEGMENTER of the workers (as the protocol's)")
    p.add_argument("--motors", default="dynamixel")
    p.add_argument("--hours", type=float, default=3.0)
    p.add_argument("--limit", type=int, help="dry run: poses per block")
    p.add_argument("--dry", action="store_true", help="build the run and list its steps; nothing moves")
    p.add_argument("--resume", type=Path, help="a run folder: go on where its night stopped")
    a = p.parse_args()
    if a.resume:
        go(a.resume.expanduser().absolute(), a.workers, a.segmenter, a.hours)
    else:
        main(a.hand_id, a.tag, a.workers, a.segmenter, a.motors, a.hours, a.limit, a.dry)
