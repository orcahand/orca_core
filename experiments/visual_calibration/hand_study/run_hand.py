"""Measure one hand on the camera rig, end to end: a new run folder under the rig
(``<ORCA_HOME>/rigs/<rig id>/runs/<hand id>/<YYYYMMDD-HHMMSS>/``, hand_study.paths), the hand's
config in force written there (orca_core's standalone copy of its model with the hand's measured
ranges: the plans span it; the packaged config is never edited), the quick run (night_quick)
built and run with its solve workers, and the fitted hard stops recorded as the hand's
joint_ranges.yaml in its folder (in force at its next connect).

    python -m hand_study.run_hand [--hand-id ID] [--tag TAG] [--workers 2] [--limit N] [--dry]

The hand is the one connected to orca-ui (/api/hand/info), or --hand-id, a hand with a folder in
orca_core's hand store on this computer (preflight then checks it is the one connected). The
plans are built in a child process: they read the run's config when imported (STUDY_ROM_CONFIG).
A run that stopped resumes with the command it prints. orca-ui starts this as a child process of its
own and shows a run from its files (progress, results).
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
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
    from hand_study.mapping import orca_core_config
    folder = hand_store.hand_dir(hand_id)
    model = (read_yaml(folder / hand_store.HAND_FILE) or {}).get("model")
    if not model:
        raise SystemExit(f"no hand {hand_id} with a model on this computer ({folder}): connect it once first")
    measured = (read_yaml(folder / hand_store.JOINT_RANGES_FILE) or {}).get("joint_roms")
    return hand_id, model, {j: list(r) for j, r in orca_core_config(model, measured).joint_roms_dict.items()}


def run_config(run: Path, model: str, roms: dict[str, list[float]]) -> Path:
    """The model's complete config in the run, with the hand's ranges in force."""
    path = Path(write_standalone_config(str(run / "bench" / model / "config.yaml"), model_name=model))
    doc = read_yaml(path)
    doc["joint_roms"] = {**doc["joint_roms"], **roms}
    write_yaml_atomic(path, doc)
    return path


def main(hand_id: str | None, tag: str | None, workers: int, segmenter: str, motors: str, hours: float,
         limit: int | None, dry: bool) -> Path:
    hand, model, roms = the_hand(hand_id)
    run = paths.new_run(RigConfig.load(), hand, protocol="night_quick", model=model)
    tag = tag or f"{hand}-{run.name}"
    cfg = run_config(run, model, roms)
    env = {**os.environ, paths.RUN_ENV: str(run), "STUDY_ROM_CONFIG": str(cfg),
           "PYTHONPATH": os.pathsep.join(filter(None, [str(HERE), os.environ.get("PYTHONPATH")]))}
    py = [sys.executable, "-m"]
    subprocess.run([*py, "hand_study.night_quick", tag, "--run", "q" + run.name, "--bench", str(cfg.parent),
                    "--hand", hand, "--motors", motors], cwd=HERE, env=env, check=True)
    spec_path = run / "night" / f"{tag}.json"
    spec = json.loads(spec_path.read_text())
    if limit:                                   # a dry run: a few poses per block
        for s in spec["steps"]:
            if s["kind"] == "collect" and s["name"] != "N00-baseline":
                s["limit"] = limit
        spec_path.write_text(json.dumps(spec, indent=1))
    print(f"run {run}\ntag {tag}", flush=True)
    night = [*py, "hand_study.night", tag, "--hours", str(hours)]
    if dry:
        subprocess.run([*night, "--dry"], cwd=HERE, env=env, check=True)
        return run
    procs = [subprocess.Popen([*py, "hand_study.worker", *spec["sessions"], "--follow"], cwd=HERE,
                              env={**env, "STUDY_SEGMENTER": segmenter}, stderr=subprocess.STDOUT,
                              stdout=(run / "night" / f"{tag}-worker{i}.log").open("a"))
             for i in range(workers)]
    try:
        subprocess.run(night, cwd=HERE, env=env)
    finally:
        for p in procs:
            p.send_signal(signal.SIGTERM)
    state = read_json(run / "night" / f"{tag}.state.json")
    out = read_json(run / "proposed" / "hardstops.json")
    print(f"night {state.get('state')}" + (f": {state.get('error')}" if state.get("error") else ""))
    print(f"joint ranges: {out.get('joint_ranges') or 'none recorded'}")
    if state.get("state") != "done":
        print(f"resume: {paths.RUN_ENV}={run} STUDY_ROM_CONFIG={cfg} python -m hand_study.night {tag} --workers")
    return run


def read_json(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


# What a console shows of a run (orca-ui's "Measure joint ranges"): read from its files, never from the process.

STEP_LABELS = {"preflight": "preflight", "snapshot": "calibration snapshot", "N00-baseline": "baseline pose",
               "mount": "mount fit", "N07-short": "short block", "N08-V": "validation check",
               "ranges": "joint ranges fit", "accuracy": "accuracy check"}


def runs_dir(hand_id: str) -> Path:
    """Where the rig keeps `hand_id`'s runs (paths.new_run)."""
    return Path(RigConfig.load().own_file("runs")) / hand_store.hand_dir(hand_id).name


def latest_run(hand_id: str, with_results: bool = False) -> Path | None:
    """The hand's newest run folder on this rig (with_results: the newest with a fit), or None."""
    root = runs_dir(hand_id)
    runs = sorted((d for d in root.iterdir() if (d / paths.RUN_FILE).exists()), reverse=True) if root.is_dir() else []
    return next((d for d in runs if not with_results or (d / "proposed" / "hardstops.json").exists()), None)


def night_files(run: Path) -> tuple[dict, dict]:
    """The run's night spec and state ({} before night_quick wrote them)."""
    specs = [p for p in sorted((run / "night").glob("*.json")) if not p.name.endswith(".state.json")]
    if not specs:
        return {}, {}
    return read_json(specs[0]), read_json(specs[0].with_name(specs[0].stem + ".state.json"))


def progress(run: Path) -> dict:
    """Each step (done, running or to do; a block's poses), the time left (from the spec's minutes, a
    block's scaled by the poses still to take), and the newest photo of each camera (paths in the run)."""
    spec, state = night_files(run)
    current = state.get("step") if state.get("state") == "running" else None
    steps, left_s = [], 0.0
    for st in spec.get("steps", []):
        done = st["name"] in state.get("done", {})
        item = {"name": st["name"], "label": STEP_LABELS.get(st["name"], st["name"]),
                "state": "done" if done else "running" if st["name"] == current else "todo"}
        seconds = 60.0 * float(st.get("minutes") or 0)
        if st.get("session"):
            total = min(int(st["poses"]), int(st.get("limit") or st["poses"]))
            taken = len(list(Path(st["session"]).glob("configs/*/meta.json")))
            item.update(poses=total, taken=min(taken, total))
            seconds *= max(total - taken, 0) / max(total, 1)
        if item["state"] == "running" and not st.get("session"):
            seconds = max(seconds - (time.time() - float(state.get("step_t0") or time.time())), 0.0)
        if not done:
            left_s += seconds
        steps.append(item)
    return {"run": str(run), "tag": spec.get("tag"), "state": state.get("state") or ("planned" if spec else "starting"),
            "error": state.get("error"), "step": current, "steps": steps,
            "eta_s": round(left_s) if spec and state.get("state") not in ("done", "aborted") else None,
            "photos": last_photos(run, spec)}


def last_photos(run: Path, spec: dict) -> dict[str, str]:
    """The newest photo of each camera (v<stop>.jpg is camera a, v<stop>_<c>.jpg camera c) in the
    latest block that has any, relative to the run."""
    for st in reversed([st for st in spec.get("steps", []) if st.get("session")]):
        found: dict[str, Path] = {}
        for f in Path(st["session"]).glob("configs/*/v*.jpg"):
            cam = f.stem.split("_", 1)[1] if "_" in f.stem else "a"
            if cam not in found or f.stat().st_mtime > found[cam].stat().st_mtime:
                found[cam] = f
        if found:
            return {c: str(f.relative_to(run)) for c, f in sorted(found.items())}
    return {}


def results(run: Path) -> dict | None:
    """Per joint the range in force before the run and the fitted one, whether it was measured (the
    fit's coverage check, hardstops.MIN_SPAN), and the accuracy on the validation block before and
    after (quick.check); None before the ranges step ran."""
    fit = read_json(run / "proposed" / "hardstops.json")
    if not fit:
        return None
    joints = {j: {"old": s["old"], "new": s["new"], "measured": bool(s["identifiable"]), "n": s["n"],
                  "rms": s["rms"], "t_span": s["t_span"]} for j, s in fit["stops"].items()}
    info = paths.run_info(run)
    return {"run": str(run), "hand": info.get("hand"), "rig": info.get("rig"), "started": info.get("started"),
            "poses": fit["n"], "min_span": fit.get("min_span"), "joint_ranges": fit.get("joint_ranges"),
            "joints": joints, "accuracy": read_json(run / "proposed" / "accuracy.json") or None}


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
    a = p.parse_args()
    main(a.hand_id, a.tag, a.workers, a.segmenter, a.motors, a.hours, a.limit, a.dry)
