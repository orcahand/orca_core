"""The 8 h night protocol for a new hand (ser-7841, Clemens' order), run by night.py in resilient
mode (a recoverable fault never ends the night), with the solve workers and the offline analysis
(offline.py) beside it. Phases:

    P0 setup          preflight, orca_core calibration with the packaged config (wrist too),
                      baseline pose, mount fit
    P1 standard       V1 held out; scored offline: packaged / ser-4485 / ser-7577 stops
    P2 short          S1 short block (3 points per joint), own stops refitted and applied
                      (config.yaml, orca-ui restarted), orca_core calibration, V2
    P3 exercise       orca_core setup's motion test (open <-> power grasp) for 15 min
    P4 drift          V3, same stops, no recalibration
    P5 recalibration  orca_core calibration (fingers, as a customer runs it), V4, S2 short block
    P6 deep           the advanced collection trimmed to ~1 h (sweeps from both sides, wrist x
                      fingers, mcp/pip pairs, loaded holds; current logged), V5
    P7 extra          random held-out poses until the 8 h are up

Every V set is the same: the quick run's validation poses plus six of them visited again at the
end (repeatability). Plans span the hand's packaged config ROM (STUDY_ROM_CONFIG).

    STUDY_ROM_CONFIG=~/orca_bench/ser-7841/orcahand-right/config.yaml \\
        python -m hand_study.protocol build TAG [--hand ser-7841]
    python -m hand_study.protocol run TAG          # runner + workers + offline, restarted if killed
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

import numpy as np

from hand_study.configs import JOINTS, ROM
from hand_study.night_advanced import FLEX_LEVELS, PRESS_DEG, approach, clip, flexion
from hand_study.night_plan import S_PER_POSE, sweep_up, validation
from hand_study.plan import ABDS, SPARSE_ANGLES, STUDY, Builder

NIGHT = STUDY / "night"
HOURS = 8.0
REPEATS = ("v-open-a", "v-random-000", "v-random-001", "v-ref-pinch", "v-ref-mid", "v-mix")
PHASES = {"P0": "Derisk", "P1": "Setup", "P2": "Standard calibration", "P3": "Short optimisation", "P4": "Exercise",
          "P5": "Drift", "P6": "Recalibration", "P7": "Deep calibration (adaptive)", "P8": "Collision (experimental)",
          "P9": "Recalibration after collisions", "P10": "Free experiment", "P11": "Extra held-out poses"}
EIGHT = [0.0, 45.0, 112.5, 135.0, 180.0, 225.0, 270.0, 292.5]     # V5: the 5 sparse stops + 3 (views ablation)
BASELINE_LABEL = "baseline (ser-7841, production)"
WORKERS = {"birefnet": 3, "grabcut": 1, "hqsam": 1}        # est.json / est_grabcut.json / est_hqsam.json
COMPARE = ("V1", "S1", "V2", "V3", "V4", "V5", "V6")               # the comparison segmenters fit these sets only
PRIMARY = "birefnet"


def vset() -> Builder:
    b = validation(wrist_lo=False)
    first = {c["id"]: c for c in b.out}
    for cid in REPEATS:
        if cid in first:
            b.out.append({**json.loads(json.dumps(first[cid])), "id": f"{cid}-r", "group": "repeat", "repeat_of": cid})
    return b


def short3() -> Builder:
    b = Builder()
    b.add("open-0", {}, kind="repeat", ref="open", side="start", visit=0)
    for j in JOINTS:
        sweep_up(b, j, 3, "short")
    from hand_study.night_plan import approach as below
    below(b)
    return b


def deep() -> list[tuple[str, Builder]]:
    """~1 h: the advanced collection trimmed (night_advanced has the full one)."""
    sw = Builder()
    for j in JOINTS:
        lo, hi = ROM[j]
        for direction, fr in (("down", 2 / 3), ("down", 1 / 3), ("up", 1 / 2)):
            v = round(lo + fr * (hi - lo), 1)
            q = sw.free_with_neighbour(j, v, {})
            cid = f"sw-{j}-{direction}-{int(fr * 100)}"
            if q is None:
                sw.skipped.append({"id": cid, "q": {j: v}, "pair": "no free neighbour", "gap": None})
                continue
            if sw.add(cid, q, kind="sweep", joint=j, dir=direction, value=v) and direction == "down":
                sw.out[-1]["dirs"] = {j: -1}
    wr = Builder()
    lo, hi = ROM["wrist"]
    for name, f in FLEX_LEVELS.items():
        for k, v in enumerate(np.linspace(lo, hi, 3).round(1)):
            q = next((q for s in (1.0, 0.85, 0.7) if wr.c.free(q := {**flexion(f * s), "wrist": float(v)})), None)
            if q is not None:
                wr.add(f"wr-{name}-up-{k}", q, kind="wrist", dir="up", flex=name)
    for k, v in enumerate((lo + np.array([0.75, 0.25]) * (hi - lo)).round(1)):
        if wr.add(f"wr-mid-down-{k}", {**flexion(FLEX_LEVELS["mid"]), "wrist": float(v)}, kind="wrist", dir="down"):
            wr.out[-1]["dirs"] = {"wrist": -1}
    pr = Builder()
    for f in ("index", "middle", "ring", "pinky"):
        pr.grid(f, f"{f}_mcp", f"{f}_pip", 2, 2)
    pr.grid("thumb_flex", "thumb_mcp", "thumb_dip", 2, 2)
    ld = Builder()
    from hand_study.night_advanced import loaded
    loaded(ld)
    out = []
    for name, b in (("D1-sweeps", sw), ("D2-wrist", wr), ("D3-pairs", pr), ("D4-loaded", ld)):
        approach(b)
        out.append((name, b))
    return out


def extra(n: int = 300) -> Builder:
    b = Builder()
    b.random(n, JOINTS, seed=101, tag="x")
    for c in b.out:
        c["kind"] = "val"
    from hand_study.night_plan import approach as below
    below(b)
    return b


def build(tag: str, hand: str, bench: Path, runs: dict[str, str], limit: int | None = None, hours: float = HOURS) -> dict:
    assert os.environ.get("STUDY_ROM_CONFIG"), "set STUDY_ROM_CONFIG to the hand's (packaged) config.yaml"
    pdir = STUDY / "protocol" / tag
    pdir.mkdir(parents=True, exist_ok=True)
    if not (bench / "config.orig.yaml").exists():           # the packaged config, kept: config.yaml changes in P2
        import shutil
        shutil.copy2(bench / "config.yaml", bench / "config.orig.yaml")
    from hand_study.colour import skin_tone
    spec = {"tag": tag, "mode": "protocol", "hand": hand, "bench": str(bench), "skin": skin_tone(bench), "motors": "dynamixel", "track_ma": 400.0,
            "resilient": True, "retries": 8, "hours": hours, "rom_config": os.environ["STUDY_ROM_CONFIG"],
            "protocol_dir": str(pdir), "packaged_config": str(bench / "config.orig.yaml"), "runs": {"A": runs["p"], "B": runs["b"], **runs}, "angles": SPARSE_ANGLES,
            "phases": PHASES, "steps": [], "sessions": [], "sets": {}, "workers": WORKERS, "primary": PRIMARY,
            "foreign": {"ser-4485": str(STUDY / "report" / "results.json"),
                        "ser-7577": str(Path.home() / "orca_bench/ser-7577/orcahand-right/config.yaml")}}

    def session(name, run, b=None, mount="pending", angles=None, **extra):
        sdir = STUDY / "sessions" / f"{tag}-{name}"
        sdir.mkdir(parents=True, exist_ok=True)
        (sdir / "plan.json").write_text(json.dumps(
            {"calibration": run, "part": name.split("-", 1)[-1], "pilot": False, "angles": angles or SPARSE_ANGLES,
             "fit_stops": SPARSE_ANGLES, "mount": mount, "joints": JOINTS, "configs": b.out if b else [],
             "skipped": b.skipped if b else [], "night": tag, "block": name, "closed_loop": False,
             "current_each_view": True, "mode": "protocol", **extra}, indent=1))
        spec["sessions"].append(str(sdir))
        return sdir

    def collect(phase, name, b, run, mount="pending", label="", role=None, angles=None, adaptive=False):
        sdir = session(name, run, b, mount, angles)
        spec["sessions"].pop()
        per = S_PER_POSE * (1 + 0.3 * (len(angles or SPARSE_ANGLES) - 5) / 5)
        spec["steps"].append({"phase": phase, "name": name, "kind": "collect", "run": run, "session": str(sdir),
                              "poses": len(b.out), "minutes": round(len(b.out) * per / 60), "label": label,
                              **({"adaptive": True} if adaptive else {})})
        spec["sessions"].append(str(sdir))
        if role:
            spec["sets"].setdefault(role, []).append(str(sdir))

    def step(phase, name, kind, minutes, label, **kw):
        spec["steps"].append({"phase": phase, "name": name, "kind": kind, "minutes": minutes, "label": label, **kw})

    from hand_study import collision
    prs = collision.pairs()
    spec["label"] = BASELINE_LABEL
    spec["collision_pairs"] = str(collision.PAIRS)
    for k in range(1, 5):                                    # mount checks after a reconnect (filled by the runner)
        session(f"MC{k:02d}-mount", runs["p"], mount=None)
    nsl = [0]

    def slack(phase, run, why):
        nsl[0] += 1
        name = f"SL{nsl[0]}-slack"
        sdir = session(name, run)
        spec["sets"].setdefault("slack", []).append(str(sdir))
        step(phase, name, "slack", 5, f"slack check {nsl[0]}: deadzone per joint (camera + current), {why}",
             run=run, session=str(sdir), poses=8)
    base = Builder()
    base.add("zero", {}, kind="repeat", ref="open", side="start", visit=0)
    dz = Builder()
    dz.add("zero", {}, kind="repeat", ref="open", side="start", visit=0)
    step("P0", "preflight", "preflight", 1, f"{hand} connected, 17 motors, plate, camera, disk")
    step("P0", "derisk", "derisk", 15, "plate travel, camera, extremes + wrist stops, one light contact, one solve, "
         "a fault on purpose; go/no-go, then wait for Clemens (10 min if no problems)",
         session=str(session("D0-derisk", runs["p"], dz, mount=None)), pair="index_abd-middle_pp-down")
    step("P1", "calibrate-packaged", "calibrate", 4, "orca_core calibration, packaged config (wrist too)",
         run=runs["p"], params={"force_wrist": True})
    collect("P1", "N00-baseline", base, runs["p"], mount=None, label="baseline pose (mount solved)")
    step("P1", "mount", "mount", 4, "mount fixed from the baseline", run=runs["p"],
         **{"from": str(STUDY / "sessions" / f"{tag}-N00-baseline")})
    slack("P1", runs["p"], "start of the night")
    collect("P2", "V1", vset(), runs["p"], label="held out, packaged config", role="V1")
    collect("P3", "S1-short", short3(), runs["p"], label="short block, 3 points per joint", role="S1")
    step("P3", "refit-stops", "refit", 15, "own stops from S1 -> config.yaml, orca-ui restarted",
         run_from=runs["p"], backup="packaged", **{"from": [str(STUDY / "sessions" / f"{tag}-S1-short")]})
    step("P3", "calibrate-own", "calibrate", 4, "orca_core calibration with the own stops (wrist too)",
         run=runs["a"], params={"force_wrist": True})
    slack("P3", runs["a"], "after the own stops")
    collect("P3", "V2", vset(), runs["a"], label="held out, own stops", role="V2")
    step("P4", "exercise", "exercise", 15, "setup motion test: open <-> power grasp, 15 min")
    slack("P4", runs["a"], "right after the exercise")
    collect("P5", "V3", vset(), runs["a"], label="held out after the exercise (drift)", role="V3")
    step("P6", "recalibrate", "calibrate", 3, "orca_core recalibration (fingers, as a customer runs it)", run=runs["b"])
    collect("P6", "V4", vset(), runs["b"], label="held out after the recalibration (before the collisions)", role="V4")
    slack("P6", runs["b"], "after the recalibration")
    collect("P6", "S2-short", short3(), runs["b"], label="short block again (refit after recalibration; adaptive)",
            role="S2", adaptive=True)
    for name, b in deep():
        collect("P7", name, b, runs["b"], label={"D1-sweeps": "sweeps from both sides", "D2-wrist": "wrist x fingers",
                                                  "D3-pairs": "mcp x pip pairs", "D4-loaded": "loaded holds (gentle)"}[name],
                role="deep", adaptive=True)
    slack("P7", runs["b"], "after the deep blocks")
    d5 = session("D5-adaptive", runs["b"])
    step("P7", "D5-adaptive", "adaptive", 20, "densify: large residuals, hysteresis, gaps; re-aim cut-off / collided poses",
         run=runs["b"], session=str(d5), poses=30)
    spec["sets"].setdefault("deep", []).append(str(d5))
    collect("P7", "V5", vset(), runs["b"], label="held out before the collisions (8 plate stops)", role="V5", angles=EIGHT)
    cs = session("C1-collision", runs["b"], mount="pending")
    groups = {"C1-abd": ("abduction neighbours", [p for p in prs if p["joint"].endswith("_abd")]),
              "C2-thumb": ("thumb on finger", [p for p in prs if p["joint"].startswith("thumb")]),
              "C3-flexion": ("finger on thumb / palm", [p for p in prs if not p["joint"].endswith("_abd")
                                                         and not p["joint"].startswith("thumb")])}
    for name, (what, ps) in groups.items():
        if ps:
            step("P8", name, "collision", round(len(ps) * 80 / 60), f"{what}: {len(ps)} URDF contacts x {collision.REPS} "
                 f"(moving motor at {collision.CAP_MA:.0f} mA, obstacle held), photos at the first contact",
                 run=runs["b"], session=str(cs), poses=len(ps) * collision.REPS, pairs=[p["id"] for p in ps])
    spec["sets"]["collision"] = [str(cs)]
    slack("P8", runs["b"], "right after the collisions")
    step("P9", "recalibrate-post", "calibrate", 3, "orca_core recalibration after the high-current contacts (slack?)",
         run=runs["c"])
    collect("P9", "V6", vset(), runs["c"], label="held out after the collisions + recalibration (vs V5)", role="V6")
    f1 = session("F1-free", runs["c"])
    step("P10", "F1-free", "free", 30, "own experiment from the data so far (plan + reason in the Study feed first)",
         run=runs["c"], session=str(f1), poses=40)
    spec["sets"]["F"] = [str(f1)]
    slack("P11", runs["c"], "end of the night")
    step("P11", "revisit", "revisit", 5, "poses left for later (a dead motor), if any")
    x = extra(150)
    collect("P11", "X-extra", x, runs["c"], label="random held-out poses until the deadline", role="X")
    spec["steps"][-1]["minutes"] = None
    step("P11", "archive", "archive", 5, f"everything kept as the reference: {BASELINE_LABEL}",
         dest=str(bench.parent / f"study-{tag}"))
    if limit:                                   # a dry run: a few poses per block, a short exercise
        for s in spec["steps"]:
            if s["kind"] == "collect" and s["name"] != "N00-baseline":
                s["limit"] = limit
            if s["kind"] == "exercise":
                s["minutes"] = 0.3
            if s["kind"] in ("collision", "adaptive", "free", "slack"):
                s["limit"] = max(1, limit // 2)
    (NIGHT / f"{tag}.json").write_text(json.dumps(spec, indent=1))
    return spec


QUICK = ("v-open-a", "v-random-000", "v-random-001", "v-random-002", "v-random-003", "v-random-004", "v-ref-pinch")


def quick(tag: str, hand: str, bench: Path, pre: str) -> dict:
    """~20 min: orca_core calibration (packaged config), baseline + mount, 10 random training poses (all
    joints at once) and 7 of the V set held out; own stops refitted offline, nothing written to the hand."""
    spec = build(tag, hand, bench, {k: f"{pre}{k}" for k in "pabc"})
    keep = {"preflight", "calibrate-packaged", "N00-baseline", "mount", "V1"}
    spec["steps"] = [s for s in spec["steps"] if s["name"] in keep]
    spec["sessions"] = [x for x in spec["sessions"] if any(x == s.get("session") for s in spec["steps"])]
    v1 = Path(next(s["session"] for s in spec["steps"] if s["name"] == "V1"))
    plan = json.loads((v1 / "plan.json").read_text())
    plan["configs"] = [c for c in plan["configs"] if c["id"] in QUICK]
    (v1 / "plan.json").write_text(json.dumps(plan, indent=1))
    b = Builder()
    b.add("open-0", {}, kind="repeat", ref="open", side="start", visit=0)
    b.random(10, JOINTS, seed=7, tag="q-random")
    sdir = STUDY / "sessions" / f"{tag}-Q1-train"
    sdir.mkdir(parents=True, exist_ok=True)
    (sdir / "plan.json").write_text(json.dumps({**plan, "part": "train", "block": "Q1-train", "configs": b.out,
                                                "skipped": b.skipped}, indent=1))
    i = next(k for k, s in enumerate(spec["steps"]) if s["name"] == "V1")
    spec["steps"].insert(i, {"phase": "P3", "name": "Q1-train", "kind": "collect", "run": f"{pre}p", "session": str(sdir),
                             "poses": len(b.out), "minutes": round(len(b.out) * S_PER_POSE / 60),
                             "label": "training: 10 random poses, every joint"})
    for s in spec["steps"]:
        if s["name"] == "V1":
            s["poses"], s["minutes"], s["label"] = len(plan["configs"]), round(len(plan["configs"]) * S_PER_POSE / 60), \
                "held out (7 of the V set), packaged config"
    spec["sessions"].append(str(sdir))
    spec["sets"] = {"S1": [str(sdir)], "V1": [str(v1)]}
    spec["workers"] = {"birefnet": 2}
    spec["hours"] = 0.6
    spec["rms_max_px"] = 12.0
    spec["label"] = f"quick 20 min ({hand})"
    spec["protocol"] = "quick"
    (NIGHT / f"{tag}.json").write_text(json.dumps(spec, indent=1))
    return spec


def timeline(spec: dict) -> list[tuple[str, str, int, int | None]]:
    t, out = 0, []
    for s in spec["steps"]:
        out.append((s["phase"], s["name"], t, s.get("poses")))
        t += s["minutes"] or 0
    return out


# running it

def workers(spec: dict, log) -> list[subprocess.Popen]:
    here = Path(__file__).parents[1]
    procs = []
    for seg, n in spec["workers"].items():
        out = "est.json" if seg == spec["primary"] else f"est_{seg}.json"
        env = {**os.environ, "STUDY_SEGMENTER": seg, "PYTHONPATH": str(here), "VECLIB_MAXIMUM_THREADS": "2",
               "OMP_NUM_THREADS": "2"}
        sessions = spec["sessions"] if seg == spec["primary"] else \
            [s for k in COMPARE for s in spec["sets"].get(k, [])]
        for i in range(n):
            f = (NIGHT / f"{spec['tag']}-worker-{seg}{i}.log").open("a")
            nice = 0 if seg == spec["primary"] else 10
            procs.append(subprocess.Popen(["nice", "-n", str(nice), sys.executable, "-m", "hand_study.worker",
                                           *sessions, "--follow", "--out", out],
                                          stdout=f, stderr=subprocess.STDOUT, cwd=here, env=env))
    log(f"workers: {', '.join(f'{n} {s}' for s, n in spec['workers'].items())}")
    return procs


def run(tag: str) -> None:
    """The night runner, restarted when it dies without a safety stop (a kill, a crash); the
    workers and the offline analysis beside it. Ends when the night is done or aborted."""
    spec = json.loads((NIGHT / f"{tag}.json").read_text())
    here = Path(__file__).parents[1]
    logp = NIGHT / f"{tag}.supervisor.log"

    def log(m):
        line = f"{time.strftime('%H:%M:%S')} {m}"
        print(line, flush=True)
        with logp.open("a") as f:
            f.write(line + "\n")
    env = {**os.environ, "STUDY_ROM_CONFIG": spec["rom_config"], "STUDY_HOLD_STUCK": "1", "PYTHONPATH": str(here)}
    procs = workers(spec, log)
    import threading

    def ablation():
        """The fit/segmentation ablations once V5 (their held-out set) and the training blocks are solved."""
        while True:
            st = json.loads((NIGHT / f"{tag}.state.json").read_text()) if (NIGHT / f"{tag}.state.json").exists() else {}
            if "V5" in st.get("done", {}):
                ses = [Path(p) for k in ("V5", "S1", "S2", "deep") for p in spec["sets"].get(k, [])]
                n_meta = sum(len(list(s.glob("configs/*/meta.json"))) for s in ses)
                n_est = sum(len(list(s.glob("configs/*/est.json"))) for s in ses)
                if n_meta and n_est >= 0.97 * n_meta:
                    break
            if st.get("state") in ("done", "aborted") and "V5" not in st.get("done", {}):
                return
            time.sleep(60)
        log("fit ablations started")
        subprocess.run([sys.executable, "-m", "hand_study.ablate", "run", tag, "--procs", "4", "--gpu", "1"], cwd=here, env=env,
                       stdout=(NIGHT / f"{tag}-ablate.log").open("a"), stderr=subprocess.STDOUT)
        log("fit ablations done")
    abl = threading.Thread(target=ablation, daemon=True)
    abl.start()
    off = subprocess.Popen([sys.executable, "-m", "hand_study.offline", tag, "--follow"], cwd=here, env=env,
                           stdout=(NIGHT / f"{tag}-offline.log").open("a"), stderr=subprocess.STDOUT)
    try:
        for k in range(50):
            r = subprocess.run([sys.executable, "-m", "hand_study.night", tag, "--hours", str(spec["hours"])], cwd=here, env=env)
            st = json.loads((NIGHT / f"{tag}.state.json").read_text()) if (NIGHT / f"{tag}.state.json").exists() else {}
            if st.get("state") == "done":
                log("night done")
                break
            if st.get("state") == "aborted" and r.returncode == 1 and "Safety" in str(st.get("error")):
                log(f"safety stop, not restarted: {st.get('error')}")
                break
            log(f"runner ended ({r.returncode}, {st.get('state')}: {st.get('error')}); restarting in 2 min")
            time.sleep(120)
        st = json.loads((NIGHT / f"{tag}.state.json").read_text())
        if st.get("state") == "done":           # the solves and the analysis finish after the hardware
            t0 = time.time()
            todo = [(s, e) for s in spec["sessions"] for e in ["est.json"]] + \
                [(s, f"est_{g}.json") for g in spec["workers"] if g != spec["primary"]
                 for k in COMPARE for s in spec["sets"].get(k, [])]
            while time.time() - t0 < 6 * 3600 and (abl.is_alive() or any(
                    not (d / e).exists() and not (d / f"{e}.error").exists()
                    for s, e in todo for d in Path(s).glob("configs/*") if (d / "meta.json").exists())):
                time.sleep(60)
    finally:
        for p in procs:
            p.send_signal(signal.SIGTERM)
        time.sleep(600 if off.poll() is None else 0)
        off.send_signal(signal.SIGTERM)
        log("supervisor done")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("cmd", choices=("build", "run", "timeline"))
    p.add_argument("tag")
    p.add_argument("--hand", default="ser-7841")
    p.add_argument("--bench", type=Path, default=Path.home() / "orca_bench/ser-7841/orcahand-right")
    p.add_argument("--limit", type=int, help="dry run: poses per block")
    p.add_argument("--hours", type=float, default=HOURS)
    p.add_argument("--quick", action="store_true", help="the ~20 min protocol")
    a = p.parse_args()
    if a.cmd == "build" and a.quick:
        s = quick(a.tag, a.hand, a.bench, "q" + a.hand.removeprefix("ser-"))
    elif a.cmd == "build":
        pre = "dry" if a.limit else "s7841"
        s = build(a.tag, a.hand, a.bench, {"p": f"{pre}p", "a": f"{pre}a", "b": f"{pre}b", "c": f"{pre}c"}, a.limit, a.hours)
    if a.cmd == "run":
        run(a.tag)
    else:
        s = json.loads((NIGHT / f"{a.tag}.json").read_text())
        for ph, name, t, n in timeline(s):
            t = int(t)
            print(f"{t // 60:2d}:{t % 60:02d}  {ph}  {name:20s} {n if n is not None else '':>4}")
        tot = int(sum(x['minutes'] or 0 for x in s['steps']))
        print(f"hardware steps besides the extra block: {tot // 60} h {tot % 60:02d} min; the extra block fills to {s['hours']:g} h")
