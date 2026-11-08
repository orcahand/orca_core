"""The overnight runner (STUDY.md): the steps of night/<tag>.json in order, resumable
(state in night/<tag>.state.json; a finished step is skipped, a collection resumes at its
first pose without photos). Safety: the hand and the plate never move together, torque off
after every block and on any error; an Abort (e-stop, motor error, > 65 °C, disk) ends the
night, other errors are retried (3x, 60 s apart). A heartbeat file feeds watchdog.py.

    python -m hand_study.night TAG [--hours 8] [--dry]
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

from hand_study import collect, hw, wear
from hand_study.hw import Abort, Hand, Stream, Turntable, beat, call

STUDY = Path.home() / ".orca_ui/visual_calibration/hand_study"
NIGHT = STUDY / "night"
BENCH = Path.home() / "orca_bench/ser-4485/orcahand-touch-right"
RETRIES = 3
WORKERS = 5


class Night:
    def __init__(self, tag: str, hours: float):
        self.tag = tag
        self.spec = json.loads((NIGHT / f"{tag}.json").read_text())
        self.state_path = NIGHT / f"{tag}.state.json"
        self.log_path = NIGHT / f"{tag}.log"
        self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else \
            {"tag": tag, "started": time.time(), "done": {}, "errors": []}
        self.deadline = self.state.setdefault("deadline", self.state["started"] + hours * 3600)
        hw.HEARTBEAT = NIGHT / f"{tag}.heartbeat"

    def log(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        with self.log_path.open("a") as f:
            f.write(line + "\n")

    def save(self, **kv) -> None:
        self.state.update(kv, t=time.time())
        tmp = self.state_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.state, indent=1))
        tmp.replace(self.state_path)

    def reconnect_plate(self) -> None:
        """A wedged turntable port ('Port is in use') clears with a release and a new connection."""
        try:
            call("/visual-calibration/turntable/release", {}, timeout=20)
            time.sleep(1)
            st = call("/visual-calibration/turntable/connect", {}, timeout=30)
            self.log(f"turntable reconnected at {st.get('angle')}")
        except Exception as e:
            self.log(f"turntable reconnect failed: {e}")

    def safe_off(self) -> None:
        for path in ("/torque/disable", "/visual-calibration/turntable/torque-off"):
            try:
                call(path, {}, timeout=20)
            except Exception as e:
                self.log(f"torque off {path}: {e}")

    # steps

    def calibrate(self, step: dict) -> None:
        run = step["run"]
        if (STUDY / f"calib_{run}" / "calibration.yaml").exists():
            self.log(f"{run} already archived")
            return
        if call("/status").get("torque_enabled"):
            call("/torque/disable", {})
        call("/operation/calibrate/start", {"params": {}})
        t0 = time.time()
        while True:
            op = call("/operation").get("operation") or {}
            beat(f"calibrate {op.get('phase')}")
            if op.get("state") in ("done", "error") or not op:
                break
            if time.time() - t0 > 900:
                call("/operation/stop", {})
                raise Abort("orca_core calibration did not finish in 15 min")
            time.sleep(2)
        if op.get("state") != "done":
            raise Abort(f"orca_core calibration ended {op.get('state')}: {op.get('detail')}")
        t1 = time.time()
        while time.time() - t1 < 120:
            st = call("/status")
            if st.get("state") == "connected" and not st.get("leaving_maintenance"):
                break
            beat("waiting for the hand after calibration")
            time.sleep(2)
        else:
            raise Abort("the hand did not come back after orca_core's calibration")
        log = call("/operation/log")
        out = STUDY / f"calib_{run}"
        out.mkdir(exist_ok=True)
        for f in ("calibration.yaml", "config.yaml", "setup.yaml"):
            if (BENCH / f).exists():
                shutil.copy2(BENCH / f, out / f)
        (out / "operation_log.json").write_text(json.dumps(log, indent=1))
        call("/torque/disable", {})
        self.log(f"orca_core calibration done in {time.time() - t0:.0f} s -> calib_{run}")

    def collect(self, step: dict) -> None:
        s = Path(step["session"])
        until = self.deadline if step["name"].endswith("extra") else None
        plan = json.loads((s / "plan.json").read_text())
        if until:
            left = int((until - time.time()) / 40)
            if left <= 0:
                self.log("no time left for the extra block")
                return
            collect.run(s, limit=left, log=self.log)
        else:
            collect.run(s, limit=step.get("limit"), log=self.log)
        self.log(f"{step['name']}: {len(list((s / 'configs').glob('*/meta.json')))}/{len(plan['configs'])} poses")

    def wear(self, step: dict) -> None:
        prog = self.spec["wear"][step["programme"]]
        hand = Hand(self.stream)
        logp = NIGHT / f"{self.tag}-{step['name']}.jsonl"
        fn = wear.cycle if step["programme"] == "cycle" else wear.loaded
        n = fn(hand, prog, step["minutes"], logp, self.log)
        self.log(f"{step['name']}: {n} moves in {step['minutes']} min")

    def closedloop(self, step: dict) -> None:
        """The validation set commanded through the new hard stops: fit them on the dense block,
        write proposed/config.yaml (+ diff), and command each pose so the motor goes where
        orca_core with that config (after its calibration, this run's motor limits) would send it."""
        from hand_study import hardstops
        dense = Path(self.spec.get("dense_session") or
                     next(x["session"] for x in self.spec["steps"] if x["name"].endswith("dense")))
        t0 = time.time()
        while time.time() - t0 < 1800:
            n_meta = len(list((dense / "configs").glob("*/meta.json")))
            n_est = len(list((dense / "configs").glob("*/est.json")))
            if n_est >= 0.95 * n_meta:
                break
            beat("waiting for solves")
            time.sleep(30)
        res = hardstops.main([dense], self.spec["runs"]["A"])
        self.log(f"new hard stops ({res['n']} poses): {len(res['diff'])} numbers changed")
        s = Path(step["session"])
        plan = json.loads((s / "plan.json").read_text())
        if not plan.get("emulated"):
            new = hardstops.Linear(step["run"], {j: v["new"] for j, v in res["stops"].items()})
            old = hardstops.Linear(step["run"])
            for c in plan["configs"]:
                c["q_wanted"] = dict(c["q"])
                c["q"] = {j: round(old.joint(j, new.motor(j, min(max(v, new.rom[j][0]), new.rom[j][1]))), 2)
                          for j, v in c["q"].items()}
                c["via"] = [{j: round(old.joint(j, new.motor(j, min(max(v, new.rom[j][0]), new.rom[j][1]))), 2)
                             for j, v in via.items()} for via in c.get("via") or []]
            plan.update(emulated=True, new_stops={j: v["new"] for j, v in res["stops"].items()})
            (s / "plan.json").write_text(json.dumps(plan, indent=1))
        self.collect(step)

    def kapandji(self, step: dict) -> None:
        """Thumb tip to each fingertip (touch_poses.json) under three maps: the old config, the
        new hard stops (emulated as in closedloop) and the LUT (inverse). Reached slowly through
        the same via poses, tactile zeroed with the hand open, torque off after each pose."""
        from hand_study import hardstops
        from hand_study.configs import REST
        s = Path(step["session"])
        plan = json.loads((s / "plan.json").read_text())
        if not plan.get("built"):
            hs = STUDY / "proposed" / "hardstops.json"
            if not hs.exists():
                dense = Path(self.spec.get("dense_session") or
                             next(x["session"] for x in self.spec["steps"] if x["name"].endswith("dense")))
                hardstops.main([dense], self.spec["runs"]["A"])
            stops = json.loads(hs.read_text())["stops"]
            new = hardstops.Linear(step["run"], {j: v["new"] for j, v in stops.items()})
            old = hardstops.Linear(step["run"])
            maps = {"old": lambda q: dict(q),
                    "new": lambda q: {j: round(old.joint(j, new.motor(j, min(max(v, new.rom[j][0]), new.rom[j][1]))), 2)
                                      for j, v in q.items()}}
            try:
                from hand_study import lut as lutmod
                lutmod.main()
                L = lutmod.Lut.load()
                maps["lut"] = lambda q: {j: round(v, 2) for j, v in L.inverse(q).items()}
            except (Exception, SystemExit) as e:
                self.log(f"kapandji: no LUT ({e!r}), old and new maps only")
            touch = json.loads((STUDY / "touch_poses.json").read_text())
            configs = []
            for f, t in touch.items():
                for name, fn in maps.items():
                    cmd = fn(t["q"])
                    configs.append({"id": f"kap-{f}-{name}", "q": cmd, "q_wanted": t["q"], "kind": "kapandji",
                                    "finger": f, "map": name, "speed": 5.0,
                                    "via": [dict(REST), {j: round(0.8 * v, 2) for j, v in cmd.items()}]})
            plan.update(configs=configs, built=True, torque_off_each=True, tactile=True)
            (s / "plan.json").write_text(json.dumps(plan, indent=1))
        hand = Hand(self.stream)
        hand.torque(True)
        try:
            hand.move(dict(REST))
            time.sleep(1.0)
            call("/tactile/zero", {})
        finally:
            hand.torque(False)
        self.collect(step)

    def sac(self, step: dict) -> None:
        """Super Advanced Calibration: only after a complete main study with a good hard-stop fit.
        Touch contacts (sac.py), the stops refitted with them (sac_fit.py), then the Kapandji
        poses once more under the SAC stops."""
        from hand_study import hardstops, sac, sac_fit
        from hand_study.configs import REST
        main = [x["name"] for x in self.spec["steps"] if x["kind"] not in ("sac",) and not x["name"].endswith("extra")]
        missing = [n for n in main if n not in self.state["done"]]
        hs = STUDY / "proposed" / "hardstops.json"
        good = hs.exists() and (lambda r: r["b_new_stops"]["all"]["rms"] < r["a_old"]["all"]["rms"])(
            json.loads(hs.read_text())["train"])
        if missing or not good:
            self.log(f"SAC skipped: missing {missing}, hard-stop fit good: {good}")
            return
        s = Path(step["session"])
        hand = Hand(self.stream)
        if not (s / "contacts.jsonl").exists():
            sac.CONTACTS_PER_PAIR = step.get("per_pair", sac.CONTACTS_PER_PAIR)
            n = sac.run(s, hand, self.log, fingers=step.get("fingers"))
            self.log(f"SAC: {n} contacts")
        dense = Path(self.spec.get("dense_session") or
                     next(x["session"] for x in self.spec["steps"] if x["name"].endswith("dense")))
        try:
            fit = sac_fit.main([dense], s, self.spec["runs"]["A"], step["run"])
        except (Exception, SystemExit) as e:
            self.log(f"SAC fit failed: {e!r}")
            return
        self.log(f"SAC fit: gaps {fit['gap_before_mm']} -> {fit['gap_after_mm']} mm")
        kap = next(x for x in self.spec["steps"] if x["kind"] == "kapandji")
        ks = Path(kap["session"])
        plan = json.loads((ks / "plan.json").read_text())
        if not any(c.get("map") == "sac" for c in plan["configs"]):
            new = hardstops.Linear(step["run"], {j: v["sac"] for j, v in fit["stops"].items()})
            old = hardstops.Linear(step["run"])
            touch = json.loads((STUDY / "touch_poses.json").read_text())
            for f, t in touch.items():
                cmd = {j: round(old.joint(j, new.motor(j, min(max(v, new.rom[j][0]), new.rom[j][1]))), 2)
                       for j, v in t["q"].items()}
                plan["configs"].append({"id": f"kap-{f}-sac", "q": cmd, "q_wanted": t["q"], "kind": "kapandji",
                                        "finger": f, "map": "sac", "speed": 5.0,
                                        "via": [dict(REST), {j: round(0.8 * v, 2) for j, v in cmd.items()}]})
            (ks / "plan.json").write_text(json.dumps(plan, indent=1))
        hand.torque(True)
        try:
            hand.move(dict(REST))
            time.sleep(1.0)
            call("/tactile/zero", {})
        finally:
            hand.torque(False)
        self.collect({**kap, "name": "N11-kapandji-sac"})

    def workers(self) -> list[subprocess.Popen]:
        env_py = sys.executable
        cmd = [env_py, "-m", "hand_study.worker", *self.spec["sessions"], "--follow"]
        out = []
        for i in range(WORKERS):
            f = (NIGHT / f"{self.tag}-worker{i}.log").open("a")
            out.append(subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=Path(__file__).parents[1]))
        return out

    def run(self, dry: bool = False) -> None:
        self.log(f"night {self.tag}: {len(self.spec['steps'])} steps, deadline {time.strftime('%H:%M', time.localtime(self.deadline))}")
        if dry:
            for st in self.spec["steps"]:
                self.log(f"  {st['name']:16s} {'done' if st['name'] in self.state['done'] else 'todo'}")
            return
        self.save(state="running", pid=__import__("os").getpid())
        self.stream = Stream()
        try:
            if Turntable().state().get("busy"):
                raise Abort("the turntable is busy")
            for st in self.spec["steps"]:
                if st["name"] in self.state["done"]:
                    continue
                if time.time() > self.deadline and not st["name"].startswith("N09"):
                    self.log(f"deadline reached before {st['name']}")
                    break
                for attempt in range(RETRIES + 1):
                    self.save(step=st["name"])
                    self.log(f"-> {st['name']}")
                    t0 = time.time()
                    try:
                        getattr(self, st["kind"])(st)
                        break
                    except Abort:
                        raise
                    except Exception as e:
                        self.safe_off()
                        self.state["errors"].append({"step": st["name"], "error": repr(e), "t": time.time()})
                        self.log(f"error in {st['name']}: {e!r}\n{traceback.format_exc()}")
                        if attempt == RETRIES:
                            raise
                        if "turntable" in repr(e):
                            self.reconnect_plate()
                        for _ in range(6):
                            beat("retry wait")
                            time.sleep(10)
                    finally:
                        self.safe_off()
                self.state["done"][st["name"]] = {"t0": t0, "t1": time.time()}
                self.save()
            self.save(state="done")
            self.log("night done")
        except BaseException as e:
            self.save(state="aborted", error=repr(e))
            self.log(f"ABORTED: {e!r}")
            raise
        finally:
            self.safe_off()
            self.stream.close()
            self.log("torque off (hand + plate)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--hours", type=float, default=8.0)
    p.add_argument("--dry", action="store_true")
    p.add_argument("--workers", action="store_true", help="also start the solve workers")
    a = p.parse_args()
    n = Night(a.tag, a.hours)
    procs = n.workers() if a.workers and not a.dry else []
    n.run(a.dry)
