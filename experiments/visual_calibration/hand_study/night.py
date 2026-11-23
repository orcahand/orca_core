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
import os
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

from hand_study import collect, hw, wear
from hand_study.configs import JOINTS, ROM


def ROM_OF(j: str) -> tuple[float, float]:
    import yaml
    try:
        return tuple(map(float, yaml.safe_load(BENCH.joinpath("config.yaml").read_text())["joint_roms"][j]))
    except Exception:
        return tuple(map(float, ROM[j]))


def collect_joints() -> list[str]:
    return list(JOINTS)


def projected_gb(spec: dict) -> float:
    """What the night writes: 4K JPEG q95 photos (~0.7 MB), slack films (960 px, ~1.5k frames a check),
    full-rate telemetry (~25 MB/h), masks and fits (~0.3 MB a pose)."""
    poses = sum(int(s.get("poses") or 0) for s in spec["steps"] if s["kind"] in ("collect", "adaptive", "free"))
    poses += sum(int(s.get("poses") or 0) // 3 for s in spec["steps"] if s["kind"] == "collision")
    views = 5.3
    slack = sum(s["kind"] == "slack" for s in spec["steps"])
    return poses * views * 0.7e-3 + poses * 0.3e-3 + slack * 1500 * 45e-6 + spec.get("hours", 8) * 0.025 + 0.2
from hand_study.hw import TT, Abort, FaultLog, Hand, MotorFault, Stream, Turntable, Wrapped, beat, call, recover
from hand_study import paths
from hand_study.paths import STUDY, UI_STATE

TT_STATE = TT

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
        self.state.pop("cut_short", None)            # a resumed night is not cut short (yet)
        self.deadline = self.state.setdefault("deadline", self.state["started"] + hours * 3600)
        global BENCH
        BENCH = Path(self.spec.get("bench") or BENCH).expanduser()
        hw.MOTORS = self.spec.get("motors", hw.MOTORS)
        from hand_study import hardstops
        hardstops.RMS_MAX_PX = float(self.spec.get("rms_max_px", hardstops.RMS_MAX_PX))
        hw.TRACK_MA = float(self.spec.get("track_ma", hw.TRACK_MA))
        hw.RESILIENT = bool(self.spec.get("resilient"))
        hw.HEARTBEAT = NIGHT / f"{tag}.heartbeat"
        hw.FAULTS = FaultLog(NIGHT / f"{tag}-faults.jsonl")

    NOTE_ON = ("MOTOR FAULT", "HOT at", "WRAP", "cooled to", "cooling:", "turntable to", "turntable re-homed",
               "turntable reconnect", "restarting orca-ui", "orca-ui back", "own stops from", "mount ", "recalibrating after",
               "deadline reached", "night done", "ABORTED", "preflight:")

    def log(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        with self.log_path.open("a") as f:
            f.write(line + "\n")
        if msg.startswith(self.NOTE_ON) or (msg.startswith("motors ") and " back" in msg):
            self.note(msg.split("\n")[0])

    def note(self, text: str) -> None:
        """One line to orca-ui's study feed (the Study tab), as "runner"; never blocks or fails the night."""
        import threading
        import urllib.request
        if self.spec.get("mode") != "protocol" or os.environ.get("STUDY_NO_NOTES"):
            return
        url = hw.UI + f"/visual-calibration/study/notes/{self.tag}"
        body = json.dumps({"who": "runner", "text": text[:240]}).encode()

        def post():
            try:
                urllib.request.urlopen(urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"},
                                                              method="POST"), timeout=5).read()
            except Exception:
                pass
        threading.Thread(target=post, daemon=True).start()

    def step_line(self, st: dict) -> str:
        what = f"{st['name']} ({st.get('label') or st['kind']})"
        if st.get("session"):
            c = Path(st["session"]) / "configs"
            return f"{what}: {len(list(c.glob('*/meta.json')))}/{st.get('poses')} poses"
        return what

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
            if self.spec.get("resilient"):
                call("/torque/disable", {}, timeout=20)          # the hand still while the plate homes
                st = call("/visual-calibration/turntable/home", {}, timeout=120)
                call("/visual-calibration/turntable/torque-off", {}, timeout=20)
                self.log(f"turntable re-homed: {st.get('angle')}")
        except Exception as e:
            self.log(f"turntable reconnect failed: {e}")

    def connected(self, wait_s: float = 10.0) -> bool:
        t0 = time.time()
        while True:
            try:
                if call("/status", timeout=5).get("state") == "connected":
                    return True
            except Exception:
                pass
            if time.time() - t0 > wait_s:
                return False
            beat("waiting for the hand")
            time.sleep(3)

    def board_of_hand(self) -> str | None:
        """The hand's motor port now (it may have re-enumerated, e.g. usbmodem1101 -> 1103)."""
        try:
            bs = [b for b in call("/boards", timeout=60)["boards"] if b.get("hand_id") == self.spec["hand"] and b.get("role") != "rig"]
        except Exception:
            return None
        return bs[0]["device"] if bs else None

    def unstick_bus(self) -> None:
        """'Port is in use' / failing sync reads for > hw.BUS_STUCK_S: reconnect the hand; still
        stuck 15 s later, restart orca-ui (one instance, on the hand's current port)."""
        self.log("bus stuck: reconnecting the hand")
        try:
            call("/reconnect", {}, timeout=30)
        except Exception as x:
            self.log(f"bus reconnect: {x}")
        self.connected(60)
        for _ in range(15):
            beat("bus check")
            time.sleep(1)
        if hw.BUS_STUCK[0] and time.time() - hw.BUS_STUCK[0] > hw.BUS_STUCK_S:
            self.log("bus still stuck after the reconnect: restarting orca-ui")
            self.ui_restart(board=self.board_of_hand())
            hw.BUS_STUCK[0] = 0.0

    def ladder(self, e: BaseException, st: dict, attempt: int) -> None:
        """Clemens' escalation ladder for any fault: cool, clear/reboot, re-read (recalibrate on a lost
        turn), reconnect the board (or its new port), restart orca-ui, re-home the plate, check the
        mount; the step then resumes at its first unfinished pose. Only hw.Safety ends the night."""
        self.note(f"fault in {st['name']}: {str(e)[:150]}; recovering (attempt {attempt + 1})")
        self.safe_off()
        steps = []
        try:                                                        # 1 torque off, cool
            h = Hand(self.stream)
            if h.hottest() >= hw.PAUSE_TEMP_C:
                h.cool_if_hot(self.log, cool_to=hw.COOL_TO_C, max_wait_s=3600)
                h.torque(False)
                steps.append("cooled")
        except hw.Safety:
            raise
        except Exception as x:
            self.log(f"ladder 1 (cool): {x}")
        try:                                                        # 2 clear errors / reboot, 3 lost turn
            h = Hand(self.stream)
            bad = [Hand._fault_info(x) for x in h.motors()["motors"] if x["hw_error"] and x["joint"] not in hw.DEAD]
            if bad:
                recover(h, MotorFault("hardware error", bad), st["name"], self.log)
                steps.append("rebooted")
        except Wrapped:
            self.rewrap(st)
            steps.append("recalibrated (lost turn)")
        except hw.Safety:
            raise
        except Exception as x:
            self.log(f"ladder 2 (reboot): {x}")
        if isinstance(e, hw.BusStuck) or (hw.BUS_STUCK[0] and time.time() - hw.BUS_STUCK[0] > hw.BUS_STUCK_S):
            self.unstick_bus()                                      # 4a a stuck bus: reconnect, else restart orca-ui
            steps.append("bus unstuck")
            self.stream.close()
            self.stream = Stream()
        if not self.connected():                                    # 4 the board
            try:
                call("/reconnect", {}, timeout=30)
            except Exception as x:
                self.log(f"ladder 4 (reconnect): {x}")
            ok = self.connected(90)
            if not ok:
                dev = self.board_of_hand()
                if dev:
                    try:
                        call("/board/select", {"device": dev}, timeout=30)
                        call("/reconnect", {}, timeout=30)
                    except Exception as x:
                        self.log(f"ladder 4 (board {dev}): {x}")
                    ok = self.connected(90)
            if not ok:                                              # 5 orca-ui (one instance: the same one restarted)
                try:
                    self.ui_restart(board=self.board_of_hand())
                    ok = True
                    steps.append("orca-ui restarted")
                except Exception as x:
                    self.log(f"ladder 5 (restart orca-ui): {x}")
            else:
                steps.append("board reconnected")
            self.stream.close()
            self.stream = Stream()
        if "turntable" in repr(e) or "plate" in repr(e) or any("orca-ui" in x or "board" in x or "bus" in x for x in steps):
            self.reconnect_plate()                                  # 6 re-home the plate
            steps.append("plate re-homed")
        if any("orca-ui" in x or "board" in x or "bus" in x or "recalibrated" in x for x in steps):
            self.mount_due = True                                   # 7 mount check before the next block
        self.log(f"recovered from {type(e).__name__} in {st['name']}: {', '.join(steps) or 'torque off and wait'}; resuming")
        self.note(f"recovered in {st['name']}: {', '.join(steps) or 'torque off and wait'}; resuming at the next unfinished pose")

    def mount_check(self, run: str) -> None:
        """One zero pose with the mount solved free, in the next free MC session (precreated, so the
        workers know it): after a reconnect or a recalibration the analysis checks the fixed mount."""
        free = [c for c in sorted(STUDY.glob(f"sessions/{self.tag}-MC*"))
                if not json.loads((c / "plan.json").read_text())["configs"]]
        self.mount_due = False
        if not free:
            self.log("mount check: no free MC session left")
            return
        s = free[0]
        plan = json.loads((s / "plan.json").read_text())
        plan.update(calibration=run, configs=[{"id": "zero", "q": {}, "kind": "repeat"}])
        (s / "plan.json").write_text(json.dumps(plan, indent=1))
        self.log(f"mount check -> {s.name}")
        collect.run(s, log=self.log)

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
        call("/operation/calibrate/start", {"params": step.get("params", {})})
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
        self.archive_hand(out)
        (out / "operation_log.json").write_text(json.dumps(log, indent=1))
        self.check_archive(out)
        call("/torque/disable", {})
        self.log(f"orca_core calibration done in {time.time() - t0:.0f} s -> calib_{run}")

    def snapshot(self, step: dict) -> None:
        """The hand's current orca_core calibration archived as calib_<run> (no new calibration),
        checked: the linear map from those files must give orca_core's live estimate."""
        from hand_study.mapping import HAND_MAP
        out = STUDY / f"calib_{step['run']}"
        if not (out / HAND_MAP).exists():                   # written last: the archive is complete
            self.archive_hand(out)
        self.check_archive(out)
        self.log(f"calibration archived as calib_{step['run']} (matches the live estimate)")

    def archive_hand(self, out: Path) -> None:
        """The connected hand's calibration as orca_core uses it, into `out`: hand_map.yaml (each
        joint's motor, direction and range in force, from orca-ui's /hand/info) and the files of the
        hand's folder (orca_core 0.14 keeps them in <ORCA_HOME>/hands/<id>/; the config is the
        packaged model's and holds no hand's numbers)."""
        import yaml
        from hand_study.mapping import HAND_FILES, HAND_MAP, hand_map
        m = hand_map(call("/hand/info"))
        if not m["folder"] or not (Path(m["folder"]) / "calibration.yaml").exists():
            raise Abort(f"no calibration on file for the connected hand ({m['hand']}, folder {m['folder']})")
        out.mkdir(parents=True, exist_ok=True)
        for f in HAND_FILES:
            if (Path(m["folder"]) / f).exists():
                shutil.copy2(Path(m["folder"]) / f, out / f)
        (out / HAND_MAP).write_text(yaml.safe_dump(m, sort_keys=False))

    def check_archive(self, out: Path) -> None:
        """The linear map rebuilt from the archive must give orca_core's live estimate."""
        from hand_study.mapping import LinearMap
        L = LinearMap(out)
        m = call("/motors/direct")["motors"]
        mine = L.joints_of([x["id"] for x in m], [x["position"] for x in m])
        live = Hand(self.stream).estimate()
        off = {j: round(mine[j] - live[j], 2) for j in mine if j in live and abs(mine[j] - live[j]) > 0.5}
        if off or not live:
            raise Abort(f"{out.name} does not match the live estimate: {off or 'no estimate'}{self.not_in_force(out)}")

    @staticmethod
    def not_in_force(out: Path) -> str:
        """Why an archive may not match: the hand's files changed since it connected (new joint ranges
        or calibration on file are put in force at the next connect)."""
        import yaml
        from hand_study.mapping import HAND_MAP, measured_in_force, same_ranges, stored_ranges
        ran = yaml.safe_load((out / HAND_MAP).read_text()) or {}
        try:
            on_file = stored_ranges(Path(ran["folder"]), ran["model"])
        except (KeyError, TypeError, ValueError, OSError):
            return ""
        if same_ranges(on_file, measured_in_force(ran)):
            return ""
        return ("; the joint ranges on file for this hand are not the ones in force yet: reconnect the hand "
                "in orca-ui (they take effect at connect), then run again")

    def mount(self, step: dict) -> None:
        """The baseline pose's solved mount, fixed in every later block's plan."""
        e = Path(step["from"]) / "configs" / "zero" / "est.json"
        t0 = time.time()
        while not e.exists():
            if time.time() - t0 > 1800:
                raise Abort("the baseline pose was not solved in 30 min")
            self.check_workers()
            beat("waiting for the baseline solve")
            time.sleep(10)
        est = json.loads(e.read_text())
        if est.get("suspect") or est["rms_px"] > 12:          # rig 330 + crop: 9.6 px is a normal baseline
            raise Abort(f"baseline fit not good enough for the mount: rms {est['rms_px']}, suspect {est.get('suspect')}")
        for st in self.spec["steps"]:
            if st.get("session"):
                pp = Path(st["session"]) / "plan.json"
                plan = json.loads(pp.read_text())
                if plan.get("mount") == "pending":
                    plan["mount"] = est["mount"]
                    pp.write_text(json.dumps(plan, indent=1))
        (STUDY / f"mount_{self.tag}.json").write_text(json.dumps({"mount": est["mount"], "rms_px": est["rms_px"],
                                                                   "from": str(e)}, indent=1))
        self.log(f"mount {est['mount']} (baseline rms {est['rms_px']} px) fixed in the later plans")
        from hand_study import estimate, views
        if self.spec.get("travel"):
            views.TRAVEL = tuple(self.spec["travel"])
        if estimate.FULL_ONLY:
            todo = [Path(st["session"]) for st in self.spec["steps"] if st.get("session") and st["name"] != step["name"]
                    and not json.loads((Path(st["session"]) / "plan.json").read_text()).get("views_planned")
                    and json.loads((Path(st["session"]) / "plan.json").read_text()).get("mount") == est["mount"]]
            views.main(todo, log=self.log)

    def preflight(self, step: dict) -> None:
        """The hand the spec names is the one connected, every motor answers without error, the
        plate and camera are there, the disk has room."""
        st = call("/status")
        hand = (call("/hand/info").get("hand") or {}).get("id")         # orca_core's hand id (its folder's name)
        if self.study_map_on():                                          # a study must see orca_core's own map
            call("/visual-calibration/study-map", {"mode": "off"})
            self.log("preflight: the study map was on in orca-ui; switched to standard")
        want = self.spec.get("hand")
        if st.get("state") != "connected":
            raise Abort(f"hand not connected: {st.get('state')}")
        if want and hand != want:
            raise Abort(f"connected hand is {hand!r}, not {want}")
        cfg = f"{st.get('model')} on {st.get('board_pinned')}"
        m = call("/motors/direct")["motors"]
        bad = [x["id"] for x in m if x["hw_error"]]
        if len(m) != 17 or bad:
            raise Abort(f"motors: {len(m)} found, in error {bad}")
        if not call(TT_STATE).get("connected"):
            self.reconnect_plate()
        rig = self.check_rig()
        import shutil as sh
        free = sh.disk_usage(Path.home()).free / 1e9
        self.log(f"preflight: {want} connected ({cfg}), 17 motors ok, hottest {max(x['temp_c'] for x in m):.0f} °C, "
                 f"plate {call(TT_STATE).get('angle')}, {rig}, {free:.0f} GB free")

    def check_rig(self) -> str:
        """The rig calibration this run's solves use is on file (in the rig's folder, or the one left
        in the checkout) and is the one orca-ui uses: the same file, calibrated at the same time."""
        from visual_calibration.rig_config import RigConfig, RigConfigError
        try:
            rig = RigConfig.load(rig_id=paths.run_rig_id())
            path = rig.calibration_path
        except RigConfigError as e:
            raise Abort(f"rig calibration: {e}") from e
        at = rig.load_calibration().get("calibrated_at")
        if not at:
            raise Abort(f"no rig calibration at {path}: calibrate the rig (motor, 1a, 1b) before measuring a hand")
        ui = call("/visual-calibration/status")
        theirs = ui.get("calibration_path")
        if not theirs or os.path.realpath(theirs) != os.path.realpath(path) or ui.get("calibrated_at") != at:
            raise Abort(f"orca-ui uses the rig calibration {theirs or '(not reported)'} of {ui.get('calibrated_at')}, "
                        f"this run {path} of {at}: run both on the same rig and ORCA_HOME, and restart orca-ui "
                        f"after a new rig calibration")
        return f"rig calibrated {at}"

    @staticmethod
    def study_map_on() -> bool:
        """Whether orca-ui applies a study map; one started without --research-ui has none (404)."""
        try:
            return bool(call("/visual-calibration/study-map").get("enabled"))
        except hw.ApiError as e:
            if ": 404 " in str(e):
                return False
            raise

    def ui_process(self) -> tuple[str | None, str | None, str | None]:
        """(pid, command line, cwd) of the orca-ui on hw.UI's port; the last ones seen are kept, so a
        crashed instance can be started again the same way."""
        port = hw.UI.rsplit(":", 1)[1].split("/")[0]
        pid = subprocess.run(["lsof", "-ti", f"tcp:{port}", "-sTCP:LISTEN"], capture_output=True, text=True).stdout.split()
        if pid:
            pid = pid[0]
            cmd = subprocess.run(["ps", "-o", "command=", "-p", pid], capture_output=True, text=True).stdout.strip()
            cwd = next((l[1:] for l in subprocess.run(["lsof", "-a", "-p", pid, "-d", "cwd", "-Fn"], capture_output=True,
                                                       text=True).stdout.split() if l.startswith("n")), str(Path.home()))
            self.ui_seen = (cmd, cwd)
            return pid, cmd, cwd
        cmd, cwd = getattr(self, "ui_seen", (None, None))
        return None, cmd, cwd

    def ui_restart(self, board: str | None = None) -> None:
        """orca-ui reads a hand's config once, when it adopts the hand: a changed config.yaml takes
        a restart. The running instance's own command line and folder are reused (one instance only);
        `board`: a new --board (the port re-enumerated)."""
        pid, cmd, cwd = self.ui_process()
        if cmd is None:
            raise RuntimeError("no orca-ui seen on the port this night: cannot restart it")
        self.safe_off()
        self.stream.close()
        self.log(f"restarting orca-ui ({cmd} in {cwd})")
        if pid:
            subprocess.run(["kill", pid])
            t0 = time.time()
            while subprocess.run(["kill", "-0", pid], capture_output=True).returncode == 0:
                if time.time() - t0 > 30:
                    subprocess.run(["kill", "-9", pid])
                time.sleep(0.5)
        log = open(UI_STATE / f"orca-ui-dev.{time.strftime('%H%M%S')}.log", "a")
        argv = cmd.split()
        if board and "--board" in argv:
            argv[argv.index("--board") + 1] = board
            self.ui_seen = (" ".join(argv), cwd)
        env = {k: v for k, v in __import__("os").environ.items()
               if k not in ("ORCA_RIG_AUTODETECT", "PYTHONPATH") and not k.startswith("STUDY_")}
        subprocess.Popen(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        t0 = time.time()
        while True:
            try:
                if call("/status", timeout=5).get("state") == "connected":
                    break
            except Exception:
                pass
            if time.time() - t0 > 180:
                raise RuntimeError("orca-ui did not come back with the hand in 3 min")
            beat("waiting for orca-ui")
            time.sleep(2)
        self.stream = Stream()
        self.reconnect_plate()
        self.log(f"orca-ui back after {time.time() - t0:.0f} s")

    def wait_for_solves(self, sessions: list[Path], what: str) -> None:
        """Until the solve workers have done (or failed) 97% of the poses of `sessions`, at most 60 min."""
        t0 = time.time()
        while True:
            n_meta = sum(len(list((s / "configs").glob("*/meta.json"))) for s in sessions)
            n_est = sum(len(list((s / "configs").glob("*/est.json"))) + len(list((s / "configs").glob("*/est.json.error")))
                        for s in sessions)
            if n_meta and n_est >= 0.97 * n_meta:
                return
            if time.time() - t0 > 3600:
                self.log(f"{what}: only {n_est}/{n_meta} solved after 60 min, fitting with those")
                return
            self.check_workers()
            beat(f"{what}: waiting for solves {n_est}/{n_meta}")
            time.sleep(15)

    @staticmethod
    def check_workers() -> None:
        """A solve worker that died (run_hand's workers.json) ends the night: nothing would solve its poses."""
        workers = paths.read_json(NIGHT / "workers.json") or []
        dead = [w["log"] for w in workers if not w.get("alive", True) and w.get("returncode") not in (None, -15)]
        if dead:
            raise Abort(f"a solve worker died: see {', '.join(dead)}")

    def ranges(self, step: dict) -> None:
        """The hand's own hard stops from the short block (hardstops), recorded as its joint_ranges.yaml
        in its folder on this computer (in force at its next connect); proposed/ keeps the fit."""
        from hand_study import hardstops
        sessions = [Path(s) for s in step["from"]]
        self.wait_for_solves(sessions, "ranges")
        res = hardstops.main(sessions, step["run"], hand_id=self.spec["hand"])
        self.log(f"{res['n']} poses: {hardstops.ranges_summary(res['joint_ranges'])}")
        self.note(f"joint ranges recorded for {self.spec['hand']}: {', '.join(res['joint_ranges']['recorded']) or 'none'}")

    def accuracy(self, step: dict) -> None:
        """The hand's accuracy on the validation block before and after the ranges step's stops
        (quick.check), as proposed/accuracy.json beside its hardstops.json."""
        from hand_study import hardstops, quick
        val = Path(step["from"])
        self.wait_for_solves([val], "accuracy")
        fit = paths.read_json(hardstops.OUT / "hardstops.json")
        res = quick.check(val, step["run"], fit["stops"], (fit.get("joint_ranges") or {}).get("recorded"))
        paths.write_json(hardstops.OUT / "accuracy.json", res)
        a, b = res["before"]["all"] or {}, res["after"]["all"] or {}
        self.log(f"accuracy on {res['n']} validation poses: rms {a.get('rms')} -> {b.get('rms')} deg, "
                 f"max {a.get('max')} -> {b.get('max')} deg")

    def refit(self, step: dict) -> None:
        """The hand's own hard stops from the short block (hardstops.fit), written into its
        config.yaml (joint_roms only; the old one kept as config.<backup>.yaml), orca-ui restarted."""
        from hand_study import analysis, hardstops
        sessions = [Path(s) for s in step["from"]]
        self.wait_for_solves(sessions, "refit")
        out = Path(self.spec["protocol_dir"]) if self.spec.get("protocol_dir") else STUDY / "proposed"
        out.mkdir(parents=True, exist_ok=True)
        R = analysis.rows(sessions)
        stops = hardstops.fit(R, step["run_from"])
        cfg = BENCH / "config.yaml"
        backup = BENCH / f"config.{step.get('backup', 'before_refit')}.yaml"
        if not backup.exists():
            shutil.copy2(cfg, backup)
        text, diff = hardstops.proposed_config(backup, stops)
        (out / f"{step['name']}.json").write_text(json.dumps({"n": len(R), "stops": stops, "diff": diff}, indent=1))
        cfg.write_text(text)
        self.log(f"own stops from {len(R)} poses written to {cfg} ({len(diff)} numbers changed; backup {backup.name})")
        self.ui_restart()

    def exercise(self, step: dict) -> None:
        """orca_core's setup motion test (scripts/setup.py run_motion_test): open_hand <-> power_grasp
        (main demo poses as ROM fractions of the config in force, neutral elsewhere), a 0.5 s ramp,
        2 s hold, for step['minutes']; the plate off and still. Cooling pauses do not count."""
        import yaml
        from orca_core.demo_poses import load_demo_poses
        cfg = yaml.safe_load((BENCH / "config.yaml").read_text())
        rom, neutral = cfg["joint_roms"], cfg.get("neutral_position") or {}
        poses = load_demo_poses()["main"].pose_fractions

        def pose(fr):
            q = {j: float(neutral.get(j, 0.0)) for j in rom}
            q.update({j: rom[j][0] + float(f) * (rom[j][1] - rom[j][0]) for j, f in fr.items() if j in rom})
            return {j: min(max(v, rom[j][0]), rom[j][1]) for j, v in q.items()}
        seq = [pose(poses["open_hand"]), pose(poses["power_grasp"])]
        call("/visual-calibration/turntable/torque-off", {}, timeout=20)
        hand = Hand(self.stream)
        hand.torque(True)
        done, k, logp = float(self.state.get("exercise_s", {}).get(step["name"], 0.0)), 0, NIGHT / f"{self.tag}-{step['name']}.jsonl"
        try:
            while done < step["minutes"] * 60:
                t0 = time.time()
                goal = seq[k % 2]
                span = max(abs(goal[j] - hand.target.get(j, 0.0)) for j in goal)
                hand.move(goal, max(30.0, span / 0.5))
                time.sleep(2.0)
                hand.guard()
                k += 1
                done += time.time() - t0
                if k % 20 == 0:
                    s = hand.sample(n=2)
                    with logp.open("a") as f:
                        f.write(json.dumps({"t": time.time(), "cycles": k, "s": round(done), "temp_c": s["temp_c"],
                                            "current_ma": s["current_ma"]}) + "\n")
                    self.state.setdefault("exercise_s", {})[step["name"]] = done
                    self.save()
                    beat(f"exercise {k} moves, {done / 60:.1f} min")
                    if max(s["temp_c"]) >= hw.PAUSE_TEMP_C:
                        hand.cool_if_hot(self.log)
            hand.move(seq[0], 30.0)
        finally:
            hand.torque(False)
        self.log(f"{step['name']}: {k} moves, {done / 60:.1f} min")

    def rewrap(self, step: dict) -> None:
        """A reboot lost a motor's turn: recalibrate orca_core (its connect re-places the turn, the
        calibration a clean run), and every session not yet done is tracked with the new run."""
        if os.environ.get(paths.CONSOLE_ENV):     # orca-ui's hand run: no operation may take the hand meanwhile
            raise Abort("a motor lost its turn: recalibrate the hand, then Measure again")
        k = self.state.setdefault("rewraps", 0) + 1
        self.state["rewraps"] = k
        base = step.get("run") or self.spec["runs"]["B"]
        label = f"{base}w{k}"
        self.log(f"recalibrating after a lost turn -> {label}")
        self.calibrate({"run": label, "params": {"force_wrist": True} if hw.RESILIENT else {}})
        for st in self.spec["steps"]:
            if st["name"] in self.state["done"] or not st.get("session"):
                continue
            plan_path = Path(st["session"]) / "plan.json"
            if plan_path.exists():
                plan = json.loads(plan_path.read_text())
                plan["calibration"] = label
                plan_path.write_text(json.dumps(plan, indent=1))
            st["run"] = label
        (NIGHT / f"{self.tag}.json").write_text(json.dumps(self.spec, indent=1))
        self.save()

    def collect(self, step: dict) -> None:
        s = Path(step["session"])
        until = self.deadline if step["name"].endswith("extra") else None
        plan = json.loads((s / "plan.json").read_text())
        if step.get("adaptive") and not plan.get("adapted") and not list((s / "configs").glob("*/meta.json")):
            from hand_study import adaptive
            try:
                st = adaptive.state(self.spec)
                cut = adaptive.prune(self.tag, plan, st) if st["joints"] else []
                plan["adapted"] = {"t": time.time(), "skipped": cut}
                (s / "plan.json").write_text(json.dumps(plan, indent=1))
                if cut:
                    js = sorted({c.split("-")[1] for c in cut if c.count("-") >= 2})
                    self.note(f"adaptive {step['name']}: {len(cut)} poses skipped, already well determined ({', '.join(js[:6])})")
            except Exception as e:
                self.log(f"adaptive pruning skipped: {e!r}")
        if until:
            left = int((until - time.time()) / 40)
            if left <= 0:
                self.log("no time left for the extra block")
                return
            collect.run(s, limit=left, log=self.log)
        else:
            collect.run(s, limit=step.get("limit"), log=self.log)
        self.log(f"{step['name']}: {len(list((s / 'configs').glob('*/meta.json')))}/{len(plan['configs'])} poses")

    def adaptive(self, step: dict) -> None:
        """D5: poses where the data so far say the model is weakest, and re-aimed failed poses."""
        from hand_study import adaptive
        s = Path(step["session"])
        plan = json.loads((s / "plan.json").read_text())
        if not plan["configs"]:
            st = adaptive.state(self.spec)
            b, made = adaptive.densify(self.tag, st, n_max=int(step.get("poses", 30)))
            plan.update(configs=b.out, skipped=b.skipped, adapted={"t": time.time(), "made": made})
            (s / "plan.json").write_text(json.dumps(plan, indent=1))
            top = "; ".join(m["why"] for m in made[:3])
            self.note(f"adaptive D5: {len(b.out)} poses from {st['n']} solved training poses; first: {top}")
        collect.run(s, limit=step.get("limit"), log=self.log)

    def free(self, step: dict) -> None:
        """The free experiment: protocol_dir/free_plan.json ({"why": ..., "configs": [...]}) written by
        Isha from the data, or after 10 min the automatic design (adaptive.free_design); the plan and its
        reason go to the Study feed before anything moves."""
        from hand_study import adaptive
        s = Path(step["session"])
        plan = json.loads((s / "plan.json").read_text())
        if not plan["configs"]:
            f = Path(self.spec["protocol_dir"]) / "free_plan.json"
            self.note("free experiment: waiting up to 10 min for a plan (protocol/free_plan.json), else the automatic design")
            t0 = time.time()
            while not f.exists() and time.time() - t0 < 600 and not step.get("limit"):
                beat("free experiment: waiting for a plan")
                time.sleep(10)
            if f.exists():
                fp = json.loads(f.read_text())
                configs, why = fp["configs"], fp["why"]
            else:
                res = Path(self.spec["protocol_dir"]) / "results.json"
                b, why = adaptive.free_design(self.spec, adaptive.state(self.spec), json.loads(res.read_text()) if res.exists() else None,
                                              n=int(step.get("poses", 40)))
                configs = b.out
            plan.update(configs=configs, why=why)
            (s / "plan.json").write_text(json.dumps(plan, indent=1))
            self.note(why)
        collect.run(s, limit=step.get("limit"), log=self.log)

    def slack(self, step: dict) -> None:
        from hand_study import slack
        from hand_study.hw import Camera
        slack.run(Path(step["session"]), Camera(), self.log, limit=step.get("limit"))
        self.note(f"{step['name']} done (deadzones are computed offline, Study tab)")

    def collision(self, step: dict) -> None:
        from hand_study import collision
        prs = json.loads(Path(self.spec["collision_pairs"]).read_text())
        if step.get("pairs"):
            prs = [p for p in prs if p["id"] in step["pairs"]]
        if step.get("limit"):
            prs = prs[: step["limit"]]
        self.note(f"collisions: {len(prs)} URDF contacts, moving motor at {collision.CAP_MA:.0f} mA, obstacle held")
        collision.run(Path(step["session"]), prs, reps=1 if step.get("limit") else collision.REPS, log=self.log)
        ids = {p["id"] for p in prs}
        res = [json.loads(f.read_text()) for f in Path(step["session"]).glob("configs/*/contact.json")]
        res = [r for r in res if r["pair"]["id"] in ids]
        n = sum(r["result"] == "contact" for r in res)
        bad = sum(r["result"] == "unexpected" for r in res)
        self.note(f"{step['name']} done: {n}/{len(res)} contacts inside the URDF window" + (f", {bad} unexpected stalls" if bad else ""))

    def revisit(self, step: dict) -> None:
        """Poses left out while a motor was dead: every collect session again (shot poses are skipped)."""
        if hw.DEAD:
            self.log(f"revisit: motors still dead {sorted(hw.DEAD)}")
        for st in self.spec["steps"]:
            if st["kind"] == "collect" and not st["name"].endswith("extra") and st["name"] in self.state["done"]:
                s = Path(st["session"])
                plan = json.loads((s / "plan.json").read_text())
                left = [c for c in plan["configs"] if not (s / "configs" / c["id"] / "meta.json").exists()
                        and not (s / "configs" / c["id"] / "skipped.json").exists()]
                if left:
                    self.log(f"revisit {st['name']}: {len(left)} poses")
                    collect.run(s, limit=st.get("limit"), log=self.log)

    def archive(self, step: dict) -> None:
        """The reference copy: photos, motor logs, calibrations, maps, configs, results (nothing moved)."""
        dest = Path(step["dest"])
        dest.mkdir(parents=True, exist_ok=True)
        for sp in self.spec["sessions"]:
            if Path(sp).exists():
                shutil.copytree(sp, dest / "sessions" / Path(sp).name, dirs_exist_ok=True)
        for f in NIGHT.glob(f"{self.tag}*"):
            if f.is_file():
                (dest / "night").mkdir(exist_ok=True)
                shutil.copy2(f, dest / "night" / f.name)
        for run in set(self.spec["runs"].values()):
            if (STUDY / f"calib_{run}").exists():
                shutil.copytree(STUDY / f"calib_{run}", dest / "calibrations" / f"calib_{run}", dirs_exist_ok=True)
        if Path(self.spec["protocol_dir"]).exists():
            shutil.copytree(self.spec["protocol_dir"], dest / "protocol", dirs_exist_ok=True)
        (dest / "configs").mkdir(exist_ok=True)
        for f in BENCH.glob("config*.yaml"):
            shutil.copy2(f, dest / "configs" / f.name)
        for f in (Path(self.spec["collision_pairs"]), STUDY / "advanced_seed.json"):
            if f.exists():
                shutil.copy2(f, dest / f.name)
        (dest / "README.txt").write_text(f"{self.spec.get('label', '')}: night {self.tag}, archived {time.strftime('%Y-%m-%d %H:%M')}.\n"
                                         "sessions/: photos + meta (motor positions, currents, temperatures) + solves; night/: spec, "
                                         "state, logs, faults, exercise/adaptive logs; calibrations/: orca_core's runs; configs/: "
                                         "the hand's config.yaml versions; protocol/: results.json, fit ablations, refit stops.\n")
        man = {}
        for sub in sorted(p for p in dest.iterdir() if p.is_dir()):
            fs = [f for f in sub.rglob("*") if f.is_file()]
            kinds = {}
            for f in fs:
                k = f.suffix or f.name
                kinds.setdefault(k, [0, 0])
                kinds[k][0] += 1
                kinds[k][1] += f.stat().st_size
            man[sub.name] = {"files": len(fs), "mb": round(sum(f.stat().st_size for f in fs) / 1e6, 1),
                             "by_type": {k: {"files": n, "mb": round(b / 1e6, 1)} for k, (n, b) in kinds.items()}}
        (dest / "manifest.json").write_text(json.dumps({"tag": self.tag, "label": self.spec.get("label"), "t": time.time(),
                                                        "total_gb": round(sum(v["mb"] for v in man.values()) / 1e3, 2),
                                                        "parts": man}, indent=1))
        self.log(f"archived to {dest}")
        self.note(f"archived everything as the reference ({self.spec.get('label', '')}) in {dest}")

    def derisk(self, step: dict) -> None:
        """Phase 0, while Clemens is awake: everything the night touches, once, quickly; then go/no-go
        in the Study feed and a pause until he confirms (10 min without problems: on)."""
        import cv2
        from hand_study import collision
        from hand_study.hw import Camera
        problems, done, warn = [], [], []
        d = Path(self.spec["protocol_dir"]) / "derisk"

        def retried(fn, what):
            """A motor fault in the derisk is recovered (that is part of the test) and the item retried once."""
            for k in (0, 1):
                try:
                    return fn()
                except MotorFault as e:
                    warn.append(f"{what}: {e} (recovered)")
                    recover(Hand(self.stream), e, f"derisk/{what}", self.log)
                    if k:
                        raise
        d.mkdir(parents=True, exist_ok=True)
        plate = Turntable()
        try:                                                        # the plate's full travel
            call("/visual-calibration/turntable/torque-off", {}, timeout=20)
            for a in (0.0, 315.0, 45.0):
                plate.move(a, self.log)
            plate.off()
            done.append("plate 0 -> 315 -> 45")
        except Exception as e:
            problems.append(f"plate: {e}")
        try:                                                        # the camera
            collect.lock_camera(self.log)
            img = Camera().grab()
            cv2.imwrite(str(d / "camera.jpg"), img)
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            h, w = g.shape
            sharp = float(cv2.Laplacian(g[h // 4: 3 * h // 4, w // 8: 5 * w // 8], cv2.CV_64F).var())
            done.append(f"camera {w}x{h}, sharpness {sharp:.0f}")
            if sharp < 20:
                problems.append(f"camera looks out of focus (sharpness {sharp:.0f})")
        except Exception as e:
            problems.append(f"camera: {e}")
        hand = Hand(self.stream)
        try:                                                        # extremes, wrist stops
            from hand_study.plan import Builder
            c = Builder().c
            rom = {j: ROM_OF(j) for j in collect_joints()}
            poses = {"open": {j: rom[j][0] + 5 for j in rom if j.endswith(("_mcp", "_pip", "_dip"))},
                     "fist": {j: rom[j][1] - 8 for j in rom if j.endswith(("_mcp", "_pip")) and not j.startswith("thumb")},
                     "spread": {j: (rom[j][0] + 5 if j.startswith(("index", "middle")) else rom[j][1] - 5) for j in rom if j.endswith("_abd") and not j.startswith("thumb")},
                     "wrist_lo": {"wrist": rom["wrist"][0] + 2}, "wrist_hi": {"wrist": rom["wrist"][1] - 2}}
            hand.torque(True)
            for name, q in poses.items():
                for f in (1.0, 0.8, 0.6):
                    qq = {j: v * f for j, v in q.items()}
                    if c.free(qq):
                        break
                else:
                    done.append(f"{name}: skipped (planner: not collision-free)")
                    continue

                def go(qq=qq):
                    if hand.target is None:
                        hand.torque(True)
                    hand.move(qq, 30.0)
                    time.sleep(1.0)
                    hand.guard()
                    hand.move({j: 0.0 for j in qq}, 30.0)
                retried(go, name)
            done.append(f"poses {', '.join(poses)}")
        except Exception as e:
            problems.append(f"hand poses: {e}")
        try:                                                        # one light contact
            prs = {p["id"]: p for p in json.loads(Path(self.spec["collision_pairs"]).read_text())}
            p = prs.get(step.get("pair")) or next(iter(prs.values()))
            from hand_study.mapping import LinearMap
            run = self.spec["runs"]["p"]
            L = LinearMap(STUDY / f"calib_{run}") if (STUDY / f"calib_{run}").exists() else None
            if L is None:
                done.append("contact: skipped (no calibration archived yet)")
            else:
                cap = collision.CAP_MA
                collision.CAP_MA = 150.0
                try:
                    def touch():
                        if hand.target is None:
                            hand.torque(True)
                        return collision.approach(hand, L, p, self.log)
                    r = retried(touch, "contact")
                finally:
                    collision.CAP_MA = cap
                (d / "contact.json").write_text(json.dumps({k: v for k, v in r.items()}, default=float))
                done.append(f"contact {p['id']}: {r['result']}" + (f" at {r['at']['est']:.1f}° (URDF {p['angle']:.1f}°)" if "at" in r else ""))
                if r["result"] == "unexpected":
                    problems.append(f"contact {p['id']} stalled outside its window: check the map before the collision phase")
        except Exception as e:
            problems.append(f"contact: {e}")
        try:                                                        # a fault on purpose: torque off behind the runner's back
            hand.torque(True)
            call("/torque/disable", {})
            try:
                hand.guard()
                problems.append("torque loss not detected by the guard")
            except hw.TorqueLost as e:
                self.ladder(e, step, 0)
                hand = Hand(self.stream)
                hand.torque(True)
                hand.move({}, 30.0)
                hand.guard()
                done.append("fault on purpose: torque loss detected, recovered, resumed")
        except Exception as e:
            problems.append(f"recovery test: {e}")
        finally:
            try:
                hand.torque(False)
            except Exception:
                pass
        try:                                                        # one pose end to end
            s = Path(step["session"])
            collect.run(s, log=self.log)
            t0 = time.time()
            e = s / "configs" / "zero" / "est.json"
            while not e.exists() and time.time() - t0 < 300:
                beat("derisk: waiting for the solve")
                time.sleep(5)
            if e.exists():
                est = json.loads(e.read_text())
                done.append(f"solve: {est['rms_px']:.1f} px in {est.get('solve_s')} s")
                if est["rms_px"] > 12:
                    problems.append(f"solve residual {est['rms_px']:.1f} px is high (rig/camera moved?)")
            else:
                problems.append("the derisk pose was not solved in 5 min (workers running?)")
        except Exception as e:
            problems.append(f"solve: {e}")
        try:
            import shutil as sh
            free = sh.disk_usage(Path.home()).free / 1e9
            proj = projected_gb(self.spec)
            done.append(f"disk {free:.0f} GB free, the night logs ~{proj:.1f} GB")
            if free < proj + hw.MIN_FREE_GB + 5:
                problems.append(f"disk: {free:.0f} GB free for ~{proj:.1f} GB of data")
        except Exception as e:
            problems.append(f"disk: {e}")
        verdict = "GO" if not problems else "NO-GO"
        (d / "result.json").write_text(json.dumps({"verdict": verdict, "done": done, "problems": problems, "recovered": warn}, indent=1))
        self.log(f"derisk {verdict}: {'; '.join(done)}" + (f" | problems: {'; '.join(problems)}" if problems else ""))
        if warn:
            done.append(f"recovered on the way: {'; '.join(warn)}")
        self.note(f"derisk {verdict}: " + ("; ".join(problems) if problems else "; ".join(done))
                  + (" -- Clemens, please reply 'go' in the feed (or fix and reply)" if problems else
                     " -- continuing in 10 min unless Clemens says 'stop'"))
        self.wait_for_go(bool(problems), limit=bool(step.get("limit")))

    def wait_for_go(self, problems: bool, limit: bool = False) -> None:
        """Clemens' reply in the feed: 'go'/'ok' goes on, 'stop'/'wait' holds; without problems 10 min of
        silence also goes on; with problems reminders every 10 min, then on after 45 min (the night never stops)."""
        import re
        import urllib.request
        url = hw.UI + f"/visual-calibration/study/notes/{self.tag}"
        t0, hold = time.time(), False
        last_remind = t0
        while True:
            if limit:
                return
            try:
                notes = json.load(urllib.request.urlopen(url, timeout=5))
            except Exception:
                notes = []
            mine = [n for n in notes if n.get("t", 0) > t0 and n.get("who") not in ("runner", "isha")]
            for n in mine:
                txt = n.get("text", "").lower()
                if re.search(r"\b(stop|wait|halt|hold)\b", txt):
                    hold = True
                if re.search(r"\b(go|ok|okay|confirm|confirmed|weiter|passt|continue)\b", txt):
                    self.log(f"derisk: {n.get('who')} says {n.get('text')!r}: going on")
                    return
            waited = time.time() - t0
            if not hold and not problems and waited > 600:
                self.note("derisk: no reply in 10 min and no problems: going on")
                return
            if waited > 2700 and not hold:
                self.note("derisk: no reply in 45 min: going on (the night never stops)")
                return
            if time.time() - last_remind > 600:
                last_remind = time.time()
                self.note("derisk: still waiting for a 'go' in the feed")
            beat("derisk: waiting for go")
            time.sleep(10)

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
        self.telemetry = hw.Telemetry(NIGHT / f"{self.tag}-telemetry") if self.spec.get("mode") == "protocol" else None
        self.buswatch = hw.BusWatch(self.log)
        try:
            if Turntable().state().get("busy"):
                raise Abort("the turntable is busy")
            for st in self.spec["steps"]:
                if st["name"] in self.state["done"]:
                    continue
                late = time.time() > self.deadline and not st["name"].startswith("N09")
                if late and self.spec.get("mode") == "protocol":
                    late = st.get("minutes") is None        # a protocol runs every phase; only the filler waits
                if late:
                    self.log(f"deadline reached before {st['name']}")
                    self.save(cut_short=st["name"])
                    break
                retries = int(self.spec.get("retries", 8)) if hw.RESILIENT else RETRIES
                failed = None
                for attempt in range(retries + 1):
                    if attempt == 0:
                        self.save(step=st["name"], step_t0=time.time())
                    if getattr(self, "mount_due", False) and st["kind"] == "collect":
                        try:
                            self.mount_check(st["run"])
                        except Exception as x:
                            self.log(f"mount check failed: {x!r}")
                            self.mount_due = False
                    self.log(f"-> {st['name']}" + (f" (attempt {attempt + 1})" if attempt else ""))
                    self.note(f"{'started' if not attempt else f'retrying (attempt {attempt + 1})'} {self.step_line(st)}")
                    t0 = time.time()
                    failed = None
                    try:
                        getattr(self, st["kind"])(st)
                        break
                    except Abort as e:
                        if not hw.RESILIENT or isinstance(e, hw.Safety):
                            raise
                        failed = e
                        self.state["errors"].append({"step": st["name"], "error": repr(e), "t": time.time()})
                        self.log(f"stop-worthy in a normal night, recovered here: {e!r}")
                        try:
                            self.ladder(e, st, attempt)
                        except hw.Safety:
                            raise
                        except Exception as x:
                            self.log(f"ladder failed: {x!r}")
                    except Wrapped as e:
                        failed = e
                        self.safe_off()
                        self.state["errors"].append({"step": st["name"], "error": repr(e), "t": time.time()})
                        self.rewrap(st)
                    except MotorFault as e:
                        failed = e
                        try:
                            recover(Hand(self.stream), e, st["name"], self.log)
                        except Wrapped as w:
                            self.safe_off()
                            self.state["errors"].append({"step": st["name"], "error": repr(w), "t": time.time()})
                            self.rewrap(st)
                        except hw.Safety:
                            raise
                        except Exception as x:
                            if not hw.RESILIENT:
                                raise
                            self.ladder(x, st, attempt)
                        self.state["errors"].append({"step": st["name"], "error": repr(e), "t": time.time()})
                    except Exception as e:
                        failed = e
                        self.safe_off()
                        self.state["errors"].append({"step": st["name"], "error": repr(e), "t": time.time()})
                        self.log(f"error in {st['name']}: {e!r}\n{traceback.format_exc()}")
                        if hw.RESILIENT:
                            try:
                                self.ladder(e, st, attempt)
                            except hw.Safety:
                                raise
                            except Exception as x:
                                self.log(f"ladder failed: {x!r}")
                        elif "turntable" in repr(e):
                            self.reconnect_plate()
                        wait = min(60 * 2 ** attempt, 600) if hw.RESILIENT else 60
                        for _ in range(int(wait / 10)):
                            beat("retry wait")
                            time.sleep(10)
                    finally:
                        self.safe_off()
                if failed is not None:
                    if not hw.RESILIENT:
                        raise Abort(f"{st['name']}: {failed!r} kept coming") from failed
                    self.log(f"{st['name']} FAILED after {retries + 1} attempts ({failed!r}); the night goes on")
                    self.note(f"{st['name']} FAILED after {retries + 1} attempts ({str(failed)[:120]}); the night goes on")
                else:
                    self.note(f"done {self.step_line(st)} in {(time.time() - t0) / 60:.0f} min")
                self.state["done"][st["name"]] = {"t0": t0, "t1": time.time(), **({"failed": repr(failed)} if failed else {})}
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
            if getattr(self, "telemetry", None):
                self.telemetry.close()
            if getattr(self, "buswatch", None):
                self.buswatch.close()
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
