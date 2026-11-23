"""The rig through orca-ui: hand (REST + its websocket), turntable, camera."""

from __future__ import annotations

import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

UI = os.environ.get("ORCA_UI", "http://127.0.0.1:5001") + "/api"
WS = UI.replace("http", "ws", 1)[:-4] + "/ws"
TT = "/visual-calibration/turntable"
RAMP_DEG_S = 30.0
RAMP_HZ = 20.0
MAX_TEMP_C = 55.0
PAUSE_TEMP_C = 60.0          # night guard: cool down (torque off) above this
ABORT_TEMP_C = 65.0          # abort the night (motors shut down at 70, wrist 80)
MIN_FREE_GB = 8.0
HEARTBEAT = None             # set by the night runner: a file touched on every step


class Abort(RuntimeError):
    """A safety stop: never retried (e-stop, overheating, disk full, a motor that does not recover)."""


class Safety(Abort):
    """The only stop of a resilient night: smoke-level heat after cooling, or the hand and plate moving together."""


class TorqueLost(RuntimeError):
    """The torque went off behind the runner's back: a resilient night recovers and resumes."""


class BusStuck(RuntimeError):
    """orca-ui's bus is stuck ("Port is in use", sync reads failing) for longer than BUS_STUCK_S:
    telemetry is stale. The ladder reconnects the hand, else restarts orca-ui."""


class Wrapped(RuntimeError):
    """A reboot moved a motor's reading by a whole turn (its position was past a turn boundary,
    so the Dynamixel lost its turn count): every command in that frame drives the joint into
    its stop. Not retried: the runner reconnects/recalibrates the hand first (night.py)."""

    def __init__(self, msg: str, ids: list[int]):
        super().__init__(msg)
        self.ids = ids


TURN = 2 * 3.141592653589793
WRAP_RAD = 3.0         # a reading that jumps this much across a reboot lost a turn


class MotorFault(RuntimeError):
    """A motor hardware error (overload, ...) or a joint stuck far from its target under load:
    recovered by a reboot (recover()), the pose retried once, then skipped."""

    def __init__(self, msg: str, motors: list[dict]):
        super().__init__(msg)
        self.motors = motors


class Hot(MotorFault):
    """A motor at ABORT_TEMP_C in a resilient night: torque off, cool to COOL_TO_C, the pose retried."""


TRACK_DEG = 12.0             # a joint this far from its target ...
TRACK_MA = float(os.environ.get("STUDY_TRACK_MA", 400.0))  # ... while its motor pulls this hard is stuck
MOTORS = os.environ.get("STUDY_MOTORS", "dynamixel")      # "feetech": no reboot, no turn to lose
RESILIENT = False            # night spec "resilient": cool and resume instead of aborting on heat
COOL_TO_C = 45.0
SAFETY_C = 70.0              # still this hot after cooling: the night's only thermal stop
DEAD: dict[str, dict] = {}   # joint -> {"id", "since", "tried"}: a motor the whole ladder could not bring back
DEAD_RETRY_S = 900.0
HAND_SETTLE_S = 0.6
HAND_MOVED = [0.0]           # the last hand target's time, and until when the plate may still be moving:
PLATE_MOVING = [0.0]         # the hand and the plate never move together (a breach is the other hard stop)
BUS_STUCK_S = 10.0
BUS_STUCK = [0.0]            # since when orca-ui's log has been reporting a stuck bus (0: healthy)
FAULT_WINDOW_S = 600.0
FAULT_LIMIT = 3              # more faults than this per motor in the window: back off


class FaultLog:
    """Every motor fault, with time, motor, pose, temperature and load (night/<tag>-faults.jsonl)."""

    def __init__(self, path: Path | None = None):
        self.path, self.events, self.banned = path, [], set()

    def add(self, ev: dict) -> None:
        self.events.append(ev)
        if self.path is not None:
            with self.path.open("a") as f:
                f.write(json.dumps(ev) + "\n")
        n = sum(1 for e in self.events if e["joint"] == ev["joint"] and ev["t"] - e["t"] <= FAULT_WINDOW_S)
        if n > FAULT_LIMIT:
            self.banned.add(ev["joint"])


FAULTS = FaultLog()


def recover(hand: "Hand", fault: MotorFault, where: str, log=print) -> None:
    """Torque off, reboot the faulted motors (Dynamixel reboot via orca-ui), check they are back."""
    for m in fault.motors:
        FAULTS.add({"t": time.time(), "where": where, "why": str(fault), **m})
    fault.motors = [m for m in fault.motors if m.get("joint") not in DEAD]
    if not fault.motors:
        return
    if isinstance(fault, Hot):
        log(f"HOT at {where}: {fault}; torque off, cooling to {COOL_TO_C:.0f} °C")
        hand.torque(False)
        t0 = time.time()
        while hand.hottest() > COOL_TO_C:
            if time.time() - t0 > 3600:
                if hand.hottest() >= SAFETY_C:
                    raise Safety(f"SAFETY: still {hand.hottest():.0f} °C after an hour of cooling")
                log(f"still {hand.hottest():.0f} °C after an hour of cooling: going on (below {SAFETY_C:.0f} °C)")
                break
            beat("cooling")
            time.sleep(10)
        log(f"cooled to {hand.hottest():.0f} °C after {time.time() - t0:.0f} s")
        return
    if MOTORS == "feetech":
        return recover_feetech(hand, fault, where, log)
    log(f"MOTOR FAULT at {where}: {fault}; torque off, reboot {[m['joint'] for m in fault.motors]}")
    try:
        hand.torque(False)
    except Exception as e:
        log(f"torque off failed: {e}")
    ids = sorted({m["id"] for m in fault.motors})
    before = {x["id"]: float(x["position"]) for x in hand.motors()["motors"]}
    past = [i for i in ids if not 0.0 <= before.get(i, 1.0) < TURN]
    if past:
        log(f"motors {past} sit past a turn boundary {[round(before[i], 3) for i in past]}: "
            "the reboot will lose the turn, recalibrating after it")
    for attempt in range(6 if RESILIENT else 3):
        if attempt == 3:
            log(f"motors {ids} still in error after 3 reboots: torque off, 2 min rest, 3 more")
            hand.torque(False)
            for _ in range(12):
                beat("resting before more reboots")
                time.sleep(10)
        for i in ids:
            try:
                call(f"/motors/{i}/reboot", {}, timeout=30)
            except Exception as e:
                log(f"reboot motor {i}: {e}")
        time.sleep(1.5)
        now = hand.motors()["motors"]
        wrapped = sorted({x["id"] for x in now if x["id"] in before and abs(float(x["position"]) - before[x["id"]]) > WRAP_RAD}
                         | set(past))
        if wrapped:
            hand.torque(False)
            for i in wrapped:
                FAULTS.add({"t": time.time(), "where": where, "why": "reboot lost a turn", "id": i,
                            "joint": next((x["joint"] for x in now if x["id"] == i), None),
                            "before": before.get(i), "after": next((x["position"] for x in now if x["id"] == i), None)})
            log(f"WRAP: motors {wrapped} lost a turn in the reboot ({ {i: round(before[i], 3) for i in wrapped} } -> "
                f"{ {x['id']: round(float(x['position']), 3) for x in now if x['id'] in wrapped} })")
            raise Wrapped(f"motors {wrapped} lost a turn in the reboot", wrapped)
        left = [x["id"] for x in now if x["hw_error"]]
        if not left:
            log(f"motors {ids} back (attempt {attempt + 1})")
            beat("recovered")
            return
        ids = left
    if RESILIENT:
        joints = {x["id"]: x["joint"] for x in now}
        for i in ids:
            DEAD[joints.get(i) or str(i)] = {"id": i, "since": time.time(), "tried": time.time()}
        log(f"DEAD: motors {ids} ({[joints.get(i) for i in ids]}) still in error after 6 reboots: skipping the poses "
            f"that need them, retrying every {DEAD_RETRY_S / 60:.0f} min")
        return
    raise Abort(f"motors {ids} still in error after 3 reboots")


def retry_dead(hand: "Hand", log=print) -> None:
    """Every DEAD_RETRY_S: torque off, reboot the dead motors, back in if their error cleared."""
    due = [j for j, d in DEAD.items() if time.time() - d["tried"] > DEAD_RETRY_S]
    if not due:
        return
    held = hand.target is not None
    hand.torque(False)
    for j in due:
        DEAD[j]["tried"] = time.time()
        try:
            call(f"/motors/{DEAD[j]['id']}/reboot", {}, timeout=30)
        except Exception as e:
            log(f"dead motor {j}: reboot failed: {e}")
    time.sleep(1.5)
    m = {x["joint"]: x for x in hand.motors()["motors"]}
    for j in due:
        if j in m and not m[j]["hw_error"]:
            d = DEAD.pop(j)
            log(f"motors {j} back after {time.time() - d['since']:.0f} s dead")
        else:
            log(f"dead motor {j} still in error, next try in {DEAD_RETRY_S / 60:.0f} min")
    if held:
        hand.torque(True)


def recover_feetech(hand: "Hand", fault: MotorFault, where: str, log=print) -> None:
    """Feetech: no reboot. Torque off clears an overload; wait until the motors report no error
    and have not jumped (a jump means the reading is wrong: recalibrate, as for a lost turn)."""
    log(f"MOTOR FAULT at {where}: {fault}; torque off, no reboot (Feetech) {[m['joint'] for m in fault.motors]}")
    try:
        hand.torque(False)
    except Exception as e:
        log(f"torque off failed: {e}")
    ids = sorted({m["id"] for m in fault.motors})
    before = {m["id"]: float(m["position"]) for m in fault.motors}
    for attempt in range(6):
        time.sleep(2.0 + 3.0 * attempt)
        now = hand.motors()["motors"]
        jumped = [x["id"] for x in now if x["id"] in before and abs(float(x["position"]) - before[x["id"]]) > WRAP_RAD]
        if jumped:
            raise Wrapped(f"motors {jumped} jumped by more than {WRAP_RAD} rad with the torque off", jumped)
        left = [x["id"] for x in now if x["hw_error"]]
        if not left:
            log(f"motors {ids} clear after {attempt + 1} checks")
            beat("recovered")
            return
        beat("waiting for the motor error to clear")
    if RESILIENT:
        for x in hand.motors()["motors"]:
            if x["id"] in left:
                DEAD[x["joint"]] = {"id": x["id"], "since": time.time(), "tried": time.time()}
        log(f"DEAD: motors {left} still in error with the torque off: skipping the poses that need them")
        return
    raise Abort(f"motors {left} still in error with the torque off")


def beat(note: str = "") -> None:
    if HEARTBEAT is not None:
        tmp = HEARTBEAT.with_suffix(".tmp")
        tmp.write_text(json.dumps({"t": time.time(), "note": note}))
        tmp.replace(HEARTBEAT)


class ApiError(RuntimeError):
    pass


def call(path: str, body: dict | None = None, timeout: float = 120.0):
    req = urllib.request.Request(UI + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"},
                                 method="GET" if body is None else "POST")
    try:
        return json.load(urllib.request.urlopen(req, timeout=timeout))
    except urllib.error.HTTPError as e:
        msg = f"{path}: {e.code} {e.read().decode()}"
        if path == "/joints/target" and e.code == 409 and not RESILIENT:
            raise Abort(msg) from None          # torque gone or control taken: stop, never retry
        raise ApiError(msg) from None


class Telemetry:
    """The whole night at full rate, for reuse: every websocket message of orca-ui's motor telemetry,
    joint estimate and target, status and control state (no extra bus traffic), and the turntable's
    state at 2 Hz; gzip JSON lines, one file per hour (night/<tag>-telemetry/<YYYYmmdd-HH>.jsonl.gz)."""

    TOPICS = ["motors.telemetry", "joints.estimate", "joints.target", "status", "control.state"]

    def __init__(self, out: Path):
        self.out = out
        out.mkdir(parents=True, exist_ok=True)
        self.buf: list[str] = []
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.n = 0
        for f in (self._ws, self._plate, self._flush):
            threading.Thread(target=f, daemon=True).start()

    def _add(self, kind: str, data) -> None:
        with self.lock:
            self.buf.append(json.dumps({"t": round(time.time(), 4), "k": kind, "d": data}, separators=(",", ":")))
            self.n += 1

    def _ws(self):
        from websockets.sync.client import connect
        while not self._stop.is_set():
            try:
                with connect(WS, max_size=None) as ws:
                    ws.send(json.dumps({"type": "subscribe", "data": {"topics": self.TOPICS}}))
                    while not self._stop.is_set():
                        try:
                            m = json.loads(ws.recv(timeout=1.0))
                        except TimeoutError:
                            continue
                        self._add(m.get("type"), m.get("data"))
            except Exception:
                time.sleep(1.0)

    def _plate(self):
        while not self._stop.is_set():
            try:
                self._add("turntable", call(TT, timeout=2))
            except Exception:
                pass
            time.sleep(0.5)

    def _flush(self):
        import gzip
        while not self._stop.wait(30):
            self.flush(gzip)

    def flush(self, gzip=None) -> None:
        import gzip as gz
        with self.lock:
            rows, self.buf = self.buf, []
        if rows:
            with gz.open(self.out / f"{time.strftime('%Y%m%d-%H')}.jsonl.gz", "at") as f:
                f.write("\n".join(rows) + "\n")

    def close(self) -> None:
        self._stop.set()
        self.flush()


class BusWatch:
    """Tails the log of the orca-ui listening on hw.UI's port (its stdout, found again after a
    restart) and sets BUS_STUCK while bus errors keep coming: a line of BAD within the last 3 s."""

    BAD = ("Port is in use", "Sync read failed", "Device not configured", "Port handler in use")

    def __init__(self, log=print):
        self.log = log
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def _logfile() -> str | None:
        import subprocess
        port = UI.rsplit(":", 1)[1].split("/")[0]
        pid = subprocess.run(["lsof", "-ti", f"tcp:{port}", "-sTCP:LISTEN"], capture_output=True, text=True).stdout.split()
        if not pid:
            return None
        out = subprocess.run(["lsof", "-a", "-p", pid[0], "-d", "1", "-Fn"], capture_output=True, text=True).stdout.split("\n")
        return next((l[1:] for l in out if l.startswith("n/")), None)

    def _run(self):
        path, f, last_bad, t_path = None, None, 0.0, 0.0
        while not self._stop.is_set():
            now = time.time()
            if now - t_path > 15:
                t_path = now
                try:
                    p = self._logfile()
                except Exception:
                    p = None
                if p and p != path:
                    if f:
                        f.close()
                    path, f = p, open(p, errors="replace")
                    f.seek(0, 2)
            hit = False
            if f:
                for line in f.readlines():
                    if any(b in line for b in self.BAD):
                        hit = True
            if hit:
                last_bad = now
                if not BUS_STUCK[0]:
                    BUS_STUCK[0] = now
            elif BUS_STUCK[0] and now - last_bad > 3:
                self.log(f"bus healthy again after {now - BUS_STUCK[0]:.0f} s of errors")
                BUS_STUCK[0] = 0.0
            time.sleep(0.5)

    def stuck(self) -> bool:
        return bool(BUS_STUCK[0]) and time.time() - BUS_STUCK[0] > BUS_STUCK_S

    def close(self):
        self._stop.set()


class Stream:
    """The latest message of each websocket topic."""

    TOPICS = ["joints.estimate", "joints.target", "motors.telemetry", "status", "control.state", "tactile.forces"]

    def __init__(self):
        self.latest: dict[str, dict] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        t0 = time.time()
        while "joints.estimate" not in self.latest and time.time() - t0 < 5:
            time.sleep(0.05)

    def _run(self):
        from websockets.sync.client import connect
        while not self._stop.is_set():
            try:
                with connect(WS, max_size=None) as ws:
                    ws.send(json.dumps({"type": "subscribe", "data": {"topics": self.TOPICS}}))
                    while not self._stop.is_set():
                        try:
                            m = json.loads(ws.recv(timeout=1.0))
                        except TimeoutError:
                            continue
                        self.latest[m.get("type")] = {**m.get("data", {}), "_t": time.time()}
            except Exception:
                time.sleep(0.5)

    def get(self, topic: str) -> dict:
        return self.latest.get(topic, {})

    def close(self):
        self._stop.set()


class Hand:
    def __init__(self, stream: Stream):
        self.s = stream
        self.target: dict[str, float] | None = None

    def status(self) -> dict:
        return call("/status")

    def torque(self, on: bool) -> None:
        if on:
            seed = call("/torque/enable", {})["seed"]
            self.target = {j: float(v) for j, v in seed.items()}
        else:
            call("/torque/disable", {})
            self.target = None

    def estimate(self) -> dict[str, float]:
        return dict(self.s.get("joints.estimate").get("angles", {}))

    def motors(self) -> dict:
        return call("/motors/direct")

    def guard(self) -> None:
        """Abort when the torque went off behind our back (e-stop), a motor reports an error,
        or a motor is too hot; the disk is checked too."""
        import shutil
        if DEAD:
            retry_dead(self)
        if BUS_STUCK[0] and time.time() - BUS_STUCK[0] > BUS_STUCK_S:
            raise BusStuck(f"orca-ui bus stuck for {time.time() - BUS_STUCK[0]:.0f} s (Port is in use / sync read failed)")
        m = self.motors()["motors"]
        bad = [x for x in m if x["hw_error"] and x["joint"] not in DEAD]
        if bad:
            raise MotorFault(f"hardware error {[(x['joint'], x['hw_error'], x.get('hw_error_flags')) for x in bad]}",
                             [self._fault_info(x) for x in bad])
        st = self.status()
        if self.target is not None and not st.get("torque_enabled"):
            if RESILIENT:
                raise TorqueLost("torque went off behind the runner's back")
            raise Abort("torque went off (e-stop?)")
        hot = max([x["temp_c"] for x in m if x["joint"] not in DEAD] or [0.0])
        if hot >= ABORT_TEMP_C:
            if RESILIENT:
                raise Hot(f"motor at {hot:.0f} °C", [self._fault_info(x) for x in m if x["temp_c"] >= ABORT_TEMP_C])
            raise Abort(f"motor at {hot:.0f} °C")
        if shutil.disk_usage(Path.home()).free / 1e9 < MIN_FREE_GB:
            raise Abort("disk almost full")
        beat("guard")

    @staticmethod
    def _fault_info(x: dict) -> dict:
        return {"id": x["id"], "joint": x["joint"], "hw_error": x["hw_error"], "flags": x.get("hw_error_flags"),
                "temp_c": x["temp_c"], "current_ma": x["current_ma"], "position": x["position"]}

    def check_tracking(self, sample: dict, assumed: dict[str, float], skip: set[str] = frozenset()) -> None:
        """A joint far from its target while its motor pulls hard: stuck (a jam), not just slow."""
        if self.target is None:
            return
        stuck = []
        for k, j in enumerate(sample["motor_joint"]):
            if j in skip or j not in self.target or j not in assumed:
                continue
            err = abs(assumed[j] - self.target[j])
            if err > TRACK_DEG and (abs(sample["current_ma"][k]) > TRACK_MA or err > 2 * TRACK_DEG):
                stuck.append({"id": sample["motor_ids"][k], "joint": j, "hw_error": 0, "flags": ["stuck"],
                              "temp_c": sample["temp_c"][k], "current_ma": sample["current_ma"][k],
                              "position": sample["motor_pos"][k], "target_deg": self.target[j],
                              "assumed_deg": round(assumed[j], 1)})
        if stuck:
            raise MotorFault(f"stuck {[(s['joint'], s['assumed_deg'], s['target_deg']) for s in stuck]}", stuck)

    def hottest(self) -> float:
        return max(x["temp_c"] for x in self.motors()["motors"])

    def cool_if_hot(self, log=print, cool_to: float = 45.0, max_wait_s: float = 1800) -> None:
        """Above PAUSE_TEMP_C: torque off until every motor is below `cool_to`, then on again."""
        hot = self.hottest()
        if hot < PAUSE_TEMP_C:
            return
        log(f"cooling: {hot:.0f} °C, torque off")
        held = self.target is not None
        self.torque(False)
        t0 = time.time()
        while self.hottest() > cool_to:
            if time.time() - t0 > max_wait_s:
                if not RESILIENT or self.hottest() >= SAFETY_C:
                    raise Safety(f"SAFETY: still {self.hottest():.0f} °C after {max_wait_s / 60:.0f} min of cooling")
                log(f"still {self.hottest():.0f} °C after {max_wait_s / 60:.0f} min of cooling: going on")
                break
            beat("cooling")
            time.sleep(10)
        log(f"cooled to {self.hottest():.0f} °C after {time.time() - t0:.0f} s")
        if held:
            self.torque(True)

    def move(self, goal: dict[str, float], speed: float = RAMP_DEG_S) -> None:
        """Ramp every joint linearly to the goal (deg) at `speed`, streaming targets."""
        if self.target is None:
            raise RuntimeError("torque is off")
        start = dict(self.target)
        goal = {**start, **goal}
        span = max(abs(goal[j] - start[j]) for j in goal)
        n = max(1, int(np.ceil(span / speed * RAMP_HZ)))
        for k in range(1, n + 1):
            f = k / n
            if PLATE_MOVING[0] > time.time():
                raise Safety("SAFETY: hand target while the plate moves")
            HAND_MOVED[0] = time.time()
            call("/joints/target", {"angles": {j: start[j] + f * (goal[j] - start[j]) for j in goal if j not in DEAD}})
            time.sleep(1.0 / RAMP_HZ)
        self.target = goal

    def sample(self, n: int = 5, period: float = 0.1) -> dict:
        """Averaged motor positions/currents and the orca_core joint estimate, at rest."""
        pos, cur, est, temps, errs, tac = [], [], [], [], [], []
        for _ in range(n):
            f = self.s.get("tactile.forces").get("forces")
            if f:
                tac.append(f)
            m = self.motors()["motors"]
            pos.append([x["position"] for x in m])
            cur.append([x["current_ma"] for x in m])
            temps.append([x["temp_c"] for x in m])
            errs.append([x["hw_error"] for x in m])
            est.append(self.estimate())
            time.sleep(period)
        ids = [x["id"] for x in m]
        joints = [x["joint"] for x in m]
        pos, cur = np.array(pos), np.array(cur)
        return {
            "motor_ids": ids, "motor_joint": joints,
            "motor_pos": pos.mean(0).tolist(), "motor_pos_std": pos.std(0).tolist(),
            "current_ma": cur.mean(0).tolist(), "temp_c": np.max(temps, 0).tolist(),
            "hw_error": np.max(errs, 0).tolist(),
            "estimate": {j: float(np.mean([e[j] for e in est if e.get(j) is not None] or [np.nan])) for j in est[-1]},
            "target": dict(self.target or {}),
            "tactile": {k: np.mean([t[k] for t in tac if k in t], 0).round(4).tolist() for k in tac[-1]} if tac else None,
        }


class Turntable:
    def state(self) -> dict:
        return call(TT)

    STEP_DEG = 15.0
    RETRY_STEPS = (5.0, 2.5)
    MISS_DEG = 1.5            # the photo keeps the measured angle; only a real miss matters

    def _go(self, angle: float, step: float) -> None:
        cur = float(self.state()["angle"])
        n = max(1, int(np.ceil(abs(angle - cur) / step - 1e-6)))
        for k in range(1, n + 1):
            wait = HAND_SETTLE_S - (time.time() - HAND_MOVED[0])
            if wait > 0:                       # the hand's last target is fresh: let it settle first
                time.sleep(wait)
            PLATE_MOVING[0] = time.time() + 30.0
            try:
                call(TT + "/move", {"angle": round(cur + (angle - cur) * k / n, 3)})
            finally:
                PLATE_MOVING[0] = 0.0
        got = float(self.state()["angle"])
        if abs(got - angle) > self.MISS_DEG:
            raise ApiError(f"turntable missed the stop: at {got:.2f}, wanted {angle:.2f}")

    def move(self, angle: float, log=print) -> dict:
        """In <= STEP_DEG moves (the heavier hand's plate stalled on a long one); a stall, lag or
        missed stop: torque off, then again in smaller steps; still failing there: an error."""
        beat(f"plate {angle:.0f}")
        for step in (self.STEP_DEG, *self.RETRY_STEPS):
            try:
                self._go(float(angle), step)
                return self.state()
            except ApiError as e:
                at = self.state().get("angle")
                log(f"turntable to {angle:g} (steps {step:g}): {e}; at {at}, torque off, retrying smaller")
                self.off()
                time.sleep(2.0)
                last = e
        raise ApiError(f"turntable kept failing near {at} on the way to {angle:g}: {last}")

    def off(self) -> None:
        try:
            call(TT + "/torque-off", {})
        except Exception:
            pass


class Camera:
    def __init__(self, size=(3840, 2160)):
        self.fake = os.environ.get("STUDY_FAKE_CAMERA")          # a dry run: this photo, no camera
        if self.fake:
            import cv2
            self.img = cv2.imread(self.fake)
            return
        from visual_calibration.sweep import Webcam
        self.cam = Webcam(0, size)

    def grab(self) -> np.ndarray:
        return self.img.copy() if self.fake else self.cam.grab()

    def close(self):
        """Kept open: one capture for the runner's whole life (a reopen per block is what the
        decoder must not see); released at exit (capture.SharedCamera)."""
