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


class MotorFault(RuntimeError):
    """A motor hardware error (overload, ...) or a joint stuck far from its target under load:
    recovered by a reboot (recover()), the pose retried once, then skipped."""

    def __init__(self, msg: str, motors: list[dict]):
        super().__init__(msg)
        self.motors = motors


TRACK_DEG = 12.0             # a joint this far from its target ...
TRACK_MA = 400.0             # ... while its motor pulls this hard is stuck
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
    log(f"MOTOR FAULT at {where}: {fault}; torque off, reboot {[m['joint'] for m in fault.motors]}")
    try:
        hand.torque(False)
    except Exception as e:
        log(f"torque off failed: {e}")
    ids = sorted({m["id"] for m in fault.motors})
    for attempt in range(3):
        for i in ids:
            try:
                call(f"/motors/{i}/reboot", {}, timeout=30)
            except Exception as e:
                log(f"reboot motor {i}: {e}")
        time.sleep(1.5)
        left = [x["id"] for x in hand.motors()["motors"] if x["hw_error"]]
        if not left:
            log(f"motors {ids} back (attempt {attempt + 1})")
            beat("recovered")
            return
        ids = left
    raise Abort(f"motors {ids} still in error after 3 reboots")


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
        if path == "/joints/target" and e.code == 409:
            raise Abort(msg) from None          # torque gone or control taken: stop, never retry
        raise ApiError(msg) from None


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
        m = self.motors()["motors"]
        bad = [x for x in m if x["hw_error"]]
        if bad:
            raise MotorFault(f"hardware error {[(x['joint'], x['hw_error'], x.get('hw_error_flags')) for x in bad]}",
                             [self._fault_info(x) for x in bad])
        st = self.status()
        if self.target is not None and not st.get("torque_enabled"):
            raise Abort("torque went off (e-stop?)")
        hot = max(x["temp_c"] for x in m)
        if hot >= ABORT_TEMP_C:
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
                raise Abort(f"still {self.hottest():.0f} °C after {max_wait_s / 60:.0f} min of cooling")
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
            call("/joints/target", {"angles": {j: start[j] + f * (goal[j] - start[j]) for j in goal}})
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

    def move(self, angle: float) -> dict:
        beat(f"plate {angle:.0f}")
        call(TT + "/move", {"angle": float(angle)})
        return self.state()

    def off(self) -> None:
        try:
            call(TT + "/torque-off", {})
        except Exception:
            pass


class Camera:
    def __init__(self, size=(3840, 2160)):
        from visual_calibration.sweep import Webcam
        self.cam = Webcam(0, size)

    def grab(self) -> np.ndarray:
        return self.cam.grab()

    def close(self):
        self.cam.cap.release()
