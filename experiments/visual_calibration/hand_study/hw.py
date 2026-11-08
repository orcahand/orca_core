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
    """A safety stop: never retried (e-stop, overheating, motor error, disk full)."""


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
        st = self.status()
        if self.target is not None and not st.get("torque_enabled"):
            raise Abort("torque went off (e-stop?)")
        m = self.motors()["motors"]
        bad = [(x["joint"], x["hw_error"], x.get("hw_error_flags")) for x in m if x["hw_error"]]
        if bad:
            raise Abort(f"motor hardware error: {bad}")
        hot = max(x["temp_c"] for x in m)
        if hot >= ABORT_TEMP_C:
            raise Abort(f"motor at {hot:.0f} °C")
        if shutil.disk_usage(Path.home()).free / 1e9 < MIN_FREE_GB:
            raise Abort("disk almost full")
        beat("guard")

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
