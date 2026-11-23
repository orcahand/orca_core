"""A stand-in for orca-ui for dry runs of the night runner: the REST calls and websocket topics
hand_study uses, a simulated hand (motor positions follow the targets through the bench's linear
map) and turntable, orca_core's calibration as a copy of the packaged calibration.yaml, and
injected faults (a motor error, overheating, a turntable miss) to exercise the recovery.
Nothing touches hardware.

    python -m hand_study.mock_ui --port 5091 --bench DIR --calibration PACKAGED_CALIBRATION.yaml \\
        [--faults error@40,hot@90,plate@25] [--hand ser-7841]
"""

from __future__ import annotations

import argparse
import asyncio
import json
import shutil
import threading
import time
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Request, WebSocket

from hand_study.mapping import LinearMap

app = FastAPI()
S: dict = {}
LOCK = threading.Lock()


def lm() -> LinearMap | None:
    b = S["bench"]
    return LinearMap(b) if (b / "calibration.yaml").exists() else None


def estimate() -> dict:
    L = lm()
    if L is None:
        return {}
    ids = sorted(S["pos"])
    return L.joints_of(ids, [S["pos"][i] for i in ids])


def fault(kind: str) -> bool:
    """True once when the counter of `kind` reaches its injection point."""
    n = S["count"][kind] = S["count"].get(kind, 0) + 1
    return n in S["faults"].get(kind, ())


@app.get("/api/status")
def status():
    return {"state": "connected", "torque_enabled": S["torque"], "leaving_maintenance": False, "model": "orcahand-right",
            "board_pinned": "mock", "message": "mock"}


@app.get("/api/visual-calibration/study-map")
def study_map():
    return {"hand": S["hand"], "mode": "off", "options": []}


@app.post("/api/torque/enable")
def torque_on():
    S["torque"] = True
    return {"seed": estimate()}


@app.post("/api/torque/disable")
def torque_off():
    S["torque"] = False
    return {}


@app.post("/api/joints/target")
async def target(req: Request):
    body = await req.json()
    if not S["torque"]:
        raise HTTPException(409, "torque off")
    L = lm()
    if time.time() - S["plate_t"] < 0.3:
        raise HTTPException(409, "the plate is moving (mock check)")
    S["target_t"] = time.time()
    with LOCK:
        for j, q in body["angles"].items():
            m = L.motor[j]
            q = float(q)
            S["cur"].pop(m, None)
            if m in S["caps"]:                     # a capped motor stops at the URDF contact (2° early)
                for p in S["pairs"].get(j, []):
                    c = p["angle"] - 2.0 * p["sign"]
                    if (q - c) * p["sign"] > 0 and abs(q - p["angle"]) < 20:
                        q, S["cur"][m] = c, 0.9 * S["caps"][m]
                        break
            S["pos"][m] = L.to_motor(j, q) + np.random.normal(0, 0.002)
        if fault("error"):
            S["err"][7] = 32
        if fault("hot"):
            S["temp"][9] = 66.0
    return {}


@app.get("/api/motors/direct")
def motors():
    L = lm()
    joint = {v: k for k, v in L.motor.items()} if L else {}
    for i in S["temp"]:                      # cooling with the torque off
        if not S["torque"] and S["temp"][i] > 35:
            S["temp"][i] = max(35.0, S["temp"][i] - 3.0)
    return {"motors": [{"id": i, "joint": joint.get(i, ""), "position": S["pos"][i],
                        "current_ma": S["cur"].get(i, 15.0 + 10 * np.random.rand()),
                        "temp_c": S["temp"][i], "hw_error": S["err"][i], "hw_error_flags": ["overload"] if S["err"][i] else []}
                       for i in sorted(S["pos"])]}


@app.post("/api/motors/{mid}/reboot")
def reboot(mid: int):
    S["err"][mid] = 0
    return {}


@app.post("/api/operation/calibrate/start")
async def calibrate(req: Request):
    body = await req.json()
    S["op"] = {"state": "running", "phase": "sweep", "t0": time.time(), "params": body.get("params")}
    S["calibrations"].append(body.get("params"))
    return S["op"]


@app.get("/api/operation")
def operation():
    op = S.get("op")
    if op and op["state"] == "running" and time.time() - op["t0"] > 2:
        if not (S["bench"] / "calibration.yaml").exists():
            shutil.copy2(S["packaged_cal"], S["bench"] / "calibration.yaml")
        op["state"] = "done"
    return {"operation": op}


@app.get("/api/operation/log")
def oplog():
    return {"log": ["mock calibration"], "params": S["calibrations"]}


@app.post("/api/control/motor_currents")
async def motor_currents(req: Request):
    body = await req.json()
    S["caps"] = {int(k): float(v) for k, v in (body.get("ma") or {}).items()}
    if not S["caps"]:
        S["cur"].clear()
    S["cap_calls"] += 1
    return {"ok": True, "currents": S["caps"]}


@app.get("/api/boards")
def boards():
    return {"boards": [{"device": "/dev/mock1101", "hand_id": S["hand"], "role": "hand"}]}


@app.post("/api/reconnect")
def reconnect():
    return {"ok": True, "status": status()}


@app.post("/api/board/select")
def board_select():
    return {"ok": True}


@app.get("/api/visual-calibration/study/notes/{tag}")
def notes_get(tag: str):
    return S["notes"].get(tag, [])


@app.post("/api/visual-calibration/study/notes/{tag}")
async def notes_post(tag: str, req: Request):
    b = await req.json()
    S["notes"].setdefault(tag, []).append({"t": time.time(), **b})
    return {"ok": True}


@app.get("/api/mock")
def mock_state():
    return {k: S[k] for k in ("calibrations", "plate_moves", "violations", "count", "torque", "angle", "cap_calls")} | \
        {"notes": {k: len(v) for k, v in S["notes"].items()}}


@app.post("/api/operation/stop")
def opstop():
    S["op"] = None
    return {}


@app.get("/api/visual-calibration/turntable")
def tt():
    return {"connected": True, "angle": S["angle"], "busy": False, "torque": S["tt_torque"]}


@app.post("/api/visual-calibration/turntable/move")
async def tt_move(req: Request):
    body = await req.json()
    if time.time() - S["target_t"] < 0.5:
        S["violations"] += 1
        raise HTTPException(409, "the hand moved 0.5 s ago: the plate never moves with it (mock check)")
    if fault("plate"):
        S["stuck"] = 3.0                     # a stall: off by 3° until the torque goes off
    S["angle"] = float(body["angle"]) + S.get("stuck", 0.0)
    S["tt_torque"] = True
    S["plate_moves"] += 1
    S["plate_t"] = time.time()
    return tt()


@app.post("/api/visual-calibration/turntable/{what}")
def tt_other(what: str):
    if what == "home":
        S["angle"] = 0.0
    if what in ("torque-off", "release"):
        S["tt_torque"], S["stuck"] = False, 0.0
    return tt()


@app.api_route("/api/visual-calibration/camera/uvc", methods=["GET", "POST"])
def uvc():
    return {"set": "mock"}


@app.post("/api/visual-calibration/cameras/preflight")
def cameras_preflight():
    return {"indices": {"a": 0, "b": 1}, "identified_by": "mock"}


@app.get("/api/visual-calibration/cameras/{cam}/still")
def camera_still(cam: str, format: str = "png", quality: int = 95):
    import cv2
    from fastapi.responses import Response
    if cam not in ("a", "b"):
        raise HTTPException(404, f"no camera {cam}")
    img = np.full((216, 384, 3), 90 if cam == "a" else 130, np.uint8)
    ok, buf = cv2.imencode(".jpg" if format == "jpg" else ".png", img)
    return Response(buf.tobytes(), media_type="image/jpeg" if format == "jpg" else "image/png")


@app.get("/api/visual-calibration/cameras/stills")
def camera_stills(format: str = "png", quality: int = 95):
    import base64
    return {"format": format, "cameras": {c: base64.b64encode(camera_still(c, format, quality).body).decode()
                                          for c in ("a", "b")}}


@app.post("/api/tactile/zero")
def tzero():
    return {}


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        await sock.receive_text()
        while True:
            await sock.send_text(json.dumps({"type": "joints.estimate", "data": {"angles": estimate()}}))
            await asyncio.sleep(0.1)
    except Exception:
        pass


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=5091)
    p.add_argument("--bench", type=Path, required=True)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--hand", default="ser-7841")
    p.add_argument("--faults", default="", help="kind@n,...: error (motor 7), hot (motor 9), plate (a 3° miss)")
    a = p.parse_args()
    faults: dict = {}
    for f in filter(None, a.faults.split(",")):
        k, n = f.split("@")
        faults.setdefault(k, set()).add(int(n))
    cal = LinearMap.__new__(LinearMap)
    import yaml
    limits = yaml.safe_load(a.calibration.read_text())["motor_limits"]
    S.update(bench=a.bench, packaged_cal=a.calibration, hand=a.hand, torque=False, angle=0.0, tt_torque=False,
             pos={int(k): float(v[0]) + 0.3 * (float(v[1]) - float(v[0])) for k, v in limits.items()},
             temp={int(k): 35.0 for k in limits}, err={int(k): 0 for k in limits}, faults=faults, count={},
             calibrations=[], plate_moves=0, violations=0, target_t=0.0, plate_t=0.0, caps={}, cur={}, cap_calls=0,
             notes={}, pairs={})
    pf = Path.home() / ".orca_ui/visual_calibration/hand_study/collision_pairs.json"
    if pf.exists():
        for p in json.loads(pf.read_text()):
            S["pairs"].setdefault(p["joint"], []).append(p)
    del cal
    uvicorn.run(app, host="127.0.0.1", port=a.port, log_level="warning")


if __name__ == "__main__":
    main()
