"""Run a plan of hand configurations: the hand moves (plate still), then the plate visits each
stop (hand still) and every camera takes a photo (through orca-ui, which owns the cameras). Torque: the plate is released after every
move; the hand holds during one configuration's photos and is released at the end.

    python -m hand_study.collect SESSION_DIR [--limit N]
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import shutil

import numpy as np

from hand_study import hw
from hand_study.hw import Camera, Hand, MotorFault, Stream, Turntable, beat, call, recover
from hand_study.mapping import LinearMap
from hand_study.paths import STUDY


SETTLE_S = 1.0
HOLD_STUCK = os.environ.get("STUDY_HOLD_STUCK", "0") == "1"
PLATE_SETTLE_S = 0.6


def write_json(path: Path, data) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=1))
    os.replace(tmp, path)


def lock_camera(log=print) -> None:
    """The camera's controls (exposure, white balance, focus) back to the rig config's locked values."""
    try:
        r = call("/visual-calibration/camera/uvc", {})
        log(f"camera controls locked: {r.get('set') or 'unchanged'}")
    except Exception as e:
        log(f"camera controls: {e}")


def photograph(cam, out: Path, stop: float, angle: float) -> list[dict]:
    """Every camera's photo at this stop, one view each: v<stop>.jpg for camera "a",
    v<stop>_<camera>.jpg (view `camera`) for the others (estimate.load_frames)."""
    views = []
    for c, data in cam.shots().items():
        name = f"v{int(round(stop)):03d}" + ("" if c == "a" else f"_{c}") + ".jpg"
        (out / name).write_bytes(data)
        views.append({"stop": stop, "angle": angle, "file": name, "t": time.time(), **({"camera": c} if c != "a" else {})})
    return views


def run(session: Path, limit: int | None = None, log=print) -> None:
    plan = json.loads((session / "plan.json").read_text())
    angles = plan["angles"]
    todo = [c for c in plan["configs"] if not (session / "configs" / c["id"] / "meta.json").exists()
            and not (session / "configs" / c["id"] / "skipped.json").exists()]
    todo = todo[:limit] if limit else todo
    progress = {"phase": "collect", "total": len(plan["configs"]), "todo": len(todo), "done_now": 0,
                "started": time.time(), "state": "running", "current": None, "error": None}
    write_json(session / "progress.json", progress)
    stream = Stream()
    hand, plate = Hand(stream), Turntable()
    cam = None
    forward = True
    try:
        cam = Camera()                         # orca-ui's camera preflight: refuses before anything moves
        log(f"cameras {', '.join(cam.names)}: identified, controls locked")
        if plate.state().get("busy"):
            raise RuntimeError("the turntable is busy")
        hand.torque(True)
        L = LinearMap(STUDY / f"calib_{plan['calibration']}") if (STUDY / f"calib_{plan['calibration']}").exists() else None
        contact = plan.get("tactile") or plan.get("part") in ("kapandji", "sac")

        def near_stop(c: dict) -> bool:
            from hand_study.configs import ROM
            return any(abs(c["q"].get(j, 0.0) - e) < 5.0 for j in hw.FAULTS.banned for e in ROM.get(j, ()))

        def shoot(c: dict, out: Path) -> dict:
            nonlocal forward
            t0 = time.time()
            if hand.target is None:
                hand.torque(True)
            hand.guard()
            for via in c.get("via") or []:
                hand.move(via)
            hand.move(c["q"], c.get("speed", 30.0))
            time.sleep(SETTLE_S)
            before = hand.sample()
            if L is not None and not contact and not c.get("loaded"):
                hand.check_tracking(before, L.joints_of(before["motor_ids"], before["motor_pos"]))
            hand.guard()
            views = []
            mine = c.get("angles") or angles
            for a in (mine if forward else mine[::-1]):
                st = plate.move(a, log)
                time.sleep(PLATE_SETTLE_S)
                st = plate.state()
                shot = photograph(cam, out, a, st["angle"])
                if plan.get("current_each_view"):
                    current = [x["current_ma"] for x in hand.motors()["motors"]]
                    for v in shot:
                        v["current_ma"] = current
                views += shot
            forward = not forward
            after = hand.sample()
            hand.guard()
            if L is not None and not contact and not c.get("loaded"):
                hand.check_tracking(after, L.joints_of(after["motor_ids"], after["motor_pos"]))
            if plan.get("torque_off_each"):
                hand.torque(False)
            return {**c, "before": before, "after": after, "views": views, "t0": t0, "t1": time.time(),
                    "calibration": plan.get("calibration")}

        for k, c in enumerate(todo):
            out = session / "configs" / c["id"]
            out.mkdir(parents=True, exist_ok=True)
            progress.update(current=c["id"], index=k)
            write_json(session / "progress.json", progress)
            if c.get("skip"):
                log(f"{c['id']} skipped: {c['skip']}")
                write_json(out / "skipped.json", {"why": "views", "detail": c["skip"], "t": time.time()})
                continue
            dead = [j for j in hw.DEAD if c.get("joint") == j or abs(c["q"].get(j, 0.0)) > 5.0
                    or any(abs(v.get(j, 0.0)) > 5.0 for v in c.get("via") or [])]
            if dead:
                log(f"{c['id']} left for later: motor of {dead} dead")       # no skipped.json: a revisit retries it
                continue
            if near_stop(c):
                log(f"{c['id']} skipped: {sorted(hw.FAULTS.banned)} faulted too often, pose near its stop")
                write_json(out / "skipped.json", {"why": "backoff", "joints": sorted(hw.FAULTS.banned), "t": time.time()})
                continue
            meta = None
            cmd = c
            for attempt in (0, 1):
                try:
                    meta = shoot(cmd, out)
                    break
                except MotorFault as e:
                    for f in out.glob("v*.jpg"):
                        f.unlink()
                    recover(hand, e, f"{session.name}/{c['id']}#{attempt}", log)
                    if HOLD_STUCK and all(m.get("flags") == ["stuck"] for m in e.motors):
                        # the motor-to-joint map holds wherever the joint stops: retry with the stuck
                        # joints commanded to where they stopped (2° back), so they hold unloaded
                        held = {m["joint"]: round(m["assumed_deg"] - 2.0 * np.sign(m["target_deg"] - m["assumed_deg"]), 1)
                                for m in e.motors}
                        cmd = {**c, "q": {**c["q"], **held}, "q_planned": c["q"], "held": held,
                               "via": [{**v, **{j: min(v.get(j, 0.0), a) if c["q"][j] > a else max(v.get(j, 0.0), a)
                                                for j, a in held.items()}} for v in c.get("via") or []]}
                        log(f"{c['id']}: retrying with {held} held where they stopped")
            if meta is None:
                log(f"{c['id']} skipped after a motor fault and one retry")
                write_json(out / "skipped.json", {"why": "fault", "t": time.time()})
                continue
            write_json(out / "meta.json", meta)
            progress["done_now"] = k + 1
            progress["last_s"] = round(time.time() - meta["t0"], 1)
            write_json(session / "progress.json", progress)
            beat(f"{session.name} {c['id']}")
            if max(meta["after"]["temp_c"]) >= 60.0:
                progress["state"] = "cooling"
                write_json(session / "progress.json", progress)
                hand.cool_if_hot(log)
                progress["state"] = "running"
        progress["state"] = "done"
    except BaseException as e:
        progress.update(state="error", error=repr(e))
        raise
    finally:
        try:
            hand.torque(False)
        finally:
            plate.off()
            if cam is not None:
                cam.close()
            stream.close()
            progress["ended"] = time.time()
            write_json(session / "progress.json", progress)
            print("torque off (hand + plate)", flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("session", type=Path)
    p.add_argument("--limit", type=int)
    a = p.parse_args()
    run(a.session, a.limit)
