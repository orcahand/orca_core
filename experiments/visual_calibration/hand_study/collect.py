"""Run a plan of hand configurations: the hand moves (plate still), then the plate visits each
stop (hand still) and the camera takes a photo. Torque: the plate is released after every
move; the hand holds during one configuration's photos and is released at the end.

    python -m hand_study.collect SESSION_DIR [--limit N]
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from hand_study.hw import Camera, Hand, Stream, Turntable, beat, call

SETTLE_S = 1.0
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


def run(session: Path, limit: int | None = None, log=print) -> None:
    import cv2
    plan = json.loads((session / "plan.json").read_text())
    angles = plan["angles"]
    todo = [c for c in plan["configs"] if not (session / "configs" / c["id"] / "meta.json").exists()]
    todo = todo[:limit] if limit else todo
    progress = {"phase": "collect", "total": len(plan["configs"]), "todo": len(todo), "done_now": 0,
                "started": time.time(), "state": "running", "current": None, "error": None}
    write_json(session / "progress.json", progress)
    stream = Stream()
    hand, plate = Hand(stream), Turntable()
    cam = Camera()
    lock_camera(log)
    cam.grab()
    try:
        call("/visual-calibration/camera/uvc")
    except Exception as e:
        log(f"camera controls not locked after opening: {e}")
    forward = True
    try:
        if plate.state().get("busy"):
            raise RuntimeError("the turntable is busy")
        hand.torque(True)
        for k, c in enumerate(todo):
            out = session / "configs" / c["id"]
            out.mkdir(parents=True, exist_ok=True)
            progress.update(current=c["id"], index=k)
            write_json(session / "progress.json", progress)
            t0 = time.time()
            if hand.target is None:
                hand.torque(True)
            hand.guard()
            for via in c.get("via") or []:
                hand.move(via)
            hand.move(c["q"], c.get("speed", 30.0))
            time.sleep(SETTLE_S)
            before = hand.sample()
            views = []
            for a in (angles if forward else angles[::-1]):
                st = plate.move(a)
                time.sleep(PLATE_SETTLE_S)
                st = plate.state()
                img = cam.grab()
                name = f"v{int(round(a)):03d}.jpg"
                cv2.imwrite(str(out / name), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
                views.append({"stop": a, "angle": st["angle"], "file": name, "t": time.time()})
            forward = not forward
            after = hand.sample()
            if plan.get("torque_off_each"):
                hand.torque(False)
            write_json(out / "meta.json", {**c, "before": before, "after": after, "views": views,
                                           "t0": t0, "t1": time.time(), "calibration": plan.get("calibration")})
            progress["done_now"] = k + 1
            progress["last_s"] = round(time.time() - t0, 1)
            write_json(session / "progress.json", progress)
            beat(f"{session.name} {c['id']}")
            if max(after["temp_c"]) >= 60.0:
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
