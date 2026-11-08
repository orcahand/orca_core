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

import shutil

from hand_study import hw
from hand_study.hw import Camera, Hand, MotorFault, Stream, Turntable, beat, call, recover
from hand_study.mapping import LinearMap

STUDY = Path.home() / ".orca_ui/visual_calibration/hand_study"

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
    todo = [c for c in plan["configs"] if not (session / "configs" / c["id"] / "meta.json").exists()
            and not (session / "configs" / c["id"] / "skipped.json").exists()]
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
            if L is not None and not contact:
                hand.check_tracking(before, L.joints_of(before["motor_ids"], before["motor_pos"]))
            hand.guard()
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
            hand.guard()
            if L is not None and not contact:
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
            if near_stop(c):
                log(f"{c['id']} skipped: {sorted(hw.FAULTS.banned)} faulted too often, pose near its stop")
                write_json(out / "skipped.json", {"why": "backoff", "joints": sorted(hw.FAULTS.banned), "t": time.time()})
                continue
            meta = None
            for attempt in (0, 1):
                try:
                    meta = shoot(c, out)
                    break
                except MotorFault as e:
                    for f in out.glob("v*.jpg"):
                        f.unlink()
                    recover(hand, e, f"{session.name}/{c['id']}#{attempt}", log)
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
