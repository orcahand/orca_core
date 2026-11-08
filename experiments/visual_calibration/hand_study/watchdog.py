"""Torque off (hand + plate) when the night runner dies or hangs: its process is gone, or its
heartbeat is older than STALE_S, while its state says running. Logs to night/<tag>.log.

    python -m hand_study.watchdog TAG
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

from hand_study.hw import call

NIGHT = Path.home() / ".orca_ui/visual_calibration/hand_study/night"
STALE_S = 180.0


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def log(tag: str, msg: str) -> None:
    with (NIGHT / f"{tag}.log").open("a") as f:
        f.write(f"{time.strftime('%H:%M:%S')} WATCHDOG {msg}\n")


def off(tag: str, why: str) -> None:
    for path in ("/torque/disable", "/visual-calibration/turntable/torque-off"):
        for _ in range(3):
            try:
                call(path, {}, timeout=20)
                break
            except Exception as e:
                log(tag, f"{path} failed: {e}")
                time.sleep(2)
    log(tag, f"torque off: {why}")


def main(tag: str) -> None:
    log(tag, "started")
    tripped = False
    while True:
        time.sleep(5)
        try:
            st = json.loads((NIGHT / f"{tag}.state.json").read_text())
        except Exception:
            continue
        if st.get("state") != "running":
            if st.get("state") in ("done", "aborted"):
                log(tag, f"runner {st['state']}, watchdog ends")
                return
            continue
        hb = NIGHT / f"{tag}.heartbeat"
        age = time.time() - hb.stat().st_mtime if hb.exists() else 0.0
        why = None if alive(int(st.get("pid", 0))) else f"runner {st.get('pid')} is gone"
        why = why or (f"no heartbeat for {age:.0f} s" if age > STALE_S else None)
        if why and not tripped:
            off(tag, why)
            tripped = True
            if "gone" in why:
                return
        elif not why:
            tripped = False


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    main(p.parse_args().tag)
