"""Wear programmes on the hand alone (the plate does not move): open/close cycling, and
loaded work against the hand's own joints (thumb-to-finger pinches pressed a little further,
neighbouring fingers squeezed together, every joint held near its configured stops). All
targets stay inside the config's joint limits; the motors run in current-based position
mode, so a press is bounded by their current limit. Guarded like the collection (e-stop,
motor errors, temperature), and logged to <log>.jsonl.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from hand_study.configs import REST, ROM
from hand_study import hw
from hand_study.hw import Hand, MotorFault, beat, recover


def _log(path: Path, **kv) -> None:
    with path.open("a") as f:
        f.write(json.dumps({"t": time.time(), **kv}) + "\n")


def _clip(q: dict) -> dict:
    return {j: min(max(float(v), ROM[j][0]), ROM[j][1]) for j, v in q.items()}


def _checkpoint(hand: Hand, log_path: Path, log, n: int, what: str) -> None:
    try:
        hand.guard()
    except MotorFault as e:
        recover(hand, e, f"wear {what} move {n}", log)
        hand.torque(True)
        hand.move(dict(REST), 20.0)
    m = hand.motors()["motors"]
    _log(log_path, n=n, what=what, temp_c=[x["temp_c"] for x in m], current_ma=[x["current_ma"] for x in m])
    hand.cool_if_hot(log)


def _banned(q: dict, finger: str | None, log) -> bool:
    """Back-off: a joint that faulted too often is left out of the loaded work."""
    hit = (set(q) | ({f"{finger}_mcp", "thumb_mcp"} if finger else set())) & hw.FAULTS.banned
    if hit:
        log(f"loaded: skipped, {sorted(hit)} faulted too often")
    return bool(hit)


def cycle(hand: Hand, prog: dict, minutes: float, log_path: Path, log=print) -> int:
    """Open/close cycling for `minutes`; returns the number of moves."""
    poses, seq = prog["poses"], prog["sequence"]
    speed, hold = prog["speed_deg_s"], prog["hold_s"]
    t_end = time.time() + minutes * 60
    hand.torque(True)
    n = 0
    try:
        while time.time() < t_end:
            for name in seq:
                hand.move({**REST, **poses[name]}, speed)
                time.sleep(hold)
                n += 1
            beat(f"cycle {n}")
            if n % 60 == 0:
                _checkpoint(hand, log_path, log, n, "cycle")
        hand.move(dict(REST), 30.0)
    finally:
        hand.torque(False)
    _log(log_path, n=n, what="cycle done")
    return n


def loaded(hand: Hand, prog: dict, minutes: float, log_path: Path, log=print) -> int:
    """Loaded work for `minutes`, round after round: pinches, finger presses, stops."""
    speed, hold, extra = prog["speed_deg_s"], prog["hold_s"], prog["press_extra_deg"]
    t_end = time.time() + minutes * 60
    hand.torque(True)
    n = 0
    try:
        while time.time() < t_end:
            for f, q in prog["pinches"].items():
                if _banned(q, f, log):
                    continue
                near = {j: 0.8 * v for j, v in q.items()}
                hand.move({**REST, **near}, speed)
                press = _clip({**q, f"{f}_mcp": q.get(f"{f}_mcp", 0.0) + extra, "thumb_mcp": q.get("thumb_mcp", 0.0) + extra})
                hand.move({**REST, **press}, 10.0)
                time.sleep(hold)
                hand.move({**REST, **near}, speed)
                n += 1
                beat(f"pinch {f}")
                _checkpoint(hand, log_path, log, n, "loaded")
            hand.move(dict(REST), speed)
            for k, q in prog["presses"].items():
                if _banned(q, None, log):
                    continue
                hand.move({**REST, **q}, speed / 2)
                hand.move({**REST, **_clip({j: v + extra * (1 if v > 0 else -1) / 2 for j, v in q.items()})}, 5.0)
                time.sleep(hold)
                hand.move(dict(REST), speed)
                n += 1
                beat(f"press {k}")
                _checkpoint(hand, log_path, log, n, "loaded")
            for k, q in prog["stops"].items():
                if _banned(q, None, log):
                    continue
                hand.move({**REST, **q}, speed)
                time.sleep(hold)
                hand.move(dict(REST), speed)
                n += 1
                beat(f"stop {k}")
                _checkpoint(hand, log_path, log, n, "loaded")
                if time.time() > t_end:
                    break
            _checkpoint(hand, log_path, log, n, "loaded")
        hand.move(dict(REST), 30.0)
    finally:
        hand.torque(False)
    _log(log_path, n=n, what="loaded done")
    return n
