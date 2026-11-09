"""Experimental collision phase (Clemens, 9 Nov): joint motions that end in a contact of two URDF
surfaces (finger on finger, thumb on finger, finger on palm, abduction neighbours), found from the
URDF, run on the hand: the obstacle held at the console's current, the moving joint driven slowly
into it under a capped current, contact = the current rise or a stall inside the expected range.
Each contact is a geometric fact, "these URDF surfaces touch at these motor positions", which the
map fit uses as an extra residual (offline.py / curve.Model weights).

    python -m hand_study.collision pairs            # the contact pairs from the URDF (cached)
    (the night runs them: night.py step kind "collision")
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from hand_study import hw
from hand_study.configs import JOINTS, ROM
from hand_study.hw import Hand, Turntable, beat, call

STUDY = Path.home() / ".orca_ui/visual_calibration/hand_study"
PAIRS = STUDY / "collision_pairs.json"
TOUCH_MM = 0.6           # surfaces closer than this touch (samples ~2 mm apart)
FREE_MM = 3.0            # the sweep starts at least this far apart
MARGIN_DEG = 5.0         # contacts this close to a joint limit are not used (the stop would answer)
RANGE_DEG = (-10.0, 5.0)  # expected window around the URDF contact: earlier (skin, miscalibration) or a bit later
START_BEFORE_DEG = 15.0
OVERSHOOT_DEG = 6.0      # the approach target ends this far past the URDF contact
APPROACH_DEG_S = 3.0
POLL_HZ = 20.0
CAP_MA = 250.0           # the moving motor's goal current (XC330-T288: gentle)
RISE_MA = 70.0           # a contact: current this far over the free-motion level, 2 samples running ...
STALL_DEG = 4.0          # ... or the joint this far behind its target
REPS = 3
PER_JOINT = 2

TEMPLATES = {
    "open": {},
    "half": {f"{f}_{k}": 45.0 for f in ("index", "middle", "ring", "pinky") for k in ("mcp", "pip")},
    "opposed": {"thumb_cmc": -20.0, "thumb_abd": 45.0, "thumb_mcp": 10.0, "index_mcp": 35.0, "index_pip": 35.0,
                "middle_mcp": 35.0, "middle_pip": 35.0},
    "fist": {f"{f}_mcp": 85.0 for f in ("index", "middle", "ring", "pinky")},
    "spread": {"index_abd": -15.0, "middle_abd": -5.0, "ring_abd": 5.0, "pinky_abd": 15.0},
    "hook": {f"{f}_pip": 95.0 for f in ("index", "middle", "ring", "pinky")},
    "curl": {**{f"{f}_mcp": 90.0 for f in ("index", "middle", "ring", "pinky")}, **{f"{f}_pip": 60.0 for f in ("index", "middle", "ring", "pinky")}},
    "across": {"thumb_cmc": 25.0, "thumb_abd": 50.0, "thumb_mcp": 30.0, "thumb_dip": 20.0},
    "pinch_lo": {"thumb_cmc": -20.0, "thumb_abd": 40.0, "thumb_mcp": 25.0, "index_mcp": 50.0, "index_pip": 30.0},
    "pinch_hi": {"thumb_cmc": -10.0, "thumb_abd": 50.0, "thumb_mcp": 40.0, "index_mcp": 30.0, "index_pip": 50.0,
                 "middle_mcp": 30.0, "middle_pip": 50.0},
}


class Geometry:
    """The URDF's link surfaces (mm) and their distances at a joint configuration."""

    def __init__(self, spacing: float = 2.0, per_link: int = 300):
        from visual_calibration import live_fit as lf
        from visual_calibration.hand_model import HandModel
        self.model = HandModel.load("right")
        rng = np.random.default_rng(0)
        self.cloud = {}
        for l, v in self.model.visuals.items():
            if l in ("forearm", "tower"):
                continue
            c = lf.surface(v.path, v.origin, v.scale, spacing)
            self.cloud[l] = c[rng.permutation(len(c))[:per_link * (3 if l == "carpals" else 1)]]
        self.parent = {j.child: j.parent for j in self.model.joints.values()}
        self.joint_of = {j.child: n for n, j in self.model.joints.items()}
        self.sub = {n: self._subtree(j.child) for n, j in self.model.joints.items() if n in JOINTS}
        self.adjacent = {frozenset((j.parent, j.child)) for j in self.model.joints.values()}

    def _subtree(self, link: str) -> list[str]:
        out, todo = [], [link]
        while todo:
            l = todo.pop()
            out += [l] if l in self.cloud else []
            todo += [j.child for j in self.model.joints.values() if j.parent == l]
        return out

    def points(self, q: dict[str, float]) -> dict[str, np.ndarray]:
        out = {}
        for l, p in self.model.link_poses({j: float(q.get(j, 0.0)) for j in JOINTS}).items():
            if l in self.cloud:
                out[l] = self.cloud[l] @ p.rotation.T + p.translation * 1000.0
        return out

    def nearest(self, q: dict[str, float], moving: list[str]) -> tuple[float, tuple[str, str] | None]:
        P = self.points(q)
        best, pair = np.inf, None
        for a in moving:
            A = P[a]
            ca, ra = A.mean(0), np.linalg.norm(A - A.mean(0), axis=1).max()
            for b, B in P.items():
                if b in moving or frozenset((a, b)) in self.adjacent:
                    continue
                cb, rb = B.mean(0), np.linalg.norm(B - B.mean(0), axis=1).max()
                if np.linalg.norm(ca - cb) - ra - rb > best:
                    continue
                d = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1).min())
                if d < best:
                    best, pair = float(d), (a, b)
        return best, pair

    def obstacle_joint(self, link: str) -> str | None:
        l = link
        while l in self.joint_of:
            j = self.joint_of[l]
            if j in JOINTS and j != "wrist":
                return j
            l = self.parent[l]
        return None

    def contact(self, j: str, q: dict[str, float], sign: float, lo: float, hi: float) -> dict | None:
        """Sweep joint j from q[j] towards `sign`: the first configuration where its subtree touches a
        non-adjacent link (2° steps, refined to 0.25°); None if none inside [lo, hi] or not free at the start."""
        moving = self.sub[j]
        q = dict(q)
        start = q.get(j, 0.0)
        d0, _ = self.nearest(q, moving)
        if d0 < FREE_MM:
            return None
        prev = start
        for a in np.arange(start, hi if sign > 0 else lo, 2.0 * sign):
            q[j] = float(a)
            d, pair = self.nearest(q, moving)
            if d < TOUCH_MM:
                for b in np.arange(prev, a + 0.25 * sign, 0.25 * sign):
                    q[j] = float(b)
                    d, pair = self.nearest(q, moving)
                    if d < TOUCH_MM:
                        return {"angle": round(float(b), 2), "links": list(pair)}
                return {"angle": round(float(a), 2), "links": list(pair)}
            prev = float(a)
        return None


def find_pairs(rom: dict | None = None) -> list[dict]:
    """Every joint's contacts from the templates, both directions, at most PER_JOINT per joint
    (different obstacles first); each with the expected angle and window, and how the contact moves
    with the obstacle joint (a table over its angle, ±10°)."""
    rom = rom or ROM
    g = Geometry()
    found = []
    for j in JOINTS:
        if j == "wrist":
            continue
        lo, hi = float(rom[j][0]) + MARGIN_DEG, float(rom[j][1]) - MARGIN_DEG
        cand = []
        for tname, t in TEMPLATES.items():
            for sign in (1.0, -1.0):
                q = {k: float(np.clip(v, *rom[k])) for k, v in t.items() if k != j}
                q[j] = float(np.clip(t.get(j, 0.0), lo, hi)) if sign > 0 else float(np.clip(t.get(j, 0.0), lo, hi))
                c = g.contact(j, q, sign, lo, hi)
                if c is None or not lo <= c["angle"] <= hi or abs(c["angle"] - q[j]) < START_BEFORE_DEG / 2:
                    continue
                ob = g.obstacle_joint(c["links"][1])
                cand.append({"joint": j, "template": tname, "q": q, "sign": sign, **c, "obstacle_joint": ob})
        seen, keep = set(), []
        for c in sorted(cand, key=lambda c: (c["links"][1] in seen, abs(c["angle"] - c["q"][j]))):
            if len(keep) >= PER_JOINT:
                break
            key = (c["links"][1], c["sign"])
            if key in seen:
                continue
            seen.add(c["links"][1])
            seen.add(key)
            keep.append(c)
        for c in keep:
            c["window"] = sorted([round(c["angle"] + c["sign"] * RANGE_DEG[0], 1), round(c["angle"] + c["sign"] * RANGE_DEG[1], 1)])
            ob = c["obstacle_joint"]
            if ob:
                tab = []
                for dq in (-10.0, -5.0, 0.0, 5.0, 10.0):
                    q = dict(c["q"])
                    q[ob] = float(np.clip(q.get(ob, 0.0) + dq, *rom[ob]))
                    lo2, hi2 = float(rom[j][0]), float(rom[j][1])
                    r = g.contact(j, q, c["sign"], lo2, hi2)
                    if r:
                        tab.append([q[ob], r["angle"]])
                c["table"] = tab
            c["id"] = f"{j}-{c['links'][1]}-{'up' if c['sign'] > 0 else 'down'}"
            found.append(c)
    return found


def pairs(rom: dict | None = None) -> list[dict]:
    if not PAIRS.exists():
        PAIRS.write_text(json.dumps(find_pairs(rom), indent=1))
    return json.loads(PAIRS.read_text())


def expected(pair: dict, q_obstacle: float | None) -> float:
    """The URDF contact angle of the moving joint at this obstacle angle."""
    t = pair.get("table") or []
    if q_obstacle is None or len(t) < 2:
        return float(pair["angle"])
    t = sorted(t)
    return float(np.interp(q_obstacle, [a for a, _ in t], [b for _, b in t]))


# -- on the hand ------------------------------------------------------------------
def approach(hand: Hand, L, pair: dict, log=print) -> dict:
    """One slow approach into the contact; returns what happened (contact / none / unexpected) and the trace."""
    j, sign = pair["joint"], pair["sign"]
    mid = {x["joint"]: x["id"] for x in hand.motors()["motors"]}
    start_q = float(np.clip(pair["angle"] - sign * START_BEFORE_DEG, *ROM[j]))
    goal = {**pair["q"], j: start_q}
    hand.move(goal, 30.0)
    time.sleep(1.0)
    hand.guard()
    ob = pair.get("obstacle_joint")
    call("/control/motor_currents", {"ma": {str(mid[j]): CAP_MA}}, timeout=20)
    trace, base_i = [], []
    lo_w, hi_w = pair["window"]
    end = pair["angle"] + sign * OVERSHOOT_DEG
    q = start_q
    t0 = time.time()
    out = {"result": "none"}
    hits = 0
    try:
        while (end - q) * sign > 0:
            q += sign * APPROACH_DEG_S / POLL_HZ
            hw.HAND_MOVED[0] = time.time()
            call("/joints/target", {"angles": {**{k: v for k, v in hand.target.items() if k not in hw.DEAD}, j: q}})
            hand.target[j] = q
            m = {x["joint"]: x for x in hand.motors()["motors"]}
            est = L.to_joint(j, m[j]["position"])
            i = abs(float(m[j]["current_ma"]))
            trace.append({"t": round(time.time() - t0, 3), "target": round(q, 3), "pos": m[j]["position"], "est": round(est, 3),
                          "i": round(i, 1), **({"ob_pos": m[ob]["position"], "ob_i": m[ob]["current_ma"]} if ob else {}),
                          "temp": m[j]["temp_c"]})
            if len(trace) > 1:
                trace[-1]["vel"] = round((trace[-1]["pos"] - trace[-2]["pos"]) / max(trace[-1]["t"] - trace[-2]["t"], 1e-3), 4)
            if abs(q - start_q) < 3.0:
                base_i.append(i)
                continue
            level = float(np.percentile(base_i, 90)) if base_i else 0.0
            hit = i > level + RISE_MA or abs(q - est) > STALL_DEG
            hits = hits + 1 if hit else 0
            if hits >= 2:
                k = len(trace) - 2
                c = trace[k]
                inside = lo_w <= c["est"] <= hi_w
                out = {"result": "contact" if inside else "unexpected", "at": c, "index": k,
                       "motors": {x["joint"]: x["position"] for x in m.values()},
                       "why": "current" if trace[-1]["i"] > level + RISE_MA else "stall", "level_ma": round(level, 1)}
                break
            if m[j]["temp_c"] >= hw.PAUSE_TEMP_C:
                out = {"result": "hot", "at": trace[-1]}
                break
            time.sleep(max(0.0, 1.0 / POLL_HZ - 0.02))
    finally:
        back = {**pair["q"], j: start_q}
        try:
            call("/control/motor_currents", {"ma": {}}, timeout=20)        # every motor back on the console's current
        except Exception as e:
            log(f"currents not restored: {e}")
        hand.move(back, 20.0)
    out["trace"] = trace
    out["expected"] = pair["angle"]
    return out


def run(session: Path, prs: list[dict], reps: int = REPS, photos: bool = True, log=print) -> None:
    """Every pair `reps` times (resumable: a rep with contact.json is done); the first rep of each
    pair is also photographed at the contact (meta.json, solved by the workers like any pose)."""
    import cv2
    from hand_study.collect import PLATE_SETTLE_S, lock_camera, write_json
    from hand_study.hw import Camera, Stream
    from hand_study.mapping import LinearMap
    plan = json.loads((session / "plan.json").read_text())
    L = LinearMap(STUDY / f"calib_{plan['calibration']}")
    stream = Stream()
    hand, plate = Hand(stream), Turntable()
    cam = Camera() if photos else None
    if photos:
        lock_camera(log)
    call("/visual-calibration/turntable/torque-off", {}, timeout=20)
    hand.torque(True)
    prog = {"phase": "collision", "total": len(prs) * reps, "done_now": 0, "started": time.time(), "state": "running"}
    try:
        for p in prs:
            if p["joint"] in hw.DEAD or p.get("obstacle_joint") in hw.DEAD:
                log(f"{p['id']}: a motor is dead, left for later")
                continue
            for r in range(reps):
                d = session / "configs" / f"{p['id']}-r{r}"
                if (d / "contact.json").exists():
                    continue
                d.mkdir(parents=True, exist_ok=True)
                prog.update(current=d.name)
                write_json(session / "progress.json", prog)
                beat(f"collision {d.name}")
                hand.cool_if_hot(log)
                t0 = time.time()
                res = approach(hand, L, p, log)
                write_json(d / "trace.json", res.pop("trace"))
                write_json(d / "contact.json", {**res, "pair": p, "rep": r, "t": t0})
                log(f"{d.name}: {res['result']}" + (f" at {res['at']['est']:.1f}° (URDF {p['angle']:.1f}°, {res['why']})"
                                                    if "at" in res and res["result"] != "hot" else ""))
                if res["result"] == "unexpected":
                    log(f"{p['id']}: stall outside {p['window']}: pair dropped")
                    break
                if res["result"] == "contact" and r == 0 and photos:
                    hold = {**p["q"], p["joint"]: res["at"]["target"]}
                    call("/control/motor_currents", {"ma": {str(next(x['id'] for x in hand.motors()['motors'] if x['joint'] == p['joint'])): CAP_MA}}, timeout=20)
                    try:
                        hand.move(hold, 5.0)
                        time.sleep(1.0)
                        before = hand.sample()
                        views = []
                        time.sleep(0.5)
                        for a in plan["angles"]:
                            st = plate.move(a, log)
                            time.sleep(PLATE_SETTLE_S)
                            st = plate.state()
                            name = f"v{int(round(a)):03d}.jpg"
                            cv2.imwrite(str(d / name), cam.grab(), [cv2.IMWRITE_JPEG_QUALITY, 95])
                            views.append({"stop": a, "angle": st["angle"], "file": name, "t": time.time()})
                        call("/visual-calibration/turntable/torque-off", {}, timeout=20)
                        after = hand.sample()
                        write_json(d / "meta.json", {"id": d.name, "q": hold, "kind": "contact", "joint": p["joint"],
                                                     "before": before, "after": after, "views": views, "t0": t0, "t1": time.time(),
                                                     "calibration": plan["calibration"], "contact": True})
                    finally:
                        call("/control/motor_currents", {"ma": {}}, timeout=20)
                        hand.move({**p["q"], p["joint"]: p["angle"] - p["sign"] * START_BEFORE_DEG}, 20.0)
                prog["done_now"] += 1
                write_json(session / "progress.json", prog)
        prog["state"] = "done"
    finally:
        write_json(session / "progress.json", prog)
        try:
            call("/control/motor_currents", {"ma": {}}, timeout=20)
        except Exception:
            pass
        hand.torque(False)
        stream.close()


def constraints(sessions: list[Path], base_map) -> list[dict]:
    """The contacts as pseudo-observations for the map fit: the moving joint's map input (from its
    motor) and the URDF contact angle at the obstacle's mapped angle."""
    out = []
    for s in sessions:
        for f in sorted(s.glob("configs/*/contact.json")):
            c = json.loads(f.read_text())
            if c.get("result") != "contact":
                continue
            p, j = c["pair"], c["pair"]["joint"]
            r = {"run": json.loads((s / "plan.json").read_text())["calibration"], "motor": c["motors"]}
            x = base_map(r)
            ob = p.get("obstacle_joint")
            out.append({"joint": j, "x": x, "y": expected(p, x[ob] if ob else None), "id": f"{s.name}/{f.parent.name}"})
    return out


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["pairs"]:
        PAIRS.unlink(missing_ok=True)
        ps = pairs()
        cover = sorted({p["joint"] for p in ps})
        for p in ps:
            print(f"{p['id']:36s} {p['template']:8s} contact {p['angle']:6.1f}° window {p['window']} obstacle {p['obstacle_joint']}"
                  f" table {len(p.get('table') or [])}")
        print(f"{len(ps)} pairs, joints covered {len(cover)}/{len(JOINTS) - 1}: missing {sorted(set(JOINTS) - set(cover) - {'wrist'})}")
