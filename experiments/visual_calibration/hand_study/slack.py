"""Slack (deadzone) checks, ~5 min, about hourly and right after the collisions (Clemens, 9 Nov).

Per joint group (the four MCPs together, then the PIPs, the abductions, each thumb joint, the wrist):
from a mid pose the motors reverse slowly, +-AMP_MOTOR_DEG, CYCLES times, the plate parked at one side
view. The camera films at its own rate; offline each moving link is tracked in 2D (dense optical flow
inside its URDF silhouette, no fit) and the deadzone is the motor travel after a reversal before the
link moves. The current is logged and its deadband found as orca_core's tension_sweep does
(DEADBAND_CONFIRM samples above 25 mA) and compared with the guided tension's own numbers (setup.yaml).

    python -m hand_study.slack analyse TAG        # -> protocol/<tag>/slack.json
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np

from hand_study import hw
from hand_study.configs import JOINTS, ROM
from hand_study.hw import Hand, Turntable, beat, call
from hand_study.paths import STUDY

VIEW_DEG = 112.5               # a side view (the fingers' flexion in the image plane)
AMP_MOTOR_DEG = 8.0
MOTOR_DEG_S = 4.0
CYCLES = 3
POLL_HZ = 20.0
FRAME_W = 960
MAX_FPS = 30.0
FINGERS = ("index", "middle", "ring", "pinky")
GROUPS = [("mcp", [f"{f}_mcp" for f in FINGERS]), ("pip", [f"{f}_pip" for f in FINGERS]),
          ("abd", [f"{f}_abd" for f in FINGERS]), ("thumb_cmc", ["thumb_cmc"]), ("thumb_abd", ["thumb_abd"]),
          ("thumb_mcp", ["thumb_mcp"]), ("thumb_dip", ["thumb_dip"]), ("wrist", ["wrist"])]
DEADBAND_MA = 25.0
DEADBAND_CONFIRM = 3


def mid_pose(group: list[str]) -> dict[str, float]:
    q = {j: 0.0 for j in JOINTS}
    for j in group:
        lo, hi = ROM[j]
        q[j] = round(lo + 0.45 * (hi - lo), 1) if not j.endswith("_abd") else 0.0
    return q


class Filmer(threading.Thread):
    """Frames at the camera's rate, downscaled, JPEG with their time, until stopped."""

    def __init__(self, cam, out: Path):
        super().__init__(daemon=True)
        self.cam, self.out, self.stop, self.frames = cam, out, threading.Event(), []

    def run(self):
        import cv2
        k = 0
        while not self.stop.is_set():
            img = self.cam.grab()
            t = time.time()
            s = FRAME_W / img.shape[1]
            small = cv2.resize(img, (FRAME_W, round(img.shape[0] * s)), interpolation=cv2.INTER_AREA)
            name = f"f{k:05d}.jpg"
            cv2.imwrite(str(self.out / name), small, [cv2.IMWRITE_JPEG_QUALITY, 88])
            self.frames.append({"t": t, "file": name})
            k += 1
            time.sleep(max(0.0, 1.0 / MAX_FPS - (time.time() - t)))


def run(session: Path, cam, log=print, limit: int | None = None) -> None:
    from hand_study.collect import write_json
    from hand_study.hw import Stream
    from hand_study.mapping import LinearMap
    plan = json.loads((session / "plan.json").read_text())
    L = LinearMap(STUDY / f"calib_{plan['calibration']}")
    stream = Stream()
    hand, plate = Hand(stream), Turntable()
    plate.move(VIEW_DEG, log)
    st = plate.state()
    plate.off()
    time.sleep(0.5)
    hand.torque(True)
    try:
        for name, group in GROUPS[: limit or None]:
            group = [j for j in group if j not in hw.DEAD]
            d = session / "configs" / name
            if (d / "slack.json").exists() or not group:
                continue
            d.mkdir(parents=True, exist_ok=True)
            beat(f"slack {name}")
            hand.cool_if_hot(log)
            q0 = mid_pose(group)
            hand.move(q0, 30.0)
            time.sleep(1.0)
            hand.guard()
            mids = {j: L.motor[j] for j in group}
            amp = {j: np.radians(AMP_MOTOR_DEG) / L.ratio[mids[j]] for j in group}       # joint deg per AMP motor deg
            legs = [1, -1] * CYCLES + [1]
            film = Filmer(cam, d)
            film.start()
            log_rows = []
            t0 = time.time()
            try:
                pos = 0.0
                for k, sgn in enumerate(legs):
                    target = sgn * (0.5 if k == 0 or k == len(legs) - 1 else 1.0)
                    n = max(1, int(abs(target - pos) * AMP_MOTOR_DEG / MOTOR_DEG_S * POLL_HZ))
                    for i in range(1, n + 1):
                        f = pos + (target - pos) * i / n
                        hw.HAND_MOVED[0] = time.time()
                        call("/joints/target", {"angles": {**{j: v for j, v in hand.target.items() if j not in hw.DEAD},
                                                           **{j: q0[j] + f * amp[j] for j in group}}})
                        m = {x["joint"]: x for x in hand.motors()["motors"]}
                        log_rows.append({"t": time.time(), "leg": k, "f": round(f, 4),
                                         **{j: [m[j]["position"], m[j]["current_ma"]] for j in group}})
                        time.sleep(max(0.0, 1.0 / POLL_HZ - 0.03))
                    pos = target
                for j in group:
                    hand.target[j] = q0[j] + pos * amp[j]
            finally:
                film.stop.set()
                film.join(timeout=5)
            write_json(d / "slack.json", {"group": name, "joints": group, "q0": q0, "amp_joint_deg": amp,
                                          "motor_ids": mids, "ratio": {j: L.ratio[mids[j]] for j in group},
                                          "plate_angle": st.get("angle"), "frames": film.frames, "log": log_rows,
                                          "calibration": plan["calibration"], "t0": t0, "t1": time.time()})
            log(f"slack {session.name}/{name}: {len(film.frames)} frames, {len(log_rows)} motor samples in {time.time() - t0:.0f} s")
        hand.move({j: 0.0 for j in JOINTS}, 30.0)
    finally:
        hand.torque(False)
        stream.close()


# -- analysis -------------------------------------------------------------------------
def current_deadband(deg: np.ndarray, ma: np.ndarray) -> float | None:
    """tension_sweep.summarize's rule: travel from the reversal to the first of DEADBAND_CONFIRM samples above DEADBAND_MA."""
    above = 0
    for i, a in enumerate(np.abs(ma)):
        above = above + 1 if a >= DEADBAND_MA else 0
        if above >= DEADBAND_CONFIRM:
            return float(abs(deg[i - DEADBAND_CONFIRM + 1] - deg[0]))
    return None


def track(d: Path, rec: dict, fit, j: str) -> tuple[np.ndarray, np.ndarray] | None:
    """The joint's moving link, 2D: (frame times, displacement along its main direction, px at 960)."""
    import cv2
    child = fit.model.joints[j].child
    q = {k: rec["q0"].get(k, 0.0) for k in fit.joints}
    x = np.r_[fit.x[:6], [q[k] for k in fit.joints]]
    lab = fit.label(x, rec["plate_angle"], fit.lens_s, (round(fit.lens.size[1] * fit.s), fit.width))
    links = [l for l in fit._subtree(child)]
    m = np.isin(lab, [fit.ids[l] for l in links if l in fit.ids])
    if m.sum() < 30:
        return None
    m = cv2.dilate(m.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    ys, xs = np.nonzero(m)
    v0, v1, u0, u1 = max(ys.min() - 10, 0), ys.max() + 10, max(xs.min() - 10, 0), xs.max() + 10
    prev, acc, out_t, path = None, np.zeros(2), [], []
    for f in rec["frames"]:
        img = cv2.imread(str(d / f["file"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        g = img[v0:v1, u0:u1]
        if prev is not None:
            flow = cv2.calcOpticalFlowFarneback(prev, g, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            acc = acc + flow[m[v0:v1, u0:u1][:g.shape[0], :g.shape[1]]].mean(0)
        prev = g
        out_t.append(f["t"])
        path.append(acc.copy())
    P = np.array(path)
    if len(P) < 10:
        return None
    c = P - P.mean(0)
    axis = np.linalg.svd(c, full_matrices=False)[2][0]
    return np.array(out_t), c @ axis


def deadzones(rec: dict, j: str, tr) -> dict:
    """Per reversal: motor deg before the link moves (camera) and before the current loads (tension_sweep's rule)."""
    t = np.array([r["t"] for r in rec["log"]])
    mdeg = np.degrees(np.array([r[j][0] for r in rec["log"]]))
    ma = np.array([r[j][1] for r in rec["log"]])
    leg = np.array([r["leg"] for r in rec["log"]])
    cam, cur = [], []
    for k in range(1, leg.max() + 1):
        i = np.nonzero(leg == k)[0]
        if len(i) < 5:
            continue
        cb = current_deadband(mdeg[i], ma[i])
        if cb is not None:
            cur.append(cb)
        if tr is None:
            continue
        ft, s = tr
        sel = (ft >= t[i[0]]) & (ft <= t[i[-1]])
        if sel.sum() < 4:
            continue
        fs, ftt = s[sel], ft[sel]
        swing = abs(fs[-1] - fs[0])
        thr = max(0.5, 0.12 * swing)
        moved = np.nonzero(np.abs(fs - fs[0]) > thr)[0]
        if len(moved) and swing > 2.0:
            tm = ftt[moved[0]]
            cam.append(float(abs(np.interp(tm, t[i], mdeg[i]) - mdeg[i[0]])))
    r = rec["ratio"][j]
    to_joint = (lambda v: round(float(np.radians(v) / r), 2)) if r else (lambda v: None)
    med = lambda v: round(float(np.median(v)), 2) if v else None
    return {"camera_motor_deg": med(cam), "camera_joint_deg": to_joint(np.median(cam)) if cam else None,
            "current_motor_deg": med(cur), "current_joint_deg": to_joint(np.median(cur)) if cur else None,
            "n_reversals": len(cam), "camera_all": [round(x, 2) for x in cam], "current_all": [round(x, 2) for x in cur]}


def analyse(tag: str) -> dict:
    import yaml
    from hand_study.estimate import StudyFit
    spec = json.loads((STUDY / "night" / f"{tag}.json").read_text())
    fit = None
    out = {"checks": [], "tension": None}
    try:
        su = yaml.safe_load((Path(spec["bench"]) / "setup.yaml").read_text())
        out["tension"] = su["tension_progress"]["hands"][spec["hand"]]["best"]
    except Exception:
        pass
    mount = None
    mf = STUDY / f"mount_{tag}.json"
    if mf.exists():
        mount = json.loads(mf.read_text()).get("mount")
    for s in spec["sets"].get("slack", []):
        s = Path(s)
        chk = {"name": s.name.split(f"{tag}-", 1)[-1], "t": None, "joints": {}}
        for f in sorted(s.glob("configs/*/slack.json")):
            rec = json.loads(f.read_text())
            chk["t"] = chk["t"] or rec["t0"]
            if fit is None:
                fit = StudyFit()
            fit.x = np.r_[np.array(mount) if mount else np.zeros(6), np.zeros(len(fit.joints))]
            for j in rec["joints"]:
                try:
                    tr = track(f.parent, rec, fit, j)
                except Exception:
                    tr = None
                chk["joints"][j] = deadzones(rec, j, tr)
        if chk["joints"]:
            out["checks"].append(chk)
    p = STUDY / "protocol" / tag / "slack.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, indent=1, default=float))
    return out


def prior(tag: str) -> dict[str, float]:
    """Backlash prior for advanced.Advanced: -deadzone/2 (joint deg), the median over the checks."""
    p = STUDY / "protocol" / tag / "slack.json"
    if not p.exists():
        return {}
    v: dict[str, list] = {}
    for c in json.loads(p.read_text())["checks"]:
        for j, d in c["joints"].items():
            x = d.get("camera_joint_deg") if d.get("camera_joint_deg") is not None else d.get("current_joint_deg")
            if x is not None:
                v.setdefault(j, []).append(x)
    return {j: -float(np.median(x)) / 2 for j, x in v.items()}


if __name__ == "__main__":
    import sys
    if sys.argv[1] == "analyse":
        print(json.dumps({c["name"]: {j: d["camera_joint_deg"] for j, d in c["joints"].items()} for c in analyse(sys.argv[2])["checks"]}, indent=1))
