"""Where should the camera be? Offline, from the URDF: for candidate camera poses (the mount's
rail distance, mast height, tilt; and a second camera from above) and every plate angle, how
many pixels of the hand's silhouette (and of its inner outlines, i.e. what a shaded render or
crease edges would add) change per degree of each joint. More px/deg = easier to identify.

Mount (rigs/v1/camera-rig/config.yaml camera.mount): lens at (distance + 6.5, 0, height - 0.75)
in the rig frame (plate at 0), looking along (-cos tilt, 0, -sin tilt). Rail 190..290, mast
250..350 (10 mm holes, slide only); tilt is printed into cam_platform (a reprint).

    python -m hand_study.camstudy        # report/camstudy.json + camstudy.csv
"""

from __future__ import annotations

import csv
import json
import time

import numpy as np

from hand_study.configs import JOINTS
from hand_study.estimate import StudyFit
from hand_study.plan import REFS_A, SPARSE_ANGLES, STUDY
from visual_calibration.overlay import _roll

OUT = STUDY / "report"
ANGLES = np.arange(0.0, 360.0, 22.5)
DELTA = 3.0
LENS_OFFSET = (6.5, -0.75)
NOW = {"distance": 240.0, "height": 270.0, "tilt": 3.12}


def eye_dir(distance: float, height: float, tilt: float) -> tuple[np.ndarray, np.ndarray]:
    t = np.radians(tilt)
    eye = np.array([distance + LENS_OFFSET[0], 0.0, height + LENS_OFFSET[1]])
    return eye, np.array([-np.cos(t), 0.0, -np.sin(t)])


def top(height: float, offset: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """A second camera above the plate axis (offset toward the main camera), looking down."""
    t = np.radians(4.0)                      # not exactly vertical: look_at needs an up that is not the axis
    return np.array([offset, 0.0, height]), np.array([-np.sin(t), 0.0, -np.cos(t)])


def in_frame(f: StudyFit, x: np.ndarray, shape) -> float:
    """The smallest share, over the plate angles, of the hand's surface points inside the image."""
    worst = 1.0
    for a in ANGLES:
        pts = [T.apply_to_points(f.anchor[l]) for l, T in f.cam_poses(x, a).items() if l in f.anchor]
        uv = f.lens_s.project(np.vstack(pts))
        ok = (uv[:, 0] >= 0) & (uv[:, 0] < shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < shape[0])
        worst = min(worst, float(ok.mean()))
    return worst


def sensitivity(f: StudyFit, cam, poses: dict[str, dict], shape) -> dict:
    """[pose][angle][joint] -> (silhouette px/deg, inner-outline px/deg) at the fit width."""
    f.cam = cam
    out = {"_in_frame": 1.0}
    for name, q in poses.items():
        x = f.x.copy()
        x[6:] = [q.get(j, 0.0) for j in f.joints]
        out["_in_frame"] = min(out["_in_frame"], in_frame(f, x, shape))
        per = []
        for a in ANGLES:
            lab0 = f.label(x, a, f.lens_s, shape)
            sil0, out0 = lab0 > 0, f.outlines(lab0) > 0
            row = []
            for j in JOINTS:
                xj = x.copy()
                k = 6 + f.joints.index(j)
                xj[k] = xj[k] + DELTA if xj[k] + DELTA <= f.hi[k - 6] else xj[k] - DELTA
                lab = f.label(xj, a, f.lens_s, shape)
                row.append(((sil0 ^ (lab > 0)).sum() / DELTA, (out0 ^ (f.outlines(lab) > 0)).sum() / DELTA))
            per.append(row)
        out[name] = np.array(per)            # angles x joints x 2
    return out


def score(S: dict, stops=SPARSE_ANGLES) -> dict:
    """Per joint: summed px/deg over the stops used today (mean over poses), the best plate angle."""
    idx = [int(np.argmin(np.abs(ANGLES - s))) for s in stops]
    A = np.mean([v for k, v in S.items() if not k.startswith("_")], 0)          # angles x joints x 2
    sil, inner = A[idx, :, 0].sum(0), A[idx, :, 1].sum(0)
    return {"stops_sil": sil.round(1).tolist(), "stops_inner": inner.round(1).tolist(),
            "best_angle": [float(ANGLES[i]) for i in A[:, :, 0].argmax(0)],
            "per_angle_sil": A[:, :, 0].round(1).tolist(),
            "geo_mean_sil": float(np.exp(np.mean(np.log(sil + 1e-3)))), "min_sil": float(sil.min()),
            "in_frame": round(S["_in_frame"], 3),
            "weakest": JOINTS[int(np.argmin(sil))]}


def main() -> dict:
    t0 = time.time()
    f = StudyFit()
    shape = (round(f.lens.size[1] * f.s), f.width)
    bundle = STUDY / "mount_bundle.json"
    if bundle.exists():
        f.x[:6] = json.loads(bundle.read_text())["mount"]
    poses = {"open": {}, "mid": REFS_A["mid"], "pinch": REFS_A["pinch"]}
    target_h = 150.0
    cands = []
    for d in np.arange(190.0, 291.0, 20.0):
        for h in np.arange(250.0, 351.0, 20.0):
            cands.append({"kind": "slide", "distance": float(d), "height": float(h), "tilt": NOW["tilt"]})
    for tilt in (0.0, 10.0, 20.0, 30.0):
        for h in (270.0, 350.0):
            cands.append({"kind": "reprint", "distance": NOW["distance"], "height": h, "tilt": tilt})
    cands.append({"kind": "now", **NOW})
    for h, off in ((350.0, 0.0), (450.0, 0.0), (400.0, 80.0)):
        cands.append({"kind": "top", "height": h, "offset": off})
    out = []
    for c in cands:
        if c["kind"] == "top":
            eye, d = top(c["height"], c["offset"])
        else:
            eye, d = eye_dir(c["distance"], c["height"], c["tilt"])
        cam = _roll(eye, eye + 300.0 * d, f.lens, 270.0)
        S = sensitivity(f, cam, poses, shape)
        out.append({**c, **score(S)})
        print(f"{time.time() - t0:5.0f}s {c} geo {out[-1]['geo_mean_sil']:.1f} in frame {out[-1]['in_frame']:.2f} weakest {out[-1]['weakest']}", flush=True)
    now = next(o for o in out if o["kind"] == "now")
    rel = []
    for o in out:
        r = {k: o[k] for k in ("kind", "distance", "height", "tilt", "offset") if k in o}
        r["gain_geo"] = round(o["geo_mean_sil"] / now["geo_mean_sil"], 3)
        r["gain_per_joint"] = (np.array(o["stops_sil"]) / np.maximum(now["stops_sil"], 1e-3)).round(2).tolist()
        rel.append(r)
    res = {"joints": JOINTS, "angles": ANGLES.tolist(), "stops": SPARSE_ANGLES, "delta_deg": DELTA,
           "fit_width": f.width, "poses": list(poses), "now": now, "candidates": out, "relative": rel,
           "note": "px/deg of silhouette change at the fit width (960); inner = depth-ordered inner outlines"}
    OUT.mkdir(exist_ok=True)
    (OUT / "camstudy.json").write_text(json.dumps(res, indent=1))
    with (OUT / "camstudy.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["kind", "distance", "height", "tilt", "offset", "joint", "stops_sil", "stops_inner", "best_angle", "gain_vs_now"])
        for o, r in zip(out, rel):
            for k, j in enumerate(JOINTS):
                w.writerow([o["kind"], o.get("distance"), o.get("height"), o.get("tilt"), o.get("offset"), j,
                            o["stops_sil"][k], o["stops_inner"][k], o["best_angle"][k], r["gain_per_joint"][k]])
    return res


if __name__ == "__main__":
    main()
