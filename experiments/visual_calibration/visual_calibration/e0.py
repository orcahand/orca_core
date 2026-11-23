"""Experiment E0 (README.md): can we get the index finger's joint
angles back from synthetic fingertip pixels?

The URDF hand stands on the plate, the BRIO (portrait, nominal lens) looks at
it from 0.38 m, 20° down. True angles -> project the chosen points at the
chosen stops -> add pixel noise -> fit from a start up to ±10° off (the motor
angles) -> compare. Occlusion is ignored: every point counts as seen.

    uv run python -m visual_calibration.e0 [--out ~/Downloads/visual_calibration/e0]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from orca_core.kinematics.transforms import Transform, rotation_about_axis

from visual_calibration.camera import Camera, brio_portrait, look_at
from visual_calibration.fit import Observation, fit_joints
from visual_calibration.hand_model import HandModel
from visual_calibration.rig import Rig

DISTANCE, TILT, TARGET_HEIGHT = 0.38, 20.0, 0.22
ALL_STOPS = [15.0 * k for k in range(22)]
POSES = {
    "straight": {"index_abd": 0.0, "index_mcp": 0.0, "index_pip": 0.0},
    "half": {"index_abd": 5.0, "index_mcp": 30.0, "index_pip": 45.0},
    "curled": {"index_abd": -5.0, "index_mcp": 60.0, "index_pip": 90.0},
}
FLEX = ["index_mcp", "index_pip"]
ALL3 = ["index_abd", "index_mcp", "index_pip"]


def nominal_rig() -> Rig:
    t = np.deg2rad(TILT)
    target = np.array([0.0, 0.0, TARGET_HEIGHT])
    eye = target + DISTANCE * np.array([0.0, -np.cos(t), np.sin(t)])
    return Rig(Camera(brio_portrait(), look_at(eye, target)))


def side_on_stop(rig: Rig, model: HandModel) -> float:
    """The stop where the PIP axis points most along the camera's view: the
    finger flexes in the image plane."""
    poses = model.link_poses({})
    axis = poses["index_pp"].rotation @ model.joints["index_pip"].axis
    view = rig.camera.pose.rotation[:, 2]
    return max(ALL_STOPS, key=lambda s: abs(view @ (rig.hand_in_rig(s).rotation @ axis)))


def points(model: HandModel, features: str) -> list[tuple[str, np.ndarray]]:
    tip = model.fingertip("index")
    return [tip] if features == "tip" else [tip, ("index_dp", np.zeros(3))]   # + the PIP joint centre


def observe(rig, model, q, stops, pts, noise_px, rng) -> list[Observation]:
    obs = []
    poses = model.link_poses(q)
    w, h = rig.camera.lens.size
    for s in stops:
        uv = rig.project_poses(poses, s, pts)
        for (link, p), px in zip(pts, uv):
            if 0 <= px[0] < w and 0 <= px[1] < h:
                obs.append(Observation(s, link, p, px + rng.normal(0.0, noise_px, 2)))
    return obs


def trial_errors(rig, true_rig, model, q_true, joints, stops, pts, noise_px, trials, rng):
    errs, stds, rms = [], [], []
    for _ in range(trials):
        obs = observe(true_rig, model, q_true, stops, pts, noise_px, rng)
        start = {j: q_true[j] + rng.uniform(-10.0, 10.0) for j in joints}
        q, fit = fit_joints(rig, model, obs, joints, start, q_fixed=q_true)
        errs.append([q[j] - q_true[j] for j in joints])
        stds.append(fit.std)
        rms.append(fit.rms)
    errs = np.array(errs)
    return {
        "rms_deg": dict(zip(joints, np.sqrt((errs ** 2).mean(0)).round(3).tolist())),
        "max_deg": dict(zip(joints, np.abs(errs).max(0).round(3).tolist())),
        "mean_deg": dict(zip(joints, errs.mean(0).round(3).tolist())),
        "fit_std_deg": dict(zip(joints, np.median(stds, 0).round(3).tolist())),
        "fit_rms_px": round(float(np.median(rms)), 3),
        "wrong_over_2deg": int((np.abs(errs).max(1) > 2.0).sum()),
    }


def perturbed(rig: Rig, mm: float, deg: float) -> Rig:
    """The same rig with the camera off by ``mm`` along each axis and ``deg`` about each."""
    a = np.deg2rad(deg)
    r = rotation_about_axis(np.array([1.0, 1.0, 1.0]), np.sqrt(3) * a)
    delta = Transform.from_rotation_translation(r, np.full(3, mm / 1000))
    return Rig(Camera(rig.camera.lens, rig.camera.pose @ delta), rig.hand_mount)


def run(trials: int = 100, seed: int = 0) -> dict:
    model, rig = HandModel.load("right"), nominal_rig()
    rng = np.random.default_rng(seed)
    side = side_on_stop(rig, model)
    eight = [45.0 * k for k in range(8)]
    cases = [
        ("A", "1 stop, side-on", [side], "tip", FLEX),
        ("B", "8 stops", eight, "tip", FLEX),
        ("C", "8 stops", eight, "tip", ALL3),
        ("D", "22 stops", ALL_STOPS, "tip", ALL3),
        ("E", "8 stops", eight, "tip+pip", ALL3),
    ]
    out = {"setup": {"camera_distance_m": DISTANCE, "camera_tilt_deg": TILT, "lens": "BRIO portrait 2160x3840, 65° diag",
                     "mm_per_px": round(DISTANCE / rig.camera.lens.fx * 1000, 3), "side_on_stop_deg": side,
                     "trials": trials, "start_error_deg": 10.0, "urdf": str(model.urdf)},
           "cases": []}
    for key, label, stops, feats, joints in cases:
        for noise in (0.5, 2.0):
            for name, q in POSES.items():
                r = trial_errors(rig, rig, model, q, joints, stops, points(model, feats), noise, trials, rng)
                out["cases"].append({"case": key, "views": label, "points": feats, "joints": joints,
                                     "noise_px": noise, "pose": name, **r})
    # Rig error not fitted: the camera 0.5 mm / 0.1° off, no pixel noise.
    true_rig = perturbed(rig, 0.5, 0.1)
    for key, label, stops, feats, joints in [cases[1], cases[2]]:
        for name, q in POSES.items():
            r = trial_errors(rig, true_rig, model, q, joints, stops, points(model, feats), 0.0, 5, rng)
            out["cases"].append({"case": key + "-rig", "views": label, "points": feats, "joints": joints,
                                 "noise_px": 0.0, "camera_error": "0.5 mm, 0.1°", "pose": name, **r})
    out["overlay"] = overlay_svg(rig, model)
    return out


def overlay_svg(rig: Rig, model: HandModel, stops=(0.0, 90.0, 180.0, 270.0), scale=0.1) -> str:
    """The hand's mesh points (every 15th vertex) as the camera sees them, half pose."""
    w, h = (np.array(rig.camera.lens.size) * scale).astype(int)
    q = POSES["half"]
    poses = model.link_poses(q)
    tip_link, tip = model.fingertip("index")
    parts = [(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w * len(stops)}" height="{h + 20}" '
              f'style="background:#fff;font:12px sans-serif">')]
    for i, s in enumerate(stops):
        ox = i * w
        parts.append(f'<rect x="{ox}" y="0" width="{w}" height="{h}" fill="#f4f4f4" stroke="#999"/>')
        for link in model.visuals:
            pts = rig.project_poses(poses, s, [(link, p) for p in model.mesh(link)[::15]]) * scale
            color = "#d33" if link.startswith("index") else "#555"
            parts += [f'<circle cx="{ox + u:.1f}" cy="{v:.1f}" r="0.6" fill="{color}"/>' for u, v in pts]
        u, v = rig.project_poses(poses, s, [(tip_link, tip)])[0] * scale
        parts.append(f'<circle cx="{ox + u:.1f}" cy="{v:.1f}" r="4" fill="none" stroke="#06c" stroke-width="1.5"/>')
        parts.append(f'<text x="{ox + 6}" y="{h + 15}">stop {s:g}°</text>')
    parts.append("</svg>")
    return "\n".join(parts)


def report(result: dict) -> str:
    s = result["setup"]
    lines = [
        "# E0: index finger from synthetic fingertip pixels", "",
        (f"BRIO portrait at {s['camera_distance_m']} m, {s['camera_tilt_deg']}° down, "
         f"{s['mm_per_px']} mm/px at the axis. {s['trials']} trials per row, start up to "
         f"±{s['start_error_deg']}° off. Side-on stop: {s['side_on_stop_deg']:g}°. Occlusion ignored."),
        f"Model: `{s['urdf']}`.", "",
        ("Cases: A 1 stop side-on, tip, mcp+pip · B 8 stops, tip, mcp+pip · C 8 stops, tip, abd+mcp+pip · "
         "D 22 stops, tip, abd+mcp+pip · E 8 stops, tip + PIP centre, abd+mcp+pip · *-rig: camera 0.5 mm/0.1° off, "
         "not fitted, no noise."), "",
        "| case | noise px | pose | RMS error ° (abd / mcp / pip) | max ° | >2° | fit px |",
        "|---|---|---|---|---|---|---|",
    ]
    def fmt(d):
        return " / ".join(f"{d[j]:.2f}" if j in d else "–" for j in ALL3)

    for c in result["cases"]:
        lines.append(f"| {c['case']} | {c['noise_px']} | {c['pose']} | {fmt(c['rms_deg'])} | {fmt(c['max_deg'])} "
                     f"| {c['wrong_over_2deg']} | {c['fit_rms_px']} |")
    lines += ["", "![views](overlay.svg)", ""]
    return "\n".join(lines)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, default=Path.home() / "Downloads" / "visual_calibration" / "e0")
    p.add_argument("--trials", type=int, default=100)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    result = run(a.trials)
    (a.out / "overlay.svg").write_text(result.pop("overlay"))
    (a.out / "results.json").write_text(json.dumps(result, indent=1))
    (a.out / "report.md").write_text(report(result))
    print(report(result))


if __name__ == "__main__":
    main()
