"""Per-joint maps from the study for orca-ui's dashboard layer (orca_ui/hand/study_map.py):
command -> true angle (to control) and orca_core's estimate -> true angle (to display), each a
smooth monotone 1-D table over the joint's range. Joints whose vision noise is too high stay
uncorrected (low confidence).

    python -m hand_study.joint_map      # writes joint_map.json
"""

from __future__ import annotations

import json
import time

import numpy as np

from hand_study import analysis
from hand_study.configs import JOINTS, ROM
from hand_study.lut import JointTable

STUDY = analysis.STUDY
NOISE_MAX_DEG = 3.5          # median view-split noise above this: low confidence, not corrected
MIN_POSES = 8
KNOTS = 13
LOW_CONFIDENCE = {"thumb_dip", "thumb_mcp"}   # thumb poorly seen from the plate: shown, not corrected


def monotone(y: np.ndarray, increasing: bool) -> np.ndarray:
    return np.maximum.accumulate(y) if increasing else np.minimum.accumulate(y)


def fit_1d(j: str, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    t = JointTable(j, [], pair=False)
    t.fit({j: x}, y)
    k = np.linspace(*ROM[j], KNOTS)
    v = t.predict({j: k})
    return k, monotone(v, v[-1] >= v[0])


def main() -> dict:
    R = analysis.rows()
    vis = analysis.vision(R)["joints"]
    bundle = json.loads((STUDY / "mount_bundle.json").read_text()) if (STUDY / "mount_bundle.json").exists() else None
    out = {"created": time.strftime("%Y-%m-%d %H:%M"), "n_poses": len(R),
           "excluded": [x["id"] for x in analysis.EXCLUDED],
           "mount": bundle and bundle["mount"], "noise_max_deg": NOISE_MAX_DEG,
           "hysteresis": "not corrected: the same command reached from above or below differs by up to 3-7 deg",
           "joints": {}}
    for j in JOINTS:
        cmd = np.array([r["commanded"][j] for r in R])
        ass = np.array([r["assumed"][j] for r in R])
        true = np.array([r["measured"][j] for r in R])
        noise = vis[j]["split_med"]
        kc, vc = fit_1d(j, cmd, true)
        ka, va = fit_1d(j, ass, true)
        resid = true - np.interp(cmd, kc, vc)
        span = np.ptp(cmd) / (ROM[j][1] - ROM[j][0])
        good = (j not in LOW_CONFIDENCE and noise is not None and noise <= NOISE_MAX_DEG
                and len(R) >= MIN_POSES and span >= 0.5)
        out["joints"][j] = {
            "applied": bool(good),
            "confidence": "low" if not good else "good" if noise <= 1.5 else "fair",
            "noise_deg": noise, "n": len(R), "coverage": round(float(span), 2),
            "cmd_knots": kc.round(2).tolist(), "cmd_true": vc.round(2).tolist(),
            "assumed_knots": ka.round(2).tolist(), "assumed_true": va.round(2).tolist(),
            "reach": [round(float(min(vc[0], vc[-1])), 1), round(float(max(vc[0], vc[-1])), 1)],
            "rom": list(ROM[j]), "fit_rms_deg": round(float(np.sqrt(np.mean(resid ** 2))), 2),
            "offset_at_0": round(float(np.interp(0.0, kc, vc)), 2),
            "gain": round(float((vc[-1] - vc[0]) / (kc[-1] - kc[0])), 3),
        }
    path = STUDY / "joint_map.json"
    path.write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    o = main()
    for j, t in o["joints"].items():
        print(f"{j:12s} {t['confidence']:5s} applied={t['applied']!s:5s} noise {t['noise_deg']} gain {t['gain']} "
              f"0->{t['offset_at_0']} reach {t['reach']} rom {t['rom']} rms {t['fit_rms_deg']}")
