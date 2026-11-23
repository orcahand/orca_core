"""The quick visual recalibration of another hand (STUDY.md, ser-7577): its calibration archived,
a baseline pose that fixes the mount, the short block (4 points per joint + 6 random) and the
validation set, with no tactile, Kapandji or SAC. Plans span the hand's own config ROM
(STUDY_ROM_CONFIG, set before import).

    STUDY_ROM_CONFIG=<bench>/config.yaml python -m hand_study.night_quick TAG --run RUN --bench DIR
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from hand_study.configs import JOINTS
from hand_study.night_plan import S_PER_POSE, short, validation
from hand_study.plan import SPARSE_ANGLES, STUDY, Builder


def main(tag: str, run: str, bench: Path, motors: str, track_ma: float) -> dict:
    assert os.environ.get("STUDY_ROM_CONFIG"), "set STUDY_ROM_CONFIG to the hand's config.yaml"
    base = Builder()
    base.add("zero", {}, kind="repeat", ref="open", side="start", visit=0)
    blocks = [("N00-baseline", base, None), ("N07-short", short(), "pending"),
              ("N08-V", validation(wrist_lo=False), "pending")]
    night = {"tag": tag, "runs": {"A": run, "B": run}, "angles": SPARSE_ANGLES, "bench": str(bench),
             "motors": motors, "track_ma": track_ma, "rom_config": os.environ["STUDY_ROM_CONFIG"],
             "steps": [{"name": "snapshot", "kind": "snapshot", "run": run, "minutes": 1}], "sessions": []}
    for name, b, mount in blocks:
        sdir = STUDY / "sessions" / f"{tag}-{name}"
        sdir.mkdir(parents=True, exist_ok=True)
        (sdir / "plan.json").write_text(json.dumps(
            {"calibration": run, "part": name.split("-", 1)[1], "pilot": False, "angles": SPARSE_ANGLES,
             "mount": mount, "joints": JOINTS, "configs": b.out, "skipped": b.skipped, "night": tag, "block": name,
             "closed_loop": False}, indent=1))
        night["steps"].append({"name": name, "kind": "collect", "run": run, "session": str(sdir), "poses": len(b.out),
                               "minutes": round(len(b.out) * S_PER_POSE / 60)})
        night["sessions"].append(str(sdir))
        if name == "N00-baseline":
            night["steps"].append({"name": "mount", "kind": "mount", "run": run, "from": str(sdir), "minutes": 3})
    (STUDY / "night" / f"{tag}.json").write_text(json.dumps(night, indent=1))
    return night


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("tag")
    p.add_argument("--run", required=True)
    p.add_argument("--bench", type=Path, required=True)
    p.add_argument("--motors", default="feetech")
    p.add_argument("--track-ma", type=float, default=500.0)
    a = p.parse_args()
    n = main(a.tag, a.run, a.bench.expanduser(), a.motors, a.track_ma)
    for s in n["steps"]:
        print(f"{s['name']:14s} {s.get('poses', ''):>4} poses {s['minutes']:4d} min")
