"""Camera soak: one SharedCamera for N minutes, a still every few seconds, memory logged every 30 s.

    python -m visual_calibration.soak --minutes 45 [--index 0] [--size 3840x2160] [--log soak.log]

Logs our RSS and every VTDecoderXPCService / UVCAssistant footprint (top MEM; the leaking
decoder runs as _cmiodalassistants, so ps RSS undercounts it). Passes when, after a 5 min
warm-up, neither grew by more than --slack MB.
"""
from __future__ import annotations

import argparse
import re
import resource
import subprocess
import sys
import time

from visual_calibration.capture import SharedCamera

UNITS = {"B": 1 / 2**20, "K": 1 / 1024, "M": 1.0, "G": 1024.0, "T": 2**20}


def footprints() -> dict[str, float]:
    out = {}
    for name in ("VTDecoderXPCService", "UVCAssistant"):
        pids = subprocess.run(["pgrep", "-f", name], capture_output=True, text=True).stdout.split()
        for pid in pids:
            mem = subprocess.run(["top", "-l", "1", "-pid", pid, "-stats", "mem"], capture_output=True,
                                 text=True).stdout.strip().splitlines()[-1].strip().rstrip("+-")
            m = re.fullmatch(r"([\d.]+)([BKMGT])", mem)
            if m:
                out[f"{name[:3].lower()}:{pid}"] = float(m[1]) * UNITS[m[2]]
    return out


def rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 2**20     # macOS: bytes (peak)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--minutes", type=float, default=45)
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--size", default="3840x2160")
    p.add_argument("--every", type=float, default=3.0, help="s between stills")
    p.add_argument("--slack", type=float, default=50.0, help="MB growth allowed after warm-up")
    p.add_argument("--log", default="soak.log")
    a = p.parse_args()
    size = tuple(int(v) for v in a.size.split("x"))
    cam = SharedCamera.get(a.index, size)
    t0 = time.monotonic()
    rows, stills, next_log = [], 0, 0.0
    with open(a.log, "a") as log:
        while (t := time.monotonic() - t0) < a.minutes * 60:
            cam.still()
            stills += 1
            if t >= next_log:
                row = {"t_min": round(t / 60, 1), "ours": round(rss_mb(), 1), **footprints(), **cam.stats()}
                rows.append(row)
                print(row, file=log, flush=True)
                next_log += 30
            time.sleep(a.every)
    cam.close()
    warm = [r for r in rows if r["t_min"] >= 5] or rows
    keys = [k for k in warm[0] if k == "ours" or ":" in k]
    grown = {k: round(warm[-1].get(k, 0) - warm[0].get(k, 0), 1) for k in keys}
    ok = all(g <= a.slack for g in grown.values()) and cam.opens == 1
    print(f"{'PASS' if ok else 'FAIL'}: {stills} stills, opens={cam.opens}, growth after warm-up (MB) {grown}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
