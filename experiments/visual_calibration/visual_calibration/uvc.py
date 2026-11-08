"""The camera's UVC controls (focus, exposure, gain, white balance) through uvcc,
and the check a sweep runs before the motor moves: the controls as in
config.yaml camera.uvc, then one frame against the stored reference.

OpenCV on macOS cannot read or set these; uvcc (node, ``npx uvcc``) talks UVC
over USB. Units are uvcc's: exposure in 100 µs, auto_exposure_mode 1 = manual.
"""
from __future__ import annotations

import json
import os
import shlex
import subprocess
from collections.abc import Callable, Mapping

import numpy as np

TOOL = os.environ.get("ORCA_UVCC", "npx --yes uvcc@8.0.0")
AUTO_FLAGS = ("auto_focus", "auto_exposure_mode", "auto_exposure_priority", "auto_white_balance_temperature")
OVERRIDDEN_BY = {"absolute_focus": ("auto_focus", 0), "absolute_exposure_time": ("auto_exposure_mode", 1),
                 "white_balance_temperature": ("auto_white_balance_temperature", 0)}
CLIP_LEVEL = 250
FOCUS_APPROACH = 20            # the lens has hysteresis: always come to the focus from below


class CameraSettingsError(RuntimeError):
    pass


class Uvcc:
    def __init__(self, vendor: int, product: int, tool: str = TOOL, run: Callable = subprocess.run):
        self.vendor, self.product, self.tool, self.run = int(vendor), int(product), tool, run

    @classmethod
    def from_config(cls, uvc: Mapping, **kw) -> Uvcc:
        return cls(uvc["device"]["vendor"], uvc["device"]["product"], **kw)

    def _call(self, *args) -> str:
        cmd = [*shlex.split(self.tool), "--vendor", str(self.vendor), "--product", str(self.product), *map(str, args)]
        p = self.run(cmd, capture_output=True, text=True, timeout=60)
        if p.returncode:
            raise CameraSettingsError(f"uvcc {' '.join(map(str, args))}: {(p.stderr or p.stdout).strip()[-300:]}")
        return p.stdout

    def export(self) -> dict:
        return json.loads(self._call("export"))

    def set(self, control: str, value) -> None:
        self._call("set", control, *(value if isinstance(value, list) else [value]))


def wanted(uvc: Mapping) -> dict:
    """The controls to hold, auto flags first; values an auto flag overrides are left out."""
    want = {k: v for k, v in uvc.items() if k != "device"}
    want = {**{k: want[k] for k in AUTO_FLAGS if k in want}, **{k: v for k, v in want.items() if k not in AUTO_FLAGS}}
    return {k: v for k, v in want.items()
            if k not in OVERRIDDEN_BY or want.get(OVERRIDDEN_BY[k][0], OVERRIDDEN_BY[k][1]) == OVERRIDDEN_BY[k][1]}


def differences(want: Mapping, have: Mapping) -> dict:
    return {k: (v, have.get(k)) for k, v in want.items() if have.get(k) != v}


def ensure(cam: Uvcc, uvc: Mapping, write: bool = True) -> dict:
    """Read the controls; set the ones that differ (auto flags first); read them back.
    Raises before anything moves if they still differ (or differ at all with write=False)."""
    want = wanted(uvc)
    before = cam.export()
    diff = differences(want, before)
    if diff and write:
        for k in diff:
            if k == "absolute_focus":
                cam.set(k, max(0, want[k] - FOCUS_APPROACH))
            cam.set(k, want[k])
    after = cam.export() if diff and write else before
    left = differences(want, after)
    if left:
        raise CameraSettingsError("camera controls differ from config.yaml camera.uvc: "
                                  + ", ".join(f"{k} {h} (want {w})" for k, (w, h) in left.items()))
    return {"before": before, "set": {k: w for k, (w, _) in diff.items()} if write else {}, "after": after}


def frame_stats(gray: np.ndarray, detections, obj, letters: Mapping | None = None) -> dict:
    """Per tile: sharpness (Laplacian variance), scale (px per square), corners, clipped share;
    and the brightness over all tiles."""
    import cv2
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    sp = obj.spec
    lo, (w, h) = sp.margin, (sp.squares[0] * sp.square_size, sp.squares[1] * sp.square_size)
    region = np.array([[[lo, lo], [lo + w, lo], [lo + w, lo + h], [lo, lo + h]]], float)
    tiles, values = {}, []
    for d in detections:
        H, _ = cv2.findHomography(sp.corners[d.ids][:, :2], d.uv)
        if H is None:
            continue
        quad = cv2.perspectiveTransform(region, H)[0]
        poly = quad.astype(np.int32)
        mask = np.zeros(gray.shape, np.uint8)
        cv2.fillPoly(mask, [poly], 1)
        m = mask.astype(bool)
        px = gray[m]
        values.append(px)
        tiles[(letters or {}).get(d.tile, d.tile)] = {
            "sharpness": float(lap[m].var()), "scale": float(sp.square_size * np.sqrt(cv2.contourArea(quad.astype(np.float32)) / (w * h))),
            "corners": len(d.ids), "clipped": float((px >= CLIP_LEVEL).mean())}
    px = np.concatenate(values) if values else gray.ravel()
    return {"tiles": tiles, "brightness": float(px.mean()), "clipped": float((px >= CLIP_LEVEL).mean())}


def compare(stats: Mapping, ref: Mapping, tolerance: float = 0.25, max_clipped: float = 0.01) -> list[str]:
    """What is off against the reference frame (same stop): tiles missing, sharpness, scale
    (focus breathing), brightness, clipping."""
    problems = []
    if stats["clipped"] > max_clipped:
        problems.append(f"{100 * stats['clipped']:.1f}% of the tile pixels clipped (max {100 * max_clipped:g}%)")
    if abs(stats["brightness"] / ref["brightness"] - 1) > tolerance:
        problems.append(f"brightness {stats['brightness']:.0f}, reference {ref['brightness']:.0f}")
    for k, r in ref["tiles"].items():
        s = stats["tiles"].get(k)
        if s is None:
            problems.append(f"tile {k} not found")
            continue
        if s["sharpness"] < (1 - tolerance) * r["sharpness"]:
            problems.append(f"tile {k} sharpness {s['sharpness']:.0f}, reference {r['sharpness']:.0f}")
        if abs(s["scale"] / r["scale"] - 1) > 0.01:
            problems.append(f"tile {k} scale {s['scale']:.2f} px, reference {r['scale']:.2f} (focus moved?)")
    return problems
