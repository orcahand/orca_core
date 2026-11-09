"""One visual-calibration run on disk, so it can be solved again later.

    <base>/<YYYYmmdd-HHMMSS>-<step>/
        session.yaml        step, rig, how the plate is turned, the photos
        photos/stop_<deg>.png       camera "a" (the single-camera names)
        photos/stop_<deg>_<cam>.png other cameras (photo entry `camera`; missing means "a")
        detections/<name>.json
        result.yaml         the last solve

Steps: "camera" (1a: lens + tiles, every photo free), "rotation" (1b: camera pose,
axis, stops, starting from the newest solved camera session next to it and
refining its lens and tiles with them: held fixed they cost ~3 px on 11 stops,
the free tiles of 1a absorb a little lens error), "hand" (photos only, so far).
A rotation session created with hold=True keeps the applied lenses and tiles (the quick
re-check after a camera was knocked: 5 stops, only the poses, axis and stops solved).
"""
from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from visual_calibration.calibration_object import CalibrationObject
from visual_calibration.detect import TileDetection, TileDetector, read_image
from visual_calibration.checks import solve_warnings
from visual_calibration.mounting import config_mountings, mounting_from_tiles
from visual_calibration.rig_calibration import (
    Photo,
    calibrate_camera,
    calibrate_rig,
    camera_block,
    camera_order,
    lens_from_calibration,
    plain,
    tile_poses_from_calibration,
)
from visual_calibration.rig_config import DEFAULT_CAMERA, DEFAULT_RIG_NAME, RigConfig

STEPS = ("camera", "rotation", "hand")
APPLIED = {"camera": ("lens", "tile_poses", "camera_fit", "mounting"),
           "rotation": ("lens", "tile_poses", "camera", "axis", "tower", "stop_angle_corrections", "fit", "mounting")}
MULTI = ("cameras",)            # applied when the result has them (several cameras)


class Session:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.meta = yaml.safe_load((self.root / "session.yaml").read_text())
        self._detector: TileDetector | None = None

    @classmethod
    def create(cls, base: Path, step: str = "camera", rig: str = DEFAULT_RIG_NAME, version: str | None = None,
               hand_turned: bool = False, hold: bool = False) -> Session:
        if step not in STEPS:
            raise ValueError(f"step must be one of {STEPS}")
        config = RigConfig.load(rig, version)
        root = Path(base) / f"{datetime.now():%Y%m%d-%H%M%S}-{step}"
        (root / "photos").mkdir(parents=True)
        (root / "detections").mkdir()
        meta = {"step": step, "rig": config.name, "rig_version": config.version, "hand_turned": hand_turned,
                "created": datetime.now().isoformat(timespec="seconds"), "image_size": None, "photos": []}
        if hold:
            meta["hold"] = ["lens", "tile_poses"]
        (root / "session.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
        return cls(root)

    @classmethod
    def list(cls, base: Path) -> list[Session]:
        base = Path(base)
        return [cls(p) for p in sorted(base.iterdir(), reverse=True) if (p / "session.yaml").is_file()] if base.is_dir() else []

    @property
    def id(self) -> str:
        return self.root.name

    @property
    def rig(self) -> RigConfig:
        return RigConfig.load(self.meta["rig"], self.meta["rig_version"])

    @property
    def object(self) -> CalibrationObject:
        return CalibrationObject.from_config(self.rig.raw["calibration_object"])

    @property
    def detector(self) -> TileDetector:
        if self._detector is None:
            self._detector = TileDetector(self.object)
        return self._detector

    def _save(self) -> None:
        (self.root / "session.yaml").write_text(yaml.safe_dump(self.meta, sort_keys=False))

    def add_photo(self, data: bytes, stop_deg: float, ext: str = "png",
                  plate_deg: float | None = None, camera: str = DEFAULT_CAMERA) -> tuple[dict, list[TileDetection]]:
        """Store a photo taken at ``stop_deg`` by ``camera`` (replacing one at the same stop) and detect
        its tiles. ``plate_deg``: the motor's read-back angle when the photo was taken; the solves use it
        instead of the nominal stop (the plate stops a little short or past)."""
        image = read_image(data)
        size = [int(image.shape[1]), int(image.shape[0])]
        if self.meta["image_size"] and self.meta["image_size"] != size:
            raise ValueError(f"photo is {size[0]}x{size[1]}, the session's are {self.meta['image_size']}")
        name = f"stop_{stop_deg:07.2f}" + ("" if camera == DEFAULT_CAMERA else f"_{camera}")
        (self.root / "photos" / f"{name}.{ext}").write_bytes(data)
        detections = self.detector.detect(image)
        (self.root / "detections" / f"{name}.json").write_text(json.dumps(
            {d.tile: {"ids": d.ids.tolist(), "uv": d.uv.round(3).tolist(), "markers": d.markers} for d in detections}))
        entry = {"name": name, "file": f"photos/{name}.{ext}", "stop_deg": float(stop_deg),
                 "plate_deg": None if plate_deg is None else round(float(plate_deg), 3),
                 "captured_at": datetime.now().isoformat(timespec="seconds"),
                 "tiles": len(detections), "corners": int(sum(len(d.ids) for d in detections))}
        if camera != DEFAULT_CAMERA:
            entry["camera"] = camera
        self.meta["image_size"] = size
        self.meta["photos"] = [p for p in self.meta["photos"] if p["name"] != name] + [entry]
        self.meta["photos"].sort(key=lambda p: (p["stop_deg"], p.get("camera", DEFAULT_CAMERA)))
        self._save()
        return entry, detections

    def detections(self, name: str) -> list[TileDetection]:
        raw = json.loads((self.root / "detections" / f"{name}.json").read_text())
        return [TileDetection(t, np.array(d["ids"], int), np.array(d["uv"], float), d["markers"]) for t, d in raw.items()]

    def photos(self) -> list[Photo]:
        return [Photo(p["stop_deg"] if p.get("plate_deg") is None else p["plate_deg"], self.detections(p["name"]), p["name"],
                      p.get("camera", DEFAULT_CAMERA))
                for p in self.meta["photos"]]

    @property
    def cameras(self) -> list[str]:
        return camera_order(self.photos())

    def solve(self):
        step, rig = self.meta["step"], self.rig
        size = tuple(self.meta["image_size"] or ())
        loss = float(rig.raw["solver"].get("loss_scale_px", 1.0))
        mountings = config_mountings(rig.raw["calibration_object"])
        if step == "camera":
            result = calibrate_camera(self.object, self.photos(), size, search=rig.search, loss_scale_px=loss)
            out = result.to_calibration()
            out["camera_fit"] = out.pop("fit")
            out["mounting"] = mounting_from_tiles(self.object, self.photos(), mountings=mountings).as_dict()
        elif step == "rotation":
            hold = bool(self.meta.get("hold"))
            cam = None if hold else self.camera_session()
            if cam is None and not hold:
                raise ValueError("solve the camera step first")
            c = rig.load_calibration() if hold else cam.result()
            cams, search = self.cameras, rig.search["camera"]
            priors = {}
            for k in cams:
                nominal = rig.nominal_of(k) or {}
                if nominal.get("position"):
                    priors[k] = {**nominal, "bound_mm": max(search["translation"]["bound"]),
                                 "sigma_mm": search["translation"]["sigma"][0], "bound_deg": max(search["rotation"]["bound"])}
            lenses = {k: lens_from_calibration(c, k) for k in cams if camera_block(c, k).get("lens")}
            if hold and set(lenses) != set(cams):
                raise ValueError(f"hold: no applied lens for camera(s) {sorted(set(cams) - set(lenses))}")
            result = calibrate_rig(self.object, self.photos(), size, search=rig.search, lenses=lenses,
                                   camera_priors=priors, hand_turned=self.meta["hand_turned"], loss_scale_px=loss,
                                   mountings=mountings, fixed_lens=hold,
                                   tile_poses=tile_poses_from_calibration(c) if hold else None)
            out = result.to_calibration()
            if cam is not None:
                out["camera_session"] = cam.id
            else:
                out["held"] = self.meta["hold"]
        else:
            raise NotImplementedError("the hand step cannot be solved yet")
        out["per_photo_px"] = {k: round(v, 3) for k, v in result.photo_rms.items()}
        out["per_tile_px"] = {k: round(v, 3) for k, v in result.tile_rms.items()}
        out["std"] = result.std
        photos = self.photos()
        out["photos"], out["corners"] = len(photos), int(sum(len(d.ids) for p in photos for d in p.detections))
        out["warnings"] = solve_warnings(rig, step, out, photos)
        (self.root / "result.yaml").write_text(yaml.safe_dump(plain(out), sort_keys=False))
        return result

    def camera_session(self) -> Session | None:
        """The newest solved camera session next to this one."""
        return next((s for s in Session.list(self.root.parent)
                     if s.meta["step"] == "camera" and s.result() is not None), None)

    def result(self) -> dict | None:
        path = self.root / "result.yaml"
        return yaml.safe_load(path.read_text()) if path.is_file() else None

    def apply(self) -> str:
        """Write the last solve into the rig's calibration.yaml (the old one goes to history)."""
        result = self.result()
        if result is None:
            raise ValueError("solve first")
        rig = self.rig
        old = {k: v for k, v in rig.load_calibration().items() if k != "history"}
        keep = {k: v for k, v in result.items() if k in APPLIED.get(self.meta["step"], ()) + MULTI}
        if "cameras" in keep:            # per camera: a camera step's lens keeps the rotation's pose
            keep["cameras"] = {c: {**(old.get("cameras") or {}).get(c, {}), **v} for c, v in keep["cameras"].items()}
            keep["cameras"] = {**(old.get("cameras") or {}), **keep["cameras"]}
        rig.save_calibration({**old, **keep, "calibrated_at": result["calibrated_at"], f"{self.meta['step']}_session": self.id})
        return rig.calibration_path

    def summary(self) -> dict:
        return {"id": self.id, **self.meta, "result": self.result()}
