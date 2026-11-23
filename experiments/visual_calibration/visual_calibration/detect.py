"""Find the calibration object's tiles in a photo (needs OpenCV, see README).

Markers are found once per image; each tile's ChArUco corners are then
interpolated from its own markers only.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from visual_calibration.calibration_object import CalibrationObject


def _cv2():
    try:
        import cv2
    except ImportError as e:  # pragma: no cover - depends on the install
        raise ImportError("tile detection needs OpenCV: uv run --with opencv-contrib-python-headless ...") from e
    return cv2


@dataclass(frozen=True)
class TileDetection:
    tile: str
    ids: np.ndarray            # ChArUco corner ids, (n,)
    uv: np.ndarray             # pixels, (n, 2)
    markers: int               # markers of this tile found


class TileDetector:
    def __init__(self, obj: CalibrationObject, min_corners: int = 4):
        cv2 = _cv2()
        self.obj, self.min_corners = obj, min_corners
        s = obj.spec
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, s.dictionary))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.markers = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.boards = {
            name: cv2.aruco.CharucoDetector(cv2.aruco.CharucoBoard(
                s.squares, s.square_size, s.marker_size, self.dictionary, t.marker_ids(s)))
            for name, t in obj.tiles.items()}

    def detect(self, image: np.ndarray) -> list[TileDetection]:
        cv2 = _cv2()
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.markers.detectMarkers(gray)
        if ids is None:
            return []
        ids = ids.ravel()
        found = []
        for name, det in self.boards.items():
            t = self.obj.tiles[name]
            mine = (ids >= t.first_marker) & (ids < t.first_marker + self.obj.spec.markers_per_tile)
            if mine.sum() < 2:
                continue
            mc = tuple(c for c, m in zip(corners, mine) if m)
            cc, ci, _, _ = det.detectBoard(gray, markerCorners=mc, markerIds=ids[mine].reshape(-1, 1))
            if ci is not None and len(ci) >= self.min_corners:
                found.append(TileDetection(name, ci.ravel().astype(int), cc.reshape(-1, 2).astype(float), int(mine.sum())))
        return found


def read_image(data: bytes) -> np.ndarray:
    """Decode an encoded image (PNG/JPEG bytes) to grey."""
    cv2 = _cv2()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("not an image")
    return img


def display_rotation(obj: CalibrationObject, detections: list[TileDetection]) -> dict | None:
    """Which clockwise display turn (0/90/180/270) puts the tower's up at the top of the
    screen, from each tile's up direction in the raw frame (its corner grid). None
    without tiles; `agreement` 0..1 is how well the tiles agree."""
    ups, w = [], []
    for d in detections:
        if len(d.ids) < 4:
            continue
        xy = obj.spec.corners[d.ids][:, :2]
        A = np.linalg.lstsq(np.column_stack([xy, np.ones(len(xy))]), d.uv, rcond=None)[0][:2].T
        up = A @ [0.0, -1.0]                         # tile frame: y runs down the tile
        if np.linalg.norm(up) > 0:
            ups.append(up / np.linalg.norm(up))
            w.append(len(d.ids))
    if not ups:
        return None
    up = np.average(ups, axis=0, weights=w)

    def screen_up(r):                                # turned clockwise on screen (y down)
        a = np.radians(r)
        return -(np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ up)[1]

    best = max((0, 90, 180, 270), key=screen_up)
    return {"rotation": best, "agreement": round(float(np.linalg.norm(up)), 3),
            "up_raw": np.round(up / max(np.linalg.norm(up), 1e-12), 3).tolist()}
