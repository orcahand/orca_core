"""The study's progress and report page: http://127.0.0.1:5010

    python -m hand_study.server [--port 5010]
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from hand_study import analysis
from hand_study.analysis import STUDY

WEB = Path(__file__).parent / "web"
CACHE_S = 20.0
GREEN, ORANGE = (60, 190, 40), (52, 104, 235)
TILE_H = 260
TILE_STOPS = (112, 292)
SKIP = ("forearm", "tower")


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.report: tuple[float, str] = (0.0, "")
        self.fit = None
        self.extra: dict[str, str] = {}

    def report_json(self) -> str:
        with self.lock:
            if time.time() - self.report[0] > CACHE_S:
                data = analysis.report()
                for name in ("lut.json",):
                    p = STUDY / name
                    if p.exists():
                        data[name.split(".")[0]] = json.loads(p.read_text())
                self.report = (time.time(), json.dumps(data, allow_nan=False, default=_nan))
            return self.report[1]

    def ensure_fit(self):
        if self.fit is None:
            from hand_study.estimate import StudyFit
            self.fit = StudyFit()
        return self.fit

    def overlay(self, d: Path, stop: int) -> bytes:
        import cv2
        return cv2.imencode(".jpg", self.overlay_img(d, stop)[0], [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()

    def overlay_img(self, d: Path, stop: int):
        """The photo at the fit's width with the URDF outlines (solved green, assumed orange), and
        the box (x0, y0, x1, y1) around the drawn hand."""
        import cv2
        meta = json.loads((d / "meta.json").read_text())
        v = min(meta["views"], key=lambda v: abs(v["stop"] - stop))
        img = cv2.imread(str(d / v["file"]))
        box = [img.shape[1], img.shape[0], 0, 0]
        with self.lock:
            self.ensure_fit()
            f = self.fit
            img = cv2.resize(img, (f.width, round(img.shape[0] * f.width / img.shape[1])), interpolation=cv2.INTER_AREA)
            est_p = d / "est.json"
            if est_p.exists():
                est = json.loads(est_p.read_text())
                mount = np.array(est["mount"])
                for q, col in ((est["assumed"], ORANGE), (est["joints"], GREEN)):
                    x = np.r_[mount, [q.get(j, 0.0) for j in f.joints]]
                    poses = {l: T for l, T in f.cam_poses(x, float(v["angle"])).items() if l not in SKIP}
                    for _, _, u0, v0, m in f.parts(poses, f.lens_s, img.shape[:2]):
                        box = [min(box[0], u0), min(box[1], v0), max(box[2], u0 + m.shape[1]), max(box[3], v0 + m.shape[0])]
                        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
                        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(img, cs, -1, col, 2, cv2.LINE_AA, offset=(int(u0), int(v0)))
        return img, box

    def inputs(self, d: Path, stop: int):
        """The solver's inputs at one view, full fit-width images: mask (0/1), edges (0/1), and
        the model silhouette at the solution; from views.npz, else re-made at the solved pose."""
        meta = json.loads((d / "meta.json").read_text())
        v = min(meta["views"], key=lambda v: abs(v["stop"] - stop))
        est = json.loads((d / "est.json").read_text())
        f = self.ensure_fit()
        h, w = round(f.lens.size[1] * f.s), f.width
        x = np.array(est["x"])
        remade = False
        npz = d / "views.npz"
        if npz.exists():
            z = np.load(npz)
            k = int(np.argmin(np.abs(np.array(z["angle"]) - v["angle"])))
            fg_c, e_c, off = z[f"fg{k}"], z[f"edges{k}"], z["offset"][k]
        else:
            import cv2
            img = cv2.imread(str(d / v["file"]))
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            f.x = x.copy()
            view = f.prepare(img, float(v["angle"]), f.lens_s)
            fg_c, e_c, off, remade = view.fg > 0, view.edges > 0, view.offset, True
        fg, edges = np.zeros((h, w), bool), np.zeros((h, w), bool)
        u0, v0 = int(off[0]), int(off[1])
        fg[v0:v0 + fg_c.shape[0], u0:u0 + fg_c.shape[1]] = fg_c
        edges[v0:v0 + e_c.shape[0], u0:u0 + e_c.shape[1]] = e_c
        model = f.label(x, float(v["angle"]), f.lens_s, (h, w)) > 0
        return fg, edges, model, remade

    def tile(self, d: Path):
        """Per side view, three panels cropped around the hand: the photo with the outlines
        (solved green, assumed orange), the mask the solver used (cyan) with its edges (yellow),
        and mask vs. model at the solution (red: mask only, blue: model only, grey: both)."""
        import cv2
        est_p, cache = d / "est.json", d / "tile.jpg"
        if cache.exists() and est_p.exists() and cache.stat().st_mtime >= est_p.stat().st_mtime:
            return cv2.imread(str(cache))
        rows, remade = [], False
        for stop in TILE_STOPS:
            img, (x0, y0, x1, y1) = self.overlay_img(d, stop)
            pad = 20
            x0, y0 = max(0, x0 - pad), max(0, y0 - pad)
            x1, y1 = min(img.shape[1], x1 + pad), min(img.shape[0], y1 + pad)
            panels = [img]
            if est_p.exists():
                with self.lock:
                    fg, edges, model, rm = self.inputs(d, stop)
                remade |= rm
                raw = cv2.resize(cv2.imread(str(d / min(json.loads((d / "meta.json").read_text())["views"],
                                                         key=lambda v: abs(v["stop"] - stop))["file"])),
                                 (img.shape[1], img.shape[0]), interpolation=cv2.INTER_AREA)
                m = (raw * 0.35).astype(np.uint8)
                m[fg] = (0.45 * m[fg] + 0.55 * np.array([230, 200, 40])).astype(np.uint8)
                m[edges] = (0, 220, 255)
                dis = np.full(img.shape, 30, np.uint8)
                dis[fg & model] = (150, 150, 150)
                dis[fg & ~model] = (60, 60, 230)
                dis[model & ~fg] = (230, 120, 40)
                panels += [m, dis]
            crops = [p[y0:y1, x0:x1] for p in panels]
            crops = [cv2.resize(c, (round(c.shape[1] * TILE_H / c.shape[0]), TILE_H), interpolation=cv2.INTER_AREA)
                     for c in crops]
            rows.append(np.hstack([np.pad(c, ((0, 0), (0, 4), (0, 0)), constant_values=255) for c in crops]))
        w = max(r.shape[1] for r in rows)
        body = np.vstack([np.pad(r, ((0, 4), (0, w - r.shape[1]), (0, 0)), constant_values=255) for r in rows])
        lines = [d.name]
        color = (20, 20, 20)
        if est_p.exists():
            est = json.loads(est_p.read_text())
            lines[0] += f"   fit {est['rms_px']:.1f} px" + ("   mask re-made" if remade else "")
            cmd = json.loads((d / "meta.json").read_text())["q"]
            moved = sorted((j for j in est["joints"] if abs(cmd.get(j, 0)) > 0.5),
                           key=lambda j: -abs(est["joints"][j] - est["assumed"][j]))[:4]
            lines.append("  ".join(f"{j} {est['assumed'][j]:.0f}>{est['joints'][j]:.0f}" for j in moved) or "all at 0")
            if est.get("suspect"):
                lines.append("SUSPECT MASK: " + "; ".join(est["suspect"])[:150])
                color = (30, 30, 200)
        else:
            lines.append("not solved yet")
        head = np.full((20 * len(lines) + 6, body.shape[1], 3), 255, np.uint8)
        for k, t in enumerate(lines):
            cv2.putText(head, t, (6, 16 + 20 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5 if k == 0 else 0.42,
                        color if k != 1 else (80, 80, 80), 1, cv2.LINE_AA)
        out = np.vstack([head, body])
        if est_p.exists():
            cv2.imwrite(str(cache), out, [cv2.IMWRITE_JPEG_QUALITY, 88])
        return out

    def contact(self, session: str, cols: int = 2):
        """Every solved pose of a session as a tile, in a grid."""
        cfg = STUDY / "sessions" / session / "configs"
        tiles = [self.tile(d) for d in sorted(cfg.glob("*")) if (d / "est.json").exists()]
        if not tiles:
            return np.full((60, 400, 3), 255, np.uint8)
        w, h = max(t.shape[1] for t in tiles), max(t.shape[0] for t in tiles)
        tiles = [np.pad(t, ((0, h - t.shape[0] + 6), (0, w - t.shape[1] + 6), (0, 0)), constant_values=255) for t in tiles]
        rows = [np.hstack(tiles[i:i + cols] + [np.full_like(tiles[0], 255)] * (cols - len(tiles[i:i + cols])))
                for i in range(0, len(tiles), cols)]
        legend = np.full((30, rows[0].shape[1], 3), 255, np.uint8)
        import cv2
        cv2.putText(legend, f"{session} - per pose, plate {TILE_STOPS[0]} and {TILE_STOPS[1]} deg: photo with outlines (green solved, orange assumed by orca_core) | "
                    "mask used (cyan) + edges (yellow) | mask vs model (grey both, red mask only, blue model only). Labels: joint assumed>solved deg",
                    (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
        return np.vstack([legend, *rows])


def _nan(o):
    return None


def _clean(o):
    if isinstance(o, float):
        return o if np.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, list):
        return [_clean(v) for v in o]
    return o


def progress() -> dict:
    out = {"sessions": [], "t": time.time()}
    for s in sorted((STUDY / "sessions").glob("*")):
        if not (s / "plan.json").exists():
            continue
        plan = json.loads((s / "plan.json").read_text())
        cfg = s / "configs"
        collected = [d.name for d in sorted(cfg.glob("*")) if (d / "meta.json").exists()] if cfg.exists() else []
        solved = [d.name for d in sorted(cfg.glob("*"), key=lambda d: (d / "est.json").stat().st_mtime
                                         if (d / "est.json").exists() else 0) if (d / "est.json").exists()] \
            if cfg.exists() else []
        errs = [d.name for d in cfg.glob("*") if (d / "est.error").exists()] if cfg.exists() else []
        prog = json.loads((s / "progress.json").read_text()) if (s / "progress.json").exists() else {}
        out["sessions"].append({"name": s.name, "run": plan.get("calibration"), "part": plan.get("part"),
                                "planned": len(plan["configs"]), "collected": len(collected), "solved": len(solved),
                                "errors": errs, "progress": prog, "latest": solved[-1] if solved else None,
                                "latest_collected": collected[-1] if collected else None,
                                "angles": plan.get("angles")})
    for key, path in (("hand", "/status"), ("turntable", "/visual-calibration/turntable")):
        try:
            d = json.load(urllib.request.urlopen("http://127.0.0.1:5001/api" + path, timeout=2))
            out[key] = {"torque": d.get("torque_enabled", d.get("torque")), "state": d.get("state"),
                        "angle": d.get("angle"), "busy": d.get("busy")}
        except Exception as e:
            out[key] = {"error": str(e)}
    return out


def config_detail(session: str, cid: str) -> dict:
    d = STUDY / "sessions" / session / "configs" / cid
    meta = json.loads((d / "meta.json").read_text())
    est = json.loads((d / "est.json").read_text()) if (d / "est.json").exists() else None
    return {"session": session, "id": cid, "q": meta["q"], "kind": meta.get("kind"),
            "views": [{"stop": int(round(v["stop"])), "angle": v["angle"]} for v in meta["views"]],
            "est": est and {k: est[k] for k in ("joints", "std", "assumed", "rms_px", "view_rms_px", "split", "spread",
                                               "solve_s") if k in est}}


STATE = State()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, body: bytes, ctype: str, code: int = 200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        p = self.path.split("?")[0]
        try:
            if p in ("/", "/index.html"):
                return self.send((WEB / "index.html").read_bytes(), "text/html; charset=utf-8")
            if p.startswith("/web/"):
                f = WEB / p[5:]
                ctype = "text/javascript" if f.suffix == ".js" else "text/css" if f.suffix == ".css" else "text/plain"
                return self.send(f.read_bytes(), ctype)
            if p == "/api/progress":
                return self.send(json.dumps(_clean(progress())).encode(), "application/json")
            if p == "/api/report":
                return self.send(STATE.report_json().encode(), "application/json")
            if p.startswith("/api/config/"):
                _, _, _, session, cid = p.split("/")
                return self.send(json.dumps(_clean(config_detail(session, cid))).encode(), "application/json")
            if p.startswith("/tile/"):
                import cv2
                _, _, session, cid = p.split("/")
                img = STATE.tile(STUDY / "sessions" / session / "configs" / cid.removesuffix(".jpg"))
                return self.send(cv2.imencode(".jpg", img)[1].tobytes(), "image/jpeg")
            if p.startswith("/contact/"):
                import cv2
                img = STATE.contact(p.split("/")[2].removesuffix(".jpg"))
                return self.send(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes(), "image/jpeg")
            if p == "/api/night":
                f = STUDY / "report" / "results.json"
                return self.send(f.read_bytes() if f.exists() else b"null", "application/json")
            if p.startswith("/report/") and ".." not in p:
                f = STUDY / p.lstrip("/")
                ctype = {".jpg": "image/jpeg", ".csv": "text/csv", ".json": "application/json"}.get(f.suffix, "text/plain")
                return self.send(f.read_bytes(), ctype)
            if p.startswith("/api/solved/"):
                session = p.split("/")[3]
                cfg = STUDY / "sessions" / session / "configs"
                ids, sus = [], {}
                for d in sorted(cfg.glob("*")):
                    if (d / "est.json").exists():
                        ids.append(d.name)
                        why = json.loads((d / "est.json").read_text()).get("suspect")
                        if why:
                            sus[d.name] = why
                return self.send(json.dumps({"session": session, "ids": ids, "suspect": sus}).encode(), "application/json")
            if p.startswith("/overlay/"):
                _, _, session, cid, stop = p.split("/")
                d = STUDY / "sessions" / session / "configs" / cid
                return self.send(STATE.overlay(d, int(stop.split(".")[0])), "image/jpeg")
            self.send(b"not found", "text/plain", 404)
        except Exception as e:
            self.send(json.dumps({"error": repr(e)}).encode(), "application/json", 500)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5010)
    a = ap.parse_args()
    ThreadingHTTPServer(("127.0.0.1", a.port), Handler).serve_forever()
