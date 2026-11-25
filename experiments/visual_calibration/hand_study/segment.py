"""Optional learned hand mask for StudyFit, instead of GrabCut (opt-in: STUDY_SEGMENTER, default "grabcut").

STUDY_SEGMENTER=hqsam - HQ-SAM vit-b (Apache-2.0, lkeab/hq-sam), prompted from the URDF at the fit's current pose:
one object per finger, the palm, each with its box, a point inside every link and background points around it (the
segtest benchmark's "fingers" variant, ~1.5 s per 4K photo on MPS). Uses segtest's install (segment_anything_hq, weights
in segtest/models/hf), appended to sys.path, not installed in this venv.
STUDY_SEGMENTER=birefnet[:LONG_SIDE] - BiRefNet-dynamic (MIT, HF ZhengPeng7/BiRefNet_dynamic), prompt-free, on the
full-resolution photo (the hand's part of it) at LONG_SIDE px (default 1024), MPS if available. It needs no seed, so
each photo is segmented once and reused across rounds, halves and re-solves; the fit keeps it within GrabCut's own
band around the model. Benchmark and rationale: ~/Downloads/hand_segmentation_models/report.html (8 Nov 2026).
Needs orca_core's `rig` extra (torch, torchvision, transformers, timm, kornia, einops); the first use downloads the
weights (~450 MB).
"""
from __future__ import annotations

import os

import numpy as np

MEAN, STD = np.array([0.485, 0.456, 0.406]), np.array([0.229, 0.224, 0.225])
X_MAX = 2900 / 3840                          # on the rig the hand never reaches the frame's right quarter


def name() -> str:
    return os.environ.get("STUDY_SEGMENTER", "grabcut").strip().lower() or "grabcut"


class BiRefNet:
    prompted = False

    def __init__(self, long_side: int = 1024, repo: str = "ZhengPeng7/BiRefNet_dynamic"):
        os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
        import torch
        from transformers import AutoModelForImageSegmentation
        self.torch, self.long = torch, long_side
        self.dev = "mps" if torch.backends.mps.is_available() else "cpu"
        m = AutoModelForImageSegmentation.from_pretrained(repo, trust_remote_code=True).eval()
        # transformers 5 loads the checkpoint's own dtype (float16), which a CPU cannot run
        self.model = m.to(self.dev).half() if self.dev == "mps" else m.float()
        self.cache: dict = {}

    def prob(self, img: np.ndarray) -> np.ndarray:
        """Hand probability (float32, 0..1) at the photo's own resolution; cached per photo array."""
        import cv2
        key = (img.shape, hash(img[::61, ::67].tobytes()))   # by content: freed arrays' addresses get reused
        if key in self.cache:
            return self.cache[key]
        H, W = img.shape[:2]
        crop = cv2.cvtColor(img[:, : round(W * X_MAX)], cv2.COLOR_BGR2RGB)
        h, w = crop.shape[:2]
        s = self.long / max(h, w)
        th, tw = max(32, round(h * s / 32) * 32), max(32, round(w * s / 32) * 32)
        x = (cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA) / 255.0 - MEAN) / STD
        t = self.torch.from_numpy(x.transpose(2, 0, 1)[None]).float().to(self.dev)
        with self.torch.no_grad():
            p = self.model(t.half() if self.dev == "mps" else t)[-1].sigmoid().float().cpu().numpy()[0, 0]
        out = np.zeros((H, W), np.float32)
        out[:, :w] = cv2.resize(p, (w, h), interpolation=cv2.INTER_LINEAR)
        if self.dev == "mps":
            self.torch.mps.empty_cache()
        if len(self.cache) >= 16:                # a config's photos (≤ 8) are all that is reused
            self.cache.pop(next(iter(self.cache)))
        self.cache[key] = out
        return out


SEGTEST = "/Users/ccc/orca_dev/segtest"
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


class HQSam:
    prompted = True
    NEG_NEAR, NEG_FAR = 60, 160                 # full-res px outside the model: sure background

    def __init__(self):
        import sys
        os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
        site = f"{SEGTEST}/.venv/lib/python3.12/site-packages"
        if site not in sys.path:
            sys.path.append(site)                  # last: this venv's own packages win
        import torch
        from huggingface_hub import hf_hub_download
        from segment_anything_hq import SamPredictor, sam_model_registry
        self.torch = torch
        self.dev = "mps" if torch.backends.mps.is_available() else "cpu"
        sam = sam_model_registry["vit_b"]()
        w = hf_hub_download("lkeab/hq-sam", "sam_hq_vit_b.pth", cache_dir=f"{SEGTEST}/models/hf/hub")
        sam.load_state_dict(torch.load(w, map_location="cpu"))
        self.pred = SamPredictor(sam.to(self.dev).eval())
        self.cache: dict = {}

    @staticmethod
    def _inner(m):
        import cv2
        d = cv2.distanceTransform(np.pad(m.astype(np.uint8), 1), cv2.DIST_L2, 5)[1:-1, 1:-1]
        v, u = np.unravel_index(np.argmax(d), d.shape)
        return [float(u), float(v)]

    def prob(self, img: np.ndarray, lab: np.ndarray, ids: dict[str, int]) -> np.ndarray:
        """Hand probability (0/1) at the photo's resolution; lab: the model's link labels at the current pose."""
        import cv2
        key = (img.shape, hash(img[::61, ::67].tobytes()), hash(lab[::31, ::37].tobytes()))
        if key in self.cache:
            return self.cache[key]
        groups = [[i for l, i in ids.items() if l.startswith(f + "_")] for f in FINGERS]
        groups.append([i for l, i in ids.items() if not l.startswith(FINGERS) and l not in ("forearm", "tower")])
        hand = lab > 0
        ys, xs = np.nonzero(hand)
        out = np.zeros(lab.shape, np.float32)
        if not len(xs):
            return out
        H, W = lab.shape
        u0, v0 = max(xs.min() - 200, 0), max(ys.min() - 200, 0)
        u1, v1 = min(xs.max() + 200, W - 1), min(ys.max() + 200, H - 1)
        l = lab[v0:v1 + 1, u0:u1 + 1]
        far = cv2.distanceTransform((l == 0).astype(np.uint8), cv2.DIST_L2, 5)
        rng = np.random.default_rng(0)
        objs = []
        for g in groups:
            prior = np.isin(l, g)
            if prior.sum() < 400:
                continue
            gy, gx = np.nonzero(prior)
            b = [gx.min(), gy.min(), gx.max(), gy.max()]
            p = 0.1 * max(b[2] - b[0], b[3] - b[1]) + 10
            b = [max(b[0] - p, 0), max(b[1] - p, 0), min(b[2] + p, l.shape[1] - 1), min(b[3] + p, l.shape[0] - 1)]
            pos = [self._inner(l == i) for i in g if (l == i).sum() > 400]
            ring = (far > self.NEG_NEAR) & (far < self.NEG_FAR)
            ring[:int(b[1])] = False
            ring[int(b[3]) + 1:] = False
            ring[:, :int(b[0])] = False
            ring[:, int(b[2]) + 1:] = False
            ry, rx = np.nonzero(ring)
            k = rng.choice(len(rx), min(6, len(rx)), replace=False) if len(rx) else []
            neg = [[float(rx[i]), float(ry[i])] for i in k]
            objs.append((b, pos + neg, [1] * len(pos) + [0] * len(neg), prior))
        if objs:
            torch = self.torch
            self.pred.set_image(cv2.cvtColor(img[v0:v1 + 1, u0:u1 + 1], cv2.COLOR_BGR2RGB))
            tr, n = self.pred.transform, max(len(o[1]) for o in objs)
            shape = l.shape
            pts = torch.tensor([o[1] + [[0.0, 0.0]] * (n - len(o[1])) for o in objs], dtype=torch.float32)
            lbl = torch.tensor([o[2] + [-1] * (n - len(o[2])) for o in objs])
            box = torch.tensor([o[0] for o in objs], dtype=torch.float32)
            with torch.no_grad():
                m, sc, _ = self.pred.predict_torch(tr.apply_coords_torch(pts, shape).to(self.dev), lbl.to(self.dev),
                                                   tr.apply_boxes_torch(box, shape).to(self.dev), multimask_output=True,
                                                   return_logits=True)
            m, sc = m.float().cpu().numpy(), sc.float().cpu().numpy()
            acc = out[v0:v1 + 1, u0:u1 + 1]
            for k, (b, _, _, prior) in enumerate(objs):
                ious = np.array([((lg > 0) & prior).sum() / max(((lg > 0) | prior).sum(), 1) for lg in m[k]])
                ok = ious >= 0.8 * ious.max()          # of SAM's 3 answers those the model agrees with, SAM's best
                lg = m[k][int(np.argmax(np.where(ok, sc[k], -1)))]
                p = 0.3 * max(b[2] - b[0], b[3] - b[1])
                keep = np.zeros(shape, bool)
                keep[max(int(b[1] - p), 0):int(b[3] + p) + 1, max(int(b[0] - p), 0):int(b[2] + p) + 1] = True
                acc[(lg > 0) & keep] = 1.0
            if self.dev == "mps":
                torch.mps.empty_cache()
        if len(self.cache) >= 16:
            self.cache.pop(next(iter(self.cache)))
        self.cache[key] = out
        return out


_model = None


def masker():
    """The configured learned segmenter (loaded once per process), or None for GrabCut."""
    global _model
    n = name()
    if n == "grabcut":
        return None
    kind, _, size = n.partition(":")
    if kind not in ("birefnet", "hqsam"):
        raise ValueError(f"STUDY_SEGMENTER={n!r}: use grabcut, birefnet[:LONG_SIDE] or hqsam")
    if _model is None:
        _model = HQSam() if kind == "hqsam" else BiRefNet(int(size or 1024))
    return _model
