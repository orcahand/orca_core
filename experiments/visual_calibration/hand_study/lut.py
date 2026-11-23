"""The lookup-table calibration: commanded joint angles -> true joint angles, per joint a 1-D
table over its own command plus additive 1-D tables over the joints it depends on (same
finger, wrist, and any significant coupling), and a 2-D table over the finger's flexion pair.
Fitted by regularised least squares on the study's poses, checked on held-out poses, and
inverted for control (true target -> command).

    python -m hand_study.lut          # fit on the study data, write lut.json + the report
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from hand_study.configs import JOINTS, ROM
from hand_study.paths import STUDY

OWN_KNOTS = 9
PARTNER_KNOTS = 4
PAIR_KNOTS = 4
SMOOTH = 3.0
RIDGE = 0.3
FLEX_PAIR = {f"{f}_{a}": f"{f}_{b}" for f, a, b in
             (("index", "pip", "mcp"), ("middle", "pip", "mcp"), ("ring", "pip", "mcp"), ("pinky", "pip", "mcp"),
              ("thumb", "dip", "mcp"))}


def finger(j: str) -> str:
    return j.split("_")[0]


def hat(x: np.ndarray, knots: np.ndarray) -> np.ndarray:
    """Linear-interpolation weights of x on the knots (rows: x, cols: knots), clamped at the ends."""
    x = np.clip(x, knots[0], knots[-1])
    k = np.clip(np.searchsorted(knots, x, side="right") - 1, 0, len(knots) - 2)
    t = (x - knots[k]) / (knots[k + 1] - knots[k])
    B = np.zeros((len(x), len(knots)))
    B[np.arange(len(x)), k] = 1 - t
    B[np.arange(len(x)), k + 1] = t
    return B


def knots_of(j: str, n: int) -> np.ndarray:
    return np.linspace(*ROM[j], n)


class JointTable:
    def __init__(self, j: str, partners: list[str], pair: bool = True):
        self.j, self.partners = j, [p for p in partners if p != j]
        self.own = knots_of(j, OWN_KNOTS)
        self.pk = {p: knots_of(p, PARTNER_KNOTS) for p in self.partners}
        self.pair = FLEX_PAIR.get(j) if pair else None
        self.pair_k = (knots_of(j, PAIR_KNOTS), knots_of(self.pair, PAIR_KNOTS)) if self.pair else None
        self.w: np.ndarray | None = None

    def design(self, C: dict[str, np.ndarray]) -> tuple[np.ndarray, list[tuple[int, int, str]]]:
        """Columns and the blocks (start, stop, kind) for the smoothness penalty."""
        cols, blocks, at = [hat(C[self.j], self.own)], [(0, OWN_KNOTS, "own")], OWN_KNOTS
        for p in self.partners:
            B = hat(C[p], self.pk[p])
            cols.append(B[:, 1:])                       # the first knot is the reference (0)
            blocks.append((at, at + PARTNER_KNOTS - 1, "partner"))
            at += PARTNER_KNOTS - 1
        if self.pair:
            a, b = hat(C[self.j], self.pair_k[0]), hat(C[self.pair], self.pair_k[1])
            T = (a[:, :, None] * b[:, None, :]).reshape(len(a), -1)
            cols.append(T)
            blocks.append((at, at + T.shape[1], "pair"))
        return np.hstack(cols), blocks

    def penalty(self, blocks, n: int) -> np.ndarray:
        rows = []
        for s, e, kind in blocks:
            if kind == "pair":
                k = PAIR_KNOTS
                for i in range(k):
                    for m in range(k):
                        r = np.zeros(n)
                        r[s + i * k + m] = RIDGE * 3
                        rows.append(r)
                continue
            for i in range(s, e - 2):
                r = np.zeros(n)
                r[i], r[i + 1], r[i + 2] = SMOOTH, -2 * SMOOTH, SMOOTH
                rows.append(r)
            if kind == "partner":
                for i in range(s, e):
                    r = np.zeros(n)
                    r[i] = RIDGE
                    rows.append(r)
        return np.array(rows) if rows else np.zeros((0, n))

    def fit(self, C: dict[str, np.ndarray], y: np.ndarray) -> None:
        A, blocks = self.design(C)
        P = self.penalty(blocks, A.shape[1])
        # prior: identity on the own table (true = commanded) where the data is thin
        prior = np.zeros(A.shape[1])
        prior[:OWN_KNOTS] = self.own
        Ap = np.vstack([A, P, 0.05 * np.eye(A.shape[1])[:OWN_KNOTS]])
        yp = np.r_[y, np.zeros(len(P)), 0.05 * prior[:OWN_KNOTS]]
        self.w, *_ = np.linalg.lstsq(Ap, yp, rcond=None)

    def predict(self, C: dict[str, np.ndarray]) -> np.ndarray:
        A, _ = self.design(C)
        return A @ self.w

    def to_json(self) -> dict:
        return {"joint": self.j, "partners": self.partners, "own_knots": self.own.tolist(),
                "partner_knots": {p: k.tolist() for p, k in self.pk.items()}, "pair": self.pair,
                "pair_knots": [k.tolist() for k in self.pair_k] if self.pair_k else None, "w": self.w.tolist()}

    @classmethod
    def from_json(cls, d: dict) -> JointTable:
        t = cls(d["joint"], d["partners"], pair=bool(d["pair"]))
        t.own = np.array(d["own_knots"])
        t.pk = {p: np.array(k) for p, k in d["partner_knots"].items()}
        t.pair = d["pair"]
        t.pair_k = tuple(np.array(k) for k in d["pair_knots"]) if d["pair_knots"] else None
        t.w = np.array(d["w"])
        return t


class Lut:
    def __init__(self, tables: dict[str, JointTable]):
        self.tables = tables

    def forward(self, cmd: dict[str, float]) -> dict[str, float]:
        """Commanded joint angles -> the true angles the hand reaches."""
        C = {j: np.array([float(cmd.get(j, 0.0))]) for j in JOINTS}
        return {j: float(t.predict(C)[0]) for j, t in self.tables.items()}

    def inverse(self, target: dict[str, float], iters: int = 40) -> dict[str, float]:
        """The command that makes the hand reach `target` (true angles), within the joint limits."""
        c = {j: float(target.get(j, 0.0)) for j in JOINTS}
        goal = dict(c)
        for _ in range(iters):
            q = self.forward(c)
            err = {j: goal[j] - q[j] for j in self.tables}
            for j, e in err.items():
                c[j] = float(np.clip(c[j] + 0.8 * e, *ROM[j]))
            if max(abs(e) for e in err.values()) < 0.05:
                break
        return c

    def to_json(self) -> dict:
        return {"joints": JOINTS, "rom": ROM, "tables": {j: t.to_json() for j, t in self.tables.items()}}

    @classmethod
    def load(cls, path: Path = STUDY / "lut_model.json") -> Lut:
        d = json.loads(Path(path).read_text())
        return cls({j: JointTable.from_json(t) for j, t in d["tables"].items()})


def partners_of(j: str, significant: set[tuple[str, str]]) -> list[str]:
    p = [k for k in JOINTS if k != j and (finger(k) == finger(j) or k == "wrist")]
    p += [k for (a, k) in significant if a == j and k not in p]
    return p


def heldout_mask(R: list[dict]) -> np.ndarray:
    """Every third random pose and the last visit of each repeat are held out."""
    m = []
    for r in R:
        if r["kind"] and r["kind"].startswith("random"):
            m.append(int(r["id"].rsplit("-", 1)[-1]) % 3 == 0)
        elif r["kind"] == "repeat":
            m.append(r["id"].endswith("-10"))
        else:
            m.append(False)
    return np.array(m)


def fingertip_errors(Q_true: np.ndarray, Q_pred: np.ndarray) -> dict[str, float]:
    from visual_calibration.hand_model import HandModel
    model = HandModel.load("right")
    out = {f: [] for f in ("thumb", "index", "middle", "ring", "pinky")}
    tips = {f: model.fingertip(f) for f in out}
    for a, b in zip(Q_true, Q_pred):
        pa, pb = model.link_poses(dict(zip(JOINTS, a))), model.link_poses(dict(zip(JOINTS, b)))
        for f, (link, tip) in tips.items():
            out[f].append(np.linalg.norm(pa[link].apply_to_points(tip[None])[0] - pb[link].apply_to_points(tip[None])[0]) * 1000)
    return {f: float(np.sqrt(np.mean(np.square(v)))) for f, v in out.items()}


def fit_rows(train: list[dict]) -> Lut:
    """A LUT fitted on the rows given (commanded -> measured), couplings from their t values."""
    from hand_study import analysis
    deps = analysis.dependencies(train)
    sig = {(deps["joints"][a], deps["joints"][b]) for a, row in enumerate(deps["t"]) for b, t in enumerate(row)
           if t is not None and abs(t) >= analysis.T_SIGNIFICANT}
    C = {j: np.array([r["commanded"][j] for r in train]) for j in JOINTS}
    tables = {}
    for j in JOINTS:
        t = JointTable(j, partners_of(j, sig))
        t.fit(C, np.array([r["measured"][j] for r in train]))
        tables[j] = t
    return Lut(tables)


def main() -> dict:
    from hand_study import analysis
    R = analysis.rows()
    if len(R) < 20:
        raise SystemExit(f"only {len(R)} solved poses")
    held = heldout_mask(R)
    train = [r for r, h in zip(R, held) if not h]
    test = [r for r, h in zip(R, held) if h]
    deps = analysis.dependencies(train)
    sig = {(deps["joints"][a], deps["joints"][b]) for a, row in enumerate(deps["t"]) for b, t in enumerate(row)
           if t is not None and abs(t) >= analysis.T_SIGNIFICANT}

    def cols(rs, key):
        return {j: np.array([r[key][j] for r in rs]) for j in JOINTS}

    Ctr, Cte = cols(train, "commanded"), cols(test, "commanded")
    tables = {}
    for j in JOINTS:
        t = JointTable(j, partners_of(j, sig))
        t.fit(Ctr, np.array([r["measured"][j] for r in train]))
        tables[j] = t
    lut = Lut(tables)
    (STUDY / "lut_model.json").write_text(json.dumps(lut.to_json()))

    def rms(v):
        return float(np.sqrt(np.mean(np.square(v)))) if len(v) else None

    mounts = [json.loads((p / "plan.json").read_text()).get("mount") for p in analysis.sessions()]
    out = {"n_train": len(train), "n_test": len(test), "significant": sorted(f"{a}|{b}" for a, b in sig),
           "heldout": {}, "train": {}, "mount": next((m for m in mounts[::-1] if m), None)}
    Qte = np.array([[r["measured"][j] for j in JOINTS] for r in test]) if test else np.zeros((0, len(JOINTS)))
    Pte = np.array([[tables[j].predict(Cte)[i] for j in JOINTS] for i in range(len(test))]) if test else Qte
    Ate = np.array([[r["assumed"][j] for j in JOINTS] for r in test]) if test else Qte
    Kte = np.array([[r["commanded"][j] for j in JOINTS] for r in test]) if test else Qte
    for a, j in enumerate(JOINTS):
        out["heldout"][j] = {"before": rms(Qte[:, a] - Kte[:, a]), "before_assumed": rms(Qte[:, a] - Ate[:, a]),
                             "after": rms(Qte[:, a] - Pte[:, a])}
        ytr = np.array([r["measured"][j] for r in train])
        out["train"][j] = {"before": rms(ytr - Ctr[j]), "after": rms(ytr - tables[j].predict(Ctr))}
    if len(test):
        before = fingertip_errors(Qte, Kte)
        after = fingertip_errors(Qte, Pte)
        out["tips"] = {f: {"before": before[f], "after": after[f]} for f in before}
    tot_b = rms((Qte - Kte).ravel()) if len(test) else None
    tot_a = rms((Qte - Pte).ravel()) if len(test) else None
    out["summary"] = (f"Held-out poses: {len(test)} (fitted on {len(train)}). RMS joint error over all joints: "
                      f"{tot_b:.1f}° with orca_core's map → {tot_a:.1f}° with the lookup table." if len(test) else
                      f"Fitted on {len(train)} poses; no held-out poses yet.")
    (STUDY / "lut.json").write_text(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    o = main()
    print(o["summary"])
    for j in JOINTS:
        print(f"{j:12s} before {o['heldout'][j]['before']} after {o['heldout'][j]['after']}")
    print(o.get("tips"))
