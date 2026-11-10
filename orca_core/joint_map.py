"""Per-hand nonlinear joint↔motor map: ``joint_map.yaml``.

The standard map is linear between a joint's hard stops. A hand measured
externally (e.g. ORCA's visual calibration) can ship a ``joint_map.yaml`` next
to its ``config.yaml`` that carries its own hard stops and, per joint, a
monotone curve. orca_core never writes this file, so a recalibration keeps it.

Knots: ``s`` is the joint-direction fraction of the motor's travel between its
calibrated limits (0 at the joint's lower stop, 1 at its upper; ``1 - t`` on
an inverted joint, where ``t`` is the raw motor fraction). ``deg`` is the joint
angle there. ``s`` depends only on the motor limits, which every calibration
re-measures, and on nothing in the config, so a curve stays valid through
recalibrations and ROM edits. Between knots the map is piecewise linear (exact
inverse, no overshoot); outside ``s`` ∈ [0, 1] it continues linearly at the
curve's stop-to-stop slope, like the linear map does, so the joint loop's
deliberate over-travel keeps working.

Stop precedence, lowest to highest: the packaged ``config.yaml`` < a bench
``config.yaml`` (``config_path``) < ``joint_roms_v211`` (v2.1.1 hands) <
``joint_map.yaml`` ``joint_roms``. The calibration routine fits each motor's
ratio against the result; ``joint_roms_measured`` (encoder hands) still
overlays it in ``effective_joint_roms_dict``. A joint with a curve takes its
angle from the curve; every other joint keeps the linear map.

Anything unusable (no file, a joint without a curve, a malformed or
non-monotone curve) leaves that joint on the linear map; all but the missing
file are logged as warnings.
"""

from __future__ import annotations

import logging
import math
import os
from bisect import bisect_right
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from .utils.utils import read_yaml

logger = logging.getLogger(__name__)

JOINT_MAP_FILE = "joint_map.yaml"
JOINT_MAP_FORMAT_VERSION = 1
MIN_KNOTS = 2
MAX_KNOTS = 64
# How far a curve's angles may lie outside the joint's stops.
ROM_TOLERANCE_DEG = 3.0
_S_END_TOL = 1e-6


def joint_map_path(config_path: str) -> str:
    """``joint_map.yaml`` beside ``config_path``."""
    return os.path.join(os.path.dirname(os.path.abspath(config_path)), JOINT_MAP_FILE)


class JointCurve:
    """Monotone piecewise-linear map between motor fraction ``s`` and degrees."""

    __slots__ = ("s", "deg", "_s", "_deg", "_s_inner", "_deg_inner", "_fwd", "_inv")

    def __init__(self, s: Sequence[float], deg: Sequence[float]):
        self.s = tuple(float(v) for v in s)
        self.deg = tuple(float(v) for v in deg)
        span = self.deg[-1] - self.deg[0]
        # One guard knot each side carries the stop-to-stop slope past the ends.
        self._s = (-1.0, *self.s, 2.0)
        self._deg = (self.deg[0] - span, *self.deg, self.deg[-1] + span)
        self._s_inner = self._s[1:-1]
        self._deg_inner = self._deg[1:-1]
        pairs = list(zip(self._s, self._deg))
        self._fwd = tuple((d1 - d0) / (s1 - s0) for (s0, d0), (s1, d1) in zip(pairs, pairs[1:]))
        self._inv = tuple((s1 - s0) / (d1 - d0) for (s0, d0), (s1, d1) in zip(pairs, pairs[1:]))

    def forward(self, s: float) -> float:
        """Joint angle (deg) at motor fraction ``s``."""
        i = bisect_right(self._s_inner, s)
        return self._deg[i] + (s - self._s[i]) * self._fwd[i]

    def inverse(self, deg: float) -> float:
        """Motor fraction ``s`` at joint angle ``deg``."""
        i = bisect_right(self._deg_inner, deg)
        return self._s[i] + (deg - self._deg[i]) * self._inv[i]

    def __eq__(self, other) -> bool:
        return isinstance(other, JointCurve) and (self.s, self.deg) == (other.s, other.deg)

    def __repr__(self) -> str:
        return f"JointCurve({len(self.s)} knots, {self.deg[0]:.2f}..{self.deg[-1]:.2f} deg)"


class CurveBank:
    """Several joints' curves stacked for the joint loop: vectorised and
    allocation-free per call (every buffer is preallocated).

    Uses the same segment choice and arithmetic as :class:`JointCurve`, so
    both agree bit for bit.
    """

    def __init__(self, curves: Sequence[JointCurve]):
        n = len(curves)
        g = max(len(c._s) for c in curves)
        # Shorter curves are padded with +inf inner knots, which no angle
        # passes, so every row picks exactly the segment JointCurve picks.
        deg_inner = np.full((n, g - 2), np.inf)
        s0 = np.zeros((n, g - 1))
        deg0 = np.zeros((n, g - 1))
        inv = np.zeros((n, g - 1))
        for row, c in enumerate(curves):
            k = len(c._s)
            deg_inner[row, : k - 2] = c._deg_inner
            s0[row, : k - 1] = c._s[:-1]
            deg0[row, : k - 1] = c._deg[:-1]
            inv[row, : k - 1] = c._inv
        self._deg_inner = deg_inner
        self._s0 = s0.ravel()
        self._deg0 = deg0.ravel()
        self._inv = inv.ravel()
        self._row_base = np.arange(n, dtype=np.int64) * (g - 1)
        self._cmp = np.empty((n, g - 2), dtype=bool)
        self._seg = np.empty(n, dtype=np.int64)
        self._a = np.empty(n)
        self._b = np.empty(n)

    def inverse(self, deg: np.ndarray, out: np.ndarray) -> np.ndarray:
        """Motor fractions for ``deg`` (one per curve) written into ``out``."""
        np.less_equal(self._deg_inner, deg[:, None], out=self._cmp)
        np.sum(self._cmp, axis=1, out=self._seg)
        self._seg += self._row_base
        np.take(self._deg0, self._seg, out=self._a)
        np.subtract(deg, self._a, out=out)
        np.take(self._inv, self._seg, out=self._a)
        out *= self._a
        np.take(self._s0, self._seg, out=self._b)
        out += self._b
        return out


@dataclass(frozen=True)
class JointMap:
    """A loaded and validated ``joint_map.yaml``."""

    path: str
    format_version: int = JOINT_MAP_FORMAT_VERSION
    hand_serial: Optional[str] = None
    model: Optional[str] = None
    created: Optional[str] = None
    source: Optional[str] = None
    base_joint_roms: Dict[str, List[float]] = field(default_factory=dict)
    joint_roms: Dict[str, List[float]] = field(default_factory=dict)
    curves: Dict[str, JointCurve] = field(default_factory=dict)


def _rom(raw) -> Optional[List[float]]:
    try:
        lo, hi = (float(v) for v in raw)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(lo) and math.isfinite(hi) and hi > lo):
        return None
    return [lo, hi]


def curve_problem(s, deg, rom: Optional[Sequence[float]]) -> Optional[str]:
    """Why ``(s, deg)`` is not a usable curve within ``rom``, or None."""
    try:
        s = [float(v) for v in s]
        deg = [float(v) for v in deg]
    except (TypeError, ValueError):
        return "knots are not numbers"
    if len(s) != len(deg):
        return f"{len(s)} s knots but {len(deg)} deg knots"
    if not MIN_KNOTS <= len(s) <= MAX_KNOTS:
        return f"{len(s)} knots (allowed {MIN_KNOTS}..{MAX_KNOTS})"
    if not all(math.isfinite(v) for v in s + deg):
        return "non-finite knot"
    if abs(s[0]) > _S_END_TOL or abs(s[-1] - 1.0) > _S_END_TOL:
        return f"s must run from 0 to 1, got {s[0]}..{s[-1]}"
    if any(b <= a for a, b in zip(s, s[1:])):
        return "s is not strictly increasing"
    if any(b <= a for a, b in zip(deg, deg[1:])):
        return "deg is not strictly increasing (non-monotone curve)"
    if rom is not None and (
        deg[0] < rom[0] - ROM_TOLERANCE_DEG or deg[-1] > rom[1] + ROM_TOLERANCE_DEG
    ):
        return (
            f"angles {deg[0]:.2f}..{deg[-1]:.2f} lie outside the stops "
            f"[{rom[0]}, {rom[1]}] by more than {ROM_TOLERANCE_DEG} deg"
        )
    return None


def load_joint_map(
    path: str,
    joint_ids: Sequence[str],
    joint_roms: Dict[str, List[float]],
    model: Optional[str] = None,
) -> Optional[JointMap]:
    """Load and validate ``path``; None when it is absent or unusable.

    ``joint_roms`` are the config's stops, used to check curves of joints the
    file gives no stops of its own. Invalid entries are dropped with a warning.
    """
    if not os.path.exists(path):
        logger.debug("no %s: linear joint map", path)
        return None
    try:
        doc = read_yaml(path)
    except Exception as exc:
        logger.warning("%s could not be read (%s); using the linear joint map", path, exc)
        return None
    if not isinstance(doc, dict):
        logger.warning("%s is not a mapping; using the linear joint map", path)
        return None
    version = doc.get("format_version")
    if version != JOINT_MAP_FORMAT_VERSION:
        logger.warning(
            "%s has format_version %r (this orca_core reads %d); using the linear joint map",
            path, version, JOINT_MAP_FORMAT_VERSION,
        )
        return None
    if model and doc.get("model") and doc["model"] != model:
        logger.warning("%s was made for model %r but loads for %r", path, doc["model"], model)

    known = set(joint_ids)

    def roms(key: str) -> Dict[str, List[float]]:
        raw = doc.get(key) or {}
        if not isinstance(raw, dict):
            logger.warning("%s: %s is not a mapping; ignoring it", path, key)
            return {}
        out = {}
        for joint, value in raw.items():
            rom = _rom(value)
            if joint not in known or rom is None:
                logger.warning("%s: ignoring %s[%s] = %r", path, key, joint, value)
                continue
            out[joint] = rom
        return out

    own = roms("joint_roms")
    curves: Dict[str, JointCurve] = {}
    raw_curves = doc.get("curves") or {}
    if not isinstance(raw_curves, dict):
        logger.warning("%s: curves is not a mapping; ignoring them", path)
        raw_curves = {}
    for joint, raw in raw_curves.items():
        if joint not in known:
            logger.warning("%s: curve for unknown joint %r ignored", path, joint)
            continue
        knots = raw if isinstance(raw, dict) else {}
        problem = curve_problem(
            knots.get("s", ()), knots.get("deg", ()), own.get(joint) or joint_roms.get(joint)
        )
        if problem:
            logger.warning("%s: %s curve rejected (%s); %s stays linear", path, joint, problem, joint)
            continue
        curves[joint] = JointCurve(knots["s"], knots["deg"])
    linear = [j for j in joint_ids if j not in curves]
    if curves and linear:
        logger.warning("%s: no curve for %s; those joints use the linear map", path, ", ".join(linear))
    logger.info(
        "%s: %d curves, own stops for %d joints", path, len(curves), len(own)
    )
    return JointMap(
        path=os.path.abspath(path),
        format_version=version,
        hand_serial=_str_or_none(doc.get("hand_serial")),
        model=_str_or_none(doc.get("model")),
        created=_str_or_none(doc.get("created")),
        source=_str_or_none(doc.get("source")),
        base_joint_roms=roms("base_joint_roms"),
        joint_roms=own,
        curves=curves,
    )


def _str_or_none(value) -> Optional[str]:
    return None if value is None else str(value)


def from_visual_calibration(doc: dict) -> tuple[dict, Dict[str, str]]:
    """A ``joint_map.yaml`` document from a visual calibration (``quickfit``) result.

    quickfit's knot ``x`` is orca_core's linear estimate under the packaged
    stops, so ``s = (x - lo) / (hi - lo)`` with those stops; ``true`` is the
    angle. Its own stops become ``joint_roms``. Returns the document and, per
    joint left out (linear), why its curve failed validation.
    """
    stops = doc["stops"]
    base = {j: [float(v) for v in rom] for j, rom in stops["packaged"].items()}
    own = {j: [float(v) for v in rom] for j, rom in stops["joint_roms"].items()}
    curves, skipped = {}, {}
    for joint, knots in doc["curve"]["knots"].items():
        lo, hi = base[joint]
        s = [round((float(x) - lo) / (hi - lo), 6) for x in knots["x"]]
        deg = [round(float(v), 4) for v in knots["true"]]
        problem = curve_problem(s, deg, own.get(joint, base[joint]))
        if problem:
            skipped[joint] = problem
            continue
        curves[joint] = {"s": s, "deg": deg}
    out = {
        "format_version": JOINT_MAP_FORMAT_VERSION,
        "hand_serial": doc.get("hand"),
        "model": doc.get("model"),
        "created": doc.get("created"),
        "source": (
            f"visual calibration {doc.get('tag')} ({doc.get('n_train', 0) + doc.get('n_test', 0)} poses, "
            f"orca_core calibration run {doc.get('calibration_run')})"
        ),
        "base_joint_roms": base,
        "joint_roms": own,
        "curves": curves,
    }
    return out, skipped
