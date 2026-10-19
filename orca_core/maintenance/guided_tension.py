# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""Guided tendon tensioning: pose → range check → score → tension, repeated.

Each round walks one stage per finger: the operator poses the hand with torque
off, then the finger motors hold that pose at a low current while the operator
drives the stage's joints through their range by hand. A tendon with slack moves
the joint without dragging its motor along, so how far each motor is dragged —
its spread from the furthest ``+`` to the furthest ``-`` — measured against the
joint's motor-travel baseline (v2.1 hardware) scores how taut that tendon is.
Below 100 % the motors hold at a higher current while the operator takes up the
slack with the ratchet tool, and the round repeats.

The wrist is never touched. Interaction-free like the other maintenance
routines: progress goes out through ``progress_callback({"event": ...})`` and
each operator step blocks on ``prompt_callback({"action": ...}) -> str``. The
range check samples on its own thread while that prompt blocks, so the
front-end shows the spreads live and answers when the operator is done.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional, Sequence, Tuple, TYPE_CHECKING, Union

import numpy as np

from ..constants import CURRENT_BASED_POSITION, WRIST
from ..hand_config import format_hardware_version
from ..utils.utils import read_yaml, write_yaml_atomic
from .motor_reads import read_motor_pos_checked
from .tensioning import (
    TENSION_EXTRA_DEG,
    TENSION_OFFSET_DEG,
    pull_tendons,
    settle_at_rest,
    uncapped_currents,
)

if TYPE_CHECKING:
    from ..hand_config import OrcaHandConfig
    from ..hardware_hand import OrcaHand

logger = logging.getLogger(__name__)

ProgressCallback = Callable[[dict], None]
PromptCallback = Callable[[dict], str]
ShouldStop = Callable[[], bool]
Current = Union[float, Callable[[], float]]
HoldCurrent = Union[float, Callable[[str], float]]  # one value, or one per joint name
PassFraction = Union[float, Callable[[str], float]]  # one value, or one per joint name

HOLD_CURRENT_MA = 100
# The pull's current limit when a caller names none: the connected family's own
# max_current, so it is not a figure that happens to suit one motor family. The
# ratcheting hold that follows is not capped by this at all.
TENSION_CURRENT_MA = None
PASS_FRACTION = 0.85
PARTIAL_FRACTION = 0.3
SAMPLE_PERIOD_S = 0.05
LAST_TENSION_FILE = "last_tension.yaml"
PROGRESS_FILE = "tension_progress.yaml"
DEFAULT_HAND_KEY = "default"

CONTINUE = "Continue"
NEXT = "Next"
CHECK_SCORE = "Score"
TENSION = "Tension"
FINISH = "Finish"
FINISH_ANYWAY = "Finish anyway"
DONE = "Done"
PULL_MORE = "Pull more"
BACK = "Back"
GOTO = "goto:"  # answer prefix that jumps to a stage by key, or to the score or tension step
GOTO_SCORE = f"{GOTO}score"
GOTO_TENSION = f"{GOTO}tension"

STATUS_PASS = "pass"
STATUS_PARTIAL = "partial"
STATUS_LOW = "low"
STATUS_UNSCORED = "unscored"

# Why a joint is left out of a run and its score sheet.
EXCLUDED_NO_MOTOR = "motor did not answer at connect"


@dataclass
class MotorSpread:
    """How far one joint's motor has been dragged from its held centre."""

    joint: str
    motor_id: int
    expected_deg: Optional[float]
    offset_deg: float = 0.0
    plus_deg: float = 0.0
    minus_deg: float = 0.0
    kept: Optional[float] = None  # best fraction from earlier rounds or sessions

    @property
    def spread_deg(self) -> float:
        return self.plus_deg - self.minus_deg

    @property
    def live_fraction(self) -> Optional[float]:
        if not self.expected_deg:
            return None
        return self.spread_deg / self.expected_deg

    @property
    def fraction(self) -> Optional[float]:
        """Best of this check and anything kept: a joint that once passed stays passed."""
        live = self.live_fraction
        if live is None or self.kept is None:
            return live
        return max(live, self.kept)

    def status(self, pass_fraction: float) -> str:
        fraction = self.fraction
        if fraction is None:
            return STATUS_UNSCORED
        if fraction >= pass_fraction:
            return STATUS_PASS
        return STATUS_PARTIAL if fraction >= PARTIAL_FRACTION else STATUS_LOW

    def to_dict(self, pass_fraction: float) -> dict:
        fraction = self.fraction
        live = self.live_fraction
        return {
            "joint": self.joint,
            "motor": self.motor_id,
            "expected_deg": self.expected_deg,
            "offset_deg": round(self.offset_deg, 2),
            "plus_deg": round(self.plus_deg, 2),
            "minus_deg": round(self.minus_deg, 2),
            "spread_deg": round(self.spread_deg, 2),
            "fraction": None if fraction is None else round(fraction, 3),
            "live_fraction": None if live is None else round(live, 3),
            "kept": None if self.kept is None else round(self.kept, 3),
            "status": self.status(pass_fraction),
            "pass_fraction": pass_fraction,
        }


@dataclass
class GuidedTensionResult:
    completed: bool
    rounds: int
    spreads: List[dict]


@dataclass(frozen=True)
class RangeStage:
    """One finger's turn in the range check: how to pose the other fingers out
    of its way (torque off), then which joints the operator drives while the
    motors hold that pose."""

    key: str
    title: str
    joints: Tuple[str, ...]
    pose: str

    def to_dict(self) -> dict:
        return {"key": self.key, "title": self.title, "joints": list(self.joints), "pose": self.pose}


_FINGER = lambda f: (f"{f}_abd", f"{f}_mcp", f"{f}_pip")  # noqa: E731

# A finger's abduction is blocked by its neighbours, so each stage parks the
# others at the far end of theirs before the finger under test is driven.
DEFAULT_RANGE_STAGES: Tuple[RangeStage, ...] = (
    RangeStage("thumb_index", "Thumb and index",
               ("thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip") + _FINGER("index"),
               "Spread middle, ring and pinky fully away from the thumb; every thumb joint and "
               "the index abduction mid-range, index MCP and PIP bent about 45°."),
    RangeStage("middle", "Middle", _FINGER("middle"),
               "Thumb fully out; index fully toward the thumb, ring and pinky away from it; "
               "middle centred."),
    RangeStage("ring", "Ring", _FINGER("ring"),
               "Thumb fully out; index and middle fully toward the thumb, pinky away from it; "
               "ring centred."),
    RangeStage("pinky", "Pinky", _FINGER("pinky"),
               "Thumb fully out; index, middle and ring fully toward the thumb; pinky centred."),
)


def tension_targets(config: "OrcaHandConfig", *, any_hardware: bool = False,
                    absent_joints: Sequence[str] = ()) -> List[MotorSpread]:
    """One empty spread per non-wrist joint, in the hand's joint order,
    leaving out ``absent_joints`` (no motor answered for them)."""
    skip = set(absent_joints)
    return [
        MotorSpread(
            joint,
            config.joint_to_motor_map[joint],
            config.expected_motor_travel_deg(joint, any_hardware=any_hardware),
        )
        for joint in config.joint_ids
        if joint != WRIST and joint in config.joint_to_motor_map and joint not in skip
    ]


def excluded_joints(hand: "OrcaHand") -> List[dict]:
    """The score sheet's exclusion list: each absent non-wrist joint with its reason."""
    return [
        {"joint": joint, "motor": hand.config.joint_to_motor_map[joint], "reason": EXCLUDED_NO_MOTOR}
        for joint in hand.absent_joints if joint != WRIST
    ]


def pass_threshold(pass_fraction: PassFraction, joint: str) -> float:
    return float(pass_fraction(joint) if callable(pass_fraction) else pass_fraction)


def score(spreads: List[MotorSpread], pass_fraction: PassFraction = PASS_FRACTION,
          excluded: Sequence[dict] = ()) -> dict:
    """Passed / scored counts over the joints that have a baseline. ``excluded``
    names the joints the run left out, each with its reason."""
    motors = [s.to_dict(pass_threshold(pass_fraction, s.joint)) for s in spreads]
    scored = [m for m in motors if m["fraction"] is not None]
    passed = sum(m["status"] == STATUS_PASS for m in scored)
    return {
        "passed": passed,
        "scored": len(scored),
        "fraction": passed / len(scored) if scored else 0.0,
        "motors": motors,
        "excluded": list(excluded),
    }


def last_tension_path(config: "OrcaHandConfig") -> str:
    return os.path.join(os.path.dirname(config.calibration_path), LAST_TENSION_FILE)


def read_last_tension(config: "OrcaHandConfig") -> Optional[dict]:
    """The last guided tension that reached 100 %, or ``None``."""
    data = read_yaml(last_tension_path(config))
    return data or None


def record_hold_tension(config: "OrcaHandConfig") -> None:
    """Record a plain hold-and-ratchet tension, which has nothing to score."""
    write_yaml_atomic(last_tension_path(config), {
        "completed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "mode": "hold",
    })


def progress_path(config: "OrcaHandConfig") -> str:
    return os.path.join(os.path.dirname(config.calibration_path), PROGRESS_FILE)


def read_progress(config: "OrcaHandConfig", hand_key: Optional[str] = None) -> Optional[dict]:
    """Saved best score per joint for one hand (``updated_at``, ``rounds``, ``best``), or ``None``."""
    hands = (read_yaml(progress_path(config)) or {}).get("hands") or {}
    return hands.get(hand_key or DEFAULT_HAND_KEY) or None


def _write_progress(config: "OrcaHandConfig", hand_key: str, record: Optional[dict]) -> None:
    path = progress_path(config)
    hands = dict((read_yaml(path) or {}).get("hands") or {})
    if record is None:
        if hand_key not in hands:
            return
        hands.pop(hand_key)
    else:
        hands[hand_key] = record
    write_yaml_atomic(path, {"hands": hands})


def clear_progress(config: "OrcaHandConfig", hand_key: Optional[str] = None) -> None:
    _write_progress(config, hand_key or DEFAULT_HAND_KEY, None)


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("guided tension progress callback failed")


class _SpreadSampler(threading.Thread):
    """Tracks each motor's drag from ``centre`` (the round's first held pose)
    until stopped; extremes accumulate across stages, so the spread is the
    full sweep a motor made over the whole range check."""

    def __init__(self, hand, spreads, centre, pass_fraction, round_no, progress_callback,
                 excluded=()):
        super().__init__(daemon=True, name="guided-tension-sampler")
        self._hand = hand
        self._excluded = list(excluded)
        self._spreads = spreads
        self._idx = [hand.config.motor_id_to_idx_dict[s.motor_id] for s in spreads]
        self._centre = centre
        self._pass_fraction = pass_fraction
        self._round = round_no
        self._progress_callback = progress_callback
        self._stop_event = threading.Event()
        self._active = threading.Event()
        self._active.set()
        self._joints: Optional[set] = None
        self.error: Optional[BaseException] = None

    def pause(self) -> None:
        """Stop recording: posing the hand for the next stage is not range."""
        self._active.clear()

    def resume(self, joints: Optional[Sequence[str]] = None) -> None:
        """Record ``joints`` (every target when None); the others keep their last values."""
        self._joints = None if joints is None else set(joints)
        self._active.set()

    def run(self) -> None:
        try:
            while not self._stop_event.wait(SAMPLE_PERIOD_S):
                if not self._active.is_set():
                    continue
                try:
                    pos = read_motor_pos_checked(self._hand)
                except RuntimeError as exc:
                    logger.warning("%s; skipping this sample", exc)
                    continue
                for spread, idx in zip(self._spreads, self._idx):
                    if self._joints is not None and spread.joint not in self._joints:
                        continue
                    offset = math.degrees(pos[idx] - self._centre[idx])
                    spread.offset_deg = offset
                    spread.plus_deg = max(spread.plus_deg, offset)
                    spread.minus_deg = min(spread.minus_deg, offset)
                _emit(self._progress_callback, "spread", round=self._round,
                      **score(self._spreads, self._pass_fraction, self._excluded))
        except BaseException as exc:
            self.error = exc

    def stop(self) -> None:
        self._stop_event.set()
        self.join()


def joint_currents(config: "OrcaHandConfig", current: HoldCurrent,
                   absent_joints: Sequence[str] = ()) -> Dict[str, float]:
    """Holding current per present non-wrist joint, in mA."""
    return {s.joint: float(current(s.joint) if callable(current) else current)
            for s in tension_targets(config, absent_joints=absent_joints)}


def per_motor_currents(config: "OrcaHandConfig", by_joint: Dict[str, float]) -> List[float]:
    """``set_max_current`` list in motor order; motors outside ``by_joint`` get the lowest value."""
    joint_of = config.motor_to_joint_dict
    fallback = min(by_joint.values())
    return [by_joint.get(joint_of.get(m), fallback) for m in config.motor_ids]


def _hold_here(hand: "OrcaHand", motor_ids: List[int], by_joint: Dict[str, float]) -> np.ndarray:
    """Hold ``motor_ids`` where they stand under their joints' currents; return the positions."""
    hand.set_max_current(per_motor_currents(hand.config, by_joint))
    centre = read_motor_pos_checked(hand)
    idx = [hand.config.motor_id_to_idx_dict[m] for m in motor_ids]
    hand.write_motor_pos(motor_ids, centre[idx])
    hand.enable_torque(motor_ids)
    hand.write_motor_pos(motor_ids, centre[idx])
    return centre


def _stage_passed(stage: RangeStage, spreads: List[MotorSpread], pass_fraction: PassFraction) -> bool:
    """Every scored joint of ``stage`` already passes on what was kept."""
    scored = [s for s in spreads if s.joint in stage.joints and s.fraction is not None]
    return bool(scored) and all(
        s.fraction >= pass_threshold(pass_fraction, s.joint) for s in scored)


def goto_options(keys: Sequence[str], current: Optional[int] = None) -> List[str]:
    return [f"{GOTO}{k}" for j, k in enumerate(keys) if j != current]


def next_pending(passed: Sequence[bool], start: int) -> int:
    """The first stage from ``start`` on that still needs checking, or ``len(passed)``."""
    return next((j for j in range(start, len(passed)) if not passed[j]), len(passed))


def previous_pending(passed: Sequence[bool], index: int) -> Optional[int]:
    return next((j for j in range(index - 1, -1, -1) if not passed[j]), None)


def _resolve(current: Current) -> float:
    return float(current() if callable(current) else current)


def _pull_current(tension_current: "Optional[Current]", config: "OrcaHandConfig") -> float:
    """The pull's and freeze's current limit: what a caller named, else the
    connected family's own ``max_current``."""
    return float(config.max_current) if tension_current is None else _resolve(tension_current)


def _ask(prompt_callback: PromptCallback, should_stop: ShouldStop, action: str,
         options: List[str], **payload) -> str:
    answer = prompt_callback({"action": action, "options": options, **payload})
    if should_stop():
        raise InterruptedError("guided tension stopped")
    if answer not in options:
        raise ValueError(f"{action}: unexpected answer {answer!r}; expected one of {options}")
    return answer


def run_guided_tension(
    hand: "OrcaHand",
    *,
    prompt_callback: PromptCallback,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    hold_current: HoldCurrent = HOLD_CURRENT_MA,
    tension_current: Optional[Current] = TENSION_CURRENT_MA,
    tension_offset_deg: Current = TENSION_OFFSET_DEG,
    tension_extra_deg: Current = TENSION_EXTRA_DEG,
    pass_fraction: PassFraction = PASS_FRACTION,
    persist: Optional[bool] = None,
    hand_key: Optional[str] = None,
    resume: bool = True,
    stages: Sequence[RangeStage] = DEFAULT_RANGE_STAGES,
    any_hardware: bool = False,
) -> GuidedTensionResult:
    """Run centre → range check → score → tension rounds until 100 % or finish.

    Each round walks the range ``stages`` (by default one per finger; a stage
    whose joints all pass on kept scores is skipped unless jumped to). Prompts
    (``action`` → options), stage prompts carrying ``stage``, ``stage_index``
    and ``stage_count`` (its place among all ``stages``):

    - ``pose``: torque off; pose the hand as ``stage["pose"]`` says. → ``Continue``
      (``Back`` to the previous stage's pose)
    - ``range_check``: the stage's motors hold that pose at ``hold_current``
      and every other finger is frozen at ``tension_current``; drive the
      stage's joints through their range while ``spread`` events stream
      (only the stage's joints are sampled). The prompt carries the stage's
      ``hold_current`` per joint and the ``freeze_current``.
      → ``Next``, or ``Score`` on the round's last stage (``Back`` to re-pose)
    - ``score``: carries the round's score. → ``Finish`` at 100 %, else
      ``Tension`` / ``Finish anyway`` (``Back`` to the last stage)
    - ``tension``: motors hold at ``tension_current``; take up the slack with
      the ratchet tool, without overtensioning. → ``Done`` (``Pull more``
      pulls ``tension_extra_deg`` further and asks again; scores are untouched)

    Every prompt also offers ``goto:<stage key>`` for each other stage, so the
    operator can jump to any finger; spreads keep accumulating when a stage is
    revisited, and a jump from ``tension`` opens the next round on that stage.
    Stage prompts offer ``goto:score`` and ``goto:tension`` (the score so far,
    then straight to tensioning), and the score offers ``goto:tension`` too.

    Progress events: ``step`` (``step``, ``round``), ``spread`` and ``score``
    (``round``, ``passed``, ``scored``, ``fraction``, ``motors``), and
    ``cleanup_failed``.

    At the tension step the finger motors also slowly pull ``tension_offset_deg``
    counterclockwise under that current, loading the bottom spools (0 holds still);
    each pull emits ``pulled`` with ``total_deg``.

    ``hold_current`` may be a callable of the joint name, and ``tension_current``,
    ``tension_offset_deg`` and ``tension_extra_deg`` callables of nothing; each
    is read when its step starts, so a front-end
    can set holding currents per joint and change either between rounds. The
    ``range_check`` prompt carries the resolved ``hold_current`` per joint.
    ``pass_fraction`` may be a callable of the joint name, read on every
    sample, so thresholds can differ per joint and change mid-check.

    Each joint's best score is kept across rounds and, keyed by ``hand_key``
    (a hand serial or board id; one shared slot when ``None``), across runs in
    ``tension_progress.yaml``: a joint that has passed stays passed, so later
    checks only need the joints still short. ``resume=False`` starts from
    scratch and drops what was saved. Reaching 100 % clears the saved progress.

    A round that reaches 100 % is recorded in ``last_tension.yaml`` beside the
    hand's calibration (unless ``persist`` is false; mock hands default to not
    persisting). On any exit torque is disabled and the configured control
    mode and current limit are restored.

    Joints whose motor did not answer at connect (``hand.absent_joints``) are
    left out: not held, not sampled, not counted. Every score (``spread``,
    ``score`` events and the ``score`` prompt) lists them under ``excluded``
    with the reason, and stage prompts carry ``absent_joints``.

    Raises:
        ValueError: If no non-wrist joint has a motor-travel baseline: always
            the case on hardware other than v2.1 unless ``any_hardware`` is
            set, which scores an older hand against the v2.1 baseline.
        InterruptedError: If ``should_stop`` fires during a prompt.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731
    if persist is None:
        persist = getattr(hand, "_persist_calibration", True)

    config = hand.config
    absent = [j for j in hand.absent_joints if j != WRIST]
    excluded = excluded_joints(hand)
    template = tension_targets(config, any_hardware=any_hardware, absent_joints=absent)
    if absent:
        logger.warning("guided tension leaves out %s: %s", absent, EXCLUDED_NO_MOTOR)
    # A stage with no motor to drive is not a stage; one with some keeps its present joints.
    stages = [st for st in stages if any(j not in absent for j in st.joints)]
    if not stages:
        raise ValueError("no finger has a motor answering; nothing to tension")
    if not any(s.expected_deg for s in template):
        if not (config.uses_motor_travel or any_hardware):
            raise ValueError(
                f"guided tensioning needs v2.1 hardware (this hand is "
                f"{format_hardware_version(config.hardware_version)}); hold the "
                f"motors and ratchet the spools instead"
            )
        raise ValueError(
            "no joint has a motor-travel baseline; run scripts/measure_travel.py first"
        )
    motor_ids = [s.motor_id for s in template]
    control_mode = config.control_mode

    def _restore() -> None:
        hand.disable_torque()
        hand.set_max_current(config.max_current)
        hand.set_control_mode(control_mode)

    key = hand_key or DEFAULT_HAND_KEY
    saved = read_progress(config, key) if resume else None
    best: Dict[str, float] = {j: float(f) for j, f in ((saved or {}).get("best") or {}).items()
                              if f is not None}
    rounds_before = int((saved or {}).get("rounds") or 0)
    if not resume and persist:
        clear_progress(config, key)
    if best:
        _emit(progress_callback, "resumed", best=dict(best), rounds=rounds_before,
              updated_at=saved.get("updated_at"))
    template = [dataclasses.replace(s, kept=best.get(s.joint)) for s in template]

    keys = [st.key for st in stages]
    stage_of = {f"{GOTO}{k}": j for j, k in enumerate(keys)}
    round_no = 0
    completed = False
    start: Optional[int] = None
    result: dict = score(template, pass_fraction, excluded)
    try:
        hand.disable_torque()
        hand.set_control_mode(CURRENT_BASED_POSITION, motor_ids)
        while True:
            round_no += 1
            spreads = [dataclasses.replace(s, kept=best.get(s.joint)) for s in template]
            passed = [_stage_passed(st, spreads, pass_fraction) for st in stages]
            thresholds = {s.joint: pass_threshold(pass_fraction, s.joint) for s in spreads}
            sampler = None
            i = next_pending(passed, 0) if start is None else start
            try:
                while True:
                    jump = None
                    while i < len(stages):
                        stage = stages[i]
                        info = dict(stage=stage.to_dict(), stage_index=i, stage_count=len(stages),
                                    round=round_no, absent_joints=list(absent))
                        nav = goto_options(keys, i) + [GOTO_SCORE, GOTO_TENSION]
                        back = previous_pending(passed, i)
                        hand.disable_torque(motor_ids)
                        if sampler is not None:
                            sampler.pause()
                        _emit(progress_callback, "step", step="pose", **info)
                        answer = _ask(prompt_callback, should_stop, "pose",
                                      [CONTINUE] + ([BACK] if back is not None else []) + nav, **info)
                        if answer in (GOTO_SCORE, GOTO_TENSION):
                            jump = answer
                            break
                        if answer != CONTINUE:
                            i = back if answer == BACK else stage_of[answer]
                            continue

                        hold_ma = joint_currents(config, hold_current, absent)
                        # Only the stage's joints yield at the hold current; the other
                        # fingers are frozen at the tension current so a stray push
                        # cannot move them, and they are not sampled meanwhile.
                        freeze_ma = _pull_current(tension_current, config)
                        held = {j: (ma if j in stage.joints else freeze_ma) for j, ma in hold_ma.items()}
                        centre = _hold_here(hand, motor_ids, held)
                        if sampler is None:
                            sampler = _SpreadSampler(hand, spreads, centre, pass_fraction,
                                                     round_no, progress_callback, excluded)
                            sampler.start()
                        sampler.resume(stage.joints)
                        _emit(progress_callback, "step", step="range", **info)
                        last = next_pending(passed, i + 1) == len(stages)
                        answer = _ask(prompt_callback, should_stop, "range_check",
                                      [CHECK_SCORE if last else NEXT, BACK] + nav,
                                      hold_current={j: held[j] for j in hold_ma if j in stage.joints},
                                      freeze_current=freeze_ma, pass_fraction=thresholds, **info)
                        if answer in (GOTO_SCORE, GOTO_TENSION):
                            jump = answer
                            break
                        if answer != BACK:
                            i = stage_of.get(answer, next_pending(passed, i + 1))
                    if sampler is not None:
                        sampler.pause()
                        if sampler.error is not None:
                            raise sampler.error

                    result = score(spreads, pass_fraction, excluded)
                    for m in result["motors"]:
                        if m["fraction"] is not None:
                            best[m["joint"]] = max(best.get(m["joint"], 0.0), m["fraction"])
                    if persist:
                        _write_progress(config, key, {
                            "updated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                            "rounds": rounds_before + round_no,
                            "best": {j: round(f, 3) for j, f in best.items()},
                        })
                    completed = result["passed"] == result["scored"]
                    if jump == GOTO_TENSION:
                        _emit(progress_callback, "score", round=round_no, **result)
                        answer = TENSION
                        break
                    _emit(progress_callback, "step", step="score", round=round_no)
                    _emit(progress_callback, "score", round=round_no, **result)
                    options = [FINISH] if completed else [TENSION, FINISH_ANYWAY]
                    back = previous_pending(passed, len(stages))
                    options += ([BACK] if back is not None else []) + goto_options(keys) + [GOTO_TENSION]
                    answer = _ask(prompt_callback, should_stop, "score", options,
                                  round=round_no, **result)
                    if answer == GOTO_TENSION:
                        answer = TENSION
                    if answer == BACK or answer in stage_of:
                        i = back if answer == BACK else stage_of[answer]
                        continue
                    break
            finally:
                if sampler is not None:
                    sampler.stop()
            if answer != TENSION:
                break

            pull_ma = _pull_current(tension_current, config)
            pulled = _resolve(tension_offset_deg)
            _emit(progress_callback, "step", step="tension", round=round_no)

            def pull_then_hold(offset, start):
                """Pull at ``pull_ma``, settle, hold the reached pose, then uncap.

                The ratcheting hold runs at each motor's own ceiling: at its goal
                a motor draws nothing, so the ceiling only binds while the
                operator pushes, and that is the whole budget it has to refuse
                being turned with. The goal is re-baselined onto the position
                actually reached first, because a standing error under an
                uncapped ceiling sits there drawing it indefinitely.
                """
                hand.set_max_current(pull_ma)
                if offset:
                    pull_tendons(hand, motor_ids, offset, start=start, should_stop=should_stop)
                # Also with no pull: a jump here can arrive with torque off,
                # and the motors must hold.
                reached = settle_at_rest(hand, motor_ids, should_stop=should_stop)
                hand.set_max_current(uncapped_currents(hand))
                return reached

            goals = pull_then_hold(pulled, None)
            while True:
                _emit(progress_callback, "pulled", total_deg=pulled)
                answer = _ask(prompt_callback, should_stop, "tension",
                              [DONE, PULL_MORE] + goto_options(keys), round=round_no,
                              pull_current=pull_ma, pulled_deg=pulled)
                if answer != PULL_MORE:
                    break
                extra = _resolve(tension_extra_deg)
                goals = pull_then_hold(extra, goals)
                pulled += extra
            start = stage_of.get(answer)
    except BaseException:
        try:
            _restore()
        except Exception as e:
            _emit(progress_callback, "cleanup_failed", error=str(e))
            logger.warning("cleanup after aborted guided tension failed: %s", e)
        raise
    _restore()

    if completed and persist:
        clear_progress(config, key)
        write_yaml_atomic(last_tension_path(config), {
            "completed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "rounds": round_no,
            "pass_fraction": {m["joint"]: m["pass_fraction"] for m in result["motors"]},
            "spreads": {m["joint"]: m["fraction"] for m in result["motors"]},
            "excluded": result["excluded"],
        })
    return GuidedTensionResult(completed, round_no, result["motors"])
