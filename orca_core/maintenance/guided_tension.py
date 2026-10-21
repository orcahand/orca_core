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

Two scorings share the routine (``scoring``):

``fraction``
    The spread as a share of the expected travel; a joint passes at
    ``pass_fraction``. Ends are recorded on every sample. Each joint's best
    fraction is kept across rounds and sessions, so a joint that once passed
    stays passed and a stage whose joints all pass is skipped.

``slack``
    Expected travel minus the spread, in motor degrees; a joint passes when that
    shortfall is within ``pass_slack_deg`` and both ends were reached twice.
    Only settled positions count as an end, so a fast flick that overshoots the
    hardstop does not inflate the spread. Every round scores its own sweep —
    nothing is carried over — and each stage is measured from its own held pose.

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
from .tensioning import TENSION_EXTRA_DEG, TENSION_OFFSET_DEG, pull_tendons

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
PassSlack = Union[float, Callable[[str], float]]  # degrees, one value or one per joint name
Threshold = Union[float, Callable[[str], float]]

SCORING_FRACTION = "fraction"
SCORING_SLACK = "slack"
SCORINGS = (SCORING_FRACTION, SCORING_SLACK)
PASS_FRACTION = 0.85
PASS_SLACK_DEG = 4.0  # expected travel minus confirmed spread, motor degrees
PARTIAL_FRACTION = 0.3
SAMPLE_PERIOD_S = 0.05
# Slack scoring: an end counts once the motor has moved less than SETTLE_DEG
# between SETTLE_SAMPLES consecutive samples, so overshoot settles back first.
SETTLE_SAMPLES = 4
SETTLE_DEG = 1.0
# A second visit to within REPEAT_TOL_DEG of an end, after leaving it by at
# least AWAY_FRACTION of the travel (or AWAY_MIN_DEG), confirms that end.
REPEAT_TOL_DEG = 2.0
AWAY_FRACTION = 0.25
AWAY_MIN_DEG = 5.0
LAST_TENSION_FILE = "last_tension.yaml"
PROGRESS_FILE = "tension_progress.yaml"
DEFAULT_HAND_KEY = "default"

CONTINUE = "Continue"
NEXT = "Next"
CHECK_SCORE = "Score"
TENSION = "Tension"
# Score step with a check hook: run the tension check first, then tension.
CHECK = "Check & tension"
SKIP_CHECK = "Skip check"
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
STATUS_UNCONFIRMED = "unconfirmed"  # slack: within tolerance, but an end was reached only once
STATUS_SLACK = "slack"  # slack: a tension probe measured slack above the tolerance on this joint
STATUS_FAULT = "fault"  # a latched hardware error: the motor answers but never energizes
FAULT_RECHECK_S = 1.0

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
    kept: Optional[float] = None  # fraction: best fraction from earlier rounds or sessions
    plus_hits: int = 0  # slack: settled visits to the + end (2 confirm it)
    minus_hits: int = 0
    probe_slack_deg: Optional[float] = None  # slack: what a tension probe measured before this round
    fault: Optional[List[str]] = None  # latched error bits while the motor is limp

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

    @property
    def slack_deg(self) -> Optional[float]:
        """Travel the sweep did not reach, in motor degrees: the tendons' slack."""
        if not self.expected_deg:
            return None
        return self.expected_deg - self.spread_deg

    @property
    def confirmed(self) -> bool:
        return self.plus_hits >= 2 and self.minus_hits >= 2

    def status(self, threshold: float, scoring: str = SCORING_FRACTION) -> str:
        if self.fault:
            return STATUS_FAULT
        if scoring == SCORING_SLACK:
            slack = self.slack_deg
            if slack is None:
                return STATUS_UNSCORED
            if self.probe_slack_deg is not None and self.probe_slack_deg > threshold:
                return STATUS_SLACK
            if slack <= threshold:
                return STATUS_PASS if self.confirmed else STATUS_UNCONFIRMED
            return STATUS_PARTIAL if (self.live_fraction or 0.0) >= PARTIAL_FRACTION else STATUS_LOW
        fraction = self.fraction
        if fraction is None:
            return STATUS_UNSCORED
        if fraction >= threshold:
            return STATUS_PASS
        return STATUS_PARTIAL if fraction >= PARTIAL_FRACTION else STATUS_LOW

    def to_dict(self, threshold: float, scoring: str = SCORING_FRACTION) -> dict:
        fraction = self.fraction
        live = self.live_fraction
        slack = self.slack_deg
        slack_mode = scoring == SCORING_SLACK
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
            "slack_deg": None if slack is None else round(slack, 2),
            "confirmed": self.confirmed,
            "plus_hits": self.plus_hits,
            "minus_hits": self.minus_hits,
            "probe_slack_deg": (
                None if self.probe_slack_deg is None else round(self.probe_slack_deg, 2)
            ),
            "status": self.status(threshold, scoring),
            "scoring": scoring,
            "pass_fraction": None if slack_mode else threshold,
            "pass_slack_deg": threshold if slack_mode else None,
            "fault": list(self.fault) if self.fault else None,
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
        return {
            "key": self.key,
            "title": self.title,
            "joints": list(self.joints),
            "pose": self.pose,
        }


_FINGER = lambda f: (f"{f}_abd", f"{f}_mcp", f"{f}_pip")  # noqa: E731

# A finger's abduction is blocked by its neighbours, so each stage parks the
# others at the far end of theirs before the finger under test is driven.
DEFAULT_RANGE_STAGES: Tuple[RangeStage, ...] = (
    RangeStage(
        "thumb",
        "Thumb",
        ("thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"),
        "Spread index, middle, ring and pinky fully away from the thumb; every thumb "
        "joint mid-range.",
    ),
    RangeStage(
        "index",
        "Index",
        _FINGER("index"),
        "Thumb fully out; middle, ring and pinky fully away from the thumb; "
        "index centred.",
    ),
    RangeStage(
        "middle",
        "Middle",
        _FINGER("middle"),
        "Thumb fully out; index fully toward the thumb, ring and pinky away from it; "
        "middle centred.",
    ),
    RangeStage(
        "ring",
        "Ring",
        _FINGER("ring"),
        "Thumb fully out; index and middle fully toward the thumb, pinky away from it; "
        "ring centred.",
    ),
    RangeStage(
        "pinky",
        "Pinky",
        _FINGER("pinky"),
        "Thumb fully out; index, middle and ring fully toward the thumb; pinky centred.",
    ),
)


def tension_targets(
    config: "OrcaHandConfig",
    *,
    any_hardware: bool = False,
    absent_joints: Sequence[str] = (),
) -> List[MotorSpread]:
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
        {
            "joint": joint,
            "motor": hand.config.joint_to_motor_map[joint],
            "reason": EXCLUDED_NO_MOTOR,
        }
        for joint in hand.absent_joints
        if joint != WRIST
    ]


def pass_threshold(threshold: Threshold, joint: str) -> float:
    return float(threshold(joint) if callable(threshold) else threshold)


def default_threshold(scoring: str) -> float:
    return PASS_SLACK_DEG if scoring == SCORING_SLACK else PASS_FRACTION


def score(
    spreads: List[MotorSpread],
    threshold: Optional[Threshold] = None,
    excluded: Sequence[dict] = (),
    scoring: str = SCORING_FRACTION,
) -> dict:
    """Passed / scored counts over the joints that have a baseline. ``excluded``
    names the joints the run left out, each with its reason."""
    if threshold is None:
        threshold = default_threshold(scoring)
    motors = [s.to_dict(pass_threshold(threshold, s.joint), scoring) for s in spreads]
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
    write_yaml_atomic(
        last_tension_path(config),
        {
            "completed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "mode": "hold",
        },
    )


def progress_path(config: "OrcaHandConfig") -> str:
    return os.path.join(os.path.dirname(config.calibration_path), PROGRESS_FILE)


def read_progress(
    config: "OrcaHandConfig",
    hand_key: Optional[str] = None,
    scoring: str = SCORING_FRACTION,
) -> Optional[dict]:
    """One hand's saved progress for ``scoring``, or ``None``.

    Fraction: ``updated_at``, ``rounds`` and the best fraction per joint under
    ``best``. Slack: ``updated_at``, ``rounds`` and the last round's ``scores``
    per joint (``slack_deg``, ``fraction``, ``status``), kept in the record's
    ``slack`` part so the two scorings never see each other's numbers."""
    hands = (read_yaml(progress_path(config)) or {}).get("hands") or {}
    record = hands.get(hand_key or DEFAULT_HAND_KEY) or {}
    if scoring == SCORING_SLACK:
        return record.get("slack") or None
    view = {k: v for k, v in record.items() if k != "slack"}
    return view or None


def _write_progress(
    config: "OrcaHandConfig",
    hand_key: str,
    record: Optional[dict],
    scoring: Optional[str] = SCORING_FRACTION,
) -> None:
    """Replace the ``scoring`` part of one hand's record (``None`` drops it);
    ``scoring=None`` drops the whole record."""
    path = progress_path(config)
    hands = dict((read_yaml(path) or {}).get("hands") or {})
    current = dict(hands.get(hand_key) or {})
    if scoring is None:
        current = {}
    elif scoring == SCORING_SLACK:
        current.pop("slack", None)
        if record is not None:
            current["slack"] = record
    else:
        slack = current.get("slack")
        current = dict(record or {})
        if slack:
            current["slack"] = slack
    if current:
        hands[hand_key] = current
    elif hand_key in hands:
        hands.pop(hand_key)
    else:
        return
    write_yaml_atomic(path, {"hands": hands})


def clear_progress(
    config: "OrcaHandConfig",
    hand_key: Optional[str] = None,
    scoring: Optional[str] = None,
) -> None:
    """Forget one hand's progress: the ``scoring`` part, or all of it when ``None``."""
    _write_progress(config, hand_key or DEFAULT_HAND_KEY, None, scoring)


def _emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
    if progress_callback is None:
        return
    try:
        progress_callback({"event": event, **payload})
    except Exception:
        logger.exception("guided tension progress callback failed")


def latched_faults(hand: "OrcaHand", motor_ids: Sequence[int]) -> Dict[int, List[str]]:
    """``motor_ids`` whose Hardware Error Status has bits latched, with the bit
    names. A latched motor still answers the bus and still acks torque enable;
    it just never energizes, so without this check it reads as holding."""
    client = hand.motor_client
    read = getattr(client, "read_hardware_errors", None)
    if client is None or read is None or not motor_ids:
        return {}
    try:
        raw = read(list(motor_ids))
    except Exception:
        logger.debug("hardware error sweep failed", exc_info=True)
        return {}
    faults = {}
    for mid, value in raw.items():
        flags = client.decode_hardware_error(value)
        if flags:
            faults[int(mid)] = list(flags)
    return faults


def reboot_and_hold(hand: "OrcaHand", motor_id: int) -> Optional[List[str]]:
    """Reboot one latched motor and hold it again where it stands.

    The reboot clears the latch and the client puts the motor's goal current
    back; torque is then re-enabled under the routine's own hold. Returns the
    bits still latched afterwards (empty when cleared), or ``None`` if the
    motor did not answer after the reboot.
    """
    client = hand.motor_client
    reboot = getattr(client, "reboot_motor", None)
    if reboot is None:
        raise RuntimeError("this motor family has no reboot instruction")
    motor_id = int(motor_id)
    reboot(motor_id)
    try:
        pos = read_motor_pos_checked(hand)
    except RuntimeError:
        return None
    goal = pos[[hand.config.motor_id_to_idx_dict[motor_id]]]
    hand.write_motor_pos([motor_id], goal)
    hand.enable_torque([motor_id])
    hand.write_motor_pos([motor_id], goal)
    return latched_faults(hand, [motor_id]).get(motor_id, [])


class _FaultWatch:
    """Latched hardware errors over the run: each reported once as ``fault``,
    its clearing (a reboot by the front-end) as ``fault_cleared``. A faulted
    motor's spread stops accumulating until it is cleared."""

    def __init__(self, hand, progress_callback):
        self._hand = hand
        self._progress_callback = progress_callback
        self._lock = threading.Lock()
        self.faults: Dict[int, List[str]] = {}
        self._spreads: List[MotorSpread] = []
        self._joint_of = hand.config.motor_to_joint_dict

    def bind(self, spreads: List[MotorSpread]) -> None:
        """Each round's fresh spreads inherit the faults still latched."""
        with self._lock:
            self._spreads = spreads
            for s in spreads:
                s.fault = self.faults.get(s.motor_id)

    def sweep(self, motor_ids: Sequence[int]) -> None:
        """Read every motor's error register (after a torque enable or a pull)."""
        found = latched_faults(self._hand, motor_ids)
        self._apply(found, set(motor_ids))

    def poll(self) -> None:
        """Cheap: the Alert bits that rode along in the sampler's own reads."""
        take = getattr(self._hand.motor_client, "take_hardware_alerts", None)
        alerted = [m for m in (take() if take is not None else {}) if m not in self.faults]
        if alerted:
            self._apply(latched_faults(self._hand, alerted), set(alerted))

    def recheck(self) -> None:
        """Re-read the faulted motors: a reboot clears the register."""
        if self.faults:
            self._apply(latched_faults(self._hand, list(self.faults)), set(self.faults))

    def _apply(self, found: Dict[int, List[str]], asked: set) -> None:
        with self._lock:
            events = []
            for mid in asked:
                flags = found.get(mid)
                if flags and flags != self.faults.get(mid):
                    self.faults[mid] = flags
                    events.append(("fault", mid, flags))
                elif not flags and mid in self.faults:
                    del self.faults[mid]
                    events.append(("fault_cleared", mid, None))
            for s in self._spreads:
                s.fault = self.faults.get(s.motor_id)
        for event, mid, flags in events:
            joint = self._joint_of.get(mid)
            if event == "fault":
                logger.warning("motor %d (%s) has latched %s; it is not holding", mid, joint, flags)
                _emit(self._progress_callback, event, motor=mid, joint=joint, flags=flags)
            else:
                _emit(self._progress_callback, event, motor=mid, joint=joint)


class _FaultSweeper(threading.Thread):
    """Re-reads the error register while no sampler runs (the tension step):
    a motor stalled at the tension current latches there, and a reboot from
    the front-end clears there."""

    def __init__(self, watch: _FaultWatch, motor_ids: Sequence[int]):
        super().__init__(daemon=True, name="guided-tension-fault-sweep")
        self._watch = watch
        self._motor_ids = list(motor_ids)
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.wait(FAULT_RECHECK_S):
            try:
                self._watch.sweep(self._motor_ids)
            except Exception:
                logger.debug("fault sweep failed", exc_info=True)

    def stop(self) -> None:
        self._stop_event.set()
        self.join()


@dataclass
class _EndTracker:
    """Per-motor sampler state (slack scoring): settling, and whether each end
    has been left since it was last reached (so a return can count as a second visit)."""

    last: float = 0.0
    still: int = 0
    plus_away: bool = False
    minus_away: bool = False


def record_sample(
    spread: MotorSpread, track: _EndTracker, offset: float, settle: bool = False
) -> None:
    """Fold one position sample (motor degrees from the held centre) into
    ``spread``: the live offset always; an end on every sample (fraction
    scoring), or only once the motor has settled and with visits counted
    (``settle``, slack scoring)."""
    spread.offset_deg = offset
    if not settle:
        spread.plus_deg = max(spread.plus_deg, offset)
        spread.minus_deg = min(spread.minus_deg, offset)
        return
    track.still = track.still + 1 if abs(offset - track.last) <= SETTLE_DEG else 0
    track.last = offset
    away = max(AWAY_MIN_DEG, AWAY_FRACTION * (spread.expected_deg or 0.0))
    if offset < spread.plus_deg - away:
        track.plus_away = True
    if offset > spread.minus_deg + away:
        track.minus_away = True
    if track.still < SETTLE_SAMPLES - 1:
        return
    if offset > spread.plus_deg + REPEAT_TOL_DEG:
        spread.plus_deg, spread.plus_hits, track.plus_away = offset, 1, False
    elif offset >= spread.plus_deg - REPEAT_TOL_DEG:
        spread.plus_deg = max(spread.plus_deg, offset)
        if track.plus_away or spread.plus_hits == 0:
            spread.plus_hits += 1
            track.plus_away = False
    if offset < spread.minus_deg - REPEAT_TOL_DEG:
        spread.minus_deg, spread.minus_hits, track.minus_away = offset, 1, False
    elif offset <= spread.minus_deg + REPEAT_TOL_DEG:
        spread.minus_deg = min(spread.minus_deg, offset)
        if track.minus_away or spread.minus_hits == 0:
            spread.minus_hits += 1
            track.minus_away = False


class _SpreadSampler(threading.Thread):
    """Tracks each motor's drag from ``centre`` until stopped.

    Fraction scoring: the centre is the round's first held pose and extremes
    accumulate across stages, so the spread is the full sweep a motor made over
    the whole range check. Slack scoring: a stage's joints start over from the
    pose held for that stage (``resume(..., centre)``), only settled positions
    become ends, and each end wants two visits (see :func:`record_sample`)."""

    def __init__(
        self,
        hand,
        spreads,
        centre,
        threshold,
        round_no,
        progress_callback,
        excluded=(),
        watch: Optional[_FaultWatch] = None,
        scoring: str = SCORING_FRACTION,
    ):
        super().__init__(daemon=True, name="guided-tension-sampler")
        self._hand = hand
        self._watch = watch
        self._excluded = list(excluded)
        self._spreads = spreads
        self._idx = [hand.config.motor_id_to_idx_dict[s.motor_id] for s in spreads]
        self._tracks = [_EndTracker() for _ in spreads]
        self._centre = np.array(centre, dtype=float)
        self._rebase: Optional[Tuple[np.ndarray, Optional[set]]] = None
        self._threshold = threshold
        self._scoring = scoring
        self._settle = scoring == SCORING_SLACK
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

    def resume(
        self, joints: Optional[Sequence[str]] = None, centre: Optional[np.ndarray] = None
    ) -> None:
        """Record ``joints`` (every target when None); the others keep their last
        values. With ``centre`` (the pose just held for this stage, slack scoring)
        those joints start over from it: the pose step moved them with torque
        off, so offsets from an earlier stage's centre would not be their range."""
        self._joints = None if joints is None else set(joints)
        if centre is not None:
            self._rebase = (centre, self._joints)
        self._active.set()

    def _apply_rebase(self) -> None:
        pending, self._rebase = self._rebase, None
        if pending is None:
            return
        centre, joints = pending
        for n, (spread, idx) in enumerate(zip(self._spreads, self._idx)):
            if joints is not None and spread.joint not in joints:
                continue
            self._centre[idx] = centre[idx]
            spread.offset_deg = spread.plus_deg = spread.minus_deg = 0.0
            spread.plus_hits = spread.minus_hits = 0
            self._tracks[n] = _EndTracker()

    def run(self) -> None:
        try:
            next_recheck = time.monotonic() + FAULT_RECHECK_S
            while not self._stop_event.wait(SAMPLE_PERIOD_S):
                if not self._active.is_set():
                    continue
                self._apply_rebase()
                try:
                    pos = read_motor_pos_checked(self._hand)
                except RuntimeError as exc:
                    logger.warning("%s; skipping this sample", exc)
                    continue
                if self._watch is not None:
                    self._watch.poll()
                    if time.monotonic() >= next_recheck:
                        next_recheck = time.monotonic() + FAULT_RECHECK_S
                        self._watch.recheck()
                for spread, idx, track in zip(self._spreads, self._idx, self._tracks):
                    if self._joints is not None and spread.joint not in self._joints:
                        continue
                    if spread.fault:
                        continue
                    record_sample(
                        spread,
                        track,
                        math.degrees(pos[idx] - self._centre[idx]),
                        self._settle,
                    )
                _emit(
                    self._progress_callback,
                    "spread",
                    round=self._round,
                    **score(self._spreads, self._threshold, self._excluded, self._scoring),
                )
        except BaseException as exc:
            self.error = exc

    def stop(self) -> None:
        self._stop_event.set()
        self.join()


def joint_currents(
    config: "OrcaHandConfig",
    current: Optional[HoldCurrent] = None,
    absent_joints: Sequence[str] = (),
) -> Dict[str, float]:
    """Holding current per present non-wrist joint, in mA. ``None`` takes the config's
    hold currents; a joint the config names keeps its own current either way."""
    if current is None:
        current = config.tension_hold_current_for
    fixed = config.tension_hold_joint_currents
    return {
        s.joint: float(
            fixed.get(s.joint) or (current(s.joint) if callable(current) else current)
        )
        for s in tension_targets(config, absent_joints=absent_joints)
    }


def per_motor_currents(
    config: "OrcaHandConfig", by_joint: Dict[str, float]
) -> List[float]:
    """``set_max_current`` list in motor order; motors outside ``by_joint`` get the lowest value."""
    joint_of = config.motor_to_joint_dict
    fallback = min(by_joint.values())
    return [by_joint.get(joint_of.get(m), fallback) for m in config.motor_ids]


def _hold_here(
    hand: "OrcaHand", motor_ids: List[int], by_joint: Dict[str, float]
) -> np.ndarray:
    """Hold ``motor_ids`` where they stand under their joints' currents; return the positions."""
    hand.set_max_current(per_motor_currents(hand.config, by_joint))
    centre = read_motor_pos_checked(hand)
    idx = [hand.config.motor_id_to_idx_dict[m] for m in motor_ids]
    hand.write_motor_pos(motor_ids, centre[idx])
    hand.enable_torque(motor_ids)
    hand.write_motor_pos(motor_ids, centre[idx])
    return centre


def _stage_passed(
    stage: RangeStage, spreads: List[MotorSpread], threshold: Threshold, scoring: str
) -> bool:
    """Every scored joint of ``stage`` already passes (on what was kept, so
    only fraction scoring ever skips a stage)."""
    scored = [s for s in spreads if s.joint in stage.joints and s.fraction is not None]
    return bool(scored) and all(
        s.status(pass_threshold(threshold, s.joint), scoring) == STATUS_PASS for s in scored
    )


def goto_options(keys: Sequence[str], current: Optional[int] = None) -> List[str]:
    return [f"{GOTO}{k}" for j, k in enumerate(keys) if j != current]


def next_pending(passed: Sequence[bool], start: int) -> int:
    """The first stage from ``start`` on that still needs checking, or ``len(passed)``."""
    return next((j for j in range(start, len(passed)) if not passed[j]), len(passed))


def previous_pending(passed: Sequence[bool], index: int) -> Optional[int]:
    return next((j for j in range(index - 1, -1, -1) if not passed[j]), None)


def _resolve(current: Current) -> float:
    return float(current() if callable(current) else current)


def _run_check(
    hand, motor_ids, check_callback, prompt_callback, progress_callback, should_stop, round_no
) -> None:
    """Torque off so the operator can pose the hand clear of its stops, then run
    the check on it and put the motors back in current-based position mode."""
    hand.disable_torque(motor_ids)
    _emit(progress_callback, "step", step="check_pose", round=round_no)
    answer = _ask(prompt_callback, should_stop, "check_pose", [CONTINUE, SKIP_CHECK], round=round_no)
    if answer == SKIP_CHECK:
        return
    _emit(progress_callback, "step", step="check", round=round_no)
    try:
        check_callback(hand)
    finally:
        hand.disable_torque(motor_ids)
        hand.set_control_mode(CURRENT_BASED_POSITION, motor_ids)


def _ask(
    prompt_callback: PromptCallback,
    should_stop: ShouldStop,
    action: str,
    options: List[str],
    **payload,
) -> str:
    answer = prompt_callback({"action": action, "options": options, **payload})
    if should_stop():
        raise InterruptedError("guided tension stopped")
    if answer not in options:
        raise ValueError(
            f"{action}: unexpected answer {answer!r}; expected one of {options}"
        )
    return answer


def run_guided_tension(
    hand: "OrcaHand",
    *,
    prompt_callback: PromptCallback,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
    hold_current: Optional[HoldCurrent] = None,
    tension_current: Optional[Current] = None,
    tension_offset_deg: Current = TENSION_OFFSET_DEG,
    tension_extra_deg: Current = TENSION_EXTRA_DEG,
    scoring: str = SCORING_FRACTION,
    pass_fraction: PassFraction = PASS_FRACTION,
    pass_slack_deg: PassSlack = PASS_SLACK_DEG,
    probe_slack_deg: Optional[Dict[str, float]] = None,
    persist: Optional[bool] = None,
    hand_key: Optional[str] = None,
    resume: bool = True,
    stages: Sequence[RangeStage] = DEFAULT_RANGE_STAGES,
    any_hardware: bool = False,
    check_callback: Optional[Callable[["OrcaHand"], None]] = None,
    check_first: bool = False,
) -> GuidedTensionResult:
    """Run centre → range check → score → tension rounds until 100 % or finish.

    Each round walks the range ``stages`` (by default one per finger; with
    fraction scoring a stage whose joints all pass on kept scores is skipped
    unless jumped to). Prompts (``action`` → options), stage prompts carrying
    ``stage``, ``stage_index`` and ``stage_count`` (its place among all ``stages``):

    - ``pose``: torque off; pose the hand as ``stage["pose"]`` says. → ``Continue``
      (``Back`` to the previous stage's pose)
    - ``range_check``: the stage's motors hold that pose at ``hold_current``
      and every other finger is frozen at ``tension_current``; drive the
      stage's joints through their range while ``spread`` events stream
      (only the stage's joints are sampled). The prompt carries the stage's
      ``hold_current`` per joint and the ``freeze_current``.
      → ``Next``, or ``Score`` on the round's last stage (``Back`` to re-pose)
    - ``score``: carries the round's score. → ``Finish`` at 100 %, else
      ``Tension`` / ``Finish anyway`` (``Back`` to the last stage). With a
      ``check_callback`` the first option is ``Check & tension`` instead:
      torque goes off, a ``check_pose`` prompt (→ ``Continue`` / ``Skip check``)
      lets the operator pose the hand clear of its stops, then the callback
      runs on the hand (a tension check: the console's breakaway probe) under
      a ``check`` step, and the tension step follows. With ``check_first`` the
      first round opens on that check and its ratchet before any finger is
      moved; the range stages follow in round two.
    - ``tension``: motors hold at ``tension_current``; take up the slack with
      the ratchet tool, without overtensioning. → ``Done`` (``Pull more``
      pulls ``tension_extra_deg`` further and asks again; scores are untouched)

    Every prompt also offers ``goto:<stage key>`` for each other stage, so the
    operator can jump to any finger (fraction: spreads keep accumulating when a
    stage is revisited; slack: it is swept afresh from its newly held pose), and
    a jump from ``tension`` opens the next round on that stage.
    Stage prompts offer ``goto:score`` and ``goto:tension`` (the score so far,
    then straight to tensioning), and the score offers ``goto:tension`` too.

    Progress events: ``step`` (``step``, ``round``), ``spread`` and ``score``
    (``round``, ``passed``, ``scored``, ``fraction``, ``motors``),
    ``fault`` / ``fault_cleared`` (``motor``, ``joint``, ``flags``) and
    ``cleanup_failed``.

    A motor whose Hardware Error Status latches (overload, a supply dip, ...)
    keeps answering the bus and acking torque enable but never energizes, so
    the operator feels no resistance and its spread would be meaningless. The
    register is swept after every hold and pull and the Alert bit watched on
    every sample: a latched motor is reported as ``fault``, its spread stops
    accumulating and it scores ``fault`` (never ``pass``) until the register
    clears — :func:`reboot_and_hold` does that from another thread while a
    prompt blocks — which is reported as ``fault_cleared``.

    At the tension step the finger motors also slowly pull ``tension_offset_deg``
    counterclockwise under that current, loading the bottom spools (0 holds still);
    each pull emits ``pulled`` with ``total_deg``.

    ``hold_current`` and ``tension_current`` default to the config's
    ``guided_tension_currents``; a joint named there keeps its own hold current.
    ``hold_current`` may be a callable of the joint name, and ``tension_current``,
    ``tension_offset_deg`` and ``tension_extra_deg`` callables of nothing; each
    is read when its step starts, so a front-end
    can set holding currents per joint and change either between rounds. The
    ``range_check`` prompt carries the resolved ``hold_current`` per joint.
    ``scoring`` picks the rule (see the module docstring). ``pass_fraction``
    (fraction) or ``pass_slack_deg`` (slack) is the threshold; either may be a
    callable of the joint name, read on every sample, so thresholds can differ
    per joint and change mid-check. The ``range_check`` prompt carries the
    resolved thresholds under that name.

    Fraction: each joint's best score is kept across rounds and, keyed by
    ``hand_key`` (a hand serial or board id; one shared slot when ``None``),
    across runs in ``tension_progress.yaml``: a joint that has passed stays
    passed, so later checks only need the joints still short.

    Slack: ``probe_slack_deg`` is slack a tension probe measured per joint
    before this run (motor degrees): in the first round a joint above its
    tolerance there scores ``slack`` whatever its sweep says, so the probe's
    finding is tensioned out; it is dropped once the ratchet has been used.
    Every round is scored on its own sweep alone; the last round's scores and
    the round count are saved per ``hand_key`` in the record's ``slack`` part.

    Either way ``resume=False`` starts from scratch and drops what that scoring
    had saved, and reaching 100 % clears it.

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

    def _tension_ma() -> float:
        return _resolve(
            config.tension_current if tension_current is None else tension_current
        )

    def _restore() -> None:
        hand.disable_torque()
        hand.set_max_current(config.max_current)
        hand.set_control_mode(control_mode)

    if scoring not in SCORINGS:
        raise ValueError(f"scoring must be one of {SCORINGS}, not {scoring!r}")
    slack_mode = scoring == SCORING_SLACK
    threshold: Threshold = pass_slack_deg if slack_mode else pass_fraction

    key = hand_key or DEFAULT_HAND_KEY
    saved = read_progress(config, key, scoring) if resume else None
    best: Dict[str, float] = {}
    if not slack_mode:
        best = {
            j: float(f)
            for j, f in ((saved or {}).get("best") or {}).items()
            if f is not None
        }
    rounds_before = int((saved or {}).get("rounds") or 0)
    if not resume and persist:
        clear_progress(config, key, scoring)
    if slack_mode and saved:
        _emit(
            progress_callback,
            "resumed",
            scores=dict(saved.get("scores") or {}),
            rounds=rounds_before,
            updated_at=saved.get("updated_at"),
        )
    elif best:
        _emit(
            progress_callback,
            "resumed",
            best=dict(best),
            rounds=rounds_before,
            updated_at=saved.get("updated_at"),
        )
    template = [dataclasses.replace(s, kept=best.get(s.joint)) for s in template]
    probe = {}
    if slack_mode:
        probe = {j: float(v) for j, v in (probe_slack_deg or {}).items() if v is not None}

    keys = [st.key for st in stages]
    stage_of = {f"{GOTO}{k}": j for j, k in enumerate(keys)}
    round_no = 0
    completed = False
    start: Optional[int] = None
    result: dict = score(template, threshold, excluded, scoring)
    watch = _FaultWatch(hand, progress_callback)
    try:
        hand.disable_torque()
        hand.set_control_mode(CURRENT_BASED_POSITION, motor_ids)
        while True:
            round_no += 1
            spreads = [
                dataclasses.replace(
                    s, kept=best.get(s.joint), probe_slack_deg=probe.get(s.joint)
                )
                for s in template
            ]
            watch.bind(spreads)
            passed = [_stage_passed(st, spreads, threshold, scoring) for st in stages]
            thresholds = {s.joint: pass_threshold(threshold, s.joint) for s in spreads}
            threshold_key = "pass_slack_deg" if slack_mode else "pass_fraction"
            sampler = None
            i = next_pending(passed, 0) if start is None else start
            first = check_first and check_callback is not None and round_no == 1
            try:
                while True:
                    jump = None
                    if first:
                        # Straight to the check: no stage this round, nothing to score.
                        first = False
                        i = len(stages)
                        jump = CHECK
                    while i < len(stages):
                        stage = stages[i]
                        info = dict(
                            stage=stage.to_dict(),
                            stage_index=i,
                            stage_count=len(stages),
                            round=round_no,
                            absent_joints=list(absent),
                        )
                        nav = goto_options(keys, i) + [GOTO_SCORE, GOTO_TENSION]
                        back = previous_pending(passed, i)
                        hand.disable_torque(motor_ids)
                        if sampler is not None:
                            sampler.pause()
                        _emit(progress_callback, "step", step="pose", **info)
                        answer = _ask(
                            prompt_callback,
                            should_stop,
                            "pose",
                            [CONTINUE] + ([BACK] if back is not None else []) + nav,
                            **info,
                        )
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
                        freeze_ma = _tension_ma()
                        held = {
                            j: (ma if j in stage.joints else freeze_ma)
                            for j, ma in hold_ma.items()
                        }
                        centre = _hold_here(hand, motor_ids, held)
                        watch.sweep(motor_ids)
                        if sampler is None:
                            sampler = _SpreadSampler(
                                hand,
                                spreads,
                                centre,
                                threshold,
                                round_no,
                                progress_callback,
                                excluded,
                                watch,
                                scoring,
                            )
                            sampler.start()
                            sampler.resume(stage.joints)
                        else:
                            sampler.resume(stage.joints, centre if slack_mode else None)
                        _emit(progress_callback, "step", step="range", **info)
                        last = next_pending(passed, i + 1) == len(stages)
                        answer = _ask(
                            prompt_callback,
                            should_stop,
                            "range_check",
                            [CHECK_SCORE if last else NEXT, BACK] + nav,
                            hold_current={
                                j: held[j] for j in hold_ma if j in stage.joints
                            },
                            freeze_current=freeze_ma,
                            **{threshold_key: thresholds},
                            **info,
                        )
                        if answer in (GOTO_SCORE, GOTO_TENSION):
                            jump = answer
                            break
                        if answer != BACK:
                            i = stage_of.get(answer, next_pending(passed, i + 1))
                    if sampler is not None:
                        sampler.pause()
                        if sampler.error is not None:
                            raise sampler.error

                    result = score(spreads, threshold, excluded, scoring)
                    record = {
                        "updated_at": datetime.now(timezone.utc).isoformat(
                            timespec="seconds"
                        ),
                        "rounds": rounds_before + round_no,
                    }
                    if slack_mode:
                        record["scores"] = {
                            m["joint"]: {k: m[k] for k in ("slack_deg", "fraction", "status")}
                            for m in result["motors"]
                            if m["slack_deg"] is not None
                        }
                    else:
                        for m in result["motors"]:
                            if m["fraction"] is not None:
                                best[m["joint"]] = max(best.get(m["joint"], 0.0), m["fraction"])
                        record["best"] = {j: round(f, 3) for j, f in best.items()}
                    if persist and jump != CHECK:
                        _write_progress(config, key, record, scoring)
                    completed = result["passed"] == result["scored"]
                    if jump in (GOTO_TENSION, CHECK):
                        _emit(progress_callback, "score", round=round_no, **result)
                        answer = TENSION if jump == GOTO_TENSION else CHECK
                        break
                    _emit(progress_callback, "step", step="score", round=round_no)
                    _emit(progress_callback, "score", round=round_no, **result)
                    options = (
                        [FINISH] if completed
                        else [CHECK if check_callback is not None else TENSION, FINISH_ANYWAY]
                    )
                    back = previous_pending(passed, len(stages))
                    options += (
                        ([BACK] if back is not None else [])
                        + goto_options(keys)
                        + [GOTO_TENSION]
                    )
                    answer = _ask(
                        prompt_callback,
                        should_stop,
                        "score",
                        options,
                        round=round_no,
                        **result,
                    )
                    if answer == GOTO_TENSION:
                        answer = TENSION
                    if answer == BACK or answer in stage_of:
                        i = back if answer == BACK else stage_of[answer]
                        continue
                    break
            finally:
                if sampler is not None:
                    sampler.stop()
            if answer == CHECK:
                _run_check(
                    hand, motor_ids, check_callback, prompt_callback, progress_callback,
                    should_stop, round_no,
                )
                answer = TENSION
            if answer != TENSION:
                break

            tension_ma = _tension_ma()
            pulled = _resolve(tension_offset_deg)
            hand.set_max_current(tension_ma)
            _emit(progress_callback, "step", step="tension", round=round_no)
            # Also with no pull: a jump here can arrive with torque off, and the motors must hold.
            goals = pull_tendons(hand, motor_ids, pulled, should_stop=should_stop)
            watch.sweep(motor_ids)
            sweeper = _FaultSweeper(watch, motor_ids)
            sweeper.start()
            try:
                while True:
                    _emit(progress_callback, "pulled", total_deg=pulled)
                    answer = _ask(
                        prompt_callback,
                        should_stop,
                        "tension",
                        [DONE, PULL_MORE] + goto_options(keys),
                        round=round_no,
                        tension_current=tension_ma,
                        pulled_deg=pulled,
                    )
                    if answer != PULL_MORE:
                        break
                    extra = _resolve(tension_extra_deg)
                    goals = pull_tendons(
                        hand, motor_ids, extra, start=goals, should_stop=should_stop
                    )
                    pulled += extra
            finally:
                sweeper.stop()
            probe = {}
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
        clear_progress(config, key, scoring)
        motors = result["motors"]
        last = {
            "completed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "rounds": round_no,
            "scoring": scoring,
            "pass_fraction": {m["joint"]: m["pass_fraction"] for m in motors},
            "spreads": {m["joint"]: m["fraction"] for m in motors},
            "excluded": result["excluded"],
        }
        if slack_mode:
            last.update(
                pass_slack_deg={m["joint"]: m["pass_slack_deg"] for m in motors},
                slack_deg={m["joint"]: m["slack_deg"] for m in motors},
                spread_deg={m["joint"]: m["spread_deg"] for m in motors},
                expected_deg={m["joint"]: m["expected_deg"] for m in motors},
            )
            del last["pass_fraction"]
        write_yaml_atomic(last_tension_path(config), last)
    return GuidedTensionResult(completed, round_no, result["motors"])
