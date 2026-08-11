"""Finds the gain at which a joint's control loop starts oscillating.

The loop ships with a proportional gain of half what the theory says is
marginally stable, and it does oscillate when that gain is raised to one. So the
question is not why it oscillates — that much is expected of any loop closed
around a plant it can drive faster than it samples. The question is how much room
each joint actually has, and whether the shipped value is in the right place for
it. This is a measurement of that room, one joint at a time.

The joint under test keeps its gains; every other joint in the loop is set to
zero gain so the motion measured is one loop's, not seventeen. A zero-gain
channel holds no integral state, so those joints are left running feed-forward
only, on the motor's own position control.

Three things about the measurement are easy to get wrong and are worth stating,
because each one turns a stability measurement into something that looks like one:

**A marginally stable loop sits still if nothing disturbs it.** So each dwell is
excited with a slow, small sine, and the reading is taken from the quiet stretch
after the excitation stops. An oscillation still going then is the loop's own.

**The correction clamp bounds the very thing being measured.** A loop past its
margin with a clamped correction settles into a bounded limit cycle rather than
running away, so amplitude growth never arrives and the amplitude that does
arrive is the clamp's. Dwells that spend their time against the clamp are
reported as censored and no onset is read from them.

**Ringing near half the loop rate is invisible in the loop's own record.** It
aliases there to a steady offset, which reads as a joint sitting quietly off
target. Every amplitude and frequency here is taken from the encoder frame
stream, which runs five times faster than the loop.

Interaction-free, like the routines in ``orca_core.maintenance``: progress goes
out through ``progress_callback``, cancellation comes in through ``should_stop``.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

from orca_core.constants import WRIST

from ..analysis import oscillation as osc
from ..preconditions import (
    PreconditionError,
    require_closed_loop,
    require_healthy_encoders,
    require_no_motion_profile,
)
from ..record import JointAngleWindow, LoopRecorder, RecordingSession, collect_metadata
from ..record.linkstats import LinkStatsRecorder


logger = logging.getLogger(__name__)

ProgressCallback = Callable[[Dict[str, Any]], None]
ShouldStop = Callable[[], bool]

# Every duration is a module constant so a test can shorten it without a clock
# of its own. These are the ones a hand is run at.
BASELINE_EXCITE_S = 20.0
BASELINE_QUIET_S = 10.0
DWELL_EXCITE_S = 4.0
DWELL_QUIET_S = 5.0
SETTLE_S = 1.0
RECOVERY_S = 1.0

# Skipped at the start of every quiet stretch. The joint is still coming back
# from the last of the excitation, and that decay is motion the excitation
# caused. What is measured begins after it, so what is left is the loop's own.
QUIET_SETTLE_S = 1.0

# Slow and small: two full cycles inside a dwell, and a peak speed of a few
# degrees a second, which is far below anything the joint has to work to follow.
EXCITE_AMPLITUDE_DEG = 2.0
EXCITE_HZ = 0.5
APPROACH_DEG_PER_S = 15.0

COMMAND_INTERVAL_S = 0.02
LINK_SAMPLE_INTERVAL_S = 1.0

# A dwell that is already oscillating has said what it has to say, and every
# further second of it is amplitude the joint did not need to reach. The first
# check waits long enough for the excitation's own settling to have passed.
ONSET_CHECK_AFTER_S = 1.5
ONSET_CHECK_INTERVAL_S = 0.5

# How far back to look when a dwell is cut short: the stretch that ended it,
# not all of it. A dwell aborts while the excitation is still running, so a
# window covering the whole of it reports a swing that is partly the
# excitation's — and that swing is what gets compared against the joint's quiet
# baseline.
ABORT_MEASURE_S = 1.5

# Geometric, because margin is a ratio: a fifth more gain is the same step
# whether the gain is 0.5 or 2. The ceilings are well past where the linear
# analysis puts the limit, so a joint that never oscillates says something.
RAMP_FACTOR = 1.25
MAX_KP = 4.0
MAX_KI = 60.0

# The joint is driven to the middle of its travel and kept clear of both ends,
# so a growing oscillation runs out of amplitude before it runs out of joint.
ROM_MARGIN_DEG = 2.0
MIN_EXCITE_AMPLITUDE_DEG = 0.5

# Abort thresholds. Tracking error and travel are per-sample; the swing is over
# a sliding window, which catches a fast oscillation that stays near the target.
MAX_TRACKING_ERROR_DEG = 8.0
SWING_WINDOW_S = 0.4
MAX_SWING_DEG = 10.0
CLAMP_FRACTION_OF_MAX = 0.98

# A dwell whose frames arrived worse than this was measured through a stream
# that was itself misbehaving, and its amplitudes are not the joint's alone.
MAX_FRAME_GAP_P99_MS = 15.0

ARMS = ("kp", "ki")


class DwellAborted(RuntimeError):
    """A dwell was cut short. The message is the reason, and it is recorded."""


class StopRequested(DwellAborted):
    """The operator asked to stop. Distinct because everything else that cuts a
    dwell short is something the joint did, and what it did is still worth
    measuring."""


@dataclass
class Dwell:
    """One gain value, held and then measured.

    The first dwell of a joint has no verdict: it is the noise floor every
    later verdict is taken against, and there is nothing yet to compare it to.
    """

    index: int
    kp: float
    ki: float
    oscillation: Optional[osc.Oscillation] = None
    verdict: Optional[osc.Verdict] = None
    clamp_fraction: float = 0.0
    frame_gap_p99_ms: float = float("nan")
    frames: int = 0
    aborted: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index,
            "kp": round(self.kp, 4),
            "ki": round(self.ki, 4),
            "clamp_fraction": round(self.clamp_fraction, 4),
            "frame_gap_p99_ms": (
                round(self.frame_gap_p99_ms, 3)
                if np.isfinite(self.frame_gap_p99_ms)
                else None
            ),
            "frames": self.frames,
            "aborted": self.aborted,
            "measured": self.oscillation.as_dict() if self.oscillation else None,
            "verdict": self.verdict.as_dict() if self.verdict else None,
        }


@dataclass
class JointMargin:
    """What the ramp found for one joint."""

    joint: str
    arm: str
    base_pose_deg: float
    excite_amplitude_deg: float
    baseline_kp: float
    baseline_ki: float
    baseline_p2p_deg: float = float("nan")
    dwells: List[Dwell] = field(default_factory=list)
    onset_value: Optional[float] = None
    # The last gain that stayed quiet and the first that did not. The onset is
    # somewhere between them and the ramp's step size is all that is known
    # about where — reporting the upper one alone would read as a measurement
    # of the gain rather than of the interval it was found in.
    onset_bracket: Optional[List[float]] = None
    onset_frequency_hz: Optional[float] = None
    outcome: str = "not run"
    reason: str = ""

    def as_dict(self) -> Dict[str, Any]:
        return {
            "joint": self.joint,
            "arm": self.arm,
            "base_pose_deg": round(self.base_pose_deg, 3),
            "excite_amplitude_deg": round(self.excite_amplitude_deg, 3),
            "baseline_kp": self.baseline_kp,
            "baseline_ki": self.baseline_ki,
            "baseline_p2p_deg": (
                round(self.baseline_p2p_deg, 4)
                if np.isfinite(self.baseline_p2p_deg)
                else None
            ),
            "onset_value": self.onset_value,
            "onset_bracket": self.onset_bracket,
            "onset_frequency_hz": (
                round(self.onset_frequency_hz, 2)
                if self.onset_frequency_hz is not None
                else None
            ),
            "outcome": self.outcome,
            "reason": self.reason,
            "dwells": [dwell.as_dict() for dwell in self.dwells],
        }


def run_gain_margin(
    hand,
    window: JointAngleWindow,
    *,
    output_dir: Path | str,
    ritual,
    joints: Optional[Sequence[str]] = None,
    arm: str = "kp",
    frame_recorder=None,
    max_current_ma: Optional[int] = None,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> Dict[str, Any]:
    """Ramp one gain per joint until the loop oscillates, and write it down.

    The hand must already be connected with the loop running, and ``window``
    must have been installed before it connected — the encoder client is built
    during ``connect`` and cannot be given another consumer afterwards.

    Args:
        hand: A connected hand with joint feedback engaged.
        window: Live joint angles at frame rate; every amplitude and frequency
            in the result is measured from it.
        output_dir: Parent directory; a run-named subdirectory is created.
        ritual: What was done to the tendons before the run. Required: this
            commands the hand, and tension state moves everything measured here.
        joints: Joints to probe, defaulting to every joint the loop controls.
            The wrist is never probed — it runs an uncapped control mode.
        arm: ``"kp"`` ramps proportional gain with the integrator switched off,
            which is the clean linear measurement. ``"ki"`` ramps integral gain
            at the configured proportional gain, which is where a nonlinear
            mechanism shows itself.
        frame_recorder: Optional recorder whose table is added to the dataset.
        max_current_ma: Motor current ceiling for the run. Restored to the
            configured value afterwards.
        progress_callback: Receives ``run_started``, ``joint_started``,
            ``baseline_measured``, ``dwell_done``, ``onset``, ``joint_done``,
            ``run_done``, ``run_aborted``.
        should_stop: Polled throughout; returns the hand to its base pose and
            stops.

    Returns:
        The run's results: the dataset directory and one entry per joint.
    """
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}, got {arm!r}")
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    require_closed_loop(hand)
    loop = hand._loop
    probe_joints = _resolve_joints(hand, joints)
    require_healthy_encoders(hand._encoder_client, probe_joints)
    profile_velocity = require_no_motion_profile(hand)

    params = {
        "arm": arm,
        "joints": list(probe_joints),
        "excite_amplitude_deg": EXCITE_AMPLITUDE_DEG,
        "excite_hz": EXCITE_HZ,
        "dwell_excite_s": DWELL_EXCITE_S,
        "dwell_quiet_s": DWELL_QUIET_S,
        "baseline_excite_s": BASELINE_EXCITE_S,
        "baseline_quiet_s": BASELINE_QUIET_S,
        "ramp_factor": RAMP_FACTOR,
        "max_kp": MAX_KP,
        "max_ki": MAX_KI,
        "max_current_ma": max_current_ma,
    }
    metadata = collect_metadata(
        hand, "gain_margin", params, moves_hand=True, ritual=ritual
    )
    metadata["motor_profile_velocity"] = profile_velocity

    baseline_gains = hand.get_pid_gains()
    directory = Path(output_dir) / metadata["run_id"]
    loop_recorder = LoopRecorder(loop.joint_names)
    link_recorder = _link_recorder(hand)

    _emit(progress_callback, "run_started", arm=arm, joints=list(probe_joints))
    margins: List[JointMargin] = []

    with RecordingSession(directory, experiment="gain_margin", metadata=metadata) as session:
        session.add_table("loop", loop_recorder.ring, loop_recorder.columns, source=loop_recorder)
        if frame_recorder is not None:
            session.add_table(
                "frames", frame_recorder.ring, frame_recorder.columns, source=frame_recorder
            )
        if link_recorder is not None:
            session.add_table(
                "link", link_recorder.ring, link_recorder.columns, source=link_recorder
            )
        loop.attach_observer(loop_recorder)
        try:
            if max_current_ma is not None:
                _set_current(hand, max_current_ma, session)
            for joint in probe_joints:
                margin = _probe_joint(
                    hand,
                    window,
                    session,
                    joint=joint,
                    arm=arm,
                    baseline_gains=baseline_gains,
                    link_recorder=link_recorder,
                    progress_callback=progress_callback,
                    should_stop=should_stop,
                )
                margins.append(margin)
                _emit(progress_callback, "joint_done", **margin.as_dict())
                if should_stop():
                    session.record_event("stopped_early", reason="should_stop")
                    break
                if hand.get_loop_stats()["fallback_active"]:
                    # There is no way to restart a loop that has e-stopped, so
                    # every joint after this one would measure the open-loop
                    # hand and report it as margin.
                    session.record_event("loop_estopped", after=joint)
                    _emit(progress_callback, "run_aborted", reason="the loop e-stopped")
                    break
        finally:
            loop.detach_observer()
            _restore(hand, baseline_gains, max_current_ma, session)

    result = {
        "directory": str(directory),
        "arm": arm,
        "joints": [margin.as_dict() for margin in margins],
    }
    _emit(progress_callback, "run_done", **result)
    return result


def _probe_joint(
    hand,
    window: JointAngleWindow,
    session: RecordingSession,
    *,
    joint: str,
    arm: str,
    baseline_gains,
    link_recorder,
    progress_callback,
    should_stop,
) -> JointMargin:
    """Measure one joint: isolate it, take its noise floor, then ramp."""
    gains = baseline_gains[joint]
    lower, upper = _travel(hand, joint)
    base_pose = 0.5 * (lower + upper)
    # A quarter of the travel at most, so the excitation's own extremes stay
    # well inside the range an oscillation would have to cross to abort.
    amplitude = min(EXCITE_AMPLITUDE_DEG, 0.25 * (upper - lower))

    margin = JointMargin(
        joint=joint,
        arm=arm,
        base_pose_deg=base_pose,
        excite_amplitude_deg=amplitude,
        baseline_kp=gains.kp,
        baseline_ki=gains.ki,
    )
    if amplitude < MIN_EXCITE_AMPLITUDE_DEG:
        margin.outcome = "skipped"
        margin.reason = (
            f"only {upper - lower:.1f}° of travel is usable, which is too little "
            "to excite the loop without running into an end"
        )
        session.record_event("joint_skipped", joint=joint, reason=margin.reason)
        return margin

    _emit(
        progress_callback,
        "joint_started",
        joint=joint,
        base_pose_deg=round(base_pose, 2),
        amplitude_deg=round(amplitude, 2),
        kp=gains.kp,
        ki=gains.ki,
    )
    session.record_event(
        "joint_started",
        joint=joint,
        base_pose_deg=base_pose,
        amplitude_deg=amplitude,
        kp=gains.kp,
        ki=gains.ki,
    )

    _isolate(hand, joint, baseline_gains, session)
    try:
        guard = _Guard(hand, window, joint, (lower, upper), gains.correction_max_deg, should_stop)
        try:
            _approach(hand, window, joint, base_pose, guard, link_recorder)
        except DwellAborted as abort:
            margin.outcome = "aborted"
            margin.reason = f"on the way to the base pose: {abort}"
            session.record_event("approach_aborted", joint=joint, reason=str(abort))
            return margin

        baseline = _dwell(
            hand,
            window,
            session,
            joint=joint,
            base_pose=base_pose,
            amplitude=amplitude,
            kp=gains.kp,
            ki=gains.ki,
            index=0,
            excite_s=BASELINE_EXCITE_S,
            quiet_s=BASELINE_QUIET_S,
            baseline_p2p=None,
            guard=guard,
            link_recorder=link_recorder,
        )
        margin.dwells.append(baseline)
        if baseline.aborted:
            margin.outcome = "aborted"
            margin.reason = f"the baseline dwell aborted: {baseline.aborted}"
            return margin

        margin.baseline_p2p_deg = baseline.oscillation.peak_to_peak_deg
        _emit(
            progress_callback,
            "baseline_measured",
            joint=joint,
            p2p_deg=round(margin.baseline_p2p_deg, 4),
            frame_gap_p99_ms=round(baseline.frame_gap_p99_ms, 2),
        )
        if baseline.frame_gap_p99_ms > MAX_FRAME_GAP_P99_MS:
            margin.outcome = "unusable"
            margin.reason = (
                f"frames arrived up to {baseline.frame_gap_p99_ms:.1f} ms apart "
                "at the baseline, which is the loop's own staleness threshold; "
                "what this joint does under gain cannot be separated from that"
            )
            session.record_event("joint_unusable", joint=joint, reason=margin.reason)
            return margin

        _ramp(
            hand,
            window,
            session,
            margin=margin,
            gains=gains,
            base_pose=base_pose,
            amplitude=amplitude,
            arm=arm,
            guard=guard,
            link_recorder=link_recorder,
            progress_callback=progress_callback,
        )
    finally:
        _recover(hand, window, joint, base_pose, baseline_gains, session)
    return margin


def _ramp(
    hand,
    window,
    session,
    *,
    margin: JointMargin,
    gains,
    base_pose: float,
    amplitude: float,
    arm: str,
    guard,
    link_recorder,
    progress_callback,
) -> None:
    """Walk the gain up until the joint oscillates or the ceiling is reached."""
    joint = margin.joint
    value = float("nan")
    last_quiet = None
    for index, (kp, ki) in enumerate(_ramp_gains(gains, arm), start=1):
        dwell = _dwell(
            hand,
            window,
            session,
            joint=joint,
            base_pose=base_pose,
            amplitude=amplitude,
            kp=kp,
            ki=ki,
            index=index,
            excite_s=DWELL_EXCITE_S,
            quiet_s=DWELL_QUIET_S,
            baseline_p2p=margin.baseline_p2p_deg,
            guard=guard,
            link_recorder=link_recorder,
        )
        margin.dwells.append(dwell)
        _emit(progress_callback, "dwell_done", joint=joint, **dwell.as_dict())

        value = kp if arm == "kp" else ki
        verdict = dwell.verdict
        if dwell.aborted and not (verdict and verdict.oscillating):
            margin.outcome = "aborted"
            margin.reason = f"at {arm}={value:.3f}: {dwell.aborted}"
            return
        if verdict.state == "censored":
            margin.outcome = "censored"
            margin.reason = f"at {arm}={value:.3f}: {verdict.reason}"
            session.record_event("censored", joint=joint, value=value, reason=verdict.reason)
            return
        if verdict.oscillating:
            margin.onset_value = round(value, 4)
            margin.onset_bracket = [
                round(last_quiet, 4) if last_quiet is not None else None,
                round(value, 4),
            ]
            margin.onset_frequency_hz = verdict.oscillation.frequency_hz
            margin.outcome = "onset"
            margin.reason = verdict.reason
            if dwell.aborted:
                margin.reason += f"; the dwell was cut short: {dwell.aborted}"
            session.record_event(
                "onset",
                joint=joint,
                arm=arm,
                value=value,
                frequency_hz=verdict.oscillation.frequency_hz,
                peak_to_peak_deg=verdict.oscillation.peak_to_peak_deg,
                cut_short=dwell.aborted,
            )
            _emit(
                progress_callback,
                "onset",
                joint=joint,
                arm=arm,
                value=round(value, 4),
                bracket=margin.onset_bracket,
                frequency_hz=round(verdict.oscillation.frequency_hz, 2),
            )
            return
        last_quiet = value

    margin.outcome = "no onset"
    margin.reason = (
        f"held together to {arm}={value:.3f}, the ceiling this ramp stops at"
        if np.isfinite(value)
        else f"its configured {arm} is already past the ceiling this ramp stops at"
    )


def _dwell(
    hand,
    window,
    session,
    *,
    joint: str,
    base_pose: float,
    amplitude: float,
    kp: float,
    ki: float,
    index: int,
    excite_s: float,
    quiet_s: float,
    baseline_p2p: Optional[float],
    guard,
    link_recorder,
) -> Dwell:
    """Install a gain, shake the joint, then watch what it does on its own.

    The reading comes from the quiet stretch: an oscillation still running once
    the excitation has stopped is the loop sustaining it, and one that dies away
    was the excitation being followed.
    """
    session.record_event("gain_change", joint=joint, kp=kp, ki=ki, index=index)
    hand.set_pid_gains(Kp={joint: kp}, Ki={joint: ki})
    # Whatever the integrator banked under the previous gain would otherwise be
    # spent under this one, and the dwell would measure the handover.
    hand._loop.reset_integral()
    guard.reset()

    dwell = Dwell(index=index, kp=kp, ki=ki)
    try:
        session.record_event("excitation_started", joint=joint, index=index)
        _hold(
            hand,
            joint,
            lambda elapsed: base_pose + amplitude * math.sin(2 * math.pi * EXCITE_HZ * elapsed),
            excite_s,
            guard,
            link_recorder,
        )
        session.record_event("excitation_stopped", joint=joint, index=index)
        measure_from = _quiet(
            hand,
            window,
            joint,
            base_pose,
            quiet_s,
            guard,
            link_recorder,
            baseline_p2p,
        )
    except DwellAborted as abort:
        dwell.aborted = str(abort)
        dwell.clamp_fraction = guard.clamp_fraction
        # A dwell cut short because the joint was moving too much has already
        # answered the question the dwell was asking. Measuring what it did up
        # to that point is what keeps the safety limit from throwing away the
        # measurement it fired on.
        if not isinstance(abort, StopRequested):
            _measure(
                dwell,
                window,
                joint,
                base_pose,
                max(guard.since, time.monotonic() - ABORT_MEASURE_S),
                baseline_p2p,
            )
        session.record_event(
            "dwell_aborted",
            joint=joint,
            index=index,
            reason=str(abort),
            measured=dwell.oscillation.as_dict() if dwell.oscillation else None,
        )
        return dwell

    dwell.clamp_fraction = guard.clamp_fraction
    _measure(dwell, window, joint, base_pose, measure_from, baseline_p2p)
    session.record_event(
        "dwell_measured",
        joint=joint,
        index=index,
        measured=dwell.oscillation.as_dict(),
        verdict=dwell.verdict.as_dict() if dwell.verdict else None,
    )
    return dwell


def _measure(
    dwell: Dwell,
    window,
    joint: str,
    base_pose: float,
    since: float,
    baseline_p2p: Optional[float],
) -> None:
    """Read what the joint did over one stretch, and judge it if there is a
    baseline to judge it against."""
    now = time.monotonic()
    times, angles = window.column(joint, now - since, now=now)
    dwell.frames = int(times.size)
    dwell.frame_gap_p99_ms = _gap_p99_ms(times)
    dwell.oscillation = osc.describe(times, angles - base_pose)
    if baseline_p2p is not None:
        dwell.verdict = osc.onset_verdict(
            dwell.oscillation, baseline_p2p, dwell.clamp_fraction
        )


def _quiet(
    hand,
    window,
    joint: str,
    base_pose: float,
    quiet_s: float,
    guard,
    link_recorder,
    baseline_p2p: Optional[float],
) -> float:
    """Hold the joint still and watch, cutting it short once it is oscillating.

    Waiting out the full dwell would let an oscillation that has already been
    established keep growing for no more information than it has already given.
    Returns when the stretch started, which is what the measurement is taken
    over.
    """
    started = time.monotonic()
    measure_from = started + QUIET_SETTLE_S
    next_check = started + ONSET_CHECK_AFTER_S
    next_link = started
    while True:
        now = time.monotonic()
        if now - started >= quiet_s:
            return measure_from
        hand.set_joint_positions({joint: base_pose})
        guard.check(base_pose)
        if link_recorder is not None and now >= next_link:
            link_recorder.sample(now)
            next_link += LINK_SAMPLE_INTERVAL_S
        if baseline_p2p is not None and now >= next_check:
            next_check = now + ONSET_CHECK_INTERVAL_S
            times, angles = window.column(joint, now - measure_from, now=now)
            verdict = osc.onset_verdict(
                osc.describe(times, angles - base_pose),
                baseline_p2p,
                guard.clamp_fraction,
            )
            if verdict.state in ("oscillating", "censored"):
                return measure_from
        time.sleep(COMMAND_INTERVAL_S)


def _hold(hand, joint: str, target_at, duration_s: float, guard, link_recorder) -> None:
    """Drive the joint along ``target_at`` for ``duration_s``, watching it."""
    started = time.monotonic()
    next_link = started
    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= duration_s:
            return
        target = target_at(elapsed)
        hand.set_joint_positions({joint: target})
        guard.check(target)
        if link_recorder is not None and now >= next_link:
            link_recorder.sample(now)
            next_link += LINK_SAMPLE_INTERVAL_S
        time.sleep(COMMAND_INTERVAL_S)


def _approach(hand, window, joint: str, target_deg: float, guard, link_recorder) -> None:
    """Walk the joint to its base pose at a fixed speed and let it settle.

    Ramped rather than commanded outright: the loop would otherwise see a step
    the size of the whole approach, and its response to that is Stage 3's
    measurement, not this one's.
    """
    latest = window.latest()
    start = latest.get(joint) if latest else None
    if start is None or not math.isfinite(start):
        raise PreconditionError(
            f"no encoder angle for {joint}; the stream is not delivering it and "
            "nothing can be measured on it"
        )
    distance = abs(target_deg - start)
    duration = distance / APPROACH_DEG_PER_S if distance > 0 else 0.0
    guard.reset(moving=True)
    if duration > 0:
        _hold(
            hand,
            joint,
            lambda elapsed: start + (target_deg - start) * min(1.0, elapsed / duration),
            duration,
            guard,
            link_recorder,
        )
    _hold(hand, joint, lambda elapsed: target_deg, SETTLE_S, guard, link_recorder)

    # Checked once, at the end: during the move the joint is legitimately
    # behind, and a joint that is still behind after settling is one whose
    # margin cannot be measured because it is not where it was put.
    settled = (window.latest() or {}).get(joint)
    if settled is None or not math.isfinite(settled):
        raise DwellAborted(f"no encoder angle for {joint} after settling")
    if abs(settled - target_deg) > MAX_TRACKING_ERROR_DEG:
        raise DwellAborted(
            f"{joint} settled {settled - target_deg:+.1f}° from its base pose"
        )
    guard.reset()


class _Guard:
    """Watches one joint while it is driven, and says when to stop.

    Every check is against the frame-rate record rather than the loop's, for the
    same reason the measurement is: a fast oscillation is invisible at loop rate
    until it is large. The clamp count is the exception — the correction is the
    loop's own state and only it knows it — so the fraction reported is over the
    samples this guard took, not over cycles.
    """

    def __init__(self, hand, window, joint, bounds, correction_max_deg, should_stop):
        self._hand = hand
        self._window = window
        self._joint = joint
        self._lower, self._upper = bounds
        self._clamp_at = CLAMP_FRACTION_OF_MAX * correction_max_deg
        self._should_stop = should_stop
        self._moving = False
        self._since = time.monotonic()
        self._samples = 0
        self._clamped = 0

    @property
    def clamp_fraction(self) -> float:
        return self._clamped / self._samples if self._samples else 0.0

    @property
    def since(self) -> float:
        """When the phase being watched began."""
        return self._since

    def reset(self, moving: bool = False) -> None:
        """Start counting again.

        ``moving`` says the joint is being walked somewhere on purpose, which
        makes lagging behind the command and covering ground in a hurry both
        expected. Where it may go, and whether the loop is still running, are
        checked either way.
        """
        self._moving = moving
        self._since = time.monotonic()
        self._samples = 0
        self._clamped = 0

    def check(self, target_deg: float) -> None:
        if self._should_stop():
            raise StopRequested("stopped by request")
        if self._hand.get_loop_stats()["fallback_active"]:
            raise DwellAborted("the loop e-stopped and is no longer controlling")

        try:
            correction = self._hand.get_loop_correction().get(self._joint)
        except RuntimeError:
            # The loop e-stopped between the check above and this call.
            raise DwellAborted("the loop e-stopped and is no longer controlling") from None
        self._samples += 1
        if correction is not None and abs(correction) >= self._clamp_at:
            self._clamped += 1

        times, angles = self._window.column(self._joint, SWING_WINDOW_S)
        # Only what has happened since this phase began: the window still holds
        # the move that got the joint here, and a move is not a swing.
        angles = angles[times >= self._since]
        if angles.size == 0:
            return
        latest = angles[-1]
        if math.isfinite(latest):
            if latest < self._lower or latest > self._upper:
                raise DwellAborted(
                    f"{self._joint} reached {latest:.1f}°, outside the "
                    f"{self._lower:.1f}–{self._upper:.1f}° it was kept to"
                )
            if not self._moving and abs(latest - target_deg) > MAX_TRACKING_ERROR_DEG:
                raise DwellAborted(
                    f"{self._joint} is {latest - target_deg:+.1f}° from where it "
                    "was told to be"
                )
        if self._moving:
            return
        swing = osc.peak_to_peak(angles)
        if np.isfinite(swing) and swing > MAX_SWING_DEG:
            raise DwellAborted(
                f"{self._joint} swung {swing:.1f}° within {SWING_WINDOW_S:.1f}s"
            )


def _recover(hand, window, joint: str, base_pose: float, baseline_gains, session) -> None:
    """Take the joint out of whatever it was doing and give it back.

    Order matters. Zeroing the gains stops the loop asking for more, and it
    takes effect on the next cycle without touching the bus. Anything that stops
    the loop instead — the watchdog, a disconnect — leaves the motor holding the
    last large correction it was sent, with nothing left running to take it back.
    Torque stays on throughout: a finger whose torque is cut falls onto whatever
    is below it.
    """
    try:
        hand.set_pid_gains(Kp={joint: 0.0}, Ki={joint: 0.0})
        hand.set_joint_positions({joint: base_pose})
        time.sleep(RECOVERY_S)
        gains = baseline_gains[joint]
        hand.set_pid_gains(
            Kp={joint: gains.kp},
            Ki={joint: gains.ki},
            correction_max_deg={joint: gains.correction_max_deg},
        )
    except Exception as error:
        logger.exception("failed to hand %s back", joint)
        session.record_event("cleanup_failed", joint=joint, error=str(error))


def _isolate(hand, joint: str, baseline_gains, session) -> None:
    """Silence every other joint's loop so the motion measured is one joint's."""
    others = {name: 0.0 for name in baseline_gains if name != joint}
    hand.set_pid_gains(Kp=others, Ki=dict(others))
    session.record_event("isolated", joint=joint, zeroed=sorted(others))


def _restore(hand, baseline_gains, max_current_ma, session) -> None:
    """Put the gains and the current ceiling back the way they were found."""
    try:
        hand.set_pid_gains(
            Kp={joint: gains.kp for joint, gains in baseline_gains.items()},
            Ki={joint: gains.ki for joint, gains in baseline_gains.items()},
            correction_max_deg={
                joint: gains.correction_max_deg for joint, gains in baseline_gains.items()
            },
        )
    except Exception as error:
        logger.exception("failed to restore the loop gains")
        session.record_event("cleanup_failed", what="gains", error=str(error))
    if max_current_ma is not None:
        try:
            _set_current(hand, hand.config.max_current, session)
        except Exception as error:
            logger.exception("failed to restore the motor current limit")
            session.record_event("cleanup_failed", what="max_current", error=str(error))


def _set_current(hand, milliamps: int, session) -> None:
    """Write a current ceiling without colliding with the loop's own writes."""
    with hand._loop_writes_paused():
        hand.set_max_current(milliamps)
    session.record_event("max_current", milliamps=milliamps)


def _ramp_gains(gains, arm: str):
    """The (Kp, Ki) pairs a ramp visits, in order.

    The proportional arm switches the integrator off: ramping with it live moves
    two things at once, and the onset could belong to either. The integral arm
    leaves the proportional gain where the hand ships it, because what it is
    looking for is the gain at which the integrator starts hunting, which is a
    property of the pair.
    """
    if arm == "kp":
        value, ceiling = gains.kp, MAX_KP
    else:
        value, ceiling = gains.ki, MAX_KI
    while value <= ceiling:
        yield (value, 0.0) if arm == "kp" else (gains.kp, value)
        value *= RAMP_FACTOR


def _resolve_joints(hand, joints: Optional[Sequence[str]]) -> List[str]:
    """Which joints to probe, and refuse rather than silently drop any.

    The wrist is excluded whatever is asked for: it runs a position mode with no
    current ceiling, so a joint driven into oscillation there has nothing
    limiting what it can do to itself.
    """
    controlled = [name for name in hand._loop.joint_names if name != WRIST]
    if joints is None:
        return controlled
    requested = list(joints)
    if WRIST in requested:
        raise PreconditionError(
            "the wrist runs an uncapped position mode, so there is no current "
            "ceiling to limit what an oscillation there could do. It is not "
            "probed."
        )
    unknown = [name for name in requested if name not in controlled]
    if unknown:
        raise PreconditionError(
            f"the loop is not controlling {sorted(unknown)}; it controls "
            f"{controlled}"
        )
    return requested


def _travel(hand, joint: str) -> tuple[float, float]:
    """The range this joint may be driven over.

    Commands are clamped to the config range while the map is evaluated in the
    measured one, so only where they overlap does a commanded degree mean what
    it says. The margin then keeps a growing oscillation from reaching an end.
    """
    config_lower, config_upper = hand.config.joint_roms_dict[joint]
    effective_lower, effective_upper = hand.effective_joint_roms_dict[joint]
    lower = max(config_lower, effective_lower) + ROM_MARGIN_DEG
    upper = min(config_upper, effective_upper) - ROM_MARGIN_DEG
    return lower, upper


def _gap_p99_ms(times: np.ndarray) -> float:
    if times.size < 3:
        return float("nan")
    return float(np.percentile(np.diff(times) * 1000.0, 99))


def _link_recorder(hand):
    link = getattr(hand, "_encoder_link", None)
    client = getattr(hand, "_encoder_client", None)
    if link is None or client is None:
        return None
    return LinkStatsRecorder(link, client)


def _emit(callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the run."""
    if callback is None:
        return
    try:
        callback({"event": event, **payload})
    except Exception:
        logger.exception("gain margin progress callback failed")
