"""The gain-margin probe, driven against a synthetic joint.

The hand here is the mock joint-feedback hand with its constant frame pump
replaced by one fed from a plant, so the whole path the bench uses is exercised:
frames go over the mock link, the real encoder client decodes them, the real
loop closes around them, and the probe sees them through the same window it
would on hardware. What is synthetic is the mechanism, and its parameters are
known — which is the only way to check that a probe looking for instability
finds it where it is and not where it is not.
"""

from __future__ import annotations

import os
import shutil
import threading
import time

import numpy as np
import pytest

from orca_core.hardware_hand_sensing import MockOrcaHandJointFeedback

from studies.experiments import gain_margin
from studies.experiments.gain_margin import run_gain_margin
from studies.plant import PlantEncoderSource, PlantJoint
from studies.preconditions import PreconditionError
from studies.record import Dataset, FrameRecorder, JointAngleWindow, Ritual, record_frames
from studies.record.metadata import RitualNotDeclared

from tests._encoder_helpers import make_encoder_frame


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)

JOINT = "index_mcp"
PUMP_HZ = 500.0

# Slow enough to ring at a frequency a short dwell can resolve, and delayed
# enough that the loop loses its margin within a step or two of the ramp.
RINGS = PlantJoint(natural_frequency_hz=12.0, damping_ratio=1.0, dead_time_s=0.05)
STEADY = PlantJoint(natural_frequency_hz=3.0, damping_ratio=1.0)


@pytest.fixture(autouse=True)
def brief_dwells(monkeypatch):
    """Bench durations, shortened. The probe reads what it measures from the
    window, so the only thing shortening changes is how long the test takes."""
    for name, value in {
        "BASELINE_EXCITE_S": 0.3,
        "BASELINE_QUIET_S": 0.8,
        "DWELL_EXCITE_S": 0.3,
        "DWELL_QUIET_S": 0.8,
        "QUIET_SETTLE_S": 0.2,
        "SETTLE_S": 0.3,
        "RECOVERY_S": 0.05,
        "APPROACH_DEG_PER_S": 60.0,
        "ONSET_CHECK_AFTER_S": 0.5,
    }.items():
        monkeypatch.setattr(gain_margin, name, value)


class PlantPump(threading.Thread):
    """Feeds the hand's link with frames from a plant the hand is driving."""

    def __init__(self, hand, source, rate_hz: float = PUMP_HZ):
        super().__init__(daemon=True)
        self._hand = hand
        self._source = source
        self._period = 1.0 / rate_hz
        self._stop = threading.Event()

    def run(self) -> None:
        while not self._stop.is_set():
            self._source.advance()
            counts = _with_parity(self._source.current().raw_counts)
            self._hand._encoder_link.feed_bytes(make_encoder_frame(counts))
            time.sleep(self._period)

    def stop(self) -> None:
        self._stop.set()
        self.join(timeout=1.0)


def _with_parity(counts: np.ndarray) -> np.ndarray:
    """Set the chip's even-parity bit, which the decoder checks."""
    values = counts.astype(np.uint16) & 0x7FFF
    parity = np.zeros(values.shape, dtype=np.uint16)
    remaining = values.copy()
    while np.any(remaining):
        parity ^= (remaining & 1).astype(np.uint16)
        remaining >>= 1
    return (values | (parity << 15)).astype(np.uint16)


class Bench:
    """A connected hand, the plant behind it, and the window watching it."""

    def __init__(self, hand, window, frames, pump):
        self.hand = hand
        self.window = window
        self.frames = frames
        self.pump = pump

    def run(self, tmp_path, **kwargs):
        kwargs.setdefault("ritual", Ritual(notes="synthetic"))
        kwargs.setdefault("joints", [JOINT])
        return run_gain_margin(
            self.hand,
            self.window,
            output_dir=tmp_path,
            frame_recorder=self.frames,
            **kwargs,
        )


@pytest.fixture
def bench(tmp_path, request):
    template = getattr(request, "param", STEADY)
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)

    hand = MockOrcaHandJointFeedback(config_path=str(config_path))
    frames = FrameRecorder()
    window = JointAngleWindow.for_hand(hand)
    record_frames(hand, frames, window)
    connected, message = hand.connect()
    assert connected, message

    hand._encoder_pump.stop()
    source = PlantEncoderSource(
        hand, joints=hand.loop_joint_names, dt_s=1.0 / PUMP_HZ, template=template
    )
    source.align_hand_to_plant()
    pump = PlantPump(hand, source)
    pump.start()
    _wait_for_frames(window)
    hand._loop.rebase()

    yield Bench(hand, window, frames, pump)

    pump.stop()
    hand.disconnect()


def _wait_for_frames(window, minimum: int = 50, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while window.written < minimum:
        if time.monotonic() > deadline:
            raise AssertionError("the plant never fed the window")
        time.sleep(0.01)


def test_the_wrist_is_never_probed(bench, tmp_path):
    """It runs a position mode with no current ceiling, so nothing limits what
    an oscillation there could do to it."""
    with pytest.raises(PreconditionError, match="wrist"):
        bench.run(tmp_path, joints=["wrist"])


def test_a_joint_the_loop_does_not_control_is_refused(bench, tmp_path):
    with pytest.raises(PreconditionError, match="not controlling"):
        bench.run(tmp_path, joints=["nonexistent_joint"])


def test_a_run_that_moves_the_hand_needs_its_tendon_history(bench, tmp_path):
    with pytest.raises(RitualNotDeclared):
        bench.run(tmp_path, ritual=None)


@pytest.mark.parametrize("bench", [STEADY], indirect=True)
def test_a_joint_with_margin_to_spare_ramps_to_the_ceiling(bench, tmp_path, monkeypatch):
    monkeypatch.setattr(gain_margin, "MAX_KP", 0.8)

    result = bench.run(tmp_path)

    margin = result["joints"][0]
    assert margin["outcome"] == "no onset"
    assert margin["onset_value"] is None
    assert [dwell["kp"] for dwell in margin["dwells"]] == [0.5, 0.5, 0.625, 0.7812]


@pytest.mark.parametrize("bench", [RINGS], indirect=True)
def test_a_joint_that_starts_ringing_reports_where_and_how_fast(
    bench, tmp_path, monkeypatch
):
    monkeypatch.setattr(gain_margin, "MAX_KP", 2.0)

    result = bench.run(tmp_path)

    margin = result["joints"][0]
    assert margin["outcome"] == "onset"
    assert margin["onset_value"] >= margin["baseline_kp"]
    # The frequency is what the mechanism does, and this one is transport-delay
    # limited: it rings at roughly a quarter cycle per delay. What matters is
    # that the number is the mechanism's, not the loop rate or the sampling.
    assert 2.0 < margin["onset_frequency_hz"] < 30.0

    dwells = margin["dwells"]
    assert dwells[-1]["verdict"]["state"] == "oscillating"
    assert all(
        dwell["verdict"] is None or dwell["verdict"]["state"] != "oscillating"
        for dwell in dwells[:-1]
    ), "the ramp kept going after it had its answer"


def test_every_other_joint_is_silenced_while_one_is_probed(bench, tmp_path, monkeypatch):
    """A hand with seventeen loops closed on it measures seventeen loops."""
    monkeypatch.setattr(gain_margin, "MAX_KP", 0.5)
    seen = {}

    def watch(event):
        if event["event"] == "baseline_measured":
            seen.update(
                {joint: gains.kp for joint, gains in bench.hand.get_pid_gains().items()}
            )

    bench.run(tmp_path, progress_callback=watch)

    assert seen, "the baseline never ran"
    assert seen[JOINT] > 0
    assert set(value for joint, value in seen.items() if joint != JOINT) == {0.0}


def test_the_gains_are_handed_back_as_they_were_found(bench, tmp_path, monkeypatch):
    monkeypatch.setattr(gain_margin, "MAX_KP", 0.5)
    before = {
        joint: (gains.kp, gains.ki, gains.correction_max_deg)
        for joint, gains in bench.hand.get_pid_gains().items()
    }

    bench.run(tmp_path)

    after = {
        joint: (gains.kp, gains.ki, gains.correction_max_deg)
        for joint, gains in bench.hand.get_pid_gains().items()
    }
    assert after == before


def test_stopping_ends_the_dwell_and_closes_the_dataset(bench, tmp_path):
    result = bench.run(tmp_path, should_stop=lambda: True)

    margin = result["joints"][0]
    assert margin["outcome"] == "aborted"
    assert "stopped by request" in margin["reason"]
    assert Dataset(result["directory"]).status == "ok"


def test_the_dataset_carries_the_gains_and_the_verdicts(bench, tmp_path, monkeypatch):
    monkeypatch.setattr(gain_margin, "MAX_KP", 0.65)

    result = bench.run(tmp_path)

    dataset = Dataset(result["directory"])
    assert dataset.status == "ok"
    assert not dataset.truncated

    events = dataset.events()
    kinds = [event["event"] for event in events]
    assert kinds.count("gain_change") == len(result["joints"][0]["dwells"])
    assert "isolated" in kinds
    assert any(event.get("measured") for event in events if event["event"] == "dwell_measured")

    loop = dataset.table("loop")
    assert len(loop) > 0
    assert f"corr_{JOINT}" in loop
    frames = dataset.table("frames")
    assert len(frames) > len(loop), "the frame record must be the denser of the two"
