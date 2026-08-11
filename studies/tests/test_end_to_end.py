"""The capture path against the real control loop.

Everything else tests a piece in isolation. This drives an actual
``JointLoopThread`` and ``JointController`` against a synthetic joint, records
the run, and reads the dataset back — so the schema, the recorders, the session
and the reader are exercised the way a bench run exercises them.
"""

from __future__ import annotations

import os
import shutil

import numpy as np
import pytest

from orca_core.control import JointController, JointLoopThread
from orca_core.hardware.sensing.constants import ENCODER_LSB_DEG

from studies.plant import PlantEncoderSource, PlantJoint
from studies.preconditions import require_healthy_encoders
from studies.record import Dataset, LoopRecorder
from studies.record.schema import PHASE_TO_CODE
from studies.record.session import STATUS_OK, RecordingSession

from tests._loop_helpers import make_calibrated_hand


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


@pytest.fixture
def hand(tmp_path):
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)
    hand = make_calibrated_hand(str(config_path))
    yield hand
    hand.disconnect()


def build(hand, *, Kp=0.5, Ki=5.0, template=None):
    source = PlantEncoderSource(hand, dt_s=0.01, template=template)
    source.align_hand_to_plant()
    controller = JointController(num_joints=len(hand._encoder_backed_joints()))
    controller.set_gains(Kp=Kp, Ki=Ki, correction_max_deg=15.0)
    loop = JointLoopThread(hand, source, controller, target_hz=100)
    loop.prime_for_step()
    recorder = LoopRecorder(loop.joint_names, capacity=8192)
    loop.attach_observer(recorder)
    return loop, source, recorder


def run(loop, cycles=200):
    for _ in range(cycles):
        loop.step_once(dt=0.01)


def test_a_recorded_run_reads_back_as_a_complete_dataset(hand, tmp_path):
    loop, _, recorder = build(hand)

    with RecordingSession(
        tmp_path / "run", experiment="loop_capture", drain_interval_s=0.005
    ) as session:
        session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
        run(loop, cycles=50)

    dataset = Dataset(tmp_path / "run")
    assert dataset.status == STATUS_OK
    assert not dataset.truncated
    assert dataset.meta["tables"]["loop"]["dropped"] == 0

    table = dataset.table("loop")
    assert len(table) == 50
    assert list(table["cycle"]) == list(range(1, 51))
    assert set(table["phase"]) == {PHASE_TO_CODE["ok"]}


def test_the_loop_drives_the_plant_to_target(hand, tmp_path):
    """A closed loop against a plant it can reach must converge on the setpoint."""
    loop, _, recorder = build(hand, template=PlantJoint(damping_ratio=1.0))
    joint = loop.joint_names[0]

    loop.set_target({joint: 5.0})
    run(loop, cycles=600)

    from studies.record.reader import Table

    table = Table(recorder.columns, recorder.ring.drain())
    assert table[f"meas_{joint}"][-1] == pytest.approx(5.0, abs=0.2)


def test_a_mismatched_ratio_shows_up_as_a_standing_correction(hand, tmp_path):
    """The map's gain error is what the correction has to make up, and it is the
    quantity the loop's own record can be read for."""
    loop, _, recorder = build(
        hand, template=PlantJoint(damping_ratio=1.0, gain=0.8)
    )
    joint = loop.joint_names[0]

    loop.set_target({joint: 10.0})
    run(loop, cycles=800)

    from studies.record.reader import Table

    table = Table(recorder.columns, recorder.ring.drain())
    assert table[f"meas_{joint}"][-1] == pytest.approx(10.0, abs=0.3)
    # Reaching 10 deg through a mechanism that delivers 0.8 of what it is told
    # needs the command carried about a quarter further.
    assert table[f"corr_{joint}"][-1] == pytest.approx(2.5, abs=0.5)


def test_an_underdamped_joint_is_visible_as_ringing_in_the_record(hand):
    loop, _, recorder = build(
        hand,
        Kp=0.5,
        Ki=5.0,
        template=PlantJoint(natural_frequency_hz=6.0, damping_ratio=0.05),
    )
    joint = loop.joint_names[0]

    loop.set_target({joint: 5.0})
    run(loop, cycles=400)

    from studies.record.reader import Table

    table = Table(recorder.columns, recorder.ring.drain())
    measured = table[f"meas_{joint}"]
    assert measured.max() > 5.5, "an underdamped joint should overshoot"


def test_raw_counts_decode_back_to_the_recorded_angles(hand):
    """Raw counts are kept so a run can be re-read under a different
    calibration; they must agree with the angles the loop decoded."""
    from orca_core.hardware.sensing.constants import joint_encoder_polarity_for_side
    from orca_core.hardware.sensing.encoder_protocol import encoder_to_joint_angle
    from studies.record.reader import Table

    loop, _, recorder = build(hand)
    loop.set_target({loop.joint_names[0]: 3.0})
    run(loop, cycles=100)

    table = Table(recorder.columns, recorder.ring.drain())
    polarity = joint_encoder_polarity_for_side(hand.config.type)
    encoder_cal = hand.calibration.joint_encoder_calibration_dict

    decoded = encoder_to_joint_angle(
        table.block("raw")[-1].astype(np.int64),
        np.array([encoder_cal[j].enc_at_anchor_count for j in loop.joint_names]),
        np.array([polarity[j] for j in loop.joint_names]),
        np.array([hand.config.joint_roms_dict[j][1] for j in loop.joint_names]),
    )
    assert np.allclose(decoded, table.block("meas")[-1], atol=ENCODER_LSB_DEG)


def test_a_degraded_cycle_is_recorded_rather_than_dropped(hand, tmp_path):
    """A record that only covers healthy cycles would misreport the cadence
    exactly when the cadence is the thing in question."""
    loop, source, recorder = build(hand)

    with RecordingSession(
        tmp_path / "run", experiment="loop_capture", drain_interval_s=0.005
    ) as session:
        session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
        run(loop, cycles=10)
        loop.pause_writes()
        run(loop, cycles=5)
        loop.resume_writes()
        run(loop, cycles=10)

    table = Dataset(tmp_path / "run").table("loop")
    phases = list(table["phase"])
    assert phases.count(PHASE_TO_CODE["paused"]) == 5
    assert len(table) == 25
    assert list(table["cycle"]) == list(range(1, 26))


def test_the_encoder_health_check_passes_on_a_working_stream(hand):
    _, source, _ = build(hand)

    report = require_healthy_encoders(source, num_frames=50)
    assert report.frames_sampled == 50


def test_the_recorded_joint_order_survives_the_round_trip(hand, tmp_path):
    loop, _, recorder = build(hand)

    with RecordingSession(
        tmp_path / "run", experiment="loop_capture", drain_interval_s=0.005
    ) as session:
        session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
        run(loop, cycles=5)

    table = Dataset(tmp_path / "run").table("loop")
    assert table.joints("meas") == loop.joint_names
    assert table.joints("corr") == loop.joint_names
