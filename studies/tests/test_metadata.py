"""Tests for the run manifest: what it must carry, and what it refuses."""

from __future__ import annotations

import json
import os
import shutil
import time

import pytest

from studies.record.metadata import (
    TRACKED_MODULES,
    Ritual,
    RitualNotDeclared,
    collect_metadata,
    measure_scheduling_noise,
)

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


def collect(hand, **kwargs):
    kwargs.setdefault("moves_hand", False)
    return collect_metadata(hand, "timing", {"duration_s": 120}, **kwargs)


def test_manifest_is_json_serialisable(hand):
    """Numpy leaks in from calibration; the manifest has to survive it."""
    meta = collect(hand)
    json.dumps(meta)


def test_manifest_records_the_experiment_and_its_parameters(hand):
    meta = collect(hand)

    assert meta["experiment"] == "timing"
    assert meta["params"] == {"duration_s": 120}
    assert "timing" in meta["run_id"]


def test_configuration_is_embedded_not_referenced(hand):
    """calibration.yaml is untracked and the next calibration overwrites it, so
    a path alone would be stale by the time anyone read the dataset."""
    meta = collect(hand)

    assert meta["hand"]["config_yaml"] is not None
    assert "joint_roms" in meta["hand"]["config_yaml"]
    assert meta["hand"]["config_sha256"]


def test_the_calibration_actually_in_force_is_recorded(hand):
    meta = collect(hand)["calibration"]

    assert meta["calibrated"] is True
    assert set(meta["joint_to_motor_ratios"])
    assert set(meta["encoder_anchors"])
    assert meta["wrap_offsets"] is not None


def test_the_gap_between_commandable_and_mapped_range_is_recorded(hand):
    """Commands are clamped to one range while the map is defined over another;
    where they differ, part of the map cannot be reached."""
    deltas = collect(hand)["calibration"]["rom_delta"]

    assert deltas
    assert all(len(value) == 2 for value in deltas.values())


def test_gains_fall_back_to_the_configured_values_without_a_loop(hand):
    """Most runs are open-loop; asking such a hand for installed gains raises."""
    control = collect(hand)["control"]

    assert control["gains_source"] == "configured"
    assert control["loop_joints"] is None
    assert "kp" in next(iter(control["gains"].values()))


def test_control_constants_are_snapshotted(hand):
    """This branch expects to change them, so a dataset records the values it
    was produced under rather than whatever they are when it is read."""
    constants = collect(hand)["control"]["constants"]

    assert constants["DEFAULT_KP"] == 0.5
    assert constants["WATCHDOG_STOP_LOOP_MS"] == 1000


def test_the_code_that_produced_the_run_is_identified(hand):
    code = collect(hand)["code"]

    assert code["package_path"].endswith("orca_core")
    assert set(code["module_hashes"]) == set(TRACKED_MODULES)
    assert all(code["module_hashes"].values())


def test_the_host_scheduling_floor_is_measured(hand):
    """Delivery jitter and scheduler jitter arrive mixed; this is the floor
    below which a timing number says nothing about hardware."""
    noise = collect(hand)["host"]["scheduling_noise"]

    assert noise["samples"] > 0
    assert noise["max_ms"] >= noise["median_ms"]


def test_scheduling_probe_returns_a_distribution():
    noise = measure_scheduling_noise(duration_s=0.05, sleep_s=0.001)

    assert noise["samples"] > 1
    assert noise["p99_ms"] >= noise["median_ms"]


def test_encoder_frame_definition_is_recorded(hand):
    encoder = collect(hand)["encoder"]

    assert encoder["lsb_deg"] == pytest.approx(360.0 / 16384)
    assert encoder["joint_to_slot"]["wrist"] == 16
    assert encoder["polarity"] is not None


# ----- The ritual gate -----------------------------------------------------


def test_a_run_that_moves_the_hand_refuses_without_a_declared_history(hand):
    with pytest.raises(RitualNotDeclared, match="tendon history"):
        collect_metadata(hand, "step_response", moves_hand=True)


def test_a_run_that_only_listens_needs_no_history(hand):
    meta = collect_metadata(hand, "timing", moves_hand=False)

    assert meta["ritual"] == {"declared": False}


def test_a_declared_history_records_how_long_ago(hand):
    ritual = Ritual(
        tensioned_at=time.time() - 1800,
        calibrated_at=time.time() - 600,
        notes="retensioned index",
    )
    recorded = collect_metadata(hand, "step_response", moves_hand=True, ritual=ritual)[
        "ritual"
    ]

    assert recorded["declared"] is True
    assert recorded["minutes_since_tension"] == pytest.approx(30.0, abs=0.5)
    assert recorded["minutes_since_calibration"] == pytest.approx(10.0, abs=0.5)
    assert recorded["minutes_since_jitter"] is None
    assert recorded["notes"] == "retensioned index"


def test_an_empty_ritual_still_satisfies_the_gate(hand):
    """Declaring that nothing was done is a real answer; it is filterable
    afterwards, which a refusal to record is not."""
    recorded = collect_metadata(
        hand, "step_response", moves_hand=True, ritual=Ritual()
    )["ritual"]

    assert recorded["declared"] is True
    assert recorded["minutes_since_tension"] is None
