"""Unit tests for orca_core.hardware.sensing.health.

Synthetic-data tests of the sensor health-assessment logic: per-encoder-slot
verdicts, tactile wiring-mismatch inference, and the encoder-link
likely-cause diagnosis.
"""
from __future__ import annotations

import time
from collections import Counter

import numpy as np

from orca_core.hardware.hand_serial_link import LinkStats
from orca_core.hardware.joint_encoder_client import EncoderStreamStats
from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_NUM_JOINTS,
)
from orca_core.hardware.sensing.health import (
    LINK_CAUSE_NO_BYTES,
    LINK_CAUSE_NO_FRAME_ALIGNMENT,
    LINK_CAUSE_UNROUTED_FRAMES,
    LINK_CAUSE_WIRE_FORMAT_MISMATCH,
    EncoderStreamHealth,
    detect_wiring_mismatch,
    diagnose_encoder_link,
)
from orca_core.hardware.sensing.types import EncoderReading

MID_ANGLE = 0x2000


def make_reading(
    raw_counts: np.ndarray | None = None,
    parity_ok: np.ndarray | None = None,
    angle_error: np.ndarray | None = None,
) -> EncoderReading:
    if raw_counts is None:
        raw_counts = np.full(AUTO_ENC_NUM_JOINTS, MID_ANGLE, dtype=np.uint16)
    if parity_ok is None:
        parity_ok = np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool)
    if angle_error is None:
        angle_error = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool)
    return EncoderReading(
        raw_counts=raw_counts,
        parity_ok=parity_ok,
        angle_error=angle_error,
        error_byte=0,
        timestamp=time.monotonic(),
    )


def counts(value: int) -> np.ndarray:
    return np.full(AUTO_ENC_NUM_JOINTS, value, dtype=np.uint16)


# ---------------------------------------------------------------------------
# EncoderStreamHealth verdicts
# ---------------------------------------------------------------------------

def test_healthy_slot_reads_ok():
    health = EncoderStreamHealth()
    for _ in range(10):
        health.update(make_reading())

    report = health.report(0)
    assert report.healthy
    assert report.reason == "ok"
    assert report.frames == 10
    assert report.parity_errors == 0
    assert report.angle_error_flags == 0
    assert report.rail_fraction == 0.0


def test_span_tracks_min_and_max_angle():
    health = EncoderStreamHealth()
    health.update(make_reading(counts(0x1000)))
    health.update(make_reading(counts(0x1500)))
    health.update(make_reading(counts(0x1200)))

    assert health.report(0).span == 0x500


def test_parity_errors_fail_slot():
    health = EncoderStreamHealth()
    bad_parity = np.ones(AUTO_ENC_NUM_JOINTS, dtype=bool)
    bad_parity[3] = False
    health.update(make_reading(parity_ok=bad_parity))
    health.update(make_reading(parity_ok=bad_parity))
    health.update(make_reading())

    report = health.report(3)
    assert not report.healthy
    assert report.reason == "2 parity errors"
    assert health.report(0).healthy


def test_angle_error_flags_fail_slot():
    health = EncoderStreamHealth()
    err = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool)
    err[7] = True
    health.update(make_reading(angle_error=err))
    health.update(make_reading())

    report = health.report(7)
    assert not report.healthy
    assert report.reason == "1 chip angle-error flags"


def test_slot_stuck_at_rail_fails():
    health = EncoderStreamHealth()
    for _ in range(4):
        health.update(make_reading(counts(0)))

    report = health.report(0)
    assert not report.healthy
    assert report.reason == "stuck/floating bus (100% at a rail)"
    assert report.rail_fraction == 1.0


def test_high_rail_counts_as_extreme():
    health = EncoderStreamHealth()
    for _ in range(4):
        health.update(make_reading(counts(AUTO_ENC_ANGLE_MASK)))

    assert not health.report(0).healthy


def test_rail_fraction_at_threshold_passes():
    health = EncoderStreamHealth()
    health.update(make_reading(counts(0)))
    health.update(make_reading(counts(0)))
    health.update(make_reading())
    health.update(make_reading())

    report = health.report(0)
    assert report.rail_fraction == 0.5
    assert report.healthy


def test_rail_edge_band_boundaries():
    health = EncoderStreamHealth(rail_edge_lsb=64)
    health.update(make_reading(counts(64)))       # inside the band
    health.update(make_reading(counts(65)))       # just outside
    health.update(make_reading(counts(65)))
    health.update(make_reading(counts(65)))

    assert health.report(0).rail_fraction == 0.25


def test_parity_takes_precedence_over_rail():
    health = EncoderStreamHealth()
    bad_parity = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=bool)
    for _ in range(4):
        health.update(make_reading(counts(0), parity_ok=bad_parity))

    assert health.report(0).reason == "4 parity errors"


def test_parity_and_error_bits_masked_out_of_angle():
    health = EncoderStreamHealth()
    for _ in range(4):
        health.update(make_reading(counts(0x8000 | MID_ANGLE)))

    report = health.report(0)
    assert report.healthy
    assert report.rail_fraction == 0.0


def test_no_frames_reports_unhealthy():
    report = EncoderStreamHealth().report(0)
    assert not report.healthy
    assert report.reason == "no frames"
    assert report.frames == 0


# ---------------------------------------------------------------------------
# Wiring-mismatch inference
# ---------------------------------------------------------------------------

WIRING = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}


def test_no_mismatch_when_pressed_finger_responds():
    peaks = {"thumb": 3.0, "index": 0.1, "middle": 0.0, "ring": 0.0, "pinky": 0.0}
    assert detect_wiring_mismatch("thumb", peaks, WIRING, threshold_n=1.0) is None


def test_no_mismatch_when_nothing_responds():
    peaks = {f: 0.2 for f in WIRING}
    assert detect_wiring_mismatch("thumb", peaks, WIRING, threshold_n=1.0) is None


def test_mismatch_reports_strongest_other_finger():
    peaks = {"thumb": 0.3, "index": 1.5, "middle": 4.0, "ring": 0.0, "pinky": 0.0}
    mismatch = detect_wiring_mismatch("thumb", peaks, WIRING, threshold_n=1.0)

    assert mismatch is not None
    assert mismatch.pressed_finger == "thumb"
    assert mismatch.sensed_finger == "middle"
    assert mismatch.pressed_peak_n == 0.3
    assert mismatch.sensed_peak_n == 4.0
    assert mismatch.pressed_slot == 0
    assert mismatch.sensed_slot == 2


def test_missing_pressed_finger_counts_as_zero():
    peaks = {"index": 2.0}
    mismatch = detect_wiring_mismatch("thumb", peaks, WIRING, threshold_n=1.0)

    assert mismatch is not None
    assert mismatch.pressed_peak_n == 0.0
    assert mismatch.sensed_finger == "index"


def test_unknown_fingers_have_no_slots():
    mismatch = detect_wiring_mismatch("thumb", {"index": 2.0}, {}, threshold_n=1.0)

    assert mismatch is not None
    assert mismatch.pressed_slot is None
    assert mismatch.sensed_slot is None


# ---------------------------------------------------------------------------
# Encoder-link diagnosis ladder
# ---------------------------------------------------------------------------

def make_link_stats(**overrides) -> LinkStats:
    stats = LinkStats()
    for name, value in overrides.items():
        setattr(stats, name, value)
    return stats


def cause_codes(link_stats: LinkStats, stream_stats: EncoderStreamStats) -> list[str]:
    return [c.code for c in diagnose_encoder_link(link_stats, stream_stats)]


def test_healthy_stream_has_no_causes():
    link_stats = make_link_stats(frames_routed=Counter({0xA9: 500}), responses_received=3)
    assert cause_codes(link_stats, EncoderStreamStats(frames_ok=500)) == []


def test_bad_effective_length_means_wire_format_mismatch():
    link_stats = make_link_stats(frames_routed=Counter({0xA9: 7}))
    stream_stats = EncoderStreamStats(frames_bad_effective_length=7)
    assert cause_codes(link_stats, stream_stats) == [LINK_CAUSE_WIRE_FORMAT_MISMATCH]


def test_unrouted_frames_lists_offending_bytes():
    link_stats = make_link_stats(frames_dropped_no_handler=Counter({0xA8: 12}))
    causes = diagnose_encoder_link(link_stats, EncoderStreamStats())

    assert [c.code for c in causes] == [LINK_CAUSE_UNROUTED_FRAMES]
    assert "0xA8" in causes[0].description


def test_resyncs_without_frames_means_no_alignment():
    link_stats = make_link_stats(bad_header_resyncs=42)
    assert cause_codes(link_stats, EncoderStreamStats()) == [LINK_CAUSE_NO_FRAME_ALIGNMENT]


def test_resyncs_with_routed_frames_is_not_misalignment():
    link_stats = make_link_stats(
        bad_header_resyncs=2, frames_routed=Counter({0xA9: 100})
    )
    assert cause_codes(link_stats, EncoderStreamStats(frames_ok=100)) == []


def test_total_silence_means_no_bytes():
    assert cause_codes(LinkStats(), EncoderStreamStats()) == [LINK_CAUSE_NO_BYTES]


def test_descriptions_join_to_single_line():
    causes = diagnose_encoder_link(LinkStats(), EncoderStreamStats())
    flat = causes[0].description.replace("\n", " ")
    assert "\n" not in flat
    assert flat.startswith("Zero bytes received.")
