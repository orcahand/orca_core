"""Tests for the stream statistics.

The load-bearing claim is that a lost frame and a late host can be told apart
from arrival times alone. Both are constructed here from known inputs, so the
separators are checked against cases whose answer is known rather than against
a recording whose answer is what is being asked.
"""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_NUM_JOINTS,
    JOINT_TO_ENCODER_SLOT,
)

from studies.analysis import timing
from studies.record.reader import Table
from studies.record.schema import frame_columns


PERIOD_MS = timing.BOARD_EMIT_PERIOD_MS


def frames_table(gaps_ms, *, counts=None, decode_ms=0.05):
    """Build a frame table whose arrivals are separated by ``gaps_ms``."""
    n = len(gaps_ms) + 1
    times = np.concatenate([[0.0], np.cumsum(np.asarray(gaps_ms) / 1000.0)])
    columns = frame_columns()
    data = np.zeros((n, len(columns)))
    data[:, columns.index("seq")] = np.arange(1, n + 1)
    data[:, columns.index("t")] = times
    data[:, columns.index("frame_t")] = times + decode_ms / 1000.0
    if counts is None:
        counts = np.tile(np.arange(AUTO_ENC_NUM_JOINTS), (n, 1)) + np.arange(n)[:, None]
    for slot in range(AUTO_ENC_NUM_JOINTS):
        data[:, columns.index(f"raw_s{slot:02d}")] = counts[:, slot]
    return Table(columns, data)


def steady(n, period_ms=PERIOD_MS):
    return [period_ms] * n


# ----- Rate and distribution ----------------------------------------------


def test_rate_matches_the_spacing():
    table = frames_table(steady(499))

    assert timing.observed_rate_hz(table) == pytest.approx(500.0, rel=1e-6)


def test_distribution_reports_the_tail_not_just_the_mean():
    gaps = np.array(steady(999) + [40.0])
    stats = timing.distribution(gaps)

    assert stats["p50_ms"] == pytest.approx(PERIOD_MS)
    assert stats["max_ms"] == 40.0
    assert stats["mean_ms"] < 2.1


def test_distribution_of_nothing_is_empty_not_an_error():
    assert timing.distribution(np.array([])) == {}


# ----- Separating a lost frame from a late host ---------------------------


def test_a_lost_frame_lands_on_a_multiple_of_the_emit_period():
    gaps = steady(50) + [2 * PERIOD_MS] + steady(50)
    multiples = timing.period_multiples(np.array(gaps))

    assert multiples[1] == 100
    assert multiples[2] == 1


def test_two_lost_frames_land_on_three_periods():
    multiples = timing.period_multiples(np.array(steady(10) + [3 * PERIOD_MS]))

    assert multiples[3] == 1


def test_a_stream_delivered_on_its_own_cadence_is_almost_all_on_multiples():
    fraction = timing.off_period_fraction(np.array(steady(200) + [2 * PERIOD_MS]))

    assert fraction == pytest.approx(0.0, abs=1e-9)


def test_a_re_timed_stream_shows_up_as_gaps_off_the_multiples():
    """Delivery that batches and releases frames destroys the board's spacing
    while preserving the mean, so the mean cannot detect it."""
    rng = np.random.default_rng(0)
    gaps = rng.uniform(0.1, 3.9, size=500)
    fraction = timing.off_period_fraction(gaps)

    assert fraction > 0.4
    assert gaps.mean() == pytest.approx(PERIOD_MS, abs=0.2)


def test_a_late_host_is_followed_by_a_short_gap():
    """Frames buffered during the stall drain in microseconds afterwards."""
    gaps = steady(20) + [12.0, 0.02, 0.02] + steady(20)
    verdict = timing.lag1_conditional_mean(np.array(gaps))

    assert verdict["n_late"] == 1
    assert verdict["reads_as"] == "host late"
    assert verdict["following_mean_ms"] < 0.5


def test_a_genuinely_lost_frame_is_followed_by_an_ordinary_gap():
    gaps = steady(20) + [8.0] + steady(20)
    verdict = timing.lag1_conditional_mean(np.array(gaps))

    assert verdict["n_late"] == 1
    assert verdict["reads_as"] == "frames lost"
    assert verdict["following_mean_ms"] == pytest.approx(PERIOD_MS)


def test_a_clean_stream_reports_no_long_gaps():
    assert timing.lag1_conditional_mean(np.array(steady(100)))["n_late"] == 0


def test_re_timing_by_a_single_period_is_still_examined():
    """The common case is one frame held back and released with the next. A
    threshold set for multi-period loss steps straight over it and reports a
    clean stream."""
    gaps = steady(20) + [2 * PERIOD_MS, 0.05] + steady(20)
    verdict = timing.lag1_conditional_mean(np.array(gaps))

    assert verdict["n_late"] == 1
    assert verdict["reads_as"] == "host late"


def test_the_two_mechanisms_are_reported_as_a_fraction_not_only_a_label():
    """Both are usually present at once, so the label alone would over-commit."""
    gaps = steady(10) + [12.0, 0.02] + steady(10) + [12.0] + steady(10)
    verdict = timing.lag1_conditional_mean(np.array(gaps))

    assert verdict["n_late"] == 2
    assert verdict["fraction_followed_by_short_gap"] == pytest.approx(0.5)


# ----- Watchdog exposure ---------------------------------------------------


def test_watchdog_exposure_counts_gaps_a_running_loop_would_have_noticed():
    gaps = np.array(steady(100) + [20.0, 60.0, 300.0])
    exposure = timing.watchdog_exposure(
        gaps, {"warn": 15.0, "hold": 50.0, "hold_base": 200.0, "stop": 1000.0}
    )

    assert exposure == {"warn": 3, "hold": 2, "hold_base": 1, "stop": 0}


def test_a_clean_stream_crosses_no_tiers():
    summary = timing.summarise(frames_table(steady(500)))

    assert all(count == 0 for count in summary["watchdog_exposure"].values())


# ----- Per-slot integrity --------------------------------------------------


def test_slot_report_recomputes_the_chip_error_bit_from_the_stored_counts():
    """Parity and the error bit live in the counts, so they are derived rather
    than carried as columns that could disagree with them."""
    slot = JOINT_TO_ENCODER_SLOT["middle_pip"]
    n = 20
    counts = np.tile(np.arange(AUTO_ENC_NUM_JOINTS), (n, 1)) + np.arange(n)[:, None]
    counts[:, slot] |= AUTO_ENC_ANGLE_ERROR_BIT

    report = timing.slot_report(frames_table(steady(n - 1), counts=counts))
    assert report[f"s{slot:02d}"]["angle_error"] == n
    assert report[f"s{slot:02d}"]["joint"] == "middle_pip"


def test_slot_report_flags_a_jump_no_joint_could_make():
    slot = 3
    n = 20
    counts = np.tile(np.arange(AUTO_ENC_NUM_JOINTS), (n, 1)) + np.arange(n)[:, None]
    counts[10:, slot] += 4000

    report = timing.slot_report(frames_table(steady(n - 1), counts=counts))
    assert report[f"s{slot:02d}"]["impossible_jumps"] == 1


def test_slot_report_notices_a_slot_that_never_moves():
    n = 20
    counts = np.tile(np.arange(AUTO_ENC_NUM_JOINTS), (n, 1))

    report = timing.slot_report(frames_table(steady(n - 1), counts=counts))
    assert all(slot["constant"] for slot in report.values())


def test_a_count_wrapping_past_zero_is_not_a_jump():
    n = 20
    counts = np.tile(
        ((16380 + np.arange(n) * 3) % 16384)[:, None], (1, AUTO_ENC_NUM_JOINTS)
    )

    report = timing.slot_report(frames_table(steady(n - 1), counts=counts))
    assert all(slot["impossible_jumps"] == 0 for slot in report.values())


# ----- Bookkeeping ---------------------------------------------------------


def test_decode_cost_is_the_gap_between_arrival_and_decoded_frame():
    stats = timing.decode_cost_ms(frames_table(steady(50), decode_ms=0.08))

    assert stats["p50_ms"] == pytest.approx(0.08, abs=1e-6)


def test_recorder_losses_show_up_as_breaks_in_its_own_numbering():
    table = frames_table(steady(10))
    seq = table.columns.index("seq")
    table.data[5:, seq] += 7

    assert timing.sequence_gaps(table) == {"recorded": 11, "missing": 7, "breaks": 1}


def test_summarise_covers_the_whole_record():
    summary = timing.summarise(frames_table(steady(200)))

    assert summary["frames"] == 201
    assert summary["rate_hz"] == pytest.approx(500.0, rel=1e-6)
    assert summary["recorder"]["missing"] == 0
    assert set(summary["slots"]) == {
        f"s{slot:02d}" for slot in range(AUTO_ENC_NUM_JOINTS)
    }
