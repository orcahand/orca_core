"""Unit tests for the wrap-aware encoder count low-pass."""
from __future__ import annotations

import math

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import (
    AUTO_ENC_ANGLE_ERROR_BIT,
    AUTO_ENC_ANGLE_MASK,
    AUTO_ENC_PARITY_BIT,
    ENCODER_COUNTS_PER_REV,
)
from orca_core.hardware.sensing.encoder_filter import EncoderCountFilter


DT = 0.002  # the hardware stream rate, ~466 Hz
SETTLED = int(0.2 / DT)  # ~12 time constants at the 10 Hz used below
FLAG_BITS = AUTO_ENC_PARITY_BIT | AUTO_ENC_ANGLE_ERROR_BIT


def drive(filt, counts, samples, dt=DT, valid=None, start=0.0):
    """Feed one constant sample ``samples`` times; return the last output."""
    out = None
    t = start
    for _ in range(samples):
        t += dt
        out = filt.update(np.asarray(counts, dtype=np.uint16), t, valid=valid)
    return out


# ---------------------------------------------------------------------------
# Basic response
# ---------------------------------------------------------------------------

def test_first_sample_passes_through_unchanged():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    raw = np.array([0, 1234, 16383], dtype=np.uint16)

    np.testing.assert_array_equal(filt.update(raw, 0.0), raw)


def test_step_converges_to_the_new_value():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([1000], dtype=np.uint16), 0.0)

    out = drive(filt, [2000], samples=SETTLED)

    assert out[0] == pytest.approx(2000, abs=2)


def test_output_lags_rather_than_jumping():
    """One time constant of a step lands at ~63%, i.e. the cutoff is real."""
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([0], dtype=np.uint16), 0.0)
    tau = 1.0 / (2.0 * math.pi * 10.0)

    out = drive(filt, [1000], samples=round(tau / DT))

    assert out[0] == pytest.approx(1000 * (1 - math.exp(-1)), rel=0.05)


def test_noise_is_attenuated_to_below_one_lsb():
    """White noise of a few LSB reduces to sub-LSB, the encoder's own floor."""
    rng = np.random.default_rng(0)
    filt = EncoderCountFilter(cutoff_hz=10.0)
    samples = rng.normal(8000, 3.0, size=4000).round().astype(np.uint16)

    out = np.array(
        [filt.update(samples[i:i + 1], i * DT)[0] for i in range(len(samples))]
    )

    assert samples.std() > 2.5
    assert out[len(out) // 2:].std() < 1.0


# ---------------------------------------------------------------------------
# Wrap handling
# ---------------------------------------------------------------------------

def test_tracks_across_the_zero_seam_without_sweeping_the_long_way():
    """16380 -> 4 is a 8-count move, not a 16376-count one."""
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([16380], dtype=np.uint16), 0.0)

    out = drive(filt, [4], samples=SETTLED)

    assert out[0] == pytest.approx(4, abs=2)


def test_intermediate_state_stays_on_the_short_arc():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([16300], dtype=np.uint16), 0.0)

    out = drive(filt, [84], samples=4)  # partway through a +168 count move

    # Still short of the target, having crossed (or about to cross) zero.
    assert out[0] > 16300 or out[0] < 84


def test_output_never_leaves_the_14_bit_range():
    rng = np.random.default_rng(1)
    filt = EncoderCountFilter(cutoff_hz=10.0)

    for i in range(500):
        counts = rng.integers(0, ENCODER_COUNTS_PER_REV, size=17).astype(np.uint16)
        out = filt.update(counts, i * DT)
        assert np.all(out >= 0) and np.all(out < ENCODER_COUNTS_PER_REV)


# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------

def test_cutoff_holds_when_the_frame_rate_changes():
    """Same elapsed time, different frame rates -> same amount of settling."""
    outputs = []
    for dt in (0.001, 0.002, 0.005):
        filt = EncoderCountFilter(cutoff_hz=10.0)
        filt.update(np.array([0], dtype=np.uint16), 0.0)
        outputs.append(drive(filt, [4000], samples=round(0.05 / dt), dt=dt)[0])

    assert max(outputs) - min(outputs) < 20  # ~0.5% of the step


def test_long_gap_snaps_to_the_new_sample():
    """A resumed stream must not sweep in from a stale pose."""
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([0], dtype=np.uint16), 0.0)

    out = filt.update(np.array([9000], dtype=np.uint16), 5.0)

    assert out[0] == 9000


def test_backwards_timestamp_holds_state():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([500], dtype=np.uint16), 10.0)

    out = filt.update(np.array([9000], dtype=np.uint16), 9.0)

    assert out[0] == 500


def test_reset_makes_the_next_sample_the_new_starting_point():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([500], dtype=np.uint16), 0.0)

    filt.reset()
    out = filt.update(np.array([9000], dtype=np.uint16), DT)

    assert out[0] == 9000


def test_slot_count_change_reseeds_instead_of_broadcasting():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([500, 600], dtype=np.uint16), 0.0)

    out = filt.update(np.array([1000, 2000, 3000], dtype=np.uint16), DT)

    np.testing.assert_array_equal(out, [1000, 2000, 3000])


# ---------------------------------------------------------------------------
# Flags and validity
# ---------------------------------------------------------------------------

def test_flag_bits_are_carried_through_from_the_source_frame():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    raw = np.array(
        [1000 | AUTO_ENC_PARITY_BIT, 2000 | AUTO_ENC_ANGLE_ERROR_BIT, 3000],
        dtype=np.uint16,
    )
    filt.update(raw, 0.0)

    out = drive(filt, raw, samples=SETTLED)

    np.testing.assert_array_equal(out & FLAG_BITS, raw & FLAG_BITS)
    np.testing.assert_array_equal(out & AUTO_ENC_ANGLE_MASK, [1000, 2000, 3000])


def test_invalid_slots_hold_while_their_neighbours_track():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([1000, 1000], dtype=np.uint16), 0.0)

    out = drive(
        filt, [5000, 5000], samples=SETTLED, valid=np.array([False, True])
    )

    assert out[0] == 1000
    assert out[1] == pytest.approx(5000, abs=2)


def test_a_slot_resumes_tracking_once_it_is_valid_again():
    filt = EncoderCountFilter(cutoff_hz=10.0)
    filt.update(np.array([1000], dtype=np.uint16), 0.0)
    invalid_until = drive(filt, [5000], samples=20, valid=np.array([False]))

    out = drive(filt, [5000], samples=SETTLED, start=20 * DT)

    assert invalid_until[0] == 1000
    assert out[0] == pytest.approx(5000, abs=2)


# ---------------------------------------------------------------------------
# Cutoff configuration
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("bad", [0.0, -1.0, float("inf"), float("nan")])
def test_non_positive_or_non_finite_cutoff_is_rejected(bad):
    with pytest.raises(ValueError):
        EncoderCountFilter(cutoff_hz=bad)


def test_cutoff_is_assignable_and_takes_effect():
    slow, fast = EncoderCountFilter(cutoff_hz=1.0), EncoderCountFilter(cutoff_hz=1.0)
    fast.cutoff_hz = 50.0
    for filt in (slow, fast):
        filt.update(np.array([0], dtype=np.uint16), 0.0)

    slow_out = drive(slow, [4000], samples=10)
    fast_out = drive(fast, [4000], samples=10)

    assert slow.cutoff_hz == 1.0 and fast.cutoff_hz == 50.0
    assert fast_out[0] > slow_out[0]
