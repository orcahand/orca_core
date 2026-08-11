"""The oscillation detector, against signals whose answer is known.

Every case here is one the bench can produce, and most of them are ways a
detector can be confidently wrong: quantisation noise crossing zero every
sample, drift hiding real crossings, a clamped loop whose swings are flat-topped,
and a fast oscillation sampled too slowly to be anything but a lie.
"""

from __future__ import annotations

import numpy as np
import pytest

from orca_core.hardware.sensing.constants import ENCODER_LSB_DEG

from studies.analysis import oscillation as osc


FRAME_RATE_HZ = 500.0
LOOP_RATE_HZ = 100.0


def sampled(frequency_hz, amplitude_deg, *, rate_hz=FRAME_RATE_HZ, duration_s=4.0, phase=0.0):
    t = np.arange(0.0, duration_s, 1.0 / rate_hz)
    return t, amplitude_deg * np.sin(2 * np.pi * frequency_hz * t + phase)


def quantise(x):
    return np.round(x / ENCODER_LSB_DEG) * ENCODER_LSB_DEG


@pytest.mark.parametrize("frequency", [0.8, 3.0, 12.0, 30.0, 50.0])
def test_it_recovers_the_frequency_it_was_given(frequency):
    t, x = sampled(frequency, 0.5)

    result = osc.describe(t, x)

    assert result.resolvable
    assert result.frequency_hz == pytest.approx(frequency, rel=0.05)
    # Detrending a window holding a fractional number of cycles tilts it, which
    # shows up as extra swing — over a tenth of it at the slow end, where four
    # seconds is only three cycles. Amplitudes are compared between windows of
    # the same length for that reason.
    assert result.peak_to_peak_deg == pytest.approx(1.0, rel=0.15)


def test_quantisation_noise_is_not_an_oscillation():
    """A signal sitting on a count boundary changes sign every sample. Counting
    those crossings would report a joint at rest as ringing at half the frame
    rate."""
    rng = np.random.default_rng(0)
    t = np.arange(0.0, 4.0, 1.0 / FRAME_RATE_HZ)
    x = quantise(rng.normal(0.0, 0.4 * ENCODER_LSB_DEG, t.size))

    result = osc.describe(t, x)

    assert not result.resolvable
    assert np.isnan(result.frequency_hz)


def test_drift_alone_produces_no_crossings():
    t = np.arange(0.0, 4.0, 1.0 / FRAME_RATE_HZ)

    result = osc.describe(t, 2.0 * t)

    assert result.half_cycles == 0
    assert result.peak_to_peak_deg == pytest.approx(0.0, abs=1e-9)


def test_an_oscillation_riding_on_drift_is_still_found():
    """The joint creeps while it rings. Without detrending the signal never
    returns to zero and the ringing is invisible."""
    t, x = sampled(8.0, 0.3)

    result = osc.describe(t, x + 1.5 * t)

    assert result.resolvable
    assert result.frequency_hz == pytest.approx(8.0, rel=0.05)


def test_a_clipped_limit_cycle_keeps_its_frequency():
    """A loop past its margin with a clamped correction does not diverge; it
    settles into a bounded, flat-topped cycle. The amplitude of that is the
    clamp's, but the frequency is still the loop's."""
    t, x = sampled(6.0, 3.0)

    result = osc.describe(t, np.clip(x, -1.0, 1.0))

    assert result.resolvable
    assert result.frequency_hz == pytest.approx(6.0, rel=0.05)


def test_a_disturbance_or_two_is_not_a_frequency():
    t = np.arange(0.0, 4.0, 1.0 / FRAME_RATE_HZ)
    x = np.zeros_like(t)
    x[500:600] = 1.0
    x[1200:1300] = -1.0

    result = osc.describe(t, x)

    assert not result.resolvable
    assert "not a frequency" in result.note


def test_irregular_crossings_are_refused_rather_than_averaged():
    """Half cycles of wildly different lengths average to a number that
    describes nothing."""
    rng = np.random.default_rng(3)
    t = np.arange(0.0, 8.0, 1.0 / FRAME_RATE_HZ)
    walk = np.cumsum(rng.normal(0.0, 0.05, t.size))

    result = osc.describe(t, walk)

    assert not result.resolvable
    assert "periodic" in result.note or "not a frequency" in result.note


def test_a_fast_oscillation_sampled_at_loop_rate_is_refused():
    """This is why the amplitudes are read from the frame stream and not from
    the loop's own record: at 100 Hz a 50 Hz oscillation is at the sampling
    limit, and the detector must say so rather than report it."""
    t, x = sampled(50.0, 0.4, rate_hz=LOOP_RATE_HZ, phase=0.3)

    result = osc.describe(t, x)

    assert not result.resolvable
    assert "unresolvable" in result.note


def test_the_same_oscillation_is_resolved_at_frame_rate():
    t, x = sampled(50.0, 0.4, rate_hz=FRAME_RATE_HZ, phase=0.3)

    result = osc.describe(t, x)

    assert result.resolvable
    assert result.frequency_hz == pytest.approx(50.0, rel=0.02)


def test_at_loop_rate_the_worst_oscillation_can_look_like_silence():
    """A joint alternating either side of target once per loop cycle is sampled
    at the same rate it swings. Land the samples on the crossings and the loop's
    own record shows a joint sitting perfectly still, while the frame record
    shows most of a degree of swing. No detector can recover that; only sampling
    faster can."""
    t_slow, x_slow = sampled(LOOP_RATE_HZ / 2, 0.4, rate_hz=LOOP_RATE_HZ)
    t_fast, x_fast = sampled(LOOP_RATE_HZ / 2, 0.4, rate_hz=FRAME_RATE_HZ)

    at_loop_rate = osc.describe(t_slow, x_slow)
    at_frame_rate = osc.describe(t_fast, x_fast)

    assert at_loop_rate.peak_to_peak_deg < 0.01
    assert at_frame_rate.peak_to_peak_deg == pytest.approx(0.8, rel=0.05)
    assert at_frame_rate.frequency_hz == pytest.approx(50.0, rel=0.02)


def test_a_record_that_opens_before_the_oscillation_reports_the_oscillation():
    """The shape of a dwell that had to be cut short: a slow excitation running
    throughout, and a fast oscillation that only starts partway in. The record
    then holds one long half cycle from before, among a hundred short ones. It
    is a single interval out of a hundred and it must not be allowed to set the
    answer — averaged, it halves the reported frequency and makes the spread
    look like noise."""
    t = np.arange(0.0, 2.0, 1.0 / FRAME_RATE_HZ)
    excitation = 2.0 * np.sin(2 * np.pi * 0.5 * t)
    signal = excitation + 5.0 * np.sin(2 * np.pi * 12.0 * t) * (t > 1.0)

    result = osc.describe(t, signal)

    assert result.resolvable
    assert result.frequency_hz == pytest.approx(12.0, rel=0.1)
    averaged = 1.0 / (2.0 * np.mean(result.half_cycle_s))
    assert averaged < 0.75 * result.frequency_hz, "the mean should have been fooled"


def test_the_spectrum_agrees_with_the_crossings():
    t, x = sampled(11.0, 0.6)

    result = osc.describe(t, x)

    assert result.spectral_peak_hz == pytest.approx(result.frequency_hz, rel=0.05)


def test_missing_samples_do_not_become_zeros():
    """A flagged joint has no angle. Treating it as zero would put a swing into
    the record that the joint never made."""
    t, x = sampled(5.0, 0.5)
    x[100:140] = np.nan

    result = osc.describe(t, x)

    assert result.peak_to_peak_deg == pytest.approx(1.0, rel=0.1)
    assert result.frequency_hz == pytest.approx(5.0, rel=0.1)


class TestOnset:
    def baseline_and(self, frequency, amplitude, *, clamp_fraction=0.0, baseline=0.05):
        t, x = sampled(frequency, amplitude / 2.0)
        return osc.onset_verdict(osc.describe(t, x), baseline, clamp_fraction)

    def test_a_swing_well_above_the_floor_at_a_clear_frequency_is_an_onset(self):
        verdict = self.baseline_and(9.0, 1.0)

        assert verdict.state == "oscillating"
        assert verdict.amplitude_ratio == pytest.approx(20.0, rel=0.1)

    def test_a_swing_at_the_noise_floor_is_not(self):
        verdict = self.baseline_and(9.0, 0.06, baseline=0.05)

        assert verdict.state == "quiet"
        assert "baseline" in verdict.reason

    def test_a_clamped_dwell_is_censored_rather_than_called_either_way(self):
        """The clamp bounds the amplitude, so the dwell cannot say how big the
        oscillation would have been. Calling it an onset would be reading the
        safety belt."""
        verdict = self.baseline_and(9.0, 1.0, clamp_fraction=0.5)

        assert verdict.state == "censored"
        assert not verdict.oscillating

    def test_a_large_swing_that_does_not_come_back_is_not_an_onset(self):
        """Something knocked the joint. That is not the loop oscillating."""
        rng = np.random.default_rng(5)
        t = np.arange(0.0, 4.0, 1.0 / FRAME_RATE_HZ)
        knock = np.zeros_like(t)
        knock[1000:1200] = 1.0
        verdict = osc.onset_verdict(
            osc.describe(t, knock + rng.normal(0, 0.01, t.size)), 0.05, 0.0
        )

        assert verdict.state == "quiet"
        assert "did not come back" in verdict.reason

    def test_an_untidy_period_costs_the_frequency_not_the_onset(self):
        """A joint swinging twenty times its noise floor, over and over, is
        unstable whether or not its half cycles keep good time. Withholding
        that because the period wandered would report the loop as healthy on
        the strength of an untidy diagnostic."""
        # A joint that went unstable partway through a short record: half its
        # crossings belong to what it was doing before, half to the ringing,
        # and no single period describes both.
        t = np.arange(0.0, 0.85, 1.0 / FRAME_RATE_HZ)
        signal = np.where(
            t < 0.6,
            4.0 * np.sin(2 * np.pi * 5.0 * t),
            4.0 * np.sin(2 * np.pi * 13.0 * t),
        )

        oscillation = osc.describe(t, signal)
        verdict = osc.onset_verdict(oscillation, 0.4, 0.0)

        assert not oscillation.resolvable, "this signal is meant to be untidy"
        assert oscillation.repeats
        assert verdict.state == "oscillating"
        assert "about" in verdict.reason

    def test_a_period_at_the_sampling_interval_claims_nothing(self):
        """Crossings that fast could be an alias of nearly anything, so neither
        the frequency nor the onset is claimed."""
        t, x = sampled(48.0, 1.0, rate_hz=LOOP_RATE_HZ, phase=0.3)

        verdict = osc.onset_verdict(osc.describe(t, x), 0.05, 0.0)

        assert verdict.state == "quiet"
        assert "sampling interval" in verdict.reason
