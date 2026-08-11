"""Deciding whether a joint is oscillating, and at what frequency.

The frequency is the useful part. An error signal that swings a degree tells you
the loop is unhappy; how fast it swings tells you what to change:

* around half the loop rate, the loop is alternating cycle-to-cycle against a
  mechanism that has already finished moving. Nothing mechanical is limiting it,
  so a faster loop buys margin more or less proportionally.
* between roughly a tenth and a quarter of the loop rate, real mechanical lag is
  setting the limit and a faster loop buys little; the gains have to come down
  to fit the lag that is there.
* a few cycles a second or slower is not a gain problem at all. That is the
  integrator hunting across backlash, and lowering the proportional gain will
  not touch it.

So the detector has to resolve a frequency, not merely notice motion, and it has
to do it over the two decades between those cases. Counting sign changes cycle by
cycle only finds the fastest one; everything here works from the intervals
between zero crossings instead.

Two things make a zero crossing untrustworthy near zero: encoder quantisation,
which flips the sign of a signal sitting on a count boundary at the sample rate,
and slow drift, which parks the signal on one side and hides real crossings. So
the signal is detrended first, and crossings only count once the signal has
travelled past a band on either side.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from orca_core.hardware.sensing.constants import ENCODER_LSB_DEG


# Excursion a signal has to make either side of zero before its crossings are
# believed. Four counts: enough that quantisation alone cannot manufacture a
# crossing, small enough to keep a fraction-of-a-degree oscillation visible.
DEFAULT_HYSTERESIS_DEG = 4 * ENCODER_LSB_DEG

# Fewer than this many half cycles is not a frequency, it is one or two events.
MIN_HALF_CYCLES = 4

# Above this fraction of the sample rate a period is a handful of samples and
# its length is quantised by the sampling, not measured by it.
MAX_RESOLVABLE_RATE_FRACTION = 0.4

# Spread of half-cycle durations, relative to their median, above which the
# crossings are not periodic. A self-excited oscillation keeps time; noise
# crossing a band does not.
MAX_IRREGULARITY = 0.4

# Peak-to-peak, as a multiple of the same joint's quiet baseline, at which the
# motion is the loop's own rather than the noise it always has.
SUSTAINED_RATIO = 3.0

# A dwell whose correction spent more than this fraction of its samples against
# the clamp is not reporting the loop's own amplitude — the clamp is. No onset
# is reportable from one.
MAX_CLAMP_FRACTION = 0.10


@dataclass
class Oscillation:
    """What one stretch of an error signal did."""

    samples: int
    duration_s: float
    sample_rate_hz: float
    peak_to_peak_deg: float
    frequency_hz: float
    half_cycles: int
    irregularity: float
    spectral_peak_hz: float
    resolvable: bool
    at_sampling_limit: bool = False
    note: str = ""
    half_cycle_s: List[float] = field(default_factory=list, repr=False)

    @property
    def repeats(self) -> bool:
        """Whether the signal came back often enough to be something other than
        a disturbance. Weaker than :attr:`resolvable`, which also asks whether
        the period it repeats at can be trusted."""
        return self.half_cycles >= MIN_HALF_CYCLES and bool(
            np.isfinite(self.frequency_hz)
        )

    def as_dict(self) -> Dict[str, object]:
        return {
            "samples": self.samples,
            "duration_s": round(self.duration_s, 4),
            "sample_rate_hz": round(self.sample_rate_hz, 1),
            "peak_to_peak_deg": round(self.peak_to_peak_deg, 4),
            "frequency_hz": (
                round(self.frequency_hz, 3) if np.isfinite(self.frequency_hz) else None
            ),
            "half_cycles": self.half_cycles,
            "irregularity": (
                round(self.irregularity, 3) if np.isfinite(self.irregularity) else None
            ),
            "spectral_peak_hz": (
                round(self.spectral_peak_hz, 3)
                if np.isfinite(self.spectral_peak_hz)
                else None
            ),
            "resolvable": self.resolvable,
            "repeats": self.repeats,
            "at_sampling_limit": self.at_sampling_limit,
            "note": self.note,
        }


@dataclass
class Verdict:
    """Whether a dwell showed the loop oscillating on its own.

    ``state`` is ``"quiet"``, ``"oscillating"``, or ``"censored"``. Censored is
    not a weaker "oscillating": it means the clamp was holding the amplitude
    down, so the dwell cannot say how big the oscillation would have been, and
    an onset read off it would be a reading of the clamp.
    """

    state: str
    reason: str
    oscillation: Oscillation
    baseline_p2p_deg: float
    amplitude_ratio: float
    clamp_fraction: float

    @property
    def oscillating(self) -> bool:
        return self.state == "oscillating"

    def as_dict(self) -> Dict[str, object]:
        return {
            "state": self.state,
            "reason": self.reason,
            "baseline_p2p_deg": round(self.baseline_p2p_deg, 4),
            "amplitude_ratio": (
                round(self.amplitude_ratio, 2)
                if np.isfinite(self.amplitude_ratio)
                else None
            ),
            "clamp_fraction": round(self.clamp_fraction, 4),
        }


def detrend(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Remove the straight line through the signal.

    A standing correction and a slow creep both put the signal off zero, and a
    zero-crossing detector reads that as silence.
    """
    finite = np.isfinite(x)
    if np.count_nonzero(finite) < 2:
        return x - np.nanmean(x) if np.any(finite) else np.zeros_like(x)
    slope, intercept = np.polyfit(t[finite], x[finite], 1)
    return x - (slope * t + intercept)


def peak_to_peak(x: np.ndarray) -> float:
    """Full swing of the signal, ignoring samples that were never measured."""
    if not np.any(np.isfinite(x)):
        return float("nan")
    return float(np.nanmax(x) - np.nanmin(x))


def zero_crossings(
    t: np.ndarray, x: np.ndarray, hysteresis: float = DEFAULT_HYSTERESIS_DEG
) -> np.ndarray:
    """Times at which the signal crossed zero going somewhere.

    A crossing only counts once the signal has been past ``hysteresis`` on both
    sides of it, which is what keeps quantisation noise from producing a
    crossing per sample. The time returned is interpolated between the two
    samples that straddle zero, so the period does not inherit the sample
    interval as its resolution.
    """
    values = np.where(np.isfinite(x), x, 0.0)
    state = _confirmed_side(values, float(hysteresis))
    flips = np.nonzero((state[1:] != state[:-1]) & (state[1:] != 0) & (state[:-1] != 0))[0]
    # Index i marks a crossing between samples i and i+1. Written this way
    # rather than as a sign product so a sample landing exactly on zero — which
    # quantised data does often — still counts as a crossing.
    straddles = np.nonzero((values[:-1] <= 0) != (values[1:] <= 0))[0]

    times: List[float] = []
    for flip in flips:
        # The crossing is the last one before the signal committed to the new
        # side. Anything earlier belongs to a wobble the hysteresis already
        # refused to call a crossing.
        earlier = straddles[straddles <= flip]
        if earlier.size == 0:
            continue
        crossing = _interpolate_crossing(t, values, int(earlier[-1]))
        if not times or crossing > times[-1]:
            times.append(crossing)
    return np.array(times, dtype=np.float64)


def _confirmed_side(x: np.ndarray, band: float) -> np.ndarray:
    """Which side of zero the signal is committed to at each sample."""
    side = np.zeros(x.size, dtype=np.int8)
    current = 0
    for i in range(x.size):
        if x[i] > band:
            current = 1
        elif x[i] < -band:
            current = -1
        side[i] = current
    return side


def _interpolate_crossing(t: np.ndarray, x: np.ndarray, i: int) -> float:
    """Where between samples ``i`` and ``i+1`` the signal passed zero."""
    span = x[i + 1] - x[i]
    if span == 0:
        return float(t[i])
    return float(t[i] + (t[i + 1] - t[i]) * (-x[i] / span))


def spectral_peak_hz(t: np.ndarray, x: np.ndarray) -> float:
    """Frequency carrying the most energy, as an independent check.

    The verdict is not taken from here. It is a second opinion formed a
    different way, and the two disagreeing is itself worth seeing: crossings
    describe the largest excursion, a spectrum describes the most energy, and a
    signal where those are different frequencies is not the simple ringing the
    rest of this module assumes.
    """
    finite = np.isfinite(x)
    if np.count_nonzero(finite) < 16:
        return float("nan")
    values = x[finite] - np.mean(x[finite])
    times = t[finite]
    duration = times[-1] - times[0]
    if duration <= 0:
        return float("nan")
    rate = (values.size - 1) / duration
    spectrum = np.abs(np.fft.rfft(values * np.hanning(values.size)))
    frequencies = np.fft.rfftfreq(values.size, d=1.0 / rate)
    return float(frequencies[1:][np.argmax(spectrum[1:])]) if values.size > 2 else float("nan")


def describe(
    t: np.ndarray,
    x: np.ndarray,
    *,
    hysteresis: float = DEFAULT_HYSTERESIS_DEG,
    min_half_cycles: int = MIN_HALF_CYCLES,
) -> Oscillation:
    """Measure one stretch of signal: how far it swung and how fast.

    ``resolvable`` is the honest part. A window holding two swings cannot
    distinguish a periodic oscillation from two disturbances, and a period of
    three samples is a reading of the sample rate. Both come back with the
    frequency withheld rather than estimated.
    """
    t = np.asarray(t, dtype=np.float64)
    x = np.asarray(x, dtype=np.float64)
    if t.size != x.size:
        raise ValueError("times and values must be the same length")
    if t.size < 2:
        return Oscillation(
            samples=int(t.size),
            duration_s=0.0,
            sample_rate_hz=float("nan"),
            peak_to_peak_deg=float("nan"),
            frequency_hz=float("nan"),
            half_cycles=0,
            irregularity=float("nan"),
            spectral_peak_hz=float("nan"),
            resolvable=False,
            note="no samples",
        )

    duration = float(t[-1] - t[0])
    rate = (t.size - 1) / duration if duration > 0 else float("nan")
    residual = detrend(t, x)
    swing = peak_to_peak(residual)

    crossings = zero_crossings(t, residual, hysteresis)
    half_cycle_s = np.diff(crossings)
    count = int(half_cycle_s.size)

    if count < min_half_cycles:
        return Oscillation(
            samples=int(t.size),
            duration_s=duration,
            sample_rate_hz=rate,
            peak_to_peak_deg=swing,
            frequency_hz=float("nan"),
            half_cycles=count,
            irregularity=float("nan"),
            spectral_peak_hz=spectral_peak_hz(t, residual),
            resolvable=False,
            note=f"{count} half cycles in {duration:.2f}s is not a frequency",
            half_cycle_s=[float(v) for v in half_cycle_s],
        )

    # Median rather than mean, for both the period and its spread. A record can
    # hold a stretch of something else — the tail of a disturbance, the last of
    # an excitation, the quiet before the oscillation started — and one long
    # half cycle from it moves a mean-derived frequency by a factor of two and a
    # standard deviation by far more. The median describes the oscillation that
    # dominates the record, which is the one being asked about.
    median_half = float(np.median(half_cycle_s))
    if median_half <= 0:
        return Oscillation(
            samples=int(t.size),
            duration_s=duration,
            sample_rate_hz=rate,
            peak_to_peak_deg=swing,
            frequency_hz=float("nan"),
            half_cycles=count,
            irregularity=float("nan"),
            spectral_peak_hz=spectral_peak_hz(t, residual),
            resolvable=False,
            note="crossings share a timestamp, so they have no duration",
            half_cycle_s=[float(v) for v in half_cycle_s],
        )
    frequency = 1.0 / (2.0 * median_half)
    # Scaled so it reads on the same scale as a coefficient of variation would.
    irregularity = float(
        1.4826 * np.median(np.abs(half_cycle_s - median_half)) / median_half
    )

    resolvable = True
    at_sampling_limit = False
    note = ""
    if np.isfinite(rate) and frequency > MAX_RESOLVABLE_RATE_FRACTION * rate:
        resolvable = False
        at_sampling_limit = True
        note = (
            f"{frequency:.1f} Hz is unresolvable at {rate:.0f} Hz sampling; "
            "the period is a few samples long"
        )
    elif irregularity > MAX_IRREGULARITY:
        resolvable = False
        note = (
            f"half cycles vary by {irregularity * 100:.0f}% of their median, so "
            "the crossings are not periodic"
        )

    return Oscillation(
        samples=int(t.size),
        duration_s=duration,
        sample_rate_hz=rate,
        peak_to_peak_deg=swing,
        frequency_hz=frequency,
        half_cycles=count,
        irregularity=irregularity,
        spectral_peak_hz=spectral_peak_hz(t, residual),
        resolvable=resolvable,
        at_sampling_limit=at_sampling_limit,
        note=note,
        half_cycle_s=[float(v) for v in half_cycle_s],
    )


def onset_verdict(
    oscillation: Oscillation,
    baseline_p2p_deg: float,
    clamp_fraction: float,
    *,
    sustained_ratio: float = SUSTAINED_RATIO,
    max_clamp_fraction: float = MAX_CLAMP_FRACTION,
) -> Verdict:
    """Decide whether a dwell caught the loop oscillating on its own.

    Three things have to hold at once, and each one is there because of a way
    this measurement can lie:

    * the swing has to stand above the same joint's quiet baseline, because every
      joint always swings a little;
    * it has to come back, repeatedly, because one knock is not an oscillation;
    * the correction has to have stayed off its clamp, because a clamped loop
      settles into a bounded limit cycle whose size is the clamp's, and reading
      an onset off that would be reading the safety belt.

    Whether the *frequency* can be trusted is a separate question, and
    deliberately not part of this one. A joint swinging twenty times its own
    noise floor, over and over, is unstable whether or not its half cycles keep
    good time; withholding that because the period wandered would be reporting
    the loop as healthy on the strength of an untidy diagnostic. Where the
    period is untidy the verdict says so and the frequency is marked
    approximate. The one exception is a period down at the sampling interval:
    there the crossings could be an alias of nearly anything, so nothing is
    claimed at all.
    """
    ratio = (
        oscillation.peak_to_peak_deg / baseline_p2p_deg
        if baseline_p2p_deg > 0
        else float("inf")
    )

    if clamp_fraction > max_clamp_fraction:
        return Verdict(
            state="censored",
            reason=(
                f"the correction was against its clamp on "
                f"{clamp_fraction * 100:.0f}% of samples, so the amplitude here "
                "is the clamp's and not the loop's"
            ),
            oscillation=oscillation,
            baseline_p2p_deg=baseline_p2p_deg,
            amplitude_ratio=ratio,
            clamp_fraction=clamp_fraction,
        )

    if not np.isfinite(ratio) or ratio < sustained_ratio:
        return Verdict(
            state="quiet",
            reason=(
                f"swing {oscillation.peak_to_peak_deg:.3f}° is "
                f"{ratio:.1f}x the {baseline_p2p_deg:.3f}° baseline"
            ),
            oscillation=oscillation,
            baseline_p2p_deg=baseline_p2p_deg,
            amplitude_ratio=ratio,
            clamp_fraction=clamp_fraction,
        )

    if not oscillation.repeats:
        return Verdict(
            state="quiet",
            reason=(
                f"swing is {ratio:.1f}x baseline but it did not come back: "
                f"{oscillation.half_cycles} half cycles is a disturbance, not an "
                "oscillation"
            ),
            oscillation=oscillation,
            baseline_p2p_deg=baseline_p2p_deg,
            amplitude_ratio=ratio,
            clamp_fraction=clamp_fraction,
        )

    if oscillation.at_sampling_limit:
        return Verdict(
            state="quiet",
            reason=(
                f"swing is {ratio:.1f}x baseline but its period is down at the "
                f"sampling interval, so what it is cannot be said: "
                f"{oscillation.note}"
            ),
            oscillation=oscillation,
            baseline_p2p_deg=baseline_p2p_deg,
            amplitude_ratio=ratio,
            clamp_fraction=clamp_fraction,
        )

    about = "" if oscillation.resolvable else "about "
    return Verdict(
        state="oscillating",
        reason=(
            f"{oscillation.peak_to_peak_deg:.3f}° at {about}"
            f"{oscillation.frequency_hz:.1f} Hz, {ratio:.1f}x baseline, "
            "with the excitation removed"
            + ("" if oscillation.resolvable else f" ({oscillation.note})")
        ),
        oscillation=oscillation,
        baseline_p2p_deg=baseline_p2p_deg,
        amplitude_ratio=ratio,
        clamp_fraction=clamp_fraction,
    )
