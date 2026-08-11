# Gain margin and onset frequency

**Hand** `ser-9964`, orcahand-full-right, hw2 / fw2, board 4B7685ABAA282225
**Date** 2026-08-11 · **Joints** 3 of 16 · **Arm** proportional gain, integrator off
**Data** `studies/data/20260811T17{4400,4631,5535}_gain_margin_right`
**Driver** `uv run python studies/run_gain_margin.py <config.yaml> --joints <joint>`

> **Shakedown grade.** These runs were made to exercise the probe, and no tendon
> history was declared for them. Tension state moves everything measured here, so
> treat the numbers as this hand on this afternoon, and re-run the set with a
> declared history before anything is tuned against them.

## Question

The loop ships at half the proportional gain the linear analysis calls
marginally stable, and it does oscillate at that gain. How much margin is
actually there per joint — and at what frequency does it go, which is the number
that says whether a faster loop would help.

## Result

| joint | shipped Kp | quiet through | oscillated at | margin | frequency |
|---|---|---|---|---|---|
| index_mcp | 0.50 | 2.38 | 2.98 | 4.8–6.0× | **11.1 Hz** |
| index_pip | 0.50 | 2.98 | 3.73 | 6.0–7.5× | **11.1 Hz** |
| thumb_cmc | 0.35 | 1.67 | 2.09 | 4.8–6.0× | not resolved |

Baseline swing, held still at the shipped gains: 0.29–0.57°, or 13–26 encoder
counts. Every onset was reached with the correction off its clamp (≤3% of
samples), so none of these is censored.

**Every joint that gave a frequency gave about 11 Hz.** index_mcp was probed
three times and blew up at exactly the same ramp step each time, at 10.7, 11.6
and 11.1 Hz. That is a hundred times slower than the loop's own cycle and
around a tenth of its rate.

**With the integrator off, the shipped proportional gain sits at a fifth to a
seventh of where the joint goes unstable.** The thumb's reduced 0.35 is
proportionate rather than extra-conservative: its margin ratio matches the
fingers', it is only its absolute onset that is lower.

**The transition is a cliff, not a slope.** At every gain up to the last, the
joint swings its noise floor and nothing more — 0.24 to 0.53°, no trend. One
ramp step later, 25% more gain, it swings 10–11°. Nothing grew on the way. A
linear loop approaching its stability limit does not do this; something
threshold-like is involved, and it means the ×1.25 ramp step is the entire
resolution available on where the margin is.

**Consequently every onset here was caught by the amplitude limit, not by the
growth detector.** The probe's 10°-in-0.4 s abort fired inside the same dwell
the oscillation started in, and each onset was read from the stretch that
tripped it.

## Decisions this unblocks

**The loop-rate avenue looks weak, on this evidence.** An oscillation at 11 Hz
against a 100 Hz loop is not a loop alternating against a mechanism that has
already arrived — that would show up near 50 Hz. It is mechanical lag setting
the limit, and against lag of that scale a faster loop buys little. Three joints
is not sixteen, but all three agree and none is near the band where raising the
rate would pay.

**Per-joint retuning is the avenue with room in it**, and Stage 3's step
responses are what should set the numbers rather than a probe like this one.

## Decisions this does NOT support

- **Nothing about raising Kp on a shipped hand.** These onsets were measured
  with the integrator switched off. The hand runs Ki=5, which spends margin;
  the integral arm has not been run. A fivefold margin with no integrator is
  not a fivefold margin.
- **Nothing about the other thirteen joints**, or about any hand but this one.
- **No frequency for thumb_cmc.** Its crossings do not keep time — the estimate
  moved between 6 and 12 Hz depending on the window — so it is recorded as
  oscillating without a frequency rather than with a number that would not
  survive being asked twice.
- **Nothing about loaded operation.** Free-space, one pose per joint, mid-travel.
- **Nothing about where the margin is inside a bracket.** The transition is
  sharp and the ramp step is 25%; the onset is somewhere in that interval and
  the data says nothing finer.

## Follow-ups

- Run the integral arm. Given the cliff, the interesting question is whether Ki
  moves the onset gain or changes what happens below it.
- The remaining thirteen joints, with a declared tendon history.
- thumb_cmc carries 0.4–0.5° of 28–39 Hz swing in every quiet dwell, at every
  gain including the shipped one, and the amplitude does not follow the gain.
  Something is exciting that joint independently of the loop.
- The cliff itself is worth a look: a threshold that has to be crossed before
  the joint answers at all would also explain a standing error that never
  integrates away.
