# Gain margin and onset frequency

**Status** built, not yet run on a hand. The protocol below is what the driver
does; the results section is empty until it has run and stays empty rather than
being filled with expectations.

**Driver** `uv run python studies/run_gain_margin.py <config.yaml>`

## Question

The outer loop ships at half the proportional gain the linear analysis calls
marginally stable, and it does oscillate at that gain. That much is expected and
is not the question. The question is per joint: **how much margin is actually
there, and is 0.5 — 0.35 on the two stiffest thumb joints — in the right place
for each of them?**

And the more useful half: **at what frequency does it oscillate when it goes?**
That number, not the gain, says which of the three avenues is worth spending
effort on.

| onset frequency | what is setting the limit | what follows |
|---|---|---|
| near half the loop rate | nothing mechanical; the loop is alternating against a plant that has already arrived | a faster loop buys margin roughly in proportion |
| a tenth to a quarter of the loop rate | real mechanical lag | a faster loop buys little; the gains have to fit the lag, per joint |
| a few cycles a second | backlash and the integrator hunting across it | neither gain nor loop rate is the cure |

## Method

One joint at a time, every other joint's loop set to zero gain so the motion
measured belongs to one loop. The wrist is never probed: it runs a position mode
with no current ceiling.

Per joint, at the middle of the travel both the config and the measured ROM
agree on:

1. **A baseline dwell at the configured gains** — 20 s of excitation, then 10 s
   quiet. The swing over the quiet stretch is that joint's noise floor, and
   every later dwell is judged against it.
2. **A ramp**, geometric at ×1.25, dwelling 4 s excited then 5 s quiet at each
   step. `--arm kp` raises the proportional gain with the integrator off, which
   is the clean linear measurement; `--arm ki` raises the integral gain at the
   configured proportional gain, which is where a nonlinear mechanism shows.
   The integrator is discharged between dwells, so no dwell inherits the
   previous tuning's stored demand.
3. Every dwell is **excited at ±2°, 0.5 Hz**, and measured **after** that stops.
   A marginally stable loop sits still if nothing disturbs it, and an
   oscillation still running once the excitation has gone is the loop's own.

Onset is declared on a swing that stands well above the joint's own baseline, at
a frequency that resolves into regular half cycles, with the correction off its
clamp. All three have to hold; each is there because of a specific way this
measurement lies (see `studies/analysis/oscillation.py`).

**Everything is measured on the encoder frame stream at 500 Hz, never on the
loop's own 100 Hz record.** An oscillation near half the loop rate appears there
as a steady offset — a joint sitting quietly off target — which is the one
result that would be read as the opposite of what it is.

The probe stops itself: on the detected onset, on the joint reaching the ends of
its travel, on swinging more than 10° inside 0.4 s, on the loop e-stopping, and
on request. It returns the joint to its base pose with the gains it found, and
leaves torque on throughout.

## Results

Not yet run.

## Decisions this unblocks

Pending the run. It is meant to answer: whether the loop-efficiency avenue is
alive at all, whether the shipped gains are conservative or generous per joint,
and whether the cure for any joint is Kp, Ki, or neither.

## Decisions this does NOT support

- **Nothing about accuracy.** A margin is not an error.
- **Nothing about loaded operation.** Every dwell is free-space.
- **Nothing about a joint's margin at another pose.** Each joint is probed at
  one pose, in the middle of its travel.
- **Nothing whose dwell was censored.** A dwell spent against the correction
  clamp reports the clamp's amplitude, not the loop's, and no onset is read from
  one.
