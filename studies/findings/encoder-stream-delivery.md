# Encoder-stream delivery

**Hand** `ser-9964`, orcahand-full-right, hw2 / fw2, board 4B7685ABAA282225
**Date** 2026-08-11 · **Duration** 120 s per condition · **Motion** none; torque never enabled
**Data** `studies/data/20260811T16{2710,3051,3308}_timing_census_nohand`

## Question

Does the encoder stream deliver what the board emits, and can it resolve motion
faster than the control loop samples? Everything downstream is sampled through
it, so a defect here is inherited by every later measurement.

## Result

| | encoder only | + resultant tactile | + per-taxel tactile |
|---|---|---|---|
| rate | **499.6 Hz** | 450.1 Hz | **299.8 Hz** |
| frames delivered | 100% | 90% | **60%** |
| gap p90 | 3.0 ms | 4.0 ms | 10.1 ms |
| gap p99 | 4.1 ms | 5.2 ms | 12.2 ms |
| worst gap | 14.7 ms | 17.1 ms | **110.9 ms** |
| gaps over the warn tier | 0 | 1 | 13 |
| gaps over the integrator-freeze tier | 0 | 0 | **1** |
| slots reporting faults | none | none | none |

**The board emits at 500 Hz, and on its own the link delivers all of it.** The
2 ms firmware period is confirmed against the flashed firmware, not just the
source. Every slot was clean across all three conditions.

**Tactile on the same link costs encoder frames, and per-taxel mode costs
40% of them.** The loss is structured, not random: under per-taxel streaming
13.9% of gaps are *exactly* five emit periods, which is a tactile frame holding
the transmit path for ~10 ms while four encoder frames are dropped at the board.
Resultant mode costs 10% and concentrates at two periods.

**The mechanism differs between conditions, and the data says which.** With no
tactile, a long gap is followed by one averaging 0.078 ms: frames buffered and
released together, so nothing was lost — the delivery path re-timed about 10% of
them without dropping any. With tactile, the gap that follows averages 1.3–1.4 ms
and the rate falls: frames genuinely never arrived.

## Decisions this unblocks

**Sampling is not the limiting factor for measuring the loop.** At 500 Hz with a
p99 gap of 4 ms, the stream resolves anything the outer loop can act on at
100 Hz, including a two-cycle alternation at the loop's own Nyquist frequency.
Step-response and oscillation work can proceed on encoder-only runs.

**Delivery is not currently degrading the control loop, absent tactile.** No gap
came within a factor of three of the first watchdog tier.

**Per-taxel tactile streaming degrades the joint loop measurably.** One gap of
110.9 ms means a running loop would have gone ~11 cycles on a stale
measurement, past the tier where it freezes its integrator. Resultant mode does
not reach that tier but still discards a tenth of the stream.

## Decisions this does NOT support

- **Nothing about latency.** Both timestamps are host-side, so transport delay
  from sensor to host is not measurable here at any precision. Only spacing and
  loss are.
- **Nothing about wire jitter.** Host scheduling and delivery jitter are
  indistinguishable from this data alone; the ~10% re-timing in the baseline is
  as consistent with the host as with the link.
- **Nothing about the loop running.** Every condition had the loop off and the
  motor bus closed. Loop-on delivery is unmeasured and can only be worse.
- **Nothing about the left hand**, or about any hand but this one.

## Follow-ups

- Repeat with the loop running and the motor bus open; the difference is the
  cost the loop imposes on its own sensing.
- A one-byte sequence counter in the frame would separate frames the board never
  sent from frames lost in transit. The byte already exists, is transmitted, sits
  inside the checksum, and is currently a constant zero the host ignores.
- Whether per-taxel streaming is ever run alongside closed-loop control. If so,
  the loss above is a control problem, not a sensing one.
