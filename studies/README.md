# studies

Measurement tooling for the joint controller and the hand it drives. Nothing
here ships in the wheel: it exists to measure a hand, not to operate one.

Raw captures are not committed. They go to `studies/data/` (gitignored) and are
archived with the manifest's checksum; what lands in the repo is the findings,
the summary tables, and the drivers.

## Running the tests

```bash
uv run pytest studies/tests/
```

They run against a synthetic joint and mock hardware — no hand required.

## Layout

```
record/      capture: rings, sinks, recorders, session, reader
  schema.py       column layouts for every table
  ring.py         fixed-capacity buffer between a producer and the writer thread
  sink.py         where drained rows go (CSV by default)
  drain.py        the one thread that writes files
  loop_recorder.py    one row per control cycle
  frame_recorder.py   one row per encoder frame
  metadata.py     what a dataset carries to stay interpretable
  session.py      a dataset directory and its manifest
  reader.py       reading one back
plant.py     a synthetic joint: second order, backlash, transport delay
preconditions.py  checks that refuse to start a measurement
tests/       everything above, against mocks
```

## How a dataset is put together

```python
session = RecordingSession(path, experiment="...", metadata=collect_metadata(...))
session.add_table("loop", recorder.ring, recorder.columns, source=recorder)
```

Rules that the code enforces rather than documents:

- **A column lives in one table, at that table's cadence.** Nothing copies a
  slow value onto a fast row. `join_asof` is the only way to combine them and it
  stamps the age of everything it attaches. There is no interpolation anywhere.
- **A cycle that computed nothing records NaN, not zero.**
- **Every cycle is recorded, including the degraded ones.** A record covering
  only healthy cycles misreports the cadence exactly when the cadence is in
  question.
- **Raw encoder counts are kept**, so a run can be re-read under a different
  calibration.
- **A manifest says how the run ended.** One still marked `running` was cut
  short, and the reader reports it as truncated rather than presenting a partial
  table as a whole one.

## Findings

Each experiment writes a document under `findings/` with, at minimum: what was
measured, how, what it shows, **what decisions it unblocks**, and **what
decisions it does not support**. The last section is the point — a measurement
quoted beyond what it establishes is worse than no measurement.

| experiment | status | hands | headline |
|---|---|---|---|
| [encoder-stream delivery](findings/encoder-stream-delivery.md) | done 2026-08-11 | ser-9964 (right) | 500 Hz confirmed, no loss on its own; per-taxel tactile costs 40% of frames and reaches the integrator-freeze tier |
