# Local hand configs

Everything in this directory except this file (and `.gitignore`) is
gitignored — put machine-specific hand configs here rather than editing a
bundled model under `v1/`/`v2/`, which is shared, read-only-in-spirit, and
overwritten on every upgrade.

The usual reason to need one: a hand old enough to predate the identity
protocol (`ORCA_ID?`/`ORCA_INFO?`) reports no side and, with more than one
such hand attached, autodetection can't tell them apart at all — see the
warning `detect_hands()`/`scripts/detect.py` print in that case. Pin each
hand's ports explicitly instead:

```
orca_core/models/local/<name>/config.yaml
```

Copy a bundled model that matches the hand's capabilities as a starting
point (e.g. `orca_core/models/v2/orcahand-touch-right/config.yaml` for a
Feetech/Dynamixel hand with an external Paxini board), then set:

- `port:` — the hand's motor bus, e.g. `/dev/cu.usbserial-XXXX`
- `sensors: port:` — its tactile adapter, if it has one (omit the whole
  `sensors:` block for a motor-only hand)

Then pass that path wherever a script/API takes `config_path`, e.g.:

```
uv run python scripts/detect.py orca_core/models/local/hand-a/config.yaml
```

Calibrating writes `calibration.yaml` next to it, which is also gitignored.
Pick `<name>` freely — it's just a label for you to tell your hands apart;
nothing in orca_core reads it.
