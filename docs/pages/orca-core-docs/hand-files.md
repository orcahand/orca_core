# Where a hand's files live

A model's `config.yaml` describes every hand of that model and is only ever read. What is measured on, or recorded
about, one physical hand is kept in that hand's own folder on your computer:

```
~/.orca/                              (set ORCA_HOME to put it elsewhere)
  hands/<hand id>/
    hand.yaml                         which hand this is: id, serials and board ids seen, model, nickname, first and last seen
    calibration.yaml                  what the last calibration measured, when, and its sequence number
    calibration.previous.yaml         the calibration before it, so one calibration can be undone
    joint_ranges.yaml                 joint ranges measured on this hand (optional)
    setup.yaml                        the latest setup record of each kind (slack test, sweep, travel baseline, ...)
    board.bin                         the calibration store last read off the hand's board
    archive/<kind>/<name>.yaml        older setup records and every sweep's full data, one file per record
```

Nothing is written into the installed package or beside a model's `config.yaml`.

## Which folder a hand gets

The hand's controller board is asked who it is when the hand connects. Its serial names the folder; a board with no
serial yet uses its own board id, and that folder is renamed to the serial once the board reports one. A hand on a
plain USB adapter, which reports nothing, is filed under `noboard-<model>`: every such hand of one model shares that
folder. Mock hands use a temporary folder and never touch `~/.orca`.

In Python, `hand.hand_id` and `hand.hand_dir` say which folder a connected hand uses. Before it connects, a hand has no
folder and no calibration loaded. Pass `calibration_path=` to `load_hand()` or `OrcaHand()` to read and write one
specific calibration file instead; the hand's other files still go to its folder.

## Calibration

`calibration.yaml` stores measurements only: motor limits, measured motor travel and drive currents, and (on hands that
have them) encoder anchors. The joint-to-motor ratio is worked out from the motor limits and the joint ranges in force
each time the file is loaded, so a calibration made before a change of joint ranges still maps every joint correctly.
Each joint also records which motor and directions its limits were measured with; if the model's motor mapping has
changed since, those joints read as uncalibrated until you calibrate them again.

A new calibration keeps the one it replaces as `calibration.previous.yaml`. `hand.restore_previous_calibration()` swaps
the two back, and `hand.calibration_info` gives the file in use, its `calibrated_at` and `sequence`, and the previous
one's. A calibration run says which file it wrote.

## The hand's board

A hand's board can keep a copy of its calibration. The first time a computer meets a hand it has no calibration for,
it copies the board's; after that the computer's copy is the one used. If the board later holds a newer calibration
(made on another computer), `hand.board_calibration_newer()` says so, and `hand.adopt_board_calibration()` takes it.
The board is written only at the end of a finished `scripts/calibrate.py` run on a real hand, on a computer with an
`orca_firmware` checkout (set `ORCA_FIRMWARE_REPO`, or keep it beside `orca_core`).

## Hardware version

The board's hardware version is used when it reports one. `--hardware-version` (or `hand.set_hardware_version()`)
overrides it for the current run only; it is never saved. A `hardware_version:` written into a `config.yaml` is used
only for a hand whose board reports none (a hand on a plain USB adapter). When the board reports a different version
than the config pins, the board's is used and the log says so.

## Ports

A motor port you name (`port:` in a config, or `--motor-port`) is the only one tried: if no motor answers there,
connecting fails rather than finding another hand. Leave `port: auto` to have it detected.

## Upgrading from orca_core 0.13 or older

Older versions kept `calibration.yaml`, `setup.yaml`, `local_config.yaml` and tensioning records beside the model's
`config.yaml`. They are not read, moved or deleted. When a hand first connects they are named in the log, and listed
in `hand.legacy_files` until a front-end has shown the notice (`hand.dismiss_legacy_files()`). Calibrate the hand again
to give its folder a calibration.
