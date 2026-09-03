#!/usr/bin/env python
"""Split the command-to-motion delay into transport and mechanics.

Steps one joint repeatedly and times two things against the same command
instant: when the motor shaft starts turning, and when the joint follows. The
first is host, USB-to-UART bridge, serial and servo — the only part a faster
host path could change. The second is tendon slack and compliance, which no
software change touches.

The joint loop is deliberately not engaged: it would interleave its own writes
with the step and poll the same bus.

Usage:
    uv run python scripts/measure_latency.py --joint index_mcp
    uv run python scripts/measure_latency.py --joint index_mcp --trials 20 --step 12
    uv run python scripts/measure_latency.py --joint index_mcp --no-encoder
"""
import argparse

from orca_core.control.control_auxiliary import (
    DEFAULT_STEP_DEG,
    DEFAULT_TRIALS,
    measure_latency,
    summarize_latency,
)
from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.joint_encoder_client import (
    EncodersNotAvailableError,
    JointEncoderClient,
)
from orca_core.hardware.sensing.constants import (
    JOINT_TO_ENCODER_SLOT,
    joint_encoder_polarity_for_side,
)
from orca_core.hardware.sensing.encoder_protocol import encoder_to_joint_angle
from orca_core.hardware.sensing.serial_discovery import resolve_sensing_ports
from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    shutdown_hand,
)

import numpy as np


RST = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"


def open_encoder_reader(hand, joint, port_override):
    """Return ``(link, client, read_deg)`` for one joint's live angle.

    Decoded exactly as the control loop does — same anchor, polarity and ROM
    frame — so the angle here means the same thing it means everywhere else.
    """
    # "auto" is what triggers discovery; anything else is taken as a literal
    # device path, so an unset flag has to fall through to the config value.
    ports = resolve_sensing_ports(
        tactile_override="disabled",
        encoder_override=port_override or hand.config.encoder_serial_port,
    )
    if ports.encoder is None:
        raise RuntimeError("no encoder port found; pass --encoder-port")
    link = HandSerialLink(ports.encoder, baudrate=hand.config.encoder_baudrate)
    link.connect()
    client = JointEncoderClient(link)
    client.connect()
    client.start_stream(timeout=2.0)

    slot = JOINT_TO_ENCODER_SLOT[joint]
    anchor = hand.calibration.joint_encoder_calibration_dict[joint].enc_at_anchor_count
    polarity = joint_encoder_polarity_for_side(hand.config.type)[joint]
    anchor_angle = hand.effective_joint_roms_dict[joint][1]

    def read_deg():
        reading = client.get_latest_unfiltered()
        if reading is None:
            return None
        raw = np.asarray(reading.raw_counts)[slot]
        return float(encoder_to_joint_angle(
            np.array([raw]), np.array([anchor]),
            np.array([polarity]), np.array([anchor_angle]),
        )[0])

    print(f"Encoder stream active on {ports.encoder}")
    return link, client, read_deg


def print_progress(event):
    if event["event"] == "trial_started":
        print(f"\n[{event['index']:>3}] {event['joint']}  "
              f"{event['origin']:.1f} -> {event['target']:.1f} deg")
        return
    trial = event["trial"]
    def ms(value):
        return "  n/a" if value is None else f"{value * 1000:5.1f}"
    print(f"      current {ms(trial.command_to_current_s)}   "
          f"velocity {ms(trial.command_to_velocity_s)}   "
          f"shaft {ms(trial.command_to_motor_s)}   "
          f"joint {ms(trial.command_to_joint_s)} ms   "
          f"{DIM}(polled {trial.poll_hz:.0f} Hz){RST}")


def print_summary(summary):
    if summary is None:
        print("\nNo trials completed.")
        return
    def ms(value):
        return "n/a" if value is None else f"{value * 1000:.1f} ms"
    print(f"\n{BOLD}Latency breakdown — {summary.joint} (motor {summary.motor_id}){RST}")
    print(f"  trials             {summary.trials}")
    print(f"  -> current draw    {ms(summary.command_to_current_median_s)} median   "
          f"{ms(summary.command_to_current_p90_s)} p90   "
          f"{DIM}TRANSPORT: host + bridge + servo{RST}")
    print(f"  -> shaft velocity  {ms(summary.command_to_velocity_median_s)} median          "
          f"     {DIM}+ rotor inertia{RST}")
    print(f"  -> shaft position  {ms(summary.command_to_motor_median_s)} median          "
          f"     {DIM}+ threshold crossing{RST}")
    print(f"  -> joint moves     {ms(summary.command_to_joint_median_s)} median          "
          f"     {DIM}+ tendon slack — what the loop sees{RST}")
    print(f"  motor poll rate    {summary.poll_hz_median:.0f} Hz "
          f"{DIM}(resolution of the electrical figure){RST}")
    print(f"\n  {summary.verdict}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Split command-to-motion delay into transport and mechanics."
    )
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument("--joint", required=True, help="Joint to step, e.g. index_mcp.")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help=f"Steps to perform, half each way (default {DEFAULT_TRIALS}).")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG,
                        help=f"Step size in joint degrees (default {DEFAULT_STEP_DEG}).")
    parser.add_argument("--encoder-port", default=None,
                        help="Override the joint-encoder port.")
    parser.add_argument("--no-encoder", action="store_true",
                        help="Measure only the electrical half; skip the joint encoder.")
    args = parser.parse_args()

    # engage_feedback=False: the joint loop would interleave writes with the
    # step and contend for the bus we are polling.
    # engage_sensors=False as well: an OrcaHandTouch holds the sensing CDC
    # open for tactile, and the joint-encoder reader below needs that port.
    hand = create_hand_from_args(
        args, engage_feedback=False, engage_sensors=False
    )
    link = client = None
    try:
        connect_hand(hand)
        if args.joint not in hand.config.joint_ids:
            print(f"Unknown joint {args.joint!r}; this hand has "
                  f"{', '.join(hand.config.joint_ids)}")
            return 2
        hand.init_joints(move_to_neutral=False)

        read_deg = None
        if not args.no_encoder:
            try:
                link, client, read_deg = open_encoder_reader(
                    hand, args.joint, args.encoder_port
                )
            except (EncodersNotAvailableError, RuntimeError, KeyError) as exc:
                print(f"Joint encoder unavailable ({exc}); measuring the "
                      "electrical half only.")

        trials = measure_latency(
            hand, args.joint, read_deg,
            trials=args.trials, step_deg=args.step,
            progress_callback=print_progress,
        )
        print_summary(summarize_latency(trials))
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        if client is not None:
            client.disconnect()
        if link is not None:
            link.disconnect()
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
