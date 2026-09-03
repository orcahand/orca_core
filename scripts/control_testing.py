#!/usr/bin/env python
"""Drive one finger through a sequence of waypoints and log how it tracked them.

Prompts for the finger, then for whether to follow a preset trajectory CSV or
sample random targets inside each joint's reachable range. A new waypoint is
issued as soon as the finger stops moving, and every waypoint's targets,
settle time and residual error are appended to control_test_results.csv.

Arrival is inferred from motion alone, so a joint stalled against something
reads the same as one that arrived. The residual error in the CSV is what
tells the two apart.

Only the selected finger is ever commanded; the rest of the hand holds
whatever pose it was left in. Pass --neutral to move the hand to neutral
first.

Usage:
    uv run python scripts/control_testing.py
    uv run python scripts/control_testing.py CONFIG --finger index --waypoints 20
    uv run python scripts/control_testing.py --finger index --joints index_pip
    uv run python scripts/control_testing.py --trajectory sweep.csv --seed 7
"""
import argparse
import signal
from pathlib import Path

from orca_core.control.control_auxiliary import (
    DEFAULT_ARRIVAL_TOLERANCE_DEG,
    DEFAULT_DWELL_CHECKPOINTS_S,
    DEFAULT_IN_TOLERANCE_DEG,
    DEFAULT_MOTION_THRESHOLD_DEG,
    DEFAULT_POLL_HZ,
    DEFAULT_SETTLE_DWELL_S,
    DEFAULT_SETTLE_WINDOW_S,
    DEFAULT_START_GRACE_S,
    DEFAULT_START_TRAVEL_DEG,
    DEFAULT_WAYPOINT_TIMEOUT_S,
    RandomWaypoints,
    TrajectoryWaypoints,
    available_fingers,
    finger_joints,
    joint_target_range,
    read_trajectory_csv,
    resolve_joints,
    run_waypoint_test,
    summarize_run,
    write_results_csv,
    write_stats_csv,
)
from orca_core.utils.cli import (
    add_hand_arguments,
    connect_hand,
    create_hand_from_args,
    shutdown_hand,
)


DEFAULT_OUTPUT = "control_test_results.csv"

# Menu order only. The fingers actually offered come from the hand's config;
# anything this list doesn't name is appended in config order.
FINGER_MENU_ORDER = ("index", "middle", "ring", "pinky", "thumb")

RST = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
GREEN = "\033[92m"
YELLOW = "\033[93m"


def menu_fingers(hand) -> list:
    present = available_fingers(hand.config)
    ordered = [f for f in FINGER_MENU_ORDER if f in present]
    return ordered + [f for f in present if f not in ordered]


def prompt_finger(fingers: list) -> str:
    print(f"\n{BOLD}Which finger?{RST}")
    for number, finger in enumerate(fingers, start=1):
        print(f"  {number}. {finger}")
    while True:
        choice = input(f"Select 1-{len(fingers)}: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(fingers):
            return fingers[int(choice) - 1]
        print(f"Enter a number between 1 and {len(fingers)}.")


def prompt_trajectory_path() -> "str | None":
    answer = input("\nFollow a preset trajectory? [y/N]: ").strip().lower()
    if answer not in ("y", "yes"):
        return None
    while True:
        path = input("Path to trajectory CSV: ").strip()
        if path:
            return path
        print("Enter a path, or Ctrl-C to abort.")


def print_ranges(hand, finger: str, joints: list) -> None:
    held = [j for j in finger_joints(hand.config, finger) if j not in joints]
    print(f"\n{BOLD}{finger}{RST} target ranges (config ROM ∩ measured ROM, inset):")
    for joint in joints:
        lower, upper = joint_target_range(hand, joint)
        print(f"  {joint:<12} {lower:8.2f} to {upper:8.2f} deg")
    if held:
        print(f"  {DIM}holding: {', '.join(held)}{RST}")


def make_progress_printer():
    def on_event(event: dict) -> None:
        if event["event"] == "waypoint_started":
            targets = "  ".join(
                f"{j.split('_', 1)[1]}={v:.1f}" for j, v in event["targets"].items()
            )
            print(f"\n[{event['index']:>3}] -> {targets}")
            return

        record = event["record"]
        colour = GREEN if event["outcome"] == "settled" else YELLOW
        moved = record.time_to_move_s
        worst = max((abs(j.error_deg) for j in record.joints), default=0.0)
        print(
            f"      {colour}{event['outcome']:<8}{RST} "
            f"{event['duration_s']:5.2f} s   "
            f"moved off at {'never' if moved is None else f'{moved:.2f} s'}   "
            f"worst residual {worst:5.2f} deg"
        )
        for joint in record.joints:
            print(
                f"      {DIM}{joint.joint:<12} target {joint.target_deg:7.2f}  "
                f"settle {joint.settle_deg:7.2f}  final {joint.final_deg:7.2f}  "
                f"err {joint.error_deg:+6.2f}  "
                f"moved {joint.moved_deg:+7.2f}{RST}"
            )

    return on_event


def print_summary(records: list, output_path) -> None:
    if not records:
        print("\nNo waypoints completed.")
        return
    settled = [r for r in records if r.outcome == "settled"]
    durations = [r.duration_s for r in settled]
    residuals = [abs(j.error_deg) for r in records for j in r.joints]
    print(f"\n{BOLD}Summary{RST}")
    sources = sorted({r.angle_source for r in records})
    print(f"  waypoints        {len(records)}")
    print(f"  angles from      {', '.join(sources)}")
    print(f"  settled          {len(settled)}")
    print(f"  stalled          {sum(1 for r in records if r.outcome == 'stalled')}")
    print(f"  timed out        {sum(1 for r in records if r.outcome == 'timeout')}")
    in_tol = [r.time_in_tolerance_s for r in records if r.time_in_tolerance_s is not None]
    if in_tol:
        print(
            f"  in tolerance     median {sorted(in_tol)[len(in_tol) // 2]:.2f} s "
            f"({len(in_tol)}/{len(records)} waypoints)"
        )
    if durations:
        print(
            f"  settle time      mean {sum(durations) / len(durations):.2f} s   "
            f"min {min(durations):.2f} s   max {max(durations):.2f} s"
        )
    if residuals:
        print(
            f"  residual error   mean {sum(residuals) / len(residuals):.2f} deg   "
            f"max {max(residuals):.2f} deg"
        )
    print(f"  written to       {output_path}")


def _stats_path_for(output: str) -> str:
    path = Path(output)
    return str(path.with_name(f"{path.stem}_stats{path.suffix or '.csv'}"))


def print_stats(stats, path) -> None:
    print(f"\n{BOLD}Per-joint statistics{RST}")
    header = (
        f"  {'joint':<12} {'|err| med':>9} {'|err| p95':>9} {'err sd':>7} "
        f"{'hyst gap':>9} {'creep':>7} {'fit: target':>12} {'move':>7} {'R2':>6}"
    )
    print(header)
    for joint in stats.joints:
        def fmt(value, spec):
            return "-".rjust(int(spec.split('.')[0])) if value is None else format(value, spec)
        print(
            f"  {joint.joint:<12} {joint.abs_err_median:9.2f} {joint.abs_err_p95:9.2f} "
            f"{joint.err_sd:7.2f} {fmt(joint.hysteresis_gap, '9.2f')} "
            f"{joint.creep_mean:7.2f} {fmt(joint.fit_target_pct, '11.2f')}% "
            f"{fmt(joint.fit_move_pct, '6.2f')}% {fmt(joint.fit_r2, '6.2f')}"
        )
    print(f"  {DIM}hyst gap < 0 = falls short both ways (friction); "
          f"> 0 = runs past both ways (momentum){RST}")
    print(f"  {DIM}fit: error as % of target angle and of commanded move; "
          f"low R2 = mostly random scatter{RST}")
    print(f"  written to       {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Waypoint-following test for one finger, logged to CSV."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--finger", default=None,
        help="Finger to test (e.g. index); omit to be prompted.",
    )
    parser.add_argument(
        "--joints", nargs="+", default=None,
        help=(
            "Drive only these joints of the chosen finger (e.g. --joints "
            "index_pip). The rest of the finger holds its pose."
        ),
    )
    parser.add_argument(
        "--trajectory", default=None,
        help="Preset trajectory CSV to follow; omit to be prompted.",
    )
    parser.add_argument(
        "--random", dest="force_random", action="store_true",
        help="Use random targets without prompting for a trajectory.",
    )
    parser.add_argument(
        "--waypoints", type=int, default=10,
        help="Number of waypoints to command (default 10).",
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Seed for random targets, so two runs follow the same sequence.",
    )
    parser.add_argument(
        "--motion-threshold", type=float, default=DEFAULT_MOTION_THRESHOLD_DEG,
        help=(
            "Peak-to-peak joint travel over the settle window below which the "
            f"finger counts as stopped, in degrees (default {DEFAULT_MOTION_THRESHOLD_DEG})."
        ),
    )
    parser.add_argument(
        "--settle-window", type=float, default=DEFAULT_SETTLE_WINDOW_S,
        help=f"Settle window in seconds (default {DEFAULT_SETTLE_WINDOW_S}).",
    )
    parser.add_argument(
        "--ramp-accel", type=float, default=0.0,
        help=(
            "Acceleration in deg/s^2 shaping the ramp into a trapezoid that "
            "arrives at rest. 0 (default) holds the speed to the target, so "
            "the joint arrives still moving."
        ),
    )
    parser.add_argument(
        "--ramp-per-joint", action="store_true", default=False,
        help=(
            "Give each joint its own profile at the commanded speed instead "
            "of one shared timeline sized by the furthest-travelling joint."
        ),
    )
    parser.add_argument(
        "--in-tolerance", type=float, default=DEFAULT_IN_TOLERANCE_DEG,
        help=(
            "Band in degrees for time_in_tolerance_s, the condition-independent "
            "convergence metric. Keep it near steady-state capability "
            f"(default {DEFAULT_IN_TOLERANCE_DEG}); a band wider than the "
            "overshoot never times the recovery."
        ),
    )
    parser.add_argument(
        "--settle-dwell", type=float, default=DEFAULT_SETTLE_DWELL_S,
        help=(
            "Seconds to hold the target after motion stops before reading the "
            f"residual (default {DEFAULT_SETTLE_DWELL_S}). 0 disables."
        ),
    )
    parser.add_argument(
        "--dwell-checkpoints", nargs="+", type=float,
        default=list(DEFAULT_DWELL_CHECKPOINTS_S),
        help=(
            "Times after settle, in seconds, at which the angle is also "
            "recorded, giving the convergence curve from one run (max 3, "
            f"default {' '.join(str(t) for t in DEFAULT_DWELL_CHECKPOINTS_S)})."
        ),
    )
    parser.add_argument(
        "--ramp-steps", type=int, default=1,
        help=(
            "Split each move into this many interpolated setpoints instead of "
            "one step (default 1 = step)."
        ),
    )
    parser.add_argument(
        "--ramp-time", type=float, default=0.0,
        help="Seconds the ramp is spread over. Ignored when --ramp-steps is 1.",
    )
    parser.add_argument(
        "--ramp-speed", type=float, default=0.0,
        help=(
            "Commanded joint speed in deg/s. Overrides --ramp-steps and "
            "--ramp-time, deriving both from each move's size so every move "
            "runs at the same speed."
        ),
    )
    parser.add_argument(
        "--start-travel", type=float, default=DEFAULT_START_TRAVEL_DEG,
        help=(
            "Travel from the starting pose, in degrees, before stillness may "
            f"count as arrival (default {DEFAULT_START_TRAVEL_DEG})."
        ),
    )
    parser.add_argument(
        "--start-grace", type=float, default=DEFAULT_START_GRACE_S,
        help=(
            "Seconds to wait for that travel before settling anyway, for a "
            f"target the finger already holds (default {DEFAULT_START_GRACE_S})."
        ),
    )
    parser.add_argument(
        "--arrival-tolerance", type=float, default=DEFAULT_ARRIVAL_TOLERANCE_DEG,
        help=(
            "Residual in degrees above which a finger that stopped is recorded "
            f"as stalled rather than settled (default {DEFAULT_ARRIVAL_TOLERANCE_DEG})."
        ),
    )
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_WAYPOINT_TIMEOUT_S,
        help=f"Per-waypoint timeout in seconds (default {DEFAULT_WAYPOINT_TIMEOUT_S}).",
    )
    parser.add_argument(
        "--poll-hz", type=float, default=DEFAULT_POLL_HZ,
        help=f"Joint readback rate in Hz (default {DEFAULT_POLL_HZ}).",
    )
    parser.add_argument(
        "--output", default=DEFAULT_OUTPUT,
        help=f"Results CSV, appended to (default {DEFAULT_OUTPUT}).",
    )
    parser.add_argument(
        "--stats-output", default=None,
        help=(
            "Per-joint statistics CSV, appended to. Defaults to the results "
            "file with a '_stats' suffix."
        ),
    )
    parser.add_argument(
        "--neutral", dest="start_neutral", action="store_true", default=False,
        help=(
            "Move the whole hand to neutral before testing. Off by default: "
            "that move runs in POSITION mode, which has no current limit."
        ),
    )
    args = parser.parse_args()

    hand = create_hand_from_args(args)
    try:
        connect_hand(hand)

        fingers = menu_fingers(hand)
        finger = args.finger or prompt_finger(fingers)
        if finger not in fingers:
            print(f"Unknown finger {finger!r}; this hand has: {', '.join(fingers)}")
            return 2

        try:
            joints = resolve_joints(hand.config, finger, args.joints)
        except ValueError as exc:
            print(exc)
            return 2
        trajectory_path = args.trajectory
        if trajectory_path is None and not args.force_random:
            trajectory_path = prompt_trajectory_path()

        if trajectory_path is not None:
            try:
                waypoints = read_trajectory_csv(trajectory_path, joints)
            except (FileNotFoundError, ValueError) as exc:
                print(f"Could not read trajectory: {exc}")
                return 2
            source = TrajectoryWaypoints(waypoints)
            print(f"\nFollowing {len(waypoints)} waypoint(s) from {trajectory_path}")
        else:
            source = RandomWaypoints(hand, joints, seed=args.seed)
            seed_note = f" (seed {args.seed})" if args.seed is not None else ""
            print(f"\nSampling random targets{seed_note}")

        loop_joints = getattr(hand, "loop_joint_names", None) or []
        if not set(joints).issubset(loop_joints):
            print(
                f"{YELLOW}The joint loop does not cover every {finger} joint: "
                f"angles will be motor-derived estimates, not encoder "
                f"measurements.{RST}"
            )

        hand.init_joints(move_to_neutral=args.start_neutral)
        print_ranges(hand, finger, joints)

        # Ctrl-C ends the run through should_stop rather than unwinding out of
        # it, so the waypoints already completed still reach the CSV.
        stopping = False

        def request_stop(_signum, _frame):
            nonlocal stopping
            if stopping:
                raise KeyboardInterrupt
            stopping = True
            print("\nStopping after this waypoint (Ctrl-C again to abort).")

        signal.signal(signal.SIGINT, request_stop)
        print("\nCtrl-C to stop early.")

        records = run_waypoint_test(
            hand,
            finger,
            source,
            args.waypoints,
            joints=joints,
            motion_threshold_deg=args.motion_threshold,
            settle_window_s=args.settle_window,
            timeout_s=args.timeout,
            poll_hz=args.poll_hz,
            dwell_s=args.settle_dwell,
            dwell_checkpoints_s=args.dwell_checkpoints,
            ramp_steps=args.ramp_steps,
            ramp_time_s=args.ramp_time,
            ramp_speed_deg_s=args.ramp_speed,
            start_travel_deg=args.start_travel,
            start_grace_s=args.start_grace,
            arrival_tolerance_deg=args.arrival_tolerance,
            in_tolerance_deg=args.in_tolerance,
            progress_callback=make_progress_printer(),
            should_stop=lambda: stopping,
        )

        run_id = f"{finger}-{int(records[0].started_at)}" if records else finger
        output_path = write_results_csv(
            args.output,
            records,
            run_id=run_id,
            motion_threshold_deg=args.motion_threshold,
            settle_window_s=args.settle_window,
            arrival_tolerance_deg=args.arrival_tolerance,
            in_tolerance_deg=args.in_tolerance,
            settle_dwell_s=args.settle_dwell,
            ramp_speed_deg_s=args.ramp_speed,
            ramp_steps=args.ramp_steps,
            ramp_time_s=args.ramp_time,
            ramp_accel_deg_s2=args.ramp_accel,
            ramp_per_joint=args.ramp_per_joint,
        )
        stats = summarize_run(records, run_id=run_id)
        stats_path = args.stats_output or _stats_path_for(args.output)
        if stats is not None:
            stats_path = write_stats_csv(
                stats_path,
                stats,
                motion_threshold_deg=args.motion_threshold,
                settle_window_s=args.settle_window,
                arrival_tolerance_deg=args.arrival_tolerance,
                in_tolerance_deg=args.in_tolerance,
                settle_dwell_s=args.settle_dwell,
                ramp_speed_deg_s=args.ramp_speed,
                ramp_steps=args.ramp_steps,
                ramp_time_s=args.ramp_time,
                ramp_accel_deg_s2=args.ramp_accel,
                ramp_per_joint=args.ramp_per_joint,
            )
        print_summary(records, output_path)
        if stats is not None:
            print_stats(stats, stats_path)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
