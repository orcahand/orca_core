import time
import yaml
import numpy as np
import argparse
from orca_core import OrcaHand
import os
import glob as globmod


def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))


def interpolate(hand, start, end, duration, step_time):
    """Smoothly move from start to end over duration seconds."""
    n_steps = max(int(duration / step_time), 1)
    start_time = time.time()

    for step in range(n_steps + 1):
        t = step / n_steps
        alpha = ease_in_out(t)
        pose = [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]
        hand.set_joint_pos(pose)

        target_time = start_time + step * step_time
        now = time.time()
        if now < target_time:
            time.sleep(target_time - now)


def play_continuous(hand, frames, step_time):
    """Play back continuous frames at the recorded rate."""
    start_time = time.time()
    for i, pose in enumerate(frames):
        hand.set_joint_pos(pose)
        target_time = start_time + i * step_time
        now = time.time()
        if now < target_time:
            time.sleep(target_time - now)


def get_motion_duration(motion, interp_time, continuous_step_time):
    """Get the cycle duration of a single motion within a group."""
    if motion["type"] == "waypoint":
        return len(motion["angles"]) * interp_time
    else:
        return len(motion["angles"]) * continuous_step_time


def get_motion_pose_at_time(motion, local_time, interp_time, continuous_step_time):
    """Get the full joint pose for a motion at a given local time (wraps cyclically)."""
    angles = motion["angles"]

    if motion["type"] == "waypoint":
        n_wp = len(angles)
        cycle = n_wp * interp_time
        t = local_time % cycle
        wp_idx = int(t / interp_time)
        wp_frac = (t % interp_time) / interp_time
        alpha = ease_in_out(wp_frac)
        start = angles[wp_idx % n_wp]
        end = angles[(wp_idx + 1) % n_wp]
        return [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]

    else:
        n_frames = len(angles)
        cycle = n_frames * continuous_step_time
        t = local_time % cycle
        frame_idx = min(int(t / continuous_step_time), n_frames - 1)
        return list(angles[frame_idx])


def get_group_pose_at_time(group, global_time, group_cycle, interp_time, continuous_step_time, base_pose):
    """Compute merged pose for all motions in a group at a given time."""
    pose = list(base_pose)

    for motion in group["motions"]:
        phase = motion.get("phase_shift", 0.0)
        local_time = global_time + phase * group_cycle
        motion_pose = get_motion_pose_at_time(motion, local_time, interp_time, continuous_step_time)

        for j in motion["joints"]:
            pose[j] = motion_pose[j]

    return pose


def get_motion_forward_duration(motion, interp_time, continuous_step_time):
    """Duration of a one-way pass through a motion (no cycling back)."""
    if motion["type"] == "waypoint":
        return max(len(motion["angles"]) - 1, 1) * interp_time
    else:
        return (len(motion["angles"]) - 1) * continuous_step_time


def play_group(hand, group, interp_time, step_time, continuous_step_time, base_pose=None, auto=False):
    """Play a group segment.

    In auto mode, motions cycle. In manual mode, each motion plays forward
    once and holds at its final position.
    """
    if base_pose is None:
        base_pose = [float(a) for a in hand.get_joint_pos(as_list=True)]

    # Prepend base pose as implicit start for single-waypoint motions.
    motions = []
    for m in group["motions"]:
        m_copy = dict(m)
        if m_copy["type"] == "waypoint" and len(m_copy["angles"]) == 1:
            m_copy["angles"] = [list(base_pose)] + list(m_copy["angles"])
        motions.append(m_copy)

    # Compute each motion's forward duration (one-way, no cycle back) and delay
    if auto:
        fwd_durations = [get_motion_duration(m, interp_time, continuous_step_time) for m in motions]
    else:
        fwd_durations = [get_motion_forward_duration(m, interp_time, continuous_step_time) for m in motions]
    max_fwd_dur = max(fwd_durations)
    delays = [m.get("phase_shift", 0.0) * max_fwd_dur for m in motions]
    group_total = max(d + dur for d, dur in zip(delays, fwd_durations))

    # In auto mode, cycle = forward phase + smooth return to base
    # Forward: each motion plays one-way (base→target) with phase delays
    # Return: all joints interpolate back to base over interp_time
    forward_durations = [get_motion_forward_duration(m, interp_time, continuous_step_time) for m in motions]
    max_fwd = max(forward_durations)
    fwd_delays = [m.get("phase_shift", 0.0) * max_fwd for m in motions]
    forward_total = max(d + dur for d, dur in zip(fwd_delays, forward_durations))

    if auto:
        # Compute the end pose (all motions at their final position)
        end_pose = list(base_pose)
        for i, motion in enumerate(motions):
            motion_pose = get_motion_pose_at_time(motion, forward_durations[i], interp_time, continuous_step_time)
            for j in motion["joints"]:
                end_pose[j] = motion_pose[j]
        cycle_total = forward_total + interp_time
    else:
        cycle_total = group_total

    n_steps = max(int(cycle_total / step_time), 1)
    start_time = time.time()
    step = 0

    while True:
        raw_time = step * step_time
        if not auto and step > n_steps:
            break
        if auto:
            global_time = raw_time % cycle_total
        else:
            global_time = raw_time

        pose = list(base_pose)

        if auto and global_time > forward_total:
            # Return phase: smoothly interpolate from end_pose back to base_pose
            return_elapsed = global_time - forward_total
            alpha = ease_in_out(min(return_elapsed / interp_time, 1.0))
            pose = [(1 - alpha) * e + alpha * b for e, b in zip(end_pose, base_pose)]
        else:
            # Forward phase: play motions with phase delays
            for i, motion in enumerate(motions):
                elapsed = global_time - fwd_delays[i]
                if elapsed < 0:
                    continue
                if not auto:
                    elapsed = min(elapsed, fwd_durations[i])
                else:
                    elapsed = min(elapsed, forward_durations[i])
                motion_pose = get_motion_pose_at_time(motion, elapsed, interp_time, continuous_step_time)
                for j in motion["joints"]:
                    pose[j] = motion_pose[j]

        hand.set_joint_pos(pose)

        target_time = start_time + step * step_time
        now = time.time()
        if now < target_time:
            time.sleep(target_time - now)

        step += 1


def get_segment_start(seg, interp_time, continuous_step_time):
    """Get the starting pose of a segment."""
    if seg["type"] == "group":
        # Need a base pose to compute group start; use the first motion's start as base
        n_joints = len(seg["motions"][0]["angles"][0])
        base = [0.0] * n_joints
        # Compute pose at t=0
        durations = [get_motion_duration(m, interp_time, continuous_step_time) for m in seg["motions"]]
        group_cycle = max(durations)
        return get_group_pose_at_time(seg, 0.0, group_cycle, interp_time, continuous_step_time, base)
    elif seg["type"] == "waypoint":
        return seg["angles"]
    else:
        return seg["angles"][0]


def get_segment_end(seg, interp_time, continuous_step_time):
    """Get the ending pose of a segment."""
    if seg["type"] == "group":
        n_joints = len(seg["motions"][0]["angles"][0])
        base = [0.0] * n_joints
        durations = [get_motion_duration(m, interp_time, continuous_step_time) for m in seg["motions"]]
        group_cycle = max(durations)
        return get_group_pose_at_time(seg, group_cycle, group_cycle, interp_time, continuous_step_time, base)
    elif seg["type"] == "waypoint":
        return seg["angles"]
    else:
        return seg["angles"][-1]


def main():
    parser = argparse.ArgumentParser(description='Replay recorded hand movements (mixed format).')
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the orcahand model folder")
    parser.add_argument('--replay_file', type=str, default=None,
                        help="Path to the replay file. If omitted, uses the latest .yaml in replay_sequences/.")
    parser.add_argument('--step_time', type=float, default=0.02,
                        help='Timestep for interpolation (default: 0.02)')
    parser.add_argument('--interp_time', type=float, default=3.0,
                        help='Seconds per transition between segments (default: 3.0)')
    parser.add_argument('--auto', action='store_true',
                        help='Automatically play without waiting for Enter')
    args = parser.parse_args()

    # Resolve file path
    project_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    default_replay_dir = os.path.join(project_root, 'replay_sequences')

    if args.replay_file:
        user_input = args.replay_file.strip()
        if os.path.isabs(user_input):
            full_filepath = user_input
        elif os.sep in user_input:
            full_filepath = os.path.join(project_root, user_input)
        else:
            full_filepath = os.path.join(default_replay_dir, user_input)
        full_filepath = os.path.abspath(full_filepath)
    else:
        yaml_files = globmod.glob(os.path.join(default_replay_dir, '*.yaml'))
        if not yaml_files:
            print(f"No .yaml files found in {default_replay_dir}")
            return
        full_filepath = max(yaml_files, key=os.path.getmtime)
        print(f"No replay file specified, using latest: {os.path.basename(full_filepath)}")

    try:
        with open(full_filepath, "r") as file:
            replay_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"File not found: {full_filepath}")
        return

    metadata = replay_data.get("metadata", {})
    segments = replay_data.get("segments")

    # Support legacy waypoint-only files
    if segments is None:
        waypoints = replay_data.get("waypoints")
        if waypoints:
            segments = [{"type": "waypoint", "angles": w} for w in waypoints]
        else:
            angles = replay_data.get("angles")
            if angles:
                segments = [{"type": "continuous", "angles": angles}]
            else:
                print("No segments, waypoints, or angles found in the file.")
                return

    if not segments:
        print("No segments found in the file.")
        return

    # Hardcoded thumb offset
    joint_ids = metadata.get("joint_ids", [])
    thumb_offsets = {"thumb_cmc": 5.0, "thumb_abd": 5.0}
    for joint, offset in thumb_offsets.items():
        if joint in joint_ids:
            idx = joint_ids.index(joint)
            for seg in segments:
                if seg.get("type") in ("waypoint", "continuous"):
                    a = seg["angles"]
                    if isinstance(a[0], list):
                        for frame in a:
                            frame[idx] += offset
                    else:
                        a[idx] += offset
                elif seg.get("type") == "group":
                    for motion in seg.get("motions", []):
                        a = motion["angles"]
                        if isinstance(a[0], list):
                            for frame in a:
                                frame[idx] += offset
                        else:
                            a[idx] += offset
                elif seg.get("type") == "pose":
                    seg["angles"][idx] += offset

    sampling_freq = metadata.get("sampling_frequency_hz", 50.0)
    continuous_step_time = 1.0 / sampling_freq

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.enable_torque()
    print("Torque enabled. Starting replay...\n")

    def prompt(msg):
        if not args.auto:
            input(msg)

    # Check if this is a pose-based recording
    poses = {seg["name"]: seg["angles"] for seg in segments if seg.get("type") == "pose"}

    if poses:
        # Pose selection mode
        neutral = poses.get("neutral")
        pose_names = [n for n in poses if n != "neutral"]

        if neutral:
            print("Moving to neutral...")
            interpolate(hand, hand.get_joint_pos(), neutral, args.interp_time, args.step_time)

        print(f"\nAvailable poses: {', '.join(pose_names)}")
        print("Type a pose name to execute, or 'q' to quit.\n")

        current = neutral if neutral else list(hand.get_joint_pos())

        try:
            while True:
                choice = input("> ").strip().lower()
                if choice == 'q':
                    break
                if choice not in poses:
                    print(f"Unknown pose. Choose from: {', '.join(pose_names)}")
                    continue

                target = poses[choice]
                print(f"  -> {choice}")
                interpolate(hand, current, target, args.interp_time, args.step_time)
                current = list(target)

                if neutral:
                    input("  Press Enter to return to neutral...")
                    print(f"  -> neutral")
                    interpolate(hand, current, neutral, args.interp_time, args.step_time)
                    current = list(neutral)
        except KeyboardInterrupt:
            print("\nReplay interrupted.")
        finally:
            hand.disable_torque()
            print("Torque disabled.")
        return

    try:
        # Move to the start of the first segment
        first_start = get_segment_start(segments[0], args.interp_time, continuous_step_time)
        print("Moving to start position...")
        prompt("Press Enter to continue (Ctrl+C to stop)...")
        interpolate(hand, hand.get_joint_pos(), first_start, args.interp_time, args.step_time)

        while True:
            for i, seg in enumerate(segments):
                seg_num = i + 1

                # Transition from previous segment end (skip for groups — they handle it internally)
                if i > 0 and seg["type"] != "group":
                    prev_end = get_segment_end(segments[i - 1], args.interp_time, continuous_step_time)
                    seg_start = get_segment_start(seg, args.interp_time, continuous_step_time)
                    prompt("Press Enter to continue (Ctrl+C to stop)...")
                    interpolate(hand, prev_end, seg_start, args.interp_time, args.step_time)

                if seg["type"] == "waypoint":
                    print(f"Segment {seg_num}/{len(segments)}: waypoint")
                    if i == 0:
                        interpolate(hand, hand.get_joint_pos(), seg["angles"], args.interp_time, args.step_time)

                elif seg["type"] == "continuous":
                    frames = seg["angles"]
                    print(f"Segment {seg_num}/{len(segments)}: continuous ({len(frames)} frames)")
                    prompt("Press Enter to play continuous (Ctrl+C to stop)...")
                    play_continuous(hand, frames, continuous_step_time)

                elif seg["type"] == "group":
                    n_motions = len(seg["motions"])
                    print(f"Segment {seg_num}/{len(segments)}: group ({n_motions} motions)")
                    prompt("Press Enter to play group (Ctrl+C to stop)...")
                    # Use previous segment's end as the base pose for the group
                    group_base = None
                    if i > 0:
                        group_base = get_segment_end(segments[i - 1], args.interp_time, continuous_step_time)
                    play_group(hand, seg, args.interp_time, args.step_time, continuous_step_time,
                               base_pose=group_base, auto=args.auto)
                    # In manual mode, return all joints to base simultaneously
                    if not args.auto and group_base is not None:
                        input("Press Enter to return to base (Ctrl+C to stop)...")
                        interpolate(hand, hand.get_joint_pos(), group_base, args.interp_time, args.step_time)

            if not args.auto:
                # Manual mode: smooth transition back to start and wait
                first_start = get_segment_start(segments[0], args.interp_time, continuous_step_time)
                print(f"\nLooping back to start...")
                input("Press Enter to continue (Ctrl+C to stop)...")
                interpolate(hand, hand.get_joint_pos(), first_start, args.interp_time, args.step_time)

    except KeyboardInterrupt:
        print("\nReplay interrupted.")

    finally:
        hand.disable_torque()
        print("Torque disabled.")


if __name__ == "__main__":
    main()
