import time
import numpy as np
import argparse
from orca_core import OrcaHand

JOINT_ORDER = [
    "thumb_cmc", "thumb_abd", "thumb_mcp",
    "index_mcp", "index_pip",
    "middle_mcp",
    "ring_mcp",
    "pinky_mcp",
    "wrist",
]

CYCLES = 3
CYCLE_TIME = 1.0
STEP_TIME = 0.02
WRIST_SPEED_FACTOR = 2.0
WRIST_FEWER_CYCLES = 1


def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))


def smooth_move(hand, joint, start, end, duration, step_time):
    n_steps = max(int(duration / step_time), 1)
    start_time = time.time()
    for s in range(n_steps + 1):
        alpha = ease_in_out(s / n_steps)
        pos = start + alpha * (end - start)
        hand.set_joint_pos({joint: pos})
        target = start_time + s * step_time
        now = time.time()
        if now < target:
            time.sleep(target - now)


def actuate_joint(hand, joint, rom_min, rom_max, neutral, cycles, cycle_time, step_time):
    half_time = cycle_time / 2

    if abs(neutral - rom_min) > 0.5:
        smooth_move(hand, joint, neutral, rom_min, half_time, step_time)

    for _ in range(cycles):
        smooth_move(hand, joint, rom_min, rom_max, half_time, step_time)
        smooth_move(hand, joint, rom_max, rom_min, half_time, step_time)

    if abs(rom_min - neutral) > 0.5:
        smooth_move(hand, joint, rom_min, neutral, half_time, step_time)


def main():
    parser = argparse.ArgumentParser(description="Demo all DOFs of the ORCA Hand Lite.")
    parser.add_argument("model_path", type=str, nargs="?",
                        default="orca_core/models/orcahand-lite",
                        help="Path to the orcahand-lite model folder")
    parser.add_argument("--cycles", type=int, default=CYCLES,
                        help=f"Number of flex/extend cycles per joint (default: {CYCLES})")
    parser.add_argument("--cycle_time", type=float, default=CYCLE_TIME,
                        help=f"Seconds per flex/extend cycle (default: {CYCLE_TIME})")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    hand.init_joints()
    time.sleep(1)

    joint_roms = hand.joint_roms_dict
    joint_props = hand.joint_properties or {}
    available = list(hand.joint_ids)

    # Only demo actuated joints (skip fixed/passive)
    demo_joints = []
    for j in JOINT_ORDER:
        if j not in available:
            continue
        props = joint_props.get(j, {})
        if props.get('actuation') in ('fixed', 'passive'):
            continue
        rom = joint_roms.get(j, [0, 0])
        if rom[0] == rom[1]:
            continue
        demo_joints.append(j)

    print(f"Starting DOF demo: {len(demo_joints)} joints, {args.cycles} cycles each\n")

    try:
        neutral_pos = hand.get_joint_pos(as_list=False)

        for joint in demo_joints:
            rom_min, rom_max = joint_roms[joint]
            neutral = neutral_pos.get(joint, (rom_min + rom_max) / 2)
            is_wrist = joint == "wrist"
            is_thumb_top = joint in ("thumb_cmc", "thumb_abd")
            cycle_time = args.cycle_time * WRIST_SPEED_FACTOR if is_wrist else args.cycle_time
            if is_wrist or is_thumb_top:
                cycles = max(args.cycles - WRIST_FEWER_CYCLES, 1)
            else:
                cycles = args.cycles
            print(f"  {joint:16s}  ROM: [{rom_min:.0f}, {rom_max:.0f}]")
            actuate_joint(hand, joint, rom_min, rom_max, neutral, cycles, cycle_time, STEP_TIME)
            if joint == "thumb_abd":
                smooth_move(hand, joint, neutral, rom_max * 0.4, cycle_time / 2, STEP_TIME)
            time.sleep(0.2)

        print("\nDOF demo complete.")

    except KeyboardInterrupt:
        print("\nDemo interrupted.")

    finally:
        hand.disable_torque()
        print("Torque disabled.")


if __name__ == "__main__":
    main()
