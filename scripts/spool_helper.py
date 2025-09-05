import argparse
import time
from orca_core import OrcaHand

try:
    import keyboard  # pip install keyboard
except ImportError:
    print("Please install the 'keyboard' module: pip install keyboard")
    exit(1)


def main():
    parser = argparse.ArgumentParser(description="Control a motor interactively with left/right keys.")
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    parser.add_argument('--motor', type=int, default=1, help='Motor ID to control (default: 1)')
    parser.add_argument('--step', type=float, default=0.05, help='Step size for each key press (default: 0.05)')
    parser.add_argument('--interval', type=float, default=0.02, help='Polling interval in seconds (default: 0.02)')
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)
    hand.enable_torque()

    step = args.step
    interval = args.interval

    # Discover available motor IDs and set initial target
    try:
        pos_dict = hand.get_motor_pos(as_dict=True)
        available_ids = sorted(pos_dict.keys())
    except Exception:
        available_ids = [args.motor]
        pos_dict = {args.motor: 0.0}

    if not available_ids:
        print("No motors available.")
        return

    if args.motor in available_ids:
        current_index = available_ids.index(args.motor)
    else:
        current_index = 0

    motor_id = available_ids[current_index]

    # Get initial position for selected motor
    try:
        current_pos = pos_dict.get(motor_id, hand.get_motor_pos(as_dict=True)[motor_id])
    except Exception:
        print("Could not read initial motor position. Setting to 0.")
        current_pos = 0.0

    print("Use RIGHT arrow to turn right, LEFT arrow to turn left. UP/DOWN to switch motor. ESC to quit.")
    print(f"Step size: {step}")
    print(f"Selected motor: {motor_id}")

    # Debounce for motor switching
    switch_cooldown = 0.2
    last_switch_time = 0.0

    try:
        while True:
            now = time.time()
            moved = False

            # Handle motor switching with debounce
            if keyboard.is_pressed('up') and (now - last_switch_time) > switch_cooldown:
                current_index = (current_index + 1) % len(available_ids)
                motor_id = available_ids[current_index]
                try:
                    current_pos = hand.get_motor_pos(as_dict=True)[motor_id]
                except Exception:
                    pass
                print(f"\nSwitched to motor {motor_id}")
                last_switch_time = now

            if keyboard.is_pressed('down') and (now - last_switch_time) > switch_cooldown:
                current_index = (current_index - 1) % len(available_ids)
                motor_id = available_ids[current_index]
                try:
                    current_pos = hand.get_motor_pos(as_dict=True)[motor_id]
                except Exception:
                    pass
                print(f"\nSwitched to motor {motor_id}")
                last_switch_time = now

            # Handle position adjustments
            if keyboard.is_pressed('right'):
                current_pos += step
                moved = True
            if keyboard.is_pressed('left'):
                current_pos -= step
                moved = True

            if moved:
                hand._set_motor_pos({motor_id: current_pos})
                print(f"\rMotor {motor_id} position: {current_pos:.4f}   ", end='', flush=True)

            if keyboard.is_pressed('esc'):
                print("\nExiting.")
                break

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")

if __name__ == "__main__":
    main()