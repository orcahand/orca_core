from orca_core import OrcaHand
import time
import argparse

# Max operating temp (°C) — conservative across XC330 (70°C) and XC430 (72°C)
MAX_TEMP = 70
TEMP_CHECK_INTERVAL = 2.0


RST = "\033[0m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"


def temp_color(pct):
    if pct >= 90:
        return RED
    elif pct >= 70:
        return YELLOW
    return GREEN


def print_temp_table(hand, temps):
    """Print a compact color-coded temperature table grouped by finger."""
    motor_to_joint = hand.motor_to_joint_dict

    # Group motors by finger
    fingers = ['thumb', 'index', 'middle', 'ring', 'pinky', 'wrist']
    grouped = {f: [] for f in fingers}
    for mid, t in temps.items():
        joint = motor_to_joint.get(mid, f"motor_{mid}")
        finger = joint.split('_')[0]
        if finger in grouped:
            grouped[finger].append((joint, mid, t))

    # Clear screen and move cursor to top
    print("\033[2J\033[H", end="")

    print(f"{BOLD}  ORCA Hand Temperature Monitor{RST}")
    print(f"  {DIM}Max operating temp: {MAX_TEMP}°C{RST}\n")

    # Header
    print(f"  {BOLD}{'Joint':<14} {'Motor':>5} {'Temp':>6} {'%Max':>6}  {'':>10}{RST}")
    print(f"  {'─' * 48}")

    for finger in fingers:
        joints = grouped[finger]
        if not joints:
            continue
        for joint, mid, t in joints:
            pct = t / MAX_TEMP * 100
            c = temp_color(pct)
            bar_len = int(pct / 100 * 10)
            bar = f"{c}{'█' * bar_len}{DIM}{'░' * (10 - bar_len)}{RST}"
            print(f"  {joint:<14} {mid:>5} {c}{t:>4.0f}°C {pct:>5.0f}%{RST}  {bar}")

    print(f"  {'─' * 48}")
    max_t = max(temps.values())
    max_pct = max_t / MAX_TEMP * 100
    c = temp_color(max_pct)
    print(f"  {'Peak':<14} {'':>5} {c}{max_t:>4.0f}°C {max_pct:>5.0f}%{RST}\n")


def main():
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()
    hand = OrcaHand(args.model_path)

    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    last_temp_check = 0
    while True:
        now = time.monotonic()
        if now - last_temp_check >= TEMP_CHECK_INTERVAL:
            last_temp_check = now
            temps = hand.get_motor_temp(as_dict=True)
            print_temp_table(hand, temps)

        # OPEN position
        joint_dict_open = {
            # Abductions
            "index_abd": -30,
            "middle_abd": 0,
            "ring_abd": 10,
            "pinky_abd": 25,
            "thumb_abd": -20,

            # MCPs
            "index_mcp": -40,
            "middle_mcp": -40,
            "ring_mcp": -40,
            "pinky_mcp": -40,
            "thumb_mcp": 45,

            # PIPs/DIPs
            "index_pip": -10,
            "middle_pip": -10,
            "ring_pip": -10,
            "pinky_pip": -10,
            "thumb_dip": 90,

            "thumb_cmc": 40,
            # Wrist
            "wrist": -25,
        }

        hand.set_joint_pos(joint_dict_open, num_steps=25, step_size=0.001)

        time.sleep(2)

        # CLOSE position
        joint_dict_close = {
            # Abductions
            "index_abd": 15,
            "middle_abd": 15,
            "ring_abd": -15,
            "pinky_abd": -15,
            "thumb_abd": 55,

            # MCPs
            "index_mcp": 45,
            "middle_mcp": 45,
            "ring_mcp": 45,
            "pinky_mcp": 45,
            "thumb_mcp": -40,

            # PIPs/DIPs
            "index_pip": 90,
            "middle_pip": 90,
            "ring_pip": 90,
            "pinky_pip": 90,
            "thumb_dip": -30,

            "thumb_cmc": -50,

            # Wrist
            "wrist": 10,
        }

        hand.set_joint_pos(joint_dict_close, num_steps=25, step_size=0.001)
        time.sleep(2)


    hand.disable_torque()

    hand.disconnect()

if __name__ == "__main__":  # Added main execution block
    main()