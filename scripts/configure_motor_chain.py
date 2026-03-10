import sys
import time
import logging
import argparse
import os
import subprocess

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.hardware.feetech_client import FeetechClient
from orca_core.utils import read_yaml, get_model_path, auto_detect_port, get_and_choose_port

RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[94m'
PURPLE = '\033[95m'
YELLOW = '\033[93m'
ORANGE = '\033[38;5;208m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
RESET = '\033[0m'

def validate_or_detect_port(port: str, config_path: str) -> str:
    """Check if port exists, auto-detect or prompt if not."""
    if os.path.exists(port):
        return port

    print(f"{YELLOW}⚠ Port {port} not found.{RESET}")

    detected = auto_detect_port()
    if detected and os.path.exists(detected):
        print(f"{GREEN}✓ Using auto-detected port: {detected}{RESET}")
        return detected

    print("Please select a port from available devices:")
    chosen = get_and_choose_port()
    if chosen:
        print(f"{GREEN}✓ Using selected port: {chosen}{RESET}")
        return chosen

    print(f"{RED}❌ No valid port found. Check your USB connection.{RESET}")
    sys.exit(1)


def wait_for_port_removed(port: str, timeout: float = 30):
    """Wait until the serial port disappears (USB unplugged)."""
    print(f"\n{YELLOW}⚠  REMOVE POWER:{RESET} Unplug the USB cable from your computer.")
    print(f"   Waiting for {BOLD}{port}{RESET} to disappear...")
    start = time.time()
    while os.path.exists(port):
        if time.time() - start > timeout:
            print(f"{RED}❌ Timeout: port {port} still exists. Unplug the USB cable.{RESET}")
            start = time.time()
        time.sleep(0.3)
    print(f"{GREEN}✓ USB disconnected.{RESET}")


def wait_for_port_reconnected(port: str, timeout: float = 60):
    """Wait until the serial port reappears (USB plugged back in)."""
    print(f"\n{YELLOW}▶  RESTORE POWER:{RESET} Plug the USB cable back in.")
    print(f"   Waiting for {BOLD}{port}{RESET} to reappear...")
    start = time.time()
    while not os.path.exists(port):
        if time.time() - start > timeout:
            print(f"{RED}❌ Timeout: port {port} not found. Plug in the USB cable.{RESET}")
            start = time.time()
        time.sleep(0.3)
    time.sleep(0.5)
    print(f"{GREEN}✓ USB reconnected on {port}.{RESET}")


def feetech_safe_connect_prompt(port: str, connection_msg: str):
    """Guide user through safe power-off motor connection for Feetech servos."""
    wait_for_port_removed(port)
    print(f"\n{ORANGE}🔌 {connection_msg}{RESET}")
    input(f"\n   Press {BOLD}Enter{RESET} when the motor is connected correctly...")
    wait_for_port_reconnected(port)


MOTOR_TYPE_DEFAULTS = {
    'dynamixel': {'default_id': 1, 'default_baud': 57600},
    'feetech':   {'default_id': 1, 'default_baud': 1000000},
}

def create_config_client(motor_type, motor_ids, port, baudrate):
    if motor_type == 'feetech':
        return FeetechClient(motor_ids, port=port, baudrate=baudrate)
    return DynamixelClient(motor_ids, port=port, baudrate=baudrate)

def play_success_beep():
    try:
        subprocess.run(['paplay', '/usr/share/sounds/freedesktop/stereo/complete.oga'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
    except:
        try:
            subprocess.run(['beep', '-f', '800', '-l', '100'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
        except:
            print('\a', end='', flush=True)

def motor_color(model_name):
    if 'XC330' in model_name:
        return BLUE
    if 'XC430' in model_name:
        return PURPLE
    return ''

def scan_for_default_motor(expected_type: str, port: str, motor_type: str, default_id: int, default_baud: int) -> bool:
    try:
        client = create_config_client(motor_type, [], port, default_baud)
        motors = client.scan_for_motors(port=port, id_range=(default_id, default_id), baud_rates=[default_baud])
        if not motors:
            return False
        motor = motors[0]
        if motor_type == 'feetech':
            print(f"{GREEN}✓ Found default Feetech motor (ID {motor['id']}){RESET}")
            return True
        expected_color = motor_color(expected_type)
        actual_color = motor_color(motor['model_name'])
        if expected_type in motor['model_name']:
            print(f"{GREEN}✓ Found default {actual_color}{motor['model_name']}{GREEN} motor{RESET}")
            return True
        else:
            print(f"{RED}❌ Wrong motor type: {actual_color}{motor['model_name']}{RESET} (expected {expected_color}{expected_type}{RESET}){RESET}")
            return False
    except Exception as e:
        print(f"{RED}❌ Scan error: {e}{RESET}")
        return False

def configure_default_motor(target_id: int, port: str, target_baud: int, motor_type: str, default_id: int, default_baud: int) -> bool:
    try:
        client = create_config_client(motor_type, [default_id], port, default_baud)
        client.connect()
        if not client.change_motor_id(default_id, target_id):
            print(f"{RED}❌ Failed to change ID to {target_id}{RESET}")
            client.disconnect()
            return False
        client.disconnect()
        time.sleep(0.5)

        if target_baud != default_baud:
            client = create_config_client(motor_type, [target_id], port, default_baud)
            client.connect()
            if not client.change_motor_baudrate(target_id, target_baud):
                print(f"{RED}❌ Failed to change baud rate to {target_baud}{RESET}")
                client.disconnect()
                return False
            client.disconnect()

        print(f"{GREEN}✓ Successfully configured the new motor to ID={target_id}, baudrate={target_baud}{RESET}")
        play_success_beep()
        return True
    except Exception as e:
        print(f"{RED}❌ Error configuring motor: {e}{RESET}")
        return False

def scan_already_configured_motors(port: str, target_baud: int, total_motors: int, motor_type: str, default_id: int, default_baud: int, xc430_id: int = None) -> list:
    print(f"\n🔍 Pre-Scanning for {YELLOW}already configured{RESET} motors...")
    try:
        client = create_config_client(motor_type, [], port, target_baud)
        motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])
        # When target_baud == default_baud (Feetech), a motor at default_id is
        # indistinguishable from a factory-default motor — skip it in the pre-scan.
        if target_baud == default_baud:
            motors = [m for m in motors if m['id'] != default_id]
        if not motors:
            print("   No pre-configured motors detected")
            return []
        motor_by_id = {m['id']: m for m in motors}
        valid_sequence, expected_id = [], total_motors

        if motor_type == 'feetech':
            for mid in sorted(motor_by_id.keys(), reverse=True):
                if mid == expected_id:
                    valid_sequence.append(mid)
                    expected_id -= 1
                else:
                    break
        else:
            min_xc330_id = 2 if xc430_id is not None else 1
            for mid in sorted(motor_by_id.keys(), reverse=True):
                if mid in range(min_xc330_id, total_motors + 1) and mid == expected_id:
                    if 'XC330' in motor_by_id[mid]['model_name']:
                        valid_sequence.append(mid)
                        expected_id -= 1
                    else:
                        break
                elif mid not in range(min_xc330_id, total_motors + 1):
                    break
            if xc430_id is not None and xc430_id in motor_by_id and expected_id == 1:
                if 'XC430' in motor_by_id[xc430_id]['model_name']:
                    valid_sequence.append(xc430_id)

        invalid_motors = [mid for mid in motor_by_id.keys() if mid not in valid_sequence]

        if valid_sequence:
            print(f"{GREEN}\nFound {len(valid_sequence)} valid, pre-configured motors with correct ID order:{RESET}")
            for mid in sorted(valid_sequence, reverse=True):
                motor = motor_by_id[mid]
                color = motor_color(motor['model_name'])
                print(f"   • ID {mid:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps{RESET}")
        if invalid_motors:
            print(f"{RED}Found {len(invalid_motors)} motors with invalid configuration:{RESET}")
            for mid in sorted(invalid_motors, reverse=True):
                motor = motor_by_id[mid]
                color = motor_color(motor['model_name'])
                print(f"   • ID {mid:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps{RESET}")
            if motor_type == 'feetech':
                print(f"{YELLOW}\nExpected sequence for {len(motor_by_id)} connected motors: {RESET}")
                for i in range(total_motors, total_motors - len(motor_by_id), -1):
                    print(f"   • ID {i:2d}: Feetech @ {target_baud:,} bps")
            else:
                print(f"{YELLOW}\nExpected sequence for {len(motor_by_id)} connected motors: {RESET}")
                for i in range(total_motors, total_motors - len(motor_by_id), -1):
                    if xc430_id is not None and i == 1:
                        print(f"   • ID  1: {PURPLE}XC430-T240BB-T{RESET} @ {target_baud:,} bps")
                    else:
                        print(f"   • ID {i:2d}: {BLUE}XC330-T288-T{RESET} @ {target_baud:,} bps")
            defaults = MOTOR_TYPE_DEFAULTS[motor_type]
            print(f"{RED}\n🚫 CONFIGURATION CANNOT CONTINUE{RESET}")
            print(f"   {RED}Please reset the relevant motors to factory default (ID={defaults['default_id']}, baud={defaults['default_baud']:,}) and restart.{RESET}")
            sys.exit(1)
        return valid_sequence
    except Exception as e:
        print(f"{RED}❌ Error scanning: {e}{RESET}")
        return []

def verify_all_motors(configured_ids: list, port: str, target_baud: int, total_motors: int, motor_type: str) -> bool:
    if not configured_ids:
        return True
    try:
        client = create_config_client(motor_type, [], port, target_baud)
        motors = client.scan_for_motors(
            port=port,
            id_range=(1, total_motors),
            baud_rates=[target_baud]
        )
        found_ids = sorted([m['id'] for m in motors])
        expected_ids = sorted(configured_ids)
        if found_ids == expected_ids:
            return True
        else:
            print(f"{RED}❌ Motor verification failed!{RESET}")
            print(f"   Expected: {expected_ids}")
            print(f"   Found:    {found_ids}")
            print("\n🔍 Possible causes:")
            print("   • Multiple motors with same default ID were connected (causes conflicts)")
            print("   • Motor lost power or connection during configuration")
            if motor_type == 'dynamixel':
                print("Check wiring and/or use Dynamixel Wizard to inspect motor settings and connections.")
            else:
                print("Check wiring and inspect motor settings.")
            return False
    except Exception as e:
        print(f"{RED}❌ Error verifying motors: {e}{RESET}")
        return False

def reset_motor_to_factory(motor_type: str, motor_id: int, port: str, current_baud: int, default_id: int, default_baud: int) -> bool:
    try:
        if current_baud != default_baud:
            client = create_config_client(motor_type, [motor_id], port, current_baud)
            client.connect()
            if not client.change_motor_baudrate(motor_id, default_baud):
                print(f"{RED}❌ Failed to change baud rate for motor {motor_id}{RESET}")
                client.disconnect()
                return False
            client.disconnect()
            time.sleep(0.5)

        client = create_config_client(motor_type, [motor_id], port, default_baud)
        client.connect()
        if not client.change_motor_id(motor_id, default_id):
            print(f"{RED}❌ Failed to change ID for motor {motor_id}{RESET}")
            client.disconnect()
            return False
        client.disconnect()

        print(f"{GREEN}✓ Reset motor {motor_id} → ID={default_id}, baudrate={default_baud:,}{RESET}")
        return True
    except Exception as e:
        print(f"{RED}❌ Error resetting motor {motor_id}: {e}{RESET}")
        return False

def reset_loop(port: str, target_baud: int, total_motors: int, motor_type: str, default_id: int, default_baud: int):
    print(f"\n{BOLD}🔄 RESET MODE{RESET} — Scanning for configured motors to reset to factory defaults...")
    print(f"   Factory defaults: ID={default_id}, baudrate={default_baud:,}")
    print(f"   Press {BOLD}Ctrl+C{RESET} to stop.")

    if motor_type == 'feetech':
        while True:
            feetech_safe_connect_prompt(port, "Connect the motor(s) you want to reset")
            print(f"\n🔍 Scanning for motors to reset...")
            client = create_config_client(motor_type, [], port, target_baud)
            motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])
            if not motors:
                print(f"{YELLOW}   No motors found. Check connections.{RESET}")
                continue
            for motor in motors:
                color = motor_color(motor['model_name'])
                print(f"   Found motor: ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps")
                reset_motor_to_factory(
                    motor_type, motor['id'], port, motor['baud_rate'],
                    default_id, default_baud
                )
    else:
        print(f"\n🔍 Waiting for a configured motor to be connected... (Ctrl+C to stop)")
        known_ids = set()
        while True:
            client = create_config_client(motor_type, [], port, target_baud)
            motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])
            found_ids = {m['id'] for m in motors}
            new_motors = [m for m in motors if m['id'] not in known_ids]

            if new_motors:
                for motor in new_motors:
                    color = motor_color(motor['model_name'])
                    print(f"\n   Found motor: ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps")
                    reset_motor_to_factory(
                        motor_type, motor['id'], port, motor['baud_rate'],
                        default_id, default_baud
                    )
                print(f"\n🔍 Waiting for a configured motor to be connected... (Ctrl+C to stop)")

            known_ids = found_ids
            time.sleep(1)

def main():
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

    parser = argparse.ArgumentParser(
        description="Configure motor chain for OrcaHand Assembly. Specify the path to the config file of your OrcaHand model.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)")
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset configured motors back to factory defaults (runs in a loop until Ctrl+C)")
    args = parser.parse_args()

    try:
        config_path = os.path.join(get_model_path(args.model_path), "config.yaml")
        config = read_yaml(config_path)
        TARGET_BAUD = config.get('baudrate', 3000000)
        PORT = config.get('port', '/dev/ttyUSB0')
        motor_ids = config.get('motor_ids', list(range(17, 0, -1)))
        TOTAL_MOTORS = len(motor_ids)
        motor_type = config.get('motor_type', 'dynamixel')
    except Exception as e:
        print(f"{RED}❌ Error loading config: {e}{RESET}")
        return 1

    PORT = validate_or_detect_port(PORT, config_path)

    if motor_type not in MOTOR_TYPE_DEFAULTS:
        print(f"{RED}❌ Unknown motor_type '{motor_type}' in config. Supported: {list(MOTOR_TYPE_DEFAULTS.keys())}{RESET}")
        return 1

    if motor_type == 'feetech':
        TARGET_BAUD = 1000000

    defaults = MOTOR_TYPE_DEFAULTS[motor_type]
    default_id = defaults['default_id']
    default_baud = defaults['default_baud']

    motor_type_label = 'Dynamixel' if motor_type == 'dynamixel' else 'Feetech'

    if args.reset:
        print("\n" + "=" * 60)
        print(f"{BOLD}🔄 ORCAHAND {motor_type_label.upper()} CHAIN RESET 🔄{RESET}")
        print("=" * 60)
        try:
            reset_loop(PORT, TARGET_BAUD, TOTAL_MOTORS, motor_type, default_id, default_baud)
        except KeyboardInterrupt:
            print(f"\n\n{GREEN}Reset mode stopped.{RESET}\n")
        return 0

    print("\n" + "=" * 60)
    print(f"{BOLD}⚙️  ORCAHAND {motor_type_label.upper()} CHAIN CONFIGURATION ⚙️{RESET}")
    print()

    if motor_type == 'dynamixel':
        if 'wrist' in config.get('joint_to_motor_map', {}):
            XC430_ID = 1
            XC330_IDS = sorted([id for id in motor_ids if id != 1], reverse=True)
        else:
            XC430_ID = None
            XC330_IDS = sorted(motor_ids, reverse=True)

        print(f"\nFor the chosen hand model, we need to configure {TOTAL_MOTORS} motors in total:")
        print(f"   • {len(XC330_IDS)} {BLUE}XC330-T288-T{RESET} motors:    {BOLD}IDs {min(XC330_IDS)}-{max(XC330_IDS)}{RESET} @ {TARGET_BAUD:,} bps")
        if XC430_ID is not None:
            print(f"   • 1 {PURPLE}XC430-T240BB-T{RESET} motor:    {BOLD}ID 1{RESET} @ {TARGET_BAUD:,} bps")
        print("=" * 60)

        all_target_ids = XC330_IDS + ([XC430_ID] if XC430_ID is not None else [])
    else:
        XC430_ID = None
        all_target_ids = sorted(motor_ids, reverse=True)

        print(f"\nFor the chosen hand model, we need to configure {TOTAL_MOTORS} Feetech motors in total:")
        print(f"   • {TOTAL_MOTORS} motors:    {BOLD}IDs {min(motor_ids)}-{max(motor_ids)}{RESET} @ {TARGET_BAUD:,} bps")
        print("=" * 60)

    already_configured = scan_already_configured_motors(PORT, TARGET_BAUD, TOTAL_MOTORS, motor_type, default_id, default_baud, XC430_ID).copy()
    configured_ids = already_configured.copy()
    remaining_ids = [id for id in all_target_ids if id not in already_configured]

    if remaining_ids:
        print(f"\n⏳ {BOLD}{len(remaining_ids)} motors{RESET} with IDs: {remaining_ids} {YELLOW}remaining{RESET}.")
        print("\n" + "─" * 60)
        print(f"\n🔧 {BOLD}Troubleshooting guide:{RESET}")
        print("   ✓ Board connection: Properly connected to power and computer")
        print("   ✓ Motor connection: Properly wired to daisy chain")
        print(f"   ✓ Motor settings: ID={default_id}, baudrate={default_baud:,} (factory default)")
        print(f"   ✓ Serial port permissions (Linux): Your user must have read/write access to the serial port.")
        print(f"     Run {BOLD}sudo usermod -aG dialout $USER{RESET} and re-login, or {BOLD}sudo chmod 666 /dev/ttyUSB0{RESET} for a quick fix.")
        print(f"   ✓ Serial port path: Ensure {BOLD}config.yaml{RESET} has the correct port for your OS")
        print(f"     (e.g., {BOLD}/dev/ttyUSB0{RESET} on Linux, {BOLD}/dev/tty.usbserial-*{RESET} on macOS)")
        print(f"   Otherwise the motors {UNDERLINE}won't be detected{RESET} during scanning.")
        print("\n" + "─" * 60)
    else:
        print("\n" + "=" * 60)
        print("\nAll motors are already configured and verified!")
        print(f"🚀 {ORANGE}{BOLD}Ready for operation!{RESET}")
        print()
        return 0

    for step, target_id in enumerate(remaining_ids, len(already_configured) + 1):
        if motor_type == 'dynamixel':
            is_xc430 = (XC430_ID is not None and target_id == 1)
            expected_model = 'XC430-T240BB-T' if is_xc430 else 'XC330-T288-T'
            color = motor_color(expected_model)
        else:
            expected_model = 'Feetech'
            color = ''

        if target_id == max(all_target_ids):
            connection_msg = f"Connect a {color}{expected_model} {BOLD}(ID {target_id}){RESET}{ORANGE} to the {BOLD}board{RESET}"
        else:
            prev_id = target_id + 1
            connection_msg = f"Connect a {color}{expected_model} {BOLD}(ID {target_id}){RESET}{ORANGE} to the previous motor {BOLD}(ID {prev_id}){RESET}"

        print(f"{UNDERLINE}{ORANGE}\nSTEP {step}/{TOTAL_MOTORS}{RESET}")

        if motor_type == 'feetech':
            feetech_safe_connect_prompt(PORT, connection_msg)
        else:
            print(f"{connection_msg}")

        print(f"\n🔍 Scanning for default {color}{expected_model}{RESET} motor...")

        try:
            while True:
                if scan_for_default_motor(expected_model, PORT, motor_type, default_id, default_baud):
                    if configure_default_motor(target_id, PORT, TARGET_BAUD, motor_type, default_id, default_baud):
                        configured_ids.append(target_id)
                        if not verify_all_motors(configured_ids, PORT, TARGET_BAUD, TOTAL_MOTORS, motor_type):
                            return 1
                        break
                    else:
                        return 1
                else:
                    time.sleep(1)
        except Exception as e:
            print(f"\n ERROR: {e}")
            sys.exit(1)

    print("\n" + "=" * 60)
    print("\nAll motors have been configured and verified!")
    print(f"🚀 {ORANGE}{BOLD}Ready for operation!{RESET}")
    print()
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n" + "─" * 60)
        print(f"{RED}❌ CONFIGURATION INTERRUPTED ❌{RESET}")
        print(f"\n ⚠️  {YELLOW}Not all motors have been fully configured yet!{RESET}")
        print("    Rerun this script to continue with configuration.\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n ERROR: {e}")
        sys.exit(1)
