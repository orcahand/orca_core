import argparse
import contextlib
import logging
import os
import subprocess
import sys
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core.hardware.dynamixel_client import DynamixelClient, BAUD_RATE_MAP as DYNAMIXEL_BAUD_RATE_MAP
from orca_core.hardware.feetech_client import FeetechClient, FEETECH_BAUD_RATE_MAP
from orca_core.utils import auto_detect_port, get_and_choose_port, get_model_path, read_yaml


RED, GREEN, BLUE, PURPLE, YELLOW = '\033[91m', '\033[92m', '\033[94m', '\033[95m', '\033[93m'
ORANGE, BOLD, RESET = '\033[38;5;208m', '\033[1m', '\033[0m'

FEETECH, DYNAMIXEL = 'feetech', 'dynamixel'

# Per-family lookup tables. Index by motor_type instead of branching.
MOTOR_TYPE_DEFAULTS = {
    DYNAMIXEL: {'default_id': 1, 'default_baud': 57600},
    FEETECH:   {'default_id': 1, 'default_baud': 1000000},
}
MOTOR_MODELS = {
    DYNAMIXEL: {'wrist': 'XC430', 'finger': 'XC330'},
    FEETECH:   {'wrist': 'HLS3930', 'finger': 'HLS3915'},
}

logger = logging.getLogger(__name__)


# --- UI helpers -------------------------------------------------------------

def validate_or_detect_port(port: str, motor_type: str) -> str:
    """Return ``port`` if present; else auto-detect or prompt; else exit."""
    if os.path.exists(port):
        return port
    print(f"{YELLOW}⚠ Port {port} not found.{RESET}")
    for candidate, source in (
        (auto_detect_port(motor_type), "auto-detected"),
        (get_and_choose_port(), "selected"),
    ):
        if candidate and os.path.exists(candidate):
            print(f"{GREEN}✓ Using {source} port: {candidate}{RESET}")
            return candidate
    print(f"{RED}❌ No valid port found. Check your USB connection.{RESET}")
    sys.exit(1)


def wait_for_port(port: str, *, present: bool, timeout: float = 30):
    """Block until ``port`` appears (present=True) or disappears (present=False)."""
    if present:
        print(f"\n{YELLOW}▶  RESTORE POWER:{RESET} Plug the USB cable back in.")
    else:
        print(f"\n{YELLOW}⚠  REMOVE POWER:{RESET} Unplug the USB cable from your computer.")
    start = time.time()
    while os.path.exists(port) != present:
        if time.time() - start > timeout:
            print(f"{RED}❌ Timeout waiting for {port}.{RESET}")
            start = time.time()
        time.sleep(0.3)
    if present:
        time.sleep(0.5)  # let the OS finish bringing the device up
    print(f"{GREEN}✓ USB {'re' if present else 'dis'}connected.{RESET}")


def feetech_safe_connect_prompt(port: str, connection_msg: str):
    """Power-cycle dance: USB out → user plugs motor → USB back in."""
    wait_for_port(port, present=False)
    print(f"\n{ORANGE}🔌 {connection_msg}{RESET}")
    input(f"\n   Press {BOLD}Enter{RESET} when the motor is connected correctly...")
    wait_for_port(port, present=True)


def play_success_beep():
    """Best-effort audible cue when a motor finishes configuration."""
    for cmd in (
        ['paplay', '/usr/share/sounds/freedesktop/stereo/complete.oga'],
        ['beep', '-f', '800', '-l', '100'],
    ):
        try:
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1)
            return
        except Exception:
            continue
    print('\a', end='', flush=True)


def motor_color(model_name: str) -> str:
    """ANSI colour for a motor model: finger→blue, wrist→purple."""
    for family in MOTOR_MODELS.values():
        if family['finger'] in model_name:
            return BLUE
        if family['wrist'] in model_name:
            return PURPLE
    return ''


def _print_motor_row(motor: dict) -> None:
    color = motor_color(motor['model_name'])
    print(f"   • ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps{RESET}")


def _banner(title: str) -> None:
    print("\n" + "=" * 60)
    print(f"{BOLD}{title}{RESET}")
    print("=" * 60)


# --- Motor client helpers ---------------------------------------------------

def _make_client(motor_type, motor_ids, port, baudrate):
    cls = FeetechClient if motor_type == FEETECH else DynamixelClient
    return cls(motor_ids, port=port, baudrate=baudrate)


@contextlib.contextmanager
def _config_session(motor_type, motor_ids, port, baudrate):
    """Connect a client, yield it, close the port directly on exit.

    Bypasses the normal disconnect-with-torque-disable because callers may
    have just changed the motor's ID or baudrate mid-session.
    """
    client = _make_client(motor_type, motor_ids, port, baudrate)
    client.connect()
    try:
        yield client
    finally:
        try:
            client.port_handler.closePort()
        except Exception:
            pass


def detect_motor_type(port: str) -> str | None:
    """Probe ``port`` at each family's factory defaults to identify motors.

    Fresh motors ship at known (baud, ID) pairs that differ between families:
    Dynamixel @ 57600 ID 1, Feetech @ 1M ID 1.
    """
    for motor_type, defaults in MOTOR_TYPE_DEFAULTS.items():
        baud, default_id = defaults['default_baud'], defaults['default_id']
        print(f"  Probing {motor_type} (ID {default_id} @ {baud})...", end=' ', flush=True)
        try:
            client = _make_client(motor_type, [], port, baud)
            if client.scan_for_motors(port=port, id_range=(default_id, default_id), baud_rates=[baud]):
                print(f"{GREEN}found.{RESET}")
                return motor_type
            print("nothing.")
        except Exception as e:
            print(f"{YELLOW}error ({e}).{RESET}")
    return None


def scan_for_default_motor(expected_type: str, port: str, motor_type: str, default_id: int, default_baud: int) -> bool:
    """Return True if a factory-default motor of the expected model is present."""
    try:
        client = _make_client(motor_type, [], port, default_baud)
        motors = client.scan_for_motors(port=port, id_range=(default_id, default_id), baud_rates=[default_baud])
        if not motors:
            return False
        motor = motors[0]
        actual_color = motor_color(motor['model_name'])
        if expected_type in motor['model_name']:
            print(f"{GREEN}✓ Found default {actual_color}{motor['model_name']}{GREEN} motor{RESET}")
            return True
        print(
            f"{RED}❌ Wrong motor type: {actual_color}{motor['model_name']}{RESET}"
            f" (expected {motor_color(expected_type)}{expected_type}{RESET})"
        )
        return False
    except Exception as e:
        print(f"{RED}❌ Scan error: {e}{RESET}")
        return False


def configure_default_motor(target_id: int, port: str, target_baud: int, motor_type: str, default_id: int, default_baud: int) -> bool:
    """Re-program a factory-default motor to ``target_id`` and ``target_baud``."""
    try:
        with _config_session(motor_type, [default_id], port, default_baud) as client:
            if not client.change_motor_id(default_id, target_id):
                logger.error("Failed to change ID to %d", target_id)
                return False
        time.sleep(0.5)

        if target_baud != default_baud:
            with _config_session(motor_type, [target_id], port, default_baud) as client:
                if not client.change_motor_baudrate(target_id, target_baud):
                    logger.error("Failed to change baud rate to %d", target_baud)
                    return False

        print(f"{GREEN}✓ Successfully configured motor → ID={target_id}, baudrate={target_baud}{RESET}")
        play_success_beep()
        return True
    except Exception as e:
        logger.error("Error configuring motor: %s", e)
        return False


def scan_already_configured_motors(port: str, target_baud: int, total_motors: int, motor_type: str, default_id: int, default_baud: int, wrist_id: int | None = None) -> list:
    """Pre-scan the chain for motors already at their target ID + baud.

    Returns the valid pre-configured motor IDs (contiguous, descending from
    ``total_motors`` with optional wrist at ID 1). Aborts when motors are
    present but don't form a valid prefix of the expected sequence — the
    user must reset the offenders to factory defaults before re-running.
    """
    print(f"\n🔍 Pre-scanning for {YELLOW}already configured{RESET} motors...")
    try:
        client = _make_client(motor_type, [], port, target_baud)
        motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])

        finger_model = MOTOR_MODELS[motor_type]['finger']
        wrist_model = MOTOR_MODELS[motor_type]['wrist']

        # On Feetech target_baud == default_baud, so a fresh HLS3915 at ID 1
        # is indistinguishable from a configured wrist; drop it (real HLS3930
        # wrist stays — model name disambiguates).
        if target_baud == default_baud:
            motors = [m for m in motors if m['id'] != default_id or wrist_model in m['model_name']]
        if not motors:
            print("   No pre-configured motors detected")
            return []

        motor_by_id = {m['id']: m for m in motors}
        min_finger_id = 2 if wrist_id is not None else 1
        valid_sequence: list[int] = []
        expected_id = total_motors

        # Walk IDs descending; accept each only if it sits in the finger
        # range, matches the next-expected ID, and is the right model.
        # Stops at the first gap so partial chains never count as "valid".
        for motor_id in sorted(motor_by_id.keys(), reverse=True):
            if motor_id not in range(min_finger_id, total_motors + 1):
                break
            if motor_id != expected_id or finger_model not in motor_by_id[motor_id]['model_name']:
                break
            valid_sequence.append(motor_id)
            expected_id -= 1

        if wrist_id is not None and wrist_id in motor_by_id and expected_id == 1:
            if wrist_model in motor_by_id[wrist_id]['model_name']:
                valid_sequence.append(wrist_id)

        invalid_motors = [mid for mid in motor_by_id if mid not in valid_sequence]

        if valid_sequence:
            print(f"{GREEN}\nFound {len(valid_sequence)} valid, pre-configured motors:{RESET}")
            for motor_id in sorted(valid_sequence, reverse=True):
                _print_motor_row(motor_by_id[motor_id])

        if invalid_motors:
            print(f"{RED}Found {len(invalid_motors)} motors with invalid configuration:{RESET}")
            for motor_id in sorted(invalid_motors, reverse=True):
                _print_motor_row(motor_by_id[motor_id])
            print(f"{YELLOW}\nExpected sequence for {len(motor_by_id)} connected motors:{RESET}")
            for i in range(total_motors, total_motors - len(motor_by_id), -1):
                model = wrist_model if (wrist_id is not None and i == 1) else finger_model
                colour = PURPLE if model == wrist_model else BLUE
                print(f"   • ID {i:2d}: {colour}{model}{RESET} @ {target_baud:,} bps")
            print(
                f"{RED}\n🚫 CONFIGURATION CANNOT CONTINUE — reset the offending motors to "
                f"factory default (ID={default_id}, baud={default_baud:,}) and restart.{RESET}"
            )
            sys.exit(1)
        return valid_sequence
    except Exception as e:
        print(f"{RED}❌ Error scanning: {e}{RESET}")
        return []


def verify_all_motors(configured_ids: list, port: str, target_baud: int, total_motors: int, motor_type: str) -> bool:
    """Re-scan after each configure step to detect dropped or duplicated motors."""
    if not configured_ids:
        return True
    try:
        client = _make_client(motor_type, [], port, target_baud)
        motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])
        found_ids = sorted(m['id'] for m in motors)
        expected_ids = sorted(configured_ids)
        if found_ids == expected_ids:
            return True
        print(f"{RED}❌ Motor verification failed!{RESET}")
        print(f"   Expected: {expected_ids}")
        print(f"   Found:    {found_ids}")
        print("   Likely cause: duplicate default IDs on the bus, or a motor lost power.")
        return False
    except Exception as e:
        print(f"{RED}❌ Error verifying motors: {e}{RESET}")
        return False


def change_motor_baudrate_only(motor_type: str, motor_id: int, port: str, current_baud: int, new_baud: int) -> bool:
    """Change a motor's baudrate, leaving its ID alone."""
    if current_baud == new_baud:
        logger.info("Motor %d already at %s bps", motor_id, f"{new_baud:,}")
        return True
    try:
        with _config_session(motor_type, [motor_id], port, current_baud) as client:
            if not client.change_motor_baudrate(motor_id, new_baud):
                logger.error("Failed to change baud rate for motor %d", motor_id)
                return False
        print(f"{GREEN}✓ Motor {motor_id} baudrate: {current_baud:,} → {new_baud:,}{RESET}")
        return True
    except Exception as e:
        logger.error("Error changing baudrate for motor %d: %s", motor_id, e)
        return False


def reset_motor_to_factory(motor_type: str, motor_id: int, port: str, current_baud: int, default_id: int, default_baud: int) -> bool:
    """Revert a configured motor back to factory defaults."""
    try:
        if current_baud != default_baud:
            with _config_session(motor_type, [motor_id], port, current_baud) as client:
                if not client.change_motor_baudrate(motor_id, default_baud):
                    logger.error("Failed to change baud rate for motor %d", motor_id)
                    return False
            time.sleep(0.5)
        with _config_session(motor_type, [motor_id], port, default_baud) as client:
            if not client.change_motor_id(motor_id, default_id):
                logger.error("Failed to change ID for motor %d", motor_id)
                return False
        print(f"{GREEN}✓ Reset motor {motor_id} → ID={default_id}, baudrate={default_baud:,}{RESET}")
        return True
    except Exception as e:
        logger.error("Error resetting motor %d: %s", motor_id, e)
        return False


# --- Bulk loops -------------------------------------------------------------

def _process_motors_loop(port: str, scan_baud: int, total_motors: int, motor_type: str, action, label: str, prompt: str):
    """Generic scan-and-act loop used by --reset and --baudrate modes."""
    print(f"\n🔍 {label} — press {BOLD}Ctrl+C{RESET} to stop.")
    if motor_type == FEETECH:
        while True:
            feetech_safe_connect_prompt(port, prompt)
            client = _make_client(motor_type, [], port, scan_baud)
            motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[scan_baud])
            if not motors:
                print(f"{YELLOW}   No motors found. Check connections.{RESET}")
                continue
            for motor in motors:
                _print_motor_row(motor)
                action(motor)
    else:
        known_ids: set[int] = set()
        while True:
            client = _make_client(motor_type, [], port, scan_baud)
            motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[scan_baud])
            found_ids = {m['id'] for m in motors}
            for motor in [m for m in motors if m['id'] not in known_ids]:
                _print_motor_row(motor)
                action(motor)
            known_ids = found_ids
            time.sleep(1)


def baudrate_change_loop(port: str, current_baud: int, new_baud: int, total_motors: int, motor_type: str):
    _process_motors_loop(
        port, current_baud, total_motors, motor_type,
        action=lambda m: change_motor_baudrate_only(motor_type, m['id'], port, m['baud_rate'], new_baud),
        label=f"Changing motor baudrates to {new_baud:,} bps",
        prompt="Connect the motor(s) you want to change baudrate",
    )


def reset_loop(port: str, target_baud: int, total_motors: int, motor_type: str, default_id: int, default_baud: int):
    _process_motors_loop(
        port, target_baud, total_motors, motor_type,
        action=lambda m: reset_motor_to_factory(motor_type, m['id'], port, m['baud_rate'], default_id, default_baud),
        label=f"Resetting motors to defaults (ID={default_id}, baud={default_baud:,})",
        prompt="Connect the motor(s) you want to reset",
    )


# --- main -------------------------------------------------------------------

def _load_config(config_path_arg: str | None) -> dict:
    """Resolve the config path and read the yaml, exiting on error."""
    try:
        if config_path_arg is None:
            config_path = os.path.join(get_model_path(), "config.yaml")
        else:
            config_path = os.path.abspath(config_path_arg)
            if not os.path.exists(config_path):
                print(f"{RED}❌ config.yaml not found: {config_path}{RESET}")
                sys.exit(1)
        return read_yaml(config_path)
    except Exception as e:
        print(f"{RED}❌ Error loading config: {e}{RESET}")
        sys.exit(1)


def _resolve_motor_type(explicit: str | None, port: str) -> str:
    if explicit is not None:
        return explicit
    print(f"\n{BOLD}Auto-detecting motor family on {port}...{RESET}")
    motor_type = detect_motor_type(port)
    if motor_type is None:
        print(
            f"\n{RED}❌ No motors responded at either family's factory defaults.{RESET}\n"
            "   Check power, wiring, and that motors are still at factory defaults\n"
            "   (re-runs of this script change them). Or pass --motor-type explicitly."
        )
        sys.exit(1)
    print(f"{GREEN}✓ Detected {motor_type} motors.{RESET}\n")
    return motor_type


def main():
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

    parser = argparse.ArgumentParser(
        description="Configure motor chain for OrcaHand Assembly. Specify the path to the config file of your OrcaHand model.")
    parser.add_argument(
        "config_path", type=str, nargs="?", default=None,
        help="Path to config.yaml. Defaults to the bundled model when omitted.")
    parser.add_argument(
        "--reset", action="store_true",
        help="Reset configured motors back to factory defaults (loops until Ctrl+C).")
    parser.add_argument(
        "--baudrate", type=int, metavar="BAUD",
        help="Change all motors to the given baudrate without modifying IDs (loops until Ctrl+C).")
    parser.add_argument(
        "--motor-type", type=str, choices=list(MOTOR_TYPE_DEFAULTS.keys()),
        help="Override motor family. Auto-detected from factory defaults when omitted.")
    args = parser.parse_args()

    config = _load_config(args.config_path)
    motor_ids = config.get('motor_ids', list(range(17, 0, -1)))
    total_motors = len(motor_ids)
    port = validate_or_detect_port(config.get('port', '/dev/ttyUSB0'), args.motor_type or DYNAMIXEL)
    motor_type = _resolve_motor_type(args.motor_type, port)

    target_baud = config.get('baudrate') or 1_000_000
    defaults = MOTOR_TYPE_DEFAULTS[motor_type]
    default_id, default_baud = defaults['default_id'], defaults['default_baud']
    label = motor_type.capitalize()

    if args.reset:
        _banner(f"🔄 ORCAHAND {label.upper()} CHAIN RESET 🔄")
        try:
            reset_loop(port, target_baud, total_motors, motor_type, default_id, default_baud)
        except KeyboardInterrupt:
            print(f"\n\n{GREEN}Reset mode stopped.{RESET}\n")
        return 0

    if args.baudrate is not None:
        valid_baudrates = (
            sorted(FEETECH_BAUD_RATE_MAP.keys(), reverse=True) if motor_type == FEETECH
            else sorted(DYNAMIXEL_BAUD_RATE_MAP.keys(), reverse=True)
        )
        if args.baudrate not in valid_baudrates:
            print(f"{RED}❌ Invalid baudrate {args.baudrate:,} for {label} motors. "
                  f"Valid: {[f'{b:,}' for b in valid_baudrates]}{RESET}")
            return 1
        _banner(f"🔄 ORCAHAND {label.upper()} BAUDRATE CHANGE 🔄")
        print(f"\nChanging all motors to {BOLD}{args.baudrate:,} bps{RESET} "
              f"(current config baud: {target_baud:,}). IDs unchanged.")
        try:
            baudrate_change_loop(port, target_baud, args.baudrate, total_motors, motor_type)
        except KeyboardInterrupt:
            print(f"\n\n{GREEN}Baudrate change mode stopped.{RESET}\n")
        return 0

    # Full chain configuration.
    _banner(f"⚙️  ORCAHAND {label.upper()} CHAIN CONFIGURATION ⚙️")

    has_wrist = 'wrist' in config.get('joint_to_motor_map', {})
    wrist_id = 1 if has_wrist else None
    finger_ids = sorted(
        [mid for mid in motor_ids if mid != 1] if has_wrist else motor_ids,
        reverse=True,
    )
    finger_model = MOTOR_MODELS[motor_type]['finger']
    wrist_model = MOTOR_MODELS[motor_type]['wrist']

    print(f"\nConfiguring {total_motors} motors:")
    print(f"   • {len(finger_ids)} {BLUE}{finger_model}{RESET}: {BOLD}IDs {min(finger_ids)}-{max(finger_ids)}{RESET} @ {target_baud:,} bps")
    if wrist_id is not None:
        print(f"   • 1 {PURPLE}{wrist_model}{RESET}: {BOLD}ID 1{RESET} @ {target_baud:,} bps")
    print("=" * 60)

    all_target_ids = finger_ids + ([wrist_id] if wrist_id is not None else [])
    already_configured = scan_already_configured_motors(
        port, target_baud, total_motors, motor_type, default_id, default_baud, wrist_id,
    )
    configured_ids = list(already_configured)
    remaining_ids = [mid for mid in all_target_ids if mid not in already_configured]

    if not remaining_ids:
        print(f"\nAll motors already configured. 🚀 {ORANGE}{BOLD}Ready for operation!{RESET}\n")
        return 0

    print(
        f"\n⏳ {BOLD}{len(remaining_ids)} motors{RESET} remaining: {remaining_ids}\n"
        f"   Factory defaults expected: ID={default_id}, baud={default_baud:,}.\n"
        f"   If a motor isn't detected, check power, daisy-chain wiring, and serial-port "
        f"permissions ({BOLD}sudo usermod -aG dialout $USER{RESET} on Linux)."
    )

    for step, target_id in enumerate(remaining_ids, len(already_configured) + 1):
        is_wrist = (wrist_id is not None and target_id == 1)
        expected_model = wrist_model if is_wrist else finger_model
        color = motor_color(expected_model)
        location = (
            f"the {BOLD}board{RESET}" if target_id == max(all_target_ids)
            else f"the previous motor {BOLD}(ID {target_id + 1}){RESET}"
        )
        msg = f"Connect a {color}{expected_model} {BOLD}(ID {target_id}){RESET}{ORANGE} to {location}"

        print(f"\n{BOLD}{ORANGE}STEP {step}/{total_motors}{RESET}")
        if motor_type == FEETECH:
            feetech_safe_connect_prompt(port, msg)
        else:
            print(msg)

        print(f"\n🔍 Scanning for default {color}{expected_model}{RESET} motor...")
        while True:
            if scan_for_default_motor(expected_model, port, motor_type, default_id, default_baud):
                if not configure_default_motor(target_id, port, target_baud, motor_type, default_id, default_baud):
                    return 1
                configured_ids.append(target_id)
                if not verify_all_motors(configured_ids, port, target_baud, total_motors, motor_type):
                    return 1
                break
            time.sleep(1)

    print(f"\nAll motors configured. 🚀 {ORANGE}{BOLD}Ready for operation!{RESET}\n")
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{RED}❌ CONFIGURATION INTERRUPTED{RESET}")
        print(f" {YELLOW}Not all motors have been configured yet — rerun to continue.{RESET}\n")
        sys.exit(1)
    except Exception as e:
        print(f"\n ERROR: {e}")
        sys.exit(1)
