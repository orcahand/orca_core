import sys
import time
import logging
import argparse
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from orca_core.hardware.dynamixel_client import DynamixelClient
from orca_core.utils import read_yaml, get_model_path

DEFAULT_ID, DEFAULT_BAUD = 1, 57600
RED = '\033[91m'
GREEN = '\033[92m'
BLUE = '\033[94m'
PURPLE = '\033[95m'
YELLOW = '\033[93m'
ORANGE = '\033[38;5;208m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
RESET = '\033[0m'

def scan_for_default_motor(expected_type: str, port: str) -> bool:
    color = BLUE if 'XC330' in expected_type else PURPLE if 'XC430' in expected_type else ''
    print(f"\n🔍 Scanning for default {color}{expected_type}{RESET} motor...")
    try:
        client = DynamixelClient([], port=port, baudrate=DEFAULT_BAUD)
        motors = client.scan_for_motors(port=port, id_range=(DEFAULT_ID, DEFAULT_ID), baud_rates=[DEFAULT_BAUD])
        if not motors:
            print(f"{RED}❌ No motors with default settings found{RESET}")
            return False
        motor=motors[0]
        if expected_type in motor['model_name']:
            color = BLUE if 'XC330' in motor['model_name'] else PURPLE if 'XC430' in motor['model_name'] else ''
            print(f"{GREEN}✓ Found default {color}{motor['model_name']}{GREEN} motor{RESET}")
            return True
        else:
            expected_color = BLUE if 'XC330' in expected_type else PURPLE if 'XC430' in expected_type else ''
            actual_color = BLUE if 'XC330' in motor['model_name'] else PURPLE if 'XC430' in motor['model_name'] else ''
            print(f"{RED}❌ Wrong motor type: {actual_color}{motor['model_name']}{RESET} (expected {expected_color}{expected_type}{RESET}){RESET}")
            return False
    except Exception as e:
        print(f"{RED}❌ Scan error: {e}{RESET}")
        return False

def configure_default_motor(target_id: int, port: str, target_baud: int) -> bool:
    """Configure motor with default settings (ID=1, baudrate=57600) to Target ID and Target Baudrate."""
    try:
        client = DynamixelClient([DEFAULT_ID], port=port, baudrate=DEFAULT_BAUD)
        client.connect()
        if not client.change_motor_id(DEFAULT_ID, target_id):
            print(f"{RED}❌ Failed to change ID to {target_id}{RESET}")
            client.port_handler.closePort()
            return False
        client.port_handler.closePort()
        time.sleep(0.5)
        client = DynamixelClient([target_id], port=port, baudrate=DEFAULT_BAUD)
        client.connect()
        if not client.change_motor_baudrate(target_id, target_baud):
            print(f"{RED}❌ Failed to change baud rate to {target_baud}{RESET}")
            client.port_handler.closePort()
            return False
        client.port_handler.closePort()
        print(f"{GREEN}✓ Successfully configured the new motor to ID={target_id}, baudrate={target_baud}{RESET}")
        return True
    except Exception as e:
        print(f"{RED}❌ Error configuring motor: {e}{RESET}")
        return False

def scan_already_configured_motors(port: str, target_baud: int, total_motors: int, xc430_id: int = None) -> list:
    """Scan for pre-configured motors and validate sequence."""
    print(f"\n🔍 Pre-Scanning for already {YELLOW}configured motors{RESET}...")
    try:
        client = DynamixelClient([], port=port, baudrate=target_baud)
        motors = client.scan_for_motors(port=port, id_range=(1, total_motors), baud_rates=[target_baud])
        if not motors:
            print("   No pre-configured motors detected")
            return []
        found_ids = sorted([m['id'] for m in motors])
        valid_sequence, expected_id = [], total_motors
        min_xc330_id = 2 if xc430_id is not None else 1
        for motor in sorted(motors, key=lambda x: x['id'], reverse=True):
            motor_id = motor['id']
            if motor_id in range(min_xc330_id, total_motors + 1) and motor_id == expected_id:
                if 'XC330' in motor['model_name']:
                    valid_sequence.append(motor_id)
                    expected_id -= 1
                else:
                    break
            elif motor_id not in range(min_xc330_id, total_motors + 1):
                break
        if xc430_id is not None and xc430_id in found_ids and expected_id == 1:
            xc430_motor = next((m for m in motors if m['id'] == xc430_id), None)
            if xc430_motor and 'XC430' in xc430_motor['model_name']:
                valid_sequence.append(xc430_id)            
        invalid_motors = [id for id in found_ids if id not in valid_sequence]
        # Display results
        if valid_sequence:
            print(f"{GREEN}\nFound {len(valid_sequence)} valid, pre-configured motors with correct ID order:{RESET}")
            for motor in sorted(motors, key=lambda x: x['id'], reverse=True):
                if motor['id'] in valid_sequence:
                    color = BLUE if 'XC330' in motor['model_name'] else PURPLE if 'XC430' in motor['model_name'] else ''
                    print(f"   • ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps{RESET}")
        if invalid_motors:
            print(f"{RED}Found {len(invalid_motors)} motors with invalid configuration:{RESET}")
            for motor in sorted(motors, key=lambda x: x['id'], reverse=True):
                if motor['id'] in invalid_motors:
                    color = BLUE if 'XC330' in motor['model_name'] else PURPLE if 'XC430' in motor['model_name'] else ''
                    print(f"   • ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps{RESET}")
            print(f"{YELLOW}\nExpected sequence for {len(found_ids)} connected motors: {RESET}")
            for i in range(total_motors, total_motors - len(found_ids), -1):
                if xc430_id is not None and i == 1:
                    icon = f"• ID  1: {PURPLE}XC430-T240BB-T{RESET}"
                else:
                    icon = f"• ID {i:2d}: {BLUE}XC330-T288-T{RESET}"
                print(f"   {icon} @ {target_baud:,} bps")
            print(f"{RED}\n🚫 CONFIGURATION CANNOT CONTINUE{RESET}")
            print(f"   {RED}Please reset the relevant motors to factory default (ID=1, baud=57,600) and restart.{RESET}")
            sys.exit(1)
        return valid_sequence
    except Exception as e:
        print(f"{RED}❌ Error scanning: {e}{RESET}")
        return []

def verify_all_motors(configured_ids: list, port: str, target_baud: int, total_motors: int) -> bool:
    """Verify that all previously configured motors are still detected."""
    if not configured_ids:
        return True      
    try:
        client = DynamixelClient([], port=port, baudrate=target_baud)
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
            print("   • Multiple motors with same ID=1 were connected (causes conflicts)")
            print("   • Motor lost power or connection during configuration)")
            print("Check wiring and/or use Dynamixel Wizard to inspect motor settings and connections.")
            return False  
    except Exception as e:
        print(f"{RED}❌ Error verifying motors: {e}{RESET}")
        return False

def main():
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')
    print("\n" + "=" * 90)
    print(f"{BOLD}🤖  ORCAHAND DYNAMIXEL CHAIN CONFIGURATION{RESET}")
    parser = argparse.ArgumentParser(
        description="Configure Dynamixel motor chain for OrcaHand Assembly. "
                   "Specify the path to the config file of your OrcaHand model.")
    parser.add_argument(
        "model_path",
        type=str,
        nargs="?",
        default=None,
        help="Path to the orcahand model folder (e.g., /path/to/orcahand_v1)")
    args = parser.parse_args()
    try:
        config_path = os.path.join(get_model_path(args.model_path), "config.yaml")
        config = read_yaml(config_path)
        TARGET_BAUD = config.get('baudrate', 3000000)
        PORT = config.get('port', '/dev/ttyUSB0')
        motor_ids = config.get('motor_ids', list(range(17, 0, -1)))
        TOTAL_MOTORS = len(motor_ids)
        if 'wrist' in config.get('joint_to_motor_map', {}):
            XC430_ID = 1
            XC330_IDS = sorted([id for id in motor_ids if id != 1], reverse=True)  # All except wrist ID, highest to lowest
        else:
            XC430_ID = None
            XC330_IDS = sorted(motor_ids, reverse=True)  # All motor IDs, highest to lowest
    except Exception as e:
        print(f"{RED}❌ Error loading config: {e}{RESET}")
        return 1
    
    print("=" * 90)
    print(f"\n 🎯 For the chosen hand type, we need to configure {TOTAL_MOTORS} motors in total:")
    print(f"   • {len(XC330_IDS)} {BLUE}XC330-T288-T{RESET} motors:    {BOLD}IDs {min(XC330_IDS)}-{max(XC330_IDS)}{RESET} @ {TARGET_BAUD:,} bps")
    if XC430_ID is not None:
        print(f"   • 1 {PURPLE}XC430-T240BB-T{RESET} motor:    {BOLD}ID 1{RESET} @ {TARGET_BAUD:,} bps")
    else:
        pass
    print("\n" + "─" * 60)

    already_configured = scan_already_configured_motors(PORT, TARGET_BAUD, TOTAL_MOTORS, XC430_ID).copy()
    configured_ids = already_configured.copy()
    all_target_ids = XC330_IDS + ([XC430_ID] if XC430_ID is not None else [])
    remaining_ids = [id for id in all_target_ids if id not in already_configured]

    if remaining_ids:
        print(f"\n⏳ Remaining: Configuration of {len(remaining_ids)} motors with IDs: {remaining_ids}")
    else:
        print("\n🎉 All motors already configured and verified!")
        print(f"🚀 {YELLOW}Ready for operation!{RESET}")
        return 0
    
    for step, target_id in enumerate(remaining_ids, len(already_configured) + 1):
        is_xc430 = (XC430_ID is not None and target_id == 1)
        motor_type = 'XC430-T240BB-T' if is_xc430 else 'XC330-T288-T'
        final_note = 'FINAL ' if is_xc430 else ''
        motor_color = BLUE if 'XC330' in motor_type else PURPLE if 'XC430' in motor_type else ''
        print(f"{BOLD}{UNDERLINE}{ORANGE}\n{final_note}STEP {step}/{TOTAL_MOTORS} → {motor_color}{motor_type}{ORANGE} (ID {target_id}):{RESET}")
        
        if target_id == max(all_target_ids):
            connection_msg = f"🔌 Connect a {motor_color}{motor_type}{RESET} to the board"
        else:
            prev_id = target_id + 1
            connection_msg = f"🔌 Connect a {motor_color}{motor_type}{RESET} to the previous motor (ID {prev_id})"
        input(f"\n• {connection_msg}, press {BOLD}ENTER{RESET}...")
        
        while not scan_for_default_motor(motor_type, PORT):
            print(f"\n🔧 {BOLD}Troubleshooting checklist:{RESET}")
            motor_color = BLUE if 'XC330' in motor_type else PURPLE if 'XC430' in motor_type else ''
            print(f"   ✓ Motor type: {motor_color}{motor_type}{RESET}")
            print("   ✓ Motor connection: Properly wired to daisy chain")
            print("   ✓ Motor settings: ID=1, baud=57,600 (factory default)")
            print("\n" + "─" * 40)
            retry = input("🔄 Try again? (y/N): ").lower().strip()
            if retry != 'y':
                print(f"\n{RED}❌ Configuration cancelled{RESET}")
                return 1
        
        if configure_default_motor(target_id, PORT, TARGET_BAUD):
            configured_ids.append(target_id)
            if not verify_all_motors(configured_ids, PORT, TARGET_BAUD, TOTAL_MOTORS):
                print(f"{RED}❌ Motor verification failed after configuration{RESET}")
                return 1
        else:
            print(f"{RED}❌ Failed to configure motor ID={target_id}{RESET}")
            return 1

    print("\n" + "─" * 60)
    print("🎉 All motors have been configured and verified!")
    print(f"🚀 {YELLOW}Ready for operation!{RESET}")
    print("─" * 60)
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