#!/usr/bin/env python
"""Configure the motor chain of a fresh ORCA hand: assign each motor its ID and baud rate.

Plug the motors in one at a time when prompted. Re-running resumes from wherever
the previous run stopped.

Usage:
    uv run python scripts/configure_motor_chain.py [CONFIG]
    uv run python scripts/configure_motor_chain.py CONFIG --reset
    uv run python scripts/configure_motor_chain.py CONFIG --baudrate 1000000
"""

import argparse
import logging
import os
import subprocess
import sys
import time

from orca_core.constants import FINGER, MOTOR_MODELS, SUPPORTED_MOTOR_TYPES, WRIST
from orca_core.maintenance import (
    MotorChainAborted,
    MotorChainError,
    change_all_baudrates,
    configure_motor_chain,
    detect_motor_type,
    plan_motor_chain,
    reset_all_motors,
    resolve_port,
    valid_baudrates,
)
from orca_core.utils import get_and_choose_port, get_model_path, read_yaml

RED, GREEN, BLUE, PURPLE, YELLOW = '\033[91m', '\033[92m', '\033[94m', '\033[95m', '\033[93m'
ORANGE, BOLD, RESET = '\033[38;5;208m', '\033[1m', '\033[0m'


def motor_color(model_name: str) -> str:
    """ANSI colour for a motor model: finger→blue, wrist→purple."""
    for family in MOTOR_MODELS.values():
        if family[FINGER] in model_name:
            return BLUE
        if family[WRIST] in model_name:
            return PURPLE
    return ''


def _print_motor_row(motor: dict) -> None:
    color = motor_color(motor['model_name'])
    print(f"   • ID {motor['id']:2d}: {color}{motor['model_name']}{RESET} @ {motor['baud_rate']:,} bps")


def _banner(title: str) -> None:
    print("\n" + "=" * 60)
    print(f"{BOLD}{title}{RESET}")
    print("=" * 60)


def play_success_beep() -> None:
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


def on_progress(event: dict) -> None:
    """Render a motor_chain progress event to the terminal."""
    name = event["event"]
    if name == "probing_motor_type":
        print(f"  Probing {event['motor_type']} (ID {event['motor_id']} @ {event['baudrate']})...")
    elif name == "motor_type_detected":
        print(f"{GREEN}✓ Detected {event['motor_type']} motors.{RESET}\n")
    elif name == "probe_failed":
        print(f"{YELLOW}  probe of {event['motor_type']} failed: {event['error']}{RESET}")
    elif name == "chain_started":
        print(f"\nConfiguring {event['total_motors']} motors @ {event['target_baud']:,} bps")
    elif name == "prescan_done":
        if event["already_configured"]:
            print(f"{GREEN}\nFound {len(event['already_configured'])} pre-configured motors:{RESET}")
            for motor in event["motors"]:
                _print_motor_row(motor)
        else:
            print("   No pre-configured motors detected")
    elif name == "step_started":
        color = motor_color(event["expected_model"])
        print(f"\n{BOLD}{ORANGE}STEP {event['step']}/{event['total']}{RESET}")
        print(f"{ORANGE}Connect a {color}{event['expected_model']} "
              f"{BOLD}(ID {event['target_id']}){RESET}{ORANGE} to the {event['attaches_to']}{RESET}")
    elif name == "waiting_for_port":
        if event["present"]:
            print(f"\n{YELLOW}▶  RESTORE POWER:{RESET} Plug the USB cable back in.")
        else:
            print(f"\n{YELLOW}⚠  REMOVE POWER:{RESET} Unplug the USB cable from your computer.")
    elif name == "port_ready":
        print(f"{GREEN}✓ USB {'re' if event['present'] else 'dis'}connected.{RESET}")
    elif name == "awaiting_motor":
        color = motor_color(event["expected_model"])
        print(f"\n🔍 Scanning for default {color}{event['expected_model']}{RESET} motor...")
    elif name == "motor_found":
        print(f"{GREEN}✓ Found default motor{RESET}")
    elif name == "wrong_motor_detected":
        print(f"{RED}❌ {event['error']} — swap it for the correct motor.{RESET}")
    elif name == "duplicate_default_motor":
        print(f"{RED}❌ A factory-default motor still answers (at {event['baudrate']:,} bps) "
              f"after programming ID {event['target_id']}.{RESET}")
    elif name == "motor_configured":
        print(f"{GREEN}✓ Configured motor → ID={event['target_id']}, "
              f"baudrate={event['baudrate']:,}{RESET}")
        play_success_beep()
    elif name == "motor_updated":
        _print_motor_row(event["motor"])
    elif name == "motor_update_failed":
        print(f"{RED}❌ Motor {event['motor']['id']}: {event['error']}{RESET}")
    elif name == "chain_done":
        print(f"\nAll motors configured. 🚀 {ORANGE}{BOLD}Ready for operation!{RESET}\n")


def on_prompt(prompt: dict) -> None:
    """Block until the operator confirms they have connected the motor."""
    print(f"\n{ORANGE}🔌 {prompt['message']}{RESET}")
    input(f"\n   Press {BOLD}Enter{RESET} when the motor is connected correctly...")


def _load_config(config_path_arg: str | None) -> dict:
    if config_path_arg is None:
        config_path = os.path.join(get_model_path(), "config.yaml")
    else:
        config_path = os.path.abspath(config_path_arg)
        if not os.path.exists(config_path):
            raise SystemExit(f"{RED}❌ config.yaml not found: {config_path}{RESET}")
    return read_yaml(config_path)


def _resolve_port(configured_port: str, motor_type: str | None) -> str:
    port = resolve_port(configured_port, motor_type)
    if port is not None:
        if port != configured_port:
            print(f"{GREEN}✓ Using auto-detected port: {port}{RESET}")
        return port
    print(f"{YELLOW}⚠ Port {configured_port} not found.{RESET}")
    chosen = get_and_choose_port()
    if chosen and os.path.exists(chosen):
        print(f"{GREEN}✓ Using selected port: {chosen}{RESET}")
        return chosen
    raise SystemExit(f"{RED}❌ No valid port found. Check your USB connection.{RESET}")


def _resolve_motor_type(explicit: str | None, port: str) -> str:
    if explicit is not None:
        return explicit
    print(f"\n{BOLD}Auto-detecting motor family on {port}...{RESET}")
    motor_type = detect_motor_type(port, progress_callback=on_progress)
    if motor_type is None:
        raise SystemExit(
            f"\n{RED}❌ No motors responded at either family's factory defaults.{RESET}\n"
            "   Check power, wiring, and that motors are still at factory defaults\n"
            "   (re-runs of this script change them). Or pass --motor-type explicitly."
        )
    return motor_type


def _loop_until_interrupt(label: str, once) -> int:
    """Repeat a single-pass operation until the operator interrupts.

    Each pass power-cycles the bus first for motor families that need it.
    Polls quietly: the no-motors warning is printed on state change, not
    once per pass.
    """
    print(f"\n🔍 {label} — press {BOLD}Ctrl+C{RESET} to stop.")
    warned_empty = False
    try:
        while True:
            if once():
                warned_empty = False
            elif not warned_empty:
                print(f"{YELLOW}   No motors found. Check connections.{RESET}")
                warned_empty = True
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n\n{GREEN}Stopped.{RESET}\n")
    return 0


def main() -> int:
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("config_path", nargs="?", default=None,
                        help="Path to config.yaml. Defaults to the bundled model when omitted.")
    parser.add_argument("--reset", action="store_true",
                        help="Reset configured motors back to factory defaults (loops until Ctrl+C).")
    parser.add_argument("--baudrate", type=int, metavar="BAUD",
                        help="Change all motors to the given baudrate without modifying IDs "
                             "(loops until Ctrl+C).")
    parser.add_argument("--motor-type", choices=SUPPORTED_MOTOR_TYPES,
                        help="Override motor family. Auto-detected from factory defaults when omitted.")
    args = parser.parse_args()

    config = _load_config(args.config_path)
    port = _resolve_port(config.get('port', '/dev/ttyUSB0'), args.motor_type)
    motor_type = _resolve_motor_type(args.motor_type, port)
    try:
        plan = plan_motor_chain(config, port, motor_type)
    except MotorChainError as exc:
        print(f"{RED}❌ {exc}{RESET}")
        return 1
    label = motor_type.capitalize()

    if args.reset:
        _banner(f"🔄 ORCAHAND {label.upper()} CHAIN RESET 🔄")
        return _loop_until_interrupt(
            f"Resetting motors to defaults (ID={plan.default_id}, baud={plan.default_baud:,})",
            lambda: reset_all_motors(plan, progress_callback=on_progress,
                                     prompt_callback=on_prompt),
        )

    if args.baudrate is not None:
        if args.baudrate not in valid_baudrates(motor_type):
            print(f"{RED}❌ Invalid baudrate {args.baudrate:,} for {label} motors. "
                  f"Valid: {[f'{b:,}' for b in valid_baudrates(motor_type)]}{RESET}")
            return 1
        _banner(f"🔄 ORCAHAND {label.upper()} BAUDRATE CHANGE 🔄")
        return _loop_until_interrupt(
            f"Changing motor baudrates to {args.baudrate:,} bps",
            lambda: change_all_baudrates(plan, args.baudrate, progress_callback=on_progress,
                                         prompt_callback=on_prompt),
        )

    _banner(f"⚙️  ORCAHAND {label.upper()} CHAIN CONFIGURATION ⚙️")
    print(f"   • {len(plan.finger_ids)} {BLUE}{plan.finger_model}{RESET}: "
          f"{BOLD}IDs {min(plan.finger_ids)}-{max(plan.finger_ids)}{RESET}")
    if plan.wrist_id is not None:
        print(f"   • 1 {PURPLE}{plan.wrist_model}{RESET}: {BOLD}ID {plan.wrist_id}{RESET}")

    try:
        configure_motor_chain(plan, progress_callback=on_progress, prompt_callback=on_prompt)
    except MotorChainAborted as exc:
        print(f"{RED}\n🚫 {exc}{RESET}")
        return 1
    except MotorChainError as exc:
        print(f"{RED}\n❌ {exc}{RESET}")
        return 1
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{RED}❌ CONFIGURATION INTERRUPTED{RESET}")
        print(f" {YELLOW}Not all motors have been configured yet — rerun to continue.{RESET}\n")
        sys.exit(1)
