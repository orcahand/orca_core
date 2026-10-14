#!/usr/bin/env python
"""Configure the motor chain of a fresh ORCA hand: assign each motor its ID and baud rate.

DYNAMIXEL motors also get their Return Delay Time set (DYNAMIXEL_RETURN_DELAY_TIME_US
in orca_core/constants.py) so the bus answers faster; Feetech motors keep theirs.

Plug the motors in one at a time when prompted. Re-running resumes from wherever
the previous run stopped.

Usage:
    uv run python scripts/configure_motor_chain.py [CONFIG]
    uv run python scripts/configure_motor_chain.py CONFIG --reset
    uv run python scripts/configure_motor_chain.py CONFIG --baudrate 1000000
    uv run python scripts/configure_motor_chain.py CONFIG --return-delay

--return-delay sets the Return Delay Time on every motor already on the bus, for
hands assembled before chain configuration set it (DYNAMIXEL only).
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
    set_all_return_delays,
    valid_baudrates,
)
from orca_core.maintenance.motor_chain import reset_motor_to_factory, survey_bus
from orca_core.utils import (
    enable_ansi_escapes,
    get_and_choose_port,
    get_model_path,
    read_yaml,
    serial_port_exists,
)

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
    try:
        import winsound
    except ImportError:
        pass
    else:
        winsound.Beep(800, 100)
        return
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
        how = " (from the configured chain)" if event.get("configured") else ""
        print(f"{GREEN}✓ Detected {event['motor_type']} motors{how}.{RESET}\n")
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
            print(f"\n{YELLOW}▶  RESTORE POWER:{RESET} Turn on the board.")
        else:
            print(f"\n{YELLOW}⚠  REMOVE POWER:{RESET} Turn off the board.")
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
    elif name == "still_waiting":
        checked = "; ".join(event.get("checked") or ["the factory defaults"])
        print(f"{YELLOW}   … still waiting for a factory-default {event['expected_model']} "
              f"({event['elapsed_s']}s). Nothing at the factory defaults; nothing else this run "
              f"could take (checked {checked}).{RESET}")
        if event.get("escalated"):
            print(f"{YELLOW}     Every survey is now the widest. If a motor is connected, it may "
                  f"be at an ID and baud outside what is checked: Ctrl+C and run this script "
                  f"with --scan to list everything on the bus.{RESET}")
    elif name == "survey_progress":
        found = ", ".join(f"ID {m['id']} {m['model_name']}" for m in event["motors"]) or "—"
        print(f"   {event['motor_type']:<10} @ {event['baudrate']:>9,} bps: {found}")
    elif name == "half_programmed_motor_adopted":
        print(f"{GREEN}✓ Found a motor left at the default ID but already at the target baud "
              f"(an interrupted run) — finishing it as ID {event['target_id']}.{RESET}")
    elif name == "stray_motor_reset":
        m = event["motor"]
        print(f"{YELLOW}↺ Motor ID {m['id']} ({m['model_name']} @ {m['baud_rate']:,} bps) is not "
              f"part of this chain — reset to factory defaults; it will be programmed next.{RESET}")
    elif name == "stray_motor_reset_failed":
        m = event["motor"]
        print(f"{RED}❌ Could not reset motor ID {m['id']} @ {m['baud_rate']:,} bps: "
              f"{event['error']} — unplug it.{RESET}")
    elif name == "configured_motor_missing":
        print(f"{RED}⚠ Configured motor(s) {event['missing']} stopped answering — check their "
              f"connectors and power.{RESET}")
    elif name == "chain_shortened":
        behind = (f" Motor(s) {event['stranded']} behind the gap rejoin once it closes."
                  if event["stranded"] else "")
        print(f"{YELLOW}↩ Chain is now {sorted(event['configured_ids'], reverse=True)}: "
              f"motor(s) {event['missing']} dropped off. Reconnect them, or plug a fresh motor "
              f"into that slot.{behind}{RESET}")
    elif name == "configured_motor_returned":
        print(f"{GREEN}✓ Motor {event['target_id']} answers again — carrying on.{RESET}")
    elif name == "port_lost":
        print(f"{RED}⚠ {event['port']} disappeared — check the USB cable and board power. "
              f"Waiting for it to come back…{RESET}")
    elif name == "write_retry":
        what = "ID" if event["setting"] == "id" else "baud rate"
        print(f"{YELLOW}   The motor did not take the {what} write (attempt {event['attempt']}); "
              f"retrying…{RESET}")
    elif name == "verify_retry":
        print(f"{YELLOW}   Chain check missed {event['missing']}; scanning again…{RESET}")
    elif name == "verify_failed":
        print(f"{RED}⚠ {event['error']}{RESET}")
        print(f"{YELLOW}   Re-reading the chain from the bus…{RESET}")
    elif name == "phantom_baud":
        print(f"{YELLOW}   The bus does not switch to {event['baudrate']:,} bps (the chain's own "
              f"motors {event['motor_ids']} answered there); that rate is skipped from now on.{RESET}")
    elif name == "motor_configured":
        print(f"{GREEN}✓ Configured motor → ID={event['target_id']}, "
              f"baudrate={event['baudrate']:,}{RESET}")
        play_success_beep()
    elif name == "motor_updated":
        m = event["motor"]
        print(f"{GREEN}   ↺ ID {m['id']:2d} ({m['model_name']} @ {m['baud_rate']:,} bps) → "
              f"factory defaults{RESET}")
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
    if serial_port_exists(chosen):
        print(f"{GREEN}✓ Using selected port: {chosen}{RESET}")
        return chosen
    raise SystemExit(f"{RED}❌ No valid port found. Check your USB connection.{RESET}")


def _resolve_motor_type(explicit: str | None, config: dict, port: str) -> str:
    """--motor-type wins, then whichever family answers at its factory defaults,
    then the config's pinned family.

    The bus decides: a config names the family the model was written for, not
    the one this hand was built from. A fresh chain answers at a family's
    factory defaults; a resumed one, whose motors have all moved off them,
    answers as the chain the config describes. Only a silent bus falls back to
    what the config declares.
    """
    if explicit is not None:
        return explicit
    print(f"\n{BOLD}Auto-detecting motor family on {port}...{RESET}")
    print("   (factory defaults first, then the chain this config describes)")
    detected = detect_motor_type(port, progress_callback=on_progress, chain_config=config)
    configured = config.get('motor_type')
    if detected is not None:
        if configured is not None and configured != detected:
            print(f"{YELLOW}⚠ Config declares {configured}, but {detected} motors "
                  f"answered; configuring them as {detected}.{RESET}")
        return detected
    if configured is not None:
        print(f"{GREEN}✓ No motor at a factory default; using the config's "
              f"family: {configured}{RESET}")
        return configured
    raise SystemExit(
        f"\n{RED}❌ No motors responded at either family's factory defaults, nor as the "
        f"chain this config describes.{RESET}\n"
        "   Check power and wiring. To see what is on the bus at any ID and baud, run\n"
        "   this script with --scan. Or pass --motor-type explicitly."
    )


def _config_for_family(config: dict, motor_type: str) -> dict:
    """Drop a pinned baudrate that belongs to another motor family.

    ``plan_motor_chain`` then falls back to the family's preferred rate rather
    than a rate the detected family may not even support.
    """
    configured = config.get('motor_type')
    if configured is None or configured == motor_type or 'baudrate' not in config:
        return config
    return {k: v for k, v in config.items() if k != 'baudrate'}


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


def _reset_until_interrupt(plan) -> int:
    """Reset every motor on the bus, then keep sweeping for more until Ctrl+C.

    Each pass sweeps IDs 1-40 at every rate the driver can set, so a motor
    from any hand is caught, and a motor plugged in a moment ago is noticed
    within a few seconds.
    """
    print(f"\n🔍 Resetting every motor on the bus to ID {plan.default_id} @ "
          f"{plan.default_baud:,} bps — connect more at any time; {BOLD}Ctrl+C{RESET} when done.")
    last_line = None
    try:
        while True:
            acted = reset_all_motors(plan, progress_callback=on_progress, prompt_callback=on_prompt)
            at_default = [m for m in acted
                          if m["id"] == plan.default_id and m["baud_rate"] == plan.default_baud]
            if not acted:
                line = f"{YELLOW}   No motors on the bus. Connect one (and check its power).{RESET}"
            elif len(at_default) == len(acted):
                line = (f"{GREEN}   {len(at_default)} motor(s) at factory defaults"
                        f"{' (duplicates answer as one)' if len(at_default) == 1 else ''}."
                        f" Connect the next, or Ctrl+C.{RESET}")
            else:
                line = None   # something was just reset and printed above; show the tally next pass
            if line and line != last_line:
                print(line)
            last_line = line
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"\n\n{GREEN}Stopped.{RESET}\n")
    return 0


def main() -> int:
    enable_ansi_escapes()
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("config_path", nargs="?", default=None,
                        help="Path to config.yaml. Defaults to the bundled model when omitted.")
    parser.add_argument("--reset", action="store_true",
                        help="Reset configured motors back to factory defaults (loops until Ctrl+C).")
    parser.add_argument("--baudrate", type=int, metavar="BAUD",
                        help="Change all motors to the given baudrate without modifying IDs "
                             "(loops until Ctrl+C).")
    parser.add_argument("--return-delay", action="store_true",
                        help="Set the Return Delay Time on every motor on the bus, leaving IDs "
                             "and baud alone (DYNAMIXEL; for hands assembled before chain "
                             "configuration set it).")
    parser.add_argument("--scan", action="store_true",
                        help="List every motor on the bus (both families, every rate, IDs 1-40) "
                             "and exit. A diagnostic for a bus that does not behave.")
    parser.add_argument("--motor-type", choices=SUPPORTED_MOTOR_TYPES,
                        help="Override motor family. Auto-detected from factory defaults when omitted.")
    args = parser.parse_args()

    config = _load_config(args.config_path)
    port = _resolve_port(config.get('port', 'auto'), args.motor_type)
    if args.scan:
        _banner("🔎 BUS SURVEY 🔎")
        print(f"   Pinging IDs 1-40 at every rate on {port} — about a minute.")
        found = survey_bus(port, progress_callback=on_progress)
        if not found:
            print(f"\n{YELLOW}Nothing answered. Check power and wiring.{RESET}")
            return 1
        print(f"\n{GREEN}{len(found)} motor(s) on the bus.{RESET}")
        return 0
    motor_type = _resolve_motor_type(args.motor_type, config, port)
    try:
        plan = plan_motor_chain(_config_for_family(config, motor_type), port, motor_type)
    except MotorChainError as exc:
        print(f"{RED}❌ {exc}{RESET}")
        return 1
    label = motor_type.capitalize()

    if args.reset:
        _banner(f"🔄 ORCAHAND {label.upper()} CHAIN RESET 🔄")
        return _reset_until_interrupt(plan)

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

    if args.return_delay:
        if plan.return_delay_time_us is None:
            print(f"{YELLOW}{label} motors have no return delay to set; nothing to do.{RESET}")
            return 0
        _banner(f"⏱️  ORCAHAND {label.upper()} RETURN DELAY ⏱️")
        print(f"   Setting {plan.return_delay_time_us} µs on every motor at {plan.target_baud:,} bps")
        try:
            motors = set_all_return_delays(plan, progress_callback=on_progress,
                                           prompt_callback=on_prompt)
        except MotorChainError as exc:
            print(f"{RED}❌ {exc}{RESET}")
            return 1
        if len(motors) != plan.total_motors:
            print(f"{YELLOW}⚠️  Found {len(motors)} of {plan.total_motors} motors; "
                  f"check the chain and re-run for the rest.{RESET}")
            return 1
        print(f"{GREEN}✅ Return delay set on all {len(motors)} motors.{RESET}")
        return 0

    _banner(f"⚙️  ORCAHAND {label.upper()} CHAIN CONFIGURATION ⚙️")
    print(f"   • {len(plan.finger_ids)} {BLUE}{plan.finger_model}{RESET}: "
          f"{BOLD}IDs {min(plan.finger_ids)}-{max(plan.finger_ids)}{RESET}")
    if plan.wrist_id is not None:
        print(f"   • 1 {PURPLE}{plan.wrist_model}{RESET}: {BOLD}ID {plan.wrist_id}{RESET}")

    while True:
        try:
            configure_motor_chain(plan, progress_callback=on_progress, prompt_callback=on_prompt)
            return 0
        except MotorChainAborted as exc:
            print(f"{RED}\n🚫 {exc}{RESET}")
            if not _offer_reset(plan, exc.invalid_motors):
                return 1
        except MotorChainError as exc:
            print(f"{RED}\n❌ {exc}{RESET}")
            return 1


def _offer_reset(plan, motors: list) -> bool:
    """Ask whether to reset the motors a run could not place, and do it.

    Returns True when the chain should be configured again.
    """
    if not motors:
        return False
    ids = [m["id"] for m in motors]
    answer = input(f"\n   Reset motor(s) {ids} to factory defaults now and continue? [y/N] ")
    if answer.strip().lower() not in ("y", "yes"):
        print(f"{YELLOW}   Left as they are. If a motor before them is loose, fix it and rerun.{RESET}")
        return False
    for m in motors:
        try:
            reset_motor_to_factory(plan, m["id"], m["baud_rate"])
            print(f"{GREEN}✓ Reset ID {m['id']} ({m['model_name']}) → "
                  f"ID {plan.default_id} @ {plan.default_baud:,} bps{RESET}")
        except MotorChainError as exc:
            print(f"{RED}❌ Motor {m['id']}: {exc}{RESET}")
            return False
    return True


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print(f"\n\n{RED}❌ CONFIGURATION INTERRUPTED{RESET}")
        print(f" {YELLOW}Not all motors have been configured yet — rerun to continue.{RESET}\n")
        sys.exit(1)
