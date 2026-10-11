"""Guided tendon tensioning: repeat centre → range check → tension until every
finger motor follows its joint through most of its travel.

See ``orca_core/maintenance/guided_tension.py`` for the protocol.
"""

import argparse
import sys
import threading

from orca_core.maintenance.guided_tension import (
    GOTO,
    HOLD_CURRENT_MA,
    PASS_FRACTION,
    TENSION_CURRENT_MA,
    read_last_tension,
    run_guided_tension,
)
from orca_core.maintenance.tensioning import TENSION_OFFSET_DEG
from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand_from_args, shutdown_hand

_COLOURS = {"pass": "\033[92m", "partial": "\033[93m", "low": "\033[91m", "unscored": "\033[90m"}
_live = threading.Event()


def _bar(motor: dict) -> str:
    fraction = motor["fraction"]
    pct = "  -" if fraction is None else f"{100 * fraction:3.0f}"
    return f"{_COLOURS[motor['status']]}{motor['joint']}:{pct}%\033[0m"


def _print_progress(event: dict) -> None:
    kind = event["event"]
    if kind == "spread" and _live.is_set():
        line = " ".join(_bar(m) for m in event["motors"])
        sys.stdout.write(f"\r\033[K{event['passed']}/{event['scored']}  {line}")
        sys.stdout.flush()
    elif kind == "score":
        print()
        for m in event["motors"]:
            print(f"  motor {m['motor']:>2}  {_bar(m)}  ({m['spread_deg']:.0f} of {m['expected_deg']} deg)")
        print(f"Score: {event['passed']}/{event['scored']} motors ({100 * event['fraction']:.0f}%)")


_round = [0]


def _prompt(prompt: dict) -> str:
    action = prompt["action"]
    options = [o for o in prompt["options"] if not o.startswith(GOTO)]
    if action == "pose":
        stage = prompt["stage"]
        if prompt["round"] != _round[0]:
            _round[0] = prompt["round"]
            print(f"\n=== Round {prompt['round']} ===")
        print(f"\n[{prompt['stage_index'] + 1}/{prompt['stage_count']}] {stage['title']} - torque is off. "
              f"{stage['pose']}")
    elif action == "range_check":
        lo, hi = min(prompt['hold_current'].values()), max(prompt['hold_current'].values())
        held = f"{lo:.0f} mA" if lo == hi else f"{lo:.0f}-{hi:.0f} mA"
        print(f"Motors hold this pose at {held}. Drive {', '.join(prompt['stage']['joints'])} "
              f"through their full range, then press Enter.")
        _live.set()
    elif action == "tension":
        print(f"Motors hold at {prompt['tension_current']:.0f} mA. Take up the slack with the "
              "ratchet tool. Tension carefully - do not overtension.")
    if len(options) == 1:
        input(f"Press Enter to {options[0].lower()}... ")
        _live.clear()
        return options[0]
    if _live.is_set():
        # Let the live spread line run until the operator is done, then show the menu below it.
        input()
        _live.clear()
        print()
    while True:
        choice = input(" / ".join(f"[{i + 1}] {o}" for i, o in enumerate(options)) + ": ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(options):
            return options[int(choice) - 1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    add_hand_arguments(parser, feedback_flag=False)
    parser.add_argument("--hold-current", type=float, default=HOLD_CURRENT_MA, help=f"Range-check hold current, mA (default {HOLD_CURRENT_MA}).")
    parser.add_argument("--tension-current", type=float, default=TENSION_CURRENT_MA,
                        help=f"Tensioning hold current, mA (default {TENSION_CURRENT_MA}).")
    parser.add_argument("--tension-offset", type=float, default=TENSION_OFFSET_DEG,
                        help=f"Degrees the finger motors turn counterclockwise while tensioning (default {TENSION_OFFSET_DEG:g}).")
    parser.add_argument("--pass-fraction", type=float, default=PASS_FRACTION,
                        help=f"Share of a joint's travel its motor must follow to pass (default {PASS_FRACTION}).")
    parser.add_argument("--any-hardware", action="store_true",
                        help="Score a hand that is not v2.1 against the v2.1 baseline anyway.")
    args = parser.parse_args()

    hand = create_hand_from_args(args, engage_feedback=False, engage_sensors=False)
    last = read_last_tension(hand.config)
    if last:
        print(f"Last successful tension: {last.get('completed_at')}")
    try:
        connect_hand(hand)
        result = run_guided_tension(
            hand,
            prompt_callback=_prompt,
            progress_callback=_print_progress,
            hold_current=args.hold_current,
            tension_current=args.tension_current,
            tension_offset_deg=args.tension_offset,
            pass_fraction=args.pass_fraction,
            any_hardware=args.any_hardware,
        )
        print("Tensioning complete. Calibrate next." if result.completed
              else "Finished below 100%.")
        return 0
    except KeyboardInterrupt:
        print("\nGuided tensioning interrupted.")
        return 1
    except ValueError as e:
        print(f"Guided tensioning cannot run: {e}")
        return 1
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
