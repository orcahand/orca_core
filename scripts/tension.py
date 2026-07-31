

import argparse

from orca_core.utils.cli import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def _print_progress(event: dict) -> None:
    """Render tension progress events on the terminal."""
    phase = event.get("phase") if event.get("event") == "phase" else None
    if phase == "winding":
        print("Winding tendons taut...")
    elif phase == "holding":
        print("Holding motors for manual tensioning — press Ctrl+C when done.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hold the hand under tension for manual tendon setup."
    )
    add_hand_arguments(parser)
    parser.add_argument(
        "--move-motors",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run the built-in preconditioning motion before holding tension (default: True).",
    )
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.tension(move_motors=args.move_motors, progress_callback=_print_progress)
        return 0
    except KeyboardInterrupt:
        print("\nTension interrupted.")
        return 0
    finally:
        shutdown_hand(hand)

if __name__ == "__main__":
    raise SystemExit(main())
