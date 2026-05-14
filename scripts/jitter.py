

"""Quick tendon-seating jitter on one or more motors."""

import argparse

from common import add_hand_arguments, connect_hand, create_hand, shutdown_hand


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_hand_arguments(parser)

    parser.add_argument(
        "--motor",
        type=int,
        action="append",
        dest="motor_ids",
        default=None,
        help="Motor ID to jitter. Repeat the flag to target multiple motors.",
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--amplitude", type=float, default=5.0)
    parser.add_argument("--frequency", type=float, default=10.0)
    parser.add_argument(
        "--include-wrist",
        action="store_true",
        help="Include the wrist when no explicit motor IDs are provided.",
    )
    args = parser.parse_args()

    hand = create_hand(args.config_path, use_mock=args.mock)
    try:
        connect_hand(hand)
        hand.enable_torque()
        hand.set_control_mode("current_based_position")
        hand.set_max_current(hand.config.max_current)

        print(
            "Running jitter with "
            f"motor_ids={args.motor_ids}, amplitude={args.amplitude} deg, "
            f"frequency={args.frequency} Hz, duration={args.duration} s."
        )
        hand.jitter(
            motor_ids=args.motor_ids,
            amplitude=args.amplitude,
            frequency=args.frequency,
            duration=args.duration,
            include_wrist=args.include_wrist,
        )
        print("Jitter complete.")
        return 0
    finally:
        shutdown_hand(hand)


if __name__ == "__main__":
    raise SystemExit(main())
