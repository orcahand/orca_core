"""Provision a hand's identity (side, hardware version) onto its OH board.

Run once at assembly, and again after reflashing the board's firmware:

    uv run python scripts/provision_hand.py --side left
    uv run python scripts/provision_hand.py --side right --hw 2
"""

import argparse
import sys

from orca_core.maintenance.provisioning import ProvisioningError, provision_hand


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", required=True, choices=["left", "right"])
    parser.add_argument("--hw", type=int, default=None,
                        help="Hand hardware version (board keeps its current value if omitted)")
    parser.add_argument("--port", default=None,
                        help="OH-board serial port (autodetected if omitted)")
    args = parser.parse_args()

    try:
        info = provision_hand(
            args.side,
            hw_version=args.hw,
            port=args.port,
            progress_callback=lambda event: print(event),
        )
    except (ProvisioningError, ValueError) as e:
        print(f"Provisioning failed: {e}", file=sys.stderr)
        return 1

    print(
        f"Hand provisioned: side={info.side} hw={info.hw_version} "
        f"fw={info.fw_version} serial={info.serial}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
