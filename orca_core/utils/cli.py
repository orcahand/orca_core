# ==============================================================================
# Copyright (c) 2025 ORCA Dexterity, Inc. All rights reserved.
#
# This file is part of ORCA Dexterity and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Shared CLI helpers for the operator scripts and examples."""

from argparse import ArgumentParser
from pathlib import Path

from orca_core import OrcaHand
from orca_core import MockOrcaHand



def add_hand_arguments(parser: ArgumentParser, *, mock_default: bool = False) -> None:
    parser.add_argument(
        "config_path",
        nargs="?",
        default=None,
        help="Path to config.yaml. Defaults to the bundled model when omitted.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        default=mock_default,
        help="Use MockOrcaHand instead of a physical hand.",
    )


def create_hand(config_path: str | None, *, use_mock: bool):
    hand_cls = MockOrcaHand if use_mock else OrcaHand
    return hand_cls(config_path=config_path)


def connect_hand(hand) -> None:
    success, message = hand.connect()
    print(f"connect() -> success={success}, message={message}")
    if not success:
        raise RuntimeError(message)


def shutdown_hand(hand) -> None:
    try:
        hand.stop_task()
    except Exception:
        pass
    try:
        success, message = hand.disconnect()
        print(f"disconnect() -> success={success}, message={message}")
    except Exception as exc:
        print(f"disconnect() failed: {exc}")


def prepare_output_dir(path: str | None, *, default_name: str = "replay_sequences") -> Path:
    output_dir = Path(path) if path is not None else Path.cwd() / default_name
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def resolve_input_path(path: str, *, default_dir: str = "replay_sequences") -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate

    if candidate.parent != Path("."):
        return (Path.cwd() / candidate).resolve()

    return (Path.cwd() / default_dir / candidate).resolve()
