"""Which hands this machine knows, and what they are called.

A hand's calibration is filed under the hand's identity, resolved in this
order:

1. the serial the OH board was provisioned with (``SN``), which follows the
   hand even if its board is replaced;
2. the board's own id (``BID``), folded from the MCU's factory serial, for a
   board that has not been given a serial;
3. an id the user assigned, for a hand with no board to ask.

The third case is why this module exists. Nothing can tell us which hand is in
front of us when there is no board, so the user says, and the names they have
used are remembered here: the calibrations directory *is* the register. An id
exists once a calibration is filed under it, or once it has been added ahead of
one -- adding is how a second board-less hand gets told apart from the first.

Names follow the same rule as a flashable serial, so an id a user invents and a
serial we assign can never be confused for different kinds of thing: 1-20
characters of ``[0-9A-Za-z._-]``, at least one alphanumeric.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Callable, List, Optional, Sequence

from .board_store import calibrations_dir

#: Suffix marking an id a person assigned. Board-derived names -- a serial or a
#: board id -- never get one, because nobody chose them: they are detected. So
#: the marker is what tells "this hand is called left-bench because I said so"
#: from "this file belongs to the hand whose board reports ser-0000".
MARKER_SUFFIX = ".hand"
_ID_RE = re.compile(r"^[0-9A-Za-z._-]{1,20}$")
_SUFFIXES = (".bin", ".yaml", MARKER_SUFFIX)


class HandIdError(ValueError):
    """A name that cannot be a hand id, or an id that is not there."""


def valid_id(name: str) -> bool:
    return bool(name) and bool(_ID_RE.match(name)) and any(c.isalnum() for c in name)


def check_id(name: str) -> str:
    """The name, or raise saying what is wrong with it."""
    if not valid_id(name):
        raise HandIdError(
            f"{name!r} cannot be a hand id: 1-20 characters of [0-9A-Za-z._-], "
            "at least one of them alphanumeric"
        )
    return name


def known_ids(directory: Optional[Path] = None) -> List[str]:
    """Every hand id this machine knows, sorted.

    Sorted because "the first one" has to mean something stable: with no board
    to ask, auto picks the alphabetically first.
    """
    directory = Path(directory) if directory else calibrations_dir()
    found = set()
    try:
        entries = list(directory.iterdir())
    except OSError:
        return []
    for entry in entries:
        for suffix in _SUFFIXES:
            if entry.name.endswith(suffix):
                stem = entry.name[: -len(suffix)]
                if valid_id(stem):
                    found.add(stem)
    return sorted(found)


def assigned_ids(directory: Optional[Path] = None) -> List[str]:
    """Only the names a person gave, sorted.

    What a picker offers. A serial or a board id is detected, not chosen, so
    listing it would invite someone to pick the hand they already have.
    """
    directory = Path(directory) if directory else calibrations_dir()
    try:
        entries = list(directory.iterdir())
    except OSError:
        return []
    return sorted(
        entry.name[: -len(MARKER_SUFFIX)]
        for entry in entries
        if entry.name.endswith(MARKER_SUFFIX) and valid_id(entry.name[: -len(MARKER_SUFFIX)])
    )


def files_for(name: str, directory: Optional[Path] = None) -> List[Path]:
    """Everything on this machine filed under that id."""
    directory = Path(directory) if directory else calibrations_dir()
    return [directory / f"{name}{suffix}" for suffix in _SUFFIXES
            if (directory / f"{name}{suffix}").exists()]


def add_id(name: str, directory: Optional[Path] = None) -> str:
    """Register an id a person chose, so one hand can be told from another.

    The marker is always written, even beside a calibration that already
    exists: it is what makes the name a *chosen* one, which is the only kind
    the pickers offer. Idempotent.
    """
    check_id(name)
    directory = Path(directory) if directory else calibrations_dir()
    directory.mkdir(parents=True, exist_ok=True)
    marker = directory / f"{name}{MARKER_SUFFIX}"
    if not marker.exists():
        marker.write_text(
            "# A hand this machine knows by a name somebody gave it, because it\n"
            "# has no board to name itself. Calibrating that hand writes its\n"
            "# calibration beside this file.\n",
            encoding="utf-8",
        )
    return name


def delete_id(name: str, directory: Optional[Path] = None) -> List[Path]:
    """Forget an id and everything filed under it. Returns what was removed.

    The caller confirms first -- this throws away a hand's calibration, which
    can only be got back by calibrating it again.
    """
    check_id(name)
    removed = []
    for path in files_for(name, directory):
        try:
            path.unlink()
        except OSError:
            continue
        removed.append(path)
    if not removed:
        raise HandIdError(f"this machine knows no hand called {name!r}")
    return removed


def resolve_from_board(serial: Optional[str], board_id: Optional[str]) -> Optional[str]:
    """The id a board names itself by: its serial, else its board id."""
    for candidate in (serial, board_id):
        if candidate and valid_id(candidate):
            return candidate
    return None


def auto_id(serial: Optional[str] = None, board_id: Optional[str] = None,
            directory: Optional[Path] = None) -> Optional[str]:
    """What "auto" resolves to: the board's name, else the first assigned one.

    Only assigned names are candidates for the fallback. A file keyed by some
    other hand's serial is that hand's, and picking it for a board-less one
    would hand it the wrong calibration.
    """
    from_board = resolve_from_board(serial, board_id)
    if from_board:
        return from_board
    assigned = assigned_ids(directory)
    return assigned[0] if assigned else None


#: How many answers the menu will take before it gives up. A terminal user gets
#: plenty of tries; a caller whose answerer keeps returning something the menu
#: cannot use ends the menu instead of spinning in it.
MAX_ATTEMPTS = 12


def choose_id(
    *,
    ask: Callable[[str], str] = input,
    directory: Optional[Path] = None,
    show: Callable[[str], None] = print,
    confirm: Optional[Callable[[str], bool]] = None,
    known: Optional[Sequence[str]] = None,
    max_attempts: int = MAX_ATTEMPTS,
) -> Optional[str]:
    """Ask which hand this is, in the same shape as the port picker.

    Numbered list of the ids this machine knows, plus ``a`` to add one and
    ``d`` to forget one; ``q`` gives up. Deleting asks again before it does it,
    because it throws away a calibration.

    Returns the chosen id, or ``None`` if the user gives up -- or if the
    answers run out, which is what an automated caller passing a fixed answer
    looks like from in here.
    """
    def default_confirm(prompt: str) -> bool:
        return ask(f"{prompt} [y/N] > ").strip().lower() in ("y", "yes")

    confirm = confirm or default_confirm

    for _ in range(max_attempts):
        names = list(known) if known is not None else assigned_ids(directory)
        if names:
            show("Which hand is this? (number, a to add, d to delete, q to quit)")
            for index, name in enumerate(names, 1):
                show(f"{index:2d}. {name}")
        else:
            show("This machine knows no hands yet. (a to add one, q to quit)")

        try:
            answer = ask("> ").strip()
        except (EOFError, KeyboardInterrupt):
            return None

        lowered = answer.lower()
        if lowered == "q":
            return None
        if lowered == "a":
            try:
                name = check_id(ask("Name for this hand > ").strip())
            except HandIdError as exc:
                show(str(exc))
                continue
            except (EOFError, KeyboardInterrupt):
                return None
            add_id(name, directory)
            show(f"Added {name}.")
            known = None            # re-read, so the new one is listed
            continue
        if lowered == "d":
            if not names:
                show("There is nothing to delete.")
                continue
            try:
                which = ask("Number to delete > ").strip()
            except (EOFError, KeyboardInterrupt):
                return None
            if not (which.isdigit() and 1 <= int(which) <= len(names)):
                show(f"Enter a number between 1 and {len(names)}.")
                continue
            name = names[int(which) - 1]
            if not confirm(f"Forget {name} and the calibration filed under it?"):
                show("Left alone.")
                continue
            try:
                removed = delete_id(name, directory)
            except HandIdError as exc:
                show(str(exc))
                continue
            show(f"Deleted {name} ({len(removed)} file(s)).")
            known = None
            continue
        if answer.isdigit() and names and 1 <= int(answer) <= len(names):
            return names[int(answer) - 1]
        show("Enter a number from the list, or a, d or q.")
    show("Giving up on choosing a hand.")
    return None
