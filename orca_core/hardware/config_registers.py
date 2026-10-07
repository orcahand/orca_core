"""Operator-editable configuration registers, declared per family.

The bench exposes a small set of settings a human changes deliberately --
which motor this is, how it talks, how it behaves. Everything else a motor
holds is either derived, measured, or tuned through a purpose-built path, and
is not here.

Each family declares what it has. A setting a family lacks is simply absent
from its table, so a front-end renders what the connected motor actually
supports instead of greying out rows against a hard-coded list.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ConfigRegister:
    """One editable register, with everything a front-end needs to render it."""

    key: str
    label: str
    address: int
    size: int                      # bytes
    eeprom: bool                   # EEPROM writes need torque off and persist
    unit: str = ""
    minimum: "int | None" = None
    maximum: "int | None" = None
    # Raw value -> what it means. Set for registers whose number is an index
    # into something (baud rate, operating mode), empty for plain numbers.
    choices: "dict[int, str]" = field(default_factory=dict)
    # Changing this costs the motor's identity on the bus, so a caller must
    # confirm and the write path has to follow the motor afterwards.
    reidentifies: bool = False
    note: str = ""

    def describe(self, raw: "int | None") -> str:
        if raw is None:
            return "--"
        if self.choices:
            return self.choices.get(raw, f"{raw} (unknown)")
        return f"{raw}{(' ' + self.unit) if self.unit else ''}"
