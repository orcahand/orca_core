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
    # What one raw register unit is worth in `unit`. A caller works entirely
    # in `unit` -- bounds, writes and reads are all in it -- so nobody has to
    # know that a return delay register counts in twos.
    scale: float = 1.0
    minimum: "int | None" = None
    maximum: "int | None" = None
    # Raw value -> what it means. Set for registers whose number is an index
    # into something (baud rate, operating mode), empty for plain numbers.
    choices: "dict[int, str]" = field(default_factory=dict)
    # Changing this costs the motor's identity on the bus, so a caller must
    # confirm and the write path has to follow the motor afterwards.
    reidentifies: bool = False
    note: str = ""

    def from_raw(self, raw: "int | None") -> "int | None":
        """Register contents as the operator sees them."""
        if raw is None:
            return None
        return int(round(raw * self.scale))

    def to_raw(self, value: int) -> int:
        """An operator's value as the register holds it.

        Rounded, because not every value is representable -- a return delay
        counts in twos, so 21 us is 20. The caller is told what actually
        stuck by reading the register back, rather than being refused here.
        """
        return int(round(value / self.scale))

    def describe(self, value: "int | None") -> str:
        """``value`` is in `unit`, not raw."""
        if value is None:
            return "--"
        if self.choices:
            return self.choices.get(value, f"{value} (unknown)")
        return f"{value}{(' ' + self.unit) if self.unit else ''}"
