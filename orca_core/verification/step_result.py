# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

"""The common face every verification step's result presents.

Steps keep rich, typed results — a sweep knows about slots, an anchor run
knows about passes — and this protocol is the narrow part a plan runner and a
report see, so neither has to know which step produced what it is holding.
"""

from __future__ import annotations

from typing import Dict, List, Mapping, Optional, Protocol, runtime_checkable


@runtime_checkable
class StepResult(Protocol):
    """What a plan runner can rely on from any step result."""

    thresholds: Dict[str, float]
    """The limit values this step applied, so a stored result stays
    re-judgeable when a limit moves."""

    messages: List[str]
    """Human-readable text for anything worth reading, each stating the
    measurement and the limit it was judged against."""

    def measurements(self) -> Dict[str, float]:
        """Every measured scalar, flat and dotted: ``<metric>.<joint>``.

        This is what the report stores and what the fleet query aggregates,
        so "distribution of ``index_mcp`` motor span across the last 50
        hands" stays one query rather than a per-step parser.
        """
        ...

    @property
    def passed(self) -> bool:
        """Whether every measurement sat inside the thresholds applied.

        A statement about the limits this step was given, nothing more.
        Whether a failure gates the hand is the plan's decision.
        """
        ...


def flat_measurements(
    metric: str, values: Mapping[str, Optional[float]]
) -> Dict[str, float]:
    """``{joint: value}`` → ``{"<metric>.<joint>": value}``.

    A value the step could not measure is left out rather than recorded as
    zero, so an absent number stays distinguishable from a measured one.
    """
    return {
        f"{metric}.{name}": float(value)
        for name, value in values.items()
        if value is not None
    }
