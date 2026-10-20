# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Progress-reporting and cancellation types shared by the maintenance routines."""

import logging
from typing import Callable, Optional

ProgressCallback = Callable[[dict], None]
ShouldStop = Callable[[], bool]


def never_stop() -> bool:
    """The ``should_stop`` default: never ask the routine to stop."""
    return False


def progress_emitter(
    log: logging.Logger, failure_message: str
) -> Callable[..., None]:
    """Build an ``emit(progress_callback, event, **payload)`` that logs callback failures to ``log``."""

    def emit(progress_callback: Optional[ProgressCallback], event: str, **payload) -> None:
        if progress_callback is None:
            return
        try:
            progress_callback({"event": event, **payload})
        except Exception:
            log.exception(failure_message)

    return emit
