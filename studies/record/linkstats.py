"""Periodic sampling of the serial link's own counters.

The demuxer discards frames before any handler sees them — bad checksums,
resyncs onto a header, frames with no registered handler. A recorder attached
downstream of that is structurally blind to all of it, so a run that loses a
tenth of its frames on the wire looks identical to one that loses none. These
counters are the only place that loss is visible.
"""

from __future__ import annotations

import numpy as np

from orca_core.hardware.sensing.constants import PROTOCOL_BYTE_AUTO_ENC

from .ring import SampleRing
from .schema import LINK_STAT_COLUMNS


DEFAULT_LINK_CAPACITY = 4096


class LinkStatsRecorder:
    """Snapshots link and stream counters into rows.

    Cumulative, not per-interval: differencing is the reader's job, and a
    counter that is already a difference cannot be re-based if a sample is lost.
    """

    def __init__(self, link, encoder_client, capacity: int = DEFAULT_LINK_CAPACITY):
        self.columns = list(LINK_STAT_COLUMNS)
        self._link = link
        self._client = encoder_client
        self.ring = SampleRing(capacity, len(self.columns))
        self._row = np.empty(len(self.columns), dtype=np.float64)

    @property
    def dropped(self) -> int:
        return self.ring.dropped

    def sample(self, t: float) -> None:
        link = self._link.get_link_stats()
        stream = self._client.get_stats()
        key = PROTOCOL_BYTE_AUTO_ENC
        self._row[:] = (
            t,
            stream.frames_ok,
            stream.frames_bad_effective_length,
            link.frames_routed.get(key, 0),
            link.frames_bad_lrc.get(key, 0),
            link.frames_dropped_no_handler.get(key, 0),
            link.handler_errors.get(key, 0),
            link.bad_header_resyncs,
            link.response_queue_dropped,
            link.responses_received,
            link.responses_implausible_length,
        )
        self.ring.push(self._row)
