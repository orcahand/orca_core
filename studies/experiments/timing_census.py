"""Records the encoder stream and characterises its delivery.

Commands nothing and never enables torque: it opens the sensing link, listens,
and writes down what arrived. That makes it the one measurement that can run
before anything else is trusted, and the one that decides whether the rest are
worth running — a stream that cannot resolve the motion being looked for makes
every later number an artefact of its own sampling.

Interaction-free, like the routines in ``orca_core.maintenance``: progress goes
out through ``progress_callback`` and cancellation comes in through
``should_stop``, so a terminal and a GUI drive it identically.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from orca_core.hardware.hand_serial_link import HandSerialLink
from orca_core.hardware.tactile_client import TactileClient
from orca_core.hardware.sensing.constants import DEFAULT_ENCODER_BAUDRATE
from orca_core.hardware.sensing.serial_discovery import (
    discover_sensing_ports,
    oh_board_ports,
    port_in_use,
    probe_orca_info,
)

from ..preconditions import PreconditionError, require_healthy_encoders
from ..record import FrameRecorder, RecordingJointEncoderClient, RecordingSession
from ..record.linkstats import LinkStatsRecorder
from ..record.metadata import collect_metadata


logger = logging.getLogger(__name__)

ProgressCallback = Callable[[Dict[str, Any]], None]
ShouldStop = Callable[[], bool]

LINK_SAMPLE_INTERVAL_S = 1.0
PROGRESS_INTERVAL_S = 5.0
DEFAULT_DURATION_S = 120.0
FIRST_FRAME_TIMEOUT_S = 2.0


def _emit(callback: Optional[ProgressCallback], event: str, **payload) -> None:
    """Fire a progress event. A misbehaving callback must not abort the run."""
    if callback is None:
        return
    try:
        callback({"event": event, **payload})
    except Exception:
        logger.exception("timing census progress callback failed")


def run_timing_census(
    *,
    output_dir: Path | str,
    port: Optional[str] = None,
    baudrate: int = DEFAULT_ENCODER_BAUDRATE,
    duration_s: float = DEFAULT_DURATION_S,
    label: str = "loop_off",
    tactile_mode: str = "off",
    check_health: bool = True,
    progress_callback: Optional[ProgressCallback] = None,
    should_stop: Optional[ShouldStop] = None,
) -> Optional[Path]:
    """Record the encoder stream for ``duration_s`` and return the dataset path.

    Returns ``None`` if ``should_stop`` fired before any data was collected. The
    link is closed on every exit path; nothing else is touched, because nothing
    else was opened.

    Args:
        output_dir: Parent directory; a run-named subdirectory is created.
        port: Encoder port. Discovered when omitted.
        baudrate: Link baud.
        duration_s: How long to listen.
        label: Recorded in the manifest to distinguish otherwise identical runs.
        tactile_mode: ``"off"``, ``"resultant"`` (what a hand runs by default)
            or ``"taxels"`` (the heaviest payload the link carries). Anything
            but ``"off"`` puts a second stream on the link and measures the
            encoder stream under it.
        check_health: Refuse to start while a slot is reporting a fault.
        progress_callback: Receives ``census_started``, ``port_resolved``,
            ``board_identified``, ``health_checked``, ``recording``,
            ``progress``, ``census_done``, ``census_aborted``.
        should_stop: Polled while recording; return ``True`` to stop early and
            keep what was collected.
    """
    if should_stop is None:
        should_stop = lambda: False  # noqa: E731

    _emit(progress_callback, "census_started", duration_s=duration_s, label=label)

    port = _resolve_port(port)
    _emit(progress_callback, "port_resolved", port=port)

    # Identity first: the probe opens the port exclusively, so it cannot run
    # once the link holds it.
    board = _identify(port)
    _emit(progress_callback, "board_identified", board=board)

    link = HandSerialLink(port, baudrate=baudrate)
    recorder = FrameRecorder()
    client = RecordingJointEncoderClient(link, recorder)
    tactile = None

    try:
        link.connect()
        client.connect()
        client.start_stream(timeout=FIRST_FRAME_TIMEOUT_S)

        if tactile_mode != "off":
            tactile = _start_tactile(link, port, tactile_mode)
            _emit(
                progress_callback,
                "tactile_started",
                mode=tactile_mode,
                port_shared=True,
            )

        if check_health:
            report = require_healthy_encoders(client)
            _emit(
                progress_callback,
                "health_checked",
                frames=report.frames_sampled,
                constant_slots=[s.joint for s in report.constant_slots],
            )
        else:
            report = None

        link_recorder = LinkStatsRecorder(link, client)
        metadata = collect_metadata(
            None,
            "timing_census",
            {
                "duration_s": duration_s,
                "label": label,
                "port": port,
                "baudrate": baudrate,
                "tactile_mode": tactile_mode,
            },
            moves_hand=False,
        )
        metadata["board"] = board
        if report is not None:
            metadata["encoder_health"] = report.as_dict()

        directory = Path(output_dir) / metadata["run_id"]
        with RecordingSession(
            directory, experiment="timing_census", metadata=metadata
        ) as session:
            session.add_table(
                "frames", recorder.ring, recorder.columns, source=recorder
            )
            session.add_table(
                "link", link_recorder.ring, link_recorder.columns, source=link_recorder
            )
            _emit(progress_callback, "recording", directory=str(directory))

            stopped = _listen(
                link_recorder,
                recorder,
                duration_s=duration_s,
                should_stop=should_stop,
                progress_callback=progress_callback,
            )
            if stopped:
                session.record_event("stopped_early", reason="should_stop")

        _emit(
            progress_callback,
            "census_done",
            directory=str(directory),
            frames=recorder.frames_recorded,
            dropped=recorder.dropped,
            stale=recorder.stale,
        )
        return directory
    except PreconditionError as error:
        _emit(progress_callback, "census_aborted", error=str(error))
        raise
    finally:
        if tactile is not None:
            try:
                tactile.stop_stream()
                tactile.disconnect()
            except Exception:
                logger.exception("failed to stop the tactile stream")
        # The link owns the port exclusively; leaving it open would block the
        # next run with a message about the port being busy.
        try:
            client.disconnect()
        except Exception:
            logger.exception("failed to disconnect the encoder client")
        try:
            link.disconnect()
        except Exception:
            logger.exception("failed to disconnect the serial link")


def _listen(
    link_recorder: LinkStatsRecorder,
    recorder: FrameRecorder,
    *,
    duration_s: float,
    should_stop: ShouldStop,
    progress_callback: Optional[ProgressCallback],
) -> bool:
    """Sample link counters and report progress until the time is up.

    Frames arrive on the demuxer thread; this one only samples the counters the
    recorder cannot see and watches for a stop request.
    """
    start = time.monotonic()
    end = start + duration_s
    next_link = start
    next_progress = start + PROGRESS_INTERVAL_S

    while True:
        now = time.monotonic()
        if now >= next_link:
            link_recorder.sample(now)
            next_link += LINK_SAMPLE_INTERVAL_S
        if now >= next_progress:
            elapsed = now - start
            _emit(
                progress_callback,
                "progress",
                elapsed_s=round(elapsed, 1),
                remaining_s=round(max(0.0, end - now), 1),
                frames=recorder.frames_recorded,
                rate_hz=(
                    round(recorder.frames_recorded / elapsed, 1) if elapsed > 0 else 0.0
                ),
            )
            next_progress += PROGRESS_INTERVAL_S
        if should_stop():
            link_recorder.sample(time.monotonic())
            return True
        if now >= end:
            link_recorder.sample(time.monotonic())
            return False
        time.sleep(0.02)


def _start_tactile(link: HandSerialLink, port: str, mode: str):
    """Put a tactile stream on the link alongside the encoder stream.

    Only meaningful when both share a port, which is what makes them compete;
    on a hand with a dedicated tactile adapter the encoder link is untouched
    and the comparison would show nothing.
    """
    if mode not in ("resultant", "taxels"):
        raise ValueError(f"unknown tactile mode {mode!r}")

    ports = discover_sensing_ports()
    if ports.tactile != port:
        raise PreconditionError(
            f"tactile is not on the encoder port ({ports.tactile or 'none found'} "
            f"vs {port}), so a tactile stream would not contend with the encoder "
            "stream. Run with tactile off."
        )

    client = TactileClient(link)
    client.connect()
    client.start_stream(resultant=True, taxels=(mode == "taxels"))
    return client


def _resolve_port(port: Optional[str]) -> str:
    if port is not None:
        if port_in_use(port):
            raise PreconditionError(
                f"{port} is held by another process. Close whatever is using the "
                "hand and retry."
            )
        return port

    ports = discover_sensing_ports()
    if ports.encoder is None:
        # Every probe opens exclusively, so a port another process holds goes
        # silent in exactly the way an absent board does. Saying "not found"
        # would send someone to check the cable while the real cause is that
        # something else has the hand.
        busy = [port for port in oh_board_ports() if port_in_use(port)]
        if busy:
            raise PreconditionError(
                f"no encoder port could be probed: {', '.join(busy)} "
                f"{'is' if len(busy) == 1 else 'are'} held by another process. "
                "Close whatever is using the hand and retry."
            )
        raise PreconditionError(
            "no encoder port found. Check the hand is connected and powered, or "
            "pass the port explicitly."
        )
    if port_in_use(ports.encoder):
        raise PreconditionError(
            f"the encoder port {ports.encoder} is held by another process. Close "
            "whatever is using the hand and retry."
        )
    return ports.encoder


def _identify(port: str) -> Optional[Dict[str, Any]]:
    """Board identity, or ``None`` when it does not answer.

    Recorded rather than required: a board with older firmware stays silent and
    that is not a reason to refuse to measure it.
    """
    try:
        info = probe_orca_info(port)
    except Exception:
        logger.exception("board identity probe failed")
        return None
    if info is None:
        return None
    return {
        field: getattr(info, field)
        for field in getattr(info, "__dataclass_fields__", {})
    }
