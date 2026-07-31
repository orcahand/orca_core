from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Queue
from typing import Any


state_dir = Path.home() / ".config" / "orca_core"


class Telemetry:
    """Small event logger owned by one hand instance."""

    endpoint = "https://orcahand.com/api/telemetry"
    # Send at most this many events in one HTTP request.
    batch_size = 100
    # Keep at most this many unsent events locally; drop new events after that.
    queue_size = 2000
    # Send queued events after this delay, unless a batch fills sooner.
    post_period_s = 5

    def __init__(
        self,
        *,
        enabled: bool = True,
        sample_hz: float = 5,
    ) -> None:
        self.enabled = enabled and opted_in()
        if not self.enabled:
            return

        self.session_id = uuid.uuid4().hex
        self.install_id = install_id()
        # Limit high-frequency sampled events, such as joint telemetry, to 5 Hz by default.
        self.sample_interval_s = 1 / sample_hz
        self.last_sample: dict[str, float] = {}
        self.queue: Queue[dict[str, Any]] = Queue(maxsize=self.queue_size)
        self.thread = threading.Thread(
            target=self.send_loop,
            name="orca-core-telemetry",
            daemon=True,
        )
        self.thread.start()

    def emit(self, event_name: str, **payload: Any) -> None:
        """Queue an event. Never blocks and never raises."""
        if not self.enabled:
            return

        try:
            self.queue.put_nowait(
                {
                    "install_id": self.install_id,
                    "session_id": self.session_id,
                    "event_name": str(event_name),
                    "occurred_at": datetime.now(timezone.utc).isoformat(),
                    "payload": payload,
                }
            )
        except Exception:
            pass

    def sample(self, event_name: str, **payload: Any) -> None:
        """Like emit(), but rate-limited to 5 Hz per event name."""
        if not self.enabled:
            return

        now = time.monotonic()
        if now - self.last_sample.get(event_name, 0.0) < self.sample_interval_s:
            return
        self.last_sample[event_name] = now
        self.emit(event_name, **payload)

    def send_loop(self) -> None:
        while True:
            rows = self.drain()
            try:
                http_post(rows)
            except Exception:
                pass

    def drain(self) -> list[dict[str, Any]]:
        rows = [self.queue.get()]
        deadline = time.monotonic() + self.post_period_s
        while len(rows) < self.batch_size:
            timeout = max(0.0, deadline - time.monotonic())
            if timeout == 0.0:
                break
            try:
                rows.append(self.queue.get(timeout=timeout))
            except Empty:
                break
        return rows


def opted_in() -> bool:
    return not (state_dir / "telemetry_disabled").exists()


def disable() -> None:
    state_dir.mkdir(parents=True, exist_ok=True)
    (state_dir / "telemetry_disabled").touch()


def enable() -> None:
    (state_dir / "telemetry_disabled").unlink(missing_ok=True)


def install_id() -> str:
    try:
        value = (state_dir / "install_id").read_text(encoding="utf-8").strip()
        if value:
            return value
    except OSError:
        pass

    value = uuid.uuid4().hex
    try:
        state_dir.mkdir(parents=True, exist_ok=True)
        (state_dir / "install_id").write_text(value, encoding="utf-8")
    except OSError:
        pass
    return value


def http_post(rows: list[dict[str, Any]]) -> None:
    req = urllib.request.Request(
        Telemetry.endpoint,
        data=json.dumps({"events": rows}, separators=(",", ":")).encode("utf-8"),
        headers={"Content-Type": "application/json", "User-Agent": "orca_core"},
        method="POST",
    )
    urllib.request.urlopen(req, timeout=5).read()


def main() -> None:
    parser = argparse.ArgumentParser(prog="python -m orca_core.telemetry")
    parser.add_argument("command", choices=["enable", "disable", "status"])
    args = parser.parse_args()

    if args.command == "enable":
        enable()
        print("Telemetry enabled.")
    elif args.command == "disable":
        disable()
        print(f"Telemetry disabled. Marker: {state_dir / 'telemetry_disabled'}")
    else:
        print("enabled" if opted_in() else "disabled")


if __name__ == "__main__":
    main()
