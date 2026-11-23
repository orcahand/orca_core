"""One long-lived capture per camera, shared, bounded (the 2026-11-08 crash: the macOS video
decoder grew to 81 GB under the camera stream and took the machine down).

A reader thread keeps only the newest frame (grab(): the driver's queue of one, no copy, no
colour conversion); a still retrieves that frame, never by reopening the device. The capture is opened once per process and per camera and released
on close, on a read failure that needs a reopen, and at exit. `still(after=t)` waits for a
frame captured after `t` (the hand settled), so no stale buffered frame is returned.
"""
from __future__ import annotations

import atexit
import threading
import time

import numpy as np

REOPEN_AFTER = 30          # failed reads in a row before the device is reopened
REOPEN_BACKOFF_S = 2.0
WRONG_SIZE_S = 2.0         # another client (a browser at 1080p) switched the device mode: reopen at ours


class SharedCamera:
    _cams: dict[tuple[int, tuple[int, int]], "SharedCamera"] = {}
    _lock = threading.Lock()

    @classmethod
    def get(cls, index: int = 0, size: tuple[int, int] = (3840, 2160)) -> "SharedCamera":
        """The process's one capture of this camera (opened on first use)."""
        key = (int(index), tuple(size))
        with cls._lock:
            cam = cls._cams.get(key)
            if cam is None or cam._stopped:
                cam = cls._cams[key] = cls(*key)
            return cam

    def __init__(self, index: int, size: tuple[int, int]):
        self.index, self.size = index, size
        self.cap = None
        self.opens = 0
        self.frames = 0
        self._t = 0.0                      # when the newest grabbed frame arrived
        self._cond = threading.Condition()
        self._io = threading.Lock()         # grab() and retrieve() never overlap
        self._stopped = False
        self._reopen = False
        self.wrong, self.last_shape = 0, None
        self._open()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"camera-{index}")
        self._thread.start()
        atexit.register(self.close)

    def _open(self) -> None:
        import cv2
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_AVFOUNDATION)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.opens += 1

    def _release(self) -> None:
        cap, self.cap = self.cap, None
        if cap is not None:
            cap.release()

    def _run(self) -> None:
        bad = 0
        while not self._stopped:
            with self._io:
                ok = self.cap is not None and not self._reopen and self.cap.grab()
            if ok:
                bad = 0
                with self._cond:
                    self._t = time.monotonic()
                    self.frames += 1
                    self._cond.notify_all()
                continue
            bad += 1
            if (bad >= REOPEN_AFTER or self._reopen) and not self._stopped:
                self._reopen = False
                with self._io:
                    self._release()
                time.sleep(REOPEN_BACKOFF_S)
                if not self._stopped:
                    with self._io:
                        self._open()
                bad = 0
            else:
                time.sleep(0.01)

    def still(self, after: float | None = None, timeout: float = 10.0) -> np.ndarray:
        """The newest frame captured after `after` (time.monotonic(); default: now)."""
        after = time.monotonic() if after is None else after
        end = time.monotonic() + timeout
        first_wrong = None
        while True:
            with self._cond:
                while self._t <= after:
                    left = end - time.monotonic()
                    if left <= 0 or self._stopped:
                        raise RuntimeError(f"no {self.size[0]}x{self.size[1]} frame from camera {self.index} "
                                           f"within {timeout:.0f} s (last {self.last_shape}, {self.frames} frames)")
                    self._cond.wait(left)
            with self._io:
                ok, img = self.cap.retrieve() if self.cap is not None else (False, None)
            self.last_shape = None if img is None else tuple(img.shape[1::-1])
            if ok and img is not None and self.last_shape == self.size:
                return img
            self.wrong += 1
            now = time.monotonic()
            first_wrong = first_wrong or now
            if now - first_wrong >= WRONG_SIZE_S:
                self._reopen, first_wrong = True, None
            after = now                        # a wrong-sized frame (mode switch): wait for the next

    def close(self) -> None:
        """Stop the reader and release the device (idempotent)."""
        if self._stopped:
            return
        self._stopped = True
        with self._cond:
            self._cond.notify_all()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=5.0)
        with self._io:
            self._release()
        with SharedCamera._lock:
            if SharedCamera._cams.get((self.index, self.size)) is self:
                del SharedCamera._cams[(self.index, self.size)]

    def stats(self) -> dict:
        return {"index": self.index, "size": list(self.size), "opens": self.opens, "frames": self.frames,
                "wrong": self.wrong, "last_shape": self.last_shape,
                "age_s": round(time.monotonic() - self._t, 2) if self._t else None, "open": not self._stopped}
