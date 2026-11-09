"""SharedCamera: one capture per camera, newest frame only, stills without reopening, released on close."""
import threading
import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from visual_calibration import capture


class FakeCap:
    opened, released = [], []

    def __init__(self, index, backend=None, fail_after=None):
        self.index, self.n, self.fail_after = index, 0, fail_after
        self.size = [640, 480]
        self.lock = threading.Lock()
        FakeCap.opened.append(self)

    def set(self, prop, value):
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            self.size[0] = int(value)
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            self.size[1] = int(value)
        return True

    def grab(self):
        time.sleep(0.002)
        if self.fail_after is not None and self.n >= self.fail_after:
            return False
        with self.lock:
            self.n += 1
        return True

    def retrieve(self):
        with self.lock:
            return True, np.full((self.size[1], self.size[0], 3), self.n % 256, np.uint8)

    def release(self):
        FakeCap.released.append(self)


@pytest.fixture(autouse=True)
def fake(monkeypatch):
    FakeCap.opened, FakeCap.released = [], []
    monkeypatch.setattr(cv2, "VideoCapture", FakeCap)
    yield
    for cam in list(capture.SharedCamera._cams.values()):
        cam.close()


def test_one_capture_newest_frame_and_release():
    cam = capture.SharedCamera.get(0, (64, 48))
    assert capture.SharedCamera.get(0, (64, 48)) is cam              # shared, not reopened
    a = cam.still()
    t = time.monotonic()
    b = cam.still(after=t)
    assert a.shape == (48, 64, 3) and cam.frames > 1 and len(FakeCap.opened) == 1
    assert b[0, 0, 0] != a[0, 0, 0] or cam.frames > 255                # a newer frame than the request
    cam.close()
    assert FakeCap.released == FakeCap.opened and not cam._thread.is_alive()
    with pytest.raises(RuntimeError):
        cam.still(timeout=0.1)


def test_reopens_after_read_failures(monkeypatch):
    monkeypatch.setattr(capture, "REOPEN_BACKOFF_S", 0.01)
    monkeypatch.setattr(capture, "REOPEN_AFTER", 5)
    orig = FakeCap.__init__
    monkeypatch.setattr(FakeCap, "__init__", lambda self, i, b=None: orig(self, i, b, fail_after=3 if not FakeCap.opened else None))
    cam = capture.SharedCamera.get(1, (32, 24))
    time.sleep(0.1)                                                     # the first device has stopped delivering
    cam.still(after=time.monotonic(), timeout=2.0)
    assert cam.opens == 2 and FakeCap.released[0] is FakeCap.opened[0]


def test_memory_stays_bounded():
    """Thousands of frames through the reader: nothing accumulates (one frame in the driver, none held)."""
    import tracemalloc
    cam = capture.SharedCamera.get(2, (320, 240))
    cam.still()
    tracemalloc.start()
    base = tracemalloc.get_traced_memory()[0]
    for _ in range(200):
        cam.still()
    grown = tracemalloc.get_traced_memory()[0] - base
    tracemalloc.stop()
    assert cam.frames > 200 and grown < 2 * 320 * 240 * 3


def test_reopens_when_another_client_switches_the_mode(monkeypatch):
    """A browser asking for 1080p switches the shared device: wrong-sized frames for longer than
    WRONG_SIZE_S make the reader reopen at our size."""
    monkeypatch.setattr(capture, "REOPEN_BACKOFF_S", 0.01)
    monkeypatch.setattr(capture, "WRONG_SIZE_S", 0.1)
    cam = capture.SharedCamera.get(3, (64, 48))
    cam.still()
    FakeCap.opened[0].size[:] = [32, 24]                                # the mode switch
    img = cam.still(timeout=3.0)
    assert img.shape == (48, 64, 3) and cam.opens == 2 and cam.wrong > 0
