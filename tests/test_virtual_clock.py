"""The virtual clock in conftest.py must behave like real time, minus the waiting."""

import threading
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

import pytest


@pytest.fixture
def second_mock_hand(mock_config_dir):
    from orca_core import MockOrcaHand

    hand = MockOrcaHand(config_path=str(mock_config_dir / "config.yaml"))
    success, msg = hand.connect()
    assert success, f"Failed to connect mock hand: {msg}"
    try:
        yield hand
    finally:
        hand.stop_task()
        hand.disconnect()


@pytest.mark.parametrize(
    "reader, to_seconds",
    [
        ("time", 1.0),
        ("monotonic", 1.0),
        ("perf_counter", 1.0),
        ("time_ns", 1e-9),
        ("monotonic_ns", 1e-9),
        ("perf_counter_ns", 1e-9),
    ],
)
def test_every_clock_reading_includes_the_sleeps(virtual_clock, reader, to_seconds):
    read = getattr(virtual_clock, reader)
    start = read()
    virtual_clock.sleep(1000.0)
    assert (read() - start) * to_seconds == pytest.approx(1000.0, abs=1.0)


def test_a_negative_sleep_raises_like_the_real_one(virtual_clock):
    with pytest.raises(ValueError):
        virtual_clock.sleep(-0.1)


def test_a_sleep_leaves_other_threads_clocks_alone(virtual_clock):
    start = virtual_clock.monotonic()
    with ThreadPoolExecutor(max_workers=1) as pool:
        pool.submit(virtual_clock.sleep, 1000.0).result()
    assert virtual_clock.monotonic() - start < 1.0


def test_a_background_routine_cannot_cut_another_run_short(
    virtual_clock, connected_mock_hand, second_mock_hand
):
    """Two hands pacing at once: one's sleeps must not bring the other's deadline forward.

    The foreground jitter parks in its first write until the background jitter
    has paced through 1 s of its own, twice the foreground's whole duration.
    """
    foreground_parked = threading.Event()
    background_paced = threading.Event()
    writes_while_parked = 0
    background_write = second_mock_hand._motor_client.write_desired_pos
    foreground_write = connected_mock_hand._motor_client.write_desired_pos

    def counting_write(*args, **kwargs):
        nonlocal writes_while_parked
        if foreground_parked.is_set():
            writes_while_parked += 1
            if writes_while_parked == 100:
                background_paced.set()
        return background_write(*args, **kwargs)

    def interleaving_write(*args, **kwargs):
        if not foreground_parked.is_set():
            foreground_parked.set()
            assert background_paced.wait(timeout=10.0)
        return foreground_write(*args, **kwargs)

    with (
        patch.object(second_mock_hand._motor_client, "write_desired_pos", counting_write),
        patch.object(connected_mock_hand._motor_client, "write_desired_pos", interleaving_write),
    ):
        second_mock_hand.jitter(duration=1e6, amplitude=2.0, blocking=False)
        start = virtual_clock.time()
        connected_mock_hand.jitter(duration=0.5, amplitude=2.0)
        elapsed = virtual_clock.time() - start
        assert second_mock_hand.stop_task(timeout=10.0)

    assert elapsed == pytest.approx(0.5, abs=0.1)
