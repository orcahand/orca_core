import unittest
import time
import inspect
import numpy as np
from unittest.mock import patch
from orca_core import MockOrcaHand


class TestOrcaHandJitter(unittest.TestCase):

    def setUp(self):
        self.hand = MockOrcaHand()
        success, msg = self.hand.connect()
        self.assertTrue(success, f"Failed to connect mock hand: {msg}")
        self.wrist_motor_id = self.hand.joint_to_motor_map.get("wrist")

    def tearDown(self):
        self.hand.stop_task()
        time.sleep(0.1)
        self.hand.disconnect()

    # ── Amplitude safety ──

    def test_amplitude_exceeds_max_raises_error(self):
        with self.assertRaises(ValueError):
            self.hand.jitter(amplitude=15.0, duration=0.1)

    def test_amplitude_at_max_allowed(self):
        self.hand.jitter(amplitude=10.0, duration=0.1)

    def test_amplitude_below_max(self):
        self.hand.jitter(amplitude=3.0, duration=0.1)

    # ── Default frequency ──

    def test_default_frequency_is_fast(self):
        sig = inspect.signature(self.hand.jitter)
        default_freq = sig.parameters['frequency'].default
        self.assertGreaterEqual(default_freq, 10.0)

    # ── Duration ──

    def test_duration_respected(self):
        start = time.time()
        self.hand.jitter(duration=1.0, amplitude=2.0)
        elapsed = time.time() - start
        self.assertAlmostEqual(elapsed, 1.0, delta=0.3)

    def test_short_duration(self):
        start = time.time()
        self.hand.jitter(duration=0.3, amplitude=2.0)
        elapsed = time.time() - start
        self.assertLess(elapsed, 1.0)

    # ── Motor selection ──

    def test_custom_motor_ids(self):
        target_id = self.hand.motor_ids[0]
        with patch.object(self.hand._motor_client, 'write_desired_pos', wraps=self.hand._motor_client.write_desired_pos) as mock_write:
            self.hand.jitter(motor_ids=[target_id], amplitude=5.0, duration=0.2)
            self.assertGreater(len(mock_write.call_args_list), 0)
            for call_args in mock_write.call_args_list:
                self.assertEqual(call_args[0][0], [target_id])

    # ── Default excludes wrist ──

    def test_default_excludes_wrist(self):
        self.assertIsNotNone(self.wrist_motor_id, "Config must have a wrist motor for this test")
        with patch.object(self.hand._motor_client, 'write_desired_pos', wraps=self.hand._motor_client.write_desired_pos) as mock_write:
            self.hand.jitter(amplitude=5.0, duration=0.2)
            self.assertGreater(len(mock_write.call_args_list), 0)
            for call_args in mock_write.call_args_list:
                self.assertNotIn(self.wrist_motor_id, call_args[0][0])

    def test_wrist_excluded_even_with_many_motors(self):
        non_wrist_ids = [mid for mid in self.hand.motor_ids if mid != self.wrist_motor_id]
        with patch.object(self.hand._motor_client, 'write_desired_pos', wraps=self.hand._motor_client.write_desired_pos) as mock_write:
            self.hand.jitter(amplitude=5.0, duration=0.2)
            self.assertGreater(len(mock_write.call_args_list), 0)
            commanded_ids = set(mock_write.call_args_list[0][0][0])
            self.assertEqual(commanded_ids, set(non_wrist_ids))

    # ── Wrist included with flag ──

    def test_include_wrist_flag(self):
        with patch.object(self.hand._motor_client, 'write_desired_pos', wraps=self.hand._motor_client.write_desired_pos) as mock_write:
            self.hand.jitter(amplitude=5.0, duration=0.2, include_wrist=True)
            self.assertGreater(len(mock_write.call_args_list), 0)
            commanded_ids = set(mock_write.call_args_list[0][0][0])
            self.assertIn(self.wrist_motor_id, commanded_ids)

    # ── Works before calibration (motor level) ──

    def test_works_without_calibration(self):
        uncalibrated = MockOrcaHand()
        uncalibrated.connect()
        try:
            uncalibrated.jitter(amplitude=5.0, duration=0.2)
        finally:
            uncalibrated.disconnect()

    # ── Returns to starting position ──

    def test_returns_to_start_position(self):
        positions_before = self.hand.get_motor_pos()
        self.hand.jitter(amplitude=5.0, duration=0.5)
        positions_after = self.hand.get_motor_pos()
        np.testing.assert_allclose(positions_after, positions_before, atol=1e-6)

    # ── Non-blocking + stop ──

    def test_non_blocking_runs_in_background(self):
        self.hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
        time.sleep(0.2)
        self.assertTrue(self.hand._task_thread.is_alive())
        self.hand.stop_task()
        time.sleep(0.2)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_early_stop_returns_to_start(self):
        positions_before = self.hand.get_motor_pos()
        self.hand.jitter(amplitude=5.0, duration=10.0, blocking=False)
        time.sleep(0.3)
        self.hand.stop_task()
        time.sleep(0.2)
        positions_after = self.hand.get_motor_pos()
        np.testing.assert_allclose(positions_after, positions_before, atol=1e-6)

    # ── Concurrent task rejection ──

    def test_second_jitter_rejected(self):
        self.hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
        time.sleep(0.1)
        self.hand.jitter(amplitude=5.0, duration=5.0, blocking=False)
        self.assertTrue(self.hand._task_thread.is_alive())
        self.assertEqual(self.hand._current_task, '_jitter')
        self.hand.stop_task()
        time.sleep(0.2)
        self.assertFalse(self.hand._task_thread.is_alive())


if __name__ == '__main__':
    unittest.main()
