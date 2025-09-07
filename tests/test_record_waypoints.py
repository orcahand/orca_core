import unittest
import time
from orca_core import MockOrcaHand
from tests.test_base import BaseTestCase

class TestOrcaHandRecordWaypoints(BaseTestCase):


    def tearDown(self):
        super().tearDown()

    def test_record_waypoints_thread_starts_and_stops(self):
        buffer = []
        def mock_input_fn(prompt=None):
            time.sleep(0.3)
            return ''
        self.hand.record_waypoints(buffer, blocking=False, on_finish=None, input_fn=mock_input_fn)
        time.sleep(0.1)
        self.assertTrue(self.hand._task_thread.is_alive(), "Task thread should be alive after starting record_waypoints")
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive(), "Task thread should be stopped after stop_task()")

    def test_record_waypoints_on_finish_called(self):
        buffer = []
        finished = []
        def on_finish():
            finished.append(True)
        def mock_input_fn(prompt=None):
            time.sleep(0.05)
            return ''
        self.hand.record_waypoints(buffer, on_finish=on_finish, blocking=False, input_fn=mock_input_fn)
        time.sleep(0.2)
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertTrue(finished, "on_finish should have been called")

    def test_record_waypoints_second_call_rejected(self):
        buffer = []
        def mock_input_fn(prompt=None):
            time.sleep(0.05)
            return ''
        self.hand.record_waypoints(buffer, blocking=False, input_fn=mock_input_fn)
        # Try to start again while already running
        self.hand.record_waypoints(buffer, blocking=False, input_fn=mock_input_fn)  # Should print a warning and not start a new thread
        self.assertTrue(self.hand._task_thread.is_alive())
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_record_waypoints_empty_buffer(self):
        buffer = []
        def mock_input_fn(prompt=None):
            time.sleep(0.05)
            return ''
        self.hand.record_waypoints(buffer, blocking=False, input_fn=mock_input_fn)
        # Immediately stop before any input/capture
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertEqual(buffer, [])

if __name__ == '__main__':
    unittest.main()