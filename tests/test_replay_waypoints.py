import unittest
import time
from orca_core import MockOrcaHand
from tests.base_test import BaseTestCase

class TestOrcaHandReplayWaypoints(BaseTestCase):

    def setUp(self):
        super().setUp()
        WAYPOINT_START = [0.0] * len(self.hand.motor_ids)
        WAYPOINT_END = [1.0] * len(self.hand.motor_ids)
        self.waypoints = [WAYPOINT_START, WAYPOINT_END]

    def test_replay_waypoints_thread_starts_and_stops(self):
        self.hand.replay_waypoints(self.waypoints, blocking=False)
        self.assertIsNotNone(self.hand._task_thread)
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_replay_waypoints_on_finish_called(self):
        finished = []
        def on_finish():
            finished.append(True)
        self.hand.replay_waypoints(self.waypoints, on_finish=on_finish, blocking=False)
        time.sleep(0.2)
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertTrue(finished, "on_finish should have been called")

    def test_replay_waypoints_second_call_rejected(self):
        # Use a long duration and a threading.Event to keep the thread alive for the assertion
        import threading
        block_event = threading.Event()
        def blocking_on_finish():
            block_event.wait(timeout=2.0)
        self.hand.replay_waypoints(self.waypoints, duration=2.0, blocking=False, on_finish=blocking_on_finish)
        # Immediately try to start again while the thread is still running
        self.hand.replay_waypoints(self.waypoints, blocking=False)
        self.assertTrue(self.hand._task_thread.is_alive())
        # Unblock the thread and clean up
        block_event.set()
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_replay_waypoints_can_be_restarted_after_stop(self):
        self.hand.replay_waypoints(self.waypoints, blocking=False)
        self.hand.stop_task()
        time.sleep(0.1)
        self.hand.replay_waypoints(self.waypoints, blocking=False)
        self.assertIsNotNone(self.hand._task_thread)
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_on_finish_not_called_more_than_once(self):
        finished = []
        def on_finish():
            finished.append(True)
        self.hand.replay_waypoints(self.waypoints, on_finish=on_finish, blocking=False)
        self.hand.stop_task()
        time.sleep(0.1)
        self.hand.stop_task()
        self.assertEqual(len(finished), 1)

if __name__ == '__main__':
    unittest.main()
