import unittest
import time
from orca_core import MockOrcaHand

class TestOrcaHandTension(unittest.TestCase):

    def setUp(self):
        """Set up a mock hand and connect."""
        self.hand = MockOrcaHand()
        success, msg = self.hand.connect()
        self.assertTrue(success, f"Failed to connect mock hand: {msg}")

    def tearDown(self):
        """Disconnect after each test."""
        self.hand.stop_task()  
        time.sleep(0.1)  
        self.hand.disconnect()

    def test_tension_move_motors_false(self):
        """Test tension function with move_motors=False."""
        self.hand.tension(move_motors=False)
        time.sleep(0.2)
        self.assertTrue(self.hand._task_thread.is_alive())
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_tension_move_motors_true(self):
        """Test tension function with move_motors=True."""
        self.hand.tension(move_motors=True)
        time.sleep(0.2)
        self.assertTrue(self.hand._task_thread.is_alive())
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

    def test_tension_interrupt_after_3_seconds(self):
        """Start tension, wait 3 seconds, then interrupt."""
        self.hand.tension(move_motors=True)
        time.sleep(3)
        self.assertTrue(self.hand._task_thread.is_alive(), "Tension task should still be running")
        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive(), "Tension task should have stopped")

    def test_second_tension_is_rejected(self):
        """Ensure second tension start is rejected while one is running."""
        self.hand.tension(move_motors=True)
        time.sleep(0.2) 

        print(self.hand.tension(move_motors=False))
        time.sleep(0.1)

        self.assertTrue(self.hand._task_thread.is_alive())
        self.assertEqual(self.hand._current_task, '_tension')

        self.hand.stop_task()
        time.sleep(0.1)
        self.assertFalse(self.hand._task_thread.is_alive())

if __name__ == '__main__':
    unittest.main()
