import os
import tempfile
import unittest

from orca_core import FileStateStore, HandState, InMemoryStateStore, load_profile


class TestStateStore(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile("orcahand_v1_right")

    def test_in_memory_round_trip(self):
        store = InMemoryStateStore()
        state = HandState.for_profile(self.profile)
        state.port = "/dev/mock"
        state.calibrated = True
        state.motor_limits[1] = [-1.0, 1.0]
        state.joint_to_motor_ratios[1] = 0.5

        store.save(self.profile, state)
        loaded = store.load(self.profile)

        self.assertEqual(loaded.port, "/dev/mock")
        self.assertTrue(loaded.calibrated)
        self.assertEqual(loaded.motor_limits[1], [-1.0, 1.0])
        self.assertEqual(loaded.joint_to_motor_ratios[1], 0.5)

    def test_file_store_writes_outside_profile(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            store = FileStateStore(temp_dir)
            state = HandState.for_profile(self.profile)
            state.port = "/dev/ttyUSB9"

            store.save(self.profile, state)

            path = store.path_for(self.profile)
            self.assertTrue(path.exists())
            self.assertIn(temp_dir, str(path))
            self.assertNotIn("models", str(path))
