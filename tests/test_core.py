import unittest

from orca_core import MockOrcaHand, OrcaHand

class TestOrcaCore(unittest.TestCase):
    def test_import_and_instantiation(self):
        try:
            hand = OrcaHand()
            self.assertIsInstance(hand, OrcaHand)
        except ImportError as e:
            self.fail(f"ImportError: {e}")
        except Exception as e:
            self.fail(f"Failed to instantiate OrcaHand: {e}")

    def test_orca_hand_exposes_hardware_methods(self):
        hand = OrcaHand()
        for method_name in [
            "connect",
            "disconnect",
            "enable_torque",
            "disable_torque",
            "set_control_mode",
            "set_max_current",
            "calibrate",
            "tension",
            "jitter",
        ]:
            self.assertTrue(hasattr(hand, method_name), f"Missing hardware method: {method_name}")

    def test_mock_connection(self):
        try:
            mock_hand = MockOrcaHand()
            status = mock_hand.connect()
            self.assertTrue(status[0], "Mock connection failed")
        except ImportError as e:
            self.fail(f"ImportError: {e}")
        except Exception as e:
            self.fail(f"Failed to connect MockOrcaHand: {e}")

if __name__ == "__main__":
    unittest.main()
    
    
