import unittest
from orca_core import OrcaHand, MockOrcaHand

class TestOrcaCore(unittest.TestCase):
    def test_import_and_instantiation(self):
        try:
            hand = OrcaHand()
            self.assertIsInstance(hand, OrcaHand)
        except ImportError as e:
            self.fail(f"ImportError: {e}")
        except Exception as e:
            self.fail(f"Failed to instantiate OrcaHand: {e}")
            
    def test_mock_connection(self):
        try:
            mock_hand = MockOrcaHand()
            status = mock_hand.connect()
            self.assertTrue(status[0], "Mock connection failed")
            self.assertEqual(status[1], "Mock connection successful")
        except ImportError as e:
            self.fail(f"ImportError: {e}")
        except Exception as e:
            self.fail(f"Failed to connect MockOrcaHand: {e}")

if __name__ == "__main__":
    unittest.main()
    
    