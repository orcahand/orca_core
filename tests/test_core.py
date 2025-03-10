import unittest
from orca_core import OrcaHand

class TestOrcaCore(unittest.TestCase):
    def test_import_and_instantiation(self):
        try:
            hand = OrcaHand()
            self.assertIsInstance(hand, OrcaHand)
        except ImportError as e:
            self.fail(f"ImportError: {e}")
        except Exception as e:
            self.fail(f"Failed to instantiate OrcaHand: {e}")

if __name__ == "__main__":
    unittest.main()