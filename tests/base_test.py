import unittest
import tempfile
import shutil
import os
from orca_core import MockOrcaHand

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class BaseTestCase(unittest.TestCase):
    model_name = "orcahand_v1_right"

    def __init__(self, *args, model_name=None, **kwargs):
        super().__init__(*args, **kwargs)
        if model_name is not None:
            self.model_name = model_name

    def setUp(self):
        model_dir = os.path.join(REPO_ROOT, "tests", "fixtures", "models", self.model_name)
        self.hand = MockOrcaHand(model_dir)
        
        status = self.hand.connect()
        self.assertTrue(status[0], "Mock connection failed")
        self.assertEqual(status[1], "Mock connection successful")
        
        self.assertTrue(self.hand.is_connected(), 
                       "Hand should be connected after setUp")

    def tearDown(self):
        if hasattr(self, 'hand') and self.hand:
            self.hand.disconnect()