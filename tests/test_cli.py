import unittest
from unittest.mock import MagicMock, patch

from orca_core.cli import main


class TestCli(unittest.TestCase):
    def test_doctor_command(self):
        exit_code = main(["doctor", "--profile", "orcahand_v1_right"])
        self.assertEqual(exit_code, 0)

    @patch("orca_core.cli._build_hand")
    def test_calibrate_command(self, mock_build_hand):
        hand = MagicMock()
        hand.connect.return_value = (True, "connected")
        mock_build_hand.return_value = hand

        exit_code = main(["calibrate", "--profile", "orcahand_v1_right"])

        self.assertEqual(exit_code, 0)
        hand.calibrate.assert_called_once()
        hand.disconnect.assert_called_once()
