import os
import tempfile
import unittest

from orca_core import list_builtin_profiles, load_profile, load_profile_from_path
from orca_core.utils import write_yaml


class TestProfiles(unittest.TestCase):
    def test_load_builtin_profile(self):
        profile = load_profile("orcahand_v1_right")
        self.assertEqual(profile.profile_id, "orcahand_v1_right")
        self.assertEqual(profile.driver.kind, "dynamixel")
        self.assertEqual(profile.schema_version, "1.0")

    def test_list_builtin_profiles(self):
        self.assertIn("orcahand_v1_right", list_builtin_profiles())

    def test_load_profile_from_config_path(self):
        profile = load_profile_from_path("orca_core/models/orcahand_v1_right/config.yaml")
        self.assertEqual(profile.profile_id, "orcahand_v1_right")

    def test_invalid_profile_missing_required_fields(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            write_yaml(os.path.join(temp_dir, "config.yaml"), {"profile_id": "broken"})
            with self.assertRaises(ValueError):
                load_profile_from_path(temp_dir)
