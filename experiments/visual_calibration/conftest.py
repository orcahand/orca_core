# Puts this folder on sys.path so `import visual_calibration` works in tests.
import os

import pytest


@pytest.hookimpl(trylast=True)              # after the tmp_path plugin has set up its factory
def pytest_configure(config):
    """HOME and ORCA_HOME point into the session's temporary folder before any test module is
    imported: folders a module resolves once (hand_study.paths) never reach the real ~/.orca."""
    home = config._tmp_path_factory.mktemp("home")
    os.environ["HOME"] = str(home)
    os.environ["ORCA_HOME"] = str(home / ".orca")
