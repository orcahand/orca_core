# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
from importlib.metadata import PackageNotFoundError, version

from .core import MockOrcaHand, OrcaHand
from .profile import DEFAULT_PROFILE_NAME, DriverConfig, HandProfile, list_builtin_profiles, load_profile, load_profile_from_path
from .state import FileStateStore, HandState, InMemoryStateStore, StateStore

try:
    __version__ = version("orca_core")
except PackageNotFoundError:
    __version__ = "0.0.0"

__all__ = [
    "DEFAULT_PROFILE_NAME",
    "DriverConfig",
    "FileStateStore",
    "HandProfile",
    "HandState",
    "InMemoryStateStore",
    "MockOrcaHand",
    "OrcaHand",
    "StateStore",
    "list_builtin_profiles",
    "load_profile",
    "load_profile_from_path",
]
