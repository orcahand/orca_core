"""Regression tests for the joint-feedback error contract: an encoder-stream
timeout must be catchable as ``JointFeedbackConnectError`` (as the class
docstring promises), and the public error/health types must be importable
from the package root and their historical deep-import paths.
"""

from __future__ import annotations

import os
import shutil

import pytest

import orca_core
from orca_core import (
    EncodersNotAvailableError,
    JointFeedbackConnectError,
    MockOrcaHandJointFeedback,
)


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_CONFIG = os.path.join(
    REPO_ROOT, "orca_core", "models", "v2", "orcahand-joint-right", "config.yaml"
)


def test_encoder_timeout_is_a_joint_feedback_connect_error():
    assert issubclass(EncodersNotAvailableError, JointFeedbackConnectError)


def test_error_and_health_types_are_exported():
    for name in (
        "JointFeedbackConnectError",
        "EncodersNotAvailableError",
        "HandConfigValidationError",
        "LinkHealth",
    ):
        assert name in orca_core.__all__
        assert getattr(orca_core, name) is not None


def test_deep_import_paths_stay_valid():
    from orca_core.hardware.joint_encoder_client import (
        EncodersNotAvailableError as deep_enc,
    )
    from orca_core.hardware_hand_sensing import (
        JointFeedbackConnectError as deep_jf,
    )

    assert deep_enc is EncodersNotAvailableError
    assert deep_jf is JointFeedbackConnectError


def test_silent_encoder_stream_surfaces_as_connect_error(tmp_path):
    """A connect whose encoder stream never produces frames must raise a type
    catchable via the documented ``except JointFeedbackConnectError``, with
    the motor bus rolled back."""
    config_path = tmp_path / "config.yaml"
    shutil.copy(REAL_CONFIG, config_path)

    class _NoFrames(MockOrcaHandJointFeedback):
        def _create_encoder_link(self, port):
            from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink

            return MockHandSerialLink(port=port, baudrate=self.config.encoder_baudrate)

    hand = _NoFrames(config_path=str(config_path))
    with pytest.raises(JointFeedbackConnectError) as excinfo:
        hand.connect()
    assert isinstance(excinfo.value, EncodersNotAvailableError)
    assert not hand.is_connected()
    assert hand._loop is None
    assert hand._encoder_link is None
