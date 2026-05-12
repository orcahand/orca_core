"""Fixtures for ``OrcaHandJointFeedback`` lifecycle tests.

The hand under test owns a real ``JointEncoderClient`` on a
``MockHandSerialLink``; a daemon pump installed on the link feeds AA A9
frames so ``start_encoder_stream`` returns within its first-frame timeout.
"""

from __future__ import annotations

import dataclasses as dc
import threading
import time
from typing import List, Optional

import numpy as np

from orca_core.calibration import JointEncoderCal
from orca_core.hardware.mock_hand_serial_link import MockHandSerialLink
from orca_core.hardware.sensing.constants import AUTO_ENC_NUM_JOINTS
from orca_core.hardware_hand_joint_feedback import MockOrcaHandJointFeedback

from tests._encoder_helpers import make_encoder_frame


_PUMP_PERIOD_S = 0.005


class _LinkFramePump:
    """Daemon thread that re-feeds the same AA A9 frame to a mock link."""

    def __init__(self, link: MockHandSerialLink, frame: bytes):
        self._link = link
        self._frame = frame
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="LinkFramePump", daemon=True,
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._link.feed_bytes(self._frame)
            except Exception:
                return
            time.sleep(_PUMP_PERIOD_S)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)


class _PumpedMockOrcaHandJointFeedback(MockOrcaHandJointFeedback):
    """``MockOrcaHandJointFeedback`` that owns a frame pump on the encoder
    link so ``start_encoder_stream`` sees a fresh AA A9 frame within its
    first-frame timeout. Lifecycle of the pump is bound to the link.
    """

    _frame_for_pump: bytes = b""

    def _create_encoder_link(self, port: str) -> MockHandSerialLink:
        link = super()._create_encoder_link(port)
        pump = _LinkFramePump(link, type(self)._frame_for_pump)
        self._pumps.append(pump)
        pump.start()
        return link

    def disconnect(self):
        for pump in self._pumps:
            pump.stop()
        self._pumps.clear()
        return super().disconnect()


def make_calibrated_joint_feedback_hand(
    config_path: str,
    raw_counts: Optional[np.ndarray] = None,
    install_encoder_calibration: bool = True,
) -> MockOrcaHandJointFeedback:
    """Build a mock hand wired for a successful joint-feedback connect.

    The encoder serial port is forced to a fixed string so
    ``resolve_sensing_ports`` skips discovery. With
    ``install_encoder_calibration=False`` the motor calibration is
    installed but the encoder calibration block is omitted, so
    ``connect()`` raises ``JointFeedbackConnectError``.
    """
    if raw_counts is None:
        raw_counts = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)

    # Bind the per-instance pump state and the per-test frame onto a
    # one-off subclass so different fixtures inside the same test session
    # don't share state.
    frame = make_encoder_frame(raw_counts=raw_counts)

    class _Bound(_PumpedMockOrcaHandJointFeedback):
        _frame_for_pump = frame

    hand = _Bound(config_path=config_path)
    hand._pumps: List[_LinkFramePump] = []
    hand.config = dc.replace(hand.config, encoder_serial_port="/dev/ttyMOCK")

    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    encoder_cal = (
        {
            joint: JointEncoderCal(enc_at_anchor_count=0)
            for joint in hand._encoder_backed_joints()
        }
        if install_encoder_calibration
        else {}
    )
    hand.calibration = dc.replace(
        hand.calibration,
        motor_limits_dict=motor_limits,
        joint_to_motor_ratios_dict=ratios,
        joint_encoder_calibration_dict=encoder_cal,
        calibrated=True,
        wrist_calibrated=True,
    )
    return hand
