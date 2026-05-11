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


def make_calibrated_joint_feedback_hand(
    config_path: str,
    raw_counts: Optional[np.ndarray] = None,
    install_encoder_calibration: bool = True,
) -> MockOrcaHandJointFeedback:
    """Build a mock hand wired for a successful joint-feedback connect.

    The encoder serial port is forced to a fixed string so
    ``resolve_sensing_ports`` skips discovery. ``_create_encoder_link`` is
    monkey-patched to attach a frame pump; ``disconnect`` is wrapped to
    stop the pump before tearing down.

    With ``install_encoder_calibration=False`` the motor calibration is
    installed but the encoder calibration block is omitted, so
    ``connect()`` raises ``OrcaHandJointFeedbackError``.
    """
    hand = MockOrcaHandJointFeedback(config_path=config_path)
    hand.config = dc.replace(hand.config, encoder_serial_port="/dev/ttyMOCK")

    motor_limits = {mid: [-0.5, 0.5] for mid in hand.config.motor_ids}
    ratios = {mid: 0.01 for mid in hand.config.motor_ids}
    encoder_cal = (
        {
            joint: JointEncoderCal(
                enc_at_anchor_count=0,
                anchor_angle_deg=0.0,
            )
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

    if raw_counts is None:
        raw_counts = np.zeros(AUTO_ENC_NUM_JOINTS, dtype=np.uint16)
    frame = make_encoder_frame(raw_counts=raw_counts)

    pumps: List[_LinkFramePump] = []
    real_create = hand._create_encoder_link

    def _patched_create_encoder_link(port: str) -> MockHandSerialLink:
        link = real_create(port)
        pump = _LinkFramePump(link, frame)
        pumps.append(pump)
        pump.start()
        return link

    hand._create_encoder_link = _patched_create_encoder_link

    real_disconnect = hand.disconnect

    def _patched_disconnect():
        for pump in pumps:
            pump.stop()
        return real_disconnect()

    hand.disconnect = _patched_disconnect
    return hand
