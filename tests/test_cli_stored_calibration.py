"""Scripts that work from a stored calibration find it in the hand's folder."""

import os

import pytest

import orca_core
from orca_core import OrcaHand, board_sync
from orca_core.hardware.sensing.serial_discovery import OrcaBoardInfo
from orca_core.utils import cli
from tests._helpers import copy_model_config

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right", "config.yaml")


@pytest.fixture
def config_path(tmp_path):
    folder = tmp_path / "orcahand-right"
    folder.mkdir()
    copy_model_config(MODEL, folder / "config.yaml")
    return str(folder / "config.yaml")


def calibrated_hand(config_path, serial):
    """A hand of this model with a stored calibration, as a session leaves it."""
    hand = OrcaHand(config_path=config_path)
    hand._settle_identity(board_sync.BoardContact(info=OrcaBoardInfo(role="motor", serial=serial)))
    limits = {m: [0.0, 1.0] for m in hand.config.motor_ids}
    import dataclasses
    hand.calibration = dataclasses.replace(hand.calibration, motor_limits_dict=limits, calibrated=True)
    hand._save_calibration(hand.calibration, include_encoder=False, start_run=True)
    return hand


def test_the_only_calibrated_hand_of_the_model_is_used(config_path):
    calibrated_hand(config_path, "ser-0001")
    fresh = OrcaHand(config_path=config_path)
    assert cli.load_stored_calibration(fresh) == "ser-0001"
    assert fresh.calibrated
    motor = fresh.config.joint_to_motor_map["index_mcp"]
    lo, hi = fresh.config.joint_roms_dict["index_mcp"]
    assert fresh.joint_to_motor_ratios_dict[motor] == pytest.approx(1.0 / (hi - lo))


def test_with_several_the_hand_must_be_named(config_path):
    calibrated_hand(config_path, "ser-0001")
    calibrated_hand(config_path, "ser-0002")
    with pytest.raises(ValueError, match="ser-0001, ser-0002"):
        cli.load_stored_calibration(OrcaHand(config_path=config_path))
    assert cli.load_stored_calibration(OrcaHand(config_path=config_path), "ser-0002") == "ser-0002"


def test_a_hand_without_a_calibration_is_refused(config_path):
    with pytest.raises(ValueError):
        cli.load_stored_calibration(OrcaHand(config_path=config_path), "ser-0009")


LEFT = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-left", "config.yaml")


def test_a_v21_hand_is_read_as_it_is_at_connect(tmp_path):
    """Its board's version, so the remapped thumb joints keep their calibration,
    and its ROM frame and stamp come with it."""
    import dataclasses

    from orca_core.board_store import BoardIdentity, pack

    folder = tmp_path / "orcahand-left"
    folder.mkdir()
    copy_model_config(LEFT, folder / "config.yaml")
    session = OrcaHand(config_path=str(folder / "config.yaml"))
    session._settle_identity(board_sync.BoardContact(
        info=OrcaBoardInfo(role="motor", serial="ser-0001", hw_version=210),
        store=pack({}, identity=BoardIdentity(side="left", hw_version=210, serial="ser-0001"))))
    assert session.hardware_version == 210
    limits = {m: [0.0, 1.0] for m in session.config.motor_ids}
    session.calibration = dataclasses.replace(
        session.calibration, motor_limits_dict=limits, calibrated=True)
    session._save_calibration(session.calibration, include_encoder=False, start_run=True)
    session.set_rom_frame("centered")

    fresh = OrcaHand(config_path=str(folder / "config.yaml"))
    assert cli.load_stored_calibration(fresh, "ser-0001") == "ser-0001"
    assert fresh.hardware_version == 210
    assert fresh.calibrated, "thumb_mcp/thumb_dip were measured under the v2.1 map"
    assert fresh.rom_frame == "centered"
    assert fresh.calibration_info["sequence"] == 1
    assert fresh.hand_id == "ser-0001"
