"""A port somebody named is the only one tried.

Falling back to auto-detection when a named port stays silent can connect to a
different hand on the same computer, and only a stray log line says so.
"""

import os
import shutil
from argparse import Namespace

import pytest

import orca_core
import orca_core.hardware_hand as hardware_hand
from orca_core import OrcaHand, board_sync
from orca_core.utils import cli, update_yaml

MODEL = os.path.join(os.path.dirname(orca_core.__file__), "models", "v2", "orcahand-right", "config.yaml")


@pytest.fixture
def config_path(tmp_path):
    shutil.copy(MODEL, tmp_path / "config.yaml")
    return str(tmp_path / "config.yaml")


@pytest.fixture
def silent_ports(monkeypatch):
    """No motor answers anywhere; record every port tried and every fallback asked."""
    tried, asked = [], []

    def fail(self, port, base_config=None):
        tried.append(port)
        raise ConnectionError("no motor responded")

    monkeypatch.setattr(OrcaHand, "_connect_on_port", fail)
    monkeypatch.setattr(hardware_hand, "auto_detect_port",
                        lambda *a, **k: asked.append("auto") or "/dev/cu.usbmodemOTHER")
    monkeypatch.setattr(hardware_hand, "find_single_usb_serial_port", lambda: "/dev/cu.usbmodemOTHER")
    monkeypatch.setattr(hardware_hand, "get_and_choose_port",
                        lambda: asked.append("picker") or "/dev/cu.usbmodemOTHER")
    return tried, asked


@pytest.mark.parametrize("interactive", [True, False])
def test_a_port_named_in_config_yaml_fails_without_looking_elsewhere(config_path, silent_ports, interactive):
    tried, asked = silent_ports
    update_yaml(config_path, "port", "/dev/cu.usbmodemXXXX")
    ok, message = OrcaHand(config_path=config_path).connect(interactive=interactive)
    assert not ok
    assert tried == ["/dev/cu.usbmodemXXXX"]
    assert asked == []
    assert "pinned port /dev/cu.usbmodemXXXX" in message


def test_the_motor_port_flag_pins_the_same_way(config_path, silent_ports):
    tried, asked = silent_ports
    args = Namespace(config_path=config_path, mock=False, model_name=None, engage_feedback=False,
                     hardware_version=None, motor_port="/dev/cu.usbmodemXXXX")
    hand = cli.create_hand_from_args(args)
    assert hand.config.port == "/dev/cu.usbmodemXXXX"
    ok, _ = hand.connect(interactive=True)
    assert not ok
    assert tried == ["/dev/cu.usbmodemXXXX"] and asked == []


def test_the_flag_is_advertised_by_every_front_end():
    import argparse

    parser = argparse.ArgumentParser()
    cli.add_hand_arguments(parser)
    assert parser.parse_args(["--motor-port", "/dev/cu.usbmodemXXXX"]).motor_port == "/dev/cu.usbmodemXXXX"


def test_auto_may_still_offer_the_picker(config_path, silent_ports):
    tried, asked = silent_ports
    ok, _ = OrcaHand(config_path=config_path).connect(interactive=True)
    assert not ok
    assert asked == ["auto", "picker"]


def test_a_port_detection_found_may_still_offer_the_picker(config_path, silent_ports):
    tried, asked = silent_ports
    import dataclasses

    hand = OrcaHand(config_path=config_path)
    hand.config = dataclasses.replace(hand.config, port="/dev/cu.usbmodemXXXX")
    hand._board_hint = ("/dev/cu.usbmodemXXXX", board_sync.NO_BOARD)
    ok, _ = hand.connect(interactive=True)
    assert not ok
    assert asked == ["picker"]
