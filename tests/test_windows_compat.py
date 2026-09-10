"""Windows portability: COM-port existence, the curses-free port picker, ANSI enabling.

Windows COM ports are not files, so every ``os.path.exists(port)`` in the
package went through :func:`serial_port_exists`; these tests pin that path
by faking ``sys.platform`` and pyserial's enumeration.
"""

import builtins
import sys
from types import SimpleNamespace

import pytest
# pyserial chooses its OS backend on first import; load it before any test
# fakes sys.platform, exactly as a real process has by the time it needs it.
import serial.tools.list_ports  # noqa: F401

from orca_core.maintenance import motor_chain as mc
from orca_core.utils import utils


def com_port(device: str, vid: int = 0x2F5D):
    return SimpleNamespace(device=device, vid=vid, description="fake", manufacturer=None)


@pytest.fixture
def windows(monkeypatch):
    monkeypatch.setattr(sys, "platform", "win32")


@pytest.fixture
def no_curses(monkeypatch):
    monkeypatch.setitem(sys.modules, "curses", None)


# --- serial_port_exists -------------------------------------------------------

def test_port_exists_on_windows_matches_an_enumerated_com_port(windows, patch_comports):
    patch_comports([com_port("COM3")])
    assert utils.serial_port_exists("COM3")
    assert utils.serial_port_exists("com3")
    assert not utils.serial_port_exists("COM4")


def test_port_exists_rejects_empty(windows, patch_comports):
    patch_comports([com_port("COM3")])
    assert not utils.serial_port_exists("")
    assert not utils.serial_port_exists(None)


def test_port_exists_on_posix_is_a_file_check(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "platform", "linux")
    dev = tmp_path / "ttyACM0"
    dev.write_text("")
    assert utils.serial_port_exists(str(dev))
    assert not utils.serial_port_exists(str(tmp_path / "ttyACM1"))


# --- motor_chain port resolution ------------------------------------------------

def test_resolve_port_accepts_a_configured_com_port(windows, patch_comports):
    patch_comports([com_port("COM7", vid=0x0403)])
    assert mc.resolve_port("COM7", "dynamixel") == "COM7"


def test_resolve_port_falls_back_to_autodetected_com_port(windows, patch_comports, monkeypatch):
    patch_comports([com_port("COM7", vid=0x0403)])
    monkeypatch.setattr(mc, "auto_detect_port", lambda motor_type: "COM7")
    assert mc.resolve_port("COM1", "dynamixel") == "COM7"


def test_wait_for_port_returns_once_com_port_appears(windows, monkeypatch):
    monkeypatch.setattr(mc, "PORT_POLL_INTERVAL_S", 0)
    monkeypatch.setattr(mc, "PORT_SETTLE_S", 0)
    import serial.tools.list_ports as ltp
    scans = iter([[], [], [com_port("COM5")]])
    monkeypatch.setattr(ltp, "comports", lambda: next(scans))
    mc.wait_for_port("COM5", present=True, timeout=1.0)


def test_wait_for_port_times_out_when_com_port_never_appears(windows, patch_comports, monkeypatch):
    monkeypatch.setattr(mc, "PORT_POLL_INTERVAL_S", 0)
    patch_comports([])
    with pytest.raises(mc.MotorChainError):
        mc.wait_for_port("COM5", present=True, timeout=0.01)


# --- port picker without curses -----------------------------------------------------

def test_port_picker_falls_back_to_numbered_prompt(no_curses, patch_comports, monkeypatch, capsys):
    patch_comports([com_port("COM3"), com_port("COM4", vid=0x0403)])
    answers = iter(["x", "9", "2"])
    monkeypatch.setattr(builtins, "input", lambda prompt="": next(answers))
    assert utils.get_and_choose_port() == "COM4"
    out = capsys.readouterr().out
    assert "1. COM3" in out and "2. COM4" in out


def test_port_picker_plain_quits_on_q(no_curses, patch_comports, monkeypatch):
    patch_comports([com_port("COM3")])
    monkeypatch.setattr(builtins, "input", lambda prompt="": "q")
    assert utils.get_and_choose_port() is None


def test_port_picker_plain_with_no_ports(no_curses, patch_comports):
    patch_comports([])
    assert utils.get_and_choose_port() is None


# --- ANSI -----------------------------------------------------------------------------

def test_enable_ansi_escapes_is_a_noop_off_windows(monkeypatch):
    monkeypatch.setattr(sys, "platform", "linux")
    utils.enable_ansi_escapes()
