"""Telling the rig's cameras apart from USB alone, on fake ioreg and system_profiler output: two
cameras of one model that report the same serial are known by their port; --assign-cameras
writes what it found into config.yaml (comments kept) and --swap trades A and B."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

from visual_calibration import camera_assign as ca
from visual_calibration.rig_cameras import CameraIdentityError, RigCameras, match_cameras
from visual_calibration.rig_config import RigConfig, get_rig_path
from visual_calibration.uvc import Uvcc, find_device, usb_devices

TWIN_VID, TWIN_PID, TWIN_SERIAL = 0x0C45, 0x6361, "SN00000000"      # two units, one serial
BRIO_VID, BRIO_PID = 0x046D, 0x085E
HUB, TWIN1, TWIN2, BOARD, BRIO = 0x01100000, 0x01110000, 0x01120000, 0x01130000, 0x02100000


def _device(name, location, vid, pid, address, speed, serial=None):
    props = [f'"idVendor" = {vid}', f'"idProduct" = {pid}', f'"locationID" = {location}',
             f'"USB Address" = {address}', f'"Device Speed" = {speed}', f'"USB Product Name" = "{name}"']
    if serial is not None:
        props.insert(2, f'"USB Serial Number" = "{serial}"')
    return [f"  | +-o {name}@{location:08x}  <class IOUSBHostDevice, id 0x100000a{address:02x}, registered, matched, "
            "active, busy 0 (1 ms), retain 20>", "  | | {", *(f"  | |   {p}" for p in props), "  | | }", "  | |"]


def ioreg_usb(plugged):
    lines = ["+-o Root  <class IORegistryEntry, id 0x100000100, retain 35>",
             "  +-o AppleT8142USBXHCI@01000000  <class AppleT8142USBXHCI, id 0x10000046a, registered, matched, "
             "active, busy 0 (9031 ms), retain 134>", "  | {", '  |   "locationID" = 16777216', "  | }",
             *_device("USB2.0 Hub", HUB, 0x05E3, 0x0610, 1, 2)]
    for d in plugged:
        lines += _device(*d)
    return "\n".join(lines) + "\n"


def _interface(location, cls, vid, pid, n):
    return [f"+-o IOUSBHostInterface@{n}  <class IOUSBHostInterface, id 0x10000b{n:03x}, registered, matched, "
            "active, busy 0 (0 ms), retain 7>", "  {", f'    "bInterfaceClass" = {cls}', f'    "idVendor" = {vid}',
            f'    "idProduct" = {pid}', f'    "locationID" = {location}', f'    "bInterfaceNumber" = {n}', "  }",
            "  +-o AppleUSBDriver  <class IOService, id 0x10000c000, registered, matched, active, busy 0, retain 5>",
            "      {", '        "locationID" = 999', "      }"]


def ioreg_interfaces(plugged):
    lines = []
    for name, location, vid, pid, *_ in plugged:
        if "Camera" in name or "BRIO" in name:
            lines += _interface(location, 14, vid, pid, 0) + _interface(location, 1, vid, pid, 2)   # video, audio
        else:
            lines += _interface(location, 2, vid, pid, 0)                                           # a CDC board
    return "\n".join(lines) + "\n"


def profiler(plugged):
    cams = [{"_name": "MacBook Camera", "spcamera_unique-id": "47B4B64B70674B9CAD2BAE273A71F4B5"}]
    cams += [{"_name": n, "spcamera_unique-id": f"0x{loc:x}{vid:04x}{pid:04x}"}
             for n, loc, vid, pid, *_ in plugged if "Camera" in n or "BRIO" in n]
    return json.dumps({"SPCameraDataType": cams})


TWINS = [("USB 2.0 Camera", TWIN2, TWIN_VID, TWIN_PID, 5, 2, TWIN_SERIAL),
         ("USB 2.0 Camera", TWIN1, TWIN_VID, TWIN_PID, 4, 2, TWIN_SERIAL)]
OH_BOARD = ("ORCA Hand", BOARD, 0x2F5D, 0x2202, 6, 1, "0123456789ABCDEF")
A_BRIO = ("Logitech BRIO", BRIO, BRIO_VID, BRIO_PID, 7, 3, "ser-0001")


def fake_run(plugged, calls=None):
    def run(cmd, **kw):
        if calls is not None:
            calls.append(cmd)
        if cmd[:3] == ["ioreg", "-p", "IOUSB"]:
            out = ioreg_usb(plugged)
        elif cmd[:4] == ["ioreg", "-r", "-c", "IOUSBHostInterface"]:
            out = ioreg_interfaces(plugged)
        elif cmd[0] == "system_profiler":
            out = profiler(plugged)
        else:
            raise AssertionError(f"unexpected command {cmd}")
        return subprocess.CompletedProcess(cmd, 0, out, "")
    return run


@pytest.fixture
def rig(tmp_path):
    path = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), path, ignore=shutil.ignore_patterns("camera_reference*.yaml"))
    return RigConfig.load(path=str(path))


def test_ioreg_lists_the_cameras_by_port_and_link_speed():
    run = fake_run([*TWINS, OH_BOARD, A_BRIO])
    devs = usb_devices(run)
    assert [d["name"] for d in devs] == ["USB2.0 Hub", "USB 2.0 Camera", "USB 2.0 Camera", "ORCA Hand", "Logitech BRIO"]
    assert ca.video_locations(run) == {TWIN1, TWIN2, BRIO}
    cams = ca.usb_cameras(run)
    assert [(c["location"], c["serial"], c["speed"]) for c in cams] == [
        (TWIN1, TWIN_SERIAL, 2), (TWIN2, TWIN_SERIAL, 2), (BRIO, "ser-0001", 3)]
    assert [c["link"] for c in cams] == ["USB 2 (480 Mb/s)", "USB 2 (480 Mb/s)", "USB 3 (5 Gb/s)"]
    assert [c["avfoundation_index"] for c in cams] == [0, 1, 2]       # sorted unique ids; the MacBook's last
    assert "0c45:6361  serial SN00000000  location 0x01110000  USB 2" in ca.describe(cams[0])


def test_identical_serials_are_assigned_by_port_and_found_by_it(rig):
    run = fake_run([*TWINS, OH_BOARD])
    config = Path(rig.path) / "config.yaml"
    before = config.read_text()
    lines = ca.assign(rig, run=run)
    text = config.read_text()
    devices = {c: yaml.safe_load(text)["cameras"][c]["uvc"]["device"] for c in "ab"}
    assert devices == {"a": {"vendor": TWIN_VID, "product": TWIN_PID, "serial": TWIN_SERIAL, "location": TWIN1},
                       "b": {"vendor": TWIN_VID, "product": TWIN_PID, "serial": TWIN_SERIAL, "location": TWIN2}}
    assert "location: 0x01110000" in text and ca.NOTE in text
    changed = [(x, y) for x, y in zip(before.splitlines(), text.splitlines()) if x != y]
    assert len(changed) == 2 and len(before.splitlines()) == len(text.splitlines())   # only the two device lines
    assert any("same serial" in l for l in lines) and any(ca.SWAP in l for l in lines)

    usb = usb_devices(run)
    found = match_cameras(devices, usb)
    assert found["a"]["location"] == TWIN1 and found["b"]["location"] == TWIN2
    calls = []

    class P:
        returncode, stdout, stderr = 0, "{}", ""
    Uvcc.from_config({"device": devices["b"]}, run=lambda cmd, **kw: (calls.append(cmd), P())[1],
                     devices=lambda: usb).export()
    assert calls[-1][-3:] == ["--address", "5", "export"]               # the blink dims camera b's own unit

    assert ca.assign(RigConfig.load(path=rig.path), do_swap=True, run=run)[0].startswith("Swapped")
    swapped = {c: yaml.safe_load(config.read_text())["cameras"][c]["uvc"]["device"] for c in "ab"}
    assert swapped == {"a": devices["b"], "b": devices["a"]}


def test_identical_serials_without_a_location_are_refused():
    usb = usb_devices(fake_run(TWINS))
    twin = {"vendor": TWIN_VID, "product": TWIN_PID, "serial": TWIN_SERIAL}
    with pytest.raises(CameraIdentityError, match=r"cameras a, b share the serial SN00000000.*--assign-cameras"):
        match_cameras({"a": twin, "b": twin}, usb)
    with pytest.raises(CameraIdentityError, match=r"at USB location 0x01200000\) is not plugged in.*another port"):
        match_cameras({"a": {**twin, "location": TWIN1}, "b": {**twin, "location": 0x01200000}}, usb)
    one = usb_devices(fake_run(TWINS[:1]))                                      # one of the twins unplugged
    with pytest.raises(CameraIdentityError, match="the one camera at USB location 0x01120000"):
        match_cameras({"a": {**twin, "location": TWIN1}, "b": {**twin, "location": TWIN2}}, one)


def test_a_camera_with_its_own_serial_is_found_on_any_port():
    usb = usb_devices(fake_run([A_BRIO, *TWINS]))
    brio = {"vendor": BRIO_VID, "product": BRIO_PID, "serial": "ser-0001", "location": 0x03100000}   # replugged
    assert find_device(**brio, devices=usb)["location"] == BRIO
    twin = {"vendor": TWIN_VID, "product": TWIN_PID, "serial": TWIN_SERIAL}
    assert find_device(**twin, location=TWIN2, devices=usb)["address"] == 5     # a spare twin plugged in: by port
    found = match_cameras({"a": brio, "b": {**twin, "location": TWIN2}}, usb)
    assert found["a"]["location"] == BRIO and found["b"]["location"] == TWIN2
    with pytest.raises(CameraIdentityError, match="reports the serial ser-0001"):
        match_cameras({"a": {**brio, "serial": None, "location": BRIO}}, usb)


def test_more_cameras_than_the_rig_are_picked_by_number(rig):
    run = fake_run([*TWINS, A_BRIO])
    with pytest.raises(ca.AssignError, match=r"(?s)1  USB 2.0 Camera.*3  Logitech BRIO.*--assign-cameras 1 2"):
        ca.assign(rig, run=run)
    ca.assign(rig, [3, 2], run=run)
    raw = yaml.safe_load((Path(rig.path) / "config.yaml").read_text())
    assert raw["cameras"]["a"]["uvc"]["device"] == {"vendor": BRIO_VID, "product": BRIO_PID, "serial": "ser-0001"}
    assert raw["cameras"]["b"]["uvc"]["device"]["location"] == TWIN2           # its twin is still plugged in
    with pytest.raises(ca.AssignError, match="one listed camera per rig camera"):
        ca.assign(rig, [1], run=run)
    with pytest.raises(ca.AssignError, match="no camera 4"):
        ca.assign(rig, [4, 1], run=run)


def test_no_cameras_writes_nothing(rig):
    config = Path(rig.path) / "config.yaml"
    before = config.read_text()
    with pytest.raises(ca.AssignError, match="(?s)none.*0 USB camera"):
        ca.assign(rig, run=fake_run([OH_BOARD]))
    assert config.read_text() == before


def test_the_assigned_cameras_pass_the_preflights_lookup(rig):
    """What --assign-cameras wrote is what RigCameras.resolve finds (fake streams, nothing opened)."""
    run = fake_run([*TWINS])
    ca.assign(rig, run=run)
    rig = RigConfig.load(path=rig.path)
    from visual_calibration.rig_cameras import avfoundation_devices
    cams = RigCameras(rig, open_camera=lambda i, size: None, uvcc=lambda uvc: None,
                      devices=lambda: usb_devices(run), av_devices=lambda: avfoundation_devices(run))
    assert cams.resolve() == {"a": 0, "b": 1}


def test_a_block_style_device_is_rewritten_in_place(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_text("cameras:\n  a:\n    uvc:\n      device:\n        vendor: 1133\n        product: 2142\n"
                      "      gain: 0   # kept\n  b:\n    uvc:\n      device: {vendor: 1, product: 2}  # old\n")
    devices = {"a": {"vendor": 3141, "product": 25441, "serial": "007", "location": TWIN1},
               "b": {"vendor": 3141, "product": 25441, "serial": None, "location": TWIN2}}
    ca.write_devices(config, devices)
    raw = yaml.safe_load(config.read_text())
    assert raw["cameras"]["a"]["uvc"] == {"device": devices["a"], "gain": 0} and raw["cameras"]["b"]["uvc"]["device"] == devices["b"]
    assert "# kept" in config.read_text() and "# old" not in config.read_text()
