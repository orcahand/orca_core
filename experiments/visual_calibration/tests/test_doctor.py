"""The rig doctor on fakes: its contract (name, status, detail, fix for every check), a ready
rig Mac, and one without the rig's hardware, packages or files; nothing is opened."""
import json
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from visual_calibration import doctor
from visual_calibration.rig_config import RigConfig, get_rig_path

TWIN = {"name": "Logitech BRIO", "vendor": 0x046D, "product": 0x085E, "serial": "SN00000000", "speed": 3,
        "bcd_usb": 0x320, "link": "USB 3 (5 Gb/s)"}                     # two BRIOs with one serial
CAMS = [{**TWIN, "location": 0x01110000, "address": 4}, {**TWIN, "location": 0x01120000, "address": 5}]
RIG_SERIAL = "ser-0001"
GOOD_IMPORTS = {"cv2": {"version": "4.13.0", "aruco": True}, "torch": {"version": "2.10.0", "mps": True},
                **{m: {"version": "1.0"} for m in doctor.SEGMENTER[1:] + doctor.RIG}}


def port(device, serial, vid=0x2F5D):
    return SimpleNamespace(device=device, serial_number=serial, vid=vid)


def completed(cmd, out="", code=0, err=""):
    return subprocess.CompletedProcess(cmd, code, out, err)


class Fake:
    """The computer the doctor sees."""

    def __init__(self, imports=None, uvcc="8.0.0\n", rosetta=True, branch="main"):
        self.imports, self.uvcc, self.rosetta, self.branch = imports or GOOD_IMPORTS, uvcc, rosetta, branch
        self.calls = []

    def run(self, cmd, **kw):
        self.calls.append(cmd)
        if cmd[1:2] == ["-c"]:
            return completed(cmd, "noise\n" + json.dumps(self.imports))
        if cmd[0] == "git":
            path = cmd[2]
            return completed(cmd, (self.branch if "orcahand_description" in path else
                                   doctor.FIRMWARE_BRANCH if "firmware" in path else "internal/rig-calibration") + "\n")
        if cmd[0] == "npx":
            assert "--no-install" in cmd and "--yes" not in cmd                 # never downloads
            if self.uvcc == "uncached":
                return completed(cmd, "", 1, 'npm error npx canceled due to missing packages and no YES option: ["uvcc@8.0.0"]')
            return completed(cmd, self.uvcc) if self.uvcc else completed(cmd, "", 1, "npm error network")
        if cmd[:4] == ["ioreg", "-r", "-c", "IOUSBHostInterface"]:
            return completed(cmd, "")
        if cmd[0] == "system_profiler":
            return completed(cmd, json.dumps({"SPCameraDataType": [
                {"_name": c["name"], "spcamera_unique-id": f"0x{c['location']:x}{c['vendor']:04x}{c['product']:04x}"}
                for c in CAMS]}))
        if cmd[0] == "arch":
            return completed(cmd, code=0 if self.rosetta else 1, err="Bad CPU type in executable")
        raise AssertionError(f"unexpected command {cmd}")


@pytest.fixture
def mac(tmp_path, monkeypatch):
    """Everything in place: the description, the firmware checkout, the weights, the rig with its twins."""
    desc = tmp_path / "orcahand_description"
    (desc / "v2" / "models" / "urdf").mkdir(parents=True)
    fw = tmp_path / "orca_firmware"
    (fw / "tools" / "bench").mkdir(parents=True)
    (fw / "tools" / "bench" / "cal_store.py").write_text("")
    (fw / ".arduino" / "bin").mkdir(parents=True)
    (fw / ".arduino" / "bin" / "arduino-cli").write_text("")
    hf = tmp_path / "hf"
    from hand_study.segment import BIREFNET_REVISION
    snap = hf / "hub" / "models--ZhengPeng7--BiRefNet_dynamic" / "snapshots" / BIREFNET_REVISION
    snap.mkdir(parents=True)
    for f in doctor.BIREFNET_FILES:
        (snap / f).write_bytes(b"x" * (1000 if f == "model.safetensors" else 1))
    ui = tmp_path / "orca_ui"
    (ui / "orca_ui").mkdir(parents=True)
    rig = tmp_path / "v1" / "camera-rig"
    shutil.copytree(get_rig_path(), rig)
    cfg = yaml.safe_load((rig / "config.yaml").read_text())
    for c, cam in zip("ab", CAMS):
        cfg["cameras"][c]["uvc"]["device"] = {"vendor": cam["vendor"], "product": cam["product"],
                                               "serial": cam["serial"], "location": cam["location"]}
    (rig / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    table = yaml.safe_load((rig / "turntable.yaml").read_text())
    table["board"]["usb_serial"] = RIG_SERIAL
    (rig / "turntable.yaml").write_text(yaml.safe_dump(table, sort_keys=False))
    monkeypatch.setenv("ORCAHAND_DESCRIPTION", str(desc))
    monkeypatch.setenv("ORCA_FIRMWARE_REPO", str(fw))
    monkeypatch.setenv("HF_HOME", str(hf))
    monkeypatch.setenv("ORCA_UI", str(ui))
    return SimpleNamespace(rig=rig, hf=hf, snap=snap, fw=fw, desc=desc, ui=ui)


def probe(mac, fake=None, cams=CAMS, ports=None, machine="arm64", python=(3, 12, 8), **kw):
    import os
    fake = fake or Fake()
    ports = [port("/dev/cu.usbmodem101", RIG_SERIAL), port("/dev/cu.usbmodem201", "0123456789ABCDEF"),
             port("/dev/cu.usbmodem203", "0123456789ABCDEF")] if ports is None else ports
    return doctor.Probe(**{"run": fake.run, "environ": dict(os.environ), "python": python,
                           "executable": "/venv/bin/python", "machine": machine,
                           "which": lambda c: f"/usr/local/bin/{c}", "comports": lambda: ports,
                           "usb_cameras": lambda: cams, "usb_devices": lambda: cams,
                           "rig": lambda: RigConfig.load(path=str(mac.rig)),
                           "orca_core_file": lambda: doctor.CHECKOUT / "orca_core" / "__init__.py", **kw})


def by_name(results):
    return {r["name"]: r for r in results}


def test_every_check_answers_name_status_detail_fix_in_order(mac):
    results = doctor.run(probe=probe(mac))
    assert [r["name"] for r in results] == list(doctor.CHECKS)
    assert all(set(r) == {"name", "status", "detail", "fix"} for r in results)
    assert all(r["status"] in ("ok", "warn", "error") and isinstance(r["detail"], str) for r in results)
    assert all(r["fix"] == "" for r in results if r["status"] == "ok")
    assert [r["name"] for r in doctor.run(["uvcc", "python"], probe=probe(mac))] == ["uvcc", "python"]
    with pytest.raises(ValueError, match="no check nope"):
        doctor.run(["nope"])


def test_a_ready_rig_mac_is_all_ok(mac):
    fake = Fake()
    results = by_name(doctor.run(probe=probe(mac, fake)))
    assert {n: r["status"] for n, r in results.items() if r["status"] != "ok"} == {}
    assert "serial SN00000000 shared: told apart by USB port" in results["usb_cameras"]["detail"]
    assert "port 0x01110000" in results["camera_identity"]["detail"]
    assert "/dev/cu.usbmodem101" in results["turntable_board"]["detail"]
    assert "usbmodem201" in results["hand_board"]["detail"] and "MPS" in results["segmenter_packages"]["detail"]
    imports = [c for c in fake.calls if c[1:2] == ["-c"]]
    assert len(imports) == 1 and imports[0][0] == "/venv/bin/python"          # one child, torch stays out of here


def test_without_the_rig_hardware_each_part_says_not_connected(mac):
    results = by_name(doctor.run(probe=probe(mac, cams=[], ports=[])))
    assert results["usb_cameras"]["status"] == "error" and "no USB camera" in results["usb_cameras"]["detail"]
    assert results["camera_identity"]["status"] == "warn" and "not on USB" in results["camera_identity"]["detail"]
    assert results["turntable_board"]["status"] == "error" and "not connected" in results["turntable_board"]["detail"]
    assert results["hand_board"]["status"] == "warn" and results["hand_board"]["fix"] == "plug in the hand"
    one = by_name(doctor.run(["usb_cameras"], probe=probe(mac, cams=CAMS[:1])))["usb_cameras"]
    assert one["status"] == "error" and "the rig has 2" in one["detail"]


def test_twins_without_a_port_need_assigning(mac):
    cfg = yaml.safe_load((mac.rig / "config.yaml").read_text())
    for c in "ab":
        cfg["cameras"][c]["uvc"]["device"].pop("location")
    (mac.rig / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    r = by_name(doctor.run(["camera_identity"], probe=probe(mac)))["camera_identity"]
    assert r["status"] == "error" and "share a serial" in r["detail"]
    assert r["fix"].endswith("-m visual_calibration.sweep --assign-cameras")
    for c in "ab":
        cfg["cameras"][c]["uvc"]["device"] = {"vendor": 1133, "product": 2142, "serial": None}
    (mac.rig / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    r = by_name(doctor.run(["camera_identity"], probe=probe(mac)))["camera_identity"]
    assert r["status"] == "error" and "no serial" in r["detail"]


def test_cameras_config_yaml_is_not_tuned_for_are_an_error(mac):
    cfg = yaml.safe_load((mac.rig / "config.yaml").read_text())
    for c in "ab":
        cfg["cameras"][c]["uvc"]["device"].update(vendor=0x0C45, product=0x6361)
    (mac.rig / "config.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    r = by_name(doctor.run(["camera_identity"], probe=probe(mac)))["camera_identity"]
    assert r["status"] == "error" and r["detail"] == (
        "config.yaml is tuned for Logitech BRIO; these cameras (0c45:6361) are untested: resolution and UVC "
        "controls must be adapted")
    assert "cameras.<c>.uvc" in r["fix"]


def test_a_usb3_camera_on_a_usb2_link_is_a_warning(mac):
    brio = {"name": "Logitech BRIO", "vendor": 0x046D, "product": 0x085E, "bcd_usb": 0x320}
    cams = [{**brio, "serial": "ser-0002", "location": 0x01100000, "speed": 2, "link": "USB 2 (480 Mb/s)"},
            {**brio, "serial": "ser-0003", "location": 0x02100000, "speed": 3, "link": "USB 3 (5 Gb/s)"}]
    r = by_name(doctor.run(["usb_cameras"], probe=probe(mac, cams=cams)))["usb_cameras"]
    assert r["status"] == "warn" and "Logitech BRIO at 0x01100000 on a USB 2 link" in r["detail"]
    assert "ser-0003" in r["detail"] and "at 0x02100000" not in r["detail"]           # the one on USB 3 is fine


def test_missing_packages_weights_and_tools_name_their_fix(mac):
    imports = {**GOOD_IMPORTS, "cv2": {"version": "4.13.0", "aruco": False},
               "transformers": {"error": "ModuleNotFoundError: No module named 'transformers'"}}
    other = mac.snap.parent / ("0" * 40)                               # another revision is not the tested one
    mac.snap.rename(other)
    p = probe(mac, Fake(imports=imports, uvcc="", rosetta=False, branch="v1-fix"), python=(3, 10, 14))
    r = by_name(doctor.run(probe=p))
    assert r["python"]["status"] == "error" and r["python"]["fix"].startswith(str(doctor.EXPERIMENT / "setup_rig.sh"))
    assert "--reinstall-package opencv-contrib-python-headless" in r["opencv"]["fix"]
    assert r["opencv"]["fix"].startswith("uv pip uninstall --python /venv/bin/python opencv-python ")
    assert r["opencv"]["status"] == "error" and "without aruco" in r["opencv"]["detail"]
    seg = r["segmenter_packages"]
    assert seg["status"] == "error" and "transformers: ModuleNotFoundError" in seg["detail"]
    assert seg["fix"] == (f"uv pip install --python /venv/bin/python -r {doctor.CHECKOUT / 'pyproject.toml'} --extra rig")
    w = r["birefnet_weights"]
    assert w["status"] == "error" and "revision 280306042f57 is not in the Hugging Face cache" in w["detail"]
    assert "BiRefNet()" in w["fix"]
    assert r["uvcc"]["status"] == "error" and "npm error network" in r["uvcc"]["detail"]
    assert r["rosetta"]["status"] == "warn" and r["rosetta"]["fix"].startswith("softwareupdate --install-rosetta")
    assert r["orcahand_description"]["status"] == "warn" and "switch main" in r["orcahand_description"]["fix"]
    assert by_name(doctor.run(["rosetta"], probe=probe(mac, machine="x86_64")))["rosetta"]["status"] == "ok"
    u = by_name(doctor.run(["uvcc"], probe=probe(mac, Fake(uvcc="uncached"))))["uvcc"]
    assert u["status"] == "warn" and "not in npx's cache" in u["detail"] and "setup_rig.sh" in u["fix"]


def test_run_imports_is_every_module_the_child_checks(mac):
    fake = Fake()
    doctor.run(["opencv"], probe=probe(mac, fake))
    child = next(c for c in fake.calls if c[1:2] == ["-c"])
    assert tuple(child[3:]) == doctor.RUN_IMPORTS
    assert {"cv2", "torch", "transformers", "timm", "kornia", "einops", "huggingface_hub"} <= set(doctor.RUN_IMPORTS)


def test_the_firmware_checkout_is_optional_and_its_gaps_are_warnings(mac, monkeypatch):
    assert by_name(doctor.run(["firmware"], probe=probe(mac)))["firmware"]["status"] == "ok"
    (mac.fw / ".arduino" / "bin" / "arduino-cli").unlink()
    r = by_name(doctor.run(["firmware"], probe=probe(mac)))["firmware"]
    assert r["status"] == "warn" and r["fix"] == f"cd {mac.fw} && ./setup.sh"
    monkeypatch.setenv("ORCA_FIRMWARE_REPO", str(mac.fw / "nowhere"))
    r = by_name(doctor.run(["firmware"], probe=probe(mac)))["firmware"]
    assert r["status"] == "warn" and "only needed to write a hand's board" in r["detail"]


def test_orca_core_from_elsewhere_is_an_error(mac):
    p = probe(mac, orca_core_file=lambda: "/somewhere/site-packages/orca_core/__init__.py")
    r = by_name(doctor.run(["orca_core"], probe=p))["orca_core"]
    assert r["status"] == "error" and "not this checkout" in r["detail"]
    assert r["fix"].startswith(f"cd {mac.ui} && ./dev local {doctor.CHECKOUT}")            # ORCA_UI honoured


def test_orca_ui_on_another_branch_or_missing_is_a_warning(mac, monkeypatch):
    assert by_name(doctor.run(["orca_ui"], probe=probe(mac)))["orca_ui"]["status"] == "ok"
    r = by_name(doctor.run(["orca_ui", "orcahand_description"], probe=probe(mac, Fake(branch="main"))))
    assert r["orcahand_description"]["status"] == "ok"
    monkeypatch.setenv("ORCA_UI", str(mac.ui.parent / "nowhere"))
    r = by_name(doctor.run(["orca_ui"], probe=probe(mac)))["orca_ui"]
    assert r["status"] == "warn" and f"git clone git@github.com:orcahand/orca_ui.git {mac.ui.parent / 'nowhere'}" in r["fix"]


def test_a_failed_import_child_runs_once(mac):
    class Broken(Fake):
        def run(self, cmd, **kw):
            if cmd[1:2] == ["-c"]:
                self.calls.append(cmd)
                raise subprocess.TimeoutExpired(cmd, 180)
            return super().run(cmd, **kw)
    fake = Broken()
    r = by_name(doctor.run(["opencv", "segmenter_packages", "rig_packages"], probe=probe(mac, fake)))
    assert all(x["status"] == "error" and "TimeoutExpired" in x["detail"] for x in r.values())
    assert sum(c[1:2] == ["-c"] for c in fake.calls) == 1


def test_the_groups_are_every_check_once_and_cameras_reuse_the_usb_listing(mac, monkeypatch, capsys):
    assert doctor.SOFTWARE + doctor.HARDWARE == tuple(doctor.CHECKS)
    fake = Fake()
    p = probe(mac, fake, usb_cameras=None)
    assert [c["location"] for c in p.cameras()] == [c["location"] for c in CAMS]
    assert not any(c[:3] == ["ioreg", "-p", "IOUSB"] for c in fake.calls)          # usb() already listed them
    ready = probe(mac)
    monkeypatch.setattr(doctor, "Probe", lambda: ready)
    doctor.main(["--json", "software", "--skip", "birefnet_weights"])
    names = [r["name"] for r in json.loads(capsys.readouterr().out)]
    assert names == [n for n in doctor.SOFTWARE if n != "birefnet_weights"]
    with pytest.raises(SystemExit):
        doctor.main(["--skip", "nope"])


def test_a_check_that_crashes_is_an_error_not_an_exception(mac):
    p = probe(mac)
    p.rig = lambda: (_ for _ in ()).throw(OSError("disk gone"))
    r = by_name(doctor.run(["camera_identity", "python"], probe=p))
    assert r["camera_identity"]["status"] == "error" and "disk gone" in r["camera_identity"]["detail"]
    assert r["python"]["status"] == "ok"


def test_the_cli_prints_json_and_exits_by_the_worst_status(mac, monkeypatch, capsys):
    ready, bare = probe(mac, python=(3, 12, 1)), probe(mac, cams=[])
    monkeypatch.setattr(doctor, "Probe", lambda: ready)
    assert doctor.main(["--json", "python", "rosetta"]) == 0
    assert [r["name"] for r in json.loads(capsys.readouterr().out)] == ["python", "rosetta"]
    monkeypatch.setattr(doctor, "Probe", lambda: bare)
    assert doctor.main(["usb_cameras"]) == 1
    assert "[error] usb_cameras: no USB camera" in capsys.readouterr().out
