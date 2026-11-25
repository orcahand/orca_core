"""Is this Mac ready for the rig? Every check prints ok / warn / error, what it found and the
exact command that fixes it. Nothing is opened (no camera, no serial port), nothing downloaded.

    python -m visual_calibration.doctor            # from experiments/visual_calibration
    python -m visual_calibration.doctor --json     # the list run() returns

``run(checks=None)`` is the contract orca-ui serves at ``GET /api/visual-calibration/doctor``: a
list of ``{name, status, detail, fix}``, ``status`` one of ok / warn / error, ``fix`` the command
or step ("" when ok), in CHECKS order; ``checks`` picks some of CHECKS by name. A check never
raises: one that fails is an error naming why. The hardware checks report "not connected" on a
computer without the rig. The package imports (RUN_IMPORTS) run in a child of this Python, so
torch never loads into the caller; uvcc is run only from npx's cache.
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import platform
import shlex
import shutil
import subprocess
import sys
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path

OK, WARN, ERROR = "ok", "warn", "error"
EXPERIMENT = Path(__file__).resolve().parents[1]          # experiments/visual_calibration
CHECKOUT = Path(__file__).resolve().parents[3]            # the orca_core checkout
PYTHON_TESTED = (3, 12)
PYTHON_MIN = (3, 11)
BIREFNET_FILES = ("config.json", "birefnet.py", "BiRefNet_config.py", "model.safetensors")
FIRMWARE_BRANCH = "internal/board-calibration-store"
RIG_BRANCH = "internal/rig-calibration"          # orca_core's and orca_ui's
MIN_CAMERAS = 2

OPENCV = ("cv2",)
SEGMENTER = ("torch", "torchvision", "transformers", "timm", "kornia", "einops", "huggingface_hub")
RIG = ("numpy", "yaml", "serial", "dynamixel_sdk", "websockets.sync.client")
RUN_IMPORTS = OPENCV + SEGMENTER + RIG      # every module the rig and a hand run import (cv2 needs aruco too)

_IMPORTS = """
import importlib, json, sys
out = {}
for m in sys.argv[1:]:
    try:
        mod = importlib.import_module(m)
        out[m] = {"version": str(getattr(mod, "__version__", "") or getattr(sys.modules.get(m.split(".")[0]), "__version__", ""))}
    except Exception as e:
        out[m] = {"error": f"{type(e).__name__}: {e}"}
if "version" in out.get("cv2", {}):
    import cv2
    a = getattr(cv2, "aruco", None)
    out["cv2"]["aruco"] = all(hasattr(a, n) for n in ("ArucoDetector", "CharucoDetector", "CharucoBoard"))
if "version" in out.get("torch", {}):
    import torch
    out["torch"]["mps"] = bool(torch.backends.mps.is_available())
print(json.dumps(out))
"""


def _comports():
    import serial.tools.list_ports
    return list(serial.tools.list_ports.comports())


@dataclass
class Probe:
    """What the checks read from this computer; tests hand in fakes."""
    run: Callable = subprocess.run
    environ: Mapping = field(default_factory=lambda: os.environ)
    python: tuple = tuple(sys.version_info[:3])
    executable: str = sys.executable
    machine: str = field(default_factory=platform.machine)
    which: Callable = shutil.which
    comports: Callable = _comports
    usb_cameras: Callable | None = None              # () -> camera_assign.usb_cameras()
    usb_devices: Callable | None = None              # () -> uvc.usb_devices()
    rig: Callable | None = None                      # () -> RigConfig
    orca_core_file: Callable | None = None           # () -> orca_core.__file__
    cache: dict = field(default_factory=dict)

    def cameras(self) -> list[dict]:
        if "cameras" not in self.cache:
            from visual_calibration.camera_assign import usb_cameras
            self.cache["cameras"] = (self.usb_cameras or (lambda: usb_cameras(self.run, usb=self.usb())))()
        return self.cache["cameras"]

    def usb(self) -> list[dict]:
        if "usb" not in self.cache:
            from visual_calibration.uvc import usb_devices
            self.cache["usb"] = (self.usb_devices or (lambda: usb_devices(self.run)))()
        return self.cache["usb"]

    def rig_config(self):
        if "rig" not in self.cache:
            from visual_calibration.rig_config import RigConfig
            self.cache["rig"] = (self.rig or RigConfig.load)()
        return self.cache["rig"]

    def imports(self) -> dict:
        if "imports" not in self.cache:
            try:
                p = self.run([self.executable, "-c", _IMPORTS, *RUN_IMPORTS],
                             capture_output=True, text=True, timeout=180)
                if p.returncode:
                    raise RuntimeError(f"{self.executable} could not run the imports: "
                                       f"{(p.stderr or p.stdout).strip().splitlines()[-1:]}")
                self.cache["imports"] = json.loads(p.stdout.strip().splitlines()[-1])
            except Exception as e:                    # once: the other package checks report it too
                self.cache["imports"] = e
        if isinstance(self.cache["imports"], Exception):
            raise self.cache["imports"]
        return self.cache["imports"]

    def home(self) -> Path:
        return Path(self.environ.get("HOME") or Path.home())

    def place(self, env: str, name: str) -> Path:
        """A sibling checkout where setup_rig.sh looks for it: $env, else beside orca_core."""
        return Path(self.environ.get(env) or CHECKOUT.parent / name)


def _result(status: str, detail: str, fix: str = "") -> dict:
    return {"status": status, "detail": detail, "fix": "" if status == OK else fix}


def _q(path) -> str:
    return shlex.quote(str(path))


def _install_rig(p: Probe, *extra: str) -> str:
    return " ".join(["uv pip install --python", _q(p.executable), *extra, "-r", _q(CHECKOUT / "pyproject.toml"),
                     "--extra rig"])


def _setup_rig() -> str:
    return f"{_q(EXPERIMENT / 'setup_rig.sh')}   # again"


def _git_branch(p: Probe, path: Path) -> str | None:
    try:
        r = p.run(["git", "-C", str(path), "branch", "--show-current"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout.strip() or None if r.returncode == 0 else None


# ---- the checks
def check_python(p: Probe) -> dict:
    v = ".".join(map(str, p.python))
    where = f"Python {v} ({p.executable})"
    fix = _setup_rig()
    if tuple(p.python[:2]) == PYTHON_TESTED:
        return _result(OK, where)
    if tuple(p.python[:2]) >= PYTHON_MIN:
        return _result(WARN, f"{where}; the rig was run on 3.12", fix)
    return _result(ERROR, f"{where}; the rig's packages need 3.11+, tested on 3.12", fix)


def _packages(p: Probe, modules: Iterable[str], what: str) -> dict:
    found = p.imports()
    bad = {m: found.get(m, {}).get("error", "not checked") for m in modules if "version" not in found.get(m, {})}
    have = ", ".join(f"{m} {found[m]['version']}".strip() for m in modules if m not in bad)
    if bad:
        return _result(ERROR, f"{what}: " + "; ".join(f"{m}: {e}" for m, e in bad.items()) + (f" (have {have})" if have else ""),
                       _install_rig(p))
    return _result(OK, have)


def check_opencv(p: Probe) -> dict:
    r = _packages(p, OPENCV, "OpenCV")
    if r["status"] == OK and not p.imports()["cv2"].get("aruco"):
        return _result(ERROR, f"OpenCV {p.imports()['cv2']['version']} without aruco (ArucoDetector, CharucoDetector): "
                              "the opencv-python package, not opencv-contrib-python-headless?",
                       f"uv pip uninstall --python {_q(p.executable)} opencv-python opencv-python-headless "
                       f"opencv-contrib-python && "
                       f"{_install_rig(p, '--reinstall-package opencv-contrib-python-headless')}")
    if r["status"] == OK:
        r["detail"] = f"OpenCV {p.imports()['cv2']['version']} with aruco"
    return r


def check_segmenter_packages(p: Probe) -> dict:
    r = _packages(p, SEGMENTER, "the BiRefNet segmenter")
    if r["status"] == OK:
        r["detail"] += "; torch on " + ("MPS" if p.imports()["torch"].get("mps") else "the CPU (no MPS: slow)")
    return r


def check_rig_packages(p: Probe) -> dict:
    return _packages(p, RIG, "the rig and the hand run")


def hub_cache(environ: Mapping, home: Path) -> Path:
    """The Hugging Face hub cache, as huggingface_hub finds it."""
    for key in ("HF_HUB_CACHE", "HUGGINGFACE_HUB_CACHE"):
        if environ.get(key):
            return Path(environ[key]).expanduser()
    hf = environ.get("HF_HOME") or os.path.join(environ.get("XDG_CACHE_HOME") or str(home / ".cache"), "huggingface")
    return Path(hf).expanduser() / "hub"


def check_birefnet_weights(p: Probe) -> dict:
    from hand_study.segment import BIREFNET_REPO, BIREFNET_REVISION
    hub = hub_cache(p.environ, p.home())
    snap = hub / f"models--{BIREFNET_REPO.replace('/', '--')}" / "snapshots" / BIREFNET_REVISION
    fix = (f"cd {_q(EXPERIMENT)} && {_q(p.executable)} -c "
           "'from hand_study.segment import BiRefNet; BiRefNet()'   # downloads ~450 MB once")
    missing = [f for f in BIREFNET_FILES if not (snap / f).is_file()]
    if not missing:
        mb = (snap / "model.safetensors").stat().st_size / 1e6
        return _result(OK, f"{BIREFNET_REPO}@{BIREFNET_REVISION[:12]} in {snap} ({mb:.0f} MB)")
    if p.environ.get("HF_HUB_OFFLINE") not in (None, "", "0"):
        fix = "unset HF_HUB_OFFLINE; " + fix
    return _result(ERROR, f"{BIREFNET_REPO} revision {BIREFNET_REVISION[:12]} is not in the Hugging Face cache {hub}"
                          + (f" (incomplete: {', '.join(missing)} missing)" if snap.is_dir() else "")
                          + ": the hand run would download it at its start", fix)


def check_orcahand_description(p: Probe) -> dict:
    from visual_calibration.hand_model import DESCRIPTION_ENV, description_root
    root = description_root()
    looked = p.place(DESCRIPTION_ENV, "orcahand_description")
    fix = (f"git clone git@github.com:orcahand/orcahand_description.git {_q(looked)}"
           f"   # or export {DESCRIPTION_ENV}=<its checkout>")
    if root is None:
        return _result(ERROR, f"no orcahand_description checkout (v2/models/urdf) at {looked}", fix)
    branch = _git_branch(p, root)
    if branch not in (None, "main"):
        return _result(WARN, f"{root} on {branch}, not main", f"git -C {_q(root)} switch main")
    return _result(OK, f"{root}" + (f" ({branch})" if branch else ""))


def check_uvcc(p: Probe) -> dict:
    from visual_calibration.uvc import TOOL
    cmd = shlex.split(TOOL)
    if not p.which(cmd[0]):
        return _result(ERROR, f"{cmd[0]} not found: uvcc (the camera controls) runs on Node.js",
                       "brew install node   # or the installer from https://nodejs.org")
    if cmd[0] == "npx":                     # only what npx has cached: the doctor downloads nothing
        cmd = [cmd[0], "--no-install", *(a for a in cmd[1:] if a not in ("--yes", "-y"))]
    fetch = f"{TOOL} --version   # downloads it once (network); setup_rig.sh does this"
    try:
        r = p.run([*cmd, "--version"], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError) as e:
        return _result(ERROR, f"{' '.join(cmd)} --version: {e}", fetch)
    out = (r.stderr or "") + (r.stdout or "")
    if r.returncode and "missing packages" in out:
        return _result(WARN, f"{TOOL} is not in npx's cache yet: the first camera check would download it", fetch)
    if r.returncode:
        why = out.strip().splitlines()[-1:] or [f"exit {r.returncode}"]
        return _result(ERROR, f"{' '.join(cmd)} --version: {why[0]}", fetch)
    return _result(OK, f"{TOOL} {r.stdout.strip().splitlines()[-1] if r.stdout.strip() else ''}".strip())


def check_orca_core(p: Probe) -> dict:
    fix = f"cd {_q(p.place('ORCA_UI', 'orca_ui'))} && ./dev local {_q(CHECKOUT)}   # or {_setup_rig()}"
    try:
        if p.orca_core_file is not None:
            where = Path(p.orca_core_file()).resolve().parent
        else:
            import orca_core
            where = Path(orca_core.__file__).resolve().parent
    except Exception as e:
        return _result(ERROR, f"orca_core does not import: {e}", fix)
    if where != (CHECKOUT / "orca_core").resolve():
        return _result(ERROR, f"orca_core is imported from {where}, not this checkout ({CHECKOUT})", fix)
    try:
        board_push = importlib.import_module("orca_core.board_push")
        if not callable(getattr(board_push, "board_contents", None)):
            raise AttributeError("no board_push.board_contents")
    except Exception as e:
        return _result(ERROR, f"orca_core at {where} lacks the board store ({e}): not internal/rig-calibration?",
                       f"git -C {_q(CHECKOUT)} switch {RIG_BRANCH}")
    branch = _git_branch(p, CHECKOUT)
    if branch not in (None, RIG_BRANCH):
        return _result(WARN, f"{where} on {branch}, not {RIG_BRANCH}", f"git -C {_q(CHECKOUT)} switch {RIG_BRANCH}")
    return _result(OK, f"{where}" + (f" ({branch})" if branch else ""))


def check_orca_ui(p: Probe) -> dict:
    ui = p.place("ORCA_UI", "orca_ui")
    if not (ui / "orca_ui").is_dir():
        return _result(WARN, f"no orca_ui checkout at {ui}",
                       f"git clone git@github.com:orcahand/orca_ui.git {_q(ui)} && git -C {_q(ui)} switch {RIG_BRANCH}"
                       "   # or export ORCA_UI=<its checkout>")
    branch = _git_branch(p, ui)
    if branch not in (None, RIG_BRANCH):
        return _result(WARN, f"{ui} on {branch}, not {RIG_BRANCH}", f"git -C {_q(ui)} switch {RIG_BRANCH}")
    return _result(OK, f"{ui}" + (f" ({branch})" if branch else ""))


def check_firmware(p: Probe) -> dict:
    from orca_core import board_push
    repo = board_push.firmware_checkout()
    pinned = p.environ.get(board_push.FIRMWARE_REPO_ENV)
    how = f"{board_push.FIRMWARE_REPO_ENV}={pinned}" if pinned else "beside orca_core"
    optional = " (only needed to write a hand's board)"
    if not repo.is_dir():
        return _result(WARN, f"no orca_firmware checkout at {repo} ({how}){optional}",
                       f"git clone git@github.com:orcahand/orca_firmware.git {_q(repo)} && "
                       f"git -C {_q(repo)} switch {FIRMWARE_BRANCH}")
    if not (repo / board_push.TOOL_RELATIVE_PATH).is_file():
        return _result(WARN, f"{repo} has no {board_push.TOOL_RELATIVE_PATH.as_posix()}: not {FIRMWARE_BRANCH}?{optional}",
                       f"git -C {_q(repo)} switch {FIRMWARE_BRANCH}")
    if not (repo / ".arduino" / "bin" / "arduino-cli").exists():
        return _result(WARN, f"{repo}: no toolchain in .arduino/{optional}", f"cd {_q(repo)} && ./setup.sh")
    branch = _git_branch(p, repo)
    if branch not in (None, FIRMWARE_BRANCH):
        return _result(WARN, f"{repo} on {branch}, not {FIRMWARE_BRANCH}{optional}", f"git -C {_q(repo)} switch {FIRMWARE_BRANCH}")
    if not pinned:
        return _result(WARN, f"{repo} with its toolchain, but {board_push.FIRMWARE_REPO_ENV} is not set: orca-ui hides "
                             "\"Write to board\"", f"export {board_push.FIRMWARE_REPO_ENV}={_q(repo)}   # rig-console.sh sets it")
    return _result(OK, f"{repo} ({how}) with cal_store.py and its toolchain")


def check_rosetta(p: Probe) -> dict:
    if p.machine != "arm64":
        return _result(OK, f"not needed on {p.machine}")
    try:
        r = p.run(["arch", "-x86_64", "/usr/bin/true"], capture_output=True, text=True, timeout=20)
        ok = r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        ok = False
    if ok:
        return _result(OK, "installed (the firmware toolchain's x86_64 compiler runs)")
    return _result(WARN, "not installed: the firmware toolchain (arm-none-eabi-gcc 7) is x86_64 only; needed to "
                         "flash a board", "softwareupdate --install-rosetta --agree-to-license")


def check_usb_cameras(p: Probe) -> dict:
    cams = p.cameras()
    from visual_calibration.camera_assign import describe
    if not cams:
        return _result(ERROR, "no USB camera connected (the Mac's own camera is not one of the rig's)",
                       "plug both rig cameras into the Mac")
    lines = "; ".join(describe(d) for d in cams)
    serials = [d.get("serial") for d in cams if d.get("serial")]
    shared = sorted({s for s in serials if serials.count(s) > 1})
    note = f"; serial {', '.join(shared)} shared: told apart by USB port" if shared else ""
    if len(cams) < MIN_CAMERAS:
        return _result(ERROR, f"{len(cams)} USB camera, the rig has {MIN_CAMERAS}: {lines}", "plug in the other rig camera")
    slow = [d for d in cams if d.get("speed") is not None and d["speed"] <= 2           # a USB 3 camera on USB 2
            and (d.get("bcd_usb") is None or d["bcd_usb"] >= 0x300)]
    if slow:
        return _result(WARN, f"{len(cams)} USB cameras: {lines}{note}; " + ", ".join(
            f"{d.get('name')} at 0x{d.get('location', 0):08x}" for d in slow)
            + " on a USB 2 link: a BRIO needs USB 3 for 4K", "plug it into a USB 3 port with a USB 3 cable (no USB 2 hub)")
    return _result(OK, f"{len(cams)} USB cameras: {lines}{note}")


def check_camera_identity(p: Probe) -> dict:
    from visual_calibration.rig_cameras import CameraIdentityError, match_cameras
    rig = p.rig_config()
    names = list(rig.active_cameras)
    config = Path(rig.path) / "config.yaml"
    if len(names) < MIN_CAMERAS:
        return _result(ERROR, f"{len(names)} camera(s) enabled in {config} ({', '.join(names)}), the rig needs {MIN_CAMERAS}",
                       f"set cameras.b.enabled: true in {config}")
    devices = {c: dict(((rig.cameras[c].get("uvc") or {}).get("device")) or {}) for c in names}
    fix = f"cd {_q(EXPERIMENT)} && {_q(p.executable)} -m visual_calibration.sweep --assign-cameras"
    for c, d in devices.items():
        if d.get("vendor") is None or d.get("product") is None:
            return _result(ERROR, f"camera {c}: no uvc.device vendor/product in {config}", fix)
    unnamed = [c for c, d in devices.items() if not d.get("serial") and d.get("location") is None]
    if unnamed:
        return _result(ERROR, f"camera {', '.join(unnamed)}: no serial in {config}", fix)
    keys = {c: (d["vendor"], d["product"], d.get("serial")) for c, d in devices.items()}
    twins = [c for c in names if list(keys.values()).count(keys[c]) > 1 and devices[c].get("location") is None]
    if twins:
        return _result(ERROR, f"cameras {', '.join(twins)} share a serial and have no USB location in {config}", fix)
    from visual_calibration.camera_assign import untuned, untuned_message
    models = untuned(devices)
    if models:
        return _result(ERROR, untuned_message(models),
                       f"adapt cameras.<c>.resolution and cameras.<c>.uvc in {_q(config)} to these cameras")
    said = "; ".join(f"{c} = {d['vendor']:04x}:{d['product']:04x} serial {d.get('serial')}"
                     + (f" port 0x{int(d['location']):08x}" if d.get("location") is not None else "")
                     for c, d in devices.items())
    try:
        match_cameras(devices, p.usb())
    except CameraIdentityError as e:
        return _result(WARN, f"configured ({said}); not on USB now: {e}",
                       "plug the cameras in (each twin into its own port), or " + fix)
    return _result(OK, f"{said}; both on USB (which is A and which B: orca-ui's Check cameras, tower on the plate)")


def _rig_serial(p: Probe) -> tuple[str | None, str | None, str]:
    """The rig board's USB serial as the rig code looks for it, and its pinned port."""
    from orca_core.utils.utils import read_yaml
    rig = p.rig_config()
    board = (rig.raw.get("turntable") or {}).get("board") or {}
    if board.get("usb_serial"):
        return str(board["usb_serial"]), board.get("port"), "turntable.yaml board.usb_serial"
    try:
        cal = (read_yaml(rig.turntable_calibration_path) or {}).get("board") or {}
    except (OSError, ValueError):
        cal = {}
    if cal.get("usb_serial"):
        return str(cal["usb_serial"]), board.get("port"), "the motor calibration's board"
    return None, board.get("port"), ""


def check_turntable_board(p: Probe) -> dict:
    serial_no, port, source = _rig_serial(p)
    ports = p.comports()
    where = Path(p.rig_config().path) / "turntable.yaml"
    if serial_no:
        found = sorted(x.device for x in ports if x.serial_number == serial_no)
        if found:
            return _result(OK, f"the rig board (USB serial {serial_no[:12]}…, {source}) on {', '.join(found)}")
        return _result(ERROR, f"the rig board (USB serial {serial_no[:12]}…, {source}) is not connected",
                       "plug in the turntable's board (and its motor power)")
    if port not in (None, "auto"):
        if any(x.device == port for x in ports):
            return _result(OK, f"{port} (turntable.yaml board.port)")
        return _result(ERROR, f"{port} (turntable.yaml board.port) is not there", "plug in the turntable's board")
    return _result(WARN, "the rig board is found only by scanning every motor board for one motor (that opens ports)",
                   f"pin its USB serial as board.usb_serial in {where}")


def check_hand_board(p: Probe) -> dict:
    from orca_core.constants import KNOWN_VIDS
    rig_serial, _, _ = _rig_serial(p)
    boards: dict = {}
    for x in p.comports():
        if x.vid in KNOWN_VIDS["oh_board"] and x.serial_number != rig_serial:
            boards.setdefault(x.serial_number, []).append(x.device)
    if not boards:
        return _result(WARN, "no hand board connected (needed for the hand run, not for the rig calibration)",
                       "plug in the hand")
    said = "; ".join(f"{', '.join(sorted(v))}" for v in boards.values())
    if len(boards) > 1 and rig_serial is None:
        return _result(WARN, f"{len(boards)} OH boards ({said}); which is the rig's is found by probing",
                       "pin the rig board's board.usb_serial in turntable.yaml")
    if len(boards) > 1:
        return _result(WARN, f"{len(boards)} hand boards ({said})", "plug in one hand only")
    return _result(OK, f"a hand board on {said}")


CHECKS: dict[str, Callable[[Probe], dict]] = {
    "python": check_python,
    "opencv": check_opencv,
    "segmenter_packages": check_segmenter_packages,
    "rig_packages": check_rig_packages,
    "birefnet_weights": check_birefnet_weights,
    "orcahand_description": check_orcahand_description,
    "uvcc": check_uvcc,
    "orca_core": check_orca_core,
    "orca_ui": check_orca_ui,
    "firmware": check_firmware,
    "rosetta": check_rosetta,
    "usb_cameras": check_usb_cameras,
    "camera_identity": check_camera_identity,
    "turntable_board": check_turntable_board,
    "hand_board": check_hand_board,
}
HARDWARE = ("usb_cameras", "camera_identity", "turntable_board", "hand_board")
SOFTWARE = tuple(n for n in CHECKS if n not in HARDWARE)
GROUPS = {"software": SOFTWARE, "hardware": HARDWARE}          # names the CLI takes for several checks


def run(checks: Iterable[str] | None = None, probe: Probe | None = None) -> list[dict]:
    """Every check (or those named), in CHECKS order: [{name, status, detail, fix}]."""
    names = list(CHECKS) if checks is None else list(checks)
    unknown = [n for n in names if n not in CHECKS]
    if unknown:
        raise ValueError(f"no check {', '.join(unknown)}; the checks: {', '.join(CHECKS)}")
    probe = probe or Probe()
    out = []
    for n in names:
        try:
            r = CHECKS[n](probe)
        except Exception as e:                       # a check that cannot run is a finding, not a crash
            r = _result(ERROR, f"the check failed: {type(e).__name__}: {e}", "")
        out.append({"name": n, "status": r["status"], "detail": r["detail"], "fix": r["fix"]})
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Check this computer for the camera rig (opens nothing).")
    ap.add_argument("checks", nargs="*", metavar="CHECK",
                    help=f"only these: {', '.join(GROUPS)} or {', '.join(CHECKS)}")
    ap.add_argument("--skip", action="append", default=[], metavar="CHECK", help="leave this check out")
    ap.add_argument("--json", action="store_true", help="print the results as JSON")
    args = ap.parse_args(argv)
    names = [n for c in (args.checks or list(CHECKS)) for n in GROUPS.get(c, (c,))]
    unknown = [s for s in args.skip if s not in CHECKS]
    if unknown:
        ap.error(f"no check {', '.join(unknown)} to skip")
    try:
        results = run([n for n in dict.fromkeys(names) if n not in args.skip])
    except ValueError as e:
        ap.error(str(e))
    if args.json:
        print(json.dumps(results, indent=1))
    else:
        for r in results:
            print(f"[{r['status']:>5}] {r['name']}: {r['detail']}")
            if r["fix"]:
                print(f"        fix: {r['fix']}")
        n = {s: sum(r["status"] == s for r in results) for s in (OK, WARN, ERROR)}
        print(f"{n[OK]} ok, {n[WARN]} warn, {n[ERROR]} error")
    return 1 if any(r["status"] == ERROR for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
