"""Which USB camera is which rig camera, written into config.yaml so nobody edits it by hand
(``python -m visual_calibration.sweep --assign-cameras``).

The plugged-in USB cameras come from the IORegistry (ioreg) and AVFoundation's list
(system_profiler); nothing is opened. Each rig camera gets its USB vendor, product and serial in
``cameras.<c>.uvc.device``, and its USB location (the port) when two cameras report the same
serial (rig_cameras.match_cameras). Which camera is A and which B cannot be known from USB: the
assignment is a guess until orca-ui's "Check cameras" (with the tower on the plate) proves each
camera's height matches its mount, or refuses it; ``--swap`` then trades the two.

The camera identity lives only in the checkout's ``rigs/<version>/camera-rig/config.yaml``
(rig_config has no per-rig layer), so this edits that tracked file in place, comments kept.
"""
from __future__ import annotations

import json
import re
import subprocess
from collections.abc import Callable, Mapping
from pathlib import Path

import yaml
from orca_core.utils.utils import write_text_atomic

from visual_calibration.uvc import SPEEDS, usb_devices

VIDEO_CLASS = 14            # USB interface class of a UVC camera
NOTE = "set by sweep --assign-cameras; Check cameras proves it"
SWAP = "python -m visual_calibration.sweep --assign-cameras --swap"


class AssignError(RuntimeError):
    pass


def video_locations(run: Callable = subprocess.run) -> set[int]:
    """USB locations of the devices with a video interface (UVC), from the IORegistry (opens nothing)."""
    p = run(["ioreg", "-r", "-c", "IOUSBHostInterface", "-l", "-w0"], capture_output=True, text=True, timeout=20)
    found, node = set(), None

    def done(n):
        if n and n.get("class") == VIDEO_CLASS and n.get("location") is not None:
            found.add(n["location"])
    for line in (p.stdout or "").splitlines():
        if "+-o " in line:
            done(node)
            node = {} if "<class IOUSBHostInterface" in line else None
            continue
        m = re.search(r'"(bInterfaceClass|locationID)" = (\d+)', line)
        if node is not None and m:
            node["class" if m.group(1) == "bInterfaceClass" else "location"] = int(m.group(2))
    done(node)
    return found


def usb_cameras(run: Callable = subprocess.run, usb: list[dict] | None = None, video: set[int] | None = None,
                av: list[dict] | None = None) -> list[dict]:
    """The plugged-in USB cameras, by location (the port): a USB device with a video interface or
    among AVFoundation's video devices, with its OpenCV index (avfoundation_index) and link speed."""
    from visual_calibration.rig_cameras import avfoundation_devices, avfoundation_index
    usb = usb_devices(run) if usb is None else usb
    video = video_locations(run) if video is None else video
    av = avfoundation_devices(run) if av is None else av
    out = []
    for d in usb:
        i = avfoundation_index(d, av)
        if d.get("location") in video or i is not None:
            out.append({**d, "avfoundation_index": i, "link": SPEEDS.get(d.get("speed"))})
    return sorted(out, key=lambda d: d.get("location", 0))


def describe(d: Mapping) -> str:
    serial = f"serial {d['serial']}" if d.get("serial") else "no serial"
    return (f"{d.get('name', '?')}  {d.get('vendor', 0):04x}:{d.get('product', 0):04x}  {serial}  "
            f"location 0x{d.get('location', 0):08x}  {d.get('link') or 'speed unknown'}")


def plan(cameras: list[dict], names: list[str], picks: list[int] | None = None) -> dict[str, dict]:
    """The uvc.device of each rig camera: the listed cameras in order (picks: their 1-based numbers,
    one per rig camera), the location kept only where serials collide or a camera has none."""
    if picks:
        if len(picks) != len(names) or len(set(picks)) != len(picks):
            raise AssignError(f"give one listed camera per rig camera ({', '.join(names)}), each once")
        bad = [n for n in picks if not 1 <= n <= len(cameras)]
        if bad:
            raise AssignError(f"no camera {bad[0]}: {len(cameras)} listed")
        chosen = [cameras[n - 1] for n in picks]
    elif len(cameras) == len(names):
        chosen = list(cameras)
    else:
        raise AssignError(f"{len(cameras)} USB camera(s) plugged in, the rig has {len(names)} "
                          f"({', '.join(names)})" + ("; pick them by number: --assign-cameras "
                                                     + " ".join(str(i + 1) for i in range(len(names)))
                                                     if len(cameras) > len(names) else ""))
    def key(d):
        return int(d["vendor"]), int(d["product"]), d.get("serial") or None
    out = {}
    for c, d in zip(names, chosen):
        vendor, product, serial = key(d)
        out[c] = {"vendor": vendor, "product": product, "serial": serial}
        if serial is None or any(key(o) == key(d) for o in cameras if o is not d):
            out[c]["location"] = int(d["location"])
    return out


def _flow(device: Mapping) -> str:
    parts = [f"vendor: {int(device['vendor'])}", f"product: {int(device['product'])}",
             f"serial: {json.dumps(device['serial']) if device.get('serial') is not None else 'null'}"]
    if device.get("location") is not None:
        parts.append(f"location: 0x{int(device['location']):08x}")
    return "{" + ", ".join(parts) + "}"


def _child(node, key: str, where: str):
    if isinstance(node, yaml.MappingNode):
        for k, v in node.value:
            if k.value == key:
                return k, v
    raise AssignError(f"config.yaml has no {where}")


def write_devices(config: Path, devices: Mapping[str, Mapping], note: str = NOTE) -> str:
    """Replace each camera's cameras.<c>.uvc.device in config.yaml, everything else (comments
    too) as it was. Returns the new text."""
    text = Path(config).read_text()
    root = yaml.compose(text)
    spans = []
    for c in devices:
        _, cams = _child(root, "cameras", "cameras:")
        _, cam = _child(cams, c, f"cameras.{c}")
        _, uvc = _child(cam, "uvc", f"cameras.{c}.uvc")
        key, node = _child(uvc, "device", f"cameras.{c}.uvc.device")
        if node.flow_style:
            start, end = node.start_mark.index, node.end_mark.index
            new = _flow(devices[c])
        else:                                   # a block mapping: from after the key to its last line
            start, end = key.end_mark.index, len(text[:node.end_mark.index].rstrip())
            new = ": " + _flow(devices[c])
        eol = text.find("\n", end)
        eol = len(text) if eol < 0 else eol
        rest = text[end:eol]
        tail = re.match(r"\s*(#.*)?$", rest)
        if tail:
            end, new = eol, new + f"   # {note}"
        spans.append((start, end, new))
    for start, end, new in sorted(spans, reverse=True):
        text = text[:start] + new + text[end:]
    loaded = yaml.safe_load(text)
    for c, d in devices.items():
        got = loaded["cameras"][c]["uvc"]["device"]
        if {k: got.get(k) for k in ("vendor", "product", "serial", "location")} != \
                {k: d.get(k) for k in ("vendor", "product", "serial", "location")}:
            raise AssignError(f"cameras.{c}.uvc.device would read back as {got}, not {dict(d)}: nothing written")
    write_text_atomic(config, text)
    return text


def configured(config: Path, names: list[str]) -> dict[str, dict]:
    raw = yaml.safe_load(Path(config).read_text()) or {}
    return {c: dict(((raw.get("cameras") or {}).get(c) or {}).get("uvc", {}).get("device") or {}) for c in names}


def swap(config: Path, names: list[str]) -> dict[str, dict]:
    """Trade the two cameras' USB identities (after Check cameras found them swapped)."""
    if len(names) != 2:
        raise AssignError(f"--swap trades two cameras, the rig has {len(names)} enabled ({', '.join(names)})")
    have = configured(config, names)
    if any(have[c].get("vendor") is None for c in names):
        raise AssignError("a camera has no uvc.device yet: run --assign-cameras first")
    a, b = names
    devices = {a: have[b], b: have[a]}
    write_devices(config, devices, NOTE + " (swapped)")
    return devices


def assign(rig, picks: list[int] | None = None, do_swap: bool = False, run: Callable = subprocess.run,
           cameras: list[dict] | None = None) -> list[str]:
    """Assign (or swap) the rig's enabled cameras in its config.yaml; the lines to print."""
    config = Path(rig.path) / "config.yaml"
    names = list(rig.active_cameras)
    if do_swap:
        devices = swap(config, names)
        lines = [f"Swapped in {config}:"]
    else:
        cameras = usb_cameras(run) if cameras is None else cameras
        lines = ["USB cameras (ioreg; nothing opened):"]
        lines += [f"  {i}  {describe(d)}" for i, d in enumerate(cameras, 1)] or ["  none"]
        try:
            devices = plan(cameras, names, picks)
        except AssignError as e:
            raise AssignError("\n".join([*lines, f"Nothing written: {e}"])) from None
        write_devices(config, devices)
        lines.append(f"Wrote {config}:")
    for c, d in devices.items():
        where = f", port 0x{d['location']:08x}" if d.get("location") is not None else ""
        lines.append(f"  camera {c} = {d['vendor']:04x}:{d['product']:04x} serial {d.get('serial')}{where}")
    if any(d.get("location") is not None for d in devices.values()):
        lines.append("These cameras report the same serial (or none), so each is known by its USB port: keep "
                     "it plugged into that port; after moving one, run --assign-cameras again.")
    lines.append("Which camera is A and which is B is a guess: USB cannot tell how they are mounted. In orca-ui › "
                 "Setup › Visual calibration, with the tower on the plate, \"Check cameras\" proves it (each "
                 "camera's height must match its mount) or refuses with \"cameras swapped?\". Then run")
    lines.append(f"  {SWAP}")
    lines.append("and Check cameras again. This edits the checkout's config.yaml (a tracked file).")
    return lines
