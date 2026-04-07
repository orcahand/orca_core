# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
import yaml
import numpy as np

from ..constants import DEFAULT_MODEL_NAME
from ..version import LATEST_VERSION

################################################################################
### Model path utils ##########################################################
################################################################################

def _get_package_root() -> str:
    return os.path.dirname(os.path.dirname(__file__))


def _get_models_dir() -> str:
    return os.path.join(_get_package_root(), "models")


def _list_available_models(models_dir: str) -> list[str]:
    available: list[str] = []
    if not os.path.isdir(models_dir):
        return available

    for version in sorted(os.listdir(models_dir)):
        version_dir = os.path.join(models_dir, version)
        if not os.path.isdir(version_dir):
            continue
        for model_name in sorted(os.listdir(version_dir)):
            model_dir = os.path.join(version_dir, model_name)
            if os.path.isdir(model_dir):
                available.append(f"{version}/{model_name}")

    return available


def _version_sort_key(version: str) -> tuple[int, str]:
    if version.startswith("v") and version[1:].isdigit():
        return (int(version[1:]), version)
    return (-1, version)


def _find_latest_model_version(models_dir: str, model_name: str) -> str | None:
    if not os.path.isdir(models_dir):
        return None

    versions = sorted(os.listdir(models_dir), key=_version_sort_key, reverse=True)
    for version in versions:
        candidate = os.path.join(models_dir, version, model_name)
        if os.path.isdir(candidate):
            return version
    return None


def _find_model_dir_across_versions(models_dir: str, model_name: str) -> str | None:
    version = _find_latest_model_version(models_dir, model_name)
    if version is None:
        return None
    return os.path.join(models_dir, version, model_name)


def get_model_path(model_path=None, model_version: str | None = None, model_name: str | None = None):
    """Resolve and validate a hand "model" directory path (containing util files 
    like ``config.yaml``).

    When *model_path* is ``None`` or ``"models"`` the bundled default model
    directory ``models/<LATEST_VERSION>/<DEFAULT_MODEL_NAME>`` is returned. An absolute
    or relative path may also be supplied.

    Args:
        model_path: Model directory path, model name string, or ``None`` for
            the auto-discovered default.
        model_version: Version folder under ``models/``. Defaults to
            :data:`LATEST_VERSION`.
        model_name: Leaf model directory name. Defaults to
            :data:`DEFAULT_MODEL_NAME`.

    Returns:
        Absolute path to the model directory (always contains ``config.yaml``).
    """
    selected_version = model_version or LATEST_VERSION
    selected_name = model_name or DEFAULT_MODEL_NAME
    models_dir = _get_models_dir()
    requested_model = model_path or f"{selected_version}/{selected_name}"

    if model_path is None or model_path == "models":
        if not os.path.exists(models_dir):
            raise FileNotFoundError("\033[1;35mModels directory not found. Did you delete them?")
        if model_version is None:
            fallback_version = _find_latest_model_version(models_dir, selected_name)
            if fallback_version is not None:
                selected_version = fallback_version
        resolved_path = os.path.join(models_dir, selected_version, selected_name)
    else:
        if os.path.isabs(model_path):
            resolved_path = model_path
        else:
            model_by_name = os.path.join(models_dir, selected_version, model_path)
            model_by_versioned_path = os.path.join(models_dir, model_path)
            if os.path.isdir(model_by_name):
                resolved_path = model_by_name
            elif os.path.isdir(model_by_versioned_path):
                resolved_path = model_by_versioned_path
            else:
                package_root = os.path.dirname(_get_package_root())
                resolved_path = os.path.join(package_root, model_path)
                fallback_model_dir = (
                    _find_model_dir_across_versions(models_dir, model_path)
                    if model_version is None
                    else None
                )
                if fallback_model_dir is not None:
                    resolved_path = fallback_model_dir
    
    if not os.path.exists(resolved_path):
        available = _list_available_models(models_dir)
        msg = f"\033[1;35mModel '{requested_model}' not found."
        if available:
            msg += f" Available models: {', '.join(available)}"
        else:
            msg += " No models found in models directory."
        msg += "\033[0m"
        raise FileNotFoundError(msg)
    
    config_file = os.path.join(resolved_path, "config.yaml")
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"\033[1;35mconfig.yaml not found in {resolved_path}. Did you specify the correct model directory?\033[0m")
    
    print("Using model path: \033[1;32m{}\033[0m".format(resolved_path))
    return resolved_path

################################################################################
### YAML utils #################################################################
################################################################################

def update_yaml(file_path, key, value):
    """Update a single top-level key in a YAML file.

    The modified key is written as the first entry to make the most recent
    change immediately visible at the top of the file. NumPy arrays are
    converted to plain Python lists before serialisation.

    Args:
        file_path: Path to the YAML file. Created if it does not exist.
        key: Top-level key to update.
        value: New value. ``np.ndarray`` instances are converted to lists.
    """
    if isinstance(value, np.ndarray):
        value = value.tolist()
    
    if isinstance(value, dict):
        value = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in value.items()}

    try:
        with open(file_path, 'r+') as file:
            data = yaml.safe_load(file) or {} 
            
            new_data = {key: value} # Make the latest chnage in the yaml file the first entry
            for existing_key, existing_value in data.items():
                if existing_key != key:
                    new_data[existing_key] = existing_value
            
            file.seek(0) 
            yaml.dump(new_data, file, default_flow_style=False, sort_keys=False) 
            file.truncate() 
    except FileNotFoundError:
        with open(file_path, 'w') as file:
            yaml.dump({key: value}, file, default_flow_style=False, sort_keys=False)


def read_yaml(file_path):
    """Reads a YAML file and returns its content."""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or None
    except FileNotFoundError:
        return {}

################################################################################
### Interpolation utils ########################################################
################################################################################

def linear_interp(t):
    return t

def ease_in_out(t):
    return 0.5 * (1 - np.cos(np.pi * t))

def interpolate_waypoints(start, end, duration, step_time, mode="linear"):
    n_steps = int(duration / step_time)
    interp_func = linear_interp if mode == "linear" else ease_in_out
    for i in range(n_steps + 1):
        t = i / n_steps
        alpha = interp_func(t)
        yield [(1 - alpha) * s + alpha * e for s, e in zip(start, end)]

def auto_detect_port(motor_type: str = "dynamixel") -> str:
    """Auto-detect a serial adapter port for the given motor type.

    Args:
        motor_type: The motor driver type (``"dynamixel"`` or ``"feetech"``).

    Returns:
        The port device string if exactly one matching adapter is found,
        otherwise ``None``.
    """
    import serial.tools.list_ports
    from ..constants import KNOWN_VIDS

    known_vids = KNOWN_VIDS.get(motor_type, [])
    ports = serial.tools.list_ports.comports()
    matches = [p for p in ports if p.vid in known_vids]

    if len(matches) == 1:
        port = matches[0]
        print(f"Auto-detected {motor_type} adapter: {port.device} ({port.description or 'unknown'})")
        return port.device

    return None


def get_and_choose_port() -> str:
    """Present an interactive terminal menu for USB port selection.

    Uses ``curses`` to render an arrow-key-navigable list of all detected
    serial ports. The user selects a port with Enter or quits with ``q`` /
    Escape.

    Returns:
        Device string of the selected port (e.g. ``"/dev/ttyUSB0"``), or
        ``None`` if the user cancels or no ports are found.
    """
    import curses
    import serial.tools.list_ports
    
    def draw_menu(stdscr, ports, selected_idx):
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        
        title = "Choose a device (use arrow keys, Enter to select, q to quit)"
        stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)
        
        for i, port in enumerate(ports):
            y_pos = i * 3 + 2
            if y_pos >= height - 1:
                break
            marker = "(x)" if i == selected_idx else "( )"

            if i == selected_idx:
                stdscr.attron(curses.A_REVERSE)
                stdscr.addstr(y_pos, 0, f"{i+1:2d}. {marker} {port.device}")
                stdscr.attroff(curses.A_REVERSE)
            else:
                stdscr.addstr(y_pos, 0, f"{i+1:2d}. {marker} {port.device}")

            if y_pos + 1 < height - 1:
                stdscr.addstr(y_pos + 1, 4, f"{port.description or 'No description'}")
            if y_pos + 2 < height - 1:
                stdscr.addstr(y_pos + 2, 4, f"{port.manufacturer or 'Unknown manufacturer'}")
        
        if len(ports) + 5 < height:
            stdscr.addstr(height - 2, 0, "Use ↑↓ arrows to navigate, Enter to select, q to quit")
        
        stdscr.refresh()
    
    def main_menu(stdscr):
        curses.curs_set(0)
        stdscr.keypad(True)
        
        ports = list(serial.tools.list_ports.comports())
        
        if not ports:
            stdscr.clear()
            stdscr.addstr(0, 0, "No USB devices found!")
            stdscr.refresh()
            stdscr.getch()
            return None
        
        selected_idx = 0
        
        while True:
            draw_menu(stdscr, ports, selected_idx)
            
            key = stdscr.getch()
            
            if key == curses.KEY_UP and selected_idx > 0:
                selected_idx -= 1
            elif key == curses.KEY_DOWN and selected_idx < len(ports) - 1:
                selected_idx += 1
            elif key == curses.KEY_ENTER or key in [10, 13]:
                return ports[selected_idx].device
            elif key == ord('q') or key == ord('Q'):
                return None
            elif key == 27:
                return None
    
    try:
        return curses.wrapper(main_menu)
    except KeyboardInterrupt:
        return None
