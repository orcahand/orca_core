# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import os
from pathlib import Path

import numpy as np
import yaml

################################################################################
### Model path utils ##########################################################
################################################################################

def get_model_path(model_path=None):
    models_dir = Path(__file__).resolve().parents[1] / "models"

    if model_path is None or model_path == "models":
        model_dirs = sorted(path for path in models_dir.iterdir() if path.is_dir())
        if not model_dirs:
            raise FileNotFoundError("\033[1;35mNo built-in model files found.\033[0m")
        resolved_path = model_dirs[0]
    else:
        candidate = Path(model_path).expanduser()
        if not candidate.is_absolute():
            builtin_candidate = models_dir / model_path
            candidate = builtin_candidate if builtin_candidate.exists() else Path.cwd() / candidate

        resolved_path = candidate.resolve()
        if resolved_path.is_file():
            resolved_path = resolved_path.parent

    if not resolved_path.exists():
        available = sorted(path.name for path in models_dir.iterdir() if path.is_dir()) if models_dir.is_dir() else []
        msg = f"\033[1;35mModel '{model_path}' not found."
        if available:
            msg += f" Available models: {', '.join(available)}"
        else:
            msg += " No models found in models directory."
        msg += "\033[0m"
        raise FileNotFoundError(msg)

    config_file = resolved_path / "config.yaml"
    if not config_file.exists():
        raise FileNotFoundError(
            f"\033[1;35mconfig.yaml not found in {resolved_path}. Did you specify the correct model directory?\033[0m"
        )

    print("Using model path: \033[1;32m{}\033[0m".format(resolved_path))
    return str(resolved_path)

################################################################################
### YAML utils #################################################################
################################################################################

def update_yaml(file_path, key, value):
    """Reads a YAML file, updates a specific key, and writes it back."""
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


def write_yaml(file_path, data):
    with open(file_path, "w") as file:
        yaml.dump(data, file, default_flow_style=False, sort_keys=False)


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

KNOWN_DYNAMIXEL_VIDS = [
    0x0403,  # FTDI (U2D2, most common)
    0x16D0,  # MCS Electronics (some Robotis boards)
]


def auto_detect_port() -> str:
    """Auto-detect a Dynamixel adapter by USB vendor ID.

    Returns the port device string if exactly one known adapter is found,
    otherwise returns None.
    """
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    matches = [p for p in ports if p.vid in KNOWN_DYNAMIXEL_VIDS]

    if len(matches) == 1:
        port = matches[0]
        print(f"Auto-detected Dynamixel adapter: {port.device} "
              f"({port.description or 'unknown'})")
        return port.device

    return None


def get_and_choose_port() -> str:
    """
    Interactive terminal UI to choose from available USB devices with arrow key navigation.
    Returns the selected port or None if the user quits.
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
