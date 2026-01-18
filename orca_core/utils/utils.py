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

################################################################################
### Model path utils ##########################################################
################################################################################

def get_model_path(model_path=None):

    if model_path is None or model_path == "models":
        models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
        if not os.path.exists(models_dir):
            raise FileNotFoundError("\033[1;35mModels directory not found. Did you delete them?")
        model_dirs = [d for d in os.listdir(models_dir) if os.path.isdir(os.path.join(models_dir, d))]
        model_dirs = sorted(model_dirs, key=lambda x: (x.lower() != 'right', x))
        if len(model_dirs) == 0:
            raise FileNotFoundError("\033[1;35mNo model files found. Did you delete them?")
        resolved_path = os.path.join(models_dir, model_dirs[0])
    else:
        if os.path.isabs(model_path):
            resolved_path = model_path # Absolute path provided
        else:
            package_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__))) #Relative path provided
            resolved_path = os.path.join(package_root, model_path)
    
    if not os.path.exists(resolved_path):
        raise FileNotFoundError(f"\033[1;35mModel directory not found: {resolved_path}\033[0m")
    
    config_file = os.path.join(resolved_path, "config.yaml")
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"\033[1;35mconfig.yaml not found in {resolved_path}. Did you specify the correct model directory?\033[0m")
    
    print("Using model path: \033[1;32m{}\033[0m".format(resolved_path))
    return resolved_path

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
            y_pos = i + 2
            marker = "(x)" if i == selected_idx else "( )"
            
            if i == selected_idx:
                stdscr.attron(curses.A_REVERSE)
                stdscr.addstr(y_pos, 0, f"{i+1:2d}. {marker} {port.device}")
                stdscr.attroff(curses.A_REVERSE)
            else:
                stdscr.addstr(y_pos, 0, f"{i+1:2d}. {marker} {port.device}")
            
            if y_pos + 1 < height:
                stdscr.addstr(y_pos + 1, 4, f"{port.description or 'No description'}")
            if y_pos + 2 < height:
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