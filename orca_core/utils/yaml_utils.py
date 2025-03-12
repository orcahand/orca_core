import yaml
import numpy as np

def update_yaml(file_path, key, value):
    """Reads a YAML file, updates a specific key, and writes it back."""
    if isinstance(value, np.ndarray):
        value = value.tolist()
    
    if isinstance(value, dict):
        value = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in value.items()}

    try:
        with open(file_path, 'r+') as file:
            data = yaml.safe_load(file) or {}  # Load existing data, default to empty dict
            data[key] = value  # Update only the given key
            file.seek(0)  # Move cursor to beginning
            yaml.dump(data, file, default_flow_style=False, sort_keys=False)  # Write updated content
            file.truncate()  # Remove any leftover content
    except FileNotFoundError:
        # If the file doesn't exist, create it
        with open(file_path, 'w') as file:
            yaml.dump({key: value}, file, default_flow_style=False, sort_keys=False)


def read_yaml(file_path):
    """Reads a YAML file and returns its content."""
    print(file_path)
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file) or None
    except FileNotFoundError:
        return {}