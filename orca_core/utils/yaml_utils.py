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
            data = yaml.safe_load(file) or {} 
            data[key] = value 
            file.seek(0) 
            yaml.dump(data, file, default_flow_style=False, sort_keys=False) 
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