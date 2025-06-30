# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
import os
def get_model_path(model_path=None):

    if model_path is None or model_path == "models":
        models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
        if not os.path.exists(models_dir):
            raise FileNotFoundError("\033[1;35mModels directory not found. Did you download them? If not find them at https://www.orcahand.com/downloads\033[0m")
        model_dirs = sorted(d for d in os.listdir(models_dir) if os.path.isdir(os.path.join(models_dir, d)))
        if len(model_dirs) == 0:
            raise FileNotFoundError("\033[1;35mNo model files found. Did you download them? If not find them at https://www.orcahand.com/downloads\033[0m")
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