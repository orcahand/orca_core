import os

def get_model_path(model_path=None):
    if model_path is None:
        models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
        if not os.path.exists(models_dir):
            raise FileNotFoundError("\033[1;35mModels directory not found. Did you download them? If not find them at https://www.orcahand.com/downloads\033[0m")
        model_dirs = sorted(d for d in os.listdir(models_dir) if os.path.isdir(os.path.join(models_dir, d)))
        if len(model_dirs) == 0:
            raise FileNotFoundError("\033[1;35mNo model files found. Did you download them? If not find them at https://www.orcahand.com/downloads\033[0m")
        model_path = os.path.join(models_dir, model_dirs[0])
    model_files = os.listdir(model_path)
    if "config.yaml" not in model_files:
        raise FileNotFoundError(f"\033[1;35mModel file not found in {model_path}. Did you specify the correct model directory? The current model directory is {model_path}\033[0m")
    return model_path