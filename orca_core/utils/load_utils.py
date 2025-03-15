import os 

def get_model_path(model_path=None):
    if model_path is None:
            models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "models")
            if not os.path.exists(models_dir):
                raise FileNotFoundError("Models directory not found. Did you download them? If not find them at https://www.orcahand.com/downloads")
            model_dirs = sorted(d for d in os.listdir(models_dir) if os.path.isdir(os.path.join(models_dir, d)))
            if len(model_dirs) == 0:
                raise FileNotFoundError("No model files found. Did you download them? If not find them at https://www.orcahand.com/downloads")
            model_path = os.path.join(models_dir, model_dirs[0])
    return model_path
    