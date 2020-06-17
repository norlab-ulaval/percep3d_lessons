import os

def get_root_path():
    path = os.getcwd()
    root_parent_path = path[:path.rfind("percep3d")]
    return path[:path.find("/", len(root_parent_path))]
