from urdfenvs.urdf_environment import UrdfEnv

def setup_environment():
    """
    Set up a basic URDF environment without obstacles.
    """
    env = UrdfEnv()
    env.reset()
    print("Base environment setup complete (no obstacles).")
    return env