from urdfenvs.urdf_environment import UrdfEnv
from urdfenvs.sensors.obstacles import SphereObstacle

def setup_environment():
    """
    Set up a URDF environment with static obstacles.
    """
    env = UrdfEnv()
    env.reset()

    # Add obstacles
    env.add_obstacle(SphereObstacle(name="static_obstacle_1", content_dict={
        "type": "sphere",
        "geometry": {"position": [5, 5, 0], "radius": 1.0},
        "rgba": [1, 0, 0, 1],  # Red obstacle
    }))

    env.add_obstacle(SphereObstacle(name="static_obstacle_2", content_dict={
        "type": "sphere",
        "geometry": {"position": [8, 3, 0], "radius": 1.0},
        "rgba": [0, 0, 1, 1],  # Blue obstacle
    }))

    print("Obstacle environment setup complete.")
    return env
