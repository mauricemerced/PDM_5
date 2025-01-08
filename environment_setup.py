from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.scene_examples.obstacles import *
import numpy as np
from car_data import robots, target_speed, max_steering_angle, car_model, find_boundingbox, car_urdf
from mpscenes.obstacles.box_obstacle import BoxObstacle
from visualization_utils import visualize_bounding_box


def create_basic_environment(render=True):
    """
    Creates a basic environment with a single robot and no obstacles.
    """
    # Create the environment with the robot
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    has_obstacles = False
    obstacle_dict = []

    return env, has_obstacles, obstacle_dict

def create_environment_with_static_obstacles(render=True, obstacles=True):
    """
    Creates an environment with a single robot and static obstacles.
    """
    # Create the environment with the robot(s) from car_data
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    # Define static obstacles
    if obstacles:

        print("Adding static obstacles...")
        env.add_obstacle(sphereObst2)           # Add a sphere 2
        # env.add_obstacle(cylinder_obstacle)     # add a cylinder
        env.add_obstacle(sphereObst1)           # add a sphere 1
        # env.add_obstacle(movable_obstacle)
        # env.add_obstacle(urdfObst1)
        # env.add_obstacle(dynamicSphereObst3)    # add a dynamic sphere 3
        # env.add_obstacle(cylinder_obstacle)     # add a cylinder

        print("Adding static obstacles...")
        static_obstacles = [
            {"type": "sphere", "position": [5, 5, 0], "radius": 1.0},
            {"type": "sphere", "position": [12, 7, 0], "radius": 1.0},
            {"type": "cylinder", "position": [10, 10, 0], "radius": 1.5},
            # {"type": "cylinder", "position": [20, 20, 0], "radius": 1.5},
            {"type": "cylinder", "position": [8, 3, 0], "radius": 1.5},
            {"type": "cylinder", "position": [10, 12, 0], "radius": 1.5},
            {"type": "cylinder", "position": [-10, 12, 0], "radius": 1.5},
            {"type": "cylinder", "position": [-10, -12, 0], "radius": 1.5},
        ]

        for obs in static_obstacles:
            if obs["type"] == "sphere":
                sphere_obstacle = SphereObstacle(name="sphere", content_dict={
                    'type': 'sphere',
                    'geometry': {'position': obs["position"], 'radius': obs["radius"]}
                })
                env.add_obstacle(sphere_obstacle)
                obstacle_dict.append({"x": obs["position"][0], "y": obs["position"][1], "radius": obs["radius"]})

            elif obs["type"] == "cylinder":
                cylinder_obstacle = CylinderObstacle(name="cylinder", content_dict={
                    'type': 'cylinder',
                    'geometry': {'position': obs["position"], 'radius': obs["radius"], 'height': 1.0}
                })
                env.add_obstacle(cylinder_obstacle)
                obstacle_dict.append({"x": obs["position"][0], "y": obs["position"][1], "radius": obs["radius"]})

    has_obstacles = True

    return env, has_obstacles, obstacle_dict

def create_environment_with_outer_walls(render=True, wall_length=30, wall_thickness=0.5):
    """
    Creates an environment with correctly aligned outer walls.

    Parameters:
    - render (bool): Whether to render the environment.
    - wall_length (float): Length of the workspace (assumed square).
    - wall_thickness (float): Thickness of the walls.

    Returns:
    - env: The environment with the walls added.
    - has_obstacles (bool): True since walls are added.
    - obstacle_dict (list): List of obstacles for collision checking.
    """
    # Create the environment
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize the obstacle dictionary
    has_obstacles = True

    # Define outer walls (box obstacles)
    outer_wall_obstacles = [
        # Right wall (aligned vertically on the right edge)
        {"type": "box", "position": [0.0, wall_length/ 2.0 , 0.4], "width": wall_thickness, "length": wall_length},
        # Left wall (aligned vertically on the left edge)
        {"type": "box", "position": [0.0, -wall_length/ 2.0 , 0.4], "width": wall_thickness, "length": wall_length},
        # Top wall (aligned horizontally on the top edge)
        {"type": "box", "position": [-wall_length / 2.0, 0.0, 0.4], "width": wall_length, "length": wall_thickness},
        # Bottom wall (aligned horizontally on the bottom edge)
        {"type": "box", "position": [wall_length / 2.0, 0.0, 0.4], "width": wall_length, "length": wall_thickness},
    ]

    # Add walls to the environment and obstacle dictionary
    for i, wall in enumerate(outer_wall_obstacles):
        box_obstacle = BoxObstacle(name=f"wall_{i}", content_dict={
            'type': 'box',
            'geometry': {
                'position': wall['position'],  # Center position of the wall
                'width': wall['width'],       # Wall width (thickness)
                'length': wall['length'],     # Wall length
                'height': 1.0                 # Wall height (arbitrary, can be adjusted)
            }
        })
        env.add_obstacle(box_obstacle)
        print(f"Added wall at position: {wall['position']}")

        # Add the wall to the obstacle dictionary for collision checking
        obstacle_dict.append({
            "x": float(wall["position"][0]),
            "y": float(wall["position"][1]),
            "width": wall["width"],
            "length": wall["length"],
        })

    return env, has_obstacles, obstacle_dict


def create_static2_environment(render=True, obstacles=True):
    """
    Creates a static environment with randomly placed obstacles within outer walls.
    """
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    # Outer wall dimensions
    outer_wall_length = 30
    wall_thickness = 0.1

    # Add outer walls
    outer_wall_obstacles_dicts = [
        {'position': [outer_wall_length / 2.0, 0.0, 0.4], 'length': outer_wall_length, 'width': wall_thickness},
        {'position': [0.0, outer_wall_length / 2.0, 0.4], 'length': wall_thickness, 'width': outer_wall_length},
        {'position': [0.0, -outer_wall_length / 2.0, 0.4], 'length': wall_thickness, 'width': outer_wall_length},
        {'position': [-outer_wall_length / 2.0, 0.0, 0.4], 'length': outer_wall_length, 'width': wall_thickness},
    ]

    for i, wall_dict in enumerate(outer_wall_obstacles_dicts):
        wall_obstacle = BoxObstacle(name=f"wall_{i}", content_dict={
            'type': 'box',
            'geometry': {
                'position': wall_dict['position'],
                'width': wall_dict['length'],
                'height': 0.8,
                'length': wall_dict['width'],
            }
        })
        env.add_obstacle(wall_obstacle)

        # Add walls to the obstacle dictionary
        obstacle_dict.append({
            "x": float(wall_dict["position"][0]),
            "y": float(wall_dict["position"][1]),
            "radius": float(wall_dict["width"] / 2)  # Approximate wall thickness as radius
        })

    # Randomly add obstacles within bounds
    if obstacles:
        num_obstacles = np.random.randint(1, 30)  # Random number of obstacles between 1 and 50
        print(f"Adding {num_obstacles} random obstacles...")

        for _ in range(num_obstacles):
            obstacle_type = np.random.choice(["sphere", "box", "cylinder"])
            x = np.random.uniform(-outer_wall_length / 2 + 1, outer_wall_length / 2 - 1)
            y = np.random.uniform(-outer_wall_length / 2 + 1, outer_wall_length / 2 - 1)
            z = 0.0
            radius = np.random.uniform(0.5, 2.0)  # Random radius/size for obstacles

            if obstacle_type == "sphere":
                sphere_obstacle = SphereObstacle(name=f"sphere_{x}_{y}", content_dict={
                    'type': 'sphere',
                    'geometry': {'position': [x, y, z], 'radius': radius}
                })
                env.add_obstacle(sphere_obstacle)
                obstacle_dict.append({"x": float(x), "y": float(y), "radius": float(radius)})

            elif obstacle_type == "cylinder":
                cylinder_obstacle = CylinderObstacle(name=f"cylinder_{x}_{y}", content_dict={
                    'type': 'cylinder',
                    'geometry': {'position': [x, y, z], 'radius': radius, 'height': 1.0}
                })
                env.add_obstacle(cylinder_obstacle)
                obstacle_dict.append({"x": float(x), "y": float(y), "radius": float(radius)})

            elif obstacle_type == "box":
                box_obstacle = BoxObstacle(name=f"box_{x}_{y}", content_dict={
                    'type': 'box',
                    'geometry': {
                        'position': [x, y, z + 0.4],
                        'width': radius * 2,
                        'height': 0.8,
                        'length': radius * 2
                    }
                })
                env.add_obstacle(box_obstacle)
                obstacle_dict.append({"x": float(x), "y": float(y), "radius": float(radius)})

    return env, has_obstacles, obstacle_dict

def load_environment(environment_type, render=True):
    """
    Loads the specified environment based on the environment type.
    """
    print(f"Loading environment type: {environment_type}")
    environments = {
        "basic": create_basic_environment,
        "static": create_environment_with_static_obstacles,
        "static2": create_static2_environment,
        "wall": create_environment_with_outer_walls,
        # "narrow": create_narrow_passage_environment,
        # "dynamic": create_dynamic_environment,
    }

    if environment_type not in environments:
        raise ValueError(f"Unknown environment type: {environment_type}")
    
    env, has_obstacles, obstacle_dict = environments[environment_type](render=render)
    print(f"Environment loaded successfully. Obstacles: {has_obstacles}")
    # min_corner, max_corner = find_boundingbox(car_urdf)  # Call the function to get the bounding box
    # visualize_bounding_box(env, min_corner, max_corner)

    return env, has_obstacles, obstacle_dict

    

if __name__ == "__main__":
    # Test environment selection
    selected_env = "static2"  # Change this to 'basic', 'static', 'static2' 'narrow', or 'dynamic'
    print(f"Testing {selected_env} environment...")
    env, has_obstacles, obstacle_dict = load_environment(selected_env)
    env.reset()

    print(f"Obstacles detected: {obstacle_dict}")
    
    # Default action for stationary model
    # default_action = np.array([0.0, 0.0])  # [velocity, steering angle]

    # Default action for high velocity and small steering
    default_action = np.array([5.0, 0.1])  # Set high velocity

    try:
        while True:
            env.step(default_action)  # Pass the action to step()
    except KeyboardInterrupt:
        print("Exiting simulation...")

# if __name__ == "__main__":
#     env = UrdfEnv(dt=0.005, robots=robots, render=True)
#     ob, *_ = env.reset(pos=np.array([2.0, 2.0, 0.0]))
#     action = np.array([5.0, 0.1])  # Set high velocity
#     while True:
#         env.step(action)



