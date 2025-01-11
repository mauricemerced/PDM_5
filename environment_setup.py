from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from urdfenvs.scene_examples.obstacles import *
import numpy as np
from car_data import robots, target_speed, max_steering_angle, car_model, find_boundingbox, car_urdf
from mpscenes.obstacles.box_obstacle import BoxObstacle
from visualization_utils import visualize_bounding_box
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt


def create_basic_environment(render=True):
    """
    Creates a basic environment with a single robot and no obstacles.
    """
    # Create the environment with the robot
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    has_obstacles = False
    obstacle_dict = []

    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (6.0, 6.0, 0.0)

    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def create_environment_with_static_obstacles(render=True, obstacles=True):
    """
    Creates an environment with a single robot and static obstacles.
    """
    # Create the environment with the robot(s) from car_data
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (15.0, 10.0, 0.0)

    # Define static obstacles
    if obstacles:
        print("Adding static obstacles...")
        static_obstacles = [
            {"type": "sphere", "position": [5, 5, 0], "radius": 1.0},
            {"type": "sphere", "position": [12, 7, 0], "radius": 1.0},
            {"type": "cylinder", "position": [10, 10, 0], "radius": 1.5},
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

    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

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

    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (3.0, 3.0, 0.0)

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

        print('Obstc_dict beofre obst dict wall appending:', obstacle_dict)
        # Add walls to the obstacle dictionary
        # obstacle_dict.append({
        #     "x": float(wall_dict["position"][0]),
        #     "y": float(wall_dict["position"][1]),
        #     "radius": float(wall_dict["width"] / 2)  # Approximate wall thickness as radius
        # })

        obstacle_dict.append({
            "x": float(wall_dict["position"][0]),
            "y": float(wall_dict["position"][1]),
            "width": wall_dict["width"],
            "length": wall_dict["length"],
        })

    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def create_random_static_environment(render=True, obstacles=True):
    """
    Creates a static environment with randomly placed obstacles within outer walls.
    """
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (10.0,0.0, 0.0)

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


        obstacle_dict.append({
            "x": float(wall_dict["position"][0]),
            "y": float(wall_dict["position"][1]),
            "width": wall_dict["width"],
            "length": wall_dict["length"],
        })



    # Randomly add obstacles within bounds
    if obstacles:
        num_obstacles = 15  # Random number of obstacles between 1 and 50
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
                obstacle_dict.append({"x": float(x), "y": float(y), "width":radius * 2, "length": radius * 2})

    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def create_static2_environment(render=True, obstacles=True):
    """
    Creates a static environment with multiple sphere obstacles within outer walls.
    """
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    start_rrt = (0.0, 0.0, 0.0)
    goal_rrt = (10.0, 10.0, 0.0)
    

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

        print('Obstc_dict beofre obst dict wall appending:', obstacle_dict)
        # Add walls to the obstacle dictionary
        # obstacle_dict.append({
        #     "x": float(wall_dict["position"][0]),
        #     "y": float(wall_dict["position"][1]),
        #     "radius": float(wall_dict["width"] / 2)  # Approximate wall thickness as radius
        # })

        obstacle_dict.append({
            "x": float(wall_dict["position"][0]),
            "y": float(wall_dict["position"][1]),
            "width": wall_dict["width"],
            "length": wall_dict["length"],
        })

        print('Obstc_dict after wall insertions:', obstacle_dict)
    # Define obstacle positions and radii
    obstacles = [
    {"type": "cylinder", "position": [8, 3, 0], "radius": 1.5},
    {"type": "cylinder", "position": [3, 3, 0], "radius": 1.5},
    {"type": "cylinder", "position": [0, 7, 0], "radius": 1.5},
    {"type": "cylinder", "position": [-7, 0, 0], "radius": 1.5},
    {"type": "cylinder", "position": [9, 9, 0], "radius": 1.5},
    {"type": "cylinder", "position": [-5, -6, 0], "radius": 1.5},
    {"type": "cylinder", "position": [-2, 8, 0], "radius": 1.5},
    {"type": "cylinder", "position": [-10, 7, 0], "radius": 1.5},
    {"type": "cylinder", "position": [-12, 14, 0], "radius": 1.5},
    {"type": "cylinder", "position": [7, -7, 0], "radius": 1.5},
    ]
    
    if obstacles:
        for obs in obstacles:
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

    print(obstacle_dict)

    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt
            
def create_simple_maze(render=True, obstacles=True):
    """
    Creates a static environment with a simple maze-like structure within outer walls.
    """
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    start_rrt = (10.0, -10.0, 0.0)
    goal_rrt = (2.0, 2.0, 0.0)

    # Outer wall dimensions
    outer_wall_length = 30
    wall_thickness = 0.1
    wall_height = 0.8

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
            "width": wall_dict["width"],
            "length": wall_dict["length"],
        })

    # Maze-specific inner obstacles
    maze_obstacles = [
        # {'position': [5.0, 0.0, 0.4], 'length': 10.0, 'width': 0.5},
        # {'position': [-5.0, -5.0, 0.4], 'length': 8.0, 'width': 0.5},
        {'position': [7.5, 7.5, 0.4], 'length': 0.5, 'width': 15.0}, # right
        {'position': [-7.5, 0.0, 0.4], 'length': 0.5, 'width': 15.0}, # middle
        {'position': [7.5, -7.5, 0.4], 'length': 0.5, 'width': 15.0}, # left 
    ]

    for i, obstacle in enumerate(maze_obstacles):
        maze_obstacle = BoxObstacle(name=f"maze_wall_{i}", content_dict={
            'type': 'box',
            'geometry': {
                'position': obstacle['position'],
                'width': obstacle['length'],
                'height': wall_height,
                'length': obstacle['width'],
            }
        })
        env.add_obstacle(maze_obstacle)

        # # Add to obstacle dictionary
        # obstacle_dict.append({
        #     "x": float(obstacle["position"][0]),
        #     "y": float(obstacle["position"][1]),
        #     "radius": float(max(obstacle["length"], obstacle["width"]) / 2),
        # })

        obstacle_dict.append({
            "x": float(obstacle["position"][0]),
            "y": float(obstacle["position"][1]),
            "width": obstacle["width"],
            "length": obstacle["length"],
        })
    
    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def create_narrow_passage_environment(render=True, obstacles=True):
    """
    Creates a static environment with a simple narrow passage structure within outer walls.
    """
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    obstacle_dict = []  # Initialize obstacle dictionary
    has_obstacles = obstacles

    start_rrt = (10.0, -10.0, 0.0)
    goal_rrt = (2.0, 2.0, 0.0)

    # Outer wall dimensions
    outer_wall_length = 30
    wall_thickness = 0.1
    wall_height = 0.8

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

    # Maze-specific inner obstacles
    narrow_passage_obstacles = [
        # {'position': [5.0, 0.0, 0.4], 'length': 10.0, 'width': 0.5},
        # {'position': [-5.0, -5.0, 0.4], 'length': 8.0, 'width': 0.5},
        {'position': [7.5, 7.5, 0.4], 'length': 0.5, 'width': 15.0}, # right
        {'position': [-7.5, 0.0, 0.4], 'length': 0.5, 'width': 15.0}, # middle
        {'position': [7.5, -7.5, 0.4], 'length': 0.5, 'width': 15.0}, # left 
    ]

    for i, obstacle in enumerate(narrow_passage_obstacles):
        maze_obstacle = BoxObstacle(name=f"maze_wall_{i}", content_dict={
            'type': 'box',
            'geometry': {
                'position': obstacle['position'],
                'width': obstacle['length'],
                'height': wall_height,
                'length': obstacle['width'],
            }
        })
        env.add_obstacle(maze_obstacle)

        # Add to obstacle dictionary
        obstacle_dict.append({
            "x": float(obstacle["position"][0]),
            "y": float(obstacle["position"][1]),
            "radius": float(max(obstacle["length"], obstacle["width"]) / 2),
        })
    
    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def load_environment(environment_type, render=True):
    """
    Loads the specified environment based on the environment type.
    """
    print(f"Loading environment type: {environment_type}")
    environments = {
        "basic": create_basic_environment,
        "static": create_environment_with_static_obstacles,
        "random": create_random_static_environment,
        "wall": create_environment_with_outer_walls,
        "static2": create_static2_environment,
        "simple": create_simple_maze,
        "narrow": create_narrow_passage_environment,
        # "dynamic": create_dynamic_environment,
    }

    if environment_type not in environments:
        raise ValueError(f"Unknown environment type: {environment_type}")
    
    env, has_obstacles, obstacle_dict, start_rrt, goal_rrt = environments[environment_type](render=render)
    print(f"Environment loaded successfully. Obstacles: {has_obstacles}")
    print(f"Plotting environment {env}")
    plot_environment(environment_type, obstacle_dict, start_rrt, goal_rrt)


    return env, has_obstacles, obstacle_dict, start_rrt, goal_rrt

def plot_environment(environment_type, obstacle_dict, start, goal, render=True, grid_width=30, grid_height=30):
    plt.figure(figsize=(10, 10))

    # plot the obstacles
    for idx, obs in enumerate(obstacle_dict):
        label = f"Obstacle {idx + 1}"
        if "radius" in obs: 
            circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='r', fill=True, alpha=0.5)
            plt.gca().add_artist(circle)
            plt.text(obs["x"], obs["y"], f"{idx + 1}", fontsize=8, ha='center', va='center', color='black')
            # plt.gca().add_patch(circle)
        elif "width" in obs and "length" in obs:  # Rectangular obstacle (walls)
            # Calculate the bottom-left corner for the rectangle
            bottom_left_x = obs["x"] - obs["width"] / 2
            bottom_left_y = obs["y"] - obs["length"] / 2
            rectangle = plt.Rectangle((bottom_left_x, bottom_left_y), obs["width"], obs["length"], 
                                       color='g', fill=True, alpha=0.5)
            plt.gca().add_artist(rectangle)
            plt.text(obs["x"], obs["y"], f"{idx + 1}", fontsize=8, ha='center', va='center', color='black')
            # plt.gca().add_patch(rectangle)

    # Plot start and Goal
    plt.plot(start[0], start[1], 'bo', label="Start")
    plt.plot(goal[0], goal[1], 'ro', label="Goal")

    plt.xlim(-grid_width/2, grid_width/2)
    plt.ylim(-grid_height/2, grid_height/2)
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.title(f"{environment_type} plot")
    plt.legend()
    plt.grid(True)

    # Save the plot instead of showing it
    plt.savefig(f"{environment_type}.png")
    print(f"Environment plot saved as '{environment_type}.png'")

    plt.close()  # Close the figure to free memory

if __name__ == "__main__":
    # Test environment selection
    selected_env = "random"  # Change this to 'basic', 'static', 'wall', 'random', 'static2' or 'simple'
    print(f"Testing {selected_env} environment...")
    env, has_obstacles, obstacle_dict, start_rrt, goal_rrt = load_environment(selected_env)
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
