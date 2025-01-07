from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
# from urdfenvs.scene_examples.obstacles import (
#     sphereObst2,
#     dynamicSphereObst1,
#     cylinder_obstacle,
#     wall_obstacles,
# )
from urdfenvs.scene_examples.obstacles import *
import numpy as np
from car_data import robots, target_speed, max_steering_angle, car_model
from mpscenes.obstacles.box_obstacle import BoxObstacle

def create_basic_environment(render=True):
    """
    Creates a basic environment with a single robot and no obstacles.
    """
    # Create the environment with the robot
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    return env

def create_environment_with_static_obstacles(render=True, obstacles=True):
    """
    Creates an environment with a single robot and static obstacles.
    """
    # Create the environment with the robot(s) from car_data
    env = UrdfEnv(dt=0.005, robots=robots, render=render)
    print("Environment initialized with robots.")

    # Define static obstacles (boxes and spheres)
    # Box obstacle
    if obstacles:
        print("Adding static obstacles...")
        env.add_obstacle(sphereObst2)           # Add a sphere 2
        env.add_obstacle(cylinder_obstacle)     # add a cylinder
        env.add_obstacle(sphereObst1)           # add a sphere 1
        # env.add_obstacle(movable_obstacle)
        # env.add_obstacle(urdfObst1)
        # env.add_obstacle(dynamicSphereObst3)    # add a dynamic sphere 3
        env.add_obstacle(cylinder_obstacle)     # add a cylinder
        
        # for wall in wall_obstacles:
        #     print(f"Adding wall: {wall}")
        #     env.add_obstacle(wall)

        # Adding a wall with the manual method:

    outer_wall_length = 30
    outer_wall_obstacles_dicts = [
    {
        'type': 'box', 
         'geometry': {
             'position': [outer_wall_length/2.0, 0.0, 0.4], 'width': outer_wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [outer_wall_length/2.0, 0.0, 0.4],
            'width': outer_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [outer_wall_length/2.0, 0.0, 0.4],
            'width': outer_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, outer_wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': outer_wall_length
        },
        'high': {
            'position' : [0.0, outer_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': outer_wall_length,
        },
        'low': {
            'position' : [0.0, outer_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': outer_wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -outer_wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': outer_wall_length
        },
        'high': {
            'position' : [0.0, -outer_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': outer_wall_length,
        },
        'low': {
            'position' : [0.0, -outer_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': outer_wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-outer_wall_length/2.0, 0.0, 0.4], 'width': outer_wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-outer_wall_length/2.0, 0.0, 0.4],
            'width': outer_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-outer_wall_length/2.0, 0.0, 0.4],
            'width': outer_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

    inner_wall_length = 10
    inner_wall_obstacles_dicts = [
    {
        'type': 'box', 
         'geometry': {
             'position': [inner_wall_length/2.0, 0.0, 0.4], 'width': inner_wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [inner_wall_length/2.0, 0.0, 0.4],
            'width': inner_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [inner_wall_length/2.0, 0.0, 0.4],
            'width': inner_wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, inner_wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': inner_wall_length
        },
        'high': {
            'position' : [0.0, inner_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': inner_wall_length,
        },
        'low': {
            'position' : [0.0, inner_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': inner_wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -inner_wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': inner_wall_length
        },
        'high': {
            'position' : [0.0, -inner_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': inner_wall_length,
        },
        'low': {
            'position' : [0.0, -inner_wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': inner_wall_length,
        },
    },
    # {
    #     'type': 'box', 
    #      'geometry': {
    #          'position': [-inner_wall_length/2.0, 0.0, 0.4], 'width': inner_wall_length, 'height': 0.8, 'length': 0.1
    #     },
    #     'high': {
    #         'position' : [-inner_wall_length/2.0, 0.0, 0.4],
    #         'width': inner_wall_length,
    #         'height': 0.8,
    #         'length': 0.1,
    #     },
    #     'low': {
    #         'position' : [-inner_wall_length/2.0, 0.0, 0.4],
    #         'width': inner_wall_length,
    #         'height': 0.8,
    #         'length': 0.1,
    #     },
    # },
]
    
    outer_wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(outer_wall_obstacles_dicts)]    
    for wall in outer_wall_obstacles:
        env.add_obstacle(wall)
    
    inner_wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(inner_wall_obstacles_dicts)]    
    for wall in inner_wall_obstacles:
        env.add_obstacle(wall)

    #     wall = {
    # "type": "box",
    # "position": [5.0, 10.0, 0.5],  # Center of the wall
    # "size": [0.1, 10.0, 1.0],      # Dimensions of the wall
    #     }
    #     env.add_obstacle(**wall)
            

    # # Define walls along the edges of the environment
    #     edge_walls = [
    #         {"type": "box", "position": [0.0, 5.0, 0.5], "size": [0.1, 10.0, 1.0]},  # Left wall
    #         {"type": "box", "position": [10.0, 5.0, 0.5], "size": [0.1, 10.0, 1.0]},  # Right wall
    #         {"type": "box", "position": [5.0, 0.0, 0.5], "size": [10.0, 0.1, 1.0]},  # Bottom wall
    #         {"type": "box", "position": [5.0, 10.0, 0.5], "size": [10.0, 0.1, 1.0]},  # Top wall
    #     ]

    #     # Add walls to the environment
    #     for wall in edge_walls:
    #         env.add_obstacle(**wall)

    return env

def load_environment(environment_type, render=True):
    """
    Loads the specified environment based on the environment type.
    """
    print(f"Loading environment type: {environment_type}")
    environments = {
        "basic": create_basic_environment,
        "static": create_environment_with_static_obstacles,
        # "narrow": create_narrow_passage_environment,
        # "dynamic": create_dynamic_environment,
    }

    if environment_type not in environments:
        raise ValueError(f"Unknown environment type: {environment_type}")
    
    env = environments[environment_type](render=render)
    print("Environment loaded successfully.")

    return env

    

if __name__ == "__main__":
    # Test environment selection
    selected_env = "static"  # Change this to 'basic', 'static', 'narrow', or 'dynamic'
    print(f"Testing {selected_env} environment...")
    env = load_environment(selected_env)
    env.reset()

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



