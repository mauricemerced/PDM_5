import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv

import time
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from analysis import plot_trajectory
from motion_primitive.lattice import lattice_planner

from car_data import L, max_steering_angle, robots

def run_prius(robot, n_steps=1000, render=False):
    
    
    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    pos0 = np.array([0, 0, 0])  # Initial position
    goal_position = np.array([-10, -1, 0])  # Goal position
    ob = env.reset(pos=pos0)
    
    target_speed = 5.0  # Fixed target speed
    print("Planning path...")
    path = lattice_planner(pos0, goal_position, target_speed)
    print("Path: ", path)

    
    if path is None:
        print("No valid path found.")
        return

    for idx, target in enumerate(path):
        marker_dict = {
            "type": "sphere",
            "geometry": {
                "position": [float(target[0]), float(target[1]), float(1.0)],  # Cast to float
                "radius": float(0.05)  # Cast to float
            },
            "rgba": [float(0.3), float(0.5), float(0.6), float(1.0)],  # Cast to float
        }
        sphere_marker = SphereObstacle(name=f"sphere_marker_{idx}", content_dict=marker_dict)
        env.add_obstacle(sphere_marker)
    
        
    
    
    current_state = ob[0]['robot_0']['joint_state']['position']
    history = []
    
    print("Executing path...")
    action = np.array([0.0, 0.0])
    for next_target in path:
        target_reached = False
          # [speed, steering angle]
        
        
        # Gradually accelerate and adjust steering
        while not target_reached:
            # Compute direction to target
            dx = next_target[0] - current_state[0]
            dy = next_target[1] - current_state[1]
            distance = np.sqrt(dx**2 + dy**2)
            
            # Adjust speed to target value
            if action[0] < target_speed:
                action[0] += 0.1  # Increment speed
            
            # Calculate desired steering angle
            desired_theta = np.arctan2(dy, dx)
            action[1] = desired_theta - current_state[2]  # Adjust to face target
            
            # Limit steering angle
            action[1] = np.clip(action[1], -max_steering_angle, max_steering_angle)
            
            ob, *_ = env.step(action)
            history.append(ob)
            
            current_state = ob['robot_0']['joint_state']['position']
            
            # Check if target is reached within a tolerance
            if distance < 0.5:  # 0.5 meters tolerance
                target_reached = True
        
        print(f"Reached target: {next_target}")
    
    print("Goal reached!")

    plot_trajectory(path, goal_position, history)
    time.sleep(15)
    # env.close()
    return history





if __name__ == "__main__":
    run_prius(robots, render=True)
