import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv

import time
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from analysis import plot_trajectory, plot_lattice_trajectories,plot_motion_trajectory
from motion_primitive.lattice import lattice_planner, potential_trajectories
from car_data import L, max_steering_angle, robots, target_speed, n_points



  
"""
Use `potential_trajectories` to create a set of predefined paths based on steering angles.

Set the robot's starting position and desired goal location.

Use `lattice_planner` to select the best path from the generated trajectories.

Call `ride_prius` to move the robot along the planned path.  
Adjust speed and steering based on heading error and target waypoints.  
Record motion history.

Verify if the robot reaches each waypoint within a defined tolerance.

Plot lattice trajectories, the planned path, and the robot's motion trajectory.

Print the path, lattice steering angles, and completion status.
"""


def ride_prius(start, path):
    env = UrdfEnv(dt=0.005, robots=robots, render=True)
    ob = env.reset(pos=start)

    current_state = ob[0]['robot_0']['joint_state']['position']
    history = []
    action = np.array([0.0, 0.0])
    
    for next_target in path:
        target_reached = False
        
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
           
            # Limit steering angle
            current_heading = current_state[2]
            
            heading_error = (desired_theta - current_heading + np.pi) % (2 * np.pi) - np.pi
            action[1] = heading_error  # Steering angle adjustment

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
  
    return history


if __name__ == "__main__":

    lattice_trajectories = potential_trajectories(robots, False)
    plot_lattice_trajectories(lattice_trajectories)
   

   
    start = np.array([0, 0, 0]) 
    goal = np.array([-2, 4, 0]) 
    
    print("planning path..." )
    path, lattice = lattice_planner(start, goal, lattice_trajectories)
   
    

    if path is None:
        print("No valid path found.")
    else:
        print("Path found!")
        print("Path:", path)
        print("Lattice (steering angles):", lattice)
        history = ride_prius(start, path)
      
        
        # plot_trajectory(path, goal, history, ends)
        plot_motion_trajectory( goal, history, path)


