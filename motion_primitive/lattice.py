import numpy as np
from collections import deque
from heapq import heappush, heappop
from car_data import L, max_steering_angle, car_model
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import L, max_steering_angle, robots, target_speed, n_points
import itertools

def heuristic(x, y, goal):
    """Euclidean distance as a heuristic."""
    return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)


def is_collision_free(x, y):
    """Check if the given (x, y) position is free of obstacles using the environment."""
    # This is a placeholder 
    # Use environment's methods to check the current position in the simulation.
    return True

def potential_trajectories(robot , render_):
    """
    Simulates potential trajectories for the prius based on various steering angles.

    Args:
        robot: The robot model used in the environment.
        render_: Boolean flag to render the environment visually.

    Returns:
        lattice_trajectories: A list of tuples where each tuple contains:
            - The target steering angle.
            - The simulated trajectory as a list of observations.

    Functionality:
        - Resets the environment for each steering angle.
        - Simulates the robot's motion under constant speed and specific steering inputs.
        - Records the trajectory of the robot for each steering angle.
    """
    DT = 0.005
    n_seconds=1.

    env = UrdfEnv(dt=0.005, robots=robots, render=render_)
   
    action = np.array([1., 0])
  
    pos0 = np.array([0, 0, 0])
    ob, *_  = env.reset(pos=pos0)

    points = []

    steering_angles = np.linspace(-np.degrees(max_steering_angle), np.degrees(max_steering_angle), n_points).tolist()
    
    lattice_trajectories = []

    for target_angle in steering_angles:
        print(target_angle, " ---------------------------------------------------------------")
        speed = ob['robot_0']['joint_state']['forward_velocity'][0]#1
        action = np.array([1., 0])
        trajectory = []
        ob, *_  = env.reset(pos=pos0)  # Reset environment for each steering angle

        end_time: Optional[float] = None
        state: int = 1

        for step in itertools.count(1):
            ob, *_ = env.step(action)

            if state == 1:
                if ob['robot_0']['joint_state']['forward_velocity'][0] >= target_speed:
                    print("speed reached")
                    state = 2
                else:
                    action[0] += 0.05

            if state == 2:
                action[0] = target_speed
                if target_angle == -25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-25)):
                    action[1] = -1.25
                elif target_angle == -12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(-12.5)):
                    action[1] = -1.25
                elif target_angle == 25 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(25)):
                    action[1] = 1.25
                elif target_angle == 12.5 and abs(ob['robot_0']['joint_state']['steering']) < abs(np.radians(12.5)):
                    action[1] = 1.25
                else:
                    action[1] = 0

                if abs(ob['robot_0']['joint_state']['steering']) >= abs(np.radians(target_angle)):
                    state = 3
                    end_time = step * DT + n_seconds

            if state == 3:
                action[0] = target_speed
                action[1] = 0.  # Stop steering
                trajectory.append(ob)

                if step * DT > end_time:
                    break
        
        
        lattice_trajectories.append((target_angle, trajectory))
    

    env.close()
    return lattice_trajectories




def lattice_planner(start, goal, lattice_trajectories, max_steps=10000):
    """
    Optimized lattice planner using A* with travel distance as cost.
    This version uses only the last state of each trajectory for faster performance.

    Parameters:
        start (tuple): Starting state (x, y, theta).
        goal (tuple): Goal state (x, y, theta).
        lattice_trajectories (list): Precomputed lattice trajectories [(steering_angle, trajectory), ...].
        max_steps (int): Maximum number of steps to allow.

    Returns:
        tuple: (path, lattice) where path is a list of (x, y, theta) states and lattice is a list of steering angles.
    """
    # Priority queue for A* search
    queue = []
    heappush(queue, (0, start[0], start[1], start[2], 0, [], []))  # (priority, x, y, theta, cost_so_far, path, lattice)
    visited = set()
    visited.add((start[0], start[1], start[2]))

    while queue:
        _, x, y, theta, cost_so_far, path, lattice = heappop(queue)

        # Check if we've reached the goal (with tolerance)
        if np.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2) < 0.5:
            return path, lattice

        # Expand all lattice trajectories from the current state
        for target_angle, trajectory in lattice_trajectories:
            # Use only the final state of the trajectory
            final_state = trajectory[-1]
            local_x = final_state['robot_0']['joint_state']['position'][0]
            local_y = final_state['robot_0']['joint_state']['position'][1]
            local_theta = final_state['robot_0']['joint_state']['position'][2]

            # Transform to global coordinates
            cos_theta, sin_theta = np.cos(theta), np.sin(theta)
            global_x = x + cos_theta * local_x - sin_theta * local_y
            global_y = y + sin_theta * local_x + cos_theta * local_y
            global_theta = theta + local_theta

            # Calculate step cost (Euclidean distance)
            trajectory_cost = np.sqrt((global_x - x) ** 2 + (global_y - y) ** 2)

            # Check for collision-free path (based on the final state)
            if not is_collision_free(global_x, global_y):
                continue  # Skip if collision is detected

            # Heuristic cost (straight-line distance to goal)
            heuristic_cost = np.sqrt((goal[0] - global_x) ** 2 + (goal[1] - global_y) ** 2)

            # Update the priority queue if the state hasn't been visited
            if (global_x, global_y, global_theta) not in visited:
                visited.add((global_x, global_y, global_theta))
                new_path = path + [(global_x, global_y, global_theta)]
                new_lattice = lattice + [target_angle]
                priority = cost_so_far + trajectory_cost + heuristic_cost
                heappush(queue, (priority, global_x, global_y, global_theta, cost_so_far + trajectory_cost, new_path, new_lattice))

        # Stop if max steps are exceeded
        if len(path) > max_steps:
            break

    return None  # No valid path found