import numpy as np
from collections import deque
from heapq import heappush, heappop
from car_data import L, max_steering_angle, car_model


def heuristic(x, y, goal):
    """Euclidean distance as a heuristic."""
    return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)



def is_collision_free(x, y):
    """Check if the given (x, y) position is free of obstacles using the environment."""
    # This is a placeholder 
    # Use environment's methods to check the current position in the simulation.
    return True





def lattice_planner(start, goal, target_speed, max_steps=100, max_steering_angle=max_steering_angle):
    """Lattice planner using A* with travel distance as cost."""
    queue = []
    heappush(queue, (0, start[0], start[1], start[2], 0, [], []))  # (priority, x, y, theta, cost_so_far, path)
    visited = set()
    visited.add((start[0], start[1], start[2]))
    
    while queue:
        _, x, y, theta, cost_so_far, path, lattice = heappop(queue)
        
        # Goal check with tolerance
        if np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2) < 0.1:
            return path, lattice
        
        for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 5):
            new_x, new_y, new_theta = car_model(x, y, theta, target_speed, steering_angle)  
            step_distance = np.sqrt((new_x - x)**2 + (new_y - y)**2)  # Distance traveled in this step
            new_lattice = lattice + [steering_angle]
            new_cost = cost_so_far + step_distance  # Accumulate distance
            
            if is_collision_free(new_x, new_y) and (new_x, new_y, new_theta) not in visited:
                visited.add((new_x, new_y, new_theta))
                new_path = path + [(new_x, new_y, new_theta)]
                priority = new_cost + heuristic(new_x, new_y, goal)  # A* priority
                
                heappush(queue, (priority, new_x, new_y, new_theta, new_cost, new_path, new_lattice))
                
                if len(new_path) > max_steps:
                    return None  # Fail if path exceeds maximum allowed steps
    
    return None  # No valid path found




def lattice_planner2(start, goal, lattice_trajectories, max_steps=10000):
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