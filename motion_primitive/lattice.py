import numpy as np
from collections import deque
from heapq import heappush, heappop
from car_data import L, max_steering_angle


 

def heuristic(x, y, goal):
    """Euclidean distance as a heuristic."""
    return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)

# Lattice Planner Helper Functions
def is_within_bounds(x, y, grid_width=20, grid_height=20):
    """Check if the (x, y) position is within grid bounds."""
    return 0 <= x < grid_width and 0 <= y < grid_height

def is_collision_free(x, y):
    """Check if the given (x, y) position is free of obstacles using the environment."""
    # This is a placeholder 
    # Use environment's methods to check the current position in the simulation.
    return True

def car_model(x, y, theta, v, steering_angle, dt=0.25):
    """Car dynamics model (bicycle model)."""
    if steering_angle != 0:
        # R = L / np.tan(steering_angle)  # Turning radius
        # dtheta = v / R
        dtheta = v / L * np.tan(steering_angle)
    else:
        dtheta = 0 #go straigth

    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    
    return x + dx * dt, y + dy * dt, theta + dtheta * dt



def lattice_planner(start, goal, target_speed, max_steps=100, max_steering_angle=np.radians(30)):
    """Lattice planner using A* with travel distance as cost."""
    queue = []
    heappush(queue, (0, start[0], start[1], start[2], 0, []))  # (priority, x, y, theta, cost_so_far, path)
    visited = set()
    visited.add((start[0], start[1], start[2]))
    
    while queue:
        _, x, y, theta, cost_so_far, path = heappop(queue)
        
        # Goal check with tolerance
        if np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2) < 0.1:
            return path
        
        for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 3):
            new_x, new_y, new_theta = car_model(x, y, theta, target_speed, steering_angle)  # Velocity fixed at 3 m/s
            step_distance = np.sqrt((new_x - x)**2 + (new_y - y)**2)  # Distance traveled in this step
            new_cost = cost_so_far + step_distance  # Accumulate distance
            
            if is_collision_free(new_x, new_y) and (new_x, new_y, new_theta) not in visited:
                visited.add((new_x, new_y, new_theta))
                new_path = path + [(new_x, new_y, new_theta)]
                priority = new_cost + heuristic(new_x, new_y, goal)  # A* priority
                
                heappush(queue, (priority, new_x, new_y, new_theta, new_cost, new_path))
                
                if len(new_path) > max_steps:
                    return None  # Fail if path exceeds maximum allowed steps
    
    return None  # No valid path found
