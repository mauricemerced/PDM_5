import numpy as np
from collections import deque
from heapq import heappush, heappop
from car_data import L, max_steering_angle


 

# def heuristic(x, y, goal):
#     """Euclidean distance as a heuristic."""
#     return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)

def heuristic(x, y, goal, theta, goal_theta, steering_angle, previous_steering_angle):
    distance_cost = np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
    orientation_cost = np.abs(goal_theta - theta)  # Penalize sharp differences in heading
    orientation_cost = min(orientation_cost, 2 * np.pi - orientation_cost)  # Normalize to [0, pi]

    # # Decrease orientation weight when far from the goal
    # orientation_weight = 1.5 if distance_cost < 5 else 0.5
    # return distance_cost + orientation_weight * orientation_cost

    # Penalize sharp steering changes
    steering_change_cost = np.abs(steering_angle - previous_steering_angle)
    
    # Weighted heuristic components
    orientation_weight = 1.5 if distance_cost < 5 else 0.5
    steering_weight = 1.0
    
    return distance_cost + orientation_weight * orientation_cost + steering_weight * steering_change_cost
    

# Lattice Planner Helper Functions
# def is_within_bounds(x, y, grid_width=20, grid_height=20):
#     """Check if the (x, y) position is within grid bounds."""
#     return 0 <= x < grid_width and 0 <= y < grid_height

def is_within_bounds(x, y, grid_width=50, grid_height=50):
    return -grid_width <= x <= grid_width and -grid_height <= y <= grid_height

def is_collision_free(x, y, env):
    """Check if the given (x, y) position is free of obstacles using the environment.
    param obstacles: List of obstacle positions
    """
    # tolerance = 0.5  # Radius around obstacles to avoid
    # for obs in obstacles:
    #     if np.sqrt((x - obs[0])**2 + (y - obs[1])**2) < tolerance:
    #         return False  # Collision detected
    # return True
    # This is a placeholder 
    # Use environment's methods to check the current position in the simulation.
    return True
    # return env.is_point_free([x, y, 0])


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

def discretize_state(x, y, theta, grid_res=1.0, theta_res=np.pi / 8):
    """Discretize state to limit the search space."""
    if np.sqrt(x**2 + y**2) < 5:  # Near the goal, refine resolution
        grid_res = 0.5
        theta_res = np.pi / 16
    grid_x = round(x / grid_res) * grid_res
    grid_y = round(y / grid_res) * grid_res
    grid_theta = round(theta / theta_res) * theta_res % (2 * np.pi)
    return (grid_x, grid_y, grid_theta)


def lattice_planner(start, targets, target_speed, env, max_steps=100, max_steering_angle=np.radians(30)):
    """Lattice planner with intermediate targets."""
    current_start = start
    full_path = []

    for goal in targets:
        partial_path = plan_single_target(current_start, goal, target_speed, env, max_steps, max_steering_angle)
        if partial_path is None:
            print("Failed to reach target:", goal)
            return None  # Exit if any sub-goal cannot be reached
        full_path.extend(partial_path)
        current_start = partial_path[-1]  # Update the start position for the next goal

    return full_path

def plan_single_target(start, goal, target_speed, env, max_steps=100, max_steering_angle=np.radians(30)):
    """Lattice planner using A* with travel distance as cost."""
    queue = []
    heappush(queue, (0, start[0], start[1], start[2], 0, [], 0.0))  # (priority, x, y, theta, cost_so_far, path, previous_steering_angle)
    visited = {}
    visited[tuple(start)] = ((round(start[0], 2), round(start[1], 2), round(start[2], 2)))
    
    while queue:
        _, x, y, theta, cost_so_far, path, previous_steering_angle  = heappop(queue)

        # Debug: Print the current state
        print(f"Checking position: x={x:.2f}, y={y:.2f}, theta={theta:.2f}, cost={cost_so_far:.2f}")
        
        # Goal check with tolerance
        if np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2) < 0.5 and abs(new_theta - theta) < np.pi / 8:
            if abs(theta - goal[2]) < np.radians(10):  # Check heading consistency
                print(f"Goal reached at x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
                return path
        

        # Dynamic resolution of steering angles
        distance_to_goal = np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
        if distance_to_goal < 3:  # Near the goal
            steering_samples = np.linspace(-max_steering_angle, max_steering_angle, 3)
        elif distance_to_goal < 10:  # Mid-range
            steering_samples = np.linspace(-max_steering_angle / 2, max_steering_angle / 2, 3)
        else:  # Far from the goal
            steering_samples = [0, -max_steering_angle / 3, max_steering_angle / 3]

        for steering_angle in steering_samples:
        # for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 3):
            # Speed-dependent steering limits
            dynamic_steering_limit = max_steering_angle * 0.75 if abs(target_speed) > 3.0 else max_steering_angle
            steering_angle = np.clip(steering_angle, -dynamic_steering_limit, dynamic_steering_limit)

            # # # # Smooth steering adjustment
            # # if 'previous_steering_angle' in locals():
            max_steering_change = np.radians(10)  # Limit per-step steering change
            delta_steering = steering_angle - previous_steering_angle
            adjusted_steering_angle = previous_steering_angle + np.clip(delta_steering, -max_steering_change, max_steering_change)
            previous_steering_angle = adjusted_steering_angle  # Update for the next iteration

            # Update the car's position
            new_x, new_y, new_theta = car_model(x, y, theta, target_speed, adjusted_steering_angle)  # Velocity fixed at 3 m/s
            # new_x, new_y, new_theta = car_model(x, y, theta, target_speed, adjusted_steering_angle)  # Velocity fixed at 3 m/s
            

            # rounded_state = (round(new_x, 2), round(new_y, 2), round(new_theta, 2)) # This helps avoid infinite loops or redundant exploration
            rounded_state = discretize_state(new_x, new_y, new_theta)
            print(f"Grid State: {discretize_state(new_x, new_y, new_theta)}")
            
            # Debug: Print the new candidate state
            print(f"  Candidate: x={new_x:.2f}, y={new_y:.2f}, theta={new_theta:.2f}, steering={steering_angle:.2f}")

            # if is_collision_free(new_x, new_y, env) and rounded_state not in visited:
            if is_within_bounds(new_x, new_y) and is_collision_free(new_x, new_y, env):
                if rounded_state not in visited or new_cost < visited[rounded_state]:
                    # visited.add((new_x, new_y, new_theta))
                    
                    step_distance = np.sqrt((new_x - x)**2 + (new_y - y)**2)  # Distance traveled in this step
                    new_cost = cost_so_far + step_distance  # Accumulate distance
                    
                    # priority = new_cost + heuristic(new_x, new_y, goal)  # A* priority
                    # priority = new_cost + heuristic(new_x, new_y, goal, new_theta, theta)  # A* priority > added parameters, still needs refinement
                    priority = new_cost + heuristic(new_x, new_y, goal, new_theta, theta, steering_angle, previous_steering_angle)

                    visited[rounded_state] = new_cost
                    new_path = path + [(new_x, new_y, new_theta)]
                    
                    # heappush(queue, (priority, new_x, new_y, new_theta, new_cost, new_path, adjusted_steering_angle))
                    heappush(queue, (priority, new_x, new_y, new_theta, new_cost, new_path, steering_angle))
                if len(queue) > 10000:  # Arbitrary limit to prevent infinite expansion
                    print("Search limit exceeded. No valid path found.")
                    return None
                if len(new_path) > max_steps:
                    return None  # Fail if path exceeds maximum allowed steps
    
    return None  # No valid path found
