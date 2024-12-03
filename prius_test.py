import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel
from collections import deque
from heapq import heappush, heappop
import time
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

# Car parameters
L = 2.5  # Length of the car (in meters)
max_steering_angle = np.radians(30)  # Maximum steering angle (in radians)

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

# def lattice_planner(start, goal, env: UrdfEnv, max_steps=100, max_steering_angle=np.radians(30)):
    # """Lattice planner using BFS with Reeds-Shepp motion primitives."""
    # queue = deque([(start[0], start[1], start[2], 0, [])])  # (x, y, theta, cost, path)
    # visited = set()
    # visited.add((start[0], start[1], start[2]))
    
    # while queue:
    #     x, y, theta, cost, path = queue.popleft()
    #     print(x,y)
        
    #     # if goal[0] - 0.01 <= x <= goal[0] + 0.01 and goal[1] - 0.01 <= y <= goal[1] + 0.01:  # Goal reached
    #     if x == goal[0] and y == goal[1]:
    #         return path
        
    #     # Generate possible next steps using Reeds-Shepp-like motion primitives
    #     for steering_angle in np.linspace(-max_steering_angle, max_steering_angle, 3):  # 5 different steering angles
    #         new_x, new_y, new_theta = car_model(x, y, theta, 3, steering_angle)  # Assuming constant speed
    #         new_cost = cost + 1  # Simplified cost function (just number of steps)
            
    #         # Check if the new position is collision-free and not visited
    #         if is_collision_free(new_x, new_y, env) and (new_x, new_y, new_theta) not in visited:
    #             visited.add((new_x, new_y, new_theta))
    #             new_path = path + [(new_x, new_y, new_theta)]
                # queue.append((new_x, new_y, new_theta, new_cost, new_path))
                
                # if new_cost > max_steps:
                #     return None  # Return None if no solution is found within max_steps
    
    # return None  # Return None if goal is unreachable

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


def run_prius(n_steps=1000, render=False):
    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius=0.31265,
            wheel_distance=0.494,
            spawn_offset=np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            facing_direction='-x'
        )
    ]
    
    env = UrdfEnv(dt=0.01, robots=robots, render=render)
    pos0 = np.array([0, 0, 0])  # Initial position
    goal_position = np.array([10, 5, 0])  # Goal position
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
    
    for next_target in path:
        target_reached = False
        action = np.array([0.0, 0.0])  # [speed, steering angle]
        
        
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



import matplotlib.pyplot as plt

def plot_trajectory(path, goal_position, history):
    """
    Plots the trajectory of the car and the goal position.
    
    Parameters:
    - path: List of (x, y, theta) tuples representing the car's trajectory.
    - goal_position: (x, y, theta) tuple representing the goal position.
    """
    hx = np.array([ob['robot_0']['joint_state']['position'][0] for ob in history])
    hy = np.array([ob['robot_0']['joint_state']['position'][1] for ob in history])

    plt.figure(figsize=(10, 6))
    plt.plot(hx, hy, label="Trajectory", marker='o', linestyle='-', color='orange', markersize=5)

    path = np.array(path)
    
    # Extract x and y coordinates
    x = path[:, 0]
    y = path[:, 1]
    
    # plt.figure(figsize=(10, 6))
    
    # Plot the trajectory
    plt.plot(hx[0], hy[0], label="Trajectory", marker='o', linestyle='-', color='blue', markersize=5)
    plt.plot(x, y, label="Trajectory", marker='o', linestyle='-', color='blue', markersize=5)
    
    # Mark the goal position
    plt.scatter(goal_position[0], goal_position[1], color='red', label="Goal", s=100, marker='*')
    
    # Plot the starting point
    plt.scatter(hx[0], hy[0], color='green', label="Start", s=100, marker='D')
    
    # Add labels and title
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Car Trajectory")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # Keep the scale of x and y the same for better visualization
    
    # Show plot
    plt.show()



if __name__ == "__main__":
    run_prius(render=True)
