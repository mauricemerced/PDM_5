import matplotlib.pyplot as plt
import pybullet as p
from RRT_star import rrt_star_with_tree, plot_tree, is_collision_free
import numpy as np
from car_data import robots, target_speed
import time
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import robots, target_speed, max_steering_angle, car_model
import math
from environment_setup_new import load_environment

def calculate_initial_heading(start_pos, next_pos):
    """Calculate initial heading based on first path segment."""
    dx = next_pos[0] - start_pos[0]
    dy = next_pos[1] - start_pos[1]
    return np.arctan2(dy, dx)

def interpolate_path(path, step_size=0.5):
    """Interpolate path with heading information."""
    interpolated_path = []
    
    # Add first point with calculated heading
    initial_heading = calculate_initial_heading(path[0], path[1])
    interpolated_path.append((path[0][0], path[0][1], initial_heading))
    
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        # Calculate heading for this segment
        heading = np.arctan2(end[1] - start[1], end[0] - start[0])
        
        dist = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        num_steps = max(1, int(dist / step_size))
        
        for step in range(1, num_steps):
            x = start[0] + (end[0] - start[0]) * step / num_steps
            y = start[1] + (end[1] - start[1]) * step / num_steps
            interpolated_path.append((x, y, heading))
            
        interpolated_path.append((end[0], end[1], heading))
    return interpolated_path

def is_path_clear(start, end, obstacles, checks=10):
    """Check if straight path between points is collision-free."""
    for i in range(checks):
        t = i / float(checks-1)
        x = start[0] + t*(end[0] - start[0])
        y = start[1] + t*(end[1] - start[1])
        if not is_collision_free(x, y, obstacles):
            return False
    return True

def simplify_path(path, obstacles):
    """Remove unnecessary waypoints while maintaining safety."""
    if len(path) < 3:
        return path
        
    simplified = [path[0]]
    current_idx = 0
    
    while current_idx < len(path) - 1:
        # Look ahead as far as possible
        for look_ahead in range(len(path)-1, current_idx, -1):
            if is_path_clear(path[current_idx], path[look_ahead], obstacles):
                simplified.append(path[look_ahead])
                current_idx = look_ahead
                break
        else:
            # If no skip possible, keep next waypoint
            current_idx += 1
            simplified.append(path[current_idx])
            
    return simplified

def ride_prius_rrt(loaded_env, start, path, target_speed, obstacles):
    """Updated version with camera following."""
    initial_state = np.array([start[0], start[1], path[0][2]])
    ob, *_ = loaded_env.reset(pos=initial_state)
    history = []
    current_target_idx = 1
    
    # Camera setup parameters
    cameraDistance = 5.0
    cameraPitch = -30.0
    cameraHeight = 0.5
    
    while current_target_idx < len(path):
        current_pos = ob['robot_0']['joint_state']['position']
        current_target = path[current_target_idx]
        
        # Look ahead for better target
        for idx in range(current_target_idx + 1, len(path)):
            if is_path_clear((current_pos[0], current_pos[1]), path[idx], obstacles):
                current_target = path[idx]
                current_target_idx = idx
            else:
                break
        
        # Drive towards current target
        dx = current_target[0] - current_pos[0]
        dy = current_target[1] - current_pos[1]
        distance = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        
        if distance < 0.5:
            current_target_idx += 1
            continue
            
        # Steering control
        heading_error = np.arctan2(np.sin(target_angle - current_pos[2]), 
                                 np.cos(target_angle - current_pos[2]))
        steering = np.clip(heading_error, -max_steering_angle, max_steering_angle)
        
        # Speed control
        speed = min(target_speed, 2.0 + 3.0 * (1.0 - abs(steering/max_steering_angle)))
        
        action = np.array([speed, steering])
        ob, *_ = loaded_env.step(action)
        history.append(ob)
        
        # Update camera position
        current_heading = current_pos[2]
        cameraYaw = math.degrees(current_heading) - 90
        p.resetDebugVisualizerCamera(
            cameraDistance=cameraDistance,
            cameraYaw=cameraYaw,
            cameraPitch=cameraPitch,
            cameraTargetPosition=[current_pos[0], current_pos[1], cameraHeight]
        )
        
        time.sleep(0.01)  # Smooth visualization
        
    return history

def plot_rrt_results(path, tree, obstacles, start, goal, grid_size):
    """Plot RRT results immediately after path finding."""
    plot_tree(
        found_path=path,
        tree=tree,
        obstacle_dict=obstacles,
        start=start,
        goal=goal,
        grid_width=grid_size[0],
        grid_height=grid_size[1]
    )
    plt.show(block=False)  # Show plot but don't block execution

def plot_final_trajectory(path, trajectory):
    """Plot final trajectory after simulation."""
    plt.figure(figsize=(8, 8))
    plt.plot([p[0] for p in path], [p[1] for p in path], 'g--', label="Planned Path")
    plt.plot([t[0] for t in trajectory], [t[1] for t in trajectory], 'b-', label="Car Path")
    plt.legend()
    plt.grid(True)
    plt.show()

def calculate_path_smoothness(path):
    """Calculate path smoothness using angle changes between segments."""
    if len(path) < 3:
        return 0.0
        
    angle_changes = []
    for i in range(len(path)-2):
        # Get three consecutive points
        p1 = np.array(path[i])
        p2 = np.array(path[i+1])
        p3 = np.array(path[i+2])
        
        # Calculate vectors between points
        v1 = p2 - p1
        v2 = p3 - p2
        
        # Calculate angle between vectors
        angle = np.abs(np.arctan2(np.cross(v1[:2], v2[:2]), np.dot(v1[:2], v2[:2])))
        angle_changes.append(angle)
    
    # Smoothness metric: average angle change (lower is smoother)
    return np.mean(angle_changes)

if __name__ == "__main__":
    # Test environment selection
    selected_env = "static"
    print(f"Testing {selected_env} environment...")
    env, has_obstacles, obstacle_dict, start_rrt, goal_rrt = load_environment(selected_env)

    start_rrt = (0.0, 0.0, 0.0)
    goal_rrt = (7.0, 10.0, 0.0)

    # Run RRT* to find a path
    grid_size = (30, 30)
    path_rrt, tree = rrt_star_with_tree(
        start=start_rrt, 
        goal=goal_rrt, 
        obstacles=obstacle_dict, 
        grid_size=grid_size,
        early_success_threshold=0.0, 
        max_iter=100000
    )

    if path_rrt is None:
        print("No path found by RRT!")
    else:
        print("RRT path found!", path_rrt)
        
        # Calculate and print path length
        path_length = sum(((path_rrt[i][0] - path_rrt[i-1][0])**2 + 
                          (path_rrt[i][1] - path_rrt[i-1][1])**2)**0.5 
                         for i in range(1, len(path_rrt)))
        print(f"RRT path length: {path_length:.2f}")
        
        # Calculate and print path smoothness
        smoothness = calculate_path_smoothness(path_rrt)
        print(f"Path smoothness (average angle change in radians): {smoothness:.3f}")
        print(f"Path smoothness (degrees): {np.degrees(smoothness):.1f}°")

        # Plot RRT results immediately
        plot_rrt_results(path_rrt, tree, obstacle_dict, start_rrt, goal_rrt, grid_size)

        # Simplify path before interpolation
        simplified_path = simplify_path(path_rrt, obstacle_dict)
        interpolated_path = interpolate_path(simplified_path)
        
        # Initial heading based on first path segment
        start_with_heading = (start_rrt[0], start_rrt[1], interpolated_path[0][2])
        
        # Ride along the RRT* path with proper initial heading
        history = ride_prius_rrt(env, start_with_heading, interpolated_path, target_speed, obstacle_dict)

        # Plot final trajectory
        trajectory = [obs['robot_0']['joint_state']['position'][:2] for obs in history]
        plot_final_trajectory(interpolated_path, trajectory)
