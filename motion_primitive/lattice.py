import numpy as np
from collections import deque
from heapq import heappush, heappop
from car_data import L, max_steering_angle, car_model
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import L, max_steering_angle, robots, target_speed, n_points
import itertools
from environment_setup import load_environment

def heuristic(x, y, goal):
    """Euclidean distance as a heuristic."""
    return np.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)


def is_collision_free(x, y, has_obstacles, obstacles, tolerance=1.0):
    """
    Check for collisions with obstacles.
    
    Parameters:
    - x, y: Coordinates to check.
    - obstacles: List of obstacles, where each obstacle is [x_center, y_center, radius].
    - tolerance: Extra buffer around obstacles.
    
    
    Returns:
    - True if the position (x, y) is collision-free; False otherwise.
    """
    



    # min_corner, max_corner = bounding_box

    # # Check if the car is outside the bounding box
    # if not (min_corner[0] <= x <= max_corner[0] and min_corner[1] <= y <= max_corner[1]):
    #     print(f"Collision: Car out of bounding box at ({x}, {y})")
    #     return False

    # Check for collisions with obstacles
    if has_obstacles and obstacles:

        inflation_radius=0.5

        inflated_obstacles = []
        for obstacle in obstacles:
            if "radius" in obstacle:
                inflated_obstacles.append({
                    "x": obstacle["x"],
                    "y": obstacle["y"],
                    "radius": obstacle["radius"] + inflation_radius
                })
            elif "width" in obstacle and "length" in obstacle:
                inflated_obstacles.append({
                    "x": obstacle["x"],
                    "y": obstacle["y"],
                    "width": obstacle["width"] + 2 * inflation_radius,
                    "length": obstacle["length"] + 2 * inflation_radius
                })

        for obstacle in inflated_obstacles:
            try: 
                if "radius" in obstacle:  # Circular obstacle
                    distance = np.sqrt((x - obstacle["x"])**2 + (y - obstacle["y"])**2)
                    if distance < (obstacle["radius"] + tolerance):
                        print(f"Collision: Car hit circular obstacle at ({x}, {y})")
                        return False  # Collision detected
                elif "width" or "length" in obstacle: # Rectangular obstacle (walls) or boxes 
                    left = obstacle["x"] - obstacle["width"] / 2 - tolerance
                    right = obstacle["x"] + obstacle["width"] / 2 + tolerance
                    bottom = obstacle["y"] - obstacle["length"] / 2 - tolerance
                    top = obstacle["y"] + obstacle["length"] / 2 + tolerance
                    if left <= x <= right and bottom <= y <= top:
                        print(f"Collision: Car hit rectangular obstacle at ({x}, {y})")
                        return False  # Collision detected
            except ValueError:
                print(f"Invalid obstacle data: {obstacle}")
                continue

            # # Ensure all values are numbers
            # if not all(isinstance(v, (int, float)) for v in [x, y, obstacle["x"], obstacle["y"], obstacle["radius"]]):
            #     print(f"Invalid types: {type(obstacle['x'])}, {type(obstacle['y'])}, {type(obstacle['radius'])}")
            #     raise ValueError(f"Invalid data in obstacle: {obstacle}")
                

    # No collisions detected
    return True

# def is_collision_free(x, y, has_obstacles, obstacles, goal, step_size=0.1, tolerance=1.0):
    """
    Bug Algorithm integrated collision-free check.
    
    Parameters:
    - x, y: Current position of the robot.
    - has_obstacles: Boolean indicating if obstacles are present.
    - obstacles: List of obstacles with properties like position and radius/size.
    - goal: Tuple representing the goal position (x_goal, y_goal).
    - step_size: Incremental step size for contour following.
    - tolerance: Buffer distance to avoid collisions.

    Returns:
    - Boolean indicating whether the robot can reach the goal without collision.
    """
    if not has_obstacles or not obstacles:
        return True  # No obstacles to avoid

    def distance(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def find_closest_point_on_obstacle(cx, cy, obstacle):
        # Compute closest point on circular obstacle
        if "radius" in obstacle:
            dx = cx - obstacle["x"]
            dy = cy - obstacle["y"]
            dist = np.sqrt(dx**2 + dy**2)
            scaling = (dist - (obstacle["radius"] + tolerance)) / dist
            return obstacle["x"] + scaling * dx, obstacle["y"] + scaling * dy
        # For simplicity, only handling circular obstacles here
        return cx, cy

    def contour_following(curr_x, curr_y, obstacle):
        """
        Follow the obstacle's contour until a clear path is found.
        """
        angle = 0  # Start angle
        while True:
            # Increment the angle slightly and compute the next position
            angle += step_size
            contour_x = obstacle["x"] + (obstacle["radius"] + tolerance) * np.cos(angle)
            contour_y = obstacle["y"] + (obstacle["radius"] + tolerance) * np.sin(angle)
            
            # Check if this new position leads to a clear path to the goal
            if is_line_to_goal_clear((contour_x, contour_y), goal, obstacles, tolerance):
                return contour_x, contour_y
            # If we've looped completely around the obstacle, return failure
            if angle > 2 * np.pi:
                return None, None

    def is_line_to_goal_clear(start, goal, obstacles, tolerance):
        """Check if a direct line to the goal is clear."""
        for obstacle in obstacles:
            if "radius" in obstacle:
                # Line-circle intersection test
                cx, cy = obstacle["x"], obstacle["y"]
                r = obstacle["radius"] + tolerance
                dx, dy = goal[0] - start[0], goal[1] - start[1]
                f = start[0] - cx, start[1] - cy
                a = dx**2 + dy**2
                b = 2 * (f[0] * dx + f[1] * dy)
                c = f[0]**2 + f[1]**2 - r**2
                discriminant = b**2 - 4 * a * c
                if discriminant >= 0:  # Collision detected
                    return False
        return True

    # Check for direct collision at the starting point
    for obstacle in obstacles:
        if "radius" in obstacle:  # Circular obstacle
            if distance((x, y), (obstacle["x"], obstacle["y"])) < obstacle["radius"] + tolerance:
                # Collision detected, switch to contour following
                closest_point = find_closest_point_on_obstacle(x, y, obstacle)
                if closest_point:
                    return contour_following(x, y, obstacle) is not None
                else:
                    return False

    # If no collision, check if line to goal is clear
    return is_line_to_goal_clear((x, y), goal, obstacles, tolerance)

def potential_trajectories(loaded_env, start_rrt):
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

    # env = UrdfEnv(dt=0.005, robots=robots, render=render_)
  
    # pos0 = np.array([0, 0, 0])
    pos0 = np.array(start_rrt)
    steering_angles = np.linspace(-np.degrees(max_steering_angle), np.degrees(max_steering_angle), n_points).tolist()
    lattice_trajectories = []

    ob, *_  = loaded_env.reset(pos=pos0)

    # points = []

    for target_angle in steering_angles:
        # print(f"Simulating trajectory for angle: {np.degrees(target_angle):.2f}")
        print(target_angle, " ---------------------------------------------------------------")
        speed = ob['robot_0']['joint_state']['forward_velocity'][0]#1
        action = np.array([1., 0])
        trajectory = []
        # ob, *_  = loaded_env.reset(pos=pos0)  # Reset environment for each steering angle

        end_time: Optional[float] = None
        state: int = 1

        for step in itertools.count(1):
            ob, *_ = loaded_env.step(action)
            trajectory.append(ob)
            print(f"Step: {step}, Position: {ob['robot_0']['joint_state']['position']}")
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
    

    # loaded_env.close()
    print(f"Generated {len(lattice_trajectories)} trajectories")
    return lattice_trajectories




def lattice_planner(has_obstacles, obstacle_dict, start, goal, lattice_trajectories, max_steps=10000):
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
        priority, x, y, theta, cost_so_far, path, lattice = heappop(queue)
        # x, y, theta = current

        # Debug current state
        # print(f"Expanding state: {x}, {y}, {theta}")

        # Check if we've reached the goal (with tolerance)
        if np.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2) < 1.0:  # std 0.5
            print(f"Goal reached at: {x}, {y}")
            # print(f"Path found: {path}" )
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
            # print(f"Global trajectory point: ({global_x}, {global_y}), global angle: {global_theta}")

            # Calculate step cost (Euclidean distance)
            trajectory_cost = np.sqrt((global_x - x) ** 2 + (global_y - y) ** 2)

            # Check for collision-free path (based on the final state)
            if not is_collision_free(global_x, global_y, has_obstacles, obstacle_dict):
                continue  # Skip if collision is detected

            # Heuristic cost (straight-line distance to goal)
            heuristic_cost = np.sqrt((goal[0] - global_x) ** 2 + (goal[1] - global_y) ** 2)

            # Update the priority queue if the state hasn't been visited
            if (global_x, global_y, global_theta) not in visited:
                # print(f"Adding state to queue: {global_x}, {global_y}, {global_theta}")
                visited.add((global_x, global_y, global_theta))
                new_path = path + [(global_x, global_y, global_theta)]
                new_lattice = lattice + [target_angle]
                priority = cost_so_far + trajectory_cost + heuristic_cost
                heappush(queue, (priority, global_x, global_y, global_theta, cost_so_far + trajectory_cost, new_path, new_lattice))

            # print(f"Expanding to: {global_x}, {global_y}, Current cost: {cost_so_far}")


        # env, has_obstacles, obstacle_dict, start_rrt, goal_rrt = load_environment(selected_env)
        # # Visualize targets and path adding markers in path
        #     for idx, target in enumerate(found_path):
        #         marker_dict = {
        #             "type": "sphere",
        #             "geometry": {
        #                 "position": [float(target[0]), float(target[1]), float(1.0)],  # Cast to float
        #                 "radius": float(0.05)  # Cast to float
        #             },
        #             "rgba": [float(0.3), float(0.5), float(0.6), float(1.0)],  # Cast to float
        #         }
        #         sphere_marker = SphereObstacle(name=f"sphere_marker_{idx}", content_dict=marker_dict)
        #         env.add_obstacle(sphere_marker)
                
        # Stop if max steps are exceeded
        if len(path) > max_steps:
            break

    print("No valid path found")
    return None, None  # No valid path found