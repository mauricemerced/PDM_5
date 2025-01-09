# Save this code as RRT_test.py and run it directly.
import numpy as np
import matplotlib.pyplot as plt
from car_data import car_model, find_boundingbox
from environment_setup import load_environment
from urdfenvs.scene_examples.obstacles import *
from visualization_utils import visualize_bounding_box

# Set a random seed for reproducibility (optional)
np.random.seed(42)

# Car parameters
L = 2.5  # wheelbase length of the car in meters
max_steering_angle = np.radians(25)  # maximum steering angle in radians
target_speed = 30.0  # target speed in m/s (~108 km/h)

class Node:
    def __init__(self, x, y, theta, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        self.cost = 0

def distance(node1, node2):
    """Euclidean distance between two nodes."""
    return np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

def steer(from_node, to_node, target_speed, obstacle_dict, has_obstacles, dt=0.25, step_size=0.5):
    """Steer from `from_node` towards `to_node` using car kinematics."""
    theta = from_node.theta
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    distance_to_target = np.sqrt(dx**2 + dy**2)
    # theta = np.arctan2(dy, dx)
    target_angle = np.arctan2(dy, dx)
    steering_angle = np.clip(target_angle - theta, -max_steering_angle, max_steering_angle)
    
    steps = int(distance_to_target / step_size)
    x, y, theta = from_node.x, from_node.y, from_node.theta

    for _ in range(steps):
        x, y, theta = car_model(x, y, theta, target_speed, steering_angle, dt)
        if not is_collision_free(x, y, has_obstacles, obstacle_dict):
            print(f"Collision detected at ({x}, {y}).")
            return None  # Return None if a collision is detected
        
    # new_x, new_y, new_theta = car_model(from_node.x, from_node.y, from_node.theta, target_speed, steering_angle, dt)
    new_node = Node(x, y, theta, from_node)
    if new_node is None:
        print(f"Steer returned None for sample ({to_node.x}, {to_node.y})")
    new_node.cost = from_node.cost + distance(from_node, new_node)
    return new_node

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
        for obstacle in obstacles:
            try: 
                if "radius" in obstacle:  # Circular obstacle
                    distance = np.sqrt((x - obstacle["x"])**2 + (y - obstacle["y"])**2)
                    if distance < (obstacle["radius"] + tolerance):
                        print(f"Collision: Car hit obstacle at ({x}, {y})")
                        return False  # Collision detected
                elif "width" or "length" in obstacle: # Rectangular obstacle (walls) or boxes 
                    left = obstacle["x"] - obstacle["width"] / 2
                    right = obstacle["x"] + obstacle["width"] / 2
                    bottom = obstacle["y"] - obstacle["length"] / 2
                    top = obstacle["y"] + obstacle["length"] / 2
                    if left <= x <= right and bottom <= y <= top:
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

    # if not has_obstacles or len(obstacles) == 0:
    #     return True  # No obstacles to check
    
    # for obstacle in obstacles:
    #     try:
    #         if "radius" in obstacle:  # Circular obstacle
    #             distance = np.sqrt((x - obstacle["x"])**2 + (y - obstacle["y"])**2)
    #             if distance < (obstacle["radius"] + tolerance):
    #                 return False  # Collision detected
    #         else:  # Rectangular obstacle (walls)
    #             left = obstacle["x"] - obstacle["width"] / 2
    #             right = obstacle["x"] + obstacle["width"] / 2
    #             bottom = obstacle["y"] - obstacle["length"] / 2
    #             top = obstacle["y"] + obstacle["length"] / 2
    #             if left <= x <= right and bottom <= y <= top:
    #                 return False  # Collision detected
    #     except ValueError:
    #         print(f"Invalid obstacle data: {obstacle}")
    #         continue

        # # Ensure all values are numbers
        # if not all(isinstance(v, (int, float)) for v in [x, y, obs_x, obs_y, radius]):
        #     print(f"Invalid types: {type(obs_x)}, {type(obs_y)}, {type(radius)}")
        #     raise ValueError(f"Invalid data in obstacle: {obstacle}")
        
        # if np.sqrt((x - obs_x)**2 + (y - obs_y)**2) < (radius + tolerance):
        #     return False  # Collision detected
    # return True

def nearest_node(tree, sample_node):
    """Find the nearest node in the tree to the sampled node."""
    return min(tree, key=lambda node: distance(node, sample_node))

def plot_tree(found_path, tree, obstacle_dict, start, goal, grid_width, grid_height):
    # min_corner, max_corner = bounding_box
    plt.figure(figsize=(8, 8))
    for node in tree:
        if node.parent:
            plt.plot([node.parent.x, node.x], [node.parent.y, node.y], 'g-', alpha=0.5)
    for obs in obstacle_dict:
        if "radius" in obs: 
            circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='r', fill=True, alpha=0.5)
            plt.gca().add_artist(circle)
        elif "width" in obs and "length" in obs:  # Rectangular obstacle (walls)
            # Calculate the bottom-left corner for the rectangle
            bottom_left_x = obs["x"] - obs["width"] / 2
            bottom_left_y = obs["y"] - obs["length"] / 2
            rectangle = plt.Rectangle((bottom_left_x, bottom_left_y), obs["width"], obs["length"], 
                                       color='r', fill=True, alpha=0.5)
            plt.gca().add_artist(rectangle)
    if found_path:
        for idx, waypoint in enumerate(found_path):
            plt.plot(waypoint[0], waypoint[1], 'bo')  # Blue circles for waypoints
        plt.plot([p[0] for p in found_path], [p[1] for p in found_path], 'r-', linewidth=2, label="RRT* Path")
        
    plt.plot(start[0], start[1], 'bo', label="Start")
    plt.plot(goal[0], goal[1], 'ro', label="Goal")
    plt.scatter([node.x for node in tree], [node.y for node in tree], s=10, c='g', alpha=0.3, label='Tree Nodes')

    plt.xlim(0, grid_width)
    plt.ylim(0, grid_height)
    plt.legend()
    plt.grid()
    plt.show()

def rrt_star_with_tree(env, has_obstacles, obstacle_dict, start, goal, target_speed, grid_width, grid_height, max_iter, radius=2.0, goal_bias=0.7):
    """
    Modified RRT* that returns both path and tree, and visualizes them.
    
    Parameters:
    - env: Environment object loaded from `environment_setup.py`.
    - start: Tuple (x, y, theta) representing the start state.
    - goal: Tuple (x, y, theta) representing the goal state.
    - target_speed: Speed at which the car moves.
    - grid_width: Width of the sampling grid.
    - grid_height: Height of the sampling grid.
    - max_iter: Maximum number of iterations to run.
    - radius: Radius to consider for rewiring.
    - goal_bias: Probability of sampling the goal node.
    
    Returns:
    - found_path: List of waypoints representing the path from start to goal.
    - tree: List of all nodes in the RRT* tree.
    """

    start_node = Node(*start)
    goal_node = Node(*goal)
    tree = [start_node]

    found_path = None
    for iteration in range(max_iter):
        

        # Goal biasing: 20% chance to sample the goal
        if np.random.rand() < goal_bias:
            sample_node = goal_node
        else:
            while True:
                x = np.random.uniform(0, grid_width)
                y = np.random.uniform(0, grid_height)
                if is_collision_free(x, y, has_obstacles, obstacle_dict, tolerance=0.5):
                    break  # Keep sampling until a collision-free point is found
            sample_node = Node(x, y, np.random.uniform(-np.pi, np.pi))
        
        # Find the nearest node in the tree to the sampled node
        nearest = nearest_node(tree, sample_node)
        
        
        # Steer to the new node, with intermediate collision checks
        new_node = steer(nearest, sample_node, target_speed, obstacle_dict, has_obstacles)
        if new_node is None:
            continue  # Skip if in collision


        # # Check for collisions 
        # if not is_collision_free(new_node.x, new_node.y, has_obstacles, obstacle_dict):
        #     print(f"Node ({new_node.x}, {new_node.y}) is in collision.")
        #     continue  # Skip to the next iteration if in collision
        # else:
        #     print(f"Node ({new_node.x}, {new_node.y}) is collision-free.")
        
            

        
        # Add the new node to the tree
        tree.append(new_node)
        
        # Rewire the tree within the specified radius to optimize path cost
        for node in tree:
            if distance(node, new_node) < radius and (new_node.cost + distance(new_node, node) < node.cost):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)
        
        # Check if the new node is close enough to the goal
        if distance(new_node, goal_node) < 0.9:  # Threshold distance to consider goal reached, std = 0.5
            goal_node.parent = new_node
            goal_node.cost = new_node.cost
            # Extract the path by backtracking from the goal to the start
            path_nodes = []
            current = goal_node
            while current.parent is not None:
                path_nodes.append((current.x, current.y, current.theta))
                current = current.parent
            path_nodes.reverse()  # Reverse to get path from start to goal
            found_path = path_nodes
            print(f"Path found in {iteration+1} iterations.")
            print(f"Path found: {found_path}" )

            # Visualize targets and path adding markers in path
            for idx, target in enumerate(found_path):
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
            break  # Exit the loop once the path is found
    print(f"Iteration {iteration}: Tree size = {len(tree)}")
    print(f"Sampling near goal: Distance to goal = {distance(sample_node, goal_node)}")

    if found_path is None:
        print(f"No path found after {max_iter} iterations!")
    
    return found_path, tree