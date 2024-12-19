# Save this code as RRT_test.py and run it directly.
import numpy as np
import matplotlib.pyplot as plt
from car_data import car_model

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

def steer(from_node, to_node, target_speed, dt=0.25):
    """Steer from `from_node` towards `to_node` using car kinematics."""
    theta = from_node.theta
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    target_angle = np.arctan2(dy, dx)
    steering_angle = np.clip(target_angle - theta, -max_steering_angle, max_steering_angle)
    
    new_x, new_y, new_theta = car_model(from_node.x, from_node.y, from_node.theta, target_speed, steering_angle, dt)
    new_node = Node(new_x, new_y, new_theta, from_node)
    new_node.cost = from_node.cost + distance(from_node, new_node)
    return new_node

def is_collision_free(x, y):
    """Placeholder for collision checking. Always returns True for this demo."""
    return True

def nearest_node(tree, sample_node):
    """Find the nearest node in the tree to the sampled node."""
    return min(tree, key=lambda node: distance(node, sample_node))

def rrt_star_with_tree(start, goal, target_speed, grid_width=20, grid_height=20, max_iter=5000, radius=2.0, goal_bias=0.2):
    """
    Modified RRT* that returns both path and tree, and visualizes them.
    
    Parameters:
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
            sample_node = Node(
                np.random.uniform(0, grid_width),
                np.random.uniform(0, grid_height),
                np.random.uniform(-np.pi, np.pi)
            )
        
        # Find the nearest node in the tree to the sampled node
        nearest = nearest_node(tree, sample_node)
        
        # Steer towards the sampled node to create a new node
        new_node = steer(nearest, sample_node, target_speed)
        
        # Check for collisions (always True in this demo)
        if not is_collision_free(new_node.x, new_node.y):
            continue  # Skip to the next iteration if in collision
        
        # Add the new node to the tree
        tree.append(new_node)
        
        # Rewire the tree within the specified radius to optimize path cost
        for node in tree:
            if distance(node, new_node) < radius and (new_node.cost + distance(new_node, node) < node.cost):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)
        
        # Check if the new node is close enough to the goal
        if distance(new_node, goal_node) < 0.5:  # Threshold distance to consider goal reached
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
            break  # Exit the loop once the path is found

    # Visualization of the RRT* tree and the final path
    plt.figure(figsize=(10, 10))
    
    # Plot the tree edges
    for node in tree:
        if node.parent is not None:
            plt.plot([node.parent.x, node.x], [node.parent.y, node.y], '-g', alpha=0.3)
    
    # Plot the start and goal points
    plt.plot(start_node.x, start_node.y, 'ro', markersize=8, label="Start")
    plt.plot(goal_node.x, goal_node.y, 'bo', markersize=8, label="Goal")
    
    # Plot the final path if found
    if found_path is not None:
        px = [p[0] for p in found_path]
        py = [p[1] for p in found_path]
        plt.plot(px, py, '-r', linewidth=2, label="RRT* Path")
    
    # Enhance the plot
    plt.title("RRT* Path Planning")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid(True)
    plt.legend()
    plt.xlim(0, grid_width)
    plt.ylim(0, grid_height)
    plt.show()

    if found_path is None:
        print(f"No path found after {max_iter} iterations!")
    
    return found_path, tree