# Save this code as RRT_test.py and run it directly.
import numpy as np
import matplotlib.pyplot as plt
from car_data import car_model

# Set a random seed for reproducibility (optional)
np.random.seed(42)

# Car parameters
L = 2.5  # wheelbase length of the car in meters
max_steering_angle = np.radians(25)  # maximum steering angle in radians
target_speed = 5.0  # target speed in m/s (~18 km/h)


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
    """Modified RRT* that returns both path and tree."""
    start_node = Node(*start)
    goal_node = Node(*goal)
    tree = [start_node]

    found_path = None
    for _ in range(max_iter):
        # Increased goal bias to 20%
        if np.random.rand() < goal_bias:
            sample_node = goal_node
        else:
            sample_node = Node(np.random.uniform(0, grid_width),
                               np.random.uniform(0, grid_height),
                               np.random.uniform(-np.pi, np.pi))
        
        # Find nearest node and steer towards the sample
        nearest = nearest_node(tree, sample_node)
        new_node = steer(nearest, sample_node, target_speed)
        
        # Check for collisions
        if not is_collision_free(new_node.x, new_node.y):
            continue
        
        # Add new node to the tree
        tree.append(new_node)
        
        # Rewire the tree
        for node in tree:
            if distance(node, new_node) < radius and (new_node.cost + distance(new_node, node) < node.cost):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)
        
        # Check if goal is reached
        if distance(new_node, goal_node) < 0.5:  # slightly relaxed condition
            goal_node.parent = new_node
            goal_node.cost = new_node.cost
            # Extract path
            path_nodes = []
            current = goal_node
            while current.parent is not None:
                path_nodes.append((current.x, current.y, current.theta))
                current = current.parent
            path_nodes.reverse()
            found_path = path_nodes
            break

    return found_path, tree

if __name__ == "__main__":
    # Define start and goal states (x, y, theta)
    start = (2.0, 2.0, 0.0)
    goal = (18.0, 18.0, 0.0)
    
    path, tree = rrt_star_with_tree(start, goal, target_speed=target_speed, max_iter=5000, radius=2.0, goal_bias=0.2)

    if path is None:
        print("No path found after 5000 iterations!")
    else:
        print("Path found!")
        # Visualization
        plt.figure(figsize=(8, 8))
        # Plot the tree edges
        for node in tree:
            if node.parent is not None:
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], '-g', alpha=0.5)

        # Plot the start and goal
        plt.plot(start[0], start[1], 'ro', label="Start")
        plt.plot(goal[0], goal[1], 'bo', label="Goal")

        # Plot the path if found
        if path is not None:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            plt.plot(px, py, '-r', linewidth=2, label="Path")

        plt.title("RRT* Path Planning")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        plt.legend()
        plt.xlim(0, 20)
        plt.ylim(0, 20)
        plt.show()
