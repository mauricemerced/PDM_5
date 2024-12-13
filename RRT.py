import numpy as np
from car_data import L, max_steering_angle

class Node:
    def __init__(self, x, y, theta, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        self.cost = 0  # Cost to reach this node

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
    """Placeholder for collision checking."""
    return True  # Replace with actual environment collision check

def nearest_node(tree, sample_node):
    """Find the nearest node in the tree to the sampled node."""
    return min(tree, key=lambda node: distance(node, sample_node))

def rrt_star(start, goal, target_speed, grid_width=20, grid_height=20, max_iter=1000, radius=2.0):
    """RRT* motion planner."""
    start_node = Node(*start)
    goal_node = Node(*goal)
    tree = [start_node]

    for _ in range(max_iter):
        # Sample a random point (with goal bias)
        if np.random.rand() < 0.1:  # 10% chance to sample the goal
            sample_node = goal_node
        else:
            sample_node = Node(np.random.uniform(0, grid_width), np.random.uniform(0, grid_height), np.random.uniform(-np.pi, np.pi))
        
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
            if distance(node, new_node) < radius and new_node.cost + distance(new_node, node) < node.cost:
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)
        
        # Check if goal is reached
        if distance(new_node, goal_node) < 0.1:
            goal_node.parent = new_node
            goal_node.cost = new_node.cost
            break

    # Extract the path
    path = []
    current = goal_node
    while current.parent is not None:
        path.append((current.x, current.y, current.theta))
        current = current.parent
    path.reverse()

    return path if path else None
