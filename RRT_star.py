# Save this code as Improved_RRT_star.py and run it directly.
import numpy as np
import matplotlib.pyplot as plt
from car_data import car_model
from environment_setup import load_environment

# Set a random seed for reproducibility
np.random.seed(42)

# Car parameters
L = 2.5  # wheelbase length in meters
max_steering_angle = np.radians(25)  # maximum steering angle in radians
target_speed = 30.0  # target speed in m/s

class Node:
    def __init__(self, x, y, theta, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent
        self.cost = 0.0 if parent is None else parent.cost + distance(parent, self)
        self.children = []

def distance(node1, node2):
    """Calculate Euclidean distance between two nodes."""
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

def is_collision_free(x, y, obstacles, tolerance=0.75):
    """Improved collision checking for both circular and wall obstacles."""
    for obs in obstacles:
        if "radius" in obs:
            # Circular obstacle
            safe_radius = obs["radius"] + tolerance
            dist = np.sqrt((x - obs["x"])**2 + (y - obs["y"])**2)
            if dist < safe_radius:
                return False
        elif "width" in obs and "length" in obs:
            # Wall obstacle - use rectangle check
            half_width = obs["width"]/2 + tolerance
            half_length = obs["length"]/2 + tolerance
            
            # Transform point to obstacle's coordinate system
            dx = abs(x - obs["x"])
            dy = abs(y - obs["y"])
            
            if dx < half_width and dy < half_length:
                return False
    return True

def nearest_node(tree, sample_node):
    """Find the nearest node in the tree to the sampled node."""
    return min(tree, key=lambda node: distance(node, sample_node))

def rewire(tree, new_node, radius, obstacles):
    """Rewire the tree to improve the cost of paths."""
    for node in tree:
        if distance(node, new_node) < radius and new_node.cost + distance(new_node, node) < node.cost:
            if is_collision_free(node.x, node.y, obstacles):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node, node)
                update_children_cost(node)

def update_children_cost(node):
    """Update the cost of all children recursively."""
    for child in node.children:
        child.cost = node.cost + distance(node, child)
        update_children_cost(child)

def extract_path(goal_node):
    """Extract the path from start to goal including theta."""
    path = []
    current = goal_node
    while current:
        path.append((current.x, current.y, current.theta))
        current = current.parent
    return path[::-1]

def informed_sample(start_node, goal_node, best_cost, grid_size):
    """Sample from an elliptical region for better paths."""
    c_min = distance(start_node, goal_node)
    if best_cost < float('inf'):
        r1 = best_cost / 2
        r2 = np.sqrt(best_cost**2 - c_min**2) / 2
        angle = np.arctan2(goal_node.y - start_node.y, goal_node.x - start_node.x)
        
        while True:
            x = np.random.uniform(-1, 1)
            y = np.random.uniform(-1, 1)
            if x**2 + y**2 <= 1:
                break
        
        x_sample = (start_node.x + goal_node.x)/2 + r1*np.cos(angle)*x - r2*np.sin(angle)*y
        y_sample = (start_node.y + goal_node.y)/2 + r1*np.sin(angle)*x + r2*np.cos(angle)*y
        
        return Node(x_sample, y_sample, 0)
    return Node(
        np.random.uniform(0, grid_size[0]),
        np.random.uniform(0, grid_size[1]),
        0
    )

def calculate_path_cost(path):
    """Calculate total path cost with proper Node initialization."""
    return sum(distance(
        Node(path[i][0], path[i][1], path[i][2]), 
        Node(path[i+1][0], path[i+1][1], path[i+1][2])
    ) for i in range(len(path)-1))

def rrt_star_with_tree(start, goal, obstacles, grid_size, early_success_threshold,  max_iter=5000, step_size=1.0, radius=2.0):
    """Optimized RRT* algorithm with better performance."""
    start_node = Node(*start)
    goal_node = Node(*goal)
    tree = [start_node]
    
    best_path = None
    best_cost = float('inf')
    min_improvement = 0.1  # Minimum cost improvement threshold
    
    # Early success parameters
    early_success_threshold = 1.5  # Factor above optimal straight-line distance
    straight_line_dist = distance(start_node, goal_node)
    early_termination_cost = straight_line_dist * early_success_threshold
    
    for i in range(max_iter):
        # Use informed sampling once we have a solution
        if best_path:
            if best_cost <= early_termination_cost:
                print(f"Found good enough path at iteration {i}")
                break
            sample = informed_sample(start_node, goal_node, best_cost, grid_size)
        else:
            # Bias sampling towards goal initially
            if np.random.random() < 0.01:  # 10% chance to sample goal
                sample = Node(goal[0], goal[1], goal[2])
            else:
                sample = Node(
                    np.random.uniform(0, grid_size[0]),
                    np.random.uniform(0, grid_size[1]),
                    0
                )
        
        nearest = nearest_node(tree, sample)
        new_node = steer(nearest, sample, step_size)
        
        if is_collision_free(new_node.x, new_node.y, obstacles):
            # Find nearby nodes for potential connections
            nearby = [n for n in tree if distance(n, new_node) <= radius]
            min_cost = float('inf')
            best_parent = None
            
            for near_node in nearby:
                if is_collision_free(near_node.x, near_node.y, obstacles):
                    potential_cost = near_node.cost + distance(near_node, new_node)
                    if potential_cost < min_cost:
                        min_cost = potential_cost
                        best_parent = near_node
            
            if best_parent:
                new_node.parent = best_parent
                new_node.cost = min_cost
                tree.append(new_node)
                
                # Only rewire if we're close to the goal region
                if distance(new_node, goal_node) < radius * 2:
                    rewire(tree, new_node, radius, obstacles)
                
                # Check if we can reach goal
                if distance(new_node, goal_node) < step_size:
                    potential_goal_cost = new_node.cost + distance(new_node, goal_node)
                    
                    if potential_goal_cost < best_cost - min_improvement:
                        goal_node.parent = new_node
                        goal_node.cost = potential_goal_cost
                        best_path = extract_path(goal_node)
                        best_cost = potential_goal_cost
                        print(f"Found better path with cost: {best_cost:.2f}")
    
    return best_path, tree

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