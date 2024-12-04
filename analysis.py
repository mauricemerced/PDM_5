import numpy as np
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


