import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(path, goal_position, history, ends):
    """
    Plots the trajectory of the car and the goal position.
    
    Parameters:
    - path: List of (x, y, theta) tuples representing the car's trajectory.
    - goal_position: (x, y, theta) tuple representing the goal position.
    """
    
    hx = np.array([ob['robot_0']['joint_state']['position'][0] for ob in history])
    hy = np.array([ob['robot_0']['joint_state']['position'][1] for ob in history])

    hxx = np.array([obs[-1]['robot_0']['joint_state']['position'][0] for obs in ends])
    hyy = np.array([obs[-1]['robot_0']['joint_state']['position'][1] for obs in ends])

    plt.figure(figsize=(10, 6))
    plt.plot(hx, hy, label="Trajectory", marker='o', linestyle='-', color='orange', markersize=3)
    plt.scatter(hxx, hyy, label="Trajectory", marker='o', color='purple', s=75)

    path = np.array(path)
    
    # Extract x and y coordinates
    x = path[:, 0]
    y = path[:, 1]
    
    # plt.figure(figsize=(10, 6))
    
    # Plot the trajectory
    plt.plot(hx[0], hy[0], label="Trajectory", marker='o', linestyle='-', color='blue', markersize=5)
    plt.scatter(x, y, label="Trajectory", marker='o', color='blue', s=50)
    
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




def plot_motion_trajectory(goal_position, history, path):
    print(path[0])
    hx = np.array([ob['robot_0']['joint_state']['position'][0] for ob in history])
    hy = np.array([ob['robot_0']['joint_state']['position'][1] for ob in history])

    plt.figure(figsize=(10, 6))
    plt.plot(hx, hy, label="Trajectory", marker='o', linestyle='-', color='orange', markersize=5)


    path = np.array(path)
    
    # Extract x and y coordinates
    x = path[:, 0]
    y = path[:, 1]
    plt.plot(x, y, label="Trajectory", marker='o', linestyle='-', color='red', markersize=5)

    
    # plt.figure(figsize=(10, 6))
    
    
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





def plot_lattice_trajectories(lattice_trajectories):
    """
    Plot the trajectories in the lattice.
    
    Args:
        lattice_trajectories: List of tuples where each tuple contains 
                              (steering_angle, trajectory).
                              Each trajectory is a list of observations.
    """
    plt.figure(figsize=(10, 10))
    # print(lattice_trajectories)
    
    for angle, trajectory in lattice_trajectories:
        # print(trajectory)
        # Extract x and y positions from the trajectory
        x_vals = [state['robot_0']['joint_state']['position'][0] for state in trajectory]  # Assuming x is at index 0
        y_vals = [state['robot_0']['joint_state']['position'][1] for state in trajectory]  # Assuming y is at index 1
        
        plt.plot(x_vals, y_vals, label=f"Steering Angle: {angle}°")
    
    plt.title("Lattice Trajectories")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()


