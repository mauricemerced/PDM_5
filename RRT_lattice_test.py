import numpy as np
from car_data import robots, target_speed
from RRT_test import rrt_star_with_tree  # Import the RRT function from RRT_test.py
from lattice_test import run_prius, lattice_planner2, plot_motion_trajectory

import time
import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from car_data import robots, target_speed, max_steering_angle
import math
                                                                                                                                                                                  
import pybullet as p

def ride_prius3(start, path):
    env = UrdfEnv(dt=0.005, robots=robots, render=True)
    ob, *_ = env.reset(pos=np.array(start))

    current_state = ob['robot_0']['joint_state']['position']
    history = []
    action = np.array([0.0, 0.0])

    # Initial camera setup parameters
    cameraDistance = 3.0  # Distance behind the car
    cameraYaw = 0         # Will be updated based on heading
    cameraPitch = -30      # Slight angle above the car
    cameraHeight = 0    # Height of the camera above the ground

    for next_target in path:
        target_reached = False
        while not target_reached:
            dx = next_target[0] - current_state[0]
            dy = next_target[1] - current_state[1]
            distance = np.sqrt(dx**2 + dy**2)

            # Accelerate gradually up to target_speed
            if action[0] < target_speed:
                action[0] += 0.1

            # Compute desired heading
            desired_theta = math.atan2(dy, dx)
            current_heading = current_state[2]
            heading_error = (desired_theta - current_heading + math.pi) % (2 * math.pi) - math.pi
            action[1] = np.clip(heading_error, -max_steering_angle, max_steering_angle)

            ob, *_ = env.step(action)
            history.append(ob)
            current_state = ob['robot_0']['joint_state']['position']

            # Update camera after moving
            # We assume current_heading is the direction the car is facing.
            # Place camera behind the car:
            # If the car faces current_heading, behind the car is opposite direction.
            cam_x = current_state[0] - cameraDistance * math.cos(current_heading)
            cam_y = current_state[1] - cameraDistance * math.sin(current_heading)
            cam_z = cameraHeight

            # Convert the current_heading to degrees for yaw
            cameraYaw = math.degrees(current_heading) - 90  # camera faces the car from behind
            p.resetDebugVisualizerCamera(cameraDistance=cameraDistance,
                                         cameraYaw=cameraYaw,
                                         cameraPitch=cameraPitch,
                                         cameraTargetPosition=[current_state[0], current_state[1], cameraHeight])

            # Slow down the simulation for better visualization
            time.sleep(0.05)

            if distance < 0.5:
                target_reached = True

        print(f"Reached target: {next_target}")

    print("Goal reached!")
    env.close()
    history_ends = 0
    return history, history_ends




if __name__ == "__main__":
    # First, generate lattice trajectories with run_prius
    lattice_trajectories = run_prius(robots, render=False)
    print("Lattice trajectories generated.")

    # Define your start and goal states for RRT
    start_rrt = (2.0, 2.0, 0.0)
    goal_rrt = (18.0, 18.0, 0.0)

    # Run RRT to find a path
    path_rrt, tree = rrt_star_with_tree(start_rrt, goal_rrt, target_speed=target_speed)

    if path_rrt is None:
        print("No path found by RRT!")
    else:
        print("RRT path found!")
        print("RRT Path:", path_rrt)

        # Use the endpoint of the RRT path as the goal for lattice planning
        # Assuming the last node in path_rrt is the goal the RRT reached
        x, y, _ = path_rrt[-1]
        start = (0, 0, 0)
        goal = (x, y)

        print("Planning path with lattice_planner2...")
        path_lattice, lattice = lattice_planner2(start, goal, lattice_trajectories)

        if path_lattice is None:
            print("No valid lattice path found.")
        else:
            print("Lattice path found!")
            print("Lattice Path:", path_lattice)
            print("Lattice (steering angles):", lattice)

            # Ride along the lattice path
            history, ends = ride_prius3(start, path_lattice)

            # Visualize the final motion
            plot_motion_trajectory(goal, history, path_lattice)
